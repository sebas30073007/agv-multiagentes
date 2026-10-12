// main.js - arma el estado, conecta UI <-> canvas <-> control, y corre el
// lazo de simulacion.
//
// ATLAS sigue exactamente igual que antes: manda su pose a /ciclo cada
// PERIODO_MS, el servidor calcula con control.py/cinematica.py, el
// navegador solo integra la fisica con directa() y dibuja. Apolo y Odiseo no
// tienen backend: cada tick se les calcula (ux,uy) segun su modelo
// (punto/trayectoria/formacion) y se integra localmente con control.js.
//
// La pieza que une a los dos: para los 3 modelos, lo que varia es
// UNICAMENTE como se calcula el objetivo (ux,uy) o el punto a perseguir;
// la cola (uniciclo->diferencial->integrar, o mandar destinoManual al
// backend de Atlas) es la misma. Ver calcularObjetivo() abajo.
"use strict";

let CFG = {
  ancho: 1280, alto: 720, camara_viva: false,
  v_max_px: 250, w_max_rad: 1.6, tolerancia_px: 30, ancho_robot_px: 250,
  ancho_via: 840, entre_ejes: 510,
  config_rodillos: "O", autoridad: 0.244, zona_muerta: 30, max_raw: 255,
  // Espejos medidos sobre el robot, iguales que en cinematica.py. Los pisa
  // /config al cargar; estan aqui para el primer cuadro, antes de que llegue.
  invertir_lateral: true, invertir_giro: false,
  escala_minima: 0.39,
  orden: [{ rueda: "RR", signo: 1 }, { rueda: "FR", signo: 1 }, { rueda: "FL", signo: 1 }, { rueda: "RL", signo: 1 }],
};

const lim = (v, a, b) => v < a ? a : v > b ? b : v;

function crearRobot(def) {
  return {
    ...def,
    pose: { ...def.spawn },
    destino: { x: -def.spawn.x, y: -def.spawn.y, th: 0 },
    modelo: "punto",
    // Campo repulsivo propio, clavado al centro del robot y viajando con
    // el. Empuja a los OTROS robots; a los obstaculos no, porque su
    // movimiento esta programado y no es fisica que se pueda desviar.
    //
    // eta 0 = apagado, y ese es el arranque: mientras nadie lo suba, la
    // pagina se comporta exactamente igual que antes de que esto existiera.
    // Los tres ultimos campos no son ajustes, son lo que necesita el dibujo.
    repulsor: {
      radio: def.radioRepulsor || 170, eta: 0, tipo: "lineal_norm", sentido: 1,
      cerca: 0, anim: 0, hacia: null,
    },
    // Lo pone calcularObjetivo(): si hay algo empujando a este robot AHORA.
    // Lo lee la zona de llegada, que no debe cerrarse mientras lo empujen.
    repelido: false,
    // ArUco que identifica a este robot en el modo Real. Editable desde el
    // panel de Control: cual marcador se le pego a cual robot es una
    // decision fisica, no una constante del codigo.
    tagAruco: def.tagAruco !== undefined ? def.tagAruco : 0,
    // Si este robot obedece de verdad. Arranca en no. Sigue guardado por
    // robot porque es lo que leen el envio y el paro, pero la casilla del
    // panel global lo escribe en los tres a la vez.
    armado: false,
    parametros: {
      // tolPos (px) y tolAng (grados): la zona en la que el lazo da el
      // destino por alcanzado y se calla. Es por robot, como el resto, y
      // arranca en lo que trae el servidor. Bajandolas se llega tan fino
      // como en Formacion, que no tiene zona de llegada ninguna.
      punto: { k: 0.8, escala: ESCALA_DEFECTO, tolPos: 30, tolAng: 5 },
      trayectoria: { tipo: "circulo", radio: 180, omega: 0.5, fase: 0, centroInicial: null },
      // Manual: PWM crudo por rueda durante un tiempo fijo. No hay ley de
      // control de por medio; es lazo abierto, para probar hardware.
      manual: {
        pwm: def.tipo === "holonomico" ? [0, 0, 0, 0] : [0, 0],
        segundos: 2, restante: 0, activo: false, real: false,
        // Como se escribe esa trama: "ruedas" a mano, caja por caja, o
        // "flechas" conduciendo con la cruceta / el teclado. Es por robot,
        // igual que el resto de parametros: Atlas se prueba rueda por
        // rueda y a Odiseo se le da de manejar, y no tienen por que
        // compartir la eleccion.
        //
        // Arranca en "flechas": entrar a manual es casi siempre para
        // manejar, y escribir PWM caja por caja es lo excepcional.
        modo: "flechas",
        // PWM con el que sale cada flecha. Por encima de la zona muerta
        // (30 crudo) o el motor solo zumba.
        potencia: 120,
        // Corte por reloj de pared, independiente del lazo de dibujo. Solo
        // lo usa el modo de cajas: conduciendo no hay corte, ver
        // arrancarManual(). finMs: cuando vence; corte: el setTimeout que
        // lo hace cumplir; latido: el que repite la trama mientras se pisa.
        finMs: 0, corte: null, latido: null,
      },
    },
    rastro: [],
    pwm: def.tipo === "holonomico" ? [0, 0, 0, 0] : [0, 0],
    velocidad: def.tipo === "holonomico" ? { vx: 0, vy: 0, w: 0 } : { v: 0, w: 0 },
    conectado: true,
    latenciaDummy: 8 + Math.round(Math.random() * 22),
    imgSrc: def.img,        // ruta, para el <img> de la tarjeta inferior
    img: cargarImagen(def.img), // elemento Image() ya cargando, para el canvas
    // solo Atlas usa esto (backend real via /ciclo)
    red: { trama: [0, 0, 0, 0], destinoManual: null, enVuelo: false, despegue: 0, ultimoContacto: 0, conectadoReal: false,
           info: { dist: 0, err_rumbo: 0, adelante: 0, izquierda: 0, giro: 0, llego: false, quieto: false },
           camara: { robot: null, destino: null } },
  };
}

function cargarImagen(src) {
  const img = new Image();
  img.src = src;
  return img;
}

// Los PNG vienen a ~1450 px de lado y se dibujan a ~300. Reescalar de
// golpe con el filtro del navegador hace que se salten pixeles enteros: en
// un render con tantos detalles finos (tornillos, cables, rodillos) eso
// aparece como moire y lineas fantasma. Reducir por MITADES sucesivas
// promedia cada paso y lo quita.
function reducirImagen(img, maxLado) {
  let w = img.naturalWidth, h = img.naturalHeight;
  if (!w || !h || Math.max(w, h) <= maxLado) return img;

  let fuente = img;
  const paso = (anchoDestino, altoDestino) => {
    const c = document.createElement("canvas");
    c.width = Math.max(1, Math.round(anchoDestino));
    c.height = Math.max(1, Math.round(altoDestino));
    const cx = c.getContext("2d");
    cx.imageSmoothingEnabled = true;
    cx.imageSmoothingQuality = "high";
    cx.drawImage(fuente, 0, 0, c.width, c.height);
    fuente = c; w = c.width; h = c.height;
  };

  while (Math.max(w, h) > maxLado * 2) paso(w / 2, h / 2);
  const k = maxLado / Math.max(w, h);
  paso(w * k, h * k);
  return fuente;
}

// Recolorea una silueta conservando su transparencia. "source-in" deja
// solo lo que ya era opaco y lo repinta del color pedido, asi que los
// bordes suavizados del PNG se mantienen en vez de quedar dentados.
function tintar(img, color) {
  const c = document.createElement("canvas");
  c.width = img.naturalWidth || img.width;
  c.height = img.naturalHeight || img.height;
  const cx = c.getContext("2d");
  cx.drawImage(img, 0, 0);
  cx.globalCompositeOperation = "source-in";
  cx.fillStyle = color;
  cx.fillRect(0, 0, c.width, c.height);
  return c;
}

function esperarImagen(img) {
  return new Promise(resolve => {
    if (img.complete && img.naturalWidth) { resolve(); return; }
    img.onload = resolve;
    img.onerror = resolve; // si falla, se sigue sin marcador - no bloquear
  });
}

// Espera a que todas las imagenes esten listas antes de arrancar el loop -
// si no, el primer frame (o los primeros, con la camara lenta) dibuja el
// placeholder de canvas.js (cuadrado redondeado) y se ve un parpadeo feo al
// cargar la pagina. De paso deja preparados el sprite ya reducido y los dos
// marcadores ya tintados, para no rehacerlos en cada frame.
function precargarImagenes(robots) {
  const real = cargarImagen(MARCA_REAL);
  const marca = cargarImagen(MARCA_DESEADA);

  const deRobots = robots.map(r => esperarImagen(r.img).then(() => {
    if (!r.img.naturalWidth) return;
    // Cada robot se reduce a lo que realmente ocupa en pantalla: los
    // uniciclos se dibujan a una fraccion del tamaño de Atlas.
    r.sprite = reducirImagen(r.img, Math.max(280, Math.round(LADO_SPRITE * (r.escalaImg || 1))));
  }));

  const deMarcadores = Promise.all([esperarImagen(real), esperarImagen(marca)]).then(() => {
    // El marcador real es negro para todos, asi que se tinta una sola vez
    // y se comparte; el deseado lleva el color de cada robot.
    const realNegro = real.naturalWidth ? tintar(real, COLOR_MARCA_REAL) : null;
    robots.forEach(r => {
      r.marcaReal = realNegro;
      // El destino se dibuja con real.png -la flecha SOLA- y no con
      // marca.png. El circulo de marca.png ya no sirve: el anillo que se
      // ve alrededor del destino es ahora la zona de llegada, la dibuja
      // canvas.js y su radio sale del panel. Con el PNG de dos piezas no
      // habia forma de que uno creciera sin el otro.
      r.flechaDeseada = real.naturalWidth ? tintar(real, r.color) : null;
      // Sobre el video, casi blanca: el piso del laboratorio es oscuro. La
      // identidad de color no se pierde, la lleva el anillo.
      r.flechaDeseadaVideo = real.naturalWidth ? tintar(real, COLOR_DESTINO_VIDEO) : null;
    });
  });

  return Promise.all(deRobots.concat([deMarcadores]));
}

// La vista elegida sobrevive a recargar. Envuelto en try: en una ventana
// privada o con el almacenamiento bloqueado, leer tambien lanza.
function leerVistaGuardada() {
  try { return localStorage.getItem("vista") === "3d" ? "3d" : "2d"; } catch (e) { return "2d"; }
}

const estado = {
  robots: ROBOTS.map(crearRobot),
  seleccionado: "atlas",
  corriendo: true,
  tiempo: 0,
  habilitados: { atlas: true, apolo: true, odiseo: true },
  // "sim" dibuja el mapa; "real" muestra el video y la pose sale de
  // la camara. Es una sola decision, no dos.
  modo: "sim",
  // "2d" o "3d". Solo cuenta en Simulacion; en Real se ve el video. Es
  // aparte de modo porque al volver de Real hay que recuperarla.
  vista: leerVistaGuardada(),
  // La figura ya no esta escrita a fuego: son tres puntos sobre una
  // rejilla, en UNIDADES DE REJILLA, que se arrastran en el panel. El
  // triangulo de arranque es el de siempre, lo mas parecido que cabe en
  // enteros. "angulo" es hacia donde mira la formacion.
  //
  // eta/r/tipo se fueron: la repulsion entre robots vive ahora en el campo
  // propio de cada uno (pestaña Robots), que ademas vale en los cuatro
  // modos y no solo aqui.
  formacion: {
    a: 370, k: 1.2, angulo: 0,
    puntos: [[0, 2], [-2, -1], [2, -1]],
  },
  // Obstaculos: GLOBALES, como la formacion. Un campo repulsivo no es de
  // ningun robot -lo sufren los tres por igual-, asi que no cuelgan de
  // ninguno. Se arranca sin ninguno; caben MAX_OBSTACULOS.
  obstaculos: [],
  seleccionadoObs: null,
};

function robotPorId(id) { return estado.robots.find(r => r.id === id); }

const viewMapa = document.getElementById("viewMapa");
const viewVideo = document.getElementById("viewVideo");
/* ============================================================
   MODO simulacion/real
   ============================================================
   Un solo interruptor decide vista Y origen de la pose:

     sim  -> lienzo del mapa. Todo simulado en el navegador.
     real -> video de la camara con la reticula y las guias encima.
             La pose de Atlas la dicta el ArUco; el destino, la
             trayectoria y la formacion siguen siendo imaginarios, que es
             lo unico que tiene sentido: en el mundo real no hay un punto
             pintado en el piso, solo un robot al que se le manda ir ahi.

   En real participan los robots con hardware: Atlas y Odiseo. Apolo se
   queda apagado en el selector hasta que su agente conteste. */
const viewVideoEl = document.getElementById("viewVideo");
const lienzoVideo = document.getElementById("lienzoVideo");

// "filtro" es Real mirando otra fuente de video: mismo hardware, mismo
// control, misma escena. Todo lo que preguntaba "estoy en real" tiene que
// seguir diciendo que si, o el panel se queda mandando ordenes a la nada.
const MODOS_REALES = ["real", "filtro"];
function esModoReal(m) { return MODOS_REALES.indexOf(m) !== -1; }
function esReal() { return esModoReal(estado.modo); }
function fuenteVideo() {
  return estado.modo === "filtro" ? "/video_realce.mjpg" : "/video.mjpg";
}
function refrescarFuenteVideo() {
  videoImg.src = fuenteVideo() + "?t=" + Date.now();
}
function lienzoActivo() { return esReal() ? lienzoVideo : lienzo; }
// Donde se pica y donde va el cursor: en 3D no es el lienzo 2D, que queda
// escondido, sino el canvas de WebGL.
function superficieActiva() { return en3D() ? escena3d.lienzo : lienzoActivo(); }
function robotsEnEscena() {
  // En Real solo se dibuja lo que existe de verdad. Es hardware y no
  // backend: Odiseo no tiene fisica en el servidor pero si esta en la
  // cancha, y dibujarlo es justo lo que se quiere.
  return esReal() ? estado.robots.filter(r => r.hardware) : estado.robots;
}

// El paro esta SIEMPRE a la vista: un boton que aparece y desaparece no se
// encuentra con prisa. En Simulacion pausa la cancha y manda ceros a los
// robots fisicos, que es inofensivo. Esc y la barra siguen siendo solo de
// Real o con MQTT armado (ver mas abajo), para no pelearse con el teclado.
function hayArmado() { return estado.robots.some(r => r.armado); }
function refrescarParo() { bparo.hidden = false; }

function aplicarModo(modo) {
  const veniaDeReal = esModoReal(estado.modo);
  const cambiaFuente = estado.modo !== modo;
  estado.modo = modo;
  document.querySelectorAll("#segModo button").forEach(x =>
    x.classList.toggle("on", x.dataset.modo === modo));
  viewMapa.classList.toggle("on", modo === "sim");
  viewVideo.classList.toggle("on", esModoReal(modo));
  // Cambiar de Real a Filtros no reabre la camara, solo cambia de que URL
  // cuelga el <img>: el MJPEG viejo hay que soltarlo o el navegador se queda
  // con la conexion anterior abierta y no pide la nueva.
  if (cambiaFuente && esModoReal(modo)) refrescarFuenteVideo();

  // En real solo se puede tener seleccionado un robot con hardware
  // detras: dejar seleccionado uno que ahi no existe deja el panel de
  // Control mandando ordenes a la nada. Ya no se fuerza a Atlas a secas,
  // porque Odiseo tambien esta en la cancha.
  const sel = estado.robots.find(r => r.id === estado.seleccionado);
  if (esModoReal(modo) && !(sel && sel.hardware)) seleccionarRobot("atlas");

  refrescarParo();
  // Cambiar de vista YA NO desarma: el armado es un interruptor propio, y
  // apagarlo por mirar el mapa seria pisar una decision que se tomo aparte.
  // Lo que si se hace, mas abajo, es mandar el cero: la vista cambia con el
  // robot andando y quien mira ya no lo tiene delante.
  UI.refrescarArmado(estado);
  // Al entrar en Real se buscan cámaras solas UNA vez, y solo si no hay
  // ninguna abierta: ahí no hay nada que interrumpir y es exactamente lo
  // que ibas a picar. Con una cámara ya funcionando no se toca, que buscar
  // obliga a cerrarla.
  if (esModoReal(modo)) {
    cargarCamaras().then(() => {
      if (cam.lista === null && cam.indice === null && !cam.escaneando) pedirCamara({ escanear: true });
    });
  }
  // Salir de Real con el robot andando lo dejaria suelto sin nadie
  // mirandolo: la vista cambia, el robot no. Solo al SALIR de real, no en
  // el arranque de la pagina: publicar por MQTT nada mas cargar abriria la
  // conexion al broker aunque nadie vaya a usar el modo real.
  if (veniaDeReal && !esModoReal(modo)) pararRobotReal();
  UI.construirSelectorRobot(estado, seleccionarRobot);
  Escena.usarLienzo(lienzoActivo(), { sobreVideo: esModoReal(modo) });
  Escena.ajustarLienzo();
  refrescarBloqueTag();
  refrescarVista();
}

document.querySelectorAll("#segModo button").forEach(b => {
  b.onclick = () => { if (!b.disabled) aplicarModo(b.dataset.modo); };
});

/* ---- que pagina ocupa la columna derecha ----
   Menu inferior: Robots, Obstaculos y Telemetria, cada una con mandos que no
   se influyen con los de las otras. Al esconderse Telemetria, sus lienzos
   quedan en 0x0 y dibujarUna() se sale sola: no hay que parar nada. */
const ICONO_PANEL = { control: "robots", obstaculos: "obstaculos", telemetria: "telemetria" };
let panelALaVista = "control";
function mostrarPanel(cual) {
  const cambia = cual !== panelALaVista;
  panelALaVista = cual;
  document.querySelectorAll("#segPanel button").forEach(b => {
    const activa = b.dataset.panel === cual;
    b.classList.toggle("on", activa);
    b.setAttribute("aria-selected", activa ? "true" : "false");
    // El icono de la pagina elegida va relleno.
    b.querySelector("use").setAttribute("href", "#i-" + ICONO_PANEL[b.dataset.panel] + (activa ? "-on" : ""));
  });
  [["control", "panelControl"], ["obstaculos", "panelObstaculos"], ["telemetria", "panelTelemetria"]]
    .forEach(([k, id]) => {
      const p = document.getElementById(id);
      if (k === cual && p.hidden && cambia) {
        p.classList.remove("pane-entra");
        void p.offsetWidth;
        p.classList.add("pane-entra");
      }
      p.hidden = k !== cual;
      if (k === cual) p.scrollTop = 0;
    });
}
document.querySelectorAll("#segPanel button").forEach(b => {
  b.onclick = () => mostrarPanel(b.dataset.panel);
});

/* ---- ajustes rapidos: hoja que sube desde abajo dentro del panel ----
   Lo que vale para los tres robots y se toca poco. El foco entra a la hoja,
   no se escapa con Tab y regresa al boton al cerrar. */
const hojaAjustes = document.getElementById("hojaAjustes");
let hojaPrevio = null, timerHoja = null;
function abrirAjustes() {
  clearTimeout(timerHoja);
  hojaPrevio = document.activeElement;
  hojaAjustes.hidden = false;
  void hojaAjustes.offsetWidth;
  hojaAjustes.classList.add("abierta");
  hojaAjustes.querySelector(".btn-cerrar").focus({ preventScroll: true });
}
function cerrarAjustes() {
  if (hojaAjustes.hidden || !hojaAjustes.classList.contains("abierta")) return;
  hojaAjustes.classList.remove("abierta");
  const reducido = matchMedia("(prefers-reduced-motion: reduce)").matches;
  timerHoja = setTimeout(() => { hojaAjustes.hidden = true; }, reducido ? 0 : 320);
  if (hojaPrevio && document.contains(hojaPrevio)) hojaPrevio.focus({ preventScroll: true });
}
document.getElementById("bAjustes").onclick = abrirAjustes;
hojaAjustes.querySelectorAll("[data-cerrar]").forEach(b => { b.onclick = cerrarAjustes; });
hojaAjustes.addEventListener("keydown", ev => {
  if (ev.key !== "Tab") return;
  const f = [...hojaAjustes.querySelectorAll("button, input")].filter(x => !x.disabled && x.offsetParent);
  if (!f.length) return;
  if (ev.shiftKey && document.activeElement === f[0]) { ev.preventDefault(); f[f.length - 1].focus(); }
  else if (!ev.shiftKey && document.activeElement === f[f.length - 1]) { ev.preventDefault(); f[0].focus(); }
});
// Esc cierra la hoja. Si ademas hay algo que parar, el listener del paro
// (mas abajo) hace su trabajo con la misma tecla.
window.addEventListener("keydown", ev => { if (ev.key === "Escape") cerrarAjustes(); });

/* ============================================================
   LIENZO
   ============================================================ */
const lienzo = document.getElementById("lienzo");
Escena.init(lienzo, CFG);
// Solo se redimensiona el lienzo VISIBLE: ajustarLienzo escribe
// canvas.width, y eso borra el contenido. Al cambiar de modo se ajusta el
// que entra, desde aplicarModo().
function ajustarLienzoActivo() {
  Escena.usarLienzo(lienzoActivo(), { sobreVideo: esReal() });
  Escena.ajustarLienzo();
  if (en3D()) escena3d.ajustar();
}
window.addEventListener("resize", ajustarLienzoActivo);
new ResizeObserver(ajustarLienzoActivo).observe(lienzo.parentElement);
new ResizeObserver(ajustarLienzoActivo).observe(viewVideoEl);
Escena.ajustarLienzo();

// Visor alto o bajo, solo en telefono (el boton no se ve en pantallas
// anchas). La clase cambia --visor-h; los ResizeObserver de arriba
// redibujan solos. Envuelto en try como la vista: sin almacenamiento solo
// no se recuerda.
const bVisor = document.getElementById("bVisor");
function fijarVisorGrande(grande) {
  document.querySelector(".app").classList.toggle("visor-grande", grande);
  const texto = grande ? "Achicar mapa" : "Agrandar mapa";
  bVisor.setAttribute("aria-pressed", String(grande));
  bVisor.setAttribute("aria-label", texto);
  bVisor.title = texto;
  bVisor.querySelector("use").setAttribute("href", grande ? "#i-achicar" : "#i-agrandar");
}
bVisor.onclick = () => {
  const grande = bVisor.getAttribute("aria-pressed") !== "true";
  fijarVisorGrande(grande);
  try { localStorage.setItem("flota.visorGrande", grande ? "1" : "0"); } catch (e) { /* no se recuerda */ }
};
try { fijarVisorGrande(localStorage.getItem("flota.visorGrande") === "1"); } catch (e) { /* arranca bajo */ }

/* ============================================================
   VISTA 3D
   ============================================================
   La misma cancha, dibujada por escena3d.js. El estado, la fisica y la
   interaccion son los de siempre: lo unico que cambia es quien dibuja y de
   donde sale el punto del mundo bajo el raton (un rayo contra el piso en
   vez de deshacer la escala del lienzo). Por eso arrastrar solo mueve en XY
   y Shift sigue girando: es el mismo codigo de alMover().

   El modulo se pide la primera vez que se abre 3D. Si no hay WebGL o algo
   falla al cargar, se vuelve a 2D y se dice por que. */
const vista3dEl = document.getElementById("vista3d");
const aviso3d = document.getElementById("aviso3d");
let escena3d = null;
let cargando3d = null;

function en3D() { return estado.vista === "3d" && !esReal() && !!escena3d; }

function cargarEscena3d() {
  if (!cargando3d) {
    cargando3d = import("/js/escena3d.js")
      .then(m => {
        escena3d = m.crear(vista3dEl, { cfg: () => CFG });
        const el = escena3d.lienzo;
        el.addEventListener("mousedown", alPresionar);
        el.addEventListener("mousemove", alPasarEncima);
        el.addEventListener("mouseleave", alSalirDelLienzo);
        el.addEventListener("touchstart", alPresionar, { passive: false });
        new ResizeObserver(ajustarLienzoActivo).observe(vista3dEl);
        return escena3d;
      });
  }
  return cargando3d;
}

function refrescarVista() {
  const tresD = estado.vista === "3d" && !esReal();
  document.getElementById("barraVista").hidden = esReal();
  document.querySelectorAll("#segVista button").forEach(b =>
    b.classList.toggle("on", b.dataset.vista === estado.vista));
  document.getElementById("segCamara3d").hidden = !tresD || !escena3d;
  lienzo.hidden = tresD;
  vista3dEl.hidden = !tresD;
  if (tresD && escena3d) escena3d.ajustar();
}

function fijarVista(vista) {
  estado.vista = vista;
  try { localStorage.setItem("vista", vista); } catch (e) { /* sin almacenamiento: solo no se recuerda */ }
  bajoCursor = null;
  arrastrando = null;
  if (vista === "3d" && !escena3d) {
    aviso3d.hidden = false;
    aviso3d.textContent = "Cargando vista 3D…";
    refrescarVista();
    cargarEscena3d()
      .then(() => { aviso3d.hidden = true; refrescarVista(); })
      .catch(err => {
        console.error("[3D]", err);
        cargando3d = null;
        estado.vista = "2d";
        refrescarVista();
        // El boton se queda apagado con el motivo a la vista: volver a
        // picarle daria el mismo error.
        const b3d = document.querySelector('#segVista button[data-vista="3d"]');
        b3d.disabled = true;
        b3d.title = "Vista 3D no disponible: " + (err && err.message || err);
      });
    return;
  }
  refrescarVista();
  ajustarLienzoActivo();
}

document.querySelectorAll("#segVista button").forEach(b => {
  b.onclick = () => fijarVista(b.dataset.vista);
});
document.querySelectorAll("#segCamara3d button").forEach(b => {
  b.onclick = () => { if (escena3d) escena3d.vista(b.dataset.camara); };
});

/* ============================================================
   CINEMATICA DIRECTA DE ATLAS (mecanum, desde la trama del servidor)
   ============================================================ */
function directaAtlas(trama) {
  const n = {};
  CFG.orden.forEach((o, i) => { n[o.rueda] = trama[i] / CFG.max_raw / o.signo; });
  const vy = (n.FL + n.FR + n.RL + n.RR) / 4;
  let vx = (n.FL - n.FR - n.RL + n.RR) / 4;
  let wn = (-n.FL + n.FR - n.RL + n.RR) / 4;
  // Los mismos espejos que cinematica_directa() en Python, en el mismo
  // orden. Si el sprite y el robot no se mueven al mismo lado, la
  // simulacion deja de servir para nada.
  if (CFG.config_rodillos === "O") vx = -vx;
  if (CFG.invertir_lateral) vx = -vx;
  if (CFG.invertir_giro) wn = -wn;
  return { adelante: vy * CFG.v_max_px, derecha: vx * CFG.v_max_px, giro: wn * CFG.w_max_rad * CFG.autoridad };
}

// Inversa de directaAtlas(): las mismas ecuaciones que mezcla_mecanum()
// en cinematica.py, con la normalizacion por el pico incluida. Se escribe
// aqui y no se le pide al servidor porque una flecha tiene que responder
// al soltarla, no despues de un viaje de ida y vuelta.
//
// El signo de cada rueda sale de CFG.orden, que es FRAME_LAYOUT tal cual:
// si algun dia se vuelve a medir y cambia, esto sigue mandando la trama
// correcta sin tocarlo.
function tramaMecanum(adelante, derecha, giro, potencia) {
  // En configuracion "O" los rodillos apuntan al reves y el eje lateral se
  // invierte; el avance no cambia en ninguna de las dos. Y encima van los
  // espejos medidos sobre el robot -invertir_lateral, invertir_giro-, en el
  // mismo orden que mezcla_mecanum() en Python.
  let vx = derecha, w = giro;
  if (CFG.config_rodillos === "O") vx = -vx;
  if (CFG.invertir_lateral) vx = -vx;
  if (CFG.invertir_giro) w = -w;
  const n = {
    FL: adelante + vx - w,
    FR: adelante - vx + w,
    RL: adelante - vx - w,
    RR: adelante + vx + w,
  };
  // Si alguna rueda se pasa de 1 se escalan TODAS por igual: recortarlas
  // una a una deformaria la mezcla y el robot saldria torcido.
  const pico = Math.max(1, ...CFG.orden.map(o => Math.abs(n[o.rueda])));
  return CFG.orden.map(o => Math.round(n[o.rueda] / pico * o.signo * potencia));
}

// Lo mismo para los uniciclo, en el orden de la pagina: [derecha, izquierda].
// Girar a la izquierda es la rueda derecha adelante y la izquierda atras,
// que es justo lo que hace crecer th (antihorario en pantalla).
function tramaDiferencial(adelante, giro, potencia) {
  const der = adelante + giro, izq = adelante - giro;
  const pico = Math.max(1, Math.abs(der), Math.abs(izq));
  return [Math.round(der / pico * potencia), Math.round(izq / pico * potencia)];
}

function agregarRastro(robot, t) {
  const u = robot.rastro[robot.rastro.length - 1];
  if (!u || Math.hypot(robot.pose.x - u.x, robot.pose.y - u.y) > PASO_RASTRO_PX) {
    robot.rastro.push({ x: robot.pose.x, y: robot.pose.y, t: t });
  }
  // La caducidad va por EDAD y corre siempre, se mueva o no el robot: si
  // solo se recortara al insertar, al converger al destino dejaria de
  // entrar gente nueva y la cola se quedaria congelada con la linea vieja.
  while (robot.rastro.length && t - robot.rastro[0].t > VENTANA_RASTRO_S) robot.rastro.shift();
  if (robot.rastro.length > MUESTRAS_RASTRO) robot.rastro.shift();
}

// integrar=false en modo Real: ahi la pose la dicta el ArUco y ya la
// escribio cicloAtlas(). Integrar ademas la trama seria contar el
// movimiento dos veces, y la pose dibujada se despegaria de la real.
function avanzarAtlas(robot, dt, t, integrar) {
  const v = directaAtlas(robot.red.trama);
  if (integrar !== false) {
    const c = Math.cos(robot.pose.th), s = Math.sin(robot.pose.th);
    robot.pose.x += (v.adelante * c + v.derecha * s) * dt;
    robot.pose.y += (v.adelante * s - v.derecha * c) * dt;
    robot.pose.th = envolver(robot.pose.th + v.giro * dt);
    const mx = CFG.ancho / 2, my = CFG.alto / 2;
    robot.pose.x = lim(robot.pose.x, -mx, mx);
    robot.pose.y = lim(robot.pose.y, -my, my);
  }
  robot.velocidad = { vx: v.adelante, vy: v.derecha, w: v.giro };
  robot.pwm = robot.red.trama.slice();
  agregarRastro(robot, t);
}

// Avance en LAZO ABIERTO: la trama la escribe el usuario, no una ley de
// control. Se reusa la misma cinematica directa que el resto, para que la
// simulacion responda igual que si la trama viniera del backend.
function avanzarManual(robot, dt, t) {
  const m = robot.parametros.manual;
  const trama = m.activo ? m.pwm.slice() : robot.pwm.map(() => 0);
  if (robot.tipo === "holonomico") {
    robot.red.trama = trama;
    avanzarAtlas(robot, dt, t);
    return;
  }
  // Uniciclo: [derecha, izquierda], el orden que usa el resto de la pagina.
  // Misma forma que la inversa de diferencial() en control.js:
  //   wDer = V/r + LW/2r ,  wIzq = V/r - LW/2r
  // Lo que cambia es el fondo de escala del PWM: W_RUEDA_PWM255, no la
  // saturacion del lazo. Ver la nota larga en control.js.
  const der = trama[0] / 255 * W_RUEDA_PWM255, izq = trama[1] / 255 * W_RUEDA_PWM255;
  const V = (der + izq) / 2 * RADIO_LLANTA;
  const W = (der - izq) * RADIO_LLANTA / ENTRE_LLANTAS;
  robot.pose.x += V * Math.cos(robot.pose.th) * dt;
  robot.pose.y += V * Math.sin(robot.pose.th) * dt;
  robot.pose.th = envolver(robot.pose.th + W * dt);
  const mx = CFG.ancho / 2, my = CFG.alto / 2;
  robot.pose.x = lim(robot.pose.x, -mx, mx);
  robot.pose.y = lim(robot.pose.y, -my, my);
  robot.velocidad = { v: V, w: W };
  robot.pwm = trama;
  agregarRastro(robot, t);
}

/* ============================================================
   OBJETIVO SEGUN MODELO (comun a los 3 tipos de control)
   ============================================================ */
// Los offsets objetivo, en pixeles y con centroide en el origen, a partir
// de los puntos de rejilla que el usuario haya dibujado.
//
// La escala NO es "un cuadro de rejilla = tantos pixeles". Se normaliza por
// la separacion MEDIA entre parejas, de modo que el riel "Separación" sigue
// valiendo lo que dice -pixeles entre robots- salga la figura que salga. Con
// el triangulo de siempre, donde las tres distancias son iguales, da
// exactamente el mismo numero que antes: el riel no cambia de significado.
function construirZdFormacion(a) {
  const P = estado.formacion.puntos;
  const n = P.length;
  const cx = P.reduce((s, p) => s + p[0], 0) / n;
  const cy = P.reduce((s, p) => s + p[1], 0) / n;
  const Q = P.map(p => [p[0] - cx, p[1] - cy]);

  let suma = 0, pares = 0;
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      suma += Math.hypot(Q[i][0] - Q[j][0], Q[i][1] - Q[j][1]);
      pares++;
    }
  }
  // Los tres puntos encimados: no hay figura que escalar, y dividir por
  // cero mandaria NaN a la ley de control y de ahi al dibujo.
  const media = pares ? suma / pares : 0;
  const k = media > 1e-6 ? a / media : 0;

  const th = estado.formacion.angulo;
  const c = Math.cos(th), sn = Math.sin(th);
  return Q.map(p => {
    const x = p[0] * k, y = p[1] * k;
    return [x * c - y * sn, x * sn + y * c];
  });
}

// Donde deberia estar cada vertice de la formacion AHORA MISMO. Zd son
// offsets con centroide en el origen, asi que el triangulo objetivo es
// esos offsets colgados del centroide actual de los robots (la ley de
// consenso conserva ese centroide, no lo arrastra).
function verticesFormacion() {
  const Z = estado.robots.map(r => [r.pose.x, r.pose.y]);
  const cx = Z.reduce((s, p) => s + p[0], 0) / Z.length;
  const cy = Z.reduce((s, p) => s + p[1], 0) / Z.length;
  return construirZdFormacion(estado.formacion.a).map(d => [cx + d[0], cy + d[1]]);
}

// Calcula, para TODOS los robots en modo formacion, su velocidad deseada
// (ux,uy) segun control_formacion (control.js). Se hace una sola vez por
// tick para las 3 posiciones a la vez (necesita verlas todas).
//
// Ya no suma repulsion aqui. La que evita que se choquen es el campo propio
// de cada robot, que calcularObjetivo() añade despues igual que en los otros
// tres modos: una sola repulsion, con un solo juego de mandos, y visible en
// la cancha. Antes habia dos -esta y la de los obstaculos- sumandose sin que
// se pudiera ver de donde salia cada empujon.
function calcularFormacionGlobal() {
  const Z = estado.robots.map(r => [r.pose.x, r.pose.y]);
  const Zd = construirZdFormacion(estado.formacion.a);
  const A = grafoCompleto(Z.length);
  return controlFormacion(Z, Zd, A, estado.formacion.k);
}

/* ============================================================
   OBSTACULOS
   ============================================================
   Circulos con un campo repulsivo dentro. Reusan tal cual las cuatro
   formulas de control.js -las mismas que la repulsion entre robots de la
   formacion, que salen de formacion.py-, asi que no hay una segunda ley de
   control que mantener: lo unico propio de aqui es que el repulsor es un
   punto que puede moverse solo, y no otro robot.

   El id NO se deriva de la posicion en la lista. Con un contador aparte,
   borrar el primero y añadir otro no reutiliza su id: si lo reutilizara,
   un arrastre en curso o la ficha seleccionada apuntarian de pronto a un
   obstaculo distinto del que se estaba tocando. */
let contadorObs = 0;

function anadirObstaculo() {
  if (estado.obstaculos.length >= MAX_OBSTACULOS) return null;
  const cuna = CUNA_OBSTACULOS[estado.obstaculos.length] || CUNA_OBSTACULOS[0];
  contadorObs++;
  const o = {
    id: "obs" + contadorObs,
    // ancla: el centro del CAMINO, que es lo que se arrastra y lo unico
    // que no se mueve solo. pos: donde esta el obstaculo AHORA, que es lo
    // que repele y lo que se dibuja. Con "fijo" son el mismo punto.
    ancla: { x: cuna.x, y: cuna.y },
    pos: { x: cuna.x, y: cuna.y },
    // 120 y 600, no 90 y 400. Medido en el banco con Apolo cruzando de
    // lado a lado: con 90/400 se aparta 25 px y a ojo parece que el
    // obstaculo no esta haciendo nada; con 120/600 se aparta 57 px y se ve
    // rodear. Manda mucho mas el radio que la intensidad -de 90 a 150 el
    // desvio se triplica, y de eta 400 a 1000 solo sube de 25 a 36-, porque
    // el normalizado vale cero en el borde y solo aprieta hondo adentro.
    radio: 120,
    eta: 600,
    // Solo lo usa "espiral": +1 antihorario, -1 horario. Se guarda siempre
    // para que cambiar de tipo y volver no lo pierda.
    sentido: 1,
    // ESPIRAL por defecto, y esto sale de medirlo, no de gusto.
    //
    // Los tres repulsores radiales empujan derecho hacia afuera, asi que
    // entrando de frente al centro no hay componente lateral que desvie:
    // el robot no rodea, se PLANTA. Medido con Atlas cruzando la cancha
    // contra un obstaculo de radio 120 y eta 600:
    //
    //   lineal_norm   0 px de desvio, se queda clavado a 250 px
    //   espiral       158 px de desvio, lo rodea y sigue
    //
    // Y con Atlas pasa casi siempre, porque su chasis mide 338 px de ancho:
    // para un cuerpo asi, cualquier acercamiento es "de frente".
    //
    // Los otros tres siguen ahi y funcionan de sobra con Apolo y Odiseo
    // (131, 146 y 116 px de desvio), pero un obstaculo es algo que hay que
    // RODEAR, y el unico que rodea es este.
    tipo: "espiral",
    mov: "fijo",
    largo: 400, angulo: 0,   // vaiven en recta
    orbita: 180,             // tamaño del circulo y del ocho
    vel: 0.8,                // rad/s de la fase; el signo invierte el sentido
    fase: 0,
    cerca: 0,                // 0..1: cuanto de dentro esta el robot mas proximo
    anim: 0,                 // fase de la marcha de las flechas, 0..1
  };
  estado.obstaculos.push(o);
  return o;
}

function obstaculoPorId(id) { return estado.obstaculos.find(o => o.id === id); }
function obsSeleccionado() { return obstaculoPorId(estado.seleccionadoObs); }

// Coloca cada obstaculo sobre su camino y mide que tan cerca le anda el
// robot mas proximo. Se llama en TODOS los cuadros, tambien en pausa (con
// dt=0): en pausa la fase no avanza, pero arrastrar el ancla tiene que
// mover el circulo igual, o el obstaculo se quedaria clavado hasta darle
// a correr.
function sincronizarObstaculos(dt) {
  estado.obstaculos.forEach(o => {
    if (o.mov === "fijo") {
      o.pos.x = o.ancla.x; o.pos.y = o.ancla.y;
    } else {
      o.fase += o.vel * dt;
      if (o.mov === "recta") {
        // Seno y no diente de sierra: el vaiven tiene que frenar y volver,
        // no teletransportarse de un extremo al otro.
        const a = o.angulo * Math.PI / 180, d = Math.sin(o.fase) * o.largo / 2;
        o.pos.x = o.ancla.x + Math.cos(a) * d;
        o.pos.y = o.ancla.y + Math.sin(a) * d;
      } else if (o.mov === "lemniscata") {
        // El mismo ocho que puede recorrer un robot, de control.js: no hay
        // por que tener dos lemniscatas distintas en la misma pagina.
        const p = trayectoriaLemniscata(o.fase, [o.ancla.x, o.ancla.y], o.orbita, 1);
        o.pos.x = p[0]; o.pos.y = p[1];
      } else {
        o.pos.x = o.ancla.x + o.orbita * Math.cos(o.fase);
        o.pos.y = o.ancla.y + o.orbita * Math.sin(o.fase);
      }
    }

    medirCercania(o, o.pos, o.radio, null);
    animar(o, dt);
  });

  // Y el campo que lleva encima cada robot, que mide lo mismo respecto de
  // los OTROS robots.
  robotsEnEscena().forEach(r => {
    const rp = r.repulsor;
    if (!rp || rp.eta <= 0) { rp.cerca = 0; rp.hacia = null; return; }
    medirCercania(rp, r.pose, rp.radio, r);
    animar(rp, dt);
  });
}

// Que tan metido esta el robot mas proximo (0..1) y POR DONDE viene. Lo
// segundo es lo que deja pintar el foco del lado correcto: un degradado
// concentrico se ve igual por los cuatro costados y no puede decir que el
// campo esta apretando justo ahi.
function medirCercania(campo, centro, radio, propietario) {
  let cerca = 0, hacia = null;
  for (const r of robotsEnEscena()) {
    if (r === propietario) continue;
    // En Manual no hay ley de control que desviar, asi que el campo no
    // empuja: seria el panel peleandose con tus flechas.
    if (r.modelo === "manual") continue;
    const dx = r.pose.x - centro.x, dy = r.pose.y - centro.y;
    // Mismo criterio que fuerzaDeCampo: el chasis, no el centro. Si no, las
    // flechas se encenderian mucho despues de que el campo ya esta
    // empujando, y el dibujo estaria mintiendo sobre lo que pasa.
    const d = Math.max(0, Math.hypot(dx, dy) - radioCuerpo(r));
    const c = 1 - Math.min(1, d / radio);
    if (c > cerca) {
      cerca = c;
      const n = Math.hypot(dx, dy);
      hacia = n > 1e-6 ? { x: dx / n, y: dy / n } : null;
    }
  }
  campo.cerca = cerca;
  campo.hacia = hacia;
}

// Las flechas solo marchan si hay a quien empujar y el reloj corre.
// Quietas dicen "soy un repulsor de este tipo"; en marcha dicen "estoy
// empujando a alguien ahora mismo".
function animar(campo, dt) {
  if (estado.corriendo) campo.anim = (campo.anim + campo.cerca * 1.1 * dt) % 1;
}

// Radio del CUERPO de un robot, en px de mundo. Sale de lo mismo que lo
// dibuja canvas.js -el ancho es ancho_robot_px * 1.35 * escalaImg-, asi que
// el circulo que se ve en la cancha y el que entra en la cuenta son el
// mismo. Aproximar el chasis por un circulo es de sobra para esto: lo que
// se estaba haciendo antes era aproximarlo por un PUNTO.
//
// En numeros: 169 px Atlas, 91 Odiseo, 61 Apolo.
function radioCuerpo(robot) {
  return (CFG.ancho_robot_px || 250) * 1.35 * (robot.escalaImg || 1) / 2;
}

// Fuerza que un campo -este en un obstaculo o encima de un robot- hace
// sobre un ROBOT. Un solo sitio: los dos usan las mismas formulas y las
// mismas trampas, y tenerlo dos veces era garantizar que se separaran.
//
// campo: { radio, eta, tipo, sentido }.
function fuerzaDeCampo(robot, cx, cy, campo) {
  const dx = robot.pose.x - cx, dy = robot.pose.y - cy;
  const dCentros = Math.hypot(dx, dy);
  if (dCentros < 1e-6 || campo.eta <= 0) return [0, 0];

  // LO QUE SE EMPUJA ES EL CHASIS, NO EL CENTRO. Esto era el fallo: se
  // median centro a centro, y un robot mide 122 px de ancho (Apolo), 183
  // (Odiseo) o 338 (Atlas). Con un campo de 80 px de radio alrededor de
  // Apolo, el CENTRO de otro robot solo entra si los dos chasis estan uno
  // encima del otro -o sea, nunca-, asi que el campo no disparaba jamas y
  // mover el radio no cambiaba absolutamente nada.
  //
  // Ahora "d" es lo que le falta al chasis del robot para llegar al centro
  // del campo. El empujon arranca justo cuando su contorno cruza el circulo
  // punteado que se ve dibujado, que es lo que cualquiera esperaria.
  const d = Math.max(0.5, dCentros - radioCuerpo(robot));
  // Fuera del circulo, cero. Es el contrato de las cuatro formulas y lo que
  // hace que el dibujo no mienta: si no lo toca, no lo empuja.
  if (d >= campo.radio) return [0, 0];

  // La magnitud la decide magnitudCampo() en control.js: alli estan las
  // cuatro formulas a escala de pixeles y el recorte a eta, y las usa
  // tambien el dibujo, para que las flechas no puedan mentir sobre la
  // fuerza que hay debajo.
  const mag = magnitudCampo(d, campo.radio, campo.eta, campo.tipo);
  if (mag === 0) return [0, 0];

  // Lo unico propio del espiral es la DIRECCION: su magnitud es la del
  // normalizado y lo que lo hace espiral es torcer el empujon GIRO_ESPIRAL
  // grados hacia el lado que diga el sentido. Ver la nota en config.js.
  const espiral = campo.tipo === "espiral";

  // La direccion sale de centro a centro, no del chasis: es de donde a
  // donde empuja, y eso no depende de lo gordo que sea el robot.
  let ux = dx / dCentros, uy = dy / dCentros;
  if (espiral) {
    const g = GIRO_ESPIRAL * (campo.sentido < 0 ? -1 : 1);
    const c = Math.cos(g), sn = Math.sin(g);
    const rx = ux * c - uy * sn;
    uy = ux * sn + uy * c;
    ux = rx;
  }
  return [mag * ux, mag * uy];
}

// Todo lo que empuja a este robot: los obstaculos, y el campo propio de
// cada uno de los DEMAS robots. El suyo no, obviamente: un campo clavado a
// tu centro solo podria empujarte a ti mismo.
function repulsionSobre(robot) {
  let fx = 0, fy = 0;
  for (const o of estado.obstaculos) {
    const [ax, ay] = fuerzaDeCampo(robot, o.pos.x, o.pos.y, o);
    fx += ax; fy += ay;
  }
  for (const otro of robotsEnEscena()) {
    if (otro === robot) continue;
    const [ax, ay] = fuerzaDeCampo(robot, otro.pose.x, otro.pose.y, otro.repulsor);
    fx += ax; fy += ay;
  }
  return [fx, fy];
}

// Devuelve [ux,uy]: la velocidad deseada en el mundo para un robot, segun
// su modelo actual.
//
// Recibe dt (no un tiempo absoluto) porque la fase de la trayectoria se
// integra paso a paso. De paso resuelve la pausa: si no corre, no se llama
// y la fase no avanza, en vez de seguir corriendo con el reloj del
// navegador y pegar un salto al reanudar.
function calcularObjetivo(robot, dt, formacionGlobal, idx) {
  const [ox, oy] = objetivoDelModelo(robot, dt, formacionGlobal, idx);
  const [rx, ry] = repulsionSobre(robot);
  // Lo apunta para la zona de llegada, que no puede cerrarse mientras algo
  // este empujando. Ver avanzarUniciclo().
  robot.repelido = rx !== 0 || ry !== 0;
  // Sin nada que esquivar, ni se toca: asi el lazo de siempre queda
  // exactamente como estaba, hasta en el orden de las operaciones.
  if (!robot.repelido) return [ox, oy];

  // SATURAR ANTES DE SUMAR, no despues. Esto no es un detalle: es lo que
  // hace que los obstaculos se noten.
  //
  // convergencia() es -k*(pose-destino), o sea que CRECE CON LA DISTANCIA y
  // no tiene techo: a 900 px del destino y con k=0.8 vale 720. La repulsion,
  // en cambio, vale como mucho eta y solo cerca del centro del circulo.
  // Sumando los dos crudos, un tiron de 720 contra un empujon de 300 apenas
  // tuerce la direccion un par de grados, y el recorte a VMAX que venia
  // despues escalaba los dos por igual, asi que tampoco arreglaba nada: el
  // robot pasaba por encima del obstaculo como si no estuviera.
  //
  // Recortando primero la atraccion a VMAX_DESEADA, las dos fuerzas juegan
  // en la misma escala -220 contra hasta 1000- y la repulsion manda de
  // verdad mientras el robot esta dentro del circulo. Es el pozo conico de
  // toda la vida: lejos del destino tira con fuerza constante, no infinita.
  const [sx, sy] = saturar(ox, oy, VMAX_DESEADA);
  return [sx + rx, sy + ry];
}

function objetivoDelModelo(robot, dt, formacionGlobal, idx) {
  if (robot.modelo === "trayectoria") {
    const t = robot.parametros.trayectoria;
    // Si el usuario ya definio un centro a mano (campos Centro X/Y), se
    // respeta; si no, se usa la pose actual como centro por defecto.
    if (!t.centroInicial) t.centroInicial = [robot.pose.x, robot.pose.y];
    // Fase ACUMULADA en vez de omega*(t-t0): asi cambiar la velocidad
    // angular cambia solo cuanto avanza de aqui en adelante, sin recalcular
    // la fase pasada con el omega nuevo (que hacia saltar el punto de
    // referencia, y se veia como si la trayectoria se reiniciara).
    t.fase += t.omega * dt;
    const p = t.tipo === "lemniscata"
      ? trayectoriaLemniscata(t.fase, t.centroInicial, t.radio, 1)
      : trayectoriaCirculo(t.fase, t.centroInicial, t.radio, 1);
    robot.destino.x = p[0]; robot.destino.y = p[1];
    return convergencia(robot.pose.x, robot.pose.y, p[0], p[1], robot.parametros.punto.k);
  }
  if (robot.modelo === "formacion") {
    return formacionGlobal[idx];
  }
  // punto
  return convergencia(robot.pose.x, robot.pose.y, robot.destino.x, robot.destino.y, robot.parametros.punto.k);
}

const VMAX_DESEADA = 220; // saturacion comun antes de entrar a uniciclo()/backend

// A cuanto tiempo vista se le pone a Atlas el punto que va a perseguir
// cuando no hay un destino fijo al que mandarlo (o cuando hay que
// desviarlo). Corto: es una zanahoria, no un destino, y se reescribe en el
// tick siguiente.
const HORIZONTE_ATLAS = 0.8;   // s

function puntoHorizonte(robot, ux, uy, th) {
  const [sx, sy] = saturar(ux, uy, VMAX_DESEADA);
  return {
    x: robot.pose.x + sx * HORIZONTE_ATLAS,
    y: robot.pose.y + sy * HORIZONTE_ATLAS,
    th: th,
  };
}

function saturar(ux, uy, vmax) {
  const n = Math.hypot(ux, uy);
  return n > vmax ? [ux / n * vmax, uy / n * vmax] : [ux, uy];
}

// Un uniciclo NO puede resolver posicion y rumbo a la vez: no se mueve de
// lado, asi que su orientacion es justamente lo que usa para desplazarse.
// Por eso va en dos fases: primero llega al punto (y mientras tanto su
// rumbo lo dicta el camino, no el destino), y solo cuando ya llego gira
// sobre si mismo hasta el rumbo pedido. Atlas no necesita esto: es
// holonomico y el backend le resuelve las dos cosas simultaneamente.
//
// Solo aplica al modelo Punto; en trayectoria y formacion el rumbo lo
// manda el camino y no hay un theta objetivo que perseguir.
function avanzarUniciclo(robot, ux, uy, dt, t) {
  const p = robot.parametros.punto;
  const dist = Math.hypot(robot.destino.x - robot.pose.x, robot.destino.y - robot.pose.y);
  // "&& !robot.repelido" es un arreglo, no un matiz. Dentro de la zona de
  // llegada este lazo pone V=0 y solo gira: da el punto por bueno y deja de
  // trasladarse. Con eso TIRABA A LA BASURA la repulsion -llegaba sumada en
  // (ux,uy) y nunca se usaba-, asi que un robot ya parado en su destino se
  // dejaba atravesar por un obstaculo sin apartarse un pixel, que es justo
  // lo que se veia con Apolo y Odiseo.
  //
  // La regla ahora: mientras algo te empuje NO has llegado. En cuanto el
  // campo se va -el obstaculo pasa de largo, o se apaga-, el robot vuelve a
  // asentarse solo. Y si el destino cae DENTRO de un campo, no se asienta
  // nunca: es la respuesta honesta, ese punto no es alcanzable sin meterse.
  const enPunto = robot.modelo === "punto" && dist < p.tolPos && !robot.repelido;

  let V, W;
  if (enPunto) {
    // Fase 2: quieto en sitio, girando hacia el rumbo pedido.
    V = 0;
    W = controlRumbo(robot.pose.th, robot.destino.th, undefined, undefined,
                     p.tolAng * Math.PI / 180);
    robot.fase = W === 0 ? "en destino" : "orientando";
  } else {
    // Fase 1: perseguir el punto. El rumbo sale del camino, no se controla.
    const [ux2, uy2] = saturar(ux, uy, VMAX_DESEADA);
    [V, W] = uniciclo(ux2, uy2, robot.pose.th, L_PUNTO);
    robot.fase = robot.modelo === "punto" ? "en ruta" : robot.modelo;
  }

  const [wDer, wIzq] = diferencial(V, W);

  robot.pose.x += V * Math.cos(robot.pose.th) * dt;
  robot.pose.y += V * Math.sin(robot.pose.th) * dt;
  robot.pose.th = envolver(robot.pose.th + W * dt);
  const mx = CFG.ancho / 2, my = CFG.alto / 2;
  robot.pose.x = lim(robot.pose.x, -mx, mx);
  robot.pose.y = lim(robot.pose.y, -my, my);

  robot.velocidad = { v: V, w: W };
  robot.pwm = [Math.round(wDer / W_MAXIMA * 255), Math.round(wIzq / W_MAXIMA * 255)];
  agregarRastro(robot, t);
}

/* ============================================================
   RED (solo Atlas)
   ============================================================ */
async function cargarConfig() {
  try {
    const r = await fetch("/config");
    CFG = await r.json();
    Escena.actualizarCFG(CFG);
    Graficas.actualizarCFG(CFG);
    // El slider no debe poder bajar de la escala minima util que reporta
    // cinematica.py: por debajo de ahi la zona muerta apaga las ruedas
    // secundarias y Atlas solo se mueve en las 8 direcciones canonicas
    // (adelante/atras, lateral y las 4 diagonales), no hacia donde se le
    // pide. No es un fallo del modelo, es el limite fisico del motor.
    if (CFG.escala_minima) {
      const minPct = Math.ceil(CFG.escala_minima * 100 / 5) * 5;
      document.getElementById("se").min = String(minPct);
    }
  } catch (e) { /* se reintenta en el proximo ciclo */ }
}

// Un fetch puede quedarse colgado para siempre. Pasa al irse a otra
// pestaña o a otra pagina: el documento se va al bfcache con una peticion
// en vuelo y al volver esa promesa no se resuelve NUNCA. Y como enVuelo es
// un cerrojo sin llave, ahi se acababa Atlas: /ciclo no se volvia a lanzar
// y el robot se quedaba sin control y sin repulsion -el destino se calcula
// en el bucle de dibujo, pero al robot solo llega por /ciclo-. Recargar la
// pagina entera era lo unico que lo revivia, porque reseteaba el cerrojo.
//
// Con el aborto la promesa SIEMPRE termina, y el finally abre el cerrojo.
const TIMEOUT_CICLO_MS = 1000;   // 20 ciclos: mas viejo que eso ya no sirve

// Numero de ciclo. Destrabar el cerrojo por reloj deja al ciclo viejo
// suelto: si esa promesa despierta despues -y al volver de una pestaña
// congelada despiertan todas de golpe- caia en el mismo objeto y hacia dos
// destrozos. Uno, escribia su respuesta rancia encima de la fresca, y en
// Real eso es la POSE: el sprite pegaba un salto a donde estaba el robot
// hace medio minuto. Dos, su finally abria el cerrojo del ciclo que si
// estaba en vuelo, y a partir de ahi se pisaban entre ellos.
//
// Con el numero, el que llega tarde se calla: ni escribe ni abre nada.
let cicloGen = 0;

async function cicloAtlas() {
  const atlas = robotPorId("atlas");
  if (atlas.red.enVuelo) {
    // Segunda red, por si el aborto tampoco llega: con la pestaña
    // congelada duerme tambien el temporizador que iba a abortar. El
    // reloj de pared no se congela, asi que esto se destraba al volver.
    if (Date.now() - atlas.red.despegue < TIMEOUT_CICLO_MS * 3) return;
    atlas.red.enVuelo = false;
  }
  const gen = ++cicloGen;
  atlas.red.enVuelo = true;
  atlas.red.despegue = Date.now();
  const corte = new AbortController();
  const reloj = setTimeout(() => corte.abort(), TIMEOUT_CICLO_MS);
  try {
    const r = await fetch("/ciclo", {
      method: "POST", signal: corte.signal, headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        x: atlas.pose.x, y: atlas.pose.y, th: atlas.pose.th,
        destino: atlas.red.destinoManual, escala: atlas.parametros.punto.escala, k: atlas.parametros.punto.k,
        // La zona de llegada viaja con cada ciclo: el servidor no guarda
        // estado del panel, y asi mover el slider se nota en el robot al
        // instante siguiente sin reiniciar nada.
        tol_px: atlas.parametros.punto.tolPos, tol_grados: atlas.parametros.punto.tolAng,
        // En real el servidor ignora la pose de arriba y usa la del ArUco,
        // y publica la trama al robot fisico. En simulacion no manda nada:
        // ni tag ni enviar, y el lazo se queda en el navegador.
        //
        // "armado" es el interruptor general: sin el, el lazo sigue
        // calculando y dibujando, pero no sale nada por MQTT. Y en Manual
        // tampoco sale de aqui: ahi manda la cruceta, y las dos cosas a la
        // vez serian la ley de control peleandose con tus flechas a 20 Hz.
        //
        // Ni el tag ni el envio miran ya la vista: armado desde el mapa el
        // robot obedece igual. El tag se manda siempre que haya alguien
        // escuchando, porque es lo que cierra el lazo con la camara, y el
        // servidor solo publica si ademas lo esta VIENDO (ver /ciclo en
        // servidor_sandbox.py): en Punto, Trayectoria y Formacion, sin ArUco
        // detectado no sale una sola orden por mucho que este armado.
        tag: (esReal() || atlas.armado) ? atlas.tagAruco : null,
        enviar: estado.corriendo && atlas.armado && atlas.modelo !== "manual",
        robot: "atlas",
      }),
    });
    const d = await r.json();
    if (gen !== cicloGen) return;   // llego tarde: ya manda otro, esto es basura
    atlas.red.trama = d.valores;
    atlas.red.info = d.info;
    atlas.red.camara = d.camara;
    atlas.red.tagVisto = !!d.tag_visto;
    // La camara puede abrir tarde, caerse a media sesion o cambiarse desde
    // el selector. CFG viene de /config, que solo se lee al cargar.
    if (d.camara_viva !== undefined) CFG.camara_viva = d.camara_viva;
    aplicarEstadoCamara(d.camara_estado);
    atlas.red.enviado = d.enviado;
    // La pose REAL pisa a la simulada. Solo si el marcador se ve: si se
    // pierde, se conserva la ultima conocida en vez de saltar al origen,
    // que se veria como si el robot se hubiera teletransportado.
    if (d.tag_visto && d.pose_usada) {
      atlas.pose.x = d.pose_usada.x;
      atlas.pose.y = d.pose_usada.y;
      atlas.pose.th = d.pose_usada.th;
    }
    if (!atlas.red.destinoManual && d.destino) {
      atlas.destino.x = d.destino.x; atlas.destino.y = d.destino.y; atlas.destino.th = d.destino.th;
    }
    atlas.red.conectadoReal = true;
    atlas.red.ultimoContacto = performance.now();
  } catch (e) {
    if (gen === cicloGen) atlas.red.conectadoReal = false;
  } finally {
    clearTimeout(reloj);
    // Solo el ciclo vigente cierra el cerrojo. Un rezagado que lo abriera
    // dejaria dos peticiones en vuelo contra el mismo estado.
    if (gen === cicloGen) atlas.red.enVuelo = false;
  }
}
setInterval(cicloAtlas, PERIODO_MS);
cargarConfig();

/* ============================================================
   CAMARA DEL SERVIDOR
   ============================================================
   La camara la abre Python, no el navegador: el detector de ArUco corre
   ahi. Asi que esto no es getUserMedia - es un selector que le dice al
   servidor cual abrir, y el video sigue llegando por /video.mjpg.

   Dos canales, por lo que cuesta cada uno: el estado corto (que indice
   esta abierto, si esta buscando) viaja gratis en cada respuesta de
   /ciclo, que ya corre a 20 Hz; la LISTA de camaras, que obliga a cerrar
   la camara y probar indice por indice, solo se pide cuando hace falta. */
const camBar = document.getElementById("camBar"),
      camSelect = document.getElementById("camSelect"),
      camBuscar = document.getElementById("camBuscar"),
      videoImg = document.getElementById("videoImg");

const cam = { indice: null, lista: null, escaneando: false, error: null };
let camaraVivaPrev = false;

function etiquetaCamara(c) {
  const nombre = c.nombre || "Cámara " + c.indice;
  return c.indice + " · " + nombre + (c.ancho ? "  " + c.ancho + "×" + c.alto : "");
}

function pintarPanelCamara() {
  // El <select> se arma con la API del DOM y no con innerHTML: el nombre
  // lo da el sistema operativo y no hay por que confiar en que venga
  // limpio de caracteres con significado en HTML.
  const opciones = [["", "— sin cámara —"]];
  (cam.lista || []).forEach(c => opciones.push([String(c.indice), etiquetaCamara(c)]));
  // La abierta puede no estar en la lista: si se escaneó antes de
  // conectarla, o si el servidor arrancó con un --camara que no salió.
  if (cam.indice !== null && !opciones.some(o => o[0] === String(cam.indice)))
    opciones.push([String(cam.indice), cam.indice + " · en uso"]);

  camSelect.innerHTML = "";
  opciones.forEach(o => camSelect.add(new Option(o[1], o[0])));
  camSelect.value = cam.indice === null ? "" : String(cam.indice);
  camBar.classList.toggle("buscando", cam.escaneando);

  // El cartel del visor dice lo que toca hacer ahora, no una instrucción
  // fija: antes decía "arranca con --camara <índice>", que ya no es cierto.
  const titulo = document.getElementById("vacioTitulo"),
        pista = document.getElementById("vacioPista");
  if (cam.escaneando) {
    titulo.textContent = "Buscando cámaras…";
    pista.textContent = "va una por una, tarda unos segundos";
  } else if (cam.error) {
    titulo.textContent = "No se pudo abrir la cámara";
    pista.textContent = cam.error;
  } else if (cam.lista && cam.lista.length === 0) {
    titulo.textContent = "Ninguna cámara respondió";
    pista.textContent = "conéctala y vuelve a buscar con ⟳";
  } else {
    titulo.textContent = "Sin cámara";
    pista.textContent = cam.lista ? "elígela en el selector de arriba"
                                  : "búscalas con ⟳, arriba a la izquierda";
  }
}

// El estado corto, el que llega en cada /ciclo.
function aplicarEstadoCamara(e) {
  if (!e) return;
  const escaneando = !!e.escaneando, error = e.error || null;
  const cambio = e.indice !== cam.indice || escaneando !== cam.escaneando || error !== cam.error;
  const acaboEscaneo = cam.escaneando && !escaneando;
  cam.indice = e.indice; cam.escaneando = escaneando; cam.error = error;

  // La resolución de la cámara ES el tamaño del mundo: mapa, retícula y
  // guías se dibujan en píxeles de imagen. Pasar de una 720p a una VGA sin
  // actualizar esto dejaría todas las guías corridas respecto al video que
  // tienen debajo.
  if (e.ancho && e.alto && (e.ancho !== CFG.ancho || e.alto !== CFG.alto)) {
    CFG.ancho = e.ancho; CFG.alto = e.alto;
  }

  // El <img> del MJPEG: si la cámara se fue y volvió, la conexión anterior
  // puede haber quedado muerta y el navegador no la reabre solo. Se vuelve
  // a pedir con un parámetro distinto para que no reuse la de antes.
  if (e.viva && !camaraVivaPrev) refrescarFuenteVideo();
  camaraVivaPrev = !!e.viva;

  // El cartel se apaga aquí y no solo en el bucle de dibujo: este camino
  // corre con el temporizador de /ciclo, que sigue vivo aunque la pestaña
  // esté en segundo plano y requestAnimationFrame no dispare.
  document.getElementById("videoVacio").hidden = !!e.viva;

  if (acaboEscaneo) cargarCamaras();   // hay lista nueva que leer
  else if (cambio) pintarPanelCamara();
}

async function cargarCamaras() {
  try {
    const d = await (await fetch("/camaras")).json();
    if (d.lista !== undefined) cam.lista = d.lista;
    aplicarEstadoCamara(d);
    pintarPanelCamara();
  } catch (e) { /* se reintenta al volver a Real */ }
}

async function pedirCamara(cuerpo) {
  try {
    const d = await (await fetch("/camaras", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(cuerpo),
    })).json();
    if (d.lista !== undefined) cam.lista = d.lista;
    aplicarEstadoCamara(d);
    pintarPanelCamara();
  } catch (e) {
    // Se fue la pildora que rotulaba esto en la barra de pestanas. Hasta
    // que tenga otro sitio donde verse, queda en la consola: perderlo del
    // todo seria quedarse sin saber por que dejo de responder.
    console.warn("[atlas] sin conexión al servidor");
  }
}

// Tocar la cámara con el lazo corriendo le quita el ojo al control: la
// pose deja de llegar y el robot se quedaría con la última orden hasta que
// lo frene el watchdog. Se para aquí, a la vista, en vez de dejarlo pasar.
function pausarPorCamara() {
  if (!esReal() || !estado.corriendo) return;
  estado.corriendo = false;
  UI.actualizarBotonCorrer(estado);
  pararRobotReal();
}

camSelect.onchange = () => {
  pausarPorCamara();
  pedirCamara({ indice: camSelect.value === "" ? null : +camSelect.value });
};
camBuscar.onclick = () => { pausarPorCamara(); pedirCamara({ escanear: true }); };
cargarCamaras();

/* ============================================================
   INTERACCION EN EL CANVAS
   ============================================================ */
let arrastrando = null;

const RADIO_ASA_CENTRO = 55;  // px de mundo: que tan cerca hay que picarle a la cruz
// El asa del destino baja con el marcador: a 48 -el radio del anillo viejo-
// se agarraba a un palmo de un blanco que ahora mide la mitad, y el destino
// saltaba al picar en lo que se veia como suelo vacio. Sigue siendo algo
// mayor que el dibujo, que para eso es un asa.
const RADIO_ASA_DESTINO = 32; // idem para el marcador de destino

// Que hay bajo el cursor, en orden de prioridad. Lo usan por igual el
// mousedown (para saber que se agarra) y el hover (para decidir el cursor
// y que pieza se realza), asi el puntero nunca promete algo que el clic no
// vaya a hacer.
//
// El orden importa: las asas chicas y precisas ganan sobre el cuerpo del
// robot, que es grande y se las come. Sin esto, un destino que caiga sobre
// un chasis es inagarrable.
// Real + Manual: sobre el video no va ninguna guia. Vive aqui, y no suelto
// en el bucle, porque el raton tiene que leer lo mismo: un asa que no se ve
// no es un atajo, es una trampa -moverias un destino invisible y lo
// descubririas al volver a Punto, con el robot ya yendose hacia el.
function sinGuias() {
  const sel = UI.robotSeleccionado(estado);
  return esReal() && !!sel && sel.modelo === "manual";
}

// cliente: coordenadas de pantalla del evento. Solo las usa la vista 3D,
// para el raycast contra los cuerpos altos (ver mas abajo).
function objetoBajo(m, cliente) {
  if (sinGuias() || !m) return null;
  for (const r of robotsEnEscena()) {
    if (r.modelo !== "trayectoria") continue;
    const c = r.parametros.trayectoria.centroInicial;
    if (c && Math.hypot(m.x - c[0], m.y - c[1]) < RADIO_ASA_CENTRO) {
      return { tipo: "centro", robot: r };
    }
  }
  for (const r of robotsEnEscena()) {
    // Solo en Punto: en trayectoria el destino es un punto movil que se
    // recalcula cada tick, agarrarlo no significaria nada.
    if (r.modelo !== "punto") continue;
    if (Math.hypot(m.x - r.destino.x, m.y - r.destino.y) < RADIO_ASA_DESTINO) {
      return { tipo: "destino", robot: r };
    }
  }
  // En 3D, robots y obstaculos se deciden con un rayo contra lo que se ve.
  // Son altos: picarle a la parte de arriba de un robot toca el piso muy
  // por detras de el, y por distancia sobre el piso se agarraria otra cosa
  // o nada. Gana lo que este mas cerca de la camara. El destino y la cruz
  // de arriba estan pegados al suelo, asi que ahi la distancia sigue valiendo.
  if (en3D() && cliente) {
    const p = escena3d.pick(cliente.x, cliente.y, panelALaVista === "obstaculos");
    // El agarre se mide en el plano a la altura del punto picado, y el
    // arrastre sigue en ese plano: asi la pieza no resbala bajo el cursor.
    const ma = p ? escena3d.aMundo(cliente.x, cliente.y, p.altura) : null;
    if (p && ma && p.tipo === "obstaculo") {
      const o = obstaculoPorId(p.id);
      if (o) return { tipo: "obstaculo", obs: o, altura: p.altura, agarre: { x: ma.x - o.ancla.x, y: ma.y - o.ancla.y } };
    } else if (p && ma && p.tipo === "robot") {
      const r = robotPorId(p.id);
      if (r) return { tipo: "pose", robot: r, altura: p.altura, agarre: { x: ma.x - r.pose.x, y: ma.y - r.pose.y } };
    }
  }
  // Los obstaculos SOLO se agarran con su pestaña a la vista. Son areas
  // grandes -hasta 360 px de radio- y agarrables siempre se comerian el
  // clic en vacio que pone el destino: querrias mandar el robot a un punto
  // dentro del circulo y en su lugar moverias el circulo. Es el mismo
  // criterio que ya gobierna el asa del centro de la trayectoria: el
  // contexto decide que se puede tocar.
  if (panelALaVista === "obstaculos") {
    // Al reves: el ultimo dibujado queda encima, y encima es lo que el
    // raton deberia agarrar cuando dos se solapan.
    for (let i = estado.obstaculos.length - 1; i >= 0; i--) {
      const o = estado.obstaculos[i];
      if (Math.hypot(m.x - o.pos.x, m.y - o.pos.y) < o.radio) {
        // El agarre se guarda para que el circulo no pegue un salto al
        // presionar: lo que sigue al raton es el punto por el que lo
        // cogiste, no su centro. Y lo que se mueve es el ANCLA, asi que
        // el camino entero viaja con el.
        return { tipo: "obstaculo", obs: o,
                 agarre: { x: m.x - o.ancla.x, y: m.y - o.ancla.y } };
      }
    }
  }
  // En Real el cuerpo del robot NO se agarra: su pose la dicta el ArUco y
  // arrastrarla no haria nada, la camara la reescribe al ciclo siguiente.
  // Peor: el chasis es un area grande, asi que un clic cerca del robot
  // -queriendo poner el destino ahi- se lo comeria y pareceria que la
  // pagina no responde.
  if (!esReal()) {
    for (const r of robotsEnEscena()) {
      if (Math.hypot(m.x - r.pose.x, m.y - r.pose.y) < (CFG.ancho_robot_px || 250) / 2) {
        return { tipo: "pose", robot: r };
      }
    }
  }
  return null;
}

// altura: en 3D, la del plano en que se agarro la pieza (ver objetoBajo).
function puntoDeEvento(ev, altura) {
  const t = ev.touches ? ev.touches[0] : ev;
  return aMundoCliente(t.clientX, t.clientY, altura);
}

// Pantalla -> mundo, en la vista que este a la vista. En 3D puede dar null:
// el rayo apunta por encima del horizonte y no toca el piso.
function aMundoCliente(clientX, clientY, altura) {
  return en3D() ? escena3d.aMundo(clientX, clientY, altura) : Escena.aMundo(clientX, clientY);
}

function clienteDeEvento(ev) {
  const t = ev.touches ? ev.touches[0] : ev;
  return { x: t.clientX, y: t.clientY };
}

// Nada se coloca fuera de la cuadricula. El robot ya estaba acotado al
// integrar su pose, pero el destino no: dejarlo afuera daba un objetivo
// inalcanzable, y el robot se quedaba empujando contra el borde sin llegar
// nunca ni declarar que llego.
function limitarAlMapa(x, y) {
  const mx = CFG.ancho / 2, my = CFG.alto / 2;
  return { x: lim(x, -mx, mx), y: lim(y, -my, my) };
}

function alPresionar(ev) {
  // En 3D los otros dos botones son de la camara (orbitar, desplazar).
  if (en3D() && ev.button !== undefined && ev.button !== 0) return;
  ev.preventDefault();
  const m = puntoDeEvento(ev);
  if (!m) return;
  const hit = objetoBajo(m, clienteDeEvento(ev));
  if (hit && hit.tipo === "obstaculo") {
    seleccionarObstaculo(hit.obs.id);
    arrastrando = hit;
  } else if (hit) {
    seleccionarRobot(hit.robot.id);
    arrastrando = { tipo: hit.tipo, robot: hit.robot, girar: ev.shiftKey };
    // En 3D el robot se agarra por donde se pico, como ya hacen los
    // obstaculos: en perspectiva, que su centro salte al cursor se nota
    // mucho. En 2D no llega agarre y se queda como estaba.
    if (hit.agarre) { arrastrando.agarre = hit.agarre; arrastrando.altura = hit.altura; }
  } else {
    const sel = UI.robotSeleccionado(estado);
    if (sel.modelo !== "punto") return; // solo se puede fijar destino a mano en modo Punto
    arrastrando = { tipo: "destino", robot: sel, girar: ev.shiftKey };
  }
  alMover(ev);
}

function alMover(ev) {
  if (!arrastrando) return;
  ev.preventDefault();
  const m = puntoDeEvento(ev, arrastrando.altura);
  if (!m) return;

  if (arrastrando.tipo === "obstaculo") {
    const o = arrastrando.obs;
    const c = limitarAlMapa(m.x - arrastrando.agarre.x, m.y - arrastrando.agarre.y);
    o.ancla.x = c.x; o.ancla.y = c.y;
    // Recolocar YA: en pausa el lazo no vuelve a pasar por aqui, y sin
    // esto el circulo se quedaria donde estaba hasta darle a correr.
    sincronizarObstaculos(0);
    if (estado.seleccionadoObs === o.id) UI.refrescarCentroObstaculo(o);
    return;
  }

  const r = arrastrando.robot;

  if (arrastrando.tipo === "centro") {
    const c = limitarAlMapa(m.x, m.y);
    r.parametros.trayectoria.centroInicial = [c.x, c.y];
    if (UI.robotSeleccionado(estado).id === r.id) {
      document.getElementById("tCentroX").value = Math.round(c.x);
      document.getElementById("tCentroY").value = Math.round(c.y);
    }
    return;
  }

  const o = arrastrando.tipo === "pose" ? r.pose : r.destino;
  const ag = arrastrando.agarre || { x: 0, y: 0 };
  if (arrastrando.girar) {
    o.th = envolver(Math.atan2(m.y - o.y, m.x - o.x));
  } else {
    const c = limitarAlMapa(m.x - ag.x, m.y - ag.y);
    o.x = c.x; o.y = c.y;
  }

  if (arrastrando.tipo === "destino") {
    if (r.backend) r.red.destinoManual = { x: o.x, y: o.y, th: o.th };
    if (UI.robotSeleccionado(estado).id === r.id) {
      document.getElementById("pDestX").value = Math.round(o.x);
      document.getElementById("pDestY").value = Math.round(o.y);
      document.getElementById("pDestTh").value = Math.round(o.th * 180 / Math.PI);
    }
  }
}

function alSoltar() {
  arrastrando = null;
  superficieActiva().style.cursor = "crosshair";
}

// Pieza bajo el cursor. Alimenta el cursor de mano y el realce al dibujar.
let bajoCursor = null;

// Que pieza se realza: la que se esta arrastrando manda sobre la que esta
// bajo el cursor, porque durante el arrastre el raton puede salirse de
// ella y el realce no deberia apagarse a media maniobra.
function factorRealce(robot, tipo) {
  const h = arrastrando || bajoCursor;
  return h && h.robot === robot && h.tipo === tipo ? FACTOR_HOVER : 1;
}

function factorRealceObs(obs) {
  const h = arrastrando || bajoCursor;
  return h && h.tipo === "obstaculo" && h.obs === obs ? FACTOR_HOVER : 1;
}

function alPasarEncima(ev) {
  const activo = superficieActiva();
  if (arrastrando) { activo.style.cursor = "grabbing"; return; }
  bajoCursor = objetoBajo(aMundoCliente(ev.clientX, ev.clientY), { x: ev.clientX, y: ev.clientY });
  activo.style.cursor = bajoCursor ? "grab" : "crosshair";
}

// Las guias se agarran igual sobre el video: son los mismos objetos
// imaginarios, solo cambia el fondo sobre el que se dibujan.
lienzoVideo.addEventListener("mousedown", alPresionar);
lienzoVideo.addEventListener("mousemove", alPasarEncima);
lienzoVideo.addEventListener("touchstart", alPresionar, { passive: false });
lienzo.addEventListener("mousedown", alPresionar);
lienzo.addEventListener("mousemove", alPasarEncima);
function alSalirDelLienzo() {
  bajoCursor = null;
  if (!arrastrando) superficieActiva().style.cursor = "crosshair";
}
lienzo.addEventListener("mouseleave", alSalirDelLienzo);
lienzoVideo.addEventListener("mouseleave", alSalirDelLienzo);
window.addEventListener("mousemove", alMover);
window.addEventListener("mouseup", alSoltar);
lienzo.addEventListener("touchstart", alPresionar, { passive: false });
window.addEventListener("touchmove", alMover, { passive: false });
window.addEventListener("touchend", alSoltar);

/* ============================================================
   PANEL DE CONTROL: seleccion de robot, modelo, parametros
   ============================================================ */
function seleccionarRobot(id) {
  // Antes de mover la seleccion: lo que este pisado es del robot que se
  // deja atras, y soltarlo despues le mandaria el cero al equivocado.
  soltarFlechas();
  estado.seleccionado = id;
  UI.construirSelectorRobot(estado, seleccionarRobot);
  const robot = robotPorId(id);
  UI.refrescarFormulario(robot, estado.formacion);
  refrescarRepulsorPanel();
  if (robot.modelo === "manual") prepararPanelManual(robot);
  Graficas.construir(robot);
  refrescarBloqueTag();
}

/* ---- Vinculo ArUco -> robot ---- */
const bloqueTag = document.getElementById("bloqueTag"),
      iTag = document.getElementById("iTag"),
      tagEstado = document.getElementById("tagEstado");

function refrescarBloqueTag() {
  const r = UI.robotSeleccionado(estado);
  // Se apaga cuando no hay lazo con la camara -ni Real ni armado- en vez de
  // esconderse: que el campo exista y se vea gris explica solo para que
  // sirve, en vez de aparecer de la nada.
  const vivo = esReal() || !!r.armado;
  bloqueTag.classList.toggle("off", !vivo);
  // El campo NO se pisa mientras se escribe en el. En Real esta funcion
  // corre en cada cuadro del bucle, asi que asignar el valor sin mirar
  // borraba lo tecleado 60 veces por segundo: el campo se veia habilitado
  // pero era imposible cambiarlo.
  if (document.activeElement !== iTag) iTag.value = r.tagAruco;
  if (!vivo) { tagEstado.className = "tag-punto"; tagEstado.title = "en Real, o con MQTT armado"; return; }
  const visto = r.backend && r.red.tagVisto;
  tagEstado.className = "tag-punto " + (visto ? "visto" : "perdido");
  // El punto no dice solo si se ve el marcador: en los modos que persiguen
  // algo, no verlo es la razon por la que el robot no se mueve.
  tagEstado.title = visto ? "ArUco " + r.tagAruco + " detectado"
    : "ArUco " + r.tagAruco + " sin detectar" + (r.modelo === "manual" ? "" : " - el robot no se movera");
}

// Mientras se teclea solo se toma el valor si ya es un id valido, sin
// reescribir la caja: recortar a cada tecla pelearia con quien esta
// escribiendo (borrar el 1 de "12" para dejar "2" daria un 0 de paso).
iTag.oninput = () => {
  const v = parseInt(iTag.value, 10);
  if (Number.isInteger(v) && v >= 0 && v <= 49) UI.robotSeleccionado(estado).tagAruco = v;
};

// Al salir del campo (o con Enter) se normaliza y se ve el valor que quedo.
iTag.onchange = () => {
  const r = UI.robotSeleccionado(estado);
  let v = Math.round(+iTag.value || 0);
  v = Math.max(0, Math.min(49, v));   // DICT_6X6_50: ids 0..49
  iTag.value = v;
  r.tagAruco = v;
  refrescarBloqueTag();
};

/* ---- Modo manual: PWM crudo por rueda, durante N segundos ---- */
const mSeg = document.getElementById("mSeg"), mCuenta = document.getElementById("mCuenta"),
      mEnviar = document.getElementById("mEnviar"), mReal = document.getElementById("mReal");

// Publica la trama al robot fisico. Se hace por el servidor y no desde el
// navegador porque la pagina se sirve por http:// y el broker solo acepta
// wss://; ademas el servidor ya sabe a que robot corresponde cada id.
async function mandarAlRobotReal(robot, pwm) {
  // La pagina numera las ruedas del uniciclo [derecha, izquierda]: es el
  // orden que produce diferencial() y el que rotulan las graficas. La
  // flota las numera al reves -- protocol.py abre diciendo "2 ruedas:
  // izquierda, derecha" y el firmware lee "<izq>,<der>".
  //
  // Cruzarlas aqui y no adentro: el orden de la pagina lo usan la
  // simulacion y la telemetria, y voltearlo alla movia mucho mas de lo
  // que arregla. Con [40,40] no se notaba porque los dos valores son
  // iguales; en cuanto se pida un giro, sale espejeado.
  const trama = robot.tipo === "holonomico" ? pwm : [pwm[1], pwm[0]];
  try {
    const r = await fetch("/manual", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ robot: robot.id, pwm: trama }),
    });
    const d = await r.json();
    if (!d.ok) console.warn("[mqtt] " + (d.error || "falló"));
  } catch (e) {
    console.warn("[mqtt] sin conexión");
  }
}

function pararManual(robot) {
  const m = robot.parametros.manual;
  clearTimeout(m.corte);
  clearInterval(m.latido);
  m.corte = null;
  m.latido = null;
  m.activo = false; m.restante = 0; m.finMs = 0;
  // El cero se manda SIEMPRE que se haya estado publicando al robot real,
  // incluso si el usuario ya desmarco la casilla: si no, el robot se
  // quedaria con la ultima trama y solo lo frenaria el watchdog de 2.5 s
  // del firmware.
  if (m.real) mandarAlRobotReal(robot, robot.parametros.manual.pwm.map(() => 0));
  m.real = false;
  refrescarManual(robot);
}

function arrancarManual(robot) {
  const m = robot.parametros.manual;
  m.activo = true;
  m.real = !!robot.armado;
  clearTimeout(m.corte);
  clearInterval(m.latido);
  m.corte = null;
  m.latido = null;

  if (m.modo === "ruedas") {
    // Con cajas se manda DURANTE un tiempo: no hay nada que sostener, asi
    // que el corte por reloj es lo unico que puede parar al robot.
    //
    // Va con un temporizador de verdad, NO con el lazo de dibujo.
    // requestAnimationFrame se detiene cuando la pestaña deja de verse, y
    // con el se detenia la cuenta regresiva: si te cambias de pestaña a
    // media prueba, el agente sigue repitiendo la ultima trama cada
    // segundo y nadie manda nunca el cero. Visto en vivo: Odiseo con una
    // rueda girando 50 segundos con "2.0 s" congelado en pantalla.
    //
    // setTimeout si dispara con la pestaña oculta (throttled a ~1 Hz, que
    // para esto sobra). finMs ademas deja calcular lo que queda por reloj,
    // asi la cuenta se ve bien aunque el lazo se haya perdido frames.
    m.segundos = Math.max(0.5, Math.min(60, +mSeg.value || 2));
    m.restante = m.segundos;
    m.finMs = Date.now() + m.segundos * 1000;
    m.corte = setTimeout(() => pararManual(robot), m.segundos * 1000);
  } else {
    // Conduciendo no hay corte: dura lo que dure la pulsacion, y quien
    // para es soltar. A cambio la trama se repite cada segundo mientras se
    // sostiene, para no depender de que el agente la reenvie solo: el
    // watchdog del firmware son 2.5 s de silencio, y con un agente que no
    // repita el robot se frenaria a media maniobra.
    m.finMs = 0;
    m.restante = 0;
    if (m.real) m.latido = setInterval(() => mandarAlRobotReal(robot, m.pwm), 1000);
  }

  if (m.real) mandarAlRobotReal(robot, m.pwm);
  refrescarManual(robot);
}

function refrescarManual(robot) {
  const m = robot.parametros.manual;
  mCuenta.textContent = m.activo && m.finMs ? Math.max(0, m.restante).toFixed(1) + " s" : "—";
  mCuenta.classList.toggle("activo", m.activo);
  mEnviar.disabled = m.activo;
  if (m.modo === "flechas") UI.refrescarFlechas(robot, CFG);
  else UI.resaltarRuedasManual(robot);
}

// Deja el panel de Manual a nombre de ESTE robot: sus ruedas (cuatro o
// dos), su forma de manejar y su potencia. Se llama al entrar al modo y al
// cambiar de robot, que antes no lo rehacia y dejaba a la vista las cajas
// del robot anterior.
function prepararPanelManual(robot) {
  UI.construirRuedasManual(robot, CFG);
  UI.mostrarModoManual(robot, CFG);
  mSeg.value = robot.parametros.manual.segundos;
  refrescarManual(robot);
}

/* ---- Conduccion con flechas ----------------------------------------
   La cruceta y el teclado no son un camino aparte: escriben la misma
   trama en m.pwm y pasan por arrancarManual()/pararManual(). Asi el corte
   por tiempo, el cero al soltar, el paro de emergencia y la pausa valen
   igual para las dos formas de manejar, sin duplicar ninguna de esas
   salvaguardas.

   Se admiten dos flechas a la vez (adelante + giro sale un arco), porque
   la mezcla ya sabe combinarlas: es una suma, no un caso especial. */
// Que hace cada tecla depende del chasis, igual que la cruceta: en el
// mecanum los costados son desplazamiento lateral y el giro se va a Q/E,
// que es donde se busca; en los uniciclo, que no se mueven de canto, los
// costados YA son el giro y Q/E queda de sinonimo.
//
// Las diagonales y las curvas no tienen tecla propia: son las dos de la
// esquina a la vez, que es exactamente lo que hacen.
function dirDeTecla(robot, tecla) {
  const holo = robot.tipo === "holonomico";
  switch (tecla) {
    case "ArrowUp": return "adelante";
    case "ArrowDown": return "atras";
    case "ArrowLeft": return holo ? "lateral-izq" : "giro-izq";
    case "ArrowRight": return holo ? "lateral-der" : "giro-der";
    case "q": case "Q": return "giro-izq";
    case "e": case "E": return "giro-der";
    default: return null;
  }
}

const dirsActivas = new Set();
let dirRaton = null;      // las que sostiene el boton del raton, si alguna

// Solo se conduce el robot seleccionado, y solo si esta en Manual con la
// cruceta a la vista. Devuelve null en cualquier otro caso.
function robotConducible() {
  const r = UI.robotSeleccionado(estado);
  return r && r.modelo === "manual" && r.parametros.manual.modo === "flechas" ? r : null;
}

function tramaFlechas(robot) {
  const m = robot.parametros.manual;
  const pisada = d => dirsActivas.has(d) ? 1 : 0;
  const adelante = pisada("adelante") - pisada("atras");
  // Girar a la izquierda es giro POSITIVO: th crece, que en pantalla es
  // antihorario. Y una esquina pisada da los dos a la vez, sin mas.
  const giro = pisada("giro-izq") - pisada("giro-der");
  if (robot.tipo !== "holonomico") return tramaDiferencial(adelante, giro, m.potencia);
  // Solo el mecanum tiene tercer grado de libertad: moverse de canto sin
  // apuntar hacia alla. De ahi salen las diagonales.
  const derecha = pisada("lateral-der") - pisada("lateral-izq");
  return tramaMecanum(adelante, derecha, giro, m.potencia);
}

// Parar y dejar la lectura en cero. El cero es lo que se acaba de mandar,
// asi que dejar a la vista la ultima trama viva haria creer que el robot
// sigue con ella.
function pararFlechas(robot) {
  const m = robot.parametros.manual;
  m.pwm.forEach((_, i) => { m.pwm[i] = 0; });
  pararManual(robot);
}

function conducir() {
  const robot = robotConducible();
  if (!robot) return;
  UI.resaltarFlechas(dirsActivas);
  if (!dirsActivas.size) { pararFlechas(robot); return; }
  // En sitio, sin cambiar el arreglo: las cajas del otro modo apuntan a
  // el, y sustituirlo las dejaria escribiendo en uno huerfano.
  tramaFlechas(robot).forEach((v, i) => { robot.parametros.manual.pwm[i] = v; });
  // Rearranca la cuenta en cada cambio de direccion: cada pulsacion nueva
  // se gana su propio tiempo de corte.
  arrancarManual(robot);
}

// Soltar TODO. Se llama cuando algo interrumpe de fuera (cambio de robot,
// de modo, pausa, paro, la ventana pierde el foco): si no, el navegador se
// queda creyendo que la tecla sigue pisada y no llega nunca el soltar.
function soltarFlechas() {
  if (!dirsActivas.size && !dirRaton) return;
  const robot = robotConducible();
  dirsActivas.clear();
  dirRaton = null;
  UI.resaltarFlechas(dirsActivas);
  if (robot && robot.parametros.manual.activo) pararFlechas(robot);
}

window.addEventListener("keydown", ev => {
  const robot = robotConducible();
  if (!robot || ev.repeat) return;
  const dir = dirDeTecla(robot, ev.key);
  if (!dir) return;
  // Escribir en una caja tiene prioridad: ahi las flechas son del campo.
  if (ev.target instanceof Element && ev.target.closest("input, select, textarea")) return;
  ev.preventDefault();   // si no, la barra lateral se desplaza sola
  dirsActivas.add(dir);
  conducir();
});

window.addEventListener("keyup", ev => {
  const robot = UI.robotSeleccionado(estado);
  const dir = robot && dirDeTecla(robot, ev.key);
  if (!dir || !dirsActivas.delete(dir)) return;
  conducir();
});

// Con la ventana sin foco el keyup no llega nunca: sin esto, alt-tab con
// una flecha pisada deja al robot andando, y desde que conducir no caduca
// ya no hay temporizador detras que lo corte. Es la salvaguarda principal.
window.addEventListener("blur", soltarFlechas);

// Y si la pagina se va del todo (cerrar la pestaña, recargar, navegar) con
// algo andando, tampoco queda nadie que mande el cero: el agente repite la
// ultima trama cada segundo, asi que el watchdog del firmware no llega a
// saltar nunca. sendBeacon sale aunque el documento ya se este descargando,
// que es justo lo que un fetch normal no garantiza.
//
// Vale para las dos formas de manejar: un Enviar de 60 s y cerrar la
// pestaña acababa igual.
window.addEventListener("pagehide", () => {
  estado.robots.forEach(robot => {
    const m = robot.parametros.manual;
    if (!m.activo || !m.real) return;
    const cuerpo = JSON.stringify({ robot: robot.id, pwm: m.pwm.map(() => 0) });
    navigator.sendBeacon("/manual", new Blob([cuerpo], { type: "application/json" }));
  });
});

// Delegado, no boton por boton: la cruceta se rehace entera al cambiar de
// robot, y unos escuchas colgados de los botones viejos se irian con ellos.
document.getElementById("manualFlechas").addEventListener("pointerdown", ev => {
  const boton = ev.target instanceof Element ? ev.target.closest(".dp") : null;
  if (!boton || !robotConducible()) return;
  ev.preventDefault();      // sin esto arrastrar desde el boton selecciona texto
  // Una esquina lleva DOS direcciones ("adelante lateral-izq"): es la misma
  // combinacion que sale de pisar sus dos teclas.
  dirRaton = boton.dataset.dir.split(" ");
  dirRaton.forEach(d => dirsActivas.add(d));
  conducir();
});

// El soltar se escucha en la ventana, no en el boton: si el cursor sale
// del boton antes de soltar, el pointerup cae en otro sitio y la rueda se
// quedaria girando.
window.addEventListener("pointerup", () => {
  if (!dirRaton) return;
  dirRaton.forEach(d => dirsActivas.delete(d));
  dirRaton = null;
  conducir();
});
window.addEventListener("pointercancel", soltarFlechas);

function fijarModoManual(robot, modo) {
  if (robot.parametros.manual.modo === modo) return;
  // Cambiar de forma de manejar con el robot andando lo dejaria acelerado
  // y sin nadie que le mande el cero.
  soltarFlechas();
  if (robot.parametros.manual.activo) pararManual(robot);
  robot.parametros.manual.modo = modo;
  prepararPanelManual(robot);
}

const mModo = document.getElementById("mModo"), sPot = document.getElementById("sPot");
mModo.onclick = () => {
  const r = UI.robotSeleccionado(estado);
  fijarModoManual(r, r.parametros.manual.modo === "flechas" ? "ruedas" : "flechas");
};
document.getElementById("mmRuedas").onclick = () => fijarModoManual(UI.robotSeleccionado(estado), "ruedas");
document.getElementById("mmFlechas").onclick = () => fijarModoManual(UI.robotSeleccionado(estado), "flechas");

sPot.oninput = () => {
  const r = UI.robotSeleccionado(estado);
  r.parametros.manual.potencia = Math.round(+sPot.value);
  UI.refrescarPotencia(r);
  // Con una flecha pisada el cambio se nota YA; si no, habria que soltar y
  // volver a pisar para que la potencia nueva saliera.
  if (dirsActivas.size) conducir();
};

// Si la pestaña deja de verse con un manual corriendo, se corta. Manejar
// en lazo abierto un robot que no estas viendo no es manejarlo, y ademas
// es justo cuando el navegador empieza a frenar todo lo demas.
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) return;
  soltarFlechas();
  estado.robots.forEach(r => {
    if (r.parametros.manual.activo) pararManual(r);
  });
});

mEnviar.onclick = () => arrancarManual(UI.robotSeleccionado(estado));
mSeg.onchange = () => { UI.robotSeleccionado(estado).parametros.manual.segundos = +mSeg.value; };

// Una casilla para los tres: arma o desarma la flota entera. Desarmar en
// caliente para a los robots YA; esperar a que acabe la maniobra en curso
// seria justo lo contrario de lo que pide quien quita la casilla.
mReal.onchange = () => {
  const armar = mReal.checked;
  estado.robots.forEach(r => {
    r.armado = armar;
    if (!armar && r.parametros.manual.activo) pararManual(r);
  });
  UI.refrescarArmado(estado);
  refrescarParo();
  refrescarBloqueTag();
};

function fijarModelo(modelo) {
  const robot = UI.robotSeleccionado(estado);
  if (robot.modelo === modelo) return;
  // Cambiar de modo con el manual corriendo dejaria el robot fisico
  // acelerado y sin nadie que le mande el cero.
  // Primero soltar: si venia conduciendo, eso ya para y deja la trama
  // en cero. Lo de abajo cubre el otro modo, el de las cajas.
  soltarFlechas();
  if (robot.modelo === "manual" && robot.parametros.manual.activo) pararManual(robot);
  robot.modelo = modelo;
  if (modelo === "manual") prepararPanelManual(robot);
  refrescarBloqueTag();
  robot.parametros.trayectoria.fase = 0;
  // El centro se ancla YA en la pose actual, no se deja en null: si no,
  // estando en pausa la curva de referencia no tendria donde dibujarse
  // (o peor, seguiria al robot al arrastrarlo).
  robot.parametros.trayectoria.centroInicial = [robot.pose.x, robot.pose.y];
  UI.refrescarFormulario(robot, estado.formacion);
  refrescarRepulsorPanel();
}

document.querySelectorAll("#modoSelect button").forEach(b => {
  b.onclick = () => fijarModelo(b.dataset.modelo);
});

const pDestX = document.getElementById("pDestX"), pDestY = document.getElementById("pDestY"),
      pDestTh = document.getElementById("pDestTh");
function fijarDestinoManual(r) {
  const c = limitarAlMapa(+pDestX.value, +pDestY.value);
  r.destino.x = c.x; r.destino.y = c.y;
  // El rumbo se envuelve a (-180, 180]: escribir 270 es lo mismo que -90,
  // y dejarlo crudo haria que el error angular tomara la vuelta larga.
  const grados = Math.round(envolver((+pDestTh.value || 0) * Math.PI / 180) * 180 / Math.PI);
  r.destino.th = grados * Math.PI / 180;
  // se reescriben los campos: si un valor se recorto o envolvio, que se vea
  pDestX.value = Math.round(c.x); pDestY.value = Math.round(c.y); pDestTh.value = grados;
  if (r.backend) r.red.destinoManual = { x: r.destino.x, y: r.destino.y, th: r.destino.th };
}
pDestX.onchange = () => fijarDestinoManual(UI.robotSeleccionado(estado));
pDestY.onchange = () => fijarDestinoManual(UI.robotSeleccionado(estado));
pDestTh.onchange = () => fijarDestinoManual(UI.robotSeleccionado(estado));

const sk = document.getElementById("sk"), se = document.getElementById("se");
sk.oninput = () => { const r = UI.robotSeleccionado(estado); r.parametros.punto.k = +sk.value; document.getElementById("vk").textContent = (+sk.value).toFixed(2); };
se.oninput = () => { const r = UI.robotSeleccionado(estado); r.parametros.punto.escala = +se.value / 100; document.getElementById("ve").textContent = se.value + "%"; };

/* ---- Zonas de parada (panel global) --------------------------------
   Los unicos dos controles de la pagina que NO son del robot seleccionado:
   se escriben en los tres de una vez. Siguen viviendo en parametros.punto
   de cada robot y no en una global aparte, porque es ahi donde los leen
   avanzarUniciclo(), el anillo del mapa y el cuerpo de /ciclo; hacerlo
   global obligaba a tocar los tres sitios para no ganar nada. */
const sTolPos = document.getElementById("sTolPos"), sTolAng = document.getElementById("sTolAng");

function fijarZona(campo, valor) {
  estado.robots.forEach(r => { r.parametros.punto[campo] = valor; });
}
sTolPos.oninput = () => {
  fijarZona("tolPos", +sTolPos.value);
  document.getElementById("vTolPos").textContent = sTolPos.value + " px";
  resumenZonas();
};
sTolAng.oninput = () => {
  fijarZona("tolAng", +sTolAng.value);
  document.getElementById("vTolAng").textContent = sTolAng.value + "°";
  resumenZonas();
};
// El HTML arranca con 30 px y 5°, que es tambien lo que trae crearRobot().
// Se vuelca igual al cargar, para que un value distinto en el markup no
// deje la barra diciendo una cosa y los robots haciendo otra.
// Plegado, el desplegable dice en que quedaron sin abrirlo.
function resumenZonas() {
  document.getElementById("tolResumen").textContent = sTolPos.value + " px · " + sTolAng.value + "°";
}
sTolPos.oninput(); sTolAng.oninput();

/* ---- panel de Obstaculos ---- */
function refrescarPanelObstaculos() {
  UI.construirSelectorObstaculos(estado, seleccionarObstaculo, crearObstaculoDesdePanel);
  UI.construirSelectorTipoRepulsor("obsTipoSelect", obsSeleccionado(), fijarTipoRepulsor);
  UI.refrescarFormularioObstaculo(estado);
}

function seleccionarObstaculo(id) {
  estado.seleccionadoObs = id;
  refrescarPanelObstaculos();
}

function crearObstaculoDesdePanel() {
  const o = anadirObstaculo();
  if (!o) return;
  sincronizarObstaculos(0);
  seleccionarObstaculo(o.id);
}

function borrarObstaculoSeleccionado() {
  const i = estado.obstaculos.findIndex(o => o.id === estado.seleccionadoObs);
  if (i < 0) return;
  estado.obstaculos.splice(i, 1);
  // Si se estaba arrastrando justo ese, soltarlo: seguir escribiendo sobre
  // un objeto que ya no esta en la lista mueve algo que nadie ve.
  if (arrastrando && arrastrando.tipo === "obstaculo" &&
      !obstaculoPorId(arrastrando.obs.id)) arrastrando = null;
  // Queda seleccionado el que ocupa su sitio, o el ultimo si borraste el
  // ultimo. Dejarlo sin seleccion obligaria a volver a picar una ficha
  // para seguir trabajando.
  const sig = estado.obstaculos[Math.min(i, estado.obstaculos.length - 1)];
  seleccionarObstaculo(sig ? sig.id : null);
}

const oX = document.getElementById("oX"), oY = document.getElementById("oY");
function fijarCentroObstaculo() {
  const o = obsSeleccionado();
  if (!o) return;
  const c = limitarAlMapa(+oX.value || 0, +oY.value || 0);
  o.ancla.x = c.x; o.ancla.y = c.y;
  sincronizarObstaculos(0);
  UI.refrescarCentroObstaculo(o);   // si el valor se recorto, que se vea
}
oX.onchange = fijarCentroObstaculo;
oY.onchange = fijarCentroObstaculo;

// Los seis rieles son el mismo cableado seis veces: escribir el campo del
// obstaculo seleccionado y su cifra al lado. Lo unico que cambia es como
// se rotula el numero.
function cablearSliderObs(idRiel, idCifra, campo, rotulo) {
  const riel = document.getElementById(idRiel);
  riel.oninput = () => {
    const o = obsSeleccionado();
    if (!o) return;
    o[campo] = +riel.value;
    document.getElementById(idCifra).textContent = rotulo(+riel.value);
    // Los que cambian la GEOMETRIA recolocan el circulo en el acto: en
    // pausa el lazo no va a volver a pasar por sincronizarObstaculos().
    sincronizarObstaculos(0);
  };
}
cablearSliderObs("sORadio", "vORadio", "radio", v => v + " px");
cablearSliderObs("sOEta", "vOEta", "eta", v => String(v));
cablearSliderObs("sOLargo", "vOLargo", "largo", v => v + " px");
cablearSliderObs("sOAng", "vOAng", "angulo", v => v + "\u00B0");
cablearSliderObs("sOOrbita", "vOOrbita", "orbita", v => v + " px");
cablearSliderObs("sOVel", "vOVel", "vel", v => v.toFixed(2));

function fijarTipoRepulsor(tipo) {
  const o = obsSeleccionado();
  if (!o) return;
  o.tipo = tipo;
  // La ficha del obstaculo lleva el glifo del tipo, asi que hay que
  // repintar tambien el selector de arriba y no solo estos controles.
  refrescarPanelObstaculos();
}

document.querySelectorAll("#obsSentido button").forEach(b => {
  b.onclick = () => {
    const o = obsSeleccionado();
    if (!o) return;
    o.sentido = +b.dataset.sentido;
    refrescarPanelObstaculos();
  };
});

document.querySelectorAll("#obsMovSelect button").forEach(b => {
  b.onclick = () => {
    const o = obsSeleccionado();
    if (!o) return;
    o.mov = b.dataset.mov;
    // La fase se reinicia al cambiar de camino: heredada de la anterior,
    // el obstaculo aparecia a media orbita de donde estaba, como si el
    // boton lo hubiera teletransportado.
    o.fase = 0;
    sincronizarObstaculos(0);
    UI.refrescarFormularioObstaculo(estado);
  };
});

document.getElementById("oBorrar").onclick = borrarObstaculoSeleccionado;

/* ---- el campo repulsivo del robot ----
   Es por robot y vive en el panel de Robots, no en el de Obstaculos: no es
   una pieza puesta en la cancha, es una propiedad de quien tengas elegido.
   Se refresca con el resto del formulario al cambiar de ficha. */
function refrescarRepulsorPanel() {
  UI.refrescarRepulsorRobot(UI.robotSeleccionado(estado), fijarTipoRepulsorRobot);
}

function fijarTipoRepulsorRobot(tipo) {
  UI.robotSeleccionado(estado).repulsor.tipo = tipo;
  refrescarRepulsorPanel();
}

function cablearSliderRepulsor(idRiel, idCifra, campo, rotulo) {
  const riel = document.getElementById(idRiel);
  riel.oninput = () => {
    const rp = UI.robotSeleccionado(estado).repulsor;
    rp[campo] = +riel.value;
    document.getElementById(idCifra).textContent = rotulo(+riel.value);
    // El bloque entero baja de tono cuando la intensidad cae a cero, asi
    // que hay que repasarlo en cuanto se mueve el riel.
    refrescarRepulsorPanel();
  };
}
cablearSliderRepulsor("sRRadio", "vRRadio", "radio", v => v + " px");
cablearSliderRepulsor("sREta", "vREta", "eta", v => String(v));

// Interruptor de la cabecera. Encenderlo abre la tarjeta y pone una
// intensidad que ya se nota (ETA_REPULSOR_ON): arrancar en 0 obligaba a
// saber que habia que subir el riel. Apagarlo es intensidad 0, y pliega.
// Es un <button> y no un checkbox: dentro de un <summary> el clic abriria o
// cerraria la tarjeta, y aqui eso lo decide el interruptor.
document.getElementById("repOn").onclick = ev => {
  ev.preventDefault();
  ev.stopPropagation();
  const rp = UI.robotSeleccionado(estado).repulsor;
  const bloque = document.getElementById("bloqueRepulsor");
  if (rp.eta > 0) { rp.eta = 0; bloque.open = false; }
  else { rp.eta = ETA_REPULSOR_ON; bloque.open = true; }
  refrescarRepulsorPanel();
};

document.querySelectorAll("#repSentido button").forEach(b => {
  b.onclick = () => {
    UI.robotSeleccionado(estado).repulsor.sentido = +b.dataset.sentido;
    refrescarRepulsorPanel();
  };
});

const tCentroX = document.getElementById("tCentroX"), tCentroY = document.getElementById("tCentroY");
const tTipo = document.getElementById("tTipo"), tRadio = document.getElementById("tRadio"), sOmega = document.getElementById("sOmega");
// Reinicia solo la FASE: el centro, si el usuario ya lo definio a mano, se
// respeta. Con la fase acumulada, ni el radio ni omega necesitan reiniciar
// nada; solo cambiar de tipo de curva, donde la fase deja de significar lo
// mismo.
function reiniciarFaseTrayectoria() { UI.robotSeleccionado(estado).parametros.trayectoria.fase = 0; }
function fijarCentroTrayectoria() {
  const c = limitarAlMapa(+tCentroX.value, +tCentroY.value);
  UI.robotSeleccionado(estado).parametros.trayectoria.centroInicial = [c.x, c.y];
  tCentroX.value = Math.round(c.x); tCentroY.value = Math.round(c.y);
}
tCentroX.onchange = fijarCentroTrayectoria;
tCentroY.onchange = fijarCentroTrayectoria;
tTipo.onchange = () => { UI.robotSeleccionado(estado).parametros.trayectoria.tipo = tTipo.value; reiniciarFaseTrayectoria(); };
// El radio no reinicia la fase: la curva solo cambia de tamaño y el punto
// de referencia sigue en el mismo sitio de la vuelta.
tRadio.oninput = () => {
  UI.robotSeleccionado(estado).parametros.trayectoria.radio = +tRadio.value;
  document.getElementById("vRadio").textContent = tRadio.value;
};
sOmega.oninput = () => { UI.robotSeleccionado(estado).parametros.trayectoria.omega = +sOmega.value; document.getElementById("vOmega").textContent = (+sOmega.value).toFixed(2); };

const sFa = document.getElementById("sFa"), sFk = document.getElementById("sFk");
sFa.oninput = () => {
  estado.formacion.a = +sFa.value;
  document.getElementById("vFa").textContent = sFa.value;
  // La rejilla se dibuja a escala, asi que cambiar la separacion tiene que
  // repintarla: si no, el editor enseñaria un tamaño y la cancha otro.
  Formacion.pintar();
};
sFk.oninput = () => { estado.formacion.k = +sFk.value; document.getElementById("vFk").textContent = (+sFk.value).toFixed(2); };


/* ============================================================
   TOPBAR: robots habilitados, correr/pausar/reiniciar
   ============================================================ */

// Corta el robot fisico YA. Dejar de mandar comandos no es parar: el
// firmware frena solo, pero a los 2.5 s de silencio, y un robot que sigue
// andando 2.5 segundos despues de que le picaste stop no cuenta como
// detenido. Se manda dos veces por si se pierde un paquete: son ceros,
// repetirlos no hace daño.
const bparo = document.getElementById("bparo");

function pararRobotReal() {
  // TODOS los que tienen hardware, no solo Atlas. Desde que Odiseo entro a
  // Real, un paro que corta a uno y deja andando al otro es peor que no
  // tener boton: quien lo pica se queda tranquilo viendo moverse un robot.
  //
  // Los ceros van crudos, sin pasar por mandarAlRobotReal(): un [0,0] es
  // igual del derecho que del reves, y en un paro no se agrega nada de por
  // medio que se pueda equivocar.
  estado.robots.filter(r => r.hardware).forEach(robot => {
    const ceros = robot.pwm.map(() => 0);
    for (let i = 0; i < 2; i++) {
      fetch("/manual", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ robot: robot.id, pwm: ceros }),
      }).catch(() => {});
    }
    robot.red.trama = ceros;
    // Y se corta el manual si estaba corriendo. Se llama a pararManual en
    // vez de bajar las banderas a mano porque ademas refresca el panel: el
    // paro deja el lazo en pausa, y sin ese refresco la tarjeta se queda
    // enseñando una cuenta regresiva viva y el boton Enviar deshabilitado.
    if (robot.parametros.manual.activo) pararManual(robot);
  });
}

function paroDeEmergencia() {
  estado.corriendo = false;
  soltarFlechas();
  // Desarmar es parte del paro: si no, basta con darle a play para que la
  // flota arranque otra vez sola, y quien acaba de picar un paro no quiere
  // eso a un clic de distancia.
  estado.robots.forEach(r => { r.armado = false; });
  UI.refrescarArmado(estado);
  refrescarParo();
  UI.actualizarBotonCorrer(estado);
  pararRobotReal();
  // El destino se planta en la pose actual: si no, al volver a darle play
  // el robot arranca de golpe hacia el destino viejo, que es lo ultimo que
  // quiere quien acaba de picar un paro.
  const atlas = robotPorId("atlas");
  atlas.destino = { x: atlas.pose.x, y: atlas.pose.y, th: atlas.pose.th };
  atlas.red.destinoManual = null;
  bparo.classList.remove("disparado");
  void bparo.offsetWidth;            // reinicia la animacion si se pica seguido
  bparo.classList.add("disparado");
}

bparo.onclick = paroDeEmergencia;
// Esc y la barra, para cuando ya no da tiempo de apuntarle al boton con el
// raton. La barra es la tecla mas grande del teclado y se encuentra a
// ciegas, que es exactamente lo que hace falta con un robot suelto.
window.addEventListener("keydown", ev => {
  if (!esReal() && !hayArmado()) return;
  if (ev.key !== "Escape" && ev.key !== " " && ev.key !== "Spacebar") return;
  // Escribiendo en una caja, la barra es un espacio. Y con el foco en un
  // boton, ademas lo activaria: por eso el preventDefault.
  if (ev.target instanceof Element && ev.target.closest("input, select, textarea")) return;
  ev.preventDefault();
  paroDeEmergencia();
});

document.getElementById("bcorrer").onclick = () => {
  estado.corriendo = !estado.corriendo;
  // Pausar tambien tiene que frenar el robot, no solo dejar de hablarle.
  // Y el manual se corta pase lo que pase, no solo en modo Real: la
  // casilla de MQTT manda al robot fisico tambien desde Simulacion, asi
  // que un pause que ahi no hiciera nada dejaria las ruedas girando.
  if (!estado.corriendo) {
    soltarFlechas();
    estado.robots.forEach(r => { if (r.parametros.manual.activo) pararManual(r); });
    if (esReal()) pararRobotReal();
  }
  UI.actualizarBotonCorrer(estado);
};
document.getElementById("breiniciar").onclick = () => {
  if (esReal()) pararRobotReal();
  estado.robots.forEach(r => {
    r.pose = { ...r.spawn };
    r.destino = { x: -r.spawn.x, y: -r.spawn.y, th: 0 };
    r.rastro = [];
    r.parametros.trayectoria.fase = 0;
    // re-anclado en la pose nueva (no a null): la curva de referencia
    // tiene que seguir dibujandose aunque la simulacion quede en pausa
    r.parametros.trayectoria.centroInicial = [r.pose.x, r.pose.y];
    if (r.backend) r.red.destinoManual = null;
  });
  estado.tiempo = 0;
  UI.refrescarFormulario(UI.robotSeleccionado(estado), estado.formacion);
  refrescarRepulsorPanel();
};

/* ============================================================
   ARRANQUE
   ============================================================ */
UI.construirSelectorRobot(estado, seleccionarRobot);
UI.refrescarFormulario(UI.robotSeleccionado(estado), estado.formacion);
refrescarRepulsorPanel();
UI.actualizarBotonCorrer(estado);
UI.iniciarLeyenda();

/* ============================================================
   BUCLE PRINCIPAL
   ============================================================ */
let ultimo = null;

/* ---- Volver a la pestaña ---------------------------------------------
   Irse y volver dejaba a Atlas mudo hasta recargar la pagina entera. No
   era una sola causa, eran tres, y todas se curan aqui de una vez en vez
   de esperar a que cada una se destrabe sola:

     1. El cerrojo de /ciclo. Ya se abre por reloj de pared, pero eso
        tarda 3 s desde el primer tick que llega, y el tick tarda lo suyo
        en volver a ser de 50 ms. Aqui se abre en el acto.
     2. El MJPEG. El <img> tiene la conexion muerta y el navegador NO la
        reabre solo: la camara nunca dejo de estar viva, asi que el
        false->true de aplicarEstadoCamara() no dispara y el video se
        queda en el ultimo cuadro para siempre.
     3. El reloj de dibujo. `ultimo` guarda un timestamp de hace un rato;
        el clamp a 0.05 ya evita el salto, pero arrancar limpio evita
        tambien el primer cuadro con dt de tope.

   Y va tambien en pageshow: navegar a otra pagina y volver con Atras no
   recarga nada, restaura el documento del bfcache tal como quedo -con el
   cerrojo cerrado y el video muerto-. */
function reanudar() {
  ultimo = null;
  cicloGen++;                       // lo que estuviera en vuelo ya no cuenta
  const atlas = robotPorId("atlas");
  atlas.red.enVuelo = false;
  // Si no, el primer cuadro sale "mudo" y se salta la integracion.
  atlas.red.ultimoContacto = performance.now();
  if (esReal()) refrescarFuenteVideo();
  cicloAtlas();
}

document.addEventListener("visibilitychange", () => { if (!document.hidden) reanudar(); });
window.addEventListener("pageshow", ev => { if (ev.persisted) reanudar(); });

function paso(t) {
  requestAnimationFrame(paso);
  if (ultimo === null) ultimo = t;
  let dt = (t - ultimo) / 1000; ultimo = t;
  dt = Math.min(dt, 0.05);

  if (estado.corriendo) estado.tiempo += dt;
  // Reloj de la simulacion: se congela en pausa. Lo usan por igual la fase
  // de las trayectorias y el eje de tiempo de las graficas.
  const tNow = estado.tiempo;

  const atlas = robotPorId("atlas");
  const mudo = performance.now() - atlas.red.ultimoContacto > TIMEOUT_MS;

  // Antes de calcular nada: los obstaculos tienen que estar ya en su sitio
  // de este cuadro, o la repulsion saldria de la posicion del anterior.
  sincronizarObstaculos(estado.corriendo ? dt : 0);

  if (estado.corriendo) {
    const formacionGlobal = estado.robots.some(r => r.modelo === "formacion") ? calcularFormacionGlobal() : null;

    estado.robots.forEach((r, idx) => {
      if (!estado.habilitados[r.id]) return;

      if (r.modelo === "manual") {
        const m = r.parametros.manual;
        // Solo para MOSTRAR lo que queda. Quien corta es el setTimeout de
        // arrancarManual: este lazo no corre con la pestaña oculta y no se
        // le puede confiar el apagado de un robot fisico. Sin finMs no hay
        // cuenta que llevar: se esta conduciendo, y eso dura lo que dure la
        // pulsacion.
        if (m.activo && m.finMs) m.restante = Math.max(0, (m.finMs - Date.now()) / 1000);
        r.fase = !m.activo ? "manual detenido"
               : m.finMs ? "manual " + Math.max(0, m.restante).toFixed(1) + " s"
               : "conduciendo";
        if (r.id === estado.seleccionado) refrescarManual(r);
        avanzarManual(r, dt, tNow);
        return;
      }

      if (r.backend) {
        // Atlas: el objetivo se traduce a destinoManual y el backend lo persigue solo.
        // Atlas no lleva lazo propio: el suyo vive en el servidor y solo
        // sabe perseguir UN PUNTO. Asi que todo -la curva, la formacion,
        // esquivar un obstaculo- se le traduce a "ve aqui".
        //
        // calcularObjetivo() devuelve ya la velocidad deseada con la
        // repulsion sumada, y de paso deja escrito r.repelido. Los tres
        // modelos pasan por el mismo sitio; lo unico que cambia es si hay
        // un punto fijo al que ir cuando nadie empuja.
        const [ux, uy] = calcularObjetivo(r, dt, formacionGlobal, idx);

        if (r.modelo === "formacion") {
          // La formacion no tiene punto fijo: su objetivo se recalcula cada
          // tick a partir de donde estan los tres, asi que siempre horizonte.
          r.red.destinoManual = puntoHorizonte(r, ux, uy, r.pose.th);
        } else if (!r.repelido) {
          // Nadie empujando: al punto de verdad -el destino, o el punto de
          // la curva que calcularObjetivo() acaba de escribir en r.destino-.
          // Byte por byte el lazo de siempre.
          r.red.destinoManual = { x: r.destino.x, y: r.destino.y, th: r.destino.th };
        } else {
          // Empujado: se le mueve el punto. El destino de verdad no se
          // toca, y se recupera solo en cuanto la repulsion vuelve a cero.
          //
          // ESTO FALTABA en trayectoria y en formacion. En trayectoria se
          // llamaba a calcularObjetivo() -o sea, se calculaba la repulsion-
          // y se tiraba el resultado a la basura con un "void ux, uy";
          // en formacion ni siquiera se llamaba. Atlas cruzaba los
          // obstaculos como si no estuvieran en los dos modos.
          r.red.destinoManual = puntoHorizonte(r, ux, uy, r.destino.th);
        }
        // No se integra cuando la camara esta poniendo la pose: en Real
        // nunca, y en el mapa solo mientras el ArUco se vea de verdad -si se
        // pierde, se sigue integrando en vez de dejar el sprite congelado.
        const conCamara = esReal() || (r.armado && r.red.tagVisto);
        if (!mudo) avanzarAtlas(r, dt, tNow, !conCamara); else agregarRastro(r, tNow);
      } else {
        const [ux, uy] = calcularObjetivo(r, dt, formacionGlobal, idx);
        avanzarUniciclo(r, ux, uy, dt, tNow);
      }
    });
  }

  // De atras hacia adelante: referencia (lo pedido, tenue) -> rastro (lo
  // logrado) -> robots -> y encima de los sprites todas las guias, que hay
  // que poder leer sin que las tape un chasis: el triangulo de destino, la
  // cruz del centro (ademas es un asa que se arrastra) y el triangulo de
  // formacion.
  // Se dibuja para TODOS los robots, no solo el seleccionado: cada uno va
  // en su color, asi se ve de un vistazo que esta haciendo cada quien.
  const sel = UI.robotSeleccionado(estado);
  // Conduciendo a mano se quitan las GUIAS: reticula, destino, referencia,
  // rastro, formacion. Todas son de los modelos que persiguen algo, y aqui
  // quien persigue eres tu. Queda el robot: el rombo sobre el ArUco -que
  // dice si la deteccion lo sigue viendo- y el nombre, para saber a cual le
  // estas hablando.
  const desnudo = sinGuias();

  if (en3D()) {
    // Mismo contenido que el 2D de abajo, en el mismo orden de ideas; el
    // orden de pintado ya lo resuelve la profundidad.
    escena3d.dibujar({
      robots: robotsEnEscena(),
      obstaculos: estado.obstaculos,
      formacion: robotsEnEscena().some(r => r.modelo === "formacion") ? verticesFormacion() : null,
      realce: factorRealce,
      realceObs: factorRealceObs,
    });
  } else {
    Escena.usarLienzo(lienzoActivo(), { sobreVideo: esReal() });
    Escena.limpiar();
    // Sobre el video no va sprite ninguno: el robot de verdad ya se ve ahi
    // debajo. Lo que se dibuja es el rombo negro sobre el ArUco, y si no cae
    // encima del marcador es que la deteccion esta mal.
    Escena.mostrarChasis(!esReal());
    if (desnudo) {
      // Realce fijo en 1: no hay nada que agarrar, asi que nada que realzar.
      robotsEnEscena().forEach(r => Escena.dibujarRobot(r, r.sprite || r.img, 1));
    } else {
      Escena.reticula();
      // Los obstaculos van DEBAJO de todo lo demas: son el terreno, no una
      // guia. Encima tienen que poder pasar los robots y sus rastros, o un
      // circulo de 300 px taparia media cancha.
      estado.obstaculos.forEach(o => Escena.dibujarObstaculo(o, factorRealceObs(o)));
      // El campo de cada robot va con los obstaculos, debajo de todo: es
      // terreno tambien, aunque viaje pegado a alguien.
      robotsEnEscena().forEach(r => Escena.dibujarRepulsorRobot(r));
      robotsEnEscena().forEach(r => Escena.dibujarReferencia(r));
      robotsEnEscena().forEach(r => Escena.dibujarRastro(r));
      robotsEnEscena().forEach(r => Escena.dibujarRobot(r, r.sprite || r.img, factorRealce(r, "pose")));
      robotsEnEscena().forEach(r => { if (r.modelo !== "formacion") Escena.dibujarDestino(r, factorRealce(r, "destino")); });
      robotsEnEscena().forEach(r => Escena.dibujarCentroTrayectoria(r, factorRealce(r, "centro")));
      if (robotsEnEscena().some(r => r.modelo === "formacion")) Escena.dibujarFormacion(verticesFormacion());
    }
  }

  UI.actualizarTimer(estado);

  estado.robots.forEach(r => {
    r.conectado = estado.habilitados[r.id] && (r.backend ? r.red.conectadoReal : true);
  });

  // Se muestrea a todos los robots aunque solo se dibuje el seleccionado:
  // al cambiar de robot su historial ya esta ahi en vez de arrancar vacio.
  if (estado.corriendo) {
    estado.robots.forEach(r => Graficas.muestrear(r, tNow));
    // El registro va en la MISMA rama que las graficas: en pausa el reloj de
    // simulacion no avanza, y seguir registrando dejaria un monton de filas
    // con el mismo t que despues hay que limpiar a mano.
    Registro.muestrear(estado.robots, tNow, estado.modo);
    refrescarBotonesRegistro();
  }
  Graficas.dibujar(sel, tNow);

  const camaraTiene = !!atlas.red.camara.robot;
  UI.actualizarTelemetria(sel, sel.backend ? atlas.red.info : { dist: 0, err_rumbo: 0, adelante: sel.velocidad.v || 0, izquierda: sel.velocidad.w || 0, giro: sel.velocidad.w || 0, llego: false, quieto: false },
    atlas.red.conectadoReal, mudo, CFG.camara_viva, camaraTiene, sel.backend ? CFG : null);

  document.getElementById("videoVacio").hidden = CFG.camara_viva;

  // El punto del ArUco se repinta siempre: armado desde el mapa es ahi donde
  // hay que poder ver si la camara lo esta viendo o no.
  refrescarBloqueTag();

  if (esReal()) {
    // La etiqueta del video dice lo unico que no se ve mirando la imagen:
    // si el marcador configurado esta siendo detectado, y si los comandos
    // estan saliendo al robot.
    const tag = atlas.tagAruco;
    const visto = atlas.red.tagVisto;
    const env = atlas.red.enviado;
    let txt = visto ? ("ArUco " + tag + " detectado") : ("buscando ArUco " + tag);
    if (visto) txt += env === true ? " · enviando al robot" : (env ? " · MQTT: " + env : " · en pausa");
    document.getElementById("tagVideo").textContent = txt;
  }
}

// El editor de la figura. Al soltar un punto no hace falta reiniciar nada:
// la ley lee construirZdFormacion() en cada tick y el triangulo objetivo del
// mapa se repinta solo en el cuadro siguiente.
Formacion.init(estado);
document.getElementById("fReiniciar").onclick = () => Formacion.reiniciar();

/* ---- registro de ensayo ----
   Grabar / Marca / CSV. El unico estado que vive aqui es el rotulo del
   boton; lo demas lo lleva Registro. */
const recGrabar = document.getElementById("recGrabar"),
      recMarca = document.getElementById("recMarca"),
      recCsv = document.getElementById("recCsv"),
      recNombre = document.getElementById("recNombre");

// Se llama tambien desde el lazo, porque Registro puede pararse SOLO al
// llegar al tope de filas y entonces el boton se quedaria diciendo "Parar".
// El guard evita reescribir el innerHTML sesenta veces por segundo.
let botonesRegistroEn = null;
function refrescarBotonesRegistro() {
  const on = Registro.estaGrabando();
  if (on === botonesRegistroEn) return;
  botonesRegistroEn = on;
  recGrabar.classList.toggle("grabando", on);
  recGrabar.innerHTML = '<i class="rec-punto"></i>' + (on ? "Parar" : "Grabar");
  recMarca.disabled = !on;
}

recGrabar.onclick = () => {
  if (Registro.estaGrabando()) Registro.parar(); else Registro.arrancar();
  refrescarBotonesRegistro();
};
recMarca.onclick = () => Registro.marcar();
recCsv.onclick = () => {
  // Si no hay ni una fila, el boton no tiene nada que hacer: se dice y ya,
  // en vez de bajar un archivo con solo la cabecera.
  if (!Registro.exportar(recNombre.value)) Registro.pintarEstado(true);
};

// M de marca. Sin modificador a proposito: durante un ensayo se pulsa a
// ciegas, con una mano en el robot. Se ignora si estas escribiendo el nombre
// del ensayo, que si no cada "m" del titulo dejaria una marca.
window.addEventListener("keydown", ev => {
  if (ev.key !== "m" && ev.key !== "M") return;
  if (ev.repeat || ev.ctrlKey || ev.metaKey || ev.altKey) return;
  if (ev.target instanceof Element && ev.target.closest("input, select, textarea")) return;
  Registro.marcar();
});

refrescarBotonesRegistro();
Registro.pintarEstado(true);

seleccionarRobot(estado.seleccionado);
refrescarRepulsorPanel();
// Sin obstaculos: el panel arranca enseñando solo el "+".
refrescarPanelObstaculos();
aplicarModo("sim");
precargarImagenes(estado.robots).then(() => {
  // Despues de las imagenes: los robots sin GLB salen en 3D con su sprite.
  if (estado.vista === "3d") fijarVista("3d");
  requestAnimationFrame(paso);
});
