// ui.js - todo el DOM: panel global, panel de control contextual, tarjetas
// inferiores, leyenda. No corre fisica ni habla con el servidor - solo lee
// y escribe sobre el "estado" que le pasa main.js.
"use strict";

const UI = (function () {
  const ICONO_PAUSA = '<svg viewBox="0 0 16 16" fill="currentColor"><rect x="3" y="2" width="3.5" height="12" rx="1"></rect><rect x="9.5" y="2" width="3.5" height="12" rx="1"></rect></svg>';
  const ICONO_CORRER = '<svg viewBox="0 0 16 16" fill="currentColor"><path d="M4 2.3v11.4c0 .8.9 1.3 1.6.9l9-5.7c.6-.4.6-1.4 0-1.8l-9-5.7C4.9 1 4 1.5 4 2.3z"></path></svg>';

  // Acepta el elemento o su id, y aguanta que no exista: hubo un tiempo en
  // que llamarlo con el id crudo reventaba con un TypeError dentro de un
  // catch, y el aviso de error no llegaba a verse nunca.
  function pillSet(el, estado, texto) {
    if (typeof el === "string") el = document.getElementById(el);
    if (!el) return;
    el.className = "pill" + (estado ? " " + estado : "");
    el.querySelector("span").textContent = texto;
  }

  // ---------------------------------------------------------------
  // panel global: boton correr/pausar y cronometro
  // ---------------------------------------------------------------
  function actualizarBotonCorrer(estado) {
    const b = document.getElementById("bcorrer");
    b.classList.toggle("btn-run", estado.corriendo);
    b.classList.toggle("btn-pause", !estado.corriendo);
    // El boton dice lo que pasa al picarlo: corriendo, "Pausar"; en pausa, "Correr".
    // El texto va en su span: en pantallas angostas se esconde a la vista
    // pero sigue siendo el nombre del boton para el lector de pantalla.
    b.innerHTML = (estado.corriendo ? ICONO_PAUSA + '<span class="btn-txt">Pausar</span>'
                                    : ICONO_CORRER + '<span class="btn-txt">Correr</span>');
    b.title = estado.corriendo ? "Pausar" : "Correr";
  }

  function actualizarTimer(estado) {
    const total = Math.floor(estado.tiempo);
    const mm = String(Math.floor(total / 60)).padStart(2, "0");
    const ss = String(total % 60).padStart(2, "0");
    document.getElementById("runTimer").textContent = mm + ":" + ss;
  }

  // ---------------------------------------------------------------
  // sidebar: selector de robot + tarjetas de modelo + parametros
  // ---------------------------------------------------------------
  function construirSelectorRobot(estado, onSeleccionar) {
    const cont = document.getElementById("robotSelect");
    cont.innerHTML = "";
    // En Real solo hay robots con hardware detras. Los demas se muestran
    // apagados en vez de esconderse: que la flota siga a la vista deja
    // claro que faltan por conectar, no que dejaron de existir.
    // "filtro" es Real con otra fuente de video: cuenta como Real.
    const real = estado.modo === "real" || estado.modo === "filtro";
    estado.robots.forEach(r => {
      // hardware, no backend: Odiseo se simula en el navegador pero es un
      // robot de verdad, asi que en Real se puede escoger y manejar.
      const disponible = !real || r.hardware;
      const activo = estado.seleccionado === r.id;
      const b = document.createElement("button");
      b.className = (activo ? "on" : "") + (disponible ? "" : " off");
      b.type = "button";
      b.disabled = !disponible;
      if (!disponible) b.title = "sin robot físico conectado";
      // El color del robot va en su punto, junto al nombre: es el mismo con
      // el que se dibuja en el mapa. La ficha elegida se marca con el aro, no
      // rellenandola de color (el relleno de Atlas se leia como alarma).
      b.style.setProperty("--c", r.color);
      // La miniatura es la misma imagen que se dibuja en el mapa: quien
      // mira la cancha reconoce cual es cual por la foto antes que por el
      // nombre. Va sin girar, igual que en la leyenda.
      const mini = document.createElement("img");
      mini.className = "robot-mini";
      mini.src = r.imgSrc;   // la ruta; r.img ya es un Image() del canvas
      mini.alt = "";
      b.appendChild(mini);
      const nom = document.createElement("span");
      nom.className = "robot-nombre";
      nom.innerHTML = "<i></i>";
      nom.appendChild(document.createTextNode(nombreBonito(r.nombre)));
      b.appendChild(nom);
      b.onclick = () => { if (disponible) onSeleccionar(r.id); };
      cont.appendChild(b);
    });
  }

  // ---------------------------------------------------------------
  // sidebar: obstaculos
  // ---------------------------------------------------------------
  // El glifo del tipo de repulsor: el mismo dibujo que el obstaculo lleva
  // en el mapa, en chico. La ficha tiene que decir QUE clase de campo es
  // sin bajar al select, igual que la ficha de robot dice cual es por la
  // foto y no por el nombre.
  //
  // Lo que separa a los cuatro: "espiral" inclina las flechas -ademas de
  // empujar, arrastra de costado- y el largo dice cuanto pega cada
  // formula, que en pixeles va de "muro" (lineal) a "casi nada" (khatib).
  function glifoRepulsor(tipo, sentido) {
    // El icono es la FORMA DEL CAMPO en tres dimensiones: la magnitud de la
    // fuerza levantada sobre el circulo y girada alrededor de su eje. Es lo
    // que de verdad distingue a las cuatro, y como rosetas de flechas se
    // veian casi iguales -las cuatro son flechas hacia afuera-.
    //
    //   lineal        satura en todo el radio y se corta en seco -> CILINDRO
    //   lineal_norm   rampa recta del centro al borde            -> CONO
    //   khatib        nada lejos y muro cerca                    -> CAMPANA
    //   espiral       el cono, pero empujando de lado            -> CONO CON GIRO
    //
    // La base es la misma elipse en los cuatro: es el circulo del alcance
    // visto en perspectiva, y tenerla igual deja comparar las alturas.
    const base = '<ellipse cx="12" cy="18.5" rx="8" ry="2.6"/>';
    const cuerpo = {
      // Tapa plana arriba y paredes rectas: la fuerza vale lo mismo en todo
      // el radio y cae a cero de golpe en el borde.
      lineal:
        '<ellipse cx="12" cy="6.5" rx="8" ry="2.6"/>' +
        '<path d="M4 6.5v12M20 6.5v12"/>',
      // Vertice arriba y faldas rectas hasta la base.
      lineal_norm:
        '<path d="M12 4.2 4 18.5M12 4.2l8 14.3"/>',
      // Hombros redondos y faldas que se van a cero: casi todo el empujon
      // vive en el centro.
      khatib:
        '<path d="M4 18.5c0-2.6.7-5.6 2.4-8.2C8 7.7 9.9 5.6 12 5.6s4 2.1 5.6 4.7' +
        'c1.7 2.6 2.4 5.6 2.4 8.2"/>',
      // El cono otra vez, y encima la vuelta que lo hace espiral.
      espiral:
        '<path d="M12 4.2 4 18.5M12 4.2l8 14.3"/>',
    };

    // El espiral cambia la BASE, no le añade una raya encima: su elipse se
    // abre y termina en punta de flecha, o sea el cono montado sobre un
    // disco que gira. Un cono con una linea cruzada se confundia con el
    // cono a secas, que es justo lo que no puede pasar ahora que el espiral
    // es el tipo por defecto.
    //
    // La flecha se refleja con el sentido, igual que las del mapa.
    if (tipo === "espiral") {
      const horario = (sentido < 0);
      const arco = horario
        ? '<path d="M19.4 17.7A8 2.6 0 1 0 19.4 19.3"/>' +
          '<path d="M17.4 15.9 20 17.6l-2.3 1.6"/>'
        : '<path d="M4.6 17.7A8 2.6 0 1 1 4.6 19.3"/>' +
          '<path d="M6.6 15.9 4 17.6l2.3 1.6"/>';
      return '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" ' +
        'stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">' +
        arco + cuerpo.espiral + '</svg>';
    }

    return '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" ' +
      'stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">' +
      base + (cuerpo[tipo] || cuerpo.lineal_norm) + '</svg>';
  }

  const NOMBRE_REPULSOR = {
    lineal: "Lineal", lineal_norm: "Lineal normalizado",
    espiral: "Espiral", khatib: "Khatib",
  };

  // Mismas fichas que el selector de robots. La ultima es el "+", y
  // desaparece sola al llegar a MAX_OBSTACULOS: un boton que no puede
  // hacer nada es peor que un boton que no esta.
  function construirSelectorObstaculos(estado, onSeleccionar, onAnadir) {
    const cont = document.getElementById("obsSelect");
    cont.innerHTML = "";

    estado.obstaculos.forEach((o, i) => {
      const activo = estado.seleccionadoObs === o.id;
      const b = document.createElement("button");
      b.type = "button";
      b.className = activo ? "on" : "";
      b.title = NOMBRE_REPULSOR[o.tipo] || o.tipo;
      b.innerHTML = '<span class="obs-glifo">' + glifoRepulsor(o.tipo, o.sentido) + "</span>";
      b.appendChild(document.createTextNode("Obstáculo " + (i + 1)));
      b.onclick = () => onSeleccionar(o.id);
      cont.appendChild(b);
    });

    if (estado.obstaculos.length < MAX_OBSTACULOS) {
      const b = document.createElement("button");
      b.type = "button";
      b.className = "obs-mas";
      b.title = "Añadir un obstáculo";
      b.innerHTML = '<span class="obs-glifo"><svg viewBox="0 0 24 24" fill="none" ' +
        'stroke="currentColor" stroke-width="1.8" stroke-linecap="round" aria-hidden="true">' +
        '<path d="M12 5.5v13M5.5 12h13"/></svg></span>';
      b.appendChild(document.createTextNode("Añadir"));
      b.onclick = onAnadir;
      cont.appendChild(b);
    }
  }

  // Las cuatro formulas como fichas, con el mismo esqueleto que el modo de
  // control. El icono es el glifo del propio repulsor: la ficha enseña que
  // campo vas a poner antes de ponerlo, que es justo lo que un desplegable
  // no deja hacer.
  //
  // Los rotulos van cortos a proposito -"Lineal +" por "Lineal
  // normalizado"- porque son un cuarto de panel: el nombre entero vive en
  // el title, y la diferencia de verdad se ve en el dibujo.
  const FICHAS_REPULSOR = [
    { tipo: "lineal", corto: "Lineal", largo: "Lineal — empuja fuerte en todo el radio" },
    { tipo: "lineal_norm", corto: "Lineal +", largo: "Lineal normalizado — el que responde mejor al slider" },
    { tipo: "espiral", corto: "Espiral", largo: "Espiral — además de alejar, rodea el obstáculo" },
    { tipo: "khatib", corto: "Khatib", largo: "Khatib (potencial clásico) — en píxeles casi no empuja" },
  ];

  // Sirve igual para un obstaculo y para el campo propio de un robot: los
  // dos son el mismo objeto {tipo, sentido}, asi que el contenedor entra
  // por parametro en vez de estar escrito aqui dentro.
  function construirSelectorTipoRepulsor(contenedorId, obs, onElegir) {
    const cont = document.getElementById(contenedorId);
    cont.innerHTML = "";
    FICHAS_REPULSOR.forEach(f => {
      const activo = !!obs && obs.tipo === f.tipo;
      const b = document.createElement("button");
      b.type = "button";
      b.className = activo ? "on" : "";
      b.title = f.largo;
      b.setAttribute("aria-pressed", activo ? "true" : "false");
      b.innerHTML = glifoRepulsor(f.tipo, obs ? obs.sentido : 1) +
        "<span>" + f.corto + "</span>";
      b.onclick = () => onElegir(f.tipo);
      cont.appendChild(b);
    });
  }

  // Las cajas del centro, aparte del resto: las escribe tambien el
  // arrastre, cuadro a cuadro, y rehacer el formulario entero a 60 Hz
  // pisaria el slider que estuvieras moviendo con la otra mano.
  function refrescarCentroObstaculo(obs) {
    document.getElementById("oX").value = Math.round(obs.ancla.x);
    document.getElementById("oY").value = Math.round(obs.ancla.y);
  }

  function refrescarFormularioObstaculo(estado) {
    const o = estado.obstaculos.find(x => x.id === estado.seleccionadoObs);
    document.getElementById("obsVacio").hidden = estado.obstaculos.length > 0;
    document.getElementById("obsParams").hidden = !o;
    if (!o) return;

    refrescarCentroObstaculo(o);
    document.getElementById("sORadio").value = o.radio;
    document.getElementById("vORadio").textContent = o.radio + " px";
    document.getElementById("sOEta").value = o.eta;
    document.getElementById("vOEta").textContent = o.eta;

    // El sentido solo existe para el espiral: es el unico que empuja de
    // costado. En los otros tres la fila no se esconde por gusto, es que no
    // habria nada que elegir.
    document.getElementById("filaSentido").hidden = o.tipo !== "espiral";
    document.querySelectorAll("#obsSentido button").forEach(b =>
      b.classList.toggle("on", +b.dataset.sentido === (o.sentido < 0 ? -1 : 1)));

    document.querySelectorAll("#obsMovSelect button").forEach(b => {
      const activo = b.dataset.mov === o.mov;
      b.classList.toggle("on", activo);
      b.setAttribute("aria-pressed", activo ? "true" : "false");
    });
    document.getElementById("paramsObsRecta").hidden = o.mov !== "recta";
    // Circulo y ocho comparten riel de tamaño; lo unico que cambia es como
    // se llama, porque en uno es un radio y en el otro el brazo del lazo.
    const conCurva = o.mov === "circular" || o.mov === "lemniscata";
    document.getElementById("filaObsOrbita").hidden = !conCurva;
    document.getElementById("oOrbitaTit").textContent =
      o.mov === "lemniscata" ? "Tamaño del ocho" : "Radio de la órbita";
    // La velocidad es de los TRES caminos, no de uno: vive fuera de sus
    // grupos y solo se esconde cuando el obstaculo esta quieto.
    document.getElementById("filaObsVel").hidden = o.mov === "fijo";

    document.getElementById("sOLargo").value = o.largo;
    document.getElementById("vOLargo").textContent = o.largo + " px";
    document.getElementById("sOAng").value = o.angulo;
    document.getElementById("vOAng").textContent = o.angulo + "\u00B0";
    document.getElementById("sOOrbita").value = o.orbita;
    document.getElementById("vOOrbita").textContent = o.orbita + " px";
    document.getElementById("sOVel").value = o.vel;
    document.getElementById("vOVel").textContent = o.vel.toFixed(2);
  }

  // "ATLAS" -> "Atlas": los nombres viven en mayusculas en config.js (asi
  // se rotulan en el mapa), pero en el panel van como nombre propio.
  function nombreBonito(n) {
    return n.charAt(0) + n.slice(1).toLowerCase();
  }

  function robotSeleccionado(estado) {
    return estado.robots.find(r => r.id === estado.seleccionado);
  }

  // ---------------------------------------------------------------
  // sidebar: el campo repulsivo que lleva encima el robot
  // ---------------------------------------------------------------
  function refrescarRepulsorRobot(robot, onElegirTipo) {
    const rp = robot.repulsor;
    document.getElementById("sRRadio").value = rp.radio;
    document.getElementById("vRRadio").textContent = rp.radio + " px";
    document.getElementById("sREta").value = rp.eta;
    document.getElementById("vREta").textContent = rp.eta;
    // Apagado, el bloque entero baja de tono: los mandos siguen ahi y se
    // pueden tocar, pero de un vistazo se ve que ese robot no empuja.
    document.getElementById("bloqueRepulsor").classList.toggle("apagado", rp.eta <= 0);
    // El interruptor de la cabecera dice si el robot lleva campo, sin abrir nada.
    const sw = document.getElementById("repOn");
    sw.classList.toggle("on", rp.eta > 0);
    sw.setAttribute("aria-checked", rp.eta > 0 ? "true" : "false");

    construirSelectorTipoRepulsor("repTipoSelect", rp, onElegirTipo);
    document.getElementById("filaSentidoRep").hidden = rp.tipo !== "espiral";
    document.querySelectorAll("#repSentido button").forEach(b =>
      b.classList.toggle("on", +b.dataset.sentido === (rp.sentido < 0 ? -1 : 1)));
  }

  function mostrarGrupoParametros(modelo) {
    document.getElementById("paramsPunto").hidden = modelo !== "punto";
    document.getElementById("paramsTrayectoria").hidden = modelo !== "trayectoria";
    document.getElementById("paramsFormacion").hidden = modelo !== "formacion";
    document.getElementById("paramsManual").hidden = modelo !== "manual";
    // El conmutador vive en la fila del tag, fuera del grupo Manual, pero
    // solo tiene sentido dentro de el.
    document.getElementById("bloqueModoManual").hidden = modelo !== "manual";
    // Las fichas de modo no guardan estado propio: la ficha encendida es la
    // del modelo del robot seleccionado, igual que en la barra de robots.
    document.querySelectorAll("#modoSelect button").forEach(b => {
      const activo = b.dataset.modelo === modelo;
      b.classList.toggle("on", activo);
      b.setAttribute("aria-pressed", activo ? "true" : "false");
    });
  }

  // Los nombres de rueda dependen del robot: Atlas es mecanum de 4, y
  // Apolo y Odiseo son uniciclos de 2. El ORDEN es el de la trama serial,
  // no uno geometrico: para Atlas lo dicta CFG.orden, del backend.
  function etiquetasRuedas(robot, cfg) {
    // Uniciclo: el orden del PWM en esta pagina es [derecha, izquierda],
    // el mismo que produce avanzarUniciclo() y que muestra la telemetria.
    // La flota lo lleva al reves, y el cruce se hace en un solo lugar, al
    // salir: ver mandarAlRobotReal() en main.js.
    if (robot.tipo !== "holonomico") return ["derecha", "izquierda"];
    if (cfg && cfg.orden && cfg.orden.length === 4) return cfg.orden.map(o => o.rueda);
    return ["RR", "FR", "FL", "RL"];
  }

  // Donde cae cada rueda sobre la imagen. Se mapea por NOMBRE, no por
  // posicion en la trama: si algun dia se vuelve a medir FRAME_LAYOUT y
  // cambia el orden, las cajas siguen quedando en la esquina correcta.
  const ESQUINA_RUEDA = {
    FL: "mr-ti", FR: "mr-td", RR: "mr-bd", RL: "mr-bi",
    izquierda: "mr-t", derecha: "mr-b",
  };

  function construirRuedasManual(robot, cfg) {
    const cont = document.getElementById("manualRuedas");
    const etiquetas = etiquetasRuedas(robot, cfg);
    const pwm = robot.parametros.manual.pwm;

    cont.innerHTML =
      '<div class="manual-mapa">' +
        '<img src="' + robot.imgSrc + '" alt="">' +
        etiquetas.map((nombre, i) =>
          '<div class="manual-rueda ' + (ESQUINA_RUEDA[nombre] || "mr-ti") + '">' +
            '<label>' + nombre + '</label>' +
            '<input type="number" id="iMan' + i + '" step="5" min="-255" max="255" ' +
              'value="' + (pwm[i] || 0) + '">' +
          '</div>').join("") +
      '</div>';

    etiquetas.forEach((_, i) => {
      const caja = document.getElementById("iMan" + i);
      const leer = () => {
        // Se recorta al rango del firmware y se reescribe en la caja: si
        // tecleas 400, ves 255, en vez de que el robot rechace la trama
        // despues sin explicacion.
        let v = Math.round(+caja.value || 0);
        v = Math.max(-255, Math.min(255, v));
        caja.value = v;
        robot.parametros.manual.pwm[i] = v;
      };
      caja.onchange = leer;
      caja.onblur = leer;
    });
  }

  // ---------------------------------------------------------------
  // cruceta del modo manual
  // ---------------------------------------------------------------
  // Un solo glifo por familia; las otras tres orientaciones salen de
  // espejearlo. Asi las cuatro diagonales (y los cuatro arcos) son
  // exactamente la misma curva, y no hay una que se vea mas gorda o mas
  // corta que sus hermanas.
  const svg = cuerpo =>
    '<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.6" ' +
    'stroke-linecap="round" stroke-linejoin="round">' + cuerpo + "</svg>";
  const espejoH = c => '<g transform="translate(16,0) scale(-1,1)">' + c + "</g>";
  const espejoV = c => '<g transform="translate(0,16) scale(1,-1)">' + c + "</g>";
  const espejoHV = c => '<g transform="translate(16,16) scale(-1,-1)">' + c + "</g>";

  const RECTA_ARRIBA = '<path d="M8 13.2V3.4"></path><path d="M4.2 7.2 8 3.4l3.8 3.8"></path>';
  const RECTA_ABAJO = '<path d="M8 2.8v9.8"></path><path d="M11.8 8.8 8 12.6 4.2 8.8"></path>';
  const RECTA_IZQ = '<path d="M13.2 8H3.4"></path><path d="M7.2 4.2 3.4 8l3.8 3.8"></path>';
  const RECTA_DER = '<path d="M2.8 8h9.8"></path><path d="M8.8 4.2 12.6 8l-3.8 3.8"></path>';
  // Diagonal al noroeste: desplazarse de canto, que es cosa del mecanum.
  const DIAGONAL = '<path d="M12.2 12.2 4.3 4.3"></path><path d="M4.3 9.5V4.3h5.2"></path>';
  // Arco al noroeste: avanzar girando, que es lo mas cerrado que puede
  // hacer un uniciclo hacia ese lado.
  // Cuarto de circunferencia de radio 7.5 centrado en (4.4, 12.4): entra
  // por abajo apuntando hacia arriba y sale por la izquierda apuntando a la
  // izquierda, que es justo el cuarto de vuelta que da el robot. Un cubico
  // a ojo salia con la curvatura amontonada al final y a 20 px se leia como
  // un gancho, no como un giro.
  const ARCO = '<path d="M12.4 12.9A8.6 8.6 0 0 0 3.8 4.3"></path>' +
               '<path d="M6.6 5.9 3.8 4.3 6.6 2.7"></path>';
  // Giro en el sitio. Es el mismo glifo que el boton de reiniciar del
  // panel global, que ya es el que se lee como "gira".
  const GIRO_DER = '<path d="M13.5 8A5.5 5.5 0 1 1 11.6 4"></path><path d="M13.5 2v3.6H9.9"></path>';
  const GIRO_IZQ = espejoH(GIRO_DER);

  function botonFlecha(dir, icono, titulo) {
    return '<button type="button" class="dp" data-dir="' + dir + '" title="' + titulo + '">' +
           svg(icono) + "</button>";
  }

  // Arma la cruceta del robot. Las dos disposiciones no son un capricho de
  // dibujo: son lo que cada chasis sabe hacer.
  //
  //   mecanum (Atlas)        uniciclo (Apolo, Odiseo)
  //   D  ^  D                A  ^  A     esquinas: avanzar/retroceder girando
  //   <  pot  >              O  pot  O   costados:  lateral / giro en el sitio
  //   D  v  D                A  v  A
  //      O  O                            el mecanum saca el giro debajo,
  //                                      porque los costados ya se los lleva
  //                                      el desplazamiento lateral
  function construirFlechasManual(robot) {
    const holo = robot.tipo === "holonomico";
    const izq = holo ? "lateral-izq" : "giro-izq";
    const der = holo ? "lateral-der" : "giro-der";
    const esquina = holo ? DIAGONAL : ARCO;
    // Las esquinas son las dos teclas a la vez, y el titulo lo dice: no hay
    // una tecla de diagonal que buscar.
    const t = holo
      ? ["Diagonal adelante-izquierda (↑ + ←)", "Diagonal adelante-derecha (↑ + →)",
         "Diagonal atrás-izquierda (↓ + ←)", "Diagonal atrás-derecha (↓ + →)",
         "Lateral izquierda (←)", "Lateral derecha (→)"]
      : ["Adelante girando a la izquierda (↑ + ←)", "Adelante girando a la derecha (↑ + →)",
         "Atrás girando a la izquierda (↓ + ←)", "Atrás girando a la derecha (↓ + →)",
         "Giro a la izquierda (←)", "Giro a la derecha (→)"];

    document.getElementById("dpadCruz").innerHTML =
      botonFlecha("adelante " + izq, esquina, t[0]) +
      botonFlecha("adelante", RECTA_ARRIBA, "Adelante (↑)") +
      botonFlecha("adelante " + der, espejoH(esquina), t[1]) +
      botonFlecha(izq, holo ? RECTA_IZQ : GIRO_IZQ, t[4]) +
      '<div class="dp-centro" id="dpPot">0</div>' +
      botonFlecha(der, holo ? RECTA_DER : GIRO_DER, t[5]) +
      botonFlecha("atras " + izq, espejoV(esquina), t[2]) +
      botonFlecha("atras", RECTA_ABAJO, "Atrás (↓)") +
      botonFlecha("atras " + der, espejoHV(esquina), t[3]);

    const giro = document.getElementById("dpadGiro");
    giro.innerHTML = holo
      ? botonFlecha("giro-izq", GIRO_IZQ, "Giro a la izquierda (Q)") +
        botonFlecha("giro-der", GIRO_DER, "Giro a la derecha (E)")
      : "";
    giro.hidden = !holo;
  }

  // Conmutador ruedas <-> flechas. No cambia lo que se manda ni por donde:
  // en los dos casos es una trama de PWM crudo por el mismo /manual. Lo
  // unico que cambia es quien escribe los numeros, asi que el resto del
  // panel (duracion, Parar, casilla de MQTT) se queda igual.
  function mostrarModoManual(robot, cfg) {
    const flechas = robot.parametros.manual.modo === "flechas";
    document.getElementById("manualRuedas").hidden = flechas;
    document.getElementById("manualFlechas").hidden = !flechas;
    const sw = document.getElementById("mModo");
    sw.classList.toggle("on", flechas);
    sw.setAttribute("aria-checked", flechas ? "true" : "false");
    document.getElementById("mmRuedas").classList.toggle("on", !flechas);
    document.getElementById("mmFlechas").classList.toggle("on", flechas);
    // Conducir no tiene duracion ni boton de enviar: dura lo que dure la
    // pulsacion, y la orden ES la pulsacion.
    document.getElementById("filaSegundos").hidden = flechas;
    document.querySelector(".manual-btns").hidden = flechas;
    construirFlechasManual(robot);
    refrescarPotencia(robot);
    refrescarFlechas(robot, cfg);
  }

  function refrescarPotencia(robot) {
    const p = robot.parametros.manual.potencia;
    document.getElementById("sPot").value = p;
    document.getElementById("dpPot").textContent = p;
  }

  // El bucle de dibujo llama a esto en cada cuadro, y la trama solo cambia
  // cuando se pisa o se suelta una flecha: rehacer el HTML 60 veces por
  // segundo para escribir lo mismo no tiene sentido.
  let ultimaTrama = null;
  function refrescarFlechas(robot, cfg) {
    const m = robot.parametros.manual;
    const html = etiquetasRuedas(robot, cfg)
      .map((nombre, i) => nombre + " <b>" + (m.pwm[i] || 0) + "</b>").join(" · ");
    if (html !== ultimaTrama) {
      document.getElementById("mTrama").innerHTML = html;
      ultimaTrama = html;
    }
    document.getElementById("dpPot").classList.toggle("activo", m.activo);
  }

  // Ilumina la flecha que corresponde EXACTAMENTE a lo que se esta pisando.
  // Con las dos teclas se enciende la esquina, no las dos rectas: lo que
  // hace el robot es la diagonal, y eso es lo que hay que ver.
  function resaltarFlechas(direcciones) {
    document.querySelectorAll("#manualFlechas .dp").forEach(b => {
      const suyas = b.dataset.dir.split(" ");
      b.classList.toggle("on",
        suyas.length === direcciones.size && suyas.every(d => direcciones.has(d)));
    });
  }

  // Marca en rojo las ruedas con PWM distinto de cero mientras corre.
  function resaltarRuedasManual(robot) {
    robot.parametros.manual.pwm.forEach((v, i) => {
      const caja = document.getElementById("iMan" + i);
      if (caja) caja.classList.toggle("vivo", robot.parametros.manual.activo && v !== 0);
    });
  }

  // Armado: si los robots fisicos obedecen de verdad. Es UNO para los tres
  // y arranca en no. Esta siempre a la vista, en Real y en Simulacion: es un
  // interruptor propio y no una consecuencia de la vista.
  //
  // Marcada si CUALQUIERA esta armado, no solo si lo estan todos: la casilla
  // arma y desarma a los tres juntos, pero si algun dia difieren, lo que
  // tiene que decir es que hay algo suelto.
  function refrescarArmado(estado) {
    const armado = estado.robots.some(r => r.armado);
    document.getElementById("mReal").checked = armado;
    document.getElementById("bloqueArmado").classList.toggle("armado", armado);
  }

  // Vuelca los parametros del robot seleccionado (y la formacion global,
  // compartida entre todos) en los controles del formulario - se llama al
  // cambiar de robot, para no mezclar valores de uno con los de otro.
  function refrescarFormulario(robot, formacionGlobal) {
    mostrarGrupoParametros(robot.modelo);

    const p = robot.parametros.punto;
    document.getElementById("pDestX").value = Math.round(robot.destino.x);
    document.getElementById("pDestY").value = Math.round(robot.destino.y);
    document.getElementById("sk").value = p.k;
    document.getElementById("vk").textContent = p.k.toFixed(2);
    document.getElementById("se").value = Math.round(p.escala * 100);
    document.getElementById("ve").textContent = Math.round(p.escala * 100) + "%";
    // Las dos zonas NO se vuelcan aqui: viven en el panel global y son las
    // mismas para los tres. Cambiar de robot no tiene por que moverlas, y
    // si se volcaran desde el seleccionado bastaria un robot con el valor
    // viejo -uno creado antes, o el reinicio- para tirar atras el riel.
    document.getElementById("pDestTh").value = Math.round(robot.destino.th * 180 / Math.PI);

    const t = robot.parametros.trayectoria;
    const centro = t.centroInicial || [robot.pose.x, robot.pose.y];
    document.getElementById("tCentroX").value = Math.round(centro[0]);
    document.getElementById("tCentroY").value = Math.round(centro[1]);
    document.getElementById("tTipo").value = t.tipo;
    document.getElementById("tRadio").value = String(t.radio);
    document.getElementById("vRadio").textContent = t.radio;
    document.getElementById("sOmega").value = t.omega;
    document.getElementById("vOmega").textContent = t.omega.toFixed(2);

    // formacion es GLOBAL (una sola formacion compartida por los 3 robots),
    // no por-robot como punto/trayectoria - ver la nota en main.js.
    const f = formacionGlobal;
    document.getElementById("sFa").value = f.a;
    document.getElementById("vFa").textContent = f.a;
    document.getElementById("sFk").value = f.k;
    document.getElementById("vFk").textContent = f.k.toFixed(2);

    // El bloque de Sistema describe la cinematica mecanum de Atlas: no
    // aplica a los Apolo, que son uniciclo.
    document.getElementById("grupoSistema").style.display = robot.backend ? "" : "none";

    // La pagina de telemetria dice de quien son las graficas.
    const tira = document.getElementById("tiraRobot");
    tira.style.setProperty("--c", robot.color);
    tira.querySelector("span").textContent = nombreBonito(robot.nombre);
  }

  // ---------------------------------------------------------------
  // telemetria (siempre del robot seleccionado)
  // ---------------------------------------------------------------
  // Los numeros de pose/control ya no viven aqui: los cuentan mejor las
  // series de tiempo de graficas.js. Esto solo mantiene el bloque de
  // Sistema -que es configuracion fija, no telemetria: no cambia con el
  // tiempo, graficarlo no diria nada- y la pildora de la camara.
  function actualizarTelemetria(robot, info, conectado, mudo, camaraViva, camaraTiene, cfg) {
    if (robot.backend && cfg) {
      document.getElementById("tcfg").textContent = cfg.config_rodillos;
      document.getElementById("taut").textContent = Math.round(cfg.autoridad * 100) + "%";
      document.getElementById("temi").textContent = Math.round(cfg.escala_minima * 100) + "%";
      const pillCamara = document.getElementById("pillCamara");
      if (!camaraViva) pillSet(pillCamara, "warn", "sin cámara");
      else if (camaraTiene) pillSet(pillCamara, "ok", "ID0 y ID1 detectados");
      else pillSet(pillCamara, "", "activa, sin marcadores");
    }
  }

  // ---------------------------------------------------------------
  // leyenda colapsable
  // ---------------------------------------------------------------
  function iniciarLeyenda() {
    document.getElementById("legendHead").onclick = () => {
      document.getElementById("legend").classList.toggle("collapsed");
    };
  }

  return {
    actualizarBotonCorrer, actualizarTimer,
    construirSelectorRobot, robotSeleccionado, mostrarGrupoParametros, refrescarFormulario,
    construirRuedasManual, resaltarRuedasManual,
    mostrarModoManual, refrescarPotencia, refrescarFlechas, resaltarFlechas,
    refrescarArmado,
    actualizarTelemetria, iniciarLeyenda,
    pillSet,
    construirSelectorObstaculos, construirSelectorTipoRepulsor,
    refrescarRepulsorRobot,
    refrescarFormularioObstaculo,
    refrescarCentroObstaculo,
  };
})();
