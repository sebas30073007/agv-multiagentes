// canvas.js - todo lo que se dibuja: reticula, rastro, destino, robots.
// No sabe nada de control ni de red; solo recibe robots ya posicionados.
"use strict";

const Escena = (function () {
  let lienzo, ctx, CFG;
  // Sobre el video la retícula va INVERTIDA: la imagen de la cámara es
  // oscura, así que las líneas claras son las que se leen. Y no se pinta
  // fondo, obviamente, o taparía el video.
  let sobreVideo = false;
  // Sobre el video el chasis NO se dibuja. La imagen de verdad ya esta
  // debajo, y un sprite encima -aunque sea translucido- tapa justo lo unico
  // que interesa mirar. Queda el rombo negro sobre el marcador, que es lo
  // que dice donde cree la deteccion que esta el robot: si no cae sobre el
  // ArUco, la pose esta mal, y eso se ve de un vistazo.
  let chasisVisible = true;

  function mostrarChasis(v) { chasisVisible = !!v; }

  // Cuadricula estilo papel milimetrico: tres pasos en proporcion 1:5:10,
  // cada uno mas oscuro que el anterior. La fina solo situa, las de decena
  // son las que se leen, asi que el contraste tiene que ir en ese orden.
  const NIVELES_GRID = [
    { paso: 20, color: "#EEF0F2", grosor: 1 },
    { paso: 100, color: "#DDE0E4", grosor: 1 },
    { paso: 200, color: "#C4C9D0", grosor: 1 },
  ];
  // Misma proporción 1:5:10, pero en claro y con alfa: sobre el video una
  // rejilla opaca compite con la imagen y estorba para ver los marcadores.
  const NIVELES_GRID_VIDEO = [
    { paso: 20, color: "rgba(255,255,255,0.13)", grosor: 1 },
    { paso: 100, color: "rgba(255,255,255,0.26)", grosor: 1 },
    { paso: 200, color: "rgba(255,255,255,0.44)", grosor: 1 },
  ];
  const COLOR_EJE_VIDEO = "rgba(255,255,255,0.72)";
  const COLOR_COTA_VIDEO = "rgba(255,255,255,0.6)";
  // Destino y referencia sobre el video: claros y gruesos. Lo que sobre
  // el mapa blanco seria gritar, sobre la imagen de la camara es lo
  // minimo para que se vea.
  const COLOR_REFERENCIA_VIDEO = "rgba(242,244,248,0.9)";
  const GROSOR_REFERENCIA_VIDEO = 3;
  const GROSOR_DESTINO_VIDEO = 3.5;
  const PASO_MAYOR = 200;
  // El eje vertical es mas corto y aguanta el doble de cotas sin apelmazarse.
  const PASO_COTA_X = 200;
  const PASO_COTA_Y = 100;
  const COLOR_EJE = "#9CA3AF";
  const COLOR_COTA = "#A3A8B0";

  function init(canvasEl, cfg) {
    lienzo = canvasEl;
    ctx = lienzo.getContext("2d");
    CFG = cfg;
  }

  // Cambia a qué lienzo se dibuja. Solo hay una vista visible a la vez, así
  // que un único módulo alterna entre el mapa y la capa sobre el video en
  // vez de mantener dos instancias con estado duplicado.
  function usarLienzo(canvasEl, opciones) {
    lienzo = canvasEl;
    ctx = lienzo.getContext("2d");
    sobreVideo = !!(opciones && opciones.sobreVideo);
  }

  function actualizarCFG(cfg) { CFG = cfg; }

  function ajustarLienzo() {
    const r = lienzo.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    lienzo.width = Math.max(1, Math.round(r.width * dpr));
    lienzo.height = Math.max(1, Math.round(r.height * dpr));
  }

  const esc = () => Math.min(lienzo.width / CFG.ancho, lienzo.height / CFG.alto);
  const offX = () => (lienzo.width - CFG.ancho * esc()) / 2;
  const offY = () => (lienzo.height - CFG.alto * esc()) / 2;
  const aX = x => offX() + (x + CFG.ancho / 2) * esc();
  const aY = y => offY() + (CFG.alto / 2 - y) * esc();

  function aMundo(clientX, clientY) {
    const r = lienzo.getBoundingClientRect();
    const bx = (clientX - r.left) / r.width * lienzo.width;
    const by = (clientY - r.top) / r.height * lienzo.height;
    return { x: (bx - offX()) / esc() - CFG.ancho / 2, y: CFG.alto / 2 - (by - offY()) / esc() };
  }

  // Primer multiplo de "paso" dentro de [-limite, limite]. Anclar en 0 y no
  // en el borde es lo que hace que las lineas caigan sobre los ejes: antes
  // se empezaba en -ancho/2 (-640) y con paso 100 la serie era -640, -540...
  // que nunca pasa por 0, asi que la cruz del origen quedaba a contratiempo
  // de la cuadricula.
  function primeraLinea(limite, paso) {
    return Math.ceil(-limite / paso) * paso;
  }

  function reticula() {
    const e = esc();
    const limX = CFG.ancho / 2, limY = CFG.alto / 2;
    const x0 = offX(), x1 = offX() + CFG.ancho * e;
    const y0 = offY(), y1 = offY() + CFG.alto * e;

    // Fuera de la cancha, el gris del visor; la cancha, blanca. Asi se ve
    // donde acaba el mundo sin necesidad de un marco.
    if (!sobreVideo) {
      ctx.fillStyle = "#E5E7EB";
      ctx.fillRect(0, 0, lienzo.width, lienzo.height);
      ctx.fillStyle = "#FFFFFF";
      ctx.fillRect(Math.min(x0, x1), Math.min(y0, y1), Math.abs(x1 - x0), Math.abs(y1 - y0));
    }

    for (const nivel of (sobreVideo ? NIVELES_GRID_VIDEO : NIVELES_GRID)) {
      // Si el paso queda por debajo de unos pocos pixeles reales, las
      // lineas se tocan y la cuadricula se vuelve una mancha gris.
      if (nivel.paso * e < 4) continue;

      ctx.strokeStyle = nivel.color;
      ctx.lineWidth = nivel.grosor;
      ctx.beginPath();
      for (let x = primeraLinea(limX, nivel.paso); x <= limX; x += nivel.paso) {
        const px = Math.round(aX(x)) + 0.5;   // sobre pixel entero: sin bordes borrosos
        ctx.moveTo(px, y0); ctx.lineTo(px, y1);
      }
      for (let y = primeraLinea(limY, nivel.paso); y <= limY; y += nivel.paso) {
        const py = Math.round(aY(y)) + 0.5;
        ctx.moveTo(x0, py); ctx.lineTo(x1, py);
      }
      ctx.stroke();
    }

    // Ejes: mismo trazo que la cuadricula pero mas oscuro, y ya caen justo
    // sobre lineas de la rejilla.
    ctx.strokeStyle = sobreVideo ? COLOR_EJE_VIDEO : COLOR_EJE;
    ctx.lineWidth = 1;
    ctx.beginPath();
    const ejeX = Math.round(aX(0)) + 0.5, ejeY = Math.round(aY(0)) + 0.5;
    ctx.moveTo(ejeX, y0); ctx.lineTo(ejeX, y1);
    ctx.moveTo(x0, ejeY); ctx.lineTo(x1, ejeY);
    ctx.stroke();

    ctx.fillStyle = sobreVideo ? COLOR_COTA_VIDEO : COLOR_COTA;
    ctx.font = "500 11px 'IBM Plex Mono', monospace";
    ctx.textAlign = "center";
    for (let x = primeraLinea(limX, PASO_COTA_X); x <= limX; x += PASO_COTA_X) {
      if (x === 0) continue;
      ctx.fillText(x + " px", aX(x), y1 - 8);
    }
    ctx.textAlign = "left";
    for (let y = primeraLinea(limY, PASO_COTA_Y); y <= limY; y += PASO_COTA_Y) {
      if (y === 0) continue;
      ctx.fillText(y + " px", x0 + 8, aY(y) - 5);
    }
  }

  // Lado de la caja de un marcador. Sale de ancho_robot_px, que es el mismo
  // para los tres robots, asi que los marcadores salen todos del mismo
  // tamaño aunque los sprites se dibujen a escalas distintas.
  function ladoMarca() {
    return (CFG.ancho_robot_px || 250) * esc() / 2 * TAM_MARCA;
  }

  // Dibuja un marcador centrado en (x,y) y girado a "th". Real y deseado
  // usan la misma caja y el mismo centro, por eso al alcanzar la pose
  // objetivo las dos flechas quedan una encima de la otra.
  function dibujarMarca(marca, x, y, th, realce) {
    if (!marca) return;
    const b = ladoMarca() * (realce || 1);
    ctx.save();
    ctx.translate(aX(x), aY(y));
    ctx.rotate(-th);
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = "high";
    ctx.drawImage(marca, -b / 2, -b / 2, b, b);
    ctx.restore();
  }

  // Pose objetivo: la FLECHA en el color del robot (real.png tintado, que
  // es marca.png sin el circulo) y, aparte, el anillo de la zona de llegada.
  //
  // Antes el anillo venia dentro del PNG y media siempre lo mismo. El
  // problema no era el tamaño sino lo que decia: un circulo alrededor del
  // destino se lee como "hasta aqui cuenta como haber llegado", y no lo
  // era -esa zona vivia en un numero del panel y no se veia por ningun
  // lado-. Ahora el anillo ES ese numero: mover "Zona de llegada" lo abre y
  // lo cierra en el mapa, y con 2 px queda un aro apretado contra la flecha.
  //
  // Solo en Punto. En trayectoria el destino es un punto que se recalcula
  // cada tick y no hay zona de llegada que dibujar; en formacion no se
  // dibuja destino en absoluto.
  //
  // Sobre el video la flecha cambia de piel. El piso del laboratorio es
  // oscuro y la flecha en el color del robot -sobre todo el morado de
  // Odiseo y el rojo de Atlas- se hunde en el fondo hasta desaparecer:
  // estaba dibujada, pero no se veia. Encima del video va la version casi
  // blanca, con sombra oscura detras para que tampoco se pierda en el
  // reflejo de la lampara; el anillo se queda en el color del robot, que es
  // lo que dice de quien es el destino.
  function dibujarDestino(robot, realce) {
    if (robot.modelo === "punto") dibujarZonaLlegada(robot, realce);

    if (!sobreVideo) {
      dibujarMarca(robot.flechaDeseada, robot.destino.x, robot.destino.y,
                   robot.destino.th, realce);
      return;
    }

    ctx.save();
    ctx.shadowColor = "rgba(0,0,0,0.65)";
    ctx.shadowBlur = 6;
    dibujarMarca(robot.flechaDeseadaVideo || robot.flechaDeseada,
                 robot.destino.x, robot.destino.y, robot.destino.th, realce);
    ctx.restore();
  }

  function dibujarZonaLlegada(robot, realce) {
    const tol = (robot.parametros.punto.tolPos || 0) * esc() * (realce || 1);
    // Por debajo de un par de pixeles no es un aro, es un punto gordo
    // encima de la flecha: se dibujaria peor de lo que no se dibuja.
    if (tol < 2) return;
    const x = aX(robot.destino.x), y = aY(robot.destino.y);

    ctx.save();
    if (sobreVideo) {
      // Trazo oscuro debajo: un aro de color sobre el reflejo de la
      // lampara cenital se pierde igual que se perdia el marcador.
      ctx.lineWidth = GROSOR_DESTINO_VIDEO + 2.5;
      ctx.strokeStyle = "rgba(0,0,0,0.55)";
      ctx.beginPath();
      ctx.arc(x, y, tol, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.lineWidth = sobreVideo ? GROSOR_DESTINO_VIDEO : 1.6;
    ctx.strokeStyle = robot.color;
    // Punteado y no continuo: no es un borde, no choca nada con el. Es
    // hasta donde el lazo da el punto por bueno y se calla.
    ctx.setLineDash(sobreVideo ? [7, 6] : [5, 5]);
    ctx.beginPath();
    ctx.arc(x, y, tol, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.restore();
  }

  // Referencia: a donde DEBERIA ir, punteado y tenue. Para un punto es el
  // segmento recto pose->destino; para una trayectoria es la curva
  // parametrica completa de una vuelta, dibujada de antemano.
  //
  // OJO CON EL ORIGEN DEL PATH: el patron de guiones se reparte desde
  // donde EMPIEZA el path. Si el path arranca en la pose del robot, cada
  // frame arranca en otro punto y los guiones se ven "correr" sobre una
  // linea que no se mueve. Por eso el segmento se traza desde el DESTINO
  // (que esta quieto) hacia el robot: la fase de los guiones queda
  // anclada y solo se acorta el extremo movil.
  function dibujarReferencia(robot) {
    ctx.save();
    // Sobre el video, guiones mas largos y linea mas gruesa en casi blanco:
    // el "+40" del final del color es un alfa del 25%, que sobre el mapa
    // claro se lee bien y sobre la imagen de la camara no se ve en absoluto.
    if (sobreVideo) {
      ctx.setLineDash([9, 7]);
      ctx.lineWidth = GROSOR_REFERENCIA_VIDEO;
      ctx.strokeStyle = COLOR_REFERENCIA_VIDEO;
    } else {
      ctx.setLineDash([6, 6]);
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = robot.color + "40";
    }
    ctx.lineDashOffset = 0;

    if (robot.modelo === "punto") {
      ctx.beginPath();
      ctx.moveTo(aX(robot.destino.x), aY(robot.destino.y));
      ctx.lineTo(aX(robot.pose.x), aY(robot.pose.y));
      ctx.stroke();
    } else if (robot.modelo === "trayectoria") {
      const t = robot.parametros.trayectoria;
      // Sin centro definido no se dibuja nada: usar la pose como respaldo
      // haria que la curva entera siguiera al robot frame a frame.
      if (t.centroInicial) {
        // Se recorre la FASE de 0 a 2*pi, no el tiempo: la forma de la
        // curva no depende de la velocidad angular, asi que mover omega no
        // tiene por que redibujarla.
        const fn = t.tipo === "lemniscata" ? trayectoriaLemniscata : trayectoriaCirculo;
        const N = 180;
        ctx.beginPath();
        for (let i = 0; i <= N; i++) {
          const p = fn(2 * Math.PI * i / N, t.centroInicial, t.radio, 1);
          const px = aX(p[0]), py = aY(p[1]);
          if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
        }
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  // Cruz del centro de la trayectoria. Es tambien el asa para arrastrarlo
  // con el mouse (el hit-test vive en main.js); escribir las coordenadas a
  // mano en los campos sirve para afinar, no para colocarlo.
  function dibujarCentroTrayectoria(robot, realce) {
    const t = robot.parametros.trayectoria;
    if (robot.modelo !== "trayectoria" || !t.centroInicial) return;
    const e = esc();
    const px = aX(t.centroInicial[0]), py = aY(t.centroInicial[1]);
    const b = TAM_CENTRO_TRAYECTORIA * e * (realce || 1);
    ctx.save();
    ctx.strokeStyle = robot.color;
    ctx.lineWidth = 2;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(px - b, py); ctx.lineTo(px + b, py);
    ctx.moveTo(px, py - b); ctx.lineTo(px, py + b);
    ctx.stroke();
    ctx.restore();
  }

  // ---------------------------------------------------------------
  // OBSTACULOS
  // ---------------------------------------------------------------
  // Un obstaculo es un circulo con un campo dentro. El circulo dice hasta
  // donde alcanza -fuera de el la fuerza es exactamente cero, las cuatro
  // formulas comparten ese contrato- y las flechas dicen que clase de
  // campo es, que si no habria que abrir el panel para saberlo.
  //
  // Las flechas estan quietas mientras no pase nada. Se ponen a marchar
  // hacia afuera cuando hay un robot dentro del radio Y la simulacion
  // corre: es la unica señal de que el obstaculo esta empujando a alguien
  // ahora mismo, y no solo de que existe.
  // CORONAS, DENSIDAD_ROBOT, LARGO_FLECHA y LARGO_MINIMO viven en config.js:
  // los usa tambien escena3d.js, y dos copias de los mismos numeros acaban
  // separandose.

  function dibujarObstaculo(obs, realce) {
    if (obs.mov !== "fijo") dibujarCaminoObstaculo(obs, esc(), COLOR_OBSTACULO);
    dibujarCampo(obs.pos.x, obs.pos.y, obs.radio, obs,
                 sobreVideo ? COLOR_OBSTACULO_VIDEO : COLOR_OBSTACULO, realce, 1);
  }

  // El campo que lleva encima un robot: mismo dibujo, centrado en el, y en
  // SU color en vez de rojo. Aqui lo que hay que leer no es "esto repele"
  // -eso ya lo dicen las flechas- sino de quien es el campo.
  function dibujarRepulsorRobot(robot) {
    const rp = robot.repulsor;
    if (!rp || rp.eta <= 0) return;
    dibujarCampo(robot.pose.x, robot.pose.y, rp.radio, rp, robot.color, 1, DENSIDAD_ROBOT);
  }

  // campo: { tipo, sentido, eta, cerca, anim, hacia }. Lo cumplen igual un
  // obstaculo y el repulsor de un robot, asi que los dos se dibujan aqui.
  function dibujarCampo(mx, my, radioMundo, campo, color, realce, densidad) {
    const cx = aX(mx), cy = aY(my);
    const R = radioMundo * esc() * (realce || 1);
    const rgb = hexARgb(color);
    const rgba = (a) => "rgba(" + rgb + "," + a.toFixed(3) + ")";

    ctx.save();

    // Degradado y no relleno plano. El plano dibujaba una moneda de borde a
    // borde y se leia como algo solido con lo que se choca; un repulsor no
    // es eso. El degradado enseña la FORMA del campo -fuerte en el centro,
    // apagandose hacia el borde-, que es literalmente lo que hacen las
    // cuatro formulas.
    const relleno = ctx.createRadialGradient(cx, cy, 0, cx, cy, R);
    relleno.addColorStop(0, rgba(0.17 + 0.20 * campo.cerca));
    relleno.addColorStop(0.55, rgba(0.075 + 0.11 * campo.cerca));
    relleno.addColorStop(1, rgba(0));
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, 2 * Math.PI);
    ctx.fillStyle = relleno;
    ctx.fill();

    // Y encima, un foco en el LADO por el que viene el robot. El campo
    // empuja mas fuerte justo ahi -es donde el robot esta metido-, y un
    // degradado concentrico no puede decir eso: se ve igual por los cuatro
    // costados. Recortado al circulo para que no manche fuera del alcance.
    if (campo.cerca > 0.01 && campo.hacia) {
      // La y del mundo va al reves que la del lienzo (aY invierte), asi que
      // el vector se voltea al pasarlo a pantalla.
      const hx = cx + campo.hacia.x * R * 0.55;
      const hy = cy - campo.hacia.y * R * 0.55;
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, 2 * Math.PI);
      ctx.clip();
      const foco = ctx.createRadialGradient(hx, hy, 0, hx, hy, R * 0.85);
      foco.addColorStop(0, rgba(0.34 * campo.cerca));
      foco.addColorStop(1, rgba(0));
      ctx.fillStyle = foco;
      ctx.fillRect(cx - R, cy - R, 2 * R, 2 * R);
      ctx.restore();
    }

    // El borde va punteado, como la zona de llegada del destino y por lo
    // mismo: es un ALCANCE, no una pared. Fuera de el la fuerza es
    // exactamente cero, y eso es todo lo que dice.
    ctx.setLineDash(sobreVideo ? [7, 6] : [5, 5]);
    ctx.lineWidth = (sobreVideo ? 1.8 : 1.1) + 0.9 * campo.cerca + grosorPorEta(campo.eta);
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.5 + 0.5 * campo.cerca;
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.restore();

    dibujarFlechasRepulsor(campo, cx, cy, R, color, densidad || 1);

    // El centro, en solido. Con el degradado apagandose hacia fuera hace
    // falta un punto duro que diga DONDE esta: es la distancia a este punto
    // la que entra en la formula, no a la mancha.
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(cx, cy, Math.max(2.5, R * 0.05), 0, 2 * Math.PI);
    ctx.fill();
    ctx.restore();
  }

  // Subir la intensidad tiene que verse, no solo notarse en como esquiva el
  // robot. Es poco a proposito -medio pixel de punta a punta del riel-:
  // suficiente para leer "este empuja mas que aquel" poniendo dos al lado,
  // y no tanto como para que un campo fuerte se coma el dibujo.
  function grosorPorEta(eta) {
    return 0.55 * Math.min(1, (eta || 0) / ETA_MAX);
  }

  function hexARgb(hex) {
    const n = parseInt(hex.slice(1), 16);
    return ((n >> 16) & 255) + "," + ((n >> 8) & 255) + "," + (n & 255);
  }

  // El camino se dibuja alrededor del ANCLA, no de la posicion: el ancla es
  // lo que se arrastra y lo que no se mueve solo. Sin dibujarlo no hay
  // forma de saber por donde va a pasar el obstaculo dentro de un segundo.
  function dibujarCaminoObstaculo(obs, e, color) {
    ctx.save();
    ctx.setLineDash([4, 7]);
    ctx.lineWidth = 1.2;
    ctx.strokeStyle = sobreVideo ? "rgba(243,244,246,0.85)" : "rgba(55,65,81,0.45)";
    ctx.beginPath();
    if (obs.mov === "recta") {
      const a = obs.angulo * Math.PI / 180, h = obs.largo / 2;
      ctx.moveTo(aX(obs.ancla.x - Math.cos(a) * h), aY(obs.ancla.y - Math.sin(a) * h));
      ctx.lineTo(aX(obs.ancla.x + Math.cos(a) * h), aY(obs.ancla.y + Math.sin(a) * h));
    } else if (obs.mov === "lemniscata") {
      // Se recorre la FASE de 0 a 2*pi y no el tiempo: asi la curva sale
      // igual sea cual sea la velocidad. Mismo criterio que la trayectoria
      // de referencia de un robot.
      const N = 120;
      for (let i = 0; i <= N; i++) {
        const p = trayectoriaLemniscata(2 * Math.PI * i / N, [obs.ancla.x, obs.ancla.y], obs.orbita, 1);
        if (i === 0) ctx.moveTo(aX(p[0]), aY(p[1])); else ctx.lineTo(aX(p[0]), aY(p[1]));
      }
    } else {
      ctx.arc(aX(obs.ancla.x), aY(obs.ancla.y), obs.orbita * e, 0, 2 * Math.PI);
    }
    ctx.stroke();
    // El ancla, como una cruz chica: es el asa del camino y hay que poder
    // verla aunque el obstaculo ande por el otro extremo.
    ctx.setLineDash([]);
    const ax = aX(obs.ancla.x), ay = aY(obs.ancla.y);
    ctx.beginPath();
    ctx.moveTo(ax - 5, ay); ctx.lineTo(ax + 5, ay);
    ctx.moveTo(ax, ay - 5); ctx.lineTo(ax, ay + 5);
    ctx.stroke();
    ctx.restore();
  }

  function dibujarFlechasRepulsor(campo, cx, cy, R, color, densidad) {
    const swirl = campo.tipo === "espiral";
    const largoMax = LARGO_FLECHA * R;
    if (largoMax < 2.2) return;   // circulo diminuto: las flechas serian ruido
    // Lo que separa a "espiral" de las otras tres de un vistazo: las demas
    // empujan derecho hacia afuera, esta ademas arrastra de costado. Y de
    // que costado lo dice el sentido, que es un mando del panel.
    // OJO CON EL SIGNO. La fisica gira el vector en coordenadas del MUNDO,
    // donde +y va hacia arriba; aqui se dibuja en coordenadas del LIENZO,
    // donde +y va hacia abajo. Una direccion del mundo a angulo t sale en
    // pantalla a angulo -t, asi que un giro de +g alla se ve como -g aqui.
    //
    // Sin este menos, las flechas apuntaban hacia un lado y el robot se iba
    // hacia el otro: el dibujo decia justo lo contrario de lo que pasaba.
    const sentido = -(campo.sentido < 0 ? -1 : 1);
    const giro = swirl ? GIRO_ESPIRAL * sentido : 0;
    const grosor = 1.05 + 1.1 * campo.cerca + grosorPorEta(campo.eta);

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.lineWidth = grosor;

    // Cada corona dibuja su flecha del largo que le toca por la FUERZA QUE
    // HAY AHI, no por un numero elegido a mano por tipo. Antes las cuatro
    // formulas se distinguian solo por un largo fijo y en el mapa se veian
    // casi iguales; ahora la forma del campo se lee del dibujo:
    //
    //   lineal        tres coronas iguales y largas  -> muro parejo
    //   lineal_norm   se van acortando hacia fuera   -> rampa
    //   espiral       igual que la rampa, torcidas   -> rodea
    //   khatib        larga dentro, muñon fuera      -> solo de cerca
    CORONAS.forEach((corona, k) => {
      // La onda sale del centro y recorre las tres coronas: cada una va un
      // tercio de vuelta por detras de la de dentro. Quieto -sin nadie a
      // quien empujar- anim no avanza, las tres se quedan en su radio y el
      // dibujo dice solo de que tipo es el campo.
      const fase = (((campo.anim - k / CORONAS.length) % 1) + 1) % 1;
      const onda = Math.cos(2 * Math.PI * fase);      // -1..1
      const rad = R * (corona.radio + 0.035 * onda);
      // El brillo tambien viaja: la corona por la que pasa la onda se
      // enciende, y solo si hay alguien cerca. Sin nadie, alfa fija.
      ctx.globalAlpha = 0.38 + 0.62 * campo.cerca * (0.5 + 0.5 * onda);
      // Media separacion de giro respecto a la corona de dentro.
      // La misma cuenta que usa la fisica, con el mismo recorte: dividida
      // entre eta da la fraccion de fuerza que hay a ese radio, 0..1.
      const fuerza = campo.eta > 0
        ? magnitudCampo(corona.radio * campo.radio, campo.radio, campo.eta, campo.tipo) / campo.eta
        : 0;
      // Sin pasarse del borde: se mide contra el radio EN REPOSO de la
      // corona, no contra el que tenga ahora mismo, para que la flecha no
      // encoja y crezca al ritmo de la animacion.
      const hueco = R * (1 - corona.radio);
      const largo = Math.min(largoMax * (LARGO_MINIMO + (1 - LARGO_MINIMO) * fuerza), hueco);
      const ala = largo * 0.44;
      const n = Math.max(3, Math.round(corona.n * densidad));
      const desfase = (k % 2) * Math.PI / n;
      for (let i = 0; i < n; i++) {
        const a = i * 2 * Math.PI / n + desfase +
                  (swirl ? campo.anim * 1.4 * sentido : 0);
        ctx.save();
        ctx.translate(cx + rad * Math.cos(a), cy + rad * Math.sin(a));
        ctx.rotate(a + giro);
        ctx.beginPath();
        ctx.moveTo(0, 0); ctx.lineTo(largo, 0);
        ctx.moveTo(largo - ala, -ala * 0.75);
        ctx.lineTo(largo, 0);
        ctx.lineTo(largo - ala, ala * 0.75);
        ctx.stroke();
        ctx.restore();
      }
    });
    ctx.restore();
  }

  // Triangulo objetivo de la formacion, punteado y gris. Va DESPUES de los
  // robots en el orden de dibujo: es la referencia que se esta siguiendo y
  // tiene que leerse por encima de los sprites, no quedar tapada.
  function dibujarFormacion(vertices) {
    ctx.save();
    ctx.setLineDash([6, 6]);
    ctx.lineDashOffset = 0;
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "#6B6F76";
    ctx.beginPath();
    vertices.forEach((v, i) => {
      const px = aX(v[0]), py = aY(v[1]);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.stroke();

    ctx.setLineDash([]);
    for (const v of vertices) {
      ctx.beginPath();
      ctx.arc(aX(v[0]), aY(v[1]), 3.5, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.restore();
  }

  // Trayectoria ejecutada: por donde REALMENTE paso. Mismo color que la
  // referencia pero mas saturado y en linea continua, para distinguir de
  // un vistazo lo pedido de lo logrado. Solo las ultimas MUESTRAS_RASTRO
  // posiciones (la cola se recorta en main.js).
  function dibujarRastro(robot) {
    if (robot.rastro.length < 2) return;
    ctx.save();
    ctx.strokeStyle = robot.color + "C0";
    ctx.lineWidth = 2.2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(aX(robot.rastro[0].x), aY(robot.rastro[0].y));
    for (const p of robot.rastro) ctx.lineTo(aX(p.x), aY(p.y));
    ctx.stroke();
    ctx.restore();
  }

  function dibujarRobot(robot, img, realce) {
    const e = esc();
    ctx.save();
    ctx.translate(aX(robot.pose.x), aY(robot.pose.y));
    ctx.rotate(-robot.pose.th);
    // Escala el marco entero, asi el chasis y su marcador negro crecen
    // juntos: son una sola pieza para quien arrastra.
    if (realce && realce !== 1) ctx.scale(realce, realce);

    const S = (CFG.ancho_robot_px || 250) * e / 2;
    const b = S * TAM_MARCA;

    // img puede ser un <img> o el canvas ya reducido de main.js, por eso
    // se leen las medidas con el fallback en vez de naturalWidth a secas.
    const iw = chasisVisible && img ? (img.naturalWidth || img.width) : 0;
    const ih = chasisVisible && img ? (img.naturalHeight || img.height) : 0;

    if (iw && ih) {
      const w = S * 2 * 1.35 * (robot.escalaImg || 1);
      const h = w * (ih / iw);
      const rot = robot.spriteOffset || 0;

      ctx.imageSmoothingEnabled = true;
      ctx.imageSmoothingQuality = "high";
      ctx.save();
      // Alinea el frente visual del PNG con el eje +x local, que es hacia
      // donde lo mueve la fisica. Sin esto el chasis apunta a un lado
      // mientras el robot avanza hacia otro.
      if (rot) ctx.rotate(-rot);
      ctx.drawImage(img, -w / 2, -h / 2, w, h);
      ctx.restore();
    } else if (chasisVisible) {
      // Respaldo por si el PNG no cargo. Sin chasis no hay respaldo que
      // valga: ahi la ausencia de dibujo es lo pedido, no un fallo.
      ctx.fillStyle = "#E5E7EB";
      ctx.strokeStyle = "#D1D5DB";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(-S, -S, S * 2, S * 2, S * 0.22);
      ctx.fill(); ctx.stroke();
    }

    // Marcador real: un solo glifo negro que dice a la vez donde esta el
    // centro del robot y hacia donde mira. Sustituye al rombo de centro y
    // al punto de frente, que decian lo mismo en dos piezas sueltas.
    // Va dentro del marco ya rotado por -th, asi que apunta al eje +x
    // local, que es hacia donde lo mueve la fisica.
    if (robot.marcaReal) {
      ctx.imageSmoothingEnabled = true;
      ctx.imageSmoothingQuality = "high";
      ctx.drawImage(robot.marcaReal, -b / 2, -b / 2, b, b);
    }

    ctx.restore();

    // Etiqueta con el nombre, colgada de lo que se acabe de dibujar: del
    // chasis si lo hay, y del rombo si no. Medirla siempre contra el chasis
    // la dejaria flotando lejos del unico dibujo que queda sobre el video,
    // y con tres robots cerca no se sabria de quien es cada nombre.
    ctx.fillStyle = robot.color;
    ctx.font = "600 11px 'IBM Plex Mono', monospace";
    ctx.textAlign = "center";
    const margen = (chasisVisible ? S : b / 2) + 8;
    if (!chasisVisible) {
      // Sobre el video el fondo es una imagen cualquiera: sin un contorno
      // detras, el nombre desaparece en cuanto cae sobre algo claro.
      ctx.lineWidth = 3;
      ctx.strokeStyle = "rgba(15,16,19,0.85)";
      ctx.lineJoin = "round";
      ctx.strokeText(robot.nombre, aX(robot.pose.x), aY(robot.pose.y) - margen);
    }
    ctx.fillText(robot.nombre, aX(robot.pose.x), aY(robot.pose.y) - margen);
  }

  function limpiar() {
    ctx.clearRect(0, 0, lienzo.width, lienzo.height);
  }

  return {
    init, usarLienzo, mostrarChasis, actualizarCFG, ajustarLienzo, aMundo, aX, aY, esc,
    reticula, dibujarReferencia, dibujarCentroTrayectoria, dibujarFormacion,
    dibujarDestino, dibujarRastro, dibujarRobot,
    dibujarObstaculo, dibujarRepulsorRobot, limpiar,
    // Para escena3d.js: la reticula 3D sale con los mismos pasos y colores.
    nivelesGrid: NIVELES_GRID, colorEje: COLOR_EJE,
  };
})();
