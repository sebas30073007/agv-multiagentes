// formacion.js - el editor de la figura: la rejilla de puntos y la brujula.
//
// Es el unico sitio de la pagina donde se DIBUJA la formacion en pequeño y
// se puede tocar con el raton. No corre control ni fisica: escribe sobre
// estado.formacion (puntos y angulo) y repinta.
//
// Por que una rejilla y no coordenadas a mano: la figura es una relacion
// entre tres robots -en fila, en cuña, en columna- y eso se dice mucho
// mejor colocandolos que tecleando seis numeros. Los puntos se enganchan a
// la rejilla para que las figuras salgan limpias sin pelear con el raton.
//
// Por que la brujula aparte y no girar arrastrando en la rejilla: son dos
// decisiones distintas -que forma tiene / hacia donde mira-, y mezcladas
// obligarian a recolocar los tres puntos cada vez que quisieras girarla.
"use strict";

const Formacion = (function () {
  // Media rejilla a cada lado del centro. 5 da 11x11 puntos, que es lo que
  // cabe legible en la columna sin que el punto se vuelva una mota.
  const RADIO_REJILLA = 5;
  const COLOR_PUNTO = "#D1D5DB";
  const COLOR_LINEA = "#9CA3AF";

  let cvRejilla, ctxR, cvBrujula, ctxB, estado, alCambiar;
  // Lo que se esta arrastrando: el indice de un robot en la rejilla, o
  // "brujula". null = nada.
  let agarrado = null;

  function init(est, onCambio) {
    estado = est;
    alCambiar = onCambio || function () {};
    cvRejilla = document.getElementById("lienzoFormacion");
    cvBrujula = document.getElementById("lienzoBrujula");
    if (!cvRejilla || !cvBrujula) return;
    ctxR = cvRejilla.getContext("2d");
    ctxB = cvBrujula.getContext("2d");

    cvRejilla.addEventListener("mousedown", presionarRejilla);
    cvBrujula.addEventListener("mousedown", presionarBrujula);
    window.addEventListener("mousemove", mover);
    window.addEventListener("mouseup", () => { agarrado = null; });
    // Tactil: el mismo gesto, y sin el la pagina hace scroll en vez de
    // mover el punto.
    cvRejilla.addEventListener("touchstart", presionarRejilla, { passive: false });
    cvBrujula.addEventListener("touchstart", presionarBrujula, { passive: false });
    window.addEventListener("touchmove", mover, { passive: false });
    window.addEventListener("touchend", () => { agarrado = null; });

    pintar();
  }

  // --- geometria de la rejilla -------------------------------------
  // El lienzo se declara al doble de su tamaño en pantalla (ver el CSS):
  // asi las lineas y los puntos salen nitidos en pantallas densas. Todo lo
  // de aqui trabaja en pixeles del LIENZO y el raton se convierte al entrar.
  function pasoRejilla() {
    return Math.min(cvRejilla.width, cvRejilla.height) / (2 * RADIO_REJILLA + 2);
  }
  function aPantalla(gx, gy) {
    const p = pasoRejilla();
    // La y de la rejilla se voltea: en el mundo +y es arriba, en el lienzo
    // es abajo. Sin esto, arrastrar un robot hacia arriba lo mandaria hacia
    // abajo en la cancha.
    return [cvRejilla.width / 2 + gx * p, cvRejilla.height / 2 - gy * p];
  }
  function aRejilla(px, py) {
    const p = pasoRejilla();
    return [(px - cvRejilla.width / 2) / p, -(py - cvRejilla.height / 2) / p];
  }
  function deEvento(ev, cv) {
    const t = ev.touches ? ev.touches[0] : ev;
    const r = cv.getBoundingClientRect();
    // De pixeles de pantalla a pixeles del lienzo.
    return [(t.clientX - r.left) * cv.width / r.width,
            (t.clientY - r.top) * cv.height / r.height];
  }

  // --- raton --------------------------------------------------------
  function presionarRejilla(ev) {
    ev.preventDefault();
    const [px, py] = deEvento(ev, cvRejilla);
    const p = pasoRejilla();
    let mejor = null, mejorD = p * 1.1;   // no se agarra desde lejos
    estado.formacion.puntos.forEach((g, i) => {
      const [x, y] = aPantalla(g[0], g[1]);
      const d = Math.hypot(px - x, py - y);
      if (d < mejorD) { mejorD = d; mejor = i; }
    });
    if (mejor !== null) { agarrado = mejor; mover(ev); }
  }

  function presionarBrujula(ev) {
    ev.preventDefault();
    agarrado = "brujula";
    mover(ev);
  }

  function mover(ev) {
    if (agarrado === null) return;
    ev.preventDefault();

    if (agarrado === "brujula") {
      const [px, py] = deEvento(ev, cvBrujula);
      const dx = px - cvBrujula.width / 2;
      const dy = cvBrujula.height / 2 - py;   // otra vez, la y al reves
      if (Math.hypot(dx, dy) > 4) estado.formacion.angulo = Math.atan2(dy, dx);
      pintar();
      alCambiar();
      return;
    }

    const [px, py] = deEvento(ev, cvRejilla);
    let [gx, gy] = aRejilla(px, py);
    // Enganche a la rejilla, y recorte para que no se salga del cuadro.
    gx = Math.max(-RADIO_REJILLA, Math.min(RADIO_REJILLA, Math.round(gx)));
    gy = Math.max(-RADIO_REJILLA, Math.min(RADIO_REJILLA, Math.round(gy)));
    // Dos robots en el mismo punto no es una figura: la ley les pediria
    // estar en el mismo sitio y se empujarian eternamente. Se ignora el
    // movimiento en vez de aceptarlo y que parezca que se comio uno.
    const chocado = estado.formacion.puntos.some(
      (g, i) => i !== agarrado && g[0] === gx && g[1] === gy);
    if (chocado) return;

    estado.formacion.puntos[agarrado] = [gx, gy];
    pintar();
    alCambiar();
  }

  // --- dibujo -------------------------------------------------------
  function pintar() {
    if (!ctxR) return;
    pintarRejilla();
    pintarBrujula();
    pintarNombre();
  }

  function pintarRejilla() {
    const p = pasoRejilla();
    ctxR.clearRect(0, 0, cvRejilla.width, cvRejilla.height);

    ctxR.fillStyle = COLOR_PUNTO;
    for (let gx = -RADIO_REJILLA; gx <= RADIO_REJILLA; gx++) {
      for (let gy = -RADIO_REJILLA; gy <= RADIO_REJILLA; gy++) {
        const [x, y] = aPantalla(gx, gy);
        ctxR.beginPath();
        ctxR.arc(x, y, p * 0.13, 0, 2 * Math.PI);
        ctxR.fill();
      }
    }

    // La figura se dibuja SIN GIRAR. La rejilla es la figura y la brujula
    // es la orientacion: dos mandos, uno al lado del otro, y ninguno pisa
    // al otro. Girandola aqui, los puntos se salian de los nodos de la
    // rejilla y el editor parecia roto justo mientras lo estabas usando.
    //
    // El efecto compuesto -figura girada- se ve donde importa, que es la
    // cancha: ahi el triangulo objetivo sale ya orientado.
    const puestos = puntosCentrados();

    ctxR.save();
    ctxR.setLineDash([6, 6]);
    ctxR.lineWidth = 2.5;
    ctxR.strokeStyle = COLOR_LINEA;
    ctxR.beginPath();
    puestos.forEach((g, i) => {
      const [x, y] = aPantalla(g[0], g[1]);
      if (i === 0) ctxR.moveTo(x, y); else ctxR.lineTo(x, y);
    });
    ctxR.closePath();
    ctxR.stroke();
    ctxR.restore();

    // Cada robot en su color: es como se reconocen en la cancha, y aqui
    // hace falta saber cual de los tres estas moviendo.
    puestos.forEach((g, i) => {
      const [x, y] = aPantalla(g[0], g[1]);
      const robot = estado.robots[i];
      ctxR.beginPath();
      ctxR.arc(x, y, p * 0.42, 0, 2 * Math.PI);
      ctxR.fillStyle = robot ? robot.color : "#14161A";
      ctxR.fill();
      if (agarrado === i) {
        ctxR.lineWidth = 2.5;
        ctxR.strokeStyle = "#14161A";
        ctxR.stroke();
      }
    });
  }

  // Los puntos tal cual se agarran. Sin girar y sin escalar: aqui se edita
  // la FIGURA, y el angulo y el tamaño son de los otros dos mandos.
  function puntosCentrados() {
    return estado.formacion.puntos;
  }

  function pintarBrujula() {
    const w = cvBrujula.width, h = cvBrujula.height;
    const R = Math.min(w, h) / 2 - 4;
    ctxB.clearRect(0, 0, w, h);

    ctxB.beginPath();
    ctxB.arc(w / 2, h / 2, R, 0, 2 * Math.PI);
    ctxB.fillStyle = "#E5E7EB";
    ctxB.fill();

    ctxB.save();
    ctxB.translate(w / 2, h / 2);
    // El menos: el angulo esta en coordenadas del mundo y aqui la y va al
    // reves. Es el mismo cambio de signo que en canvas.js.
    ctxB.rotate(-estado.formacion.angulo);
    ctxB.fillStyle = "#14161A";
    ctxB.beginPath();
    ctxB.moveTo(R * 0.78, 0);
    ctxB.lineTo(-R * 0.30, -R * 0.34);
    ctxB.lineTo(-R * 0.10, 0);
    ctxB.lineTo(-R * 0.30, R * 0.34);
    ctxB.closePath();
    ctxB.fill();
    ctxB.restore();
  }

  // Un nombre para las figuras que se reconocen. No cambia nada: es para
  // poder decir "ponlo en fila" y saber que ya lo esta.
  function pintarNombre() {
    const rotulo = document.getElementById("figuraNombre");
    if (!rotulo) return;
    const P = estado.formacion.puntos;
    const grados = Math.round(estado.formacion.angulo * 180 / Math.PI);
    rotulo.textContent = nombreFigura(P) + " · " + grados + "°";
  }

  function nombreFigura(P) {
    const n = P.length;
    // Colineales: los tres en una recta. El area del triangulo es cero.
    const area = Math.abs(
      (P[1][0] - P[0][0]) * (P[2][1] - P[0][1]) -
      (P[2][0] - P[0][0]) * (P[1][1] - P[0][1]));
    if (area < 1e-6) return "En línea";
    const d = [];
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) d.push(Math.hypot(P[i][0] - P[j][0], P[i][1] - P[j][1]));
    }
    // Se comparan en RELATIVO y no en absoluto: la rejilla no puede dar un
    // equilatero exacto -no cabe en enteros-, asi que el triangulo de
    // arranque tiene lados 3.61, 3.61 y 4. En absoluto eso no pasaba por
    // equilatero por 0.39 y se rotulaba "libre", que es justo lo que no es.
    d.sort((x, y) => x - y);
    const rel = (a, b) => Math.abs(a - b) / Math.max(a, b, 1e-6);
    if (rel(d[2], d[0]) < 0.15) return "Triángulo";
    // Dos lados parecidos y uno distinto: la punta va delante o detras.
    if (rel(d[0], d[1]) < 0.15 || rel(d[1], d[2]) < 0.15) return "Cuña";
    return "Triángulo libre";
  }

  function reiniciar() {
    estado.formacion.puntos = [[0, 2], [-2, -1], [2, -1]];
    // El angulo tambien: "reiniciar formacion" es dejarla como estaba al
    // abrir la pagina, no solo enderezar los puntos.

    estado.formacion.angulo = 0;
    pintar();
    alCambiar();
  }

  return { init, pintar, reiniciar };
})();
