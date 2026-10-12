// control.js - port a JS de Atlas_Control/control.py y Atlas_Control/formacion.py.
//
// Atlas sigue calculando su ley de control en Python, en el servidor (via
// /ciclo) - eso no cambia. Apolo y Odiseo son uniciclo y no tienen
// backend propio, asi que corren la MISMA matematica de control.py, pero
// aqui en el navegador, en el mismo dominio de PIXELES que ya usa todo el
// canvas (ver la nota de unidades en control.py: "EN PIXELES (medidos
// sobre la imagen de la camara, no con flexometro)" - por eso las
// constantes de abajo son las mismas que las de ese archivo, no hace falta
// convertir nada).
"use strict";

const envolver = a => Math.atan2(Math.sin(a), Math.cos(a));

// =========================================================
// PUNTO: control proporcional + transformacion de uniciclo
// (control.py: convergencia(), uniciclo(), diferencial())
// =========================================================
const K_CONVERGENCIA = 0.8;
const L_PUNTO = 25.0;          // punto virtual, px. No puede ser 0 (ver control.py).
const RADIO_LLANTA = 58.0;     // px
const ENTRE_LLANTAS = 120.0;   // px
const W_MAXIMA = 150.0;        // saturacion de rueda
const TOLERANCIA_UNICICLO = 15.0; // px, radio de "ya llego"

// Velocidad de rueda que representa un PWM de 255 en el modo Manual.
//
// NO es W_MAXIMA. Esa es la saturacion del lazo: 150 rad/s, unas 1430 rpm,
// un numero que ninguno de estos robots alcanza ni de lejos. Usarla como
// fondo de escala hacia que teclear 40 en la caja de Manual moviera al
// robot simulado a 1364 px/s, cruzando la cancha entera en menos de un
// segundo mientras el robot de verdad apenas se arrastraba.
//
// Se ancla a la velocidad util: PWM 255 = V_MAX_PX. Asi la simulacion se
// parece al robot fisico, donde 255 tambien es el tope (255 -> ±
// MAX_STEPS_PER_SEC en esp32c3_odiseo_serial.py).
const V_MAX_PX = 250.0;                          // igual que CFG.v_max_px
const W_RUEDA_PWM255 = V_MAX_PX / RADIO_LLANTA;  // ~4.3 rad/s

function convergencia(x, y, xd, yd, k = K_CONVERGENCIA) {
  return [-k * (x - xd), -k * (y - yd)];
}

function uniciclo(ux, uy, theta, l = L_PUNTO) {
  // Inversa cerrada de la matriz [[cos,-l sin],[sin, l cos]] (det = l).
  const c = Math.cos(theta), s = Math.sin(theta);
  const V = c * ux + s * uy;
  const W = (-s * ux + c * uy) / l;
  return [V, W];
}

function puntoVirtual(x, y, theta, l = L_PUNTO) {
  return [x + l * Math.cos(theta), y + l * Math.sin(theta)];
}

// =========================================================
// RUMBO: control P sobre el error angular
// (equivalente al giro que calcula servidor_sandbox.py para Atlas, y al
// ctrl_angulo.py del repo de RoboMaster)
// =========================================================
const K_RUMBO = 1.2;            // ganancia P sobre el error de rumbo
const W_MAX_RUMBO = 2.5;        // rad/s, saturacion
const TOLERANCIA_RUMBO = 0.09;  // rad (~5 grados): dentro de esto, quieto

// Devuelve la velocidad angular para llevar th hacia thd. El error se
// envuelve a (-pi, pi] para que gire por el lado corto en vez de dar la
// vuelta larga cuando el error cruza +-180 grados.
// La tolerancia entra por parametro: es lo que el panel deja mover en Punto
// (slider "Zona de rumbo"), y TOLERANCIA_RUMBO se queda solo de valor por
// defecto para quien llame sin decir nada.
function controlRumbo(th, thd, k = K_RUMBO, wMax = W_MAX_RUMBO, tol = TOLERANCIA_RUMBO) {
  const err = envolver(th - thd);
  if (Math.abs(err) < tol) return 0;
  const w = -k * err;
  return w > wMax ? wMax : (w < -wMax ? -wMax : w);
}

function diferencial(V, W, r = RADIO_LLANTA, L = ENTRE_LLANTAS, wMax = W_MAXIMA) {
  let wDer = V / r + (L * W) / (2 * r);
  let wIzq = V / r - (L * W) / (2 * r);
  const pico = Math.max(Math.abs(wDer), Math.abs(wIzq));
  if (pico > wMax) {
    const esc = wMax / pico;
    wDer *= esc; wIzq *= esc;
  }
  return [wDer, wIzq];
}

// =========================================================
// HOLONOMICO (Atlas usa el equivalente en Python; se deja aqui solo por
// si algun dia se simula Atlas tambien en el cliente).
// =========================================================
function holonomico(ux, uy, theta) {
  const c = Math.cos(theta), s = Math.sin(theta);
  return [ux * c + uy * s, -ux * s + uy * c]; // [adelante, izquierda]
}

// =========================================================
// REPULSORES (formacion.py: FORMULAS_REPULSOR, fuerza_repulsiva)
// Las 4 formulas comparten contrato: cero en d>=r, alejan del repulsor.
// =========================================================
const FORMULAS_REPULSOR = {
  lineal: (d, r, eta) => eta * (r - d),
  lineal_norm: (d, r, eta) => eta * (1 - d / r),
  espiral: (d, r, eta) => eta * (1 - Math.pow(r, 4) / d), // asume r < 1 unidad "natural"; en px usar r chico
  khatib: (d, r, eta) => (2 * eta / Math.pow(d, 4)) * (1 / (d * d) - 1 / (r * r)),
};

// Cuanto empuja cada formula A ESCALA DE PIXELES, ya recortada.
//
// FORMULAS_REPULSOR de arriba es la traduccion literal de formacion.py y se
// queda como esta: es la referencia cruzada con el codigo del profesor. Pero
// dos de las cuatro estan escritas para radios menores que 1 y en pixeles no
// sirven tal cual. Medido con r=120 y eta=1000:
//
//   lineal        102000 en el centro   -> se recorta a eta en todo el radio
//   lineal_norm   850                   -> util tal cual
//   espiral       -1.2e10 (NEGATIVA)    -> atraeria; se recorta a cero y no hace nada
//   khatib        0.0001                -> haria falta eta = 3.1e10 para que se note
//
// Asi que aqui viven las versiones que si funcionan en pixeles. Las dos que
// se tocan -espiral y khatib- conservan su CARACTER, que es lo que las
// distingue; lo que cambia son las unidades.
//
// Y todas se recortan a eta. El invariante se explica en una linea -ningun
// campo empuja mas fuerte que el numero que le pusiste- y hace que el riel
// signifique lo mismo con las cuatro.
function magnitudCampo(d, r, eta, tipo) {
  if (eta <= 0 || r <= 0) return 0;
  // Fraccion del radio, 0 en el centro y 1 en el borde.
  const u = Math.min(1, Math.max(1e-3, d / r));
  let m;
  switch (tipo) {
    // Rampa recta, pero multiplicada por (r-d) en PIXELES: con cualquier
    // eta usable satura en todo el circulo. O sea, un muro de fuerza
    // constante que se corta en seco en el borde.
    //
    // Ojo: lineal y lineal_norm son LA MISMA CURVA. lineal = lineal_norm * r,
    // exacto, a cualquier distancia. Lo unico que las separa es ese factor,
    // y con el recorte a eta se traduce en "muro" contra "rampa".
    case "lineal":
      m = eta * (r - d);
      break;

    // Rampa recta de eta en el centro a cero en el borde. Es la unica de las
    // cuatro que responde al riel en todo su recorrido, y por eso es la de
    // arranque.
    case "lineal_norm":
      m = eta * (1 - u);
      break;

    // Espiral: la MAGNITUD es la del normalizado; lo que la hace espiral es
    // que quien llama tuerce la direccion GIRO_ESPIRAL grados. Ver la nota
    // en config.js.
    case "espiral":
      m = eta * (1 - u);
      break;

    // Khatib en unidades naturales: (1/u - 1)^2, que es el gradiente del
    // potencial clasico con la distancia medida en fracciones del radio en
    // vez de en pixeles.
    //
    // Su caracter es "no existo hasta que existo mucho": a 90% del radio
    // empuja al 1%, a 70% al 18%, y de la mitad hacia dentro esta saturado.
    // Comparada con la rampa del normalizado -que a 70% ya va al 30%-, esta
    // te deja acercarte mucho mas antes de plantarse.
    case "khatib":
      m = eta * Math.pow(1 / u - 1, 2);
      break;

    default:
      m = eta * (1 - u);
  }
  // Suelo en cero: un repulsor que atrae es una trampa, no un repulsor.
  return Math.min(Math.max(m, 0), eta);
}

function fuerzaRepulsiva(dx, dy, eta, r, tipo) {
  const d = Math.hypot(dx, dy);
  if (d >= r || d < 1e-6) return [0, 0];
  const formula = FORMULAS_REPULSOR[tipo] || FORMULAS_REPULSOR.lineal;
  const mag = formula(d, r, eta);
  return [mag * dx / d, mag * dy / d];
}

// posiciones: array de [x,y]. Devuelve la fuerza repulsiva neta sobre cada
// una, considerando a TODAS las demas como repulsores (grafo completo
// implicito: no hace falta matriz de adyacencia para esto).
//
// YA NO SE USA EN LA PAGINA. La repulsion entre robots la hace ahora el
// campo propio de cada uno (fuerzaDeCampo en main.js), que ademas vale en
// los cuatro modos y no solo en Formacion. Se conserva porque es la
// traduccion directa de formacion.py y este archivo es la referencia
// cruzada con el: borrarla dejaria el paralelismo cojo.
function repulsionEntreRobots(posiciones, eta, r, tipo) {
  const n = posiciones.length;
  const R = posiciones.map(() => [0, 0]);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      if (i === j) continue;
      const [fx, fy] = fuerzaRepulsiva(
        posiciones[i][0] - posiciones[j][0],
        posiciones[i][1] - posiciones[j][1],
        eta, r, tipo);
      R[i][0] += fx; R[i][1] += fy;
    }
  }
  return R;
}

// =========================================================
// FORMACION POR GRAFO COMPLETO (formacion.py: grafo_completo, control_formacion)
// =========================================================
function grafoCompleto(n) {
  const A = [];
  for (let i = 0; i < n; i++) {
    A.push([]);
    for (let j = 0; j < n; j++) A[i].push(i === j ? 0 : 1);
  }
  return A;
}

// ui = -k * SUM_j A[i,j] * ((Zi-Zj) - (Zdi-Zdj)), normalizado por grado.
// Ver formacion.py para la explicacion completa de por que normalizar.
function controlFormacion(Z, Zd, A, k, normalizar = true) {
  const n = Z.length;
  const U = Z.map(() => [0, 0]);
  for (let i = 0; i < n; i++) {
    let grado = 0;
    for (let j = 0; j < n; j++) {
      if (i === j || !A[i][j]) continue;
      U[i][0] -= k * ((Z[i][0] - Z[j][0]) - (Zd[i][0] - Zd[j][0]));
      U[i][1] -= k * ((Z[i][1] - Z[j][1]) - (Zd[i][1] - Zd[j][1]));
      grado++;
    }
    if (normalizar && grado > 0) { U[i][0] /= grado; U[i][1] /= grado; }
  }
  return U;
}

// =========================================================
// TRAYECTORIAS (referencia: _vendor/.../3MarAngRepBer.py, sin el termino
// de velocidad de avance - aqui solo se usa como punto movil que el P
// persigue, no como feed-forward).
// =========================================================
function trayectoriaCirculo(t, centro, radio, omega) {
  return [centro[0] + radio * Math.cos(omega * t), centro[1] + radio * Math.sin(omega * t)];
}

function trayectoriaLemniscata(t, centro, a, omega) {
  const th = omega * t, s = Math.sin(th), c = Math.cos(th), denom = 1 + s * s;
  const raiz2 = Math.SQRT2;
  return [centro[0] + a * raiz2 * c / denom, centro[1] + a * raiz2 * c * s / denom];
}
