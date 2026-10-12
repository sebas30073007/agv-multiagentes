// graficas.js - telemetria como series de tiempo en vez de numeros.
//
// Cuatro graficas por robot: posicion, velocidad, señal de control y
// error. Se muestrea a PERIODO_MUESTREO_S y se guarda VENTANA_BUFFER_S de
// historial, pero solo se dibujan los ultimos VENTANA_VISIBLE_S: la cola
// reciente es donde se lee la dinamica del lazo (si oscila, si sobrepasa,
// si se queda corto), el historico completo solo aplana todo.
//
// El eje de tiempo es el reloj de la SIMULACION (estado.tiempo), no el del
// navegador: en pausa las graficas se congelan en vez de seguir corriendo
// y dejar un hueco plano.
//
// Idea tomada de los scripts de ploteo del repo de RoboMaster
// (_vendor/.../robofuncs.py: start_plotting_thread), donde el historial se
// mantenia con una cola acotada y se redibujaba en un hilo aparte.
"use strict";

const Graficas = (function () {
  // Series, no robots: azul, verde azulado, violeta y ambar. No reusan el
  // color de ningun robot, que en la tira ya dice de quien son las graficas.
  const PALETA = ["#2563EB", "#0D9488", "#7C3AED", "#B45309"];

  let contenedor = null;
  let lienzos = {};    // id de grafica -> canvas
  let specActual = null;
  let tipoConstruido = null;
  let cfg = null;      // lo que sirve /config, para los nombres de rueda

  function actualizarCFG(nuevo) {
    cfg = nuevo;
    // El orden puede llegar despues de construir las graficas: si ya
    // habia unas dibujadas, hay que rehacerlas o se quedan con los
    // nombres de reserva.
    specActual = null; tipoConstruido = null;
  }

  function nombresRuedas() {
    if (cfg && cfg.orden && cfg.orden.length === 4) return cfg.orden.map(o => o.rueda);
    return ["R0", "R1", "R2", "R3"];
  }

  // Que series lleva cada grafica. Depende del tipo de robot: Atlas es
  // holonomico (Vx, Vy, omega y 4 ruedas mecanum), Apolo y Odiseo son uniciclo
  // (V, omega y 2 ruedas).
  function especGraficas(r) {
    const holon = r.tipo === "holonomico";
    return [
      { id: "pos", titulo: "Posición", etiquetas: ["X", "Y", "θ"] },
      { id: "vel", titulo: "Velocidad", etiquetas: holon ? ["Vx", "Vy", "ω"] : ["V", "ω"] },
      // Las ruedas NO se pueden quemar aqui: el orden real de la trama se
      // mide en el robot y vive en FRAME_LAYOUT (cinematica.py), que llega
      // por /config. Estaban puestas como RR,FR,FL,RL, que resulto ser el
      // orden equivocado y hacia mentir la grafica.
      { id: "ctrl", titulo: "Señal de control",
        etiquetas: holon ? nombresRuedas() : ["D", "I"] },
      { id: "err", titulo: "Error", etiquetas: ["eX", "eY", "eθ"] },
    ];
  }

  function leerValores(r) {
    const holon = r.tipo === "holonomico";
    const v = r.velocidad;
    const grados = a => a * 180 / Math.PI;
    return {
      pos: [r.pose.x, r.pose.y, grados(r.pose.th)],
      vel: holon ? [v.vx, v.vy, v.w] : [v.v, v.w],
      ctrl: r.pwm.slice(),
      err: [
        r.destino.x - r.pose.x,
        r.destino.y - r.pose.y,
        grados(envolver(r.destino.th - r.pose.th)),
      ],
    };
  }

  // Una muestra por robot cada PERIODO_MUESTREO_S. Se muestrea a TODOS,
  // no solo al seleccionado, para que al cambiar de robot su historial ya
  // este ahi en vez de empezar en blanco.
  function muestrear(r, t) {
    if (!r.historial) r.historial = [];
    const h = r.historial;
    const ultimo = h[h.length - 1];
    if (ultimo && t - ultimo.t < PERIODO_MUESTREO_S) return;
    h.push({ t: t, v: leerValores(r) });
    while (h.length && t - h[0].t > VENTANA_BUFFER_S) h.shift();
  }

  function construir(robot) {
    if (!contenedor) contenedor = document.getElementById("graficas");
    specActual = especGraficas(robot);
    tipoConstruido = robot.tipo;
    lienzos = {};
    contenedor.innerHTML = "";

    specActual.forEach(g => {
      const bloque = document.createElement("div");
      bloque.className = "grafica";
      const leyenda = g.etiquetas
        .map((et, i) => '<span><i style="background:' + PALETA[i % PALETA.length] + '"></i>' + et + '</span>')
        .join("");
      bloque.innerHTML =
        '<div class="grafica-head"><h3>' + g.titulo + '</h3>' +
        '<div class="grafica-leyenda">' + leyenda + '</div></div>' +
        '<canvas></canvas>';
      contenedor.appendChild(bloque);
      lienzos[g.id] = bloque.querySelector("canvas");
    });
  }

  function formato(v) {
    const a = Math.abs(v);
    if (a >= 100) return v.toFixed(0);
    if (a >= 10) return v.toFixed(1);
    return v.toFixed(2);
  }

  function dibujarUna(cv, g, historial, tNow) {
    const ctx = cv.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return;
    if (cv.width !== Math.round(w * dpr)) {
      cv.width = Math.round(w * dpr);
      cv.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const t0 = tNow - VENTANA_VISIBLE_S;
    const vis = historial.filter(s => s.t >= t0);

    // Rango vertical automatico sobre lo VISIBLE, no sobre todo el buffer:
    // asi un pico viejo no deja aplastada la señal actual.
    let min = Infinity, max = -Infinity;
    for (const s of vis) {
      for (const val of s.v[g.id]) {
        if (val < min) min = val;
        if (val > max) max = val;
      }
    }
    if (!isFinite(min)) { min = -1; max = 1; }
    if (max - min < 1e-6) { const c = (max + min) / 2; min = c - 1; max = c + 1; }
    const pad = (max - min) * 0.12;
    min -= pad; max += pad;

    // Margenes apretados: con cuatro graficas apiladas en una columna de 380
    // px, cada pixel que se le quita al marco se lo gana la señal. El eje Y
    // solo rotula min y max, asi que 30 px de canal izquierdo sobran.
    const PL = 30, PR = 5, PT = 4, PB = 11;
    const gx = t => PL + (t - t0) / VENTANA_VISIBLE_S * (w - PL - PR);
    const gy = y => PT + (max - y) / (max - min) * (h - PT - PB);

    ctx.strokeStyle = "#E5E7EB";
    ctx.lineWidth = 1;
    ctx.strokeRect(PL + 0.5, PT + 0.5, w - PL - PR - 1, h - PT - PB - 1);

    if (min < 0 && max > 0) {
      ctx.strokeStyle = "#D1D5DB";
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(PL, gy(0)); ctx.lineTo(w - PR, gy(0));
      ctx.stroke();
      ctx.setLineDash([]);
    }

    ctx.fillStyle = "#9CA3AF";
    ctx.font = "500 8px 'IBM Plex Mono', monospace";
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    ctx.fillText(formato(max), PL - 4, gy(max) + 4);
    ctx.fillText(formato(min), PL - 4, gy(min) - 4);
    ctx.textBaseline = "alphabetic";
    ctx.textAlign = "left";
    ctx.fillText("-" + VENTANA_VISIBLE_S + "s", PL, h - 3);
    ctx.textAlign = "right";
    ctx.fillText("ahora", w - PR, h - 3);

    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.lineWidth = 1.4;
    g.etiquetas.forEach((_, i) => {
      ctx.strokeStyle = PALETA[i % PALETA.length];
      ctx.beginPath();
      let empezado = false;
      for (const s of vis) {
        const val = s.v[g.id][i];
        if (val === undefined || !isFinite(val)) continue;
        const px = gx(s.t), py = gy(val);
        if (!empezado) { ctx.moveTo(px, py); empezado = true; } else ctx.lineTo(px, py);
      }
      ctx.stroke();
    });
  }

  function dibujar(robot, tNow) {
    // Atlas y los uniciclos no llevan las mismas series: si cambio el tipo del
    // robot seleccionado hay que rehacer los bloques, no solo repintar.
    if (!specActual || tipoConstruido !== robot.tipo) construir(robot);
    const h = robot.historial || [];
    specActual.forEach(g => dibujarUna(lienzos[g.id], g, h, tNow));
  }

  return { construir, muestrear, dibujar, actualizarCFG, nombresRuedas };
})();


// ============================================================
// REGISTRO DE ENSAYO -> CSV
// ============================================================
//
// Las graficas sirven para MIRAR el lazo mientras corre; no sirven para
// contestar "¿cuanto sobrepaso?" tres dias despues. Eso se contesta con el
// numero, y el numero tiene que salir del navegador.
//
// Tres decisiones que hacen que esto valga para un ensayo de verdad:
//
//  1. El registro corre SIEMPRE, no solo al pulsar Grabar. Fuera de una
//     grabacion mantiene los ultimos VENTANA_BUFFER_S rodando; grabar solo
//     apaga el recorte. Asi la corrida que salio bien y que nadie armo
//     igual se puede exportar: es el mismo caso del rastro, nunca sabes de
//     antemano cual va a ser la buena.
//
//  2. Se registran los TRES robots, no el seleccionado. En formacion la
//     pregunta casi nunca es sobre uno solo, y cambiar de pestaña a media
//     corrida no debe abrir un hueco en los datos.
//
//  3. Cada fila lleva DOS relojes: el de pared (para casar con el video de
//     la cenital o con la libreta) y el de simulacion en segundos desde el
//     inicio del ensayo (para graficar). Con uno solo siempre falta el otro.
//
// La fila es plana y larga -una por robot y por instante- porque asi entra
// directo en pandas o en Excel sin pivotar nada: filtras por la columna
// robot y ya tienes su serie.
const Registro = (function () {
  // ~110 min de tres robots a 20 Hz. No es un limite de memoria (una fila
  // son unos cientos de bytes), es un freno para que una pestaña olvidada
  // toda la noche no se coma la RAM del laptop del banco.
  const TOPE_FILAS = 400000;

  const COLUMNAS = [
    "reloj", "t_s", "evento", "robot", "tipo", "modo", "armado", "modelo",
    "x_px", "y_px", "th_deg", "vx", "vy", "v", "w",
    "ex_px", "ey_px", "eth_deg", "dest_x", "dest_y", "dest_th_deg",
    "ruedas", "pwm0", "pwm1", "pwm2", "pwm3",
  ];

  // DOS buffers, no uno con un interruptor. Con uno solo, parar la grabacion
  // volvia a encender el recorte y el ensayo se iba borrando por la cola
  // mientras lo mirabas. Cada fila es { t, campos: [] } y se comparte entre
  // los dos: nadie la modifica despues de crearla.
  let rodante = [];        // los ultimos VENTANA_BUFFER_S, siempre
  let ensayo = [];         // la grabacion; sobrevive a parar()
  let grabando = false;
  let marcaPendiente = ""; // etiqueta que se pega a la proxima ronda
  let nMarcas = 0;
  let topeAvisado = false;
  let ultimoT = -Infinity;
  let ultimoEstado = null; // para no reescribir el rotulo 20 veces por segundo

  const grados = a => a * 180 / Math.PI;

  // Vacio en vez de "0" cuando el campo no aplica a este robot: una celda
  // vacia en pandas es NaN y se ignora sola en un promedio; un cero se
  // promedia y miente.
  function num(v) {
    if (v === undefined || v === null || !isFinite(v)) return "";
    return String(Math.round(v * 10000) / 10000);
  }

  // Hora local con milisegundos, no ISO en UTC: esto se lee al lado de una
  // libreta de laboratorio y de un video, y los dos estan en hora de aqui.
  function reloj(d) {
    const p = (n, a) => String(n).padStart(a || 2, "0");
    return d.getFullYear() + "-" + p(d.getMonth() + 1) + "-" + p(d.getDate()) + " " +
           p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds()) +
           "." + p(d.getMilliseconds(), 3);
  }

  function fila(r, modo, evento) {
    const holon = r.tipo === "holonomico";
    const v = r.velocidad || {};
    const pwm = r.pwm || [];
    const ruedas = holon ? Graficas.nombresRuedas().join("|") : "D|I";
    return [
      reloj(new Date()), "", evento,
      r.id, r.tipo, modo, r.armado ? 1 : 0, r.modelo,
      num(r.pose.x), num(r.pose.y), num(grados(r.pose.th)),
      num(v.vx), num(v.vy), num(v.v), num(v.w),
      num(r.destino.x - r.pose.x), num(r.destino.y - r.pose.y),
      num(grados(envolver(r.destino.th - r.pose.th))),
      num(r.destino.x), num(r.destino.y), num(grados(r.destino.th)),
      ruedas, num(pwm[0]), num(pwm[1]), num(pwm[2]), num(pwm[3]),
    ];
  }

  // Se llama desde el lazo principal, en la misma rama que Graficas.muestrear:
  // si la simulacion esta en pausa no se registra nada, porque serian filas
  // repetidas con el mismo t que despues hay que limpiar a mano.
  function muestrear(robots, t, modo) {
    // La tolerancia no es cosmetica: con un lazo que avanza justo el periodo,
    // el error de coma flotante deja la resta un pelo por debajo una vez de
    // cada dos y el registro cae a la mitad de la frecuencia sin avisar.
    if (t - ultimoT < PERIODO_MUESTREO_S - 1e-9) return;
    // El reloj de simulacion se reinicia al reiniciar la escena: sin esto,
    // el primer muestreo despues del reinicio caeria dentro del periodo y se
    // perderia hasta que t volviera a pasar el ultimo valor.
    if (t < ultimoT) ultimoT = -Infinity;
    ultimoT = t;

    const evento = marcaPendiente;
    marcaPendiente = "";
    const nuevas = robots.map(r => ({ t: t, campos: fila(r, modo, evento) }));

    for (const f of nuevas) rodante.push(f);
    while (rodante.length && t - rodante[0].t > VENTANA_BUFFER_S) rodante.shift();

    if (grabando) {
      if (ensayo.length >= TOPE_FILAS) {
        if (!topeAvisado) { topeAvisado = true; parar(); }
      } else {
        for (const f of nuevas) ensayo.push(f);
      }
    }
    pintarEstado();
  }

  function arrancar() {
    ensayo = []; nMarcas = 0; marcaPendiente = "";
    topeAvisado = false; grabando = true;
    pintarEstado(true);
  }

  // Parar NO borra: el ensayo se queda para poder exportarlo (y para poder
  // exportarlo otra vez si el primer archivo se guardo donde no era). Se
  // pierde al arrancar el siguiente, que es cuando ya no estorba.
  function parar() { grabando = false; pintarEstado(true); }

  function marcar(texto) {
    if (!grabando) return;
    nMarcas++;
    marcaPendiente = texto || ("marca " + nMarcas);
    pintarEstado(true);
  }

  function estaGrabando() { return grabando; }

  function csvCampo(s) {
    s = String(s);
    return /[",\n\r]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s;
  }

  // Que se exporta: el ensayo si hay uno, y si no el buffer rodante. El
  // ensayo gana siempre porque es lo que se pidio a proposito; el rodante
  // es la red de seguridad de la corrida que nadie armo.
  function fuente() { return ensayo.length ? ensayo : rodante; }

  function csv() {
    const filas = fuente();
    if (!filas.length) return null;
    // t_s cuenta desde la primera fila que hay, sea el inicio del ensayo o
    // el borde del buffer rodante: en los dos casos es "desde que empiezan
    // los datos", que es lo que uno pone en el eje X.
    const base = filas[0].t;
    const lineas = [COLUMNAS.join(",")];
    for (const f of filas) {
      const c = f.campos.slice();
      c[1] = (f.t - base).toFixed(3);
      lineas.push(c.map(csvCampo).join(","));
    }
    // CRLF y BOM: sin el BOM, Excel abre el archivo en la codificacion del
    // sistema y "posición" sale con jeroglificos. pandas se traga las dos
    // cosas sin decir nada.
    const BOM = String.fromCharCode(0xFEFF);
    return BOM + lineas.join("\r\n") + "\r\n";
  }

  function babosa(s) {
    return (s || "").toLowerCase()
      .normalize("NFD").replace(/\p{M}/gu, "")   // "posición" -> "posicion"
      .replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "").slice(0, 40);
  }

  function exportar(nombre) {
    const texto = csv();
    if (!texto) return false;
    const d = new Date();
    const p = n => String(n).padStart(2, "0");
    const sello = d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()) + "-" +
                  p(d.getHours()) + p(d.getMinutes()) + p(d.getSeconds());
    const etiqueta = babosa(nombre) || (ensayo.length ? "ensayo" : "buffer");
    const url = URL.createObjectURL(new Blob([texto], { type: "text/csv;charset=utf-8" }));
    const a = document.createElement("a");
    a.href = url;
    a.download = "atlas_" + etiqueta + "_" + sello + ".csv";
    document.body.appendChild(a);
    a.click();
    a.remove();
    // Un respiro antes de soltar el blob: en Firefox, revocar en el mismo
    // tick a veces cancela la descarga que se acaba de disparar.
    setTimeout(() => URL.revokeObjectURL(url), 1000);
    return true;
  }

  function duracion(filas) {
    if (!filas.length) return 0;
    return filas[filas.length - 1].t - filas[0].t;
  }

  function pintarEstado(forzar) {
    const el = document.getElementById("recEstado");
    if (!el) return;
    const filas = fuente();
    const seg = duracion(filas).toFixed(1);
    // Se dice de donde saldria el CSV AHORA, no que boton esta pulsado: el
    // error caro es exportar el buffer creyendo que era el ensayo.
    const marcas = " · " + nMarcas + (nMarcas === 1 ? " marca" : " marcas");
    let txt = grabando
      ? "grabando · " + seg + " s · " + filas.length + " filas" + marcas
      : (filas.length
          ? (ensayo.length
              ? "ensayo detenido · " + seg + " s · " + filas.length + " filas" + marcas
              : "buffer de " + seg + " s · " + filas.length + " filas")
          : "sin datos");
    if (topeAvisado) txt += " · TOPE, se detuvo solo";
    if (!forzar && txt === ultimoEstado) return;
    ultimoEstado = txt;
    el.textContent = txt;
    el.classList.toggle("grabando", grabando);
  }

  return { muestrear, arrancar, parar, marcar, exportar, estaGrabando, pintarEstado };
})();
