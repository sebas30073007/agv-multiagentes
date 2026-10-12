// escena3d.js - la misma cancha que canvas.js, pero en 3D.
//
// Igual que Escena, no sabe nada de control ni de red: cada cuadro recibe
// los robots y los obstaculos ya posicionados y los coloca. Todo vive en PX
// DE MUNDO (1 unidad de three = 1 px) y con Z hacia arriba, asi que las
// coordenadas del estado se usan tal cual: el piso es el plano XY de siempre
// y lo unico nuevo es la altura.
//
// Se carga solo al pedir la vista 3D (import() desde main.js): quien nunca
// la abre no paga ni el WebGL ni los megas del GLB.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { CSS2DRenderer, CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";
import { MeshoptDecoder } from "three/addons/libs/meshopt_decoder.module.js";

const PISO = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

// Altura de cada guia, en px de mundo. Son RELIEVE, no volumen: tienen que
// leerse como algo pintado con grosor sobre el suelo, que es lo que son. Lo
// unico que se levanta de verdad son los campos repulsivos.
const RELIEVE = {
  destino: 4, aro: 1.6, referencia: 1.2, rastro: 1.8, centro: 2.5,
  formacion: 1.4, camino: 1.2, borde: 3,
};

// Guiones de las cintas discontinuas, en px de mundo. En 2D se median en
// pixeles de pantalla; aqui la camara se acerca y se aleja, asi que van en
// mundo y un pelo mas largos para que de lejos no se vuelvan puntos.
const GUION_ARO = [8, 7];
const GUION_REFERENCIA = [10, 8];
const GUION_CAMINO = [7, 9];

// Alto del solido de un campo. Con raiz cuadrada y no lineal: la
// intensidad de arranque de un obstaculo (cientos) contra el tope del riel
// (ETA_MAX) daria un bulto aplastado; asi la parte baja del riel tambien se
// nota al subirla.
const ALTO_CAMPO_MIN = 30;
const ALTO_CAMPO_MAX = 200;
function altoCampo(eta) {
  return ALTO_CAMPO_MIN + (ALTO_CAMPO_MAX - ALTO_CAMPO_MIN) * Math.sqrt(Math.min(1, Math.max(0, eta) / ETA_MAX));
}

// ---------------------------------------------------------------
// CINTAS: toda guia en relieve es una polilinea con ancho y alto
// ---------------------------------------------------------------

// Parte una polilinea en guiones. La fase arranca en el PRIMER punto, igual
// que en canvas.js: quien llama decide cual es el extremo quieto para que
// los guiones no "corran" cuando el otro extremo se mueve.
function discontinuar(pts, on, off) {
  const out = [];
  let actual = null, dentro = true, resto = on;
  for (let i = 0; i < pts.length - 1; i++) {
    let ax = pts[i][0], ay = pts[i][1];
    const bx = pts[i + 1][0], by = pts[i + 1][1];
    let largo = Math.hypot(bx - ax, by - ay);
    if (largo < 1e-9) continue;
    const ux = (bx - ax) / largo, uy = (by - ay) / largo;
    while (largo > 1e-9) {
      const paso = Math.min(resto, largo);
      const cx = ax + ux * paso, cy = ay + uy * paso;
      if (dentro) {
        if (!actual) { actual = [[ax, ay]]; out.push(actual); }
        actual.push([cx, cy]);
      }
      ax = cx; ay = cy; largo -= paso; resto -= paso;
      if (resto <= 1e-9) { dentro = !dentro; resto = dentro ? on : off; actual = null; }
    }
  }
  return out;
}

function circulo(cx, cy, r, n) {
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const a = 2 * Math.PI * i / n;
    pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
  }
  return pts;
}

// Una cinta es una malla que se reescribe cuando cambia lo que dibuja. Los
// triangulos van sueltos, sin indices, para que cada cara lleve su normal
// plana: con normales suavizadas el canto sale redondo y el relieve deja de
// leerse como relieve.
class Cinta {
  constructor(material) {
    this.geo = new THREE.BufferGeometry();
    this.cap = 0;
    this.clave = null;
    this.asegurar(512);
    this.malla = new THREE.Mesh(this.geo, material);
    this.malla.frustumCulled = false;
  }

  asegurar(nVerts) {
    if (nVerts <= this.cap) return;
    this.cap = Math.ceil(nVerts * 1.5);
    this.pos = new Float32Array(this.cap * 3);
    this.nor = new Float32Array(this.cap * 3);
    this.geo.setAttribute("position", new THREE.BufferAttribute(this.pos, 3));
    this.geo.setAttribute("normal", new THREE.BufferAttribute(this.nor, 3));
  }

  // polilineas: lista de listas de [x,y]. La clave evita reescribir la
  // malla cuando nada cambio, que es casi siempre para las curvas fijas;
  // con clave null se reescribe siempre.
  fijar(clave, polilineas, ancho, alto) {
    if (clave !== null && clave === this.clave) return;
    this.clave = clave;
    let segs = 0;
    for (const p of polilineas) segs += Math.max(0, p.length - 1);
    this.asegurar(segs * 18);
    let v = 0;
    for (const p of polilineas) v = this.escribir(p, ancho / 2, alto, v);
    this.geo.setDrawRange(0, v);
    this.geo.attributes.position.needsUpdate = true;
    this.geo.attributes.normal.needsUpdate = true;
  }

  escribir(p, h, alto, v) {
    const n = p.length;
    if (n < 2) return v;
    // Desplazamiento lateral de cada vertice: la bisectriz de sus dos
    // segmentos (inglete), para que la cinta no se abra en las curvas.
    const ox = new Float32Array(n), oy = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const a = p[Math.max(0, i - 1)], b = p[i], c = p[Math.min(n - 1, i + 1)];
      let t1x = b[0] - a[0], t1y = b[1] - a[1], t2x = c[0] - b[0], t2y = c[1] - b[1];
      const l1 = Math.hypot(t1x, t1y), l2 = Math.hypot(t2x, t2y);
      if (l2 > 1e-9) { t2x /= l2; t2y /= l2; }
      if (l1 > 1e-9) { t1x /= l1; t1y /= l1; } else { t1x = t2x; t1y = t2y; }
      if (l2 <= 1e-9) { t2x = t1x; t2y = t1y; }
      let tx = t1x + t2x, ty = t1y + t2y;
      const lt = Math.hypot(tx, ty);
      if (lt < 1e-9) { tx = t1x; ty = t1y; } else { tx /= lt; ty /= lt; }
      // Topado: en un pico cerrado el inglete se iria al infinito.
      const cosm = Math.max(0.35, tx * t2x + ty * t2y);
      ox[i] = -ty * h / cosm; oy[i] = tx * h / cosm;
    }

    const P = this.pos, N = this.nor;
    const vert = (x, y, z, nx, ny, nz) => {
      const k = v * 3;
      P[k] = x; P[k + 1] = y; P[k + 2] = z;
      N[k] = nx; N[k + 1] = ny; N[k + 2] = nz;
      v++;
    };
    for (let i = 0; i < n - 1; i++) {
      const a = p[i], b = p[i + 1];
      const dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy);
      if (l < 1e-9) continue;
      const nx = -dy / l, ny = dx / l;
      const Lax = a[0] + ox[i], Lay = a[1] + oy[i], Rax = a[0] - ox[i], Ray = a[1] - oy[i];
      const Lbx = b[0] + ox[i + 1], Lby = b[1] + oy[i + 1], Rbx = b[0] - ox[i + 1], Rby = b[1] - oy[i + 1];
      // tapa
      vert(Lax, Lay, alto, 0, 0, 1); vert(Rax, Ray, alto, 0, 0, 1); vert(Rbx, Rby, alto, 0, 0, 1);
      vert(Lax, Lay, alto, 0, 0, 1); vert(Rbx, Rby, alto, 0, 0, 1); vert(Lbx, Lby, alto, 0, 0, 1);
      // pared izquierda
      vert(Lax, Lay, 0, nx, ny, 0); vert(Lbx, Lby, 0, nx, ny, 0); vert(Lbx, Lby, alto, nx, ny, 0);
      vert(Lax, Lay, 0, nx, ny, 0); vert(Lbx, Lby, alto, nx, ny, 0); vert(Lax, Lay, alto, nx, ny, 0);
      // pared derecha
      vert(Rax, Ray, 0, -nx, -ny, 0); vert(Rbx, Rby, alto, -nx, -ny, 0); vert(Rbx, Rby, 0, -nx, -ny, 0);
      vert(Rax, Ray, 0, -nx, -ny, 0); vert(Rax, Ray, alto, -nx, -ny, 0); vert(Rbx, Rby, alto, -nx, -ny, 0);
    }
    return v;
  }
}

function materialGuia(color, opacidad) {
  return new THREE.MeshLambertMaterial({
    color, side: THREE.DoubleSide,
    transparent: opacidad < 1, opacity: opacidad, depthWrite: opacidad >= 1,
  });
}

// La flecha del destino: la misma cometa que real.png, con sus proporciones
// medidas sobre el PNG (fracciones del lado de la caja del marcador). Asi la
// flecha 3D mide lo mismo que la 2D.
function geometriaFlechaDestino(lado) {
  const f = s => s * lado;
  const forma = new THREE.Shape();
  forma.moveTo(f(0.207), 0);
  forma.lineTo(f(-0.002), f(0.076));
  forma.lineTo(f(-0.07), 0);
  forma.lineTo(f(0.012), f(-0.076));
  forma.closePath();
  return new THREE.ExtrudeGeometry(forma, {
    depth: RELIEVE.destino, bevelEnabled: true,
    bevelThickness: 0.8, bevelSize: 0.8, bevelSegments: 1,
  });
}

// Flecha de los campos: unitaria a lo largo de +x, de 0 a 1. Se escala por
// instancia: x es el largo, y/z el grosor.
function geometriaFlechaCampo() {
  const asta = new THREE.BoxGeometry(0.62, 1, 1);
  asta.translate(0.31, 0, 0);
  const punta = new THREE.ConeGeometry(2.4, 0.38, 10);
  punta.rotateZ(-Math.PI / 2);
  punta.translate(0.81, 0, 0);
  return mergeGeometries([asta, punta]);
}

// ---------------------------------------------------------------
// CAMPO REPULSIVO EN 3D
// ---------------------------------------------------------------
// Si se levanta la fuerza sobre el circulo y se gira alrededor del centro,
// cada formula es un solido: lineal un cilindro, lineal_norm un cono,
// khatib una campana con la punta cortada. El perfil sale de la MISMA
// magnitudCampo() que usa la fisica, asi que el dibujo no puede mentir.
const MERIDIANOS = 12;
const NIVELES_ISO = [0.25, 0.5, 0.75];

class Campo3D {
  constructor(escena, geoFlecha, pick) {
    this.grupo = new THREE.Group();
    this.matSolido = new THREE.MeshLambertMaterial({
      transparent: true, depthWrite: false, side: THREE.DoubleSide,
    });
    this.solido = new THREE.Mesh(new THREE.BufferGeometry(), this.matSolido);
    this.solido.userData.pick = pick;
    this.grupo.add(this.solido);

    this.matLineas = new THREE.LineBasicMaterial({ transparent: true, depthWrite: false });
    this.lineas = new THREE.LineSegments(new THREE.BufferGeometry(), this.matLineas);
    this.grupo.add(this.lineas);

    this.aro = new Cinta(materialGuia(0xffffff, 0.9));
    this.grupo.add(this.aro.malla);

    this.matFlechas = new THREE.MeshLambertMaterial({ transparent: true, opacity: 0.92 });
    this.flechas = new THREE.InstancedMesh(geoFlecha, this.matFlechas, 64);
    this.flechas.frustumCulled = false;
    this.grupo.add(this.flechas);

    // El foco del lado por el que entra el robot. Siempre existe, con
    // intensidad cero cuando no hay nadie: añadir y quitar luces obliga a
    // recompilar los shaders de toda la escena.
    this.luz = new THREE.PointLight(0xffffff, 0, 1, 0);
    escena.add(this.luz);

    this.clave = null;
    this.color = new THREE.Color();
    this.grupo.visible = false;
    escena.add(this.grupo);
  }

  ocultar() {
    this.grupo.visible = false;
    this.luz.intensity = 0;
  }

  // campo: { tipo, sentido, eta, cerca, anim, hacia }, igual que en canvas.js.
  actualizar(campo, cx, cy, radio, colorHex, realce, densidad, opacidadBase) {
    this.grupo.visible = true;
    this.grupo.position.set(cx, cy, 0);
    this.grupo.scale.set(realce, realce, 1);
    const R = radio, H = altoCampo(campo.eta);
    const sentido = campo.sentido < 0 ? -1 : 1;
    const espiral = campo.tipo === "espiral";
    const frac = d => campo.eta > 0 ? magnitudCampo(d, R, campo.eta, campo.tipo) / campo.eta : 0;

    const clave = [campo.tipo, R, Math.round(campo.eta), sentido, colorHex].join("|");
    if (clave !== this.clave) {
      this.clave = clave;
      this.color.set(colorHex);
      this.construir(R, H, frac, espiral, sentido);
      this.matSolido.color.copy(this.color);
      this.matLineas.color.copy(this.color);
      this.aro.malla.material.color.copy(this.color);
      this.luz.color.copy(this.color);
      this.aro.fijar(null, discontinuar(circulo(0, 0, R, 96), GUION_ARO[0], GUION_ARO[1]), 2.2, RELIEVE.aro);
    }

    const cerca = campo.cerca || 0;
    this.matSolido.opacity = opacidadBase + 0.22 * cerca;
    this.matLineas.opacity = 0.35 + 0.5 * cerca;
    // Los meridianos del espiral giran con la animacion. Un meridiano de
    // vuelta por ciclo: al envolver anim de 1 a 0 el dibujo cae exactamente
    // sobre si mismo y no se ve el salto.
    this.lineas.rotation.z = espiral ? campo.anim * (2 * Math.PI / MERIDIANOS) * sentido : 0;

    this.colocarFlechas(campo, R, frac, H, densidad, espiral, sentido);

    if (cerca > 0.01 && campo.hacia) {
      this.luz.position.set(cx + campo.hacia.x * R * 0.55, cy + campo.hacia.y * R * 0.55, H * 0.6);
      this.luz.distance = R * 1.6;
      this.luz.intensity = 3.2 * cerca;
    } else {
      this.luz.intensity = 0;
    }
  }

  construir(R, H, frac, espiral, sentido) {
    // Perfil: del centro arriba hacia el borde abajo. Un punto justo antes
    // del borde para que el cilindro de "lineal" salga con pared vertical.
    const perfil = [];
    const N = 48;
    for (let i = 0; i <= N; i++) {
      const d = R * i / N;
      if (i === N) perfil.push(new THREE.Vector2(R * 0.998, H * frac(R * 0.998)));
      perfil.push(new THREE.Vector2(d, H * frac(d)));
    }
    const geo = new THREE.LatheGeometry(perfil, 72);
    geo.rotateX(Math.PI / 2);   // el torno gira sobre Y; aqui arriba es Z
    this.solido.geometry.dispose();
    this.solido.geometry = geo;

    // Lineas sobre la superficie: curvas de nivel (dicen la forma de un
    // vistazo) y meridianos, torcidos en el espiral.
    const seg = [];
    for (const nivel of NIVELES_ISO) {
      // Radio al que el perfil baja de ese nivel, buscando desde fuera.
      let rr = null;
      for (let i = N; i > 0; i--) {
        const d0 = R * (i - 1) / N, d1 = R * i / N;
        if (frac(d0) >= nivel && frac(d1) < nivel) { rr = d1; break; }
      }
      if (rr === null) continue;
      const z = H * nivel;
      for (let k = 0; k < 64; k++) {
        const a0 = 2 * Math.PI * k / 64, a1 = 2 * Math.PI * (k + 1) / 64;
        seg.push(rr * Math.cos(a0), rr * Math.sin(a0), z, rr * Math.cos(a1), rr * Math.sin(a1), z);
      }
    }
    const torsion = espiral ? GIRO_ESPIRAL * sentido * 1.6 : 0;
    for (let m = 0; m < MERIDIANOS; m++) {
      const base = 2 * Math.PI * m / MERIDIANOS;
      let px = null, py = 0, pz = 0;
      for (let i = 0; i <= 24; i++) {
        const d = R * i / 24;
        const a = base + torsion * (d / R);
        const x = d * Math.cos(a), y = d * Math.sin(a), z = H * frac(Math.min(d, R * 0.998)) + 0.6;
        if (px !== null) seg.push(px, py, pz, x, y, z);
        px = x; py = y; pz = z;
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(seg, 3));
    this.lineas.geometry.dispose();
    this.lineas.geometry = g;
  }

  // Las tres coronas de canvas.js, con la misma cuenta de largo y la misma
  // onda que sale del centro. Apoyadas sobre la superficie del solido.
  colocarFlechas(campo, R, frac, H, densidad, espiral, sentido) {
    const largoMax = LARGO_FLECHA * R;
    const cerca = campo.cerca || 0;
    const grosor = 1.0 + 0.9 * cerca + 0.6 * Math.min(1, (campo.eta || 0) / ETA_MAX);
    const giro = espiral ? GIRO_ESPIRAL * sentido : 0;
    // Las posiciones del espiral avanzan una separacion de la corona de
    // dentro por ciclo: es multiplo exacto de las otras dos, asi que al
    // envolver anim tampoco hay salto.
    const n0 = Math.max(3, Math.round(CORONAS[0].n * densidad));
    const arrastre = espiral ? campo.anim * (2 * Math.PI / n0) * sentido : 0;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), p = new THREE.Vector3();
    const eje = new THREE.Vector3(0, 0, 1);
    const blanco = new THREE.Color(0xffffff), c = new THREE.Color();
    const tope = this.flechas.instanceMatrix.count;
    let k = 0;
    CORONAS.forEach((corona, idx) => {
      const fase = (((campo.anim - idx / CORONAS.length) % 1) + 1) % 1;
      const onda = Math.cos(2 * Math.PI * fase);
      const rad = R * (corona.radio + 0.035 * onda);
      const fuerza = campo.eta > 0 ? frac(corona.radio * R) : 0;
      const largo = Math.min(largoMax * (LARGO_MINIMO + (1 - LARGO_MINIMO) * fuerza), R * (1 - corona.radio));
      const brillo = 0.38 + 0.62 * cerca * (0.5 + 0.5 * onda);
      c.copy(this.color).lerp(blanco, 0.55 * (1 - brillo));
      const n = Math.max(3, Math.round(corona.n * densidad));
      const desfase = (idx % 2) * Math.PI / n;
      for (let i = 0; i < n && k < tope; i++) {
        const a = i * 2 * Math.PI / n + desfase + arrastre;
        p.set(rad * Math.cos(a), rad * Math.sin(a), H * frac(rad) + grosor * 2.4 + 1);
        q.setFromAxisAngle(eje, a + giro);
        s.set(largo, grosor, grosor);
        m.compose(p, q, s);
        this.flechas.setMatrixAt(k, m);
        this.flechas.setColorAt(k, c);
        k++;
      }
    });
    this.flechas.count = largoMax < 2.2 ? 0 : k;
    this.flechas.instanceMatrix.needsUpdate = true;
    if (this.flechas.instanceColor) this.flechas.instanceColor.needsUpdate = true;
  }
}

// ---------------------------------------------------------------
// ROBOTS
// ---------------------------------------------------------------
const cacheGLB = new Map();
// Los GLB van comprimidos con meshopt (ver README): sin el decodificador
// el cargador no los abre.
const cargador = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
function cargarGLB(src) {
  if (!cacheGLB.has(src)) cacheGLB.set(src, cargador.loadAsync(src));
  return cacheGLB.get(src);
}

// Deja un GLB listo para la cancha: de Y-arriba (glTF) a Z-arriba, girado
// para que su frente caiga sobre +x -que es hacia donde lo mueve la
// fisica-, centrado en su planta y apoyado en Z=0. Los CAD traen el origen
// donde les dio la gana (el de Odiseo, en una esquina), asi que eso se
// corrige. El TAMAÑO no: glTF va en metros y los CAD ya traen la medida
// real, asi que todos pasan a px con el mismo PX_POR_METRO_3D. Escalar
// cada uno a su sprite hacia que dos robots guardaran en 3D la proporcion
// de sus PNG en vez de la de verdad.
function normalizarModelo(escenaGLB, giro) {
  const raiz = escenaGLB.clone(true);
  raiz.rotation.x = Math.PI / 2;
  const envoltura = new THREE.Group();
  envoltura.add(raiz);
  envoltura.rotation.z = giro || 0;
  const fuera = new THREE.Group();
  fuera.add(envoltura);
  fuera.updateMatrixWorld(true);
  const caja = new THREE.Box3().setFromObject(fuera);
  const tam = caja.getSize(new THREE.Vector3());
  const centro = caja.getCenter(new THREE.Vector3());
  const s = PX_POR_METRO_3D;
  envoltura.scale.setScalar(s);
  envoltura.position.set(-centro.x * s, -centro.y * s, -caja.min.z * s);
  raiz.traverse(o => { if (o.isMesh) o.castShadow = true; });
  return { grupo: fuera, largo: tam.x * s, ancho: tam.y * s, alto: tam.z * s };
}

// ---------------------------------------------------------------
// LA ESCENA
// ---------------------------------------------------------------
export function crear(contenedor, opciones) {
  const CFG0 = opciones.cfg();
  const W = CFG0.ancho, H = CFG0.alto;

  // Lanza si no hay WebGL: main.js lo atrapa y se queda en 2D.
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setClearColor(0xE5E7EB);
  renderer.domElement.className = "lienzo-3d";
  contenedor.appendChild(renderer.domElement);

  const etiquetas = new CSS2DRenderer();
  etiquetas.domElement.className = "etiquetas-3d";
  contenedor.appendChild(etiquetas.domElement);

  const escena = new THREE.Scene();
  const camara = new THREE.PerspectiveCamera(35, 1, 5, 20000);
  // Z arriba ANTES de crear los controles: OrbitControls toma el "arriba"
  // de la camara al construirse, y con el de fabrica (Y) orbitaria de lado.
  camara.up.set(0, 0, 1);

  const controles = new OrbitControls(camara, renderer.domElement);
  // El clic izquierdo es de la cancha -arrastrar robots, poner destinos-,
  // igual que en 2D. La camara se mueve con los otros dos botones y la rueda.
  controles.mouseButtons = { LEFT: -1, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE };
  controles.touches = { ONE: -1, TWO: THREE.TOUCH.DOLLY_ROTATE };
  controles.enableDamping = true;
  controles.dampingFactor = 0.12;
  controles.screenSpacePanning = false;   // desplazar sobre el piso, no en el plano de la pantalla
  controles.maxPolarAngle = THREE.MathUtils.degToRad(85);
  controles.minDistance = 150;
  controles.maxDistance = 6000;

  // ---- luz ----
  const hemi = new THREE.HemisphereLight(0xffffff, 0xC2C7CF, 1.9);
  hemi.position.set(0, 0, 1);
  escena.add(hemi);
  const sol = new THREE.DirectionalLight(0xffffff, 1.6);
  sol.position.set(-520, -760, 1500);
  sol.castShadow = true;
  sol.shadow.mapSize.set(2048, 2048);
  Object.assign(sol.shadow.camera, { left: -W * 0.75, right: W * 0.75, top: H * 0.9, bottom: -H * 0.9, near: 100, far: 4000 });
  sol.shadow.bias = -0.0005;
  escena.add(sol);
  escena.add(sol.target);

  // ---- piso ----
  const alrededor = new THREE.Mesh(new THREE.PlaneGeometry(W * 8, H * 8),
    new THREE.MeshLambertMaterial({ color: 0xE5E7EB }));
  alrededor.position.z = -1.5;
  alrededor.receiveShadow = true;
  escena.add(alrededor);
  const cancha = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshLambertMaterial({ color: 0xffffff }));
  cancha.receiveShadow = true;
  escena.add(cancha);

  // Reticula con los mismos pasos y colores que la de canvas.js. Cada
  // nivel un pelo mas arriba que el anterior para que no se peleen en el
  // z-buffer donde coinciden.
  const primera = (limite, paso) => Math.ceil(-limite / paso) * paso;
  Escena.nivelesGrid.forEach((nivel, i) => {
    const v = [];
    for (let x = primera(W / 2, nivel.paso); x <= W / 2; x += nivel.paso) v.push(x, -H / 2, 0, x, H / 2, 0);
    for (let y = primera(H / 2, nivel.paso); y <= H / 2; y += nivel.paso) v.push(-W / 2, y, 0, W / 2, y, 0);
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(v, 3));
    const l = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: nivel.color }));
    l.position.z = 0.15 + i * 0.08;
    escena.add(l);
  });
  {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute([0, -H / 2, 0, 0, H / 2, 0, -W / 2, 0, 0, W / 2, 0, 0], 3));
    const ejes = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: Escena.colorEje }));
    ejes.position.z = 0.5;
    escena.add(ejes);
  }
  const borde = new Cinta(materialGuia(0xC4C9D0, 1));
  borde.fijar(null, [[[-W / 2, -H / 2], [W / 2, -H / 2], [W / 2, H / 2], [-W / 2, H / 2], [-W / 2, -H / 2]]], 3, RELIEVE.borde);
  escena.add(borde.malla);

  function cota(texto, x, y) {
    const div = document.createElement("div");
    div.className = "cota-3d";
    div.textContent = texto;
    const o = new CSS2DObject(div);
    o.position.set(x, y, 0);
    escena.add(o);
  }
  for (let x = primera(W / 2, 200); x <= W / 2; x += 200) if (x !== 0) cota(x + " px", x, -H / 2 - 18);
  for (let y = primera(H / 2, 100); y <= H / 2; y += 100) if (y !== 0) cota(y + " px", -W / 2 - 32, y);

  // ---- piezas compartidas ----
  const geoFlechaCampo = geometriaFlechaCampo();
  const geoFlechaDestino = geometriaFlechaDestino((CFG0.ancho_robot_px || 250) / 2 * TAM_MARCA);
  const maxAniso = renderer.capabilities.getMaxAnisotropy();

  // ---- robots ----
  const entradas = new Map();

  function entradaRobot(r) {
    if (entradas.has(r.id)) return entradas.get(r.id);
    const color = new THREE.Color(r.color);

    const pose = new THREE.Group();
    const cuerpo = new THREE.Group();
    pose.add(cuerpo);
    escena.add(pose);

    // Mismo largo que el sprite en canvas.js, que es tambien el diametro
    // del cuerpo que usa la repulsion (radioCuerpo en main.js).
    const largo = (CFG0.ancho_robot_px || 250) * 1.35 * (r.escalaImg || 1);
    const proxy = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial());
    proxy.visible = false;   // no se dibuja, pero el raycast lo sigue viendo
    proxy.userData.pick = { tipo: "robot", id: r.id };
    cuerpo.add(proxy);

    const div = document.createElement("div");
    div.className = "etiqueta-3d";
    div.textContent = r.nombre;
    div.style.color = r.color;
    const etiqueta = new CSS2DObject(div);
    pose.add(etiqueta);

    const e = {
      pose, cuerpo, proxy, etiqueta, modelo: null,
      destino: new THREE.Mesh(geoFlechaDestino, materialGuia(color, 1)),
      zona: new Cinta(materialGuia(color, 0.95)),
      referencia: new Cinta(materialGuia(color, 0.45)),
      rastro: new Cinta(materialGuia(color, 0.78)),
      centro: new Cinta(materialGuia(color, 1)),
      campo: new Campo3D(escena, geoFlechaCampo, null),
    };
    e.destino.castShadow = true;
    escena.add(e.destino, e.zona.malla, e.referencia.malla, e.rastro.malla, e.centro.malla);

    // Encaja una pieza como cuerpo del robot: la tarjeta de arranque o el
    // GLB cuando termina de cargar. El proxy y la etiqueta se ajustan a lo
    // que haya.
    function montar(obj, dims) {
      if (e.modelo) cuerpo.remove(e.modelo);
      e.modelo = obj;
      cuerpo.add(obj);
      proxy.scale.set(dims.largo, dims.ancho, dims.alto);
      proxy.position.set(0, 0, dims.alto / 2);
      etiqueta.position.set(0, 0, dims.alto + 22);
    }

    // Tarjeta: el sprite de siempre, flotando un palmo sobre el piso con su
    // sombra debajo y el mismo giro que en 2D (spriteOffset). Es lo que
    // llevan los robots sin GLB, y lo que se ve mientras el GLB carga. Un
    // plano y no una caja: los PNG tienen margen transparente, y a traves
    // de el se veian las paredes de la caja.
    const img = r.sprite || r.img;
    const iw = img ? (img.naturalWidth || img.width) : 0, ih = img ? (img.naturalHeight || img.height) : 0;
    const anchoT = iw ? largo * ih / iw : largo;
    const altoT = 10;
    let matTarjeta;
    if (iw) {
      const tex = new THREE.CanvasTexture(img);
      tex.colorSpace = THREE.SRGBColorSpace;
      tex.anisotropy = maxAniso;
      // alphaTest tambien recorta la sombra: sale con la silueta del chasis.
      matTarjeta = new THREE.MeshLambertMaterial({ map: tex, alphaTest: 0.5, side: THREE.DoubleSide });
    } else {
      matTarjeta = new THREE.MeshLambertMaterial({ color, side: THREE.DoubleSide });
    }
    const tarjeta = new THREE.Mesh(new THREE.PlaneGeometry(largo, anchoT), matTarjeta);
    tarjeta.position.z = altoT;
    tarjeta.castShadow = true;
    const tarjetaGrupo = new THREE.Group();
    tarjetaGrupo.add(tarjeta);
    tarjetaGrupo.rotation.z = r.spriteOffset || 0;
    const girada = Math.abs(Math.sin(r.spriteOffset || 0)) > 0.5;
    montar(tarjetaGrupo, { largo: girada ? anchoT : largo, ancho: girada ? largo : anchoT, alto: altoT });

    if (r.malla3d && r.malla3d.src) {
      cargarGLB(r.malla3d.src).then(gltf => {
        const n = normalizarModelo(gltf.scene, r.malla3d.giro);
        montar(n.grupo, n);
      }).catch(err => console.warn("[3D] no cargo " + r.malla3d.src, err));
    }

    entradas.set(r.id, e);
    return e;
  }

  function ocultarRobot(e) {
    e.pose.visible = false;
    e.etiqueta.visible = false;
    e.destino.visible = e.zona.malla.visible = e.referencia.malla.visible = false;
    e.rastro.malla.visible = e.centro.malla.visible = false;
    e.campo.ocultar();
  }

  function dibujarRobot(r, d) {
    const e = entradaRobot(r);
    e.pose.visible = true;
    e.etiqueta.visible = true;
    e.pose.position.set(r.pose.x, r.pose.y, 0);
    e.pose.rotation.z = r.pose.th;
    e.cuerpo.scale.setScalar(d.realce(r, "pose"));

    // Campo propio: mas tenue que el de un obstaculo, para que el robot se
    // siga viendo dentro.
    const rp = r.repulsor;
    if (rp && rp.eta > 0) {
      e.campo.actualizar(rp, r.pose.x, r.pose.y, rp.radio, r.color, 1, DENSIDAD_ROBOT, 0.07);
    } else {
      e.campo.ocultar();
    }

    // Destino: en todo modelo menos formacion, igual que paso() en 2D.
    const conDestino = r.modelo !== "formacion";
    e.destino.visible = conDestino;
    if (conDestino) {
      const s = d.realce(r, "destino");
      e.destino.position.set(r.destino.x, r.destino.y, 0);
      e.destino.rotation.z = r.destino.th;
      e.destino.scale.set(s, s, 1);
    }

    // Zona de llegada: solo en Punto.
    const tol = (r.parametros.punto.tolPos || 0) * d.realce(r, "destino");
    e.zona.malla.visible = r.modelo === "punto" && tol >= 2;
    if (e.zona.malla.visible) {
      e.zona.fijar([r.destino.x, r.destino.y, tol].map(v => v.toFixed(1)).join("|"),
        discontinuar(circulo(r.destino.x, r.destino.y, tol, 64), GUION_ARO[0], GUION_ARO[1]), 2, RELIEVE.aro);
    }

    // Referencia: del destino hacia el robot (el destino es el extremo
    // quieto) o la curva completa de la trayectoria.
    const t = r.parametros.trayectoria;
    if (r.modelo === "punto") {
      e.referencia.malla.visible = true;
      e.referencia.fijar(null, discontinuar([[r.destino.x, r.destino.y], [r.pose.x, r.pose.y]],
        GUION_REFERENCIA[0], GUION_REFERENCIA[1]), 2.4, RELIEVE.referencia);
    } else if (r.modelo === "trayectoria" && t.centroInicial) {
      e.referencia.malla.visible = true;
      const fn = t.tipo === "lemniscata" ? trayectoriaLemniscata : trayectoriaCirculo;
      const clave = [t.tipo, t.radio, t.centroInicial[0], t.centroInicial[1]].join("|");
      if (clave !== e.referencia.clave) {
        const pts = [];
        for (let i = 0; i <= 180; i++) pts.push(fn(2 * Math.PI * i / 180, t.centroInicial, t.radio, 1));
        e.referencia.fijar(clave, discontinuar(pts, GUION_REFERENCIA[0], GUION_REFERENCIA[1]), 2.4, RELIEVE.referencia);
      }
    } else {
      e.referencia.malla.visible = false;
    }

    // Rastro: por donde paso de verdad, continuo.
    const ras = r.rastro;
    e.rastro.malla.visible = ras.length >= 2;
    if (e.rastro.malla.visible) {
      const u = ras[ras.length - 1];
      e.rastro.fijar(ras.length + "|" + ras[0].t + "|" + u.x + "|" + u.y,
        [ras.map(p => [p.x, p.y])], 3, RELIEVE.rastro);
    }

    // Cruz del centro de la trayectoria.
    e.centro.malla.visible = r.modelo === "trayectoria" && !!t.centroInicial;
    if (e.centro.malla.visible) {
      const b = TAM_CENTRO_TRAYECTORIA * 1.6 * d.realce(r, "centro");
      const cx = t.centroInicial[0], cy = t.centroInicial[1];
      e.centro.fijar([cx, cy, b].join("|"), [[[cx - b, cy], [cx + b, cy]], [[cx, cy - b], [cx, cy + b]]], 3, RELIEVE.centro);
    }
  }

  // ---- obstaculos ----
  const obstaculos = new Map();
  const matCamino = materialGuia(new THREE.Color(COLOR_OBSTACULO), 0.5);

  function dibujarObstaculo(o, d) {
    let e = obstaculos.get(o.id);
    if (!e) {
      e = { campo: new Campo3D(escena, geoFlechaCampo, { tipo: "obstaculo", id: o.id }), camino: new Cinta(matCamino) };
      escena.add(e.camino.malla);
      obstaculos.set(o.id, e);
    }
    e.visto = true;
    e.campo.actualizar(o, o.pos.x, o.pos.y, o.radio, COLOR_OBSTACULO, d.realceObs(o), 1, 0.13);

    e.camino.malla.visible = o.mov !== "fijo";
    if (e.camino.malla.visible) {
      const clave = [o.mov, o.ancla.x, o.ancla.y, o.angulo, o.largo, o.orbita].join("|");
      if (clave !== e.camino.clave) {
        let pts;
        if (o.mov === "recta") {
          const a = o.angulo * Math.PI / 180, h = o.largo / 2;
          pts = [[o.ancla.x - Math.cos(a) * h, o.ancla.y - Math.sin(a) * h],
                 [o.ancla.x + Math.cos(a) * h, o.ancla.y + Math.sin(a) * h]];
        } else if (o.mov === "lemniscata") {
          pts = [];
          for (let i = 0; i <= 120; i++) pts.push(trayectoriaLemniscata(2 * Math.PI * i / 120, [o.ancla.x, o.ancla.y], o.orbita, 1));
        } else {
          pts = circulo(o.ancla.x, o.ancla.y, o.orbita, 96);
        }
        // El ancla, como cruz: es el asa del camino.
        const ax = o.ancla.x, ay = o.ancla.y;
        const polis = discontinuar(pts, GUION_CAMINO[0], GUION_CAMINO[1])
          .concat([[[ax - 7, ay], [ax + 7, ay]], [[ax, ay - 7], [ax, ay + 7]]]);
        e.camino.fijar(clave, polis, 2, RELIEVE.camino);
      }
    }
  }

  // ---- formacion ----
  const formacion = new Cinta(materialGuia(0x6B6F76, 1));
  escena.add(formacion.malla);
  const geoVertice = new THREE.CylinderGeometry(4, 4, 3, 16).rotateX(Math.PI / 2).translate(0, 0, 1.5);
  const vertices = [0, 1, 2].map(() => {
    const m = new THREE.Mesh(geoVertice, formacion.malla.material);
    escena.add(m);
    return m;
  });

  // ---- camara ----
  function distanciaParaVer() {
    const t = Math.tan(THREE.MathUtils.degToRad(camara.fov / 2));
    return Math.max((H / 2) / t, (W / 2) / (t * camara.aspect)) * 1.06;
  }

  function vista(nombre) {
    const dist = distanciaParaVer();
    controles.target.set(0, 0, 0);
    if (nombre === "cenital") {
      // Un pelo fuera de la vertical: justo encima, el "arriba" de la
      // camara y su direccion coinciden y la orientacion queda indefinida.
      camara.position.set(0, -dist * 0.001, dist);
    } else {
      const polar = THREE.MathUtils.degToRad(50);
      const d2 = dist * 1.12;
      camara.position.set(0, -d2 * Math.sin(polar), d2 * Math.cos(polar));
    }
    camara.lookAt(controles.target);
    controles.update();
  }

  function ajustar() {
    const r = contenedor.getBoundingClientRect();
    const w = Math.max(1, Math.round(r.width)), h = Math.max(1, Math.round(r.height));
    renderer.setSize(w, h);
    etiquetas.setSize(w, h);
    camara.aspect = w / h;
    camara.updateProjectionMatrix();
  }

  // ---- raton -> mundo ----
  const rayo = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const puntoPiso = new THREE.Vector3();
  function lanzarRayo(clientX, clientY) {
    const r = renderer.domElement.getBoundingClientRect();
    ndc.set((clientX - r.left) / r.width * 2 - 1, -(clientY - r.top) / r.height * 2 + 1);
    rayo.setFromCamera(ndc, camara);
  }

  // Donde corta el rayo del raton un plano horizontal: el piso, o el de la
  // altura a la que se agarro algo. Sea cual sea, es horizontal: arrastrar
  // con esto solo puede mover en XY. Agarrado por arriba, el plano a esa
  // altura hace que la pieza siga al cursor en vez de resbalar en
  // perspectiva. null si el rayo apunta por encima del horizonte.
  const planoAgarre = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  function aMundo(clientX, clientY, altura) {
    lanzarRayo(clientX, clientY);
    planoAgarre.constant = -(altura || 0);
    const p = rayo.ray.intersectPlane(altura ? planoAgarre : PISO, puntoPiso);
    return p ? { x: p.x, y: p.y } : null;
  }

  // Lo mas cercano a la camara bajo el cursor, entre los cuerpos de los
  // robots y -si su pestaña esta a la vista- los solidos de los obstaculos.
  // Hace falta porque son altos: picarle a la parte de arriba de algo toca
  // el piso bastante detras de el.
  function pick(clientX, clientY, conObstaculos) {
    lanzarRayo(clientX, clientY);
    const objetivos = [];
    entradas.forEach(e => { if (e.pose.visible) objetivos.push(e.proxy); });
    if (conObstaculos) obstaculos.forEach(e => { if (e.campo.grupo.visible) objetivos.push(e.campo.solido); });
    const hits = rayo.intersectObjects(objetivos, false);
    return hits.length ? { ...hits[0].object.userData.pick, altura: hits[0].point.z } : null;
  }

  // ---- un cuadro ----
  const limite = (v, a) => Math.max(-a, Math.min(a, v));
  function dibujar(d) {
    const vivos = new Set(d.robots.map(r => r.id));
    d.robots.forEach(r => dibujarRobot(r, d));
    entradas.forEach((e, id) => { if (!vivos.has(id)) ocultarRobot(e); });

    obstaculos.forEach(e => { e.visto = false; });
    d.obstaculos.forEach(o => dibujarObstaculo(o, d));
    obstaculos.forEach(e => {
      if (e.visto) return;
      // Borrado desde el panel.
      e.campo.ocultar();
      e.camino.malla.visible = false;
    });

    formacion.malla.visible = !!d.formacion;
    vertices.forEach(m => { m.visible = !!d.formacion; });
    if (d.formacion) {
      const v = d.formacion;
      formacion.fijar(null, discontinuar(v.concat([v[0]]), 10, 8), 2.4, RELIEVE.formacion);
      v.forEach((p, i) => { if (vertices[i]) vertices[i].position.set(p[0], p[1], 0); });
    }

    // El centro de la orbita no se sale de la cancha: desplazando con el
    // boton del medio es facil perderla de vista y no encontrar el camino
    // de vuelta.
    const t = controles.target;
    t.set(limite(t.x, W / 2), limite(t.y, H / 2), 0);
    controles.update();
    renderer.render(escena, camara);
    etiquetas.render(escena, camara);
  }

  ajustar();
  vista("perspectiva");

  return { lienzo: renderer.domElement, ajustar, aMundo, pick, dibujar, vista };
}
