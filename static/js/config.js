// config.js - registro de robots y constantes compartidas.
"use strict";

// Un robot por entrada. Dos banderas distintas, que es facil confundir:
//
//   backend:  quien calcula su fisica. true = servidor_sandbox.py via
//             /ciclo (solo Atlas); false = el navegador, con control.js.
//   hardware: si hay un robot de verdad al otro lado. Decide quien se
//             puede seleccionar en modo Real y a quien se le puede mandar
//             PWM crudo desde el modo Manual.
//
// No son la misma cosa: Odiseo tiene hardware y no tiene backend. Cuando
// eran lo mismo (Atlas y nadie mas) daba igual; ya no.
const ROBOTS = [
  // El "adelante" del robot es SIEMPRE su eje +x local: es la direccion en
  // la que lo mueve la fisica (avanzarAtlas / uniciclo). Lo que cambia por
  // robot es como viene orientado su PNG.
  //
  // spriteOffset: cuanto hay que girar la imagen para que su frente visual
  //   caiga sobre ese eje +x. atlas.png se roto 90 grados al recortarlo,
  //   asi que su frente apuntaba 90 grados afuera del avance real.
  // Atlas va en cobre, el de su chasis: el rojo queda solo para PARO.
  { id: "atlas", nombre: "ATLAS", tipo: "holonomico", color: "#D9480F",
    img: "assets/atlas.png", backend: true, hardware: true, escalaImg: 1,
    spriteOffset: -Math.PI / 2,
    // ArUco pegado a cada robot: 1 Atlas, 2 Apolo, 3 Odiseo. Es el valor
    // de arranque; se cambia desde el panel de Control sin tocar codigo.
    tagAruco: 1,
    // El campo propio se queda en lineal_norm y no pasa a espiral como los
    // obstaculos: entre robots un espiral los pondria a orbitarse: aqui lo
    // que se quiere es que se aparten, no que se rodeen. Ver crearRobot().
    // Radio del campo repulsivo propio (pestaña Robots). Va por robot
    // porque los tres NO miden lo mismo: el ancho dibujado es
    // ancho_robot_px * 1.35 * escalaImg, o sea 338 px para Atlas, 183 para
    // Odiseo y 122 para Apolo. Un campo con el mismo radio para los tres
    // seria enorme alrededor de Apolo y estaria metido dentro del chasis de
    // Atlas. Estos numeros dejan el circulo pegado al contorno de cada uno.
    radioRepulsor: 170,
    spawn: { x: -350, y: -180, th: 0 } },
  // Apolo ya contesta: el 2026-09-08 se le puso apolo-pi, su agente se
  // suscribe a <prefijo>/robot1/cmd y el ESP32 maestro devuelve
  // ARMED. De ahi el hardware:true.
  //
  // Ojo, a medias: el escaneo I2C del maestro sale vacio, asi que su
  // unico esclavo no responde y solo tiene la rueda local. Se mueve,
  // pero girando sobre si mismo.
  { id: "apolo", nombre: "APOLO", tipo: "uniciclo", color: "#2E6FF2",
    img: "assets/apolo.png", backend: false, hardware: true, escalaImg: 0.361,
    // Mismo cuarto de vuelta que Odiseo: en el CAD las ruedas estan sobre su
    // eje X y en apolo.png arriba y abajo. El SIGNO no se pudo sacar del
    // modelo, que es simetrico sin la baliza ni los botones del PNG: si en
    // 3D sale de reversa, esto va en -Math.PI / 2.
    malla3d: { src: "assets/modelos/apolo.glb", giro: Math.PI / 2 },
    tagAruco: 2,
    radioRepulsor: 80,
    spawn: { x: 250, y: 220, th: Math.PI } },
  // Odiseo es el que antes se llamaba APOLO-2. Mismo tipo (diferencial de
  // dos ruedas) pero otro robot: chasis mas grande, NEMA por PUL/DIR y
  // reduccion de banda. Su firmware es esp32c3_odiseo_serial.py y contesta
  // en robot2, probado de punta a punta por MQTT.
  //
  // escalaImg sale de medirlo contra Apolo: es 1.5 veces mas grande, y en
  // canvas.js el ancho dibujado es directamente proporcional a este
  // numero (w = S * 2 * 1.35 * escalaImg), asi que 0.361 * 1.5.
  { id: "odiseo", nombre: "ODISEO", tipo: "uniciclo", color: "#8B3FE3",
    img: "assets/odiseo.png", backend: false, hardware: true, escalaImg: 0.5415,
    // Modelo para la vista 3D. "giro" hace con el GLB lo que spriteOffset
    // con el PNG: lleva su frente al eje +x. Medido contra odiseo.png, que
    // tiene las ruedas arriba y abajo y los motores en las esquinas
    // superior derecha e inferior izquierda; en el CAD las ruedas estan
    // sobre su eje X, de ahi el cuarto de vuelta. Atlas y Apolo aun no
    // tienen: en 3D salen como tarjeta con su PNG hasta que llegue el suyo.
    malla3d: { src: "assets/modelos/odiseo.glb", giro: Math.PI / 2 },
    tagAruco: 3,
    radioRepulsor: 120,
    spawn: { x: -250, y: 220, th: Math.PI / 2 } },
];

// Escala de la vista 3D, la MISMA para todos los modelos. Los GLB vienen en
// metros con la medida real del CAD, asi que no se escalan uno por uno: se
// pasan a px de mundo con este factor y cada robot sale del tamaño que
// tiene de verdad respecto de los otros.
//
// El numero sale de Odiseo, que es el robot con la escala 2D mejor medida:
// 0.740 m de largo en el CAD contra 182.8 px de su sprite (250 * 1.35 *
// escalaImg). Con el, Apolo (0.449 m) mide 111 px en 3D y 122 en 2D: su
// escalaImg es ~10% generosa. No se toca aqui porque en 2D ese numero es
// tambien el radio de colision.
const PX_POR_METRO_3D = 247;

// Marcadores compartidos. Los dos PNG llevan la MISMA flecha en la misma
// posicion dentro de su lienzo (real.png es marca.png sin el circulo), asi
// que dibujandolos centrados y del mismo tamaño, cuando el robot alcanza la
// pose deseada su flecha negra queda calzada dentro del circulo de color.
const MARCA_REAL = "assets/real.png";    // pose actual, fijada al robot
const MARCA_DESEADA = "assets/marca.png"; // pose objetivo, de referencia
const COLOR_MARCA_REAL = "#14161A";       // el real va en negro para todos
// Sobre el video el destino va casi blanco: el piso del laboratorio es
// oscuro y el marcador en el color del robot -cobre, azul, morado- se
// pierde encima. Casi blanco y no blanco puro, que sobre el reflejo de la
// lampara cenital el blanco puro desaparece igual de bien.
const COLOR_DESTINO_VIDEO = "#F2F4F8";
// Lado de la caja del marcador, como fraccion de S (que vale igual para los
// tres robots, asi que los marcadores salen del mismo tamaño en todos).
//
// Dentro de su lienzo la flecha ocupa 0.276 y el anillo 0.799, asi que este
// numero los escala a los DOS a la vez: no se pueden ajustar por separado
// sin romper el calce entre la flecha real y el circulo deseado. Subirlo
// hace la flecha mas legible sobre el chasis a costa de un anillo mas
// grande en el destino.
const TAM_MARCA = 0.70;

// El destino ya no se dibuja con marca.png (flecha + anillo de una pieza)
// sino con real.png -la flecha sola- tintada del color del robot, y el
// anillo aparte. La razon es que el anillo dejo de ser adorno: su radio ES
// la zona de llegada, la que mueve el slider del panel, asi que tiene que
// poder crecer y encoger sin arrastrar la flecha con el.
//
// La flecha se queda en TAM_MARCA, igual que la del robot: las dos dicen lo
// mismo -una pose- y al llegar tienen que calzar exactamente una sobre otra.
// Eso solo funciona si miden igual.
const TAM_CENTRO_TRAYECTORIA = 6.0;

// ---------------------------------------------------------------
// OBSTACULOS
// ---------------------------------------------------------------
// Circulos que empujan al robot. No son robots ni destinos: son campos, y
// viven aparte de los dos -son globales, como la formacion, no por robot-.
const MAX_OBSTACULOS = 3;

// Grafito. Un obstaculo no es de nadie, asi que no lleva color de robot, y
// el rojo queda para PARO. Lo que dice "esto repele" es la forma -circulo
// punteado lleno de flechas-, no el color. Sobre el video va casi blanco.
//
// El campo que lleva encima un ROBOT va en el color de ese robot: ahi lo
// que hay que poder leer de un vistazo es de quien es.
const COLOR_OBSTACULO = "#374151";

// Intensidad con la que arranca el campo de un robot al encenderlo con su
// interruptor. En 0 parecia que el interruptor no hacia nada.
const ETA_REPULSOR_ON = 600;
const COLOR_OBSTACULO_VIDEO = "#F3F4F6";

// Tope del riel de intensidad. Vive aqui porque ademas de acotar el mando
// sirve de fondo de escala para dibujar: el grosor del trazo sale de
// eta/ETA_MAX, asi que subir la intensidad se NOTA en el dibujo y no solo
// en como se mueve el robot.
const ETA_MAX = 5000;

// Ya no hay un techo fijo: la fuerza se recorta a la PROPIA intensidad del
// campo (ver fuerzaDeCampo). El invariante que da es facil de explicar y de
// creer -ningun campo empuja mas fuerte que el numero que le pusiste-, y
// ademas hace que el riel signifique lo mismo con las cuatro formulas.
//
// Antes era un 660 fijo, y con el riel llegando a 3000 la mitad de arriba
// no habria hecho nada: cualquier valor por encima de 660 daba exactamente
// el mismo empujon.
//
// Sigue habiendo suelo en cero: "espiral" y "khatib" estan escritas en
// formacion.py para radios menores que 1 y en pixeles cambian de signo. Un
// repulsor que atrae no es un repulsor.

// Cuanto se tuerce el empuje del repulsor "espiral", respecto de la linea
// que sale del centro. 0 seria empujar derecho hacia afuera -o sea, los
// otros tres-; 90 grados seria girar alrededor sin alejarse nunca.
//
// Este es el arreglo de un fallo real, no un adorno. La formula de espiral
// de formacion.py -eta*(1 - r^4/d)- esta escrita para radios menores que 1;
// en pixeles r^4 son mil millones y el resultado sale enorme y NEGATIVO,
// asi que el recorte a cero de fuerzaDeCampo() lo dejaba en nada:
// elegir "espiral" no hacia absolutamente nada. Y un espiral radial no es
// un espiral. Aqui se implementa por lo que es -alejar Y rodear-, con la
// magnitud del lineal normalizado, que es la que si responde en pixeles.
//
// De paso resuelve el minimo local: de frente, un repulsor puramente radial
// empuja justo hacia atras y el robot se queda clavado. Con el giro, no.
const GIRO_ESPIRAL = 62 * Math.PI / 180;

// Flechas de los campos repulsivos, compartidas por canvas.js y escena3d.js.
// Tres coronas de flechas, con mas flechas cuanto mas afuera. No es
// capricho: en una corona el hueco entre flechas crece con el radio, asi
// que repartir siempre las mismas dejaba el centro apelmazado y el borde
// vacio. Con 5/10/15 la densidad angular queda pareja y el circulo se lee
// como un campo lleno y no como una rueda de radios.
//
// Cada corona va girada media separacion respecto de la de dentro, para
// que las flechas no se alineen en radios y formen una estrella.
//
// La de fuera va en 0.80 y no en 0.88: la flecha mide hasta 0.19 R, asi
// que naciendo en 0.88 la punta caia en 1.07 R y las flechas de "lineal"
// -que son las tres de largo maximo- arrancaban ya asomando fuera del
// circulo. Apuntar fuera del alcance es justo lo contrario de lo que el
// circulo dice. Que la animacion las saque un pelo al respirar da igual;
// lo que no puede ser es que nazcan fuera.
const CORONAS = [
  { radio: 0.28, n: 5 },
  { radio: 0.54, n: 10 },
  { radio: 0.80, n: 15 },
];
// El campo de un robot lleva mas flechas que el de un obstaculo: es el
// unico de los dos que viaja pegado a un chasis, y con la densidad de un
// obstaculo se leia como un halo suelto en vez de como algo suyo.
const DENSIDAD_ROBOT = 1.4;
// Largo maximo de la flecha, como fraccion del radio. Con 15 flechas en la
// corona de fuera el hueco entre dos vecinas es 0.37 R, asi que de aqui no
// se puede subir sin que se toquen.
const LARGO_FLECHA = 0.19;
// Y este es el minimo, para que una corona donde el campo casi no empuja
// se vea como un muñon y no como nada: "no llega aqui" y "no hay flecha
// dibujada" no son lo mismo.
const LARGO_MINIMO = 0.28;

// Donde nace cada obstaculo nuevo. Tres sitios fijos y separados: naciendo
// todos en el centro, el segundo aparecia debajo del primero y parecia que
// el boton no habia hecho nada.
const CUNA_OBSTACULOS = [
  { x: 0, y: 0 }, { x: 320, y: -170 }, { x: -320, y: 170 },
];

// Cuanto crece la pieza que esta bajo el cursor. Es la señal de que "esto
// es lo que vas a agarrar": el marcador de destino y el chasis se solapan
// seguido, y sin realce no hay forma de saber cual de los dos responde.
const FACTOR_HOVER = 1.05;

const PERIODO_MS = 50;   // cadencia del lazo de control (20 Hz), igual para todos
const TIMEOUT_MS = 2500; // si Atlas no contesta /ciclo en este tiempo, se calla

// Cola de rastro: se guardan las ultimas N muestras y las mas viejas se
// van tirando, igual que el trail_length de los scripts de Vicon del repo
// original. El dibujo muestra el movimiento RECIENTE, no el historico
// completo - con el historico la pantalla se llena de lineas viejas y deja
// de leerse hacia donde va el robot ahora.
const MUESTRAS_RASTRO = 1200; // tope de seguridad, no el criterio principal
const PASO_RASTRO_PX = 10;    // distancia minima entre muestras guardadas
// Criterio real de caducidad: la EDAD de la muestra. Con un tope por cuenta
// nada mas, un robot que llega al destino deja de generar muestras y su
// cola se congela: la linea se queda pintada para siempre. Por tiempo, se
// va recortando sola hasta desaparecer.
const VENTANA_RASTRO_S = 18;

// Graficas de telemetria: se guarda un historial largo pero se dibuja solo
// la cola reciente, que es donde se ve la dinamica del lazo.
const VENTANA_BUFFER_S = 30;      // cuanto historial se conserva
const VENTANA_VISIBLE_S = 3;      // cuanto se dibuja en pantalla
const PERIODO_MUESTREO_S = 0.05;  // 20 Hz, igual que el lazo de control

// Escala de velocidad por defecto.
//
// Al 45% Atlas topaba en 113 px/s contra los 220 de Apolo y Odiseo -la
// mitad justa-, y en el banco eso no se leia como "va mas despacio" sino
// como que se dejaba empujar: cualquier campo lo apartaba antes de que le
// diera tiempo a responder. Al 100% son 250 px/s, o sea un pelo por encima
// de los uniciclo, y los tres juegan en la misma liga.
//
// CUIDADO: esta escala NO es solo del dibujo. Es el factor con el que sale
// la trama por MQTT, asi que con la casilla armada el Atlas de verdad
// arranca a plena potencia. Para las primeras pruebas en piso, bajalo antes
// de armar (el README del profesor recomienda 0.2-0.3).
//
// Y por debajo de escala_minima_util() -que /config reporta como
// escala_minima, ~39%- la zona muerta apaga las ruedas secundarias y Atlas
// solo se mueve en las 8 direcciones canonicas. Ver la nota larga en
// cinematica.py::escala_minima_util().
const ESCALA_DEFECTO = 1.0;

// Lado maximo al que se reduce el sprite de Atlas antes de dibujarlo (los
// demas se escalan por su escalaImg). Cubre el tamaño real en pantalla
// incluso en pantallas retina, sin arrastrar los ~1450 px del PNG.
const LADO_SPRITE = 800;
