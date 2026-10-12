# AGV Atlas — visión, cinemática y control

Módulo de visión y control para **Atlas**, el robot omnidireccional de cuatro
ruedas mecanum.

Cierra el lazo: una cámara cenital detecta marcadores ArUco, calcula a dónde
tiene que ir el robot y le manda las cuatro velocidades de rueda por MQTT.

---

## Guía rápida — levantar el sitio

Acabas de clonar el repo y quieres ver el banco de pruebas en el navegador.
Son cuatro pasos.

**1. El servidor se ejecuta desde esta carpeta**, no desde la raíz del
repositorio:

```bash
cd Atlas_Control
```

**2. Instala las dependencias.** Una sola vez:

```bash
pip install numpy
```

Y solo si vas a usar la cámara (modo Real):

```bash
pip install opencv-python opencv-contrib-python
```

**3. Levanta el servidor:**

```bash
python servidor_sandbox.py                  # con cámara
python servidor_sandbox.py --sin-camara     # sin hardware, solo simulación
python servidor_sandbox.py --puerto 8080    # si el 8000 está ocupado
```

**4. Abre <http://localhost:8000>** en el navegador. Para apagarlo, `Ctrl+C`
en esa terminal.

El mismo servidor **sirve la página**, así que no hay que abrir el HTML a mano
ni pelearse con CORS. Si no hay cámara conectada, `--sin-camara` arranca igual
y la página se queda en Simulación.

### Lo primero que verás

| En la página                | Qué es                                                          |
|-----------------------------|------------------------------------------------------------------|
| **Simulación / Real / Filtros** | Dentro del visor, arriba a la izquierda: en Simulación se ve el mapa (2D o 3D, arriba a la derecha), en Real el video |
| **Barra de arriba**         | Lo que vale para los tres robots: marcha (cronómetro, Correr/Pausar, reiniciar), el switch **MQTT** y **PARO**, siempre a la vista |
| **MQTT**                    | El switch que arma a los robots físicos, los tres a la vez. Arranca apagado |
| **Menú inferior**           | Las tres páginas del panel: **Robots**, **Obstáculos** y **Telemetría**. El botón redondo de al lado abre los **Ajustes rápidos** |
| **Las tres fichas**         | En Robots: a qué robot le estás hablando, Atlas (cobre), Apolo (azul) u Odiseo (violeta). La elegida lleva aro |
| **Modo de control**         | Cuatro fichas: Manual, Punto, Trayectoria o Formación. El ArUco del robot va en el mismo renglón |
| **Repulsión del robot**     | Plegada, con su interruptor en la cabecera. Al encenderlo se abre y arranca con intensidad 600; apagarlo la deja en 0 |
| **Error permitido**         | En Ajustes rápidos: las zonas de llegada (10–40 px) y rumbo (1–15°), comunes a los tres |
| **Pantalla angosta**        | Bajo 1100 px el panel adelgaza y la barra queda en iconos; bajo 720 px (teléfono) todo se apila sin scroll de página y un botón redondo en el visor lo agranda o achica |
| **Telemetría**              | Las cuatro gráficas del robot elegido, Sistema (solo Atlas) y la grabación a CSV |

El **aro punteado** alrededor de cada destino no es adorno: es la *zona de
llegada*, y su radio sale del slider Llegada (Ajustes rápidos → Error permitido). Ahí es donde
el lazo da el punto por bueno y se calla. Bajándolo (y con él la zona de
rumbo) el robot converge tan fino como en Formación, que no tiene zona de
llegada ninguna.

En **Formación** la figura ya no está escrita en el código: se dibuja
arrastrando los tres puntos sobre una rejilla en el propio panel, y la
brújula de al lado gira la formación entera. Los dos mandos son
independientes: la rejilla enseña la figura **sin girar** y la brújula el
ángulo por separado; el efecto compuesto se ve en la cancha, no en el panel. El rótulo dice qué figura
quedó (Triángulo, Cuña, En línea) y con qué orientación. La *Separación*
sigue valiendo lo que dice —píxeles entre robots— salga la figura que salga,
porque se normaliza por la distancia media entre parejas.

La repulsión entre robots **ya no vive en Formación**: la hace el campo
propio de cada robot, que vale en los cuatro modos y está siempre a la
vista al fondo del panel Robots. Tener las dos era la
misma idea escrita dos veces, sumándose sin que se viera de dónde salía cada
empujón.

En **Obstáculos** se ponen hasta tres círculos repulsivos. Son globales -los
sufren los tres robots- y se arrastran por el mapa con esa pestaña abierta.
Cada uno lleva radio de acción (30-150 px), intensidad, tipo de repulsión y
movimiento: quieto, vaivén sobre una recta, órbita circular u ocho. Lo que empuja un campo es el **chasis** del robot, no su centro: el empujón
arranca justo cuando el contorno cruza el círculo punteado que se ve dibujado.
**Manda
mucho más el radio que la intensidad**: medido con Apolo cruzando la cancha,
subir el radio de 90 a 150 px triplica el desvío, mientras que subir η de
400 a 1000 lo mueve de 25 a 36 px. El normalizado vale cero en el borde y
solo aprieta hondo adentro, así que un círculo chico casi no da tiempo a
girar. Las
flechas de dentro dicen qué clase de campo es, y se ponen a marchar cuando
hay un robot dentro y la simulación corre.

Cada **robot** puede llevar además su propio campo repulsivo, clavado a su
centro y viajando con él: se configura en la pestaña Robots, en la sección
plegable *Repulsión del robot* del fondo (radio, intensidad y tipo). El radio
va de 60 a 230 px y arranca distinto para cada uno —170 Atlas, 120 Odiseo, 80
Apolo— porque los tres no miden lo mismo: el ancho dibujado es 338, 183 y 122
px respectivamente, y esos radios dejan el círculo pegado al contorno de cada
chasis. Empuja a
los **otros robots**; a los obstáculos no, porque su movimiento está
programado y no es física que se pueda desviar. Arranca en intensidad 0 —o
sea apagado—, así que mientras nadie lo suba nada cambia. Se dibuja en el
color de su robot y no en rojo: ahí lo que hay que leer no es "esto repele"
sino de quién es.

### Los cuatro repulsores

Si levantas la fuerza sobre el círculo y lo giras alrededor de su eje, cada
fórmula es un sólido. Ese sólido es el icono de su ficha.

| | Figura | Perfil (fracción de η a 30 / 50 / 70 / 90 % del radio) |
|---|---|---|
| **Lineal** | **Cilindro** — satura en todo el radio y se corta en seco en el borde | 1.00 / 1.00 / 1.00 / 1.00 |
| **Lineal +** | **Cono** — rampa recta del centro al borde | 0.70 / 0.50 / 0.30 / 0.10 |
| **Espiral** | El mismo cono, pero empujando de lado 62°: rodea | 0.70 / 0.50 / 0.30 / 0.10 |
| **Khatib** | **Campana con la punta cortada** — el techo de η le corta el pico | 1.00 / 1.00 / 0.18 / 0.01 |

Lineal y Lineal + **son la misma curva**: `lineal = lineal_norm × radio`, exacto,
a cualquier distancia. Con el recorte a η eso se traduce en muro contra rampa.

Las flechas dibujadas miden lo que empuja el campo *ahí*, así que el perfil de
la tabla se lee del propio dibujo sin abrir el panel.

De los cuatro repulsores, **Espiral** es el único que además de alejar
*rodea*, y se le elige el sentido. Es también el único que saca al robot de
un choque frontal: los otros tres empujan derecho hacia atrás, así que
entrando de frente al centro no hay componente lateral y el robot se queda
clavado a media distancia. Medido en el banco, con radio 150 y η 700:
lineal normalizado se planta a 62 px y no llega nunca; espiral rodea con 79
px de desvío y llega, por abajo en antihorario y por arriba en horario.

Con Atlas es aún más marcado, porque su chasis mide 338 px de ancho y para un
cuerpo así casi cualquier acercamiento es "de frente": con Lineal + se planta
a 250 px y no rodea nunca; con Espiral se desvía 158 px y pasa. Por eso los
obstáculos nuevos salen en Espiral. El campo propio de un robot se queda en
Lineal +: entre robots un espiral los pondría a orbitarse, y ahí lo que se
quiere es que se aparten.

En Manual se conduce con las flechas del teclado (← ↑ ↓ →; dos a la vez dan
diagonales o curvas, `Q` y `E` giran, y al soltar sale un cero). El paro de
emergencia es el botón **PARO** de la barra de arriba, o `Esc` / la barra espaciadora.

> **Ojo con la escala de velocidad.** Arranca al **100 %**, no al 45 % de
> antes: al 45 % Atlas topaba en 113 px/s contra los 220 de Apolo y Odiseo, y
> en el banco eso no se leía como "va más despacio" sino como que se dejaba
> empujar. Pero esa escala **no es solo del dibujo**: es el factor con el que
> sale la trama por MQTT, así que con la casilla armada el Atlas de verdad
> arranca a plena potencia. Bájala antes de armar para las primeras pruebas
> en piso.

> **La casilla de MQTT es lo único que mueve al robot de verdad.** Arranca
> desmarcada y es **una para los tres**: arma y desarma la flota entera. No pide estar en Real ni en Manual: arma
> igual mirando el mapa, y cambiar de vista no la desarma.
>
> En **Punto, Trayectoria y Formación** no basta con marcarla: mientras la
> cámara no vea el ArUco de ese robot no sale ni una orden, porque calcular
> el control desde una pose inventada es justo como se estrella un robot. En
> **Manual** sí sale: ahí mandas tú, no una ley de control.
>
> Con algo armado, el botón **PARO** está a la vista también en Simulación.

### Vista 3D

En Simulación, el conmutador **2D | 3D** de arriba a la derecha del visor
cambia a la misma cancha en 3D. No es otra simulación: el estado, la física y
el ratón son los de siempre, solo cambia quién dibuja. Se recuerda al
recargar, y en Real/Filtros desaparece porque ahí se mira el video.

| En 3D                  | Cómo                                                  |
|------------------------|-------------------------------------------------------|
| Mover un robot, un destino, un obstáculo | Clic izquierdo y arrastrar, igual que en 2D. Solo se mueve en XY: el punto sale de cortar el rayo del ratón con un plano horizontal |
| Girar                  | **Shift** + arrastrar                                 |
| Orbitar la cámara      | Clic derecho y arrastrar                              |
| Desplazar / acercar    | Clic central / rueda                                  |
| Volver a una vista conocida | Botones **Cenital** y **Perspectiva**            |

Las guías (destino, zona de llegada, referencia, rastro, cruz del centro,
formación) van en **relieve bajo**, pintadas con grosor sobre el piso. Los
**campos repulsivos sí se levantan**: cada uno es el sólido de su fórmula
—cilindro, cono, cono con meridianos torcidos, campana— con el perfil sacado
de la misma `magnitudCampo()` que usa la física, y su alto crece con η.

**Modelos.** Un robot con `malla3d` en `static/js/config.js` sale con su GLB;
los demás, con su PNG flotando como tarjeta. Hoy tienen modelo Odiseo y Apolo
(`static/assets/modelos/`); Atlas sigue en tarjeta.

Los modelos **no se escalan uno por uno**: glTF va en metros con la medida
real del CAD, y todos pasan a px con el mismo `PX_POR_METRO_3D` (247, sacado
de Odiseo: 0.740 m contra los 182.8 px de su sprite). Así cada robot guarda
en 3D su tamaño de verdad respecto de los otros. Lo que sí se corrige es el
origen —se centra en su planta y se apoya en el piso— y `giro`, el cuarto de
vuelta que lleve su frente a +x, igual que `spriteOffset` con el PNG.

Para sumar uno: exportar el GLB del CAD, pasarlo por
`tools/optimizar_glb.mjs` (instrucciones en su cabecera) y agregar
`malla3d: { src, giro }` a su entrada. Sin optimizar, un CAD sale pesado y con
cientos de piezas sueltas: Apolo pasó de 21.3 MB y 672 llamadas de dibujo a
2.2 MB y 2; Odiseo, de 6.1 MB y 64 a 0.97 MB y 4. Van comprimidos con
meshopt, así que el cargador necesita su decodificador (ya vendorizado).

three.js va **vendorizado** (r170) en `static/vendor/three/`, así que funciona
sin internet. El módulo 3D (`static/js/escena3d.js`) solo se descarga la
primera vez que se abre la vista.

### La cámara

Se elige **desde la página**, en modo Real: el selector de arriba a la
izquierda busca las que estén conectadas y cambia de una a otra sin reiniciar
el servidor. Si no abre ninguna al arrancar, no pasa nada: se queda esperando
a que elijas. `listar_camaras.py --ver` hace lo mismo desde la consola.

En Windows, con `pip install pygrabber` la lista sale con el nombre real de
cada cámara en vez de solo el índice. Es opcional; OpenCV no expone el nombre
por ninguna otra vía.

Para el modo Real necesitas los marcadores impresos: `Aruco_0-5.pdf` del
repositorio del profesor (`4_Python_QR_aruco/`), al 100 % y en papel mate.

---

## Qué hace esto

```
cámara → detección ArUco → pose (x, y, θ) → ley de control →
cinemática inversa mecanum → trama de 4 velocidades → MQTT → Atlas
```

El marcador **ID 0** identifica al robot y el **ID 1** marca el destino, con la
orientación con la que debe llegar.

Trae además un **banco de pruebas** que sustituye a Atlas por una simulación en
el navegador, para validar todo el lazo sin arriesgar hardware.

---

## Los otros programas

Además del sandbox, cada archivo del núcleo se puede correr solo para
comprobar que hace lo que dice:

```bash
python vision.py --camara 1        # ¿detecta el marcador?
python listar_camaras.py --ver     # qué índice es cada cámara
python cinematica.py               # pruebas de la cinemática mecanum
python control.py                  # pruebas de la ley de control
python formacion.py                # pruebas de los repulsores
```

---

## Los archivos

### Núcleo — no se ejecutan solos, los importan los demás

| Archivo          | Qué contiene                                                   |
|------------------|----------------------------------------------------------------|
| `vision.py`      | Detección ArUco y pose `(x, y, θ)` de cada marcador            |
| `cinematica.py`  | Cinemática mecanum, inversa y directa. Geometría de Atlas      |
| `control.py`     | Ley de convergencia, uniciclo (diferencial) y holonómico       |

Los tres traen pruebas propias: córrelos directamente para verificarlos.

### Se ejecutan solos

| Archivo               | Para qué                                                        |
|-----------------------|------------------------------------------------------------------|
| `servidor_sandbox.py` | Sirve el sandbox y corre el control. **El más completo**         |
| `listar_camaras.py`   | Encontrar el índice de la cámara y su backend                    |
| `formacion.py`        | Repulsores y control de formación por grafo completo (referencia) |

### Y una carpeta

| Ruta       | Qué es                                                              |
|------------|----------------------------------------------------------------------|
| `static/`  | La planta simulada, en el navegador. HTML/CSS/JS que sirve `servidor_sandbox.py` |
| `static/vendor/three/` | three.js r170 y los addons que usa la vista 3D, sin tocar     |
| `_vendor/` | El código del profesor, sin tocar. Referencia, no se importa        |

---

## Documentación

- Los agentes de las Raspberry, el firmware de los robots y el sistema MQTT
  viven en el repositorio de trabajo del laboratorio, que es privado.
- **Configuración MQTT:** copia `.env.ejemplo` a `.env` (no se sube a git) y
  pon el broker, el puerto y el prefijo de temas de tu flota.

---

## Atlas, en números

| Dato                        | Valor                                 |
|-----------------------------|---------------------------------------|
| Ancho de vía (llanta 1 ↔ 2) | 840 mm                                |
| Entre-ejes (llanta 1 ↔ 3)   | 510 mm                                |
| Diámetro de llanta          | 210 mm                                |
| Configuración de rodillos   | **O**                                 |
| Autoridad de giro           | **24 %** de la que daría una config X |

Distribución vista desde arriba:

```
          FRENTE
    [1]------------[2]      1 = FL = motor local (maestro)
     |              |       2 = FR = esclavo 0x08
     |              |       3 = RL = esclavo 0x0A
    [3]------------[4]      4 = RR = esclavo 0x09
          ATRÁS
```

**El orden de la trama serial es `FL, FR, RR, RL`** — o sea `local, 0x08, 0x09,
0x0A`. No sigue ningún patrón geométrico obvio, así que no lo adivines.

Esto se **midió** en el robot el 2026-08-31, mandando una rueda a la vez y
anotando cuál giraba. Antes este README decía `RR, FR, FL, RL`: las
posiciones 0 y 2 estaban cruzadas, y con ese orden el robot se movía en
diagonal cuando se le pedía ir de frente. El maestro confirmó la posición 0
por su cuenta: con `100,0,0,0` respondió `LOCAL_SET=100`, y esa rueda resultó
ser la frontal izquierda.

El orden vive en un solo lugar, `FRAME_LAYOUT` en `cinematica.py`. El sandbox
lo lee desde `/config`, no lo duplica.

---

## Estado

**Funciona y está verificado**

- Detección ArUco, contrastada contra el `Camera.py` del profesor: mismos
  valores de X, Y y θ con el mismo marcador
- Cinemática mecanum inversa y directa, con pruebas
- Ley de control, con simulación que converge
- Sandbox en lazo cerrado navegador ↔ Python

**Pendiente**

- Medir la zona muerta real, hoy estimada en 30
- Acordar con el equipo de VR el topic y las unidades

> Esta sección va por detrás del repositorio: se escribió cuando el lazo real
> aún no andaba, y desde entonces la flota entera quedó conectada. Revísala
> antes de fiarte de ella.

---


Basado en el repositorio del profesor
[Robot-uniciclo-Ibero-2](https://github.com/HuberGiron/Robot-uniciclo-Ibero-2),
del que se conservan el diccionario ArUco (`DICT_6X6_50`), el criterio de
ángulo y la ley de convergencia. Su código original vive sin tocar en
`_vendor/`, como referencia.
