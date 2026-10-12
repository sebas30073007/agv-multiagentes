#!/usr/bin/env python3
"""
vision.py - Deteccion de robots con marcadores ArUco.

Sigue las convenciones del repo del profe (DICT_6X6_50, origen en el centro
de la imagen, angulo tomado de la arista inferior del marcador) para que el
resto de su codigo encaje sin cambios.

DOS MODOS DE USO

1. Generar marcadores para imprimir:

       python3 vision.py --generar 0 1 2 --tam-cm 15

   Crea marcador_00.png, marcador_01.png, etc. Imprimelos SIN escalar
   ("tamano real" / 100%) y en papel MATE. El papel brillante refleja las
   lamparas del techo y mata la deteccion.

2. Ver la deteccion en vivo:

       python3 vision.py
       python3 vision.py --camara 1

3. Como modulo, dentro de tu lazo de control:

       from vision import Vision
       vision = Vision(camara=0)
       poses = vision.leer()        # {0: (x, y, theta), 2: (...)}
       vision.cerrar()

UNIDADES Y EJES
Todo en PIXELES, con el origen en el centro de la imagen y la Y hacia ABAJO,
igual que el repo del profe. No hace falta homografia con una sola camara: el
lazo cerrado absorbe el factor de escala dentro de la ganancia del
controlador. Cuando metas mas camaras o quieras centimetros reales, ahi si
toca homografia.

Si mas adelante alimentas cinematica.py (el modulo de Atlas), cambia
Y_HACIA_ARRIBA a True: ese modulo espera un sistema diestro. Ver la nota
junto a la constante.
"""

import argparse
import math
import sys

try:
    import cv2
except ImportError:
    print("Falta OpenCV. Instala con:")
    print("  python3 -m pip install opencv-python opencv-contrib-python")
    sys.exit(1)

import numpy as np


RESOLUCION_X = 1280
RESOLUCION_Y = 720

# Indice de la camara. Cambia al reconectar el USB: si deja de funcionar
# sin haber tocado nada, vuelve a correr listar_camaras.py y actualiza.
CAMARA_POR_DEFECTO = 1

# DICT_6X6_50, confirmado el 2026-09-09 correlacionando el marcador
# rectificado contra TODOS los diccionarios en las cuatro rotaciones:
#   0.874  DICT_6X6_50   id=1
#   0.751  DICT_ARUCO_MIP_36h12 id=0
#   0.747  DICT_4X4_50   id=24   <- lectura erronea, no un hallazgo
# Ese id=24 de 4X4 era basura decodificada con ruido. El diccionario nunca
# fue el problema; el problema estaba en PARAMS_DETECCION, mas abajo.
DICCIONARIO = cv2.aruco.DICT_6X6_50

# Parametros de deteccion, medidos sobre 40 cuadros reales el 2026-09-09.
# Con los valores de fabrica de OpenCV este marcador NO se detecta nunca.
PARAMS_DETECCION = {
    # ESTE es el que resolvia todo. OpenCV lo trae en 4.
    #
    # Al leer los bits, el detector endereza el marcador y lo remuestrea a
    # perspectiveRemovePixelPerCell pixeles POR CELDA. Un 6X6 son 8x8 celdas
    # contando el borde, asi que con 4 se reduce el marcador a 32x32 px
    # antes de leer 36 bits. El marcador de Atlas mide ~66 px de lado desde
    # la cenital y llega con algo de desenfoque: a 32x32 la informacion de
    # cada bit ya se perdio, y el codigo sale mal o no sale.
    #
    # Con 12 remuestrea a 96x96 y los bits se leen enteros. Medido sobre 40
    # cuadros seguidos: con 4 no detecta NUNCA, con 12 detecta 40 de 40 sin
    # un solo falso positivo. Es la diferencia entre no funcionar y funcionar.
    "perspectiveRemovePixelPerCell": 12,
    "perspectiveRemoveIgnoredMarginPerCell": 0.13,

    # errorCorrectionRate se queda en el 0.6 de fabrica A PROPOSITO. Subirlo
    # a 1.0 tambien da 40/40, pero empieza a "corregir" manchas del piso
    # hasta convertirlas en marcadores validos: aparecian 2 a 5 falsos por
    # lote. Mas tolerancia al error no es mas deteccion, es mas invencion.
    "errorCorrectionRate": 0.6,

    # El minimo de perimetro se calcula como rate * max(ancho, alto) =
    # rate * 1280. El 0.03 de fabrica admite cuadrilateros de 38 px y las
    # motas de suciedad del piso (~96 px) pasaban y se decodificaban como
    # marcadores fantasma. 0.12 pide 154 px; el marcador de Atlas mide ~266
    # px de perimetro, asi que entra con margen y las motas quedan fuera.
    "minMarkerPerimeterRate": 0.12,

    # Rango ancho y paso fino de umbral adaptativo: cuesta tiempo, pero es
    # lo que rescata el marcador de la zona con reflejo de la lampara.
    "adaptiveThreshWinSizeMin": 3,
    "adaptiveThreshWinSizeMax": 45,
    "adaptiveThreshWinSizeStep": 4,

    # Perdona los bordes redondeados por el desenfoque sin llegar a aceptar
    # cualquier mancha con cuatro lados.
    "polygonalApproxAccuracyRate": 0.05,
}


def backend():
    """
    Backend de captura segun el sistema operativo.

    No es cosmetico. En Windows el backend por defecto es MSMF y abrir la
    C920 con el tarda ~40 s, sin mensaje de error: parece que el programa se
    colgo. DirectShow abre la misma camara en ~1 s. En Mac y Linux CAP_DSHOW
    no existe e impide que la camara abra, asi que cada plataforma lleva el
    suyo. Mismo criterio que listar_camaras.py.
    """
    if sys.platform == "win32":
        return cv2.CAP_DSHOW
    if sys.platform == "darwin":
        return cv2.CAP_AVFOUNDATION
    return cv2.CAP_V4L2


# CONVENCION DE EJES - LEE ESTO ANTES DE CAMBIARLO
#
# La imagen tiene la Y hacia ABAJO. Este flag decide si se voltea o no.
#
#   False -> Y hacia abajo, IGUAL QUE EL REPO DEL PROFE.
#            En Camera.py, get_anglerad() hace atan2(y, x) con coordenadas
#            crudas de imagen, y el lazo de control resta resolucion/2
#            conservando el signo. Un angulo positivo es HORARIO.
#            Usa esto si reutilizas su control_robot.py y su cinematica.
#
#   True  -> Y hacia arriba, sistema diestro normal. Angulo positivo
#            ANTIHORARIO. Usa esto si vas a alimentar cinematica.py
#            (el modulo de Atlas), que espera un sistema diestro.
#
# NO ES COSMETICO. Ambos convenios son internamente consistentes, pero
# cambian el signo de la velocidad angular W, y de ahi cual rueda recibe
# mas velocidad. Si mezclas convenciones el robot gira exactamente al
# reves y el error parece venir de la cinematica cuando en realidad
# viene de aqui.
Y_HACIA_ARRIBA = True

# Filtro exponencial contra el ruido de deteccion. 0 = sin filtro,
# 0.9 = muy suave pero con retraso. 0.5 es un buen punto de partida.
# Sin filtro, el jitter de un pixel se amplifica en el control y el robot
# tiembla al llegar a su destino.
SUAVIZADO = 0.5


# =========================================================
# REALCE PARA LA DETECCION
# =========================================================
# El realce se aplica SOLO a la copia en gris que ve detectMarkers. El
# frame que sale al navegador nunca se toca: la idea es que la deteccion
# mejore sin que tu dejes de ver lo que la camara ve de verdad.
#
# "clahe" es el que sirve cuando la luz es despareja -el reflejo de la
# lampara quemando una esquina y penumbra en la otra, que es justo lo que
# se ve en la cenital del laboratorio-. Ecualiza por zonas en vez de
# aplicar la misma curva a toda la imagen, asi que rescata el marcador de
# la zona oscura sin reventar todavia mas la zona quemada.
#
# "equalizar" es global y mas barato, pero con un reflejo fuerte suele
# empeorar las cosas: estira el histograma para complacer al brillo y
# aplasta el resto.
#
# clip alto = mas contraste y mas ruido. La rejilla es en cuantas celdas
# se parte la imagen: mas celdas = mas local.
# Arranca en "clahe" y no en "off" por un motivo medido, no por gusto: con
# el realce apagado NO detecta, ni con el diccionario y los parametros
# correctos. Se apaga desde POST /vision si algun dia sobra luz.
REALCE = "clahe"
CLAHE_CLIP = 2.0
CLAHE_REJILLA = 8
MODOS_REALCE = ("off", "clahe", "equalizar")


# =========================================================
# COMPATIBILIDAD ENTRE VERSIONES DE OPENCV
# La API de aruco cambio en OpenCV 4.7. Si tu version es distinta a la del
# profe, el codigo truena con AttributeError. Esto lo resuelve en ambos lados.
# =========================================================
API_NUEVA = hasattr(cv2.aruco, "ArucoDetector")


def _afinar(parametros):
    """
    Aplica PARAMS_DETECCION sobre el objeto de parametros.

    Uno por uno y con getattr de por medio: los nombres de estos campos han
    cambiado entre versiones de OpenCV, y un AttributeError aqui tumbaria la
    vision entera por un parametro de afinado. Si alguno no existe en la
    version instalada, se ignora y se sigue con el resto.
    """
    for nombre, valor in PARAMS_DETECCION.items():
        if hasattr(parametros, nombre):
            setattr(parametros, nombre, valor)
    return parametros


def _crear_detector():
    if API_NUEVA:
        diccionario = cv2.aruco.getPredefinedDictionary(DICCIONARIO)
        parametros = cv2.aruco.DetectorParameters()
        parametros.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return cv2.aruco.ArucoDetector(diccionario, _afinar(parametros))

    diccionario = cv2.aruco.Dictionary_get(DICCIONARIO)
    parametros = cv2.aruco.DetectorParameters_create()
    parametros.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return (diccionario, _afinar(parametros))


def _detectar(detector, gris):
    if API_NUEVA:
        esquinas, ids, _ = detector.detectMarkers(gris)
    else:
        diccionario, parametros = detector
        esquinas, ids, _ = cv2.aruco.detectMarkers(
            gris, diccionario, parameters=parametros)
    return esquinas, ids


def _dibujar_marcador(diccionario_id, marcador_id, lado_px):
    if API_NUEVA:
        diccionario = cv2.aruco.getPredefinedDictionary(diccionario_id)
        return cv2.aruco.generateImageMarker(diccionario, marcador_id, lado_px)

    diccionario = cv2.aruco.Dictionary_get(diccionario_id)
    return cv2.aruco.drawMarker(diccionario, marcador_id, lado_px)


# =========================================================
# GENERACION DE MARCADORES
# =========================================================
def generar(ids, tam_cm, dpi=300, margen_cm=2.0):
    """
    Guarda cada marcador como PNG listo para imprimir a tamano real.

    Sobre el tamano: un ArUco 6x6 mas su borde son 8x8 celdas, y necesitas
    varios pixeles por celda para que la deteccion aguante angulo y
    movimiento. Con una webcam a 3 m, un marcador de 15 cm ocupa unos 68 px
    a 1080p pero solo 45 px a 720p. Si detecta mal, lo primero es imprimir
    mas grande.
    """
    lado_px = int(round(tam_cm / 2.54 * dpi))
    margen_px = int(round(margen_cm / 2.54 * dpi))

    for marcador_id in ids:
        imagen = _dibujar_marcador(DICCIONARIO, marcador_id, lado_px)

        # El borde blanco no es decorativo: el detector necesita contraste
        # alrededor del marcador para encontrar sus bordes.
        lienzo = np.full(
            (lado_px + 2 * margen_px, lado_px + 2 * margen_px), 255, np.uint8)
        lienzo[margen_px:margen_px + lado_px,
               margen_px:margen_px + lado_px] = imagen

        nombre = "marcador_{:02d}.png".format(marcador_id)
        cv2.imwrite(nombre, lienzo)
        print("{}  ->  {} cm de lado, {} px a {} dpi".format(
            nombre, tam_cm, lado_px, dpi))

    print()
    print("Imprime SIN escalar (tamano real / 100%) y en papel MATE.")
    print("Verifica con regla que el cuadrado negro mida {} cm.".format(tam_cm))


# =========================================================
# VISION
# =========================================================
class Vision:
    def __init__(self, camara=CAMARA_POR_DEFECTO, ancho=RESOLUCION_X,
                 alto=RESOLUCION_Y, suavizado=SUAVIZADO):
        self.ancho = ancho
        self.alto = alto
        self.suavizado = suavizado
        self.detector = _crear_detector()
        self._filtradas = {}

        # Se cambian en caliente desde POST /vision, sin reabrir la camara.
        self.realce = REALCE
        self.clahe_clip = CLAHE_CLIP
        self.clahe_rejilla = CLAHE_REJILLA
        self._clahe = None
        self._clahe_firma = None

        self.captura = cv2.VideoCapture(camara, backend())
        self.captura.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
        self.captura.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
        # El autofoco cambia el encuadre efectivo a media corrida.
        self.captura.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        if not self.captura.isOpened():
            # Soltarla antes de reventar: el sandbox abre y cierra camaras a
            # peticion de la pagina, y una captura fallida sin liberar deja
            # el dispositivo tomado hasta que muera el proceso.
            self.captura.release()
            raise RuntimeError(
                "No se pudo abrir la camara {}. Prueba con otro indice "
                "o revisa que no la tenga otra app.".format(camara))

        # La resolucion pedida no siempre es la que da la camara.
        self.ancho = int(self.captura.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.alto = int(self.captura.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def _a_centro(self, punto):
        """
        Pasa de coordenadas de imagen al origen en el centro.

        El float() no es decorativo: OpenCV entrega las esquinas como
        float32 de numpy, y ese tipo revienta json.dumps con
        "Object of type float32 is not JSON serializable". Convertir aqui
        evita el problema en todo lo que consuma estas poses.
        """
        x = float(punto[0]) - self.ancho / 2.0
        y = float(punto[1]) - self.alto / 2.0
        if Y_HACIA_ARRIBA:
            y = -y
        return x, y

    def _obtener_clahe(self):
        """
        Reusa el objeto CLAHE mientras no cambien sus parametros.

        Crearlo en cada cuadro es tirar tiempo: construye las tablas de
        histograma por celda cada vez. A 20 cuadros por segundo se nota.
        """
        firma = (self.clahe_clip, self.clahe_rejilla)
        if self._clahe is None or self._clahe_firma != firma:
            lado = max(1, int(self.clahe_rejilla))
            self._clahe = cv2.createCLAHE(
                clipLimit=float(self.clahe_clip),
                tileGridSize=(lado, lado))
            self._clahe_firma = firma
        return self._clahe

    def realzar(self, gris, modo=None):
        """
        Devuelve la copia en gris con el contraste subido, para el detector.

        No toca el frame en color: quien llama a leer(con_frame=True) sigue
        recibiendo la imagen original para mostrarla.
        """
        modo = self.realce if modo is None else modo
        if modo == "clahe":
            return self._obtener_clahe().apply(gris)
        if modo == "equalizar":
            return cv2.equalizeHist(gris)
        return gris

    def comparar_realces(self, gris):
        """
        Cuantos marcadores encuentra cada modo sobre EL MISMO cuadro.

        Comparar entre cuadros distintos no dice nada: entre uno y otro se
        movio el robot, cambio el brillo o paso una sombra. Sobre el mismo
        gris, la diferencia es del realce y de nada mas.
        """
        salida = {}
        for modo in MODOS_REALCE:
            _esquinas, ids = _detectar(self.detector, self.realzar(gris, modo))
            salida[modo] = {
                "encontrados": 0 if ids is None else int(len(ids)),
                "ids": [] if ids is None else sorted(
                    int(x) for x in ids.flatten()),
            }
        return salida

    def leer(self, con_frame=False):
        """
        Devuelve {id: (x, y, theta)} con x,y en pixeles desde el centro y
        theta en radianes.
        """
        ok, frame = self.captura.read()
        if not ok:
            return ({}, None) if con_frame else {}

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Ojo: se detecta sobre la copia realzada, se devuelve el frame
        # original. Esa es toda la gracia del modo.
        esquinas, ids = _detectar(self.detector, self.realzar(gris))

        poses = {}
        if ids is not None:
            for esquina, marcador_id in zip(esquinas, ids.flatten()):
                puntos = esquina.reshape((4, 2))

                # detectMarkers entrega las esquinas en este orden:
                # 0 arriba-izq, 1 arriba-der, 2 abajo-der, 3 abajo-izq
                convertidos = [self._a_centro(p) for p in puntos]
                centro_x = sum(p[0] for p in convertidos) / 4.0
                centro_y = sum(p[1] for p in convertidos) / 4.0

                # El frente del robot lo define la arista inferior, igual que
                # en Camera.py del profe. Como las esquinas ya estan en el
                # sistema final, el signo del giro sale correcto solo.
                abajo_der, abajo_izq = convertidos[2], convertidos[3]
                theta = math.atan2(abajo_der[1] - abajo_izq[1],
                                   abajo_der[0] - abajo_izq[0])

                poses[int(marcador_id)] = self._suavizar(
                    int(marcador_id), float(centro_x), float(centro_y),
                    float(theta))

        if con_frame:
            return poses, (frame, esquinas, ids, gris)
        return poses

    def _suavizar(self, marcador_id, x, y, theta):
        if self.suavizado <= 0:
            return (x, y, theta)

        previo = self._filtradas.get(marcador_id)
        if previo is None:
            self._filtradas[marcador_id] = (x, y, theta)
            return (x, y, theta)

        a = self.suavizado
        px, py, pth = previo

        # El angulo no se puede promediar directo: entre 179 y -179 el
        # promedio ingenuo da 0, que es justo lo contrario. Se interpola
        # sobre la diferencia envuelta a [-pi, pi].
        delta = math.atan2(math.sin(theta - pth), math.cos(theta - pth))

        nuevo = (a * px + (1 - a) * x,
                 a * py + (1 - a) * y,
                 pth + (1 - a) * delta)
        self._filtradas[marcador_id] = nuevo
        return nuevo

    def cerrar(self):
        self.captura.release()


# =========================================================
# VISTA EN VIVO
# =========================================================
def vista_en_vivo(camara):
    vision = Vision(camara=camara)
    print("Camara {}: {}x{}".format(camara, vision.ancho, vision.alto))
    print("API de aruco: {}".format("nueva (>=4.7)" if API_NUEVA else "vieja (<4.7)"))
    print("ESC para salir.")

    try:
        while True:
            poses, extra = vision.leer(con_frame=True)
            if extra is None:
                continue
            frame, esquinas, ids, _gris = extra

            cv2.line(frame, (vision.ancho // 2, 0),
                     (vision.ancho // 2, vision.alto), (60, 60, 60), 1)
            cv2.line(frame, (0, vision.alto // 2),
                     (vision.ancho, vision.alto // 2), (60, 60, 60), 1)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, esquinas, ids)

            for marcador_id, (x, y, theta) in poses.items():
                px = int(x + vision.ancho / 2)
                py = int((-y if Y_HACIA_ARRIBA else y) + vision.alto / 2)

                # Flecha del frente, para ver a simple vista si theta es correcto
                largo = 60
                fx = int(px + largo * math.cos(theta))
                fy = int(py + (-1 if Y_HACIA_ARRIBA else 1) * largo * math.sin(theta))
                cv2.arrowedLine(frame, (px, py), (fx, fy), (0, 220, 220), 2, tipLength=0.3)

                cv2.putText(frame,
                            "id{}  ({:.0f}, {:.0f})  {:.0f}d".format(
                                marcador_id, x, y, math.degrees(theta)),
                            (px - 80, py - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 220), 2)

            cv2.putText(frame, "detectados: {}".format(len(poses)),
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Vision ArUco", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        vision.cerrar()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camara", type=int, default=CAMARA_POR_DEFECTO)
    parser.add_argument("--generar", type=int, nargs="+", metavar="ID",
                        help="genera marcadores con estos IDs")
    parser.add_argument("--tam-cm", type=float, default=15.0)
    args = parser.parse_args()

    if args.generar:
        generar(args.generar, args.tam_cm)
    else:
        vista_en_vivo(args.camara)


if __name__ == "__main__":
    main()
