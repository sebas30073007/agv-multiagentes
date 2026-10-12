#!/usr/bin/env python3
"""
servidor_sandbox.py - Cierra el lazo entre el control y el navegador.

    Python                              Navegador
    ------                              ---------
    camara -> vision.py                 recibe la trama
    control.py                          simula el movimiento
    cinematica.py -> trama  ---------->  integra
                            <----------  devuelve x, y, theta
    calcula la siguiente trama

El navegador hace el papel del robot: SOLO recibe cuatro numeros y devuelve
su pose. No sabe donde esta el destino ni como se calcula nada. Igual que
Atlas.

USO
    python servidor_sandbox.py
    python servidor_sandbox.py --camara 1       # arranca con esa
    python servidor_sandbox.py --sin-camara     # sin hardware
    python servidor_sandbox.py --puerto 8080

El indice de arranque es solo eso: desde la pagina, en modo Real, se puede
buscar que camaras hay y cambiar de una a otra sin reiniciar nada.

Luego abre en el navegador:
    http://localhost:8000

El servidor tambien SIRVE la pagina, asi que no hay problemas de CORS ni de
abrir el archivo con file://.

CON CAMARA
El ID 0 marca donde esta Atlas y el ID 1 a donde debe llegar, con su
orientacion. Desde la pagina puedes sincronizar el robot simulado a la pose
real cuando quieras.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cinematica
import control as ctrl

ID_ROBOT = 0
ID_DESTINO = 1

# Todo en PIXELES de la imagen de la camara.
V_MAX_PX = 250.0
W_MAX_RAD = 1.6
K_TRASLACION = 0.8
K_RUMBO = 1.2
# Respaldo por si la pagina no manda "escala"; ella siempre la manda. Ver la
# nota de ESCALA_DEFECTO en static/js/config.js: al 45% Atlas iba a la mitad
# de velocidad que los uniciclo y se dejaba empujar por cualquier campo.
ESCALA = 1.0
TOLERANCIA_PX = 30.0
TOLERANCIA_RUMBO = 0.09
ANCHO_ROBOT_PX = 250.0

# =========================================================
# PUENTE MQTT AL ROBOT FISICO (modo Manual de la pagina)
# =========================================================
# La pagina se sirve por http://, y el broker del laboratorio solo expone
# WebSockets sobre TLS (wss://). Un navegador no puede abrir wss desde una
# pagina http sin que truene por contenido mixto, asi que el salto lo da el
# servidor: la pagina hace POST /manual y aqui se publica por MQTT 1883.
# Broker, puerto y prefijo de temas salen del entorno o de un archivo .env
# junto a este script (no se versiona; ver .env.ejemplo). Un broker en la
# misma red local baja el RTT completo a la mitad frente a uno en la nube.
def _cargar_env(ruta):
    """Lee CLAVE=valor de un .env local sin pisar lo que ya este exportado."""
    try:
        with open(ruta, encoding="utf-8") as f:
            for linea in f:
                linea = linea.strip()
                if not linea or linea.startswith("#") or "=" not in linea:
                    continue
                clave, valor = linea.split("=", 1)
                os.environ.setdefault(clave.strip(), valor.strip())
    except FileNotFoundError:
        pass


_cargar_env(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))
BROKER_MQTT = os.environ.get("MQTT_BROKER", "localhost")
PUERTO_MQTT = int(os.environ.get("MQTT_PORT", "1883"))
PREFIJO_MQTT = os.environ.get("MQTT_PREFIJO", "flota")

# Que id de robot de la pagina corresponde a que robot_id del agente.
# robot2 es Odiseo: se comprobo de punta a punta publicando en
# <prefijo>/robot2/cmd y viendo contestar FRAME_OK del firmware.
ROBOT_A_TOPIC = {"atlas": "robot3", "apolo": "robot1", "odiseo": "robot2"}

_cliente_mqtt = None
_candado_mqtt = threading.Lock()


def cliente_mqtt():
    """Cliente perezoso: no se conecta hasta que alguien use el modo Manual."""
    global _cliente_mqtt
    with _candado_mqtt:
        if _cliente_mqtt is not None:
            return _cliente_mqtt
        import paho.mqtt.client as mqtt
        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sandbox-manual")
        except AttributeError:
            c = mqtt.Client(client_id="sandbox-manual")
        c.connect(BROKER_MQTT, PUERTO_MQTT, 60)
        c.loop_start()
        _cliente_mqtt = c
        print("MQTT manual conectado a {}:{}".format(BROKER_MQTT, PUERTO_MQTT))
        return c


def publicar_manual(robot_id, pwm):
    destino = ROBOT_A_TOPIC.get(robot_id)
    if destino is None:
        raise ValueError("robot desconocido: {}".format(robot_id))
    valores = [int(v) for v in pwm]
    if not all(-255 <= v <= 255 for v in valores):
        raise ValueError("pwm fuera de rango")
    topic = "{}/{}/cmd".format(PREFIJO_MQTT, destino)

    # qos 1 obliga a un PUBACK: un viaje completo de ida y vuelta extra, que
    # contra el broker viejo eran ~72 ms. Para conducir no vale la pena, una
    # orden de direccion vieja no sirve de nada y ya viene otra detras.
    #
    # El cero es la excepcion y no se toca. Con el latido del agente
    # sosteniendo el watchdog del ESP32, un stop perdido no para en 2.5 s:
    # se queda corriendo. Ese si va garantizado.
    es_paro = all(v == 0 for v in valores)
    cliente_mqtt().publish(topic, json.dumps({"pwm": valores}),
                           qos=1 if es_paro else 0)
    return topic


DIR_ESTATICO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
TIPOS_ESTATICO = {
    ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
    ".svg": "image/svg+xml", ".webp": "image/webp",
    # Modelos de la vista 3D (static/assets/modelos/).
    ".glb": "model/gltf-binary",
}

# Estado compartido entre el hilo de la camara y el del servidor.
ESTADO = {
    "robot": None,       # (x, y, th) leidos de la camara
    "destino": None,
    # Todos los marcadores vistos, {id: (x, y, th)}. La pagina elige cual
    # es cual: que ArUco corresponde a que robot es configurable ahi, no
    # una constante del servidor. ID_ROBOT/ID_DESTINO quedan solo como
    # valores por defecto para el modo sin configurar.
    "poses": {},
    "ancho": 1280,
    "alto": 720,
    "camara_viva": False,
}
CANDADO = threading.Lock()

# Ultimo frame JPEG con la deteccion pintada, para /video.mjpg.
# Va en un candado aparte: se escribe a la frecuencia de la camara y se lee
# a la frecuencia de cada cliente conectado, no tiene sentido compartir el
# lock de ESTADO y frenar el otro hilo por un cliente lento.
FRAME_JPEG = None
# Segundo stream: lo MISMO que ve el detector, ya realzado. Solo se genera
# cuando alguien lo esta mirando; con nadie viendo no se gasta el encode.
FRAME_REALCE_JPEG = None
CLIENTES_REALCE = 0
FRAME_LOCK = threading.Lock()
JPEG_QUALITY = 70
STREAM_FPS = 20

# =========================================================
# CAMARA: recados de la pagina al hilo de la camara
# =========================================================
# El hilo de la camara es el UNICO que abre, lee y cierra capturas. cv2 no
# lleva bien que dos hilos toquen el mismo dispositivo, y en Windows abrir
# un indice que ya tienes abierto simplemente falla. Asi que el servidor
# HTTP no toca la camara: deja un recado aqui y el bucle lo atiende en su
# siguiente vuelta.
CAMARA = {
    "indice": None,        # la que esta abierta ahora, None si ninguna
    "solicitado": None,    # recado: abre este indice
    "cerrar": False,       # recado: cierra la que este abierta
    "escanear": False,     # recado: busca que indices responden
    "escaneando": False,   # el escaneo esta corriendo (tarda segundos)
    "lista": None,         # ultimo escaneo; None = no se ha hecho ninguno
    "error": None,         # por que fallo el ultimo intento de abrir
}
# Los valores van aqui en crudo y NO importados de vision.py a proposito:
# vision se importa tarde, dentro de las funciones, para que el servidor
# arranque con --sin-camara en una maquina sin OpenCV. Importarlo aqui
# arriba rompe justo ese caso.
MODOS_REALCE = ("off", "clahe", "equalizar")
VISION = {
    "realce": "clahe",                    # off | clahe | equalizar
    "clip": 2.0,
    "rejilla": 8,
    "comparar": False,                    # recado: mide los tres modos
    "comparacion": None,                  # resultado de la ultima medicion
    "encontrados": 0,                     # marcadores del ultimo cuadro
}

MAX_INDICE_CAMARA = 6
# Tope para el escaneo, que corre en un proceso aparte: si se cuelga, se
# corta en vez de dejar el bucle de camara parado para siempre.
SEGUNDOS_ESCANEO = 90


def estado_camara():
    """Lo barato: cabe en cada respuesta de /ciclo, 20 veces por segundo."""
    with CANDADO:
        return {
            "indice": CAMARA["indice"],
            "escaneando": CAMARA["escaneando"],
            "error": CAMARA["error"],
            "viva": ESTADO["camara_viva"],
            "ancho": ESTADO["ancho"],
            "alto": ESTADO["alto"],
        }


def info_camaras():
    """Lo caro: el listado completo. Solo se pide cuando la pagina lo abre."""
    d = estado_camara()
    with CANDADO:
        d["lista"] = None if CAMARA["lista"] is None else list(CAMARA["lista"])
    d["max_indice"] = MAX_INDICE_CAMARA
    return d


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


# =========================================================
# CONTROL
# =========================================================
def calcular(pose, destino, escala, k, tol_px=TOLERANCIA_PX, tol_rumbo=TOLERANCIA_RUMBO):
    """Las dos tolerancias entran por parametro porque la pagina las mueve en
    vivo (los sliders "Zona de llegada" y "Zona de rumbo" del modo Punto).
    Las constantes de arriba se quedan de valor por defecto.

    Dentro de la zona, el eje correspondiente sale en cero. Antes solo la
    traslacion se callaba: el giro seguia mandando correcciones mientras el
    error de rumbo no fuera exactamente cero, y el robot temblaba en sitio
    sobre el destino."""
    x, y, th = pose
    xd, yd, thd = destino

    dist = math.hypot(x - xd, y - yd)
    llego = dist < tol_px

    if llego:
        adelante = izquierda = 0.0
    else:
        ux, uy = ctrl.convergencia(x, y, xd, yd, k)
        adelante, izquierda = ctrl.holonomico(ux, uy, th)
        adelante /= V_MAX_PX
        izquierda /= V_MAX_PX

    err = envolver(th - thd)
    giro = 0.0 if abs(err) < tol_rumbo else -K_RUMBO * err / W_MAX_RAD

    # holonomico() da la lateral hacia la IZQUIERDA; cinematica.py espera
    # vx hacia la DERECHA.
    trama = cinematica.a_frame(vx=-izquierda, vy=adelante, w=giro, escala=escala)
    valores = [0 if t == "S" else int(t) for t in trama.split(",")]

    return trama, valores, {
        "dist": round(dist, 1),
        "err_rumbo": round(math.degrees(err), 1),
        "adelante": round(adelante, 3),
        "izquierda": round(izquierda, 3),
        "giro": round(giro, 3),
        "llego": llego,
        "quieto": llego and abs(err) < tol_rumbo,
        "activas": sum(1 for v in valores if v != 0),
    }


def poses_vistas():
    with CANDADO:
        return {str(k): [float(v[0]), float(v[1]), float(v[2])]
                for k, v in ESTADO["poses"].items()}


def configuracion():
    with CANDADO:
        ancho, alto = ESTADO["ancho"], ESTADO["alto"]
        viva = ESTADO["camara_viva"]
    return {
        "ancho": ancho, "alto": alto,
        "camara_viva": viva,
        "v_max_px": V_MAX_PX,
        "w_max_rad": W_MAX_RAD,
        "tolerancia_px": TOLERANCIA_PX,
        "ancho_robot_px": ANCHO_ROBOT_PX,
        "ancho_via": cinematica.ANCHO_VIA,
        "entre_ejes": cinematica.ENTRE_EJES,
        "config_rodillos": cinematica.CONFIG_RODILLOS.upper(),
        "autoridad": round(cinematica.autoridad_de_giro(), 4),
        "zona_muerta": cinematica.DEADZONE_RAW,
        "max_raw": cinematica.MAX_CMD_RAW,
        "escala_minima": round(cinematica.escala_minima_util(), 3),
        # El navegador necesita los signos para deshacerlos en la
        # cinematica directa, igual que haria el firmware al revés.
        "orden": [{"rueda": n, "signo": s} for n, s in cinematica.FRAME_LAYOUT],
        # Los espejos medidos sobre el robot. Viajan a la pagina para que la
        # simulacion mueva el sprite hacia el mismo lado que el robot: si se
        # quedan solo en Python, la trama sale bien y el dibujo sale al reves.
        "invertir_lateral": bool(cinematica.INVERTIR_LATERAL),
        "invertir_giro": bool(cinematica.INVERTIR_GIRO),
    }


# =========================================================
# SERVIDOR
# =========================================================
def a_nativo(v):
    """
    Convierte tipos de numpy a tipos nativos de Python.

    OpenCV entrega float32, que json.dumps no sabe serializar: la peticion
    revienta entera y el navegador se queda sin respuesta, asi que la
    simulacion parece "colgada" aunque la camara este detectando bien.
    Se corrige en vision.py, pero conviene una segunda red aqui.
    """
    if v is None:
        return None
    if isinstance(v, (list, tuple)):
        return [a_nativo(x) for x in v]
    if isinstance(v, dict):
        return {k: a_nativo(x) for k, x in v.items()}
    if hasattr(v, "item"):      # escalares de numpy
        return v.item()
    return v


class Manejador(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # sin ruido en consola

    def _responder(self, codigo, cuerpo, tipo="application/json"):
        datos = cuerpo.encode("utf-8") if isinstance(cuerpo, str) else cuerpo
        self.send_response(codigo)
        self.send_header("Content-Type", tipo)
        self.send_header("Content-Length", str(len(datos)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(datos)

    def ruta(self):
        """
        La ruta sin la query. La pagina pide /video.mjpg?t=... para forzar
        una conexion nueva cuando la camara vuelve, y comparando self.path
        crudo eso no casaba con ninguna ruta: se iba al servidor de
        archivos y contestaba 404 con el stream ahi al lado.
        """
        return self.path.split("?", 1)[0]

    def do_GET(self):
        ruta = self.ruta()
        if ruta == "/config":
            self._responder(200, json.dumps(configuracion()))
        elif ruta == "/vision":
            with CANDADO:
                cuerpo = json.dumps(dict(VISION, modos=list(MODOS_REALCE)))
            self._responder(200, cuerpo)
            return
        elif ruta == "/camaras":
            self._responder(200, json.dumps(info_camaras()))
        elif ruta == "/video.mjpg":
            self._stream_video()
        elif ruta == "/video_realce.mjpg":
            self._stream_video(realce=True)
        else:
            self._servir_estatico()

    def _servir_estatico(self):
        """
        Sirve todo lo que hay bajo static/ como si fuera la raiz del sitio
        (index.html en "/", static/css/x.css en "/css/x.css", etc). Un solo
        punto de entrada para HTML/CSS/JS/imagenes en vez de una ruta por
        tipo de archivo.
        """
        ruta_pedida = self.ruta()
        subruta = "index.html" if ruta_pedida == "/" else ruta_pedida.lstrip("/")
        # sin ".." ni rutas absolutas: no se sale de DIR_ESTATICO
        ruta = os.path.normpath(os.path.join(DIR_ESTATICO, subruta))
        if os.path.commonpath([ruta, DIR_ESTATICO]) != DIR_ESTATICO:
            self._responder(403, "{}")
            return

        ext = os.path.splitext(ruta)[1].lower()
        tipo = TIPOS_ESTATICO.get(ext)
        if not tipo or not os.path.isfile(ruta):
            self._responder(404, "{}")
            return
        with open(ruta, "rb") as f:
            self._responder(200, f.read(), tipo)

    def _stream_video(self, realce=False):
        """
        MJPEG motion-JPEG: un truco viejo pero universal. Cada "parte" del
        stream es un JPEG completo separado por un boundary; el navegador
        entiende esto nativo en una <img src="/video.mjpg">, sin JS ni
        WebRTC. No es eficiente para muchos clientes a la vez, pero para
        un dashboard de banco con 1-2 personas viendo es de sobra.
        """
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        global CLIENTES_REALCE
        if realce:
            with FRAME_LOCK:
                CLIENTES_REALCE += 1

        periodo = 1.0 / STREAM_FPS
        try:
            while True:
                with FRAME_LOCK:
                    frame = FRAME_REALCE_JPEG if realce else FRAME_JPEG
                if frame is not None:
                    self.wfile.write(b"--FRAME\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write("Content-Length: {}\r\n\r\n".format(len(frame)).encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                time.sleep(periodo)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # el cliente cerro la pestana o navego a otro lado
        finally:
            if realce:
                with FRAME_LOCK:
                    CLIENTES_REALCE = max(0, CLIENTES_REALCE - 1)

    def do_POST(self):
        ruta = self.ruta()
        if ruta not in ("/ciclo", "/manual", "/camaras", "/vision"):
            self._responder(404, "{}")
            return

        largo = int(self.headers.get("Content-Length", 0))
        try:
            pet = json.loads(self.rfile.read(largo) or b"{}")
        except ValueError:
            self._responder(400, json.dumps({"error": "json invalido"}))
            return

        if ruta == "/vision":
            # Se deja el recado y lo aplica el hilo de la camara, igual que
            # con los cambios de indice: tocar el objeto Vision desde este
            # hilo seria pisarle el cuadro a media lectura.
            with CANDADO:
                if "realce" in pet:
                    modo = str(pet["realce"])
                    if modo not in MODOS_REALCE:
                        self._responder(400, json.dumps(
                            {"error": "realce debe ser uno de {}".format(
                                list(MODOS_REALCE))}))
                        return
                    VISION["realce"] = modo
                if "clip" in pet:
                    VISION["clip"] = max(0.1, min(40.0, float(pet["clip"])))
                if "rejilla" in pet:
                    VISION["rejilla"] = max(1, min(32, int(pet["rejilla"])))
                if pet.get("comparar"):
                    VISION["comparar"] = True
                    VISION["comparacion"] = None
                cuerpo = json.dumps(dict(VISION, modos=list(MODOS_REALCE)))
            self._responder(200, cuerpo)
            return

        if ruta == "/camaras":
            # Solo se deja el recado; quien abre y cierra es el hilo de la
            # camara. La respuesta es el estado de ESTE instante, todavia
            # sin atender: la pagina ve el cambio en el siguiente /ciclo.
            with CANDADO:
                if pet.get("escanear"):
                    CAMARA["escanear"] = True
                    # Se marca YA, no cuando el bucle lo atienda: asi la
                    # respuesta a este POST ya trae "escaneando" y la pagina
                    # puede poner el boton a girar sin esperar una vuelta.
                    CAMARA["escaneando"] = True
                elif "indice" in pet:
                    if pet["indice"] is None:
                        CAMARA["cerrar"] = True
                        CAMARA["solicitado"] = None
                    else:
                        CAMARA["solicitado"] = int(pet["indice"])
                        CAMARA["cerrar"] = False
            self._responder(200, json.dumps(info_camaras()))
            return

        if ruta == "/manual":
            try:
                topic = publicar_manual(pet.get("robot", ""), pet.get("pwm", []))
            except ImportError:
                self._responder(200, json.dumps(
                    {"ok": False, "error": "falta paho-mqtt: pip install paho-mqtt"}))
            except Exception as e:
                self._responder(200, json.dumps({"ok": False, "error": str(e)}))
            else:
                self._responder(200, json.dumps({"ok": True, "topic": topic}))
            return

        pose = (float(pet.get("x", 0)), float(pet.get("y", 0)), float(pet.get("th", 0)))
        escala = float(pet.get("escala", ESCALA))
        k = float(pet.get("k", K_TRASLACION))
        # La zona de llegada llega en cada ciclo desde el panel. Con tope
        # abajo: en 0 el robot no para nunca -cada paso cruza el punto de
        # largo- y se queda zumbando encima del destino.
        tol_px = max(1.0, float(pet.get("tol_px", TOLERANCIA_PX)))
        tol_rumbo = max(0.005, math.radians(float(pet.get("tol_grados",
                                                         math.degrees(TOLERANCIA_RUMBO)))))

        # MODO REAL. La pagina manda "tag": el ArUco que identifica al robot.
        # Si ese marcador se ve, su pose PISA la que mando la pagina: en real
        # la posicion la dicta la camara, no la integracion del navegador.
        #
        # El destino NO se toca. Punto, trayectoria y formacion son
        # referencias imaginarias que solo existen en la pagina; lo unico
        # real es donde esta el robot.
        tag = pet.get("tag")
        tag_visto = False
        if tag is not None:
            with CANDADO:
                vista = ESTADO["poses"].get(int(tag))
            if vista is not None:
                pose = (float(vista[0]), float(vista[1]), float(vista[2]))
                tag_visto = True

        with CANDADO:
            cam_robot = ESTADO["robot"]
            cam_destino = ESTADO["destino"]

        # El destino: manda el que elija la pagina; si no, el de la camara.
        d = pet.get("destino")
        if d:
            destino = (float(d["x"]), float(d["y"]), float(d["th"]))
        elif cam_destino:
            destino = cam_destino
        else:
            destino = (0.0, 0.0, 0.0)

        try:
            trama, valores, info = calcular(pose, destino, escala, k, tol_px, tol_rumbo)
        except Exception as e:
            self._responder(500, json.dumps({"error": str(e)}))
            return

        # Publicar al robot fisico. Va solo si la pagina lo pide Y el
        # marcador se esta viendo: mandar comandos calculados desde una pose
        # inventada es justo como se estrella un robot.
        enviado = None
        if pet.get("enviar") and tag_visto:
            try:
                publicar_manual(pet.get("robot", ""), valores)
                enviado = True
            except Exception as e:
                enviado = str(e)

        # El estado de la camara viaja en CADA ciclo, no solo en /config: la
        # pagina lee /config una sola vez al cargar, y si la camara todavia
        # no habia abierto, el cartel de "sin camara" se quedaba puesto para
        # siempre. Ahora ademas la pagina puede cambiar de camara sola, asi
        # que necesita ver el cambio sin recargar.
        cam = estado_camara()

        self._responder(200, json.dumps(a_nativo({
            "trama": trama,
            "valores": valores,
            "destino": {"x": destino[0], "y": destino[1], "th": destino[2]},
            "info": info,
            "pose_usada": {"x": pose[0], "y": pose[1], "th": pose[2]},
            "camara_viva": cam["viva"],
            "camara_estado": cam,
            "tag_visto": tag_visto,
            "enviado": enviado,
            "poses": poses_vistas(),
            "camara": {
                "robot": list(cam_robot) if cam_robot else None,
                "destino": list(cam_destino) if cam_destino else None,
            },
        })))


def arrancar_servidor(puerto):
    servidor = ThreadingHTTPServer(("", puerto), Manejador)
    hilo = threading.Thread(target=servidor.serve_forever, daemon=True)
    hilo.start()
    return servidor


# =========================================================
# CAMARA
# =========================================================
def nombres_de_camaras():
    """
    Nombre legible de cada camara, si el sistema deja saberlo.

    OpenCV no expone el nombre del dispositivo por ninguna via. En Windows
    lo da pygrabber, que enumera DirectShow en el MISMO orden que los
    indices de CAP_DSHOW; en Linux lo publica el kernel en sysfs. Las dos
    son opcionales: sin ellas la lista sale como "Camara 0", "Camara 1", y
    se identifica cual es cual mirando el video, que es lo que uno acaba
    haciendo de todas formas.
    """
    try:
        if sys.platform == "win32":
            from pygrabber.dshow_graph import FilterGraph
            return list(FilterGraph().get_input_devices())
        if sys.platform.startswith("linux"):
            import glob
            nombres = []
            for ruta in sorted(glob.glob("/sys/class/video4linux/video*/name")):
                with open(ruta) as f:
                    nombres.append(f.read().strip())
            return nombres
    except Exception:
        pass
    return []


def escanear_aqui():
    """
    Prueba los indices 0..MAX_INDICE_CAMARA y se queda con los que entregan
    imagen. Mismo criterio que listar_camaras.py, que sigue siendo la
    herramienta de consola; esto es su version para la pagina.

    OJO: corre con la camara CERRADA. En Windows, abrir el indice que ya
    tienes abierto falla, y ese fallo se leeria como "esa camara no existe",
    justo la que estas usando.

    Escanea en ESTE proceso. Casi nunca se llama directo: quien la usa es
    escanear_camaras(), que la lanza en un proceso aparte. El porque esta
    ahi, y es la razon de que esta funcion viva separada.
    """
    import cv2
    from vision import backend, RESOLUCION_X, RESOLUCION_Y

    nombres = nombres_de_camaras()
    encontradas = []
    for i in range(MAX_INDICE_CAMARA):
        captura = None
        try:
            captura = cv2.VideoCapture(i, backend())
            if not captura.isOpened():
                continue
            ok, frame = captura.read()
            if not ok or frame is None:
                continue
            # Que resolucion da al pedirle 720p: es lo que distingue de un
            # vistazo la webcam integrada de la buena.
            captura.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUCION_X)
            captura.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUCION_Y)
            encontradas.append({
                "indice": i,
                "nombre": nombres[i] if i < len(nombres) else "",
                "ancho": int(captura.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "alto": int(captura.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            })
        except Exception:
            pass
        finally:
            if captura is not None:
                captura.release()
    return encontradas


def escanear_camaras():
    """
    Escanea en un proceso NUEVO, no en este.

    En Windows, OpenCV enumera los dispositivos DirectShow una sola vez por
    proceso: la lista queda congelada como estaba al arrancar el servidor.
    Una camara conectada despues NO aparecia por mas veces que le dieras a
    Buscar, porque el escaneo corria aqui adentro y releia esa lista vieja.
    Medido: un servidor arrancado antes de conectar la USB ve solo el indice
    0; otro recien arrancado, misma maquina y mismo codigo, ve el 0 y el 1.

    Un proceso hijo enumera desde cero y se muere. Cuesta el arranque de
    Python -~1 s- sobre los segundos que el escaneo ya tardaba, y solo se
    paga al apretar el boton. A cambio, Buscar encuentra lo que este
    conectado AHORA y no hay que reiniciar el servidor para cambiar de
    camara.

    El hijo es este mismo archivo con --escanear-json, asi que la logica de
    escaneo no se duplica: es escanear_aqui() en los dos lados.
    """
    import subprocess

    try:
        salida = subprocess.run(
            [sys.executable, os.path.abspath(__file__), "--escanear-json"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, timeout=SEGUNDOS_ESCANEO)
    except subprocess.TimeoutExpired:
        # Reintentar aqui no ayudaria: si una camara cuelga al hijo, cuelga
        # igual a este proceso, y este es el hilo que mueve todo lo demas.
        # Mejor lista vacia y que el usuario vuelva a buscar.
        print("Escaneo colgado mas de {} s, lo corto".format(SEGUNDOS_ESCANEO))
        return []
    except Exception as e:
        # Empaquetado, sin interprete a la vista, permisos: da igual cual
        # sea. Se escanea aqui, que es lo que se hacia antes: lista vieja es
        # mejor que ninguna.
        print("No pude lanzar el escaneo aparte ({}); escaneo aqui".format(e))
        return escanear_aqui()

    # OpenCV escupe sus avisos por su cuenta y no se le puede pedir silencio,
    # asi que no se confia en que stdout venga limpio: se busca la ultima
    # linea que sea la lista JSON.
    for linea in reversed(salida.stdout.splitlines()):
        linea = linea.strip()
        if linea.startswith("["):
            try:
                return json.loads(linea)
            except ValueError:
                continue

    print("El escaneo aparte no devolvio lista; escaneo aqui")
    if salida.stderr.strip():
        print(salida.stderr.strip()[-400:])
    return escanear_aqui()


def motivo_de_fallo(indice, error):
    """
    Afina el "no se pudo abrir" cuando el escaneo SI vio esa camara.

    Es el otro lado de lo que cuenta escanear_camaras(): la lista de
    dispositivos DirectShow se congela al arrancar el proceso. El escaneo
    esquiva eso yendose a un proceso hijo, pero ABRIR tiene que pasar por
    aqui, y aqui la lista sigue siendo la de cuando arranco el servidor.

    Resultado: una camara conectada despues sale en el desplegable -la vio
    el hijo- y al elegirla falla con un pVih que no dice nada. El mensaje de
    vision.py manda a buscar otra app que la tenga tomada, que es justo el
    camino equivocado: no la tiene nadie, es que este proceso no sabe que
    existe. Lo unico que lo arregla es reiniciar el servidor.
    """
    with CANDADO:
        lista = CAMARA["lista"]
    if lista and any(c.get("indice") == indice for c in lista):
        return ("La camara {} existe -el buscador la ve- pero este servidor "
                "arranco antes de que la conectaras y Windows ya no se la "
                "ensena. Reinicia el servidor y estara disponible."
                .format(indice))
    return str(error)


def abrir_camara(indice):
    """Devuelve un Vision abierto, o None dejando el motivo en CAMARA."""
    from vision import Vision
    try:
        vision = Vision(camara=indice)
    except Exception as e:
        motivo = motivo_de_fallo(indice, e)
        with CANDADO:
            CAMARA["indice"] = None
            CAMARA["error"] = motivo
            ESTADO["camara_viva"] = False
        print("Camara {}: {}".format(indice, motivo))
        return None

    with CANDADO:
        CAMARA["indice"] = indice
        CAMARA["error"] = None
        ESTADO["ancho"], ESTADO["alto"] = vision.ancho, vision.alto
        ESTADO["camara_viva"] = True
    print("Camara {}: {}x{}".format(indice, vision.ancho, vision.alto))
    return vision


def cerrar_camara(vision):
    """Cierra y deja el estado limpio. Devuelve None, para reasignar."""
    global FRAME_JPEG, FRAME_REALCE_JPEG
    if vision is not None:
        try:
            vision.cerrar()
        except Exception:
            pass
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass
    with CANDADO:
        CAMARA["indice"] = None
        ESTADO["camara_viva"] = False
        # Las poses se tiran: son de la camara que se acaba de ir. Dejarlas
        # puestas haria que el marcador siguiera "detectado" sin camara, y
        # el lazo mandaria comandos calculados sobre una pose fantasma.
        ESTADO["poses"] = {}
        ESTADO["robot"] = None
        ESTADO["destino"] = None
    with FRAME_LOCK:
        FRAME_JPEG = None
    return None


def bucle_camara(indice_inicial, mostrar=True):
    """
    Corre en el hilo principal y es el UNICO que toca la camara. cv2.imshow
    desde un hilo secundario da problemas en varias plataformas, por eso el
    servidor es el que se va a un hilo aparte y no al reves.

    Cada vuelta hace tres cosas: atender los recados que dejo la pagina en
    CAMARA (cambiar de camara, escanear indices), actualizar ESTADO para el
    control, y publicar el frame con la deteccion pintada en FRAME_JPEG para
    que /video.mjpg lo sirva al navegador. El stream se alimenta siempre,
    con o sin --sin-ventana: la ventana local es solo para depurar en el
    banco, el stream web es el que ve la pagina.

    Si la camara no abre, el bucle NO se muere: se queda sirviendo sin video
    esperando a que alguien elija otra desde la pagina. Antes esto era un
    callejon sin salida - habia que matar el servidor y volver a arrancarlo
    con otro --camara.
    """
    import cv2

    global FRAME_JPEG, FRAME_REALCE_JPEG

    vision = None
    fallos = 0
    muertos = 0   # cuadros seguidos de un solo color, ver el watchdog abajo
    if indice_inicial is not None:
        with CANDADO:
            CAMARA["solicitado"] = indice_inicial

    try:
        while True:
            with CANDADO:
                pedido = CAMARA["solicitado"]
                CAMARA["solicitado"] = None
                cerrar = CAMARA["cerrar"]
                CAMARA["cerrar"] = False
                escanear = CAMARA["escanear"]
                CAMARA["escanear"] = False
                abierta = CAMARA["indice"]
                if escanear:
                    CAMARA["escaneando"] = True

            if cerrar:
                vision = cerrar_camara(vision)
                abierta = None

            if escanear:
                vision = cerrar_camara(vision)
                lista = escanear_camaras()
                with CANDADO:
                    CAMARA["lista"] = lista
                    CAMARA["escaneando"] = False
                    # El error de la camara anterior ya no describe nada:
                    # hay una lista fresca y lo que toca es elegir de ahi.
                    CAMARA["error"] = None
                print("Camaras: {}".format([c["indice"] for c in lista] or "ninguna"))
                # Buscar no cuesta perder la que estabas usando: se reabre.
                if pedido is None and abierta is not None:
                    pedido = abierta

            if pedido is not None:
                vision = cerrar_camara(vision)
                vision = abrir_camara(pedido)
                fallos = 0

            if vision is None:
                time.sleep(0.15)   # sin camara no hay nada que leer
                continue

            # Los ajustes de realce se aplican aqui, en el hilo dueno de la
            # camara, y no en el manejador HTTP.
            with CANDADO:
                vision.realce = VISION["realce"]
                vision.clahe_clip = VISION["clip"]
                vision.clahe_rejilla = VISION["rejilla"]
                comparar = VISION["comparar"]
                VISION["comparar"] = False

            poses, extra = vision.leer(con_frame=True)
            if extra is None:
                # Un frame perdido es normal. Muchos seguidos significa que
                # la camara se fue -desconectada, o tomada por otra app-, y
                # sin esto el bucle giraria en vacio quemando un nucleo.
                fallos += 1
                if fallos > 60:
                    vision = cerrar_camara(vision)
                    with CANDADO:
                        CAMARA["error"] = "la camara dejo de entregar imagen"
                    print("Camara perdida: dejo de entregar imagen")
                time.sleep(0.02)
                continue
            fallos = 0
            img, esquinas, ids, gris = extra

            # Watchdog de cuadro muerto.
            #
            # La C920 se cuelga cada tanto y se queda entregando cuadros de
            # relleno: TODOS los pixeles con el mismo valor, desviacion
            # exactamente 0.00. Un cuarto a oscuras nunca da eso, siempre
            # tiene ruido de sensor. Y read() sigue devolviendo True, asi que
            # el contador de fallos de arriba no se entera nunca: la camara
            # figura viva, el video se ve negro y la deteccion da cero sin un
            # solo error en el log. Visto el 2026-09-09, dos veces en una hora.
            #
            # Cerrarla y volver a abrirla la revive. Se submuestrea 1 de cada
            # 8 pixeles: la desviacion sobre el cuadro entero cuesta
            # milisegundos y con 14400 muestras sobra para distinguir una
            # imagen de una pared de un solo color.
            if float(gris[::8, ::8].std()) < 1.0:
                muertos += 1
                if muertos >= 15:      # ~1 s seguido, no un parpadeo
                    print("Camara congelada: cuadros de un solo color. Reabriendo.")
                    vision = cerrar_camara(vision)
                    with CANDADO:
                        CAMARA["solicitado"] = abierta
                    muertos = 0
                    continue
            else:
                muertos = 0

            with CANDADO:
                VISION["encontrados"] = 0 if ids is None else int(len(ids))
            if comparar:
                # Los tres modos sobre el MISMO gris: comparar entre cuadros
                # distintos no diria nada, entre uno y otro cambia la escena.
                # Se calcula FUERA del candado -son tres detecciones, ~25 ms-
                # y solo se guarda el resultado dentro.
                medido = vision.comparar_realces(gris)
                with CANDADO:
                    VISION["comparacion"] = medido

            with CANDADO:
                ESTADO["poses"] = dict(poses)
                ESTADO["robot"] = poses.get(ID_ROBOT)
                ESTADO["destino"] = poses.get(ID_DESTINO)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(img, esquinas, ids)
            # El texto de estado lo pone la pagina: es ella la que sabe que
            # ArUco corresponde a que robot. El cartel que habia aqui decia
            # "falta ID 0" aunque el robot estuviera configurado en otro tag,
            # que es peor que no decir nada. Aqui solo se pintan los marcadores.

            ok_jpg, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok_jpg:
                with FRAME_LOCK:
                    FRAME_JPEG = buf.tobytes()

            # La vista de filtros: exactamente el gris que se le dio al
            # detector, con los marcadores encontrados pintados encima. Sirve
            # para ver POR QUE encuentra o no encuentra, no para mirar bonito.
            with FRAME_LOCK:
                hay_publico = CLIENTES_REALCE > 0
            if hay_publico:
                realzado = cv2.cvtColor(vision.realzar(gris), cv2.COLOR_GRAY2BGR)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(realzado, esquinas, ids)
                ok_r, buf_r = cv2.imencode(
                    ".jpg", realzado, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok_r:
                    with FRAME_LOCK:
                        FRAME_REALCE_JPEG = buf_r.tobytes()

            if not mostrar:
                continue
            cv2.imshow("Camara - alimenta al sandbox", img)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        cerrar_camara(vision)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--puerto", type=int, default=8000)
    parser.add_argument("--camara", type=int, default=None)
    parser.add_argument("--sin-camara", action="store_true")
    parser.add_argument("--sin-ventana", action="store_true")
    parser.add_argument("--escanear-json", action="store_true",
                        help="escanea camaras, imprime la lista en JSON y sale")
    args = parser.parse_args()

    # Modo hijo: lo lanza escanear_camaras() para tener una enumeracion de
    # camaras fresca. Ni servidor ni bucle: escanea, imprime una linea de
    # JSON y se muere. Tambien sirve a mano para ver que hay conectado:
    #     python servidor_sandbox.py --escanear-json
    if args.escanear_json:
        print(json.dumps(escanear_aqui()))
        return

    servidor = arrancar_servidor(args.puerto)
    print()
    print("Sandbox en:  http://localhost:{}".format(args.puerto))
    print()
    print("Config rodillos {} - autoridad de giro {:.0%}".format(
        cinematica.CONFIG_RODILLOS, cinematica.autoridad_de_giro()))
    print("Zona muerta {} - escala minima util {:.0%}".format(
        cinematica.DEADZONE_RAW, cinematica.escala_minima_util()))
    print()

    if args.sin_camara:
        indice = None
        print("Arranca sin camara. Elige una desde la pagina, o arrastra el")
        print("robot y el destino a mano. Ctrl+C para salir.")
    else:
        indice = args.camara
        if indice is None:
            from vision import CAMARA_POR_DEFECTO
            indice = CAMARA_POR_DEFECTO

    # El bucle corre igual aunque no haya camara que abrir: es el que atiende
    # los cambios de camara que pida la pagina. Solo se cae aqui si no hay
    # OpenCV, y entonces el sandbox simulado sigue sirviendose igual.
    try:
        bucle_camara(indice, mostrar=not args.sin_ventana)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Sin camara: {}".format(e))
        print("Sigo sirviendo el sandbox simulado. Ctrl+C para salir.")
        # Sin bucle no hay quien atienda los recados: se deja el estado
        # coherente para que la pagina no se quede con el boton girando.
        with CANDADO:
            CAMARA["escaneando"] = False
            CAMARA["error"] = str(e)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    servidor.shutdown()
    print("\nServidor detenido.")


if __name__ == "__main__":
    main()
