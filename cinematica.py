#!/usr/bin/env python3
"""
cinematica.py - Cinematica inversa para robot omnidireccional de 4 ruedas.

Convierte un vector de velocidad (vx, vy, w) en el frame de 4 tokens que
entiende el ESP32 Maestro: "local,s1,s2,s3".

Convencion de ejes (vista desde arriba, el robot apunta hacia arriba):

              vy (+)  adelante
                 ^
                 |
    izquierda    |    derecha
       <---------+---------> vx (+)
                 |
                 v
              vy (-)  atras

    w (+) = giro antihorario (hacia la izquierda)

Distribucion fisica del robot (vista desde arriba):

        FRENTE
    [1]--------[2]        Llanta 1 = FL = esclavo 0x09
     |          |         Llanta 2 = FR = esclavo 0x08
     |          |         Llanta 3 = RL = esclavo 0x0A
    [3]--------[4]        Llanta 4 = RR = motor local (Maestro)
        ATRAS

Entradas: vx, vy, w en [-1.0, 1.0]
Salida:   string listo para mandar por serial o por MQTT.

Uso:
    from cinematica import a_frame
    a_frame(vx=0, vy=1, w=0)          # adelante a fondo
    a_frame(vx=1, vy=0, w=0, 0.3)     # lateral derecha al 30%
"""

import math

# =========================================================
# CONSTANTES DEL FIRMWARE
# Deben coincidir con main_puenteH_bueno.py
# =========================================================
MAX_CMD_RAW = 255


# =========================================================
# CALIBRACION - LLENAR DESPUES DE CARACTERIZAR EL ROBOT
# =========================================================
#
# Cada entrada corresponde a una posicion del frame "local,s1,s2,s3".
# El orden de las posiciones NO se puede cambiar, lo define el firmware.
#
#   ("NOMBRE_RUEDA", SIGNO)
#
# NOMBRE_RUEDA: cual de las 4 ruedas fisicas responde a esa posicion.
#               Valores validos: "FL", "FR", "RL", "RR"
#               (Front-Left, Front-Right, Rear-Left, Rear-Right)
#
# SIGNO:        +1 si un valor positivo empuja el robot hacia ADELANTE
#               -1 si un valor positivo empuja el robot hacia ATRAS
#
# Como llenarlo: manda "120,S,S,S", "S,120,S,S", etc. con el robot en
# blocks y anota que rueda gira y hacia donde empujaria.
#
# POSICIONES: MEDIDAS en el robot el 2026-08-31, no deducidas de la foto.
#
# Se mando "0,100,0,0", "0,0,100,0", "100,0,0,0" y "0,0,0,100" por separado,
# 5 s cada uno, anotando que rueda giraba. El resultado NO coincide con lo
# que decia el README del firmware: las posiciones 0 y 2 estaban cruzadas.
#
#   README decia:  RR, FR, FL, RL
#   Realidad:      FL, FR, RR, RL
#
# El maestro confirmo la posicion 0 por su cuenta: con "100,0,0,0" reporto
# LOCAL_SET=100, o sea que la posicion 0 es su motor local, y ese motor
# resulto ser el frontal izquierdo.
#
# Si vuelves a medir y te sale distinto, cambia AQUI y nada mas aqui: la
# pagina del sandbox lee este orden desde /config, no lo tiene duplicado.
#
# SIGNOS: MEDIDOS el 2026-08-31. Los cuatro motores estan cableados al
# reves respecto a la convencion de este modulo (positivo = adelante), asi
# que los cuatro van en -1. Al ser todos iguales, el efecto neto es que el
# robot avanzaba exactamente al contrario de lo pedido, sin deformar la
# mezcla mecanum: por eso no se notaba como "se va de lado" sino como "se
# va al reves".
#
FRAME_LAYOUT = (
    ("FL", -1),   # posicion 0 -> motor local (Maestro) = frontal izquierdo
    ("FR", -1),   # posicion 1 -> esclavo 0x08          = frontal derecho
    ("RR", -1),   # posicion 2 -> esclavo 0x09          = trasero derecho
    ("RL", -1),   # posicion 3 -> esclavo 0x0A          = trasero izquierdo
)

# Valor raw minimo al que los motores realmente arrancan.
# Abajo de esto se manda "S" en lugar de un numero, para no dejar
# motores zumbando sin girar.
# Medelo bajando de 120 hasta que la rueda deje de moverse.
DEADZONE_RAW = 30

# Cuanto pesa la rotacion frente a la traslacion.
# Depende de la distancia entre ruedas. Si el robot gira demasiado
# rapido al combinar avance y giro, bajalo a 0.7 u 0.5.
TURN_GAIN = 1.0


# =========================================================
# CONFIGURACION DE RODILLOS
# =========================================================
#
# Vista CENITAL (directamente desde arriba), mirando el rodillo mas
# alto de cada rueda:
#
#   "X" -> FL=\  FR=/  RL=/  RR=\     las 4 lineas apuntan al centro
#   "O" -> FL=/  FR=\  RL=\  RR=/     las 4 lineas se alejan del centro
#
# OJO: el rodillo que toca el piso aparece ESPEJEADO respecto al de
# arriba. La convencion se define sobre los rodillos SUPERIORES.
#
# Consecuencias de la config "O":
#   - El lateral se invierte (esto lo corrige el codigo).
#   - El torque de giro es proporcional a (media_via - medio_entreejes)
#     en lugar de la suma. Si el robot es casi cuadrado, casi no gira.
#     Eso NO se arregla por software: hay que intercambiar las ruedas
#     de cada eje (1 con 2, y 3 con 4) para convertir O en X.
#
CONFIG_RODILLOS = "O"

# Si al pedir giro el robot rota al reves, ponlo en True.
# Pasa en config "O" cuando el entre-ejes es mayor que el ancho de via.
# Con las medidas de abajo (84 > 51) NO aplica: el giro sale correcto.
INVERTIR_GIRO = False

# Si al pedir lateral el robot se va al lado contrario, ponlo en True.
#
# MEDIDO el 2026-09-02 sobre el Atlas de verdad: pidiendo lateral derecha se
# iba a la izquierda, con el avance y el giro correctos. Ese patron -solo el
# eje lateral invertido- es la firma de que la configuracion de rodillos
# declarada arriba no es la que lleva montada; el comentario de CONFIG_RODILLOS
# ya avisa de lo facil que es leerla al reves, porque el rodillo que toca el
# piso se ve espejeado respecto al de arriba.
#
# Se corrige con una bandera propia y NO cambiando CONFIG_RODILLOS a "X",
# porque esa constante gobierna dos cosas a la vez: el signo del lateral y
# la autoridad de giro (autoridad_de_giro(), 24% en "O" y 100% en "X"). El
# lateral esta medido; la autoridad no. Ponerla en "X" arreglaria el lateral
# de rebote y de paso le diria al simulador que Atlas gira cuatro veces mas
# rapido de lo que gira, que es el error contra el que avisa cinematica_directa().
#
# Si algun dia se comprueba la geometria mirando los rodillos, lo correcto es
# arreglar CONFIG_RODILLOS y devolver esta bandera a False.
INVERTIR_LATERAL = True


# =========================================================
# DIMENSIONES MEDIDAS (centro a centro de llanta)
# =========================================================
ANCHO_VIA = 84.0    # llanta 1 <-> llanta 2 (izquierda-derecha)
ENTRE_EJES = 51.0   # llanta 1 <-> llanta 3 (frente-atras)

# Diametro de llanta, en cm. NO entra en el calculo de a_frame(): el control
# trabaja en pixeles, este modulo normaliza a [-1, 1] y el firmware recibe
# crudos de -255 a 255. El factor de escala se absorbe en la ganancia.
#
# Sirve para dos cosas:
#   1. Convertir a velocidades angulares reales (rad/s) si algun dia hacen
#      falta: w_rueda = v_rueda / (DIAMETRO_LLANTA / 2)
#   2. Calibrar la escala pixel<->cm de la camara sin instrumentos, usando
#      ANCHO_VIA como regla. Ver la nota en piloto_atlas.py.
DIAMETRO_LLANTA = 21.0


def autoridad_de_giro():
    """
    Fraccion del torque de giro disponible frente a la configuracion X.

    El torque neto de rotacion es proporcional a:
        config X:  ANCHO_VIA + ENTRE_EJES
        config O:  ANCHO_VIA - ENTRE_EJES

    Devuelve 1.0 en config X. En config O devuelve la razon entre ambos:
    valores bajos significan que casi toda la fuerza de los motores se
    pierde en los rodillos raspando en vez de hacer girar el robot.
    Un valor negativo significa que el robot gira al reves de lo pedido.
    """
    if CONFIG_RODILLOS.upper() != "O":
        return 1.0
    suma = ANCHO_VIA + ENTRE_EJES
    if suma == 0:
        return 0.0
    return (ANCHO_VIA - ENTRE_EJES) / suma


# =========================================================
# HELPERS
# =========================================================
def clamp(valor, lo, hi):
    if valor < lo:
        return lo
    if valor > hi:
        return hi
    return valor


# =========================================================
# MEZCLA CINEMATICA
# =========================================================
def mezcla_mecanum(vx, vy, w):
    """
    Devuelve un dict {"FL": v, "FR": v, "RL": v, "RR": v} con valores
    normalizados en [-1.0, 1.0].

    Esta funcion es puramente geometrica: no sabe nada del cableado ni
    del orden del frame. Por eso las pruebas de abajo la validan a ella
    y no al frame final.
    """
    vx = clamp(float(vx), -1.0, 1.0)
    vy = clamp(float(vy), -1.0, 1.0)
    w = clamp(float(w), -1.0, 1.0) * TURN_GAIN

    # Las ecuaciones de abajo estan escritas para configuracion "X".
    # En "O" los rodillos apuntan al reves, asi que el eje lateral
    # se invierte. El avance no cambia en ninguna de las dos.
    if CONFIG_RODILLOS.upper() == "O":
        vx = -vx

    if INVERTIR_LATERAL:
        vx = -vx

    if INVERTIR_GIRO:
        w = -w

    ruedas = {
        "FL": vy + vx - w,
        "FR": vy - vx + w,
        "RL": vy - vx - w,
        "RR": vy + vx + w,
    }

    # Normalizacion: si alguna rueda se pasa de 1.0, escalamos TODAS por
    # igual. Asi se conserva la direccion del movimiento en lugar de
    # recortar cada rueda por separado, que deformaria la trayectoria.
    pico = max(1.0, max(abs(v) for v in ruedas.values()))
    return {nombre: valor / pico for nombre, valor in ruedas.items()}


def a_frame(vx, vy, w, escala=1.0):
    """
    Convierte (vx, vy, w) en el string "local,s1,s2,s3".

    escala: factor global de velocidad en [0.0, 1.0].
            Usa 0.2 - 0.3 para las primeras pruebas en piso.

    MANEJO DE LA ZONA MUERTA
    Lo intuitivo seria anular cada rueda que quede por debajo del umbral.
    Es un error grave: rompe la proporcion entre ruedas, que es justamente
    lo que hace funcionar la mezcla mecanum, y el robot se ATASCA.

    El caso concreto: con escala 0.20 el crudo maximo es 51. Si la zona
    muerta es 30, una rueda solo se mueve si su valor normalizado supera
    0.59. Al acercarse al destino los comandos se hacen chicos, TODAS las
    ruedas caen bajo el umbral, la trama se vuelve "S,S,S,S" y el robot se
    queda congelado a media maniobra sin llegar nunca.

    Lo correcto es escalar TODO el vector por igual hasta que la rueda
    mayor supere la zona muerta. Asi se conserva la direccion del
    movimiento y solo se pierde la capacidad de ir despacio, que es una
    limitacion fisica real del motor y no se puede evitar.
    """
    normalizado = mezcla_mecanum(vx, vy, w)
    escala = clamp(float(escala), 0.0, 1.0)

    crudos = [normalizado[nombre] * signo * escala * MAX_CMD_RAW
              for nombre, signo in FRAME_LAYOUT]

    pico = max(abs(c) for c in crudos)
    if pico < 1e-9:
        return frame_paro()

    if pico < DEADZONE_RAW:
        factor = DEADZONE_RAW / pico
        crudos = [c * factor for c in crudos]

    tokens = []
    for crudo in crudos:
        raw = clamp(int(round(crudo)), -MAX_CMD_RAW, MAX_CMD_RAW)
        # Una rueda por debajo de su zona muerta no gira aunque le mandes
        # algo. Mandar "S" es lo honesto y evita que el motor zumbe parado.
        tokens.append("S" if abs(raw) < DEADZONE_RAW else str(raw))

    return ",".join(tokens)


def frame_paro():
    """Frame de paro. Equivale a soltar todos los motores."""
    return "S,S,S,S"


def escala_minima_util(fraccion_menor=0.3):
    """
    Escala por debajo de la cual el movimiento deja de ser fiel.

    La zona muerta no solo impide ir despacio: a escalas bajas la rueda
    dominante supera el umbral pero las demas no, asi que solo giran
    algunas y el robot se mueve en una direccion APROXIMADA, no en la
    pedida. Se mueve, pero no hacia donde le dijiste.

    Una rueda con valor normalizado n arranca cuando n * escala * 255 pasa
    de DEADZONE_RAW. Para que una rueda que vale 'fraccion_menor' del pico
    tambien arranque:

        escala > DEADZONE_RAW / (fraccion_menor * MAX_CMD_RAW)

    Con DEADZONE_RAW = 30 y ruedas secundarias al 30% del pico, hace falta
    escala > 0.39. Por eso conviene medir la zona muerta real con
    calibrar.py: si sale mas baja que 30, ganas rango util directamente.
    """
    return DEADZONE_RAW / (fraccion_menor * MAX_CMD_RAW)


# =========================================================
# MOVIMIENTOS PREDEFINIDOS
# =========================================================
MOVIMIENTOS = {
    "adelante":         (0.0,  1.0,  0.0),
    "atras":            (0.0, -1.0,  0.0),
    "derecha":          (1.0,  0.0,  0.0),
    "izquierda":       (-1.0,  0.0,  0.0),
    "diag_adel_der":    (1.0,  1.0,  0.0),
    "diag_adel_izq":   (-1.0,  1.0,  0.0),
    "diag_atras_der":   (1.0, -1.0,  0.0),
    "diag_atras_izq":  (-1.0, -1.0,  0.0),
    "girar_izquierda":  (0.0,  0.0,  1.0),
    "girar_derecha":    (0.0,  0.0, -1.0),
    "paro":             (0.0,  0.0,  0.0),
}


def movimiento(nombre, escala=1.0):
    """Atajo: movimiento('adelante', 0.3)"""
    if nombre not in MOVIMIENTOS:
        raise ValueError("Movimiento desconocido: {}".format(nombre))
    vx, vy, w = MOVIMIENTOS[nombre]
    return a_frame(vx, vy, w, escala)


# =========================================================
# CINEMATICA DIRECTA
# =========================================================
def cinematica_directa(frame):
    """
    El camino inverso: de las velocidades de rueda al movimiento del robot.

    Es lo que necesita un simulador, porque la "planta" solo recibe los
    cuatro numeros de la trama, igual que el robot real. No ve el vector de
    velocidad que los origino.

    Acepta el string "40,40,40,40" o una lista de cuatro numeros.
    Devuelve (vx, vy, w) normalizados, en la misma convencion que entra a
    a_frame(): vx a la derecha, vy adelante, w antihorario.

    Se despeja del sistema de mezcla_mecanum():
        vy = ( FL + FR + RL + RR) / 4
        vx = ( FL - FR - RL + RR) / 4
        w  = (-FL + FR - RL + RR) / 4

    OJO: devuelve la rotacion CINEMATICA, sin descontar la perdida por la
    configuracion de rodillos. Un simulador fiel debe multiplicar w por
    autoridad_de_giro(); si no, el robot simulado va a girar cuatro veces
    mas rapido que el real y las ganancias que ajustes no van a servir.
    """
    if isinstance(frame, str):
        partes = [p.strip() for p in frame.split(",")]
        if len(partes) != 4:
            raise ValueError("La trama debe tener 4 tokens")
        valores = [0.0 if p.upper() in ("S", "") else float(p) for p in partes]
    else:
        valores = [float(v) for v in frame]
        if len(valores) != 4:
            raise ValueError("Se esperan 4 valores")

    ruedas = {}
    for (nombre, signo), crudo in zip(FRAME_LAYOUT, valores):
        ruedas[nombre] = crudo / MAX_CMD_RAW / signo

    vy = (ruedas["FL"] + ruedas["FR"] + ruedas["RL"] + ruedas["RR"]) / 4.0
    vx = (ruedas["FL"] - ruedas["FR"] - ruedas["RL"] + ruedas["RR"]) / 4.0
    w = (-ruedas["FL"] + ruedas["FR"] - ruedas["RL"] + ruedas["RR"]) / 4.0

    if CONFIG_RODILLOS.upper() == "O":
        vx = -vx
    if INVERTIR_LATERAL:
        vx = -vx
    if INVERTIR_GIRO:
        w = -w

    return vx, vy, w


# =========================================================
# PRUEBAS
# =========================================================
def _pruebas():
    EPS = 1e-9
    m = mezcla_mecanum

    # --- Adelante: las 4 ruedas empujan igual y hacia adelante ---
    r = m(0, 1, 0)
    assert all(abs(v - 1.0) < EPS for v in r.values()), r

    # --- Atras: espejo exacto de adelante ---
    r = m(0, -1, 0)
    assert all(abs(v + 1.0) < EPS for v in r.values()), r

    # --- Lateral derecha: diagonales opuestas, suma cero ---
    # En config "X" la diagonal FL/RR avanza; en "O" es la FR/RL. Y si el
    # lateral esta invertido a mano, todo el eje se espeja otra vez.
    r = m(1, 0, 0)
    signo = -1.0 if CONFIG_RODILLOS.upper() == "O" else 1.0
    if INVERTIR_LATERAL:
        signo = -signo
    assert r["FL"] * signo > 0 and r["RR"] * signo > 0, r
    assert r["FR"] * signo < 0 and r["RL"] * signo < 0, r
    assert abs(sum(r.values())) < EPS, r

    # --- Giro antihorario: lado izquierdo atras, derecho adelante ---
    r = m(0, 0, 1)
    g = -1.0 if INVERTIR_GIRO else 1.0
    assert r["FL"] * g < 0 and r["RL"] * g < 0, r
    assert r["FR"] * g > 0 and r["RR"] * g > 0, r

    # --- Diagonal: dos ruedas quedan quietas (firma del mecanum) ---
    r = m(1, 1, 0)
    lateral_o = (CONFIG_RODILLOS.upper() == "O") != bool(INVERTIR_LATERAL)
    quietas = ("FL", "RR") if lateral_o else ("FR", "RL")
    activas = ("FR", "RL") if lateral_o else ("FL", "RR")
    assert all(abs(r[k]) < EPS for k in quietas), r
    assert all(r[k] > 0 for k in activas), r

    # --- Nunca se satura, ni en la peor combinacion ---
    valores = (-1.0, -0.5, 0.0, 0.5, 1.0)
    for vx in valores:
        for vy in valores:
            for w in valores:
                r = m(vx, vy, w)
                pico = max(abs(v) for v in r.values())
                assert pico <= 1.0 + EPS, (vx, vy, w, r)

    # --- Ida y vuelta del eje lateral, con las banderas puestas ---
    # Es la prueba que faltaba: a_frame() y cinematica_directa() aplican los
    # espejos en sitios distintos, y sin esto una bandera nueva podia
    # colarse en uno y no en el otro.
    vx_, vy_, w_ = cinematica_directa(a_frame(vx=1.0, vy=0.0, w=0.0, escala=1.0))
    assert vx_ > 0.5 and abs(vy_) < 0.05 and abs(w_) < 0.05, (vx_, vy_, w_)

    # --- El paro siempre es paro, sin importar la calibracion ---
    assert a_frame(0, 0, 0) == "S,S,S,S"
    assert frame_paro() == "S,S,S,S"

    # --- ZONA MUERTA: cualquier peticion real debe producir movimiento ---
    # Esta es la prueba que atrapa el atascamiento. Si se anulara cada rueda
    # por separado, a escalas bajas la trama saldria "S,S,S,S" y el robot se
    # quedaria congelado a media maniobra.
    for escala_prueba in (0.05, 0.1, 0.2, 0.5, 1.0):
        for vx_p, vy_p, w_p in ((0, 1, 0), (1, 0, 0), (0, 0, 1), (0.4, 0.4, 0)):
            frame = a_frame(vx_p, vy_p, w_p, escala_prueba)
            assert frame != "S,S,S,S", (escala_prueba, vx_p, vy_p, w_p, frame)

    # --- Se conserva la proporcion cuando las cuatro ruedas estan activas ---
    def numeros(frame):
        return [0.0 if t == "S" else float(t) for t in frame.split(",")]

    alto = numeros(a_frame(0.4, 0.9, 0.2, 1.0))
    medio = numeros(a_frame(0.4, 0.9, 0.2, 0.6))
    if all(abs(v) > 0 for v in alto) and all(abs(v) > 0 for v in medio):
        for i in range(1, 4):
            assert abs(medio[i] / medio[0] - alto[i] / alto[0]) < 0.08, (medio, alto)

    # --- A escala baja se pierden ruedas: es fisica, no un error ---
    # La rueda dominante llega al umbral pero las demas no, asi que el robot
    # se mueve en una direccion aproximada, no exacta. Ver escala_minima_util().
    bajo = numeros(a_frame(0.4, 0.9, 0.2, 0.06))
    assert sum(1 for v in bajo if v != 0) < 4, bajo

    # --- El frame siempre tiene 4 tokens (el firmware lo exige) ---
    for nombre in MOVIMIENTOS:
        frame = movimiento(nombre, 1.0)
        assert len(frame.split(",")) == 4, (nombre, frame)

    # --- Ningun token se pasa del rango del firmware ---
    for vx in valores:
        for vy in valores:
            for w in valores:
                for token in a_frame(vx, vy, w).split(","):
                    if token != "S":
                        assert -MAX_CMD_RAW <= int(token) <= MAX_CMD_RAW

    # --- CINEMATICA DIRECTA: ida y vuelta exacta ---
    # Sin cuantizar ni zona muerta, a_frame y cinematica_directa deben ser
    # inversas una de otra salvo el factor de normalizacion. Se compara la
    # DIRECCION del vector, que es lo que debe conservarse.
    for vx_p, vy_p, w_p in ((0, 1, 0), (1, 0, 0), (0, 0, 1),
                            (0.7, 0.7, 0), (-0.5, 0.3, 0.4)):
        n = mezcla_mecanum(vx_p, vy_p, w_p)
        crudos = [n[nom] * sig * MAX_CMD_RAW for nom, sig in FRAME_LAYOUT]
        obtenido = cinematica_directa(crudos)
        pedido = (vx_p, vy_p, w_p)
        np_ = math.sqrt(sum(v * v for v in pedido))
        no_ = math.sqrt(sum(v * v for v in obtenido))
        if np_ > EPS and no_ > EPS:
            coseno = sum(a * b for a, b in zip(pedido, obtenido)) / (np_ * no_)
            assert coseno > 0.999, (pedido, obtenido, coseno)

    # --- El paro no produce movimiento ---
    assert all(abs(v) < EPS for v in cinematica_directa("S,S,S,S"))

    # --- Acepta string y lista por igual ---
    por_texto = cinematica_directa("60,60,60,60")
    por_lista = cinematica_directa([60, 60, 60, 60])
    assert all(abs(x - y) < EPS for x, y in zip(por_texto, por_lista))

    # --- Avanzar produce vy positiva y nada mas ---
    dvx, dvy, dw = cinematica_directa(a_frame(0, 1, 0, 1.0))
    assert dvy > 0.9 and abs(dvx) < EPS and abs(dw) < EPS, (dvx, dvy, dw)

    print("Pruebas OK\n")


def _instrucciones_polaridad():
    print("PENDIENTE: medir los signos de FRAME_LAYOUT")
    print("-" * 64)
    print("Con el robot en blocks y despues de ARM, manda uno por uno:")
    print()
    pruebas = (
        ("120,S,S,S", "local (Maestro)", "llanta 4", "RR"),
        ("S,120,S,S", "esclavo 0x08",    "llanta 2", "FR"),
        ("S,S,120,S", "esclavo 0x09",    "llanta 1", "FL"),
        ("S,S,S,120", "esclavo 0x0A",    "llanta 3", "RL"),
    )
    for frame, modulo, llanta, rueda in pruebas:
        print('  python3 enviar_comando.py "{}"'.format(frame))
        print("      -> debe girar {} ({}), asignada a {}".format(
            llanta, modulo, rueda))
        print("      -> si empuja ADELANTE deja +1, si empuja ATRAS pon -1")
        print()
    print("Anota tambien el raw minimo al que cada rueda arranca")
    print("(baja de 120 a 100, 80, 60...) para ajustar DEADZONE_RAW.")
    print()


def _tabla():
    posiciones = " / ".join(nombre for nombre, _ in FRAME_LAYOUT)
    print("Layout actual (local / s1 / s2 / s3): {}".format(posiciones))
    print("Signos: {}".format(", ".join(
        "{}={:+d}".format(n, s) for n, s in FRAME_LAYOUT)))
    print("Zona muerta: {}   Ganancia de giro: {}".format(
        DEADZONE_RAW, TURN_GAIN))
    print("Config rodillos: {}   Giro invertido: {}".format(
        CONFIG_RODILLOS.upper(), INVERTIR_GIRO))

    autoridad = autoridad_de_giro()
    print("Via {:.0f} / entre-ejes {:.0f}  ->  autoridad de giro: {:.0f}%".format(
        ANCHO_VIA, ENTRE_EJES, autoridad * 100))
    if autoridad < 0:
        print("  AVISO: torque negativo, el robot gira al reves.")
        print("  Pon INVERTIR_GIRO = True, o mejor pasa a config X.")
    elif autoridad < 0.4:
        print("  AVISO: giro debil. No hay arreglo por software: el giro")
        print("  puro ya manda 255 a las 4 ruedas. Para recuperarlo hay")
        print("  que intercambiar las llantas de cada eje (1<->2, 3<->4).")
    print()

    print("{:<18} {:>22} {:>22}".format("MOVIMIENTO", "100%", "30%"))
    print("-" * 64)
    for nombre in MOVIMIENTOS:
        print("{:<18} {:>22} {:>22}".format(
            nombre, movimiento(nombre, 1.0), movimiento(nombre, 0.3)))
    print()
    print("Recuerda: el firmware para los motores a los 2.5 s sin recibir")
    print("comando. Para movimiento continuo hay que repetir el frame.")


if __name__ == "__main__":
    _pruebas()
    _instrucciones_polaridad()
    _tabla()
