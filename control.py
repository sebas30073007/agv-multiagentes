#!/usr/bin/env python3
"""
control.py - Del error de posicion a las velocidades de rueda.

Es el bloque que va entre vision.py y el envio por MQTT. Sigue la misma
matematica del repo del profe (control_robot.py), reescrita para que los
tres robots del proyecto compartan interfaz.

EL LAZO COMPLETO

    vision.py            ->  (x, y, theta) del robot
    convergencia()       ->  (ux, uy)  velocidad deseada en el mundo
    uniciclo() / holon() ->  velocidades en el marco del robot
    diferencial()        ->  (w_derecha, w_izquierda)
    MQTT                 ->  robot

EJES
Todo asume la convencion del repo: pixeles con origen al centro, Y hacia
ABAJO, angulo positivo en sentido horario. Es la que produce vision.py con
Y_HACIA_ARRIBA = False. Si cambias una, tienes que cambiar la otra.
"""

import math

import numpy as np


# =========================================================
# PARAMETROS
# =========================================================
# Ganancia proporcional. Mas alta = converge mas rapido pero oscila al
# llegar. Empieza bajo y subela hasta que el robot llegue sin rebotar.
K_CONVERGENCIA = 0.8

# Distancia del punto virtual de control, en pixeles. Ver uniciclo().
# No puede ser cero: ahi la matriz se vuelve singular.
L_PUNTO = 25.0

# Geometria del robot, EN PIXELES (medidos sobre la imagen de la camara,
# no con flexometro). Son los valores del repo; ajustalos a tu robot.
RADIO_LLANTA = 58.0
ENTRE_LLANTAS = 120.0

# Saturacion de velocidad de rueda. El firmware recorta de todas formas,
# pero saturar aqui conserva la direccion del movimiento.
W_MAXIMA = 150.0

# Radio de llegada: dentro de esto se considera que ya llego y se detiene.
# Sin esto el robot tiembla indefinidamente alrededor del objetivo por el
# ruido de deteccion.
TOLERANCIA = 15.0


# =========================================================
# LEY DE CONTROL
# =========================================================
def convergencia(x, y, xd, yd, k=K_CONVERGENCIA):
    """
    Control proporcional puro: velocidad proporcional al error, en
    direccion contraria a el.

    Devuelve (ux, uy), la velocidad DESEADA en coordenadas del mundo. Nota
    que todavia no sabe nada del robot: podria ser un punto volando libre.
    Traducir eso a algo que el robot pueda ejecutar es el paso siguiente.
    """
    return (-k * (x - xd), -k * (y - yd))


def llego(x, y, xd, yd, tolerancia=TOLERANCIA):
    return math.hypot(x - xd, y - yd) < tolerancia


# =========================================================
# UNICICLO (Apolo y Odiseo, los dos de dos llantas)
# =========================================================
def uniciclo(ux, uy, theta, l=L_PUNTO):
    """
    Convierte (ux, uy) en (V, W) para un robot diferencial.

    POR QUE NO ES DIRECTO
    Un robot de dos llantas es NO HOLONOMICO: no puede moverse de lado. Si
    le pides una velocidad lateral, fisicamente no la puede ejecutar.

    EL TRUCO
    En vez de controlar el centro del robot, se controla un punto virtual a
    distancia l POR DELANTE. Ese punto si se puede mover en cualquier
    direccion, porque el robot lo consigue combinando avance y giro.

        [ux]   [cos(th)  -l*sin(th)] [V]
        [uy] = [sin(th)   l*cos(th)] [W]

    El determinante de esa matriz es exactamente l. Por eso l=0 la vuelve
    singular: es la forma matematica de decir que un diferencial no se
    puede controlar punto a punto sobre su propio eje.

    OJO: EL ROBOT VA A MANEJAR EN REVERSA
    Este controlador no gobierna la orientacion, solo la posicion del punto
    virtual. Si el destino queda atras, retrocede en linea recta en vez de
    dar la vuelta, porque para ese punto es el camino mas corto. Es
    correcto, no es un error, pero sorprende la primera vez que lo ves.

    Si necesitas que siempre avance de frente, hay que agregar una etapa
    previa que oriente al robot antes de moverlo. No esta en el repo del
    profe ni aqui: seria trabajo extra, decidelo segun lo que pida tu demo.

    ATLAS NO NECESITA ESTO. Es holonomico. Ver holonomico() abajo.
    """
    if abs(l) < 1e-9:
        raise ValueError("l no puede ser cero: la matriz se vuelve singular")

    A = np.array([
        [math.cos(theta), -l * math.sin(theta)],
        [math.sin(theta),  l * math.cos(theta)],
    ])
    V, W = np.linalg.solve(A, np.array([ux, uy], dtype=float))
    return float(V), float(W)


def punto_virtual(x, y, theta, l=L_PUNTO):
    """Posicion del punto que realmente se esta controlando."""
    return (x + l * math.cos(theta), y + l * math.sin(theta))


def diferencial(V, W, r=RADIO_LLANTA, L=ENTRE_LLANTAS, w_max=W_MAXIMA):
    """
    (V, W) -> velocidad angular de cada rueda.

        w_derecha  = V/r + (L*W)/(2*r)
        w_izquierda = V/r - (L*W)/(2*r)

    La saturacion escala AMBAS ruedas por igual en vez de recortar cada
    una. Recortar solo la que se pasa cambia la relacion entre ellas, y con
    eso la curvatura: el robot se desvia de la trayectoria pedida justo
    cuando va rapido, que es cuando peor se nota.
    """
    w_der = V / r + (L * W) / (2.0 * r)
    w_izq = V / r - (L * W) / (2.0 * r)

    pico = max(abs(w_der), abs(w_izq))
    if pico > w_max:
        escala = w_max / pico
        w_der *= escala
        w_izq *= escala

    return w_der, w_izq


# =========================================================
# HOLONOMICO (Atlas)
# =========================================================
def holonomico(ux, uy, theta):
    """
    Para Atlas no hace falta punto virtual ni matriz que invertir: puede
    moverse en cualquier direccion directamente. Solo hay que pasar la
    velocidad del marco del MUNDO al marco del ROBOT, que es una rotacion
    de -theta.

    DEVUELVE (adelante, IZQUIERDA), no (adelante, derecha).

    Es la convencion estandar en robotica: X adelante, Y izquierda. La
    componente lateral sale de proyectar sobre (-sin, cos), que es el rumbo
    girado 90 grados en sentido antihorario, o sea hacia la izquierda.

    OJO AL CONECTARLO CON cinematica.py: ese modulo espera vx hacia la
    DERECHA. Hay que negar:

        vx = -izquierda
        vy = adelante

    Los dos convenios son correctos y comunes; lo que falla es mezclarlos
    sin darse cuenta. El sintoma es que el robot se desplaza de lado en la
    direccion contraria a la pedida.

    Ademas queda un grado de libertad LIBRE: la orientacion. El uniciclo
    tiene que girar para poder avanzar; Atlas no. Decide aparte que quieres
    que haga theta (mantenerse fija, apuntar hacia donde va, etc.) y
    mandalo como el tercer componente.
    """
    c, s = math.cos(theta), math.sin(theta)
    adelante = ux * c + uy * s
    izquierda = -ux * s + uy * c
    return adelante, izquierda


# =========================================================
# LAZO COMPLETO PARA UN ROBOT DIFERENCIAL
# =========================================================
def paso_diferencial(pose, destino, k=K_CONVERGENCIA, l=L_PUNTO,
                     r=RADIO_LLANTA, L=ENTRE_LLANTAS, w_max=W_MAXIMA,
                     tolerancia=TOLERANCIA):
    """
    Un ciclo completo: pose actual + destino -> (w_derecha, w_izquierda).

    pose:    (x, y, theta) tal como sale de vision.py
    destino: (xd, yd)

    QUE LLEGA EXACTAMENTE AL DESTINO
    El PUNTO VIRTUAL, no el centro del robot. Al terminar, el centro queda
    l pixeles CORTO respecto al destino, a lo largo de la orientacion final.

    No es un defecto que se pueda quitar bajando la ganancia: es la
    definicion misma del metodo. Si necesitas el centro sobre el destino,
    baja l (nunca a cero) o compensa el objetivo. Con l=25 px el desfase
    ronda los 5 cm en un encuadre tipico, que casi siempre da igual.

    La llegada se mide sobre el punto virtual justamente para que sea la
    misma vara con la que se controla. Medirla sobre el centro con
    tolerancia < l hace que nunca se cumpla y el robot se quede zumbando
    cerca del destino para siempre.
    """
    x, y, theta = pose
    xd, yd = destino

    px, py = punto_virtual(x, y, theta, l)

    if llego(px, py, xd, yd, tolerancia):
        return 0.0, 0.0, True

    ux, uy = convergencia(px, py, xd, yd, k)
    V, W = uniciclo(ux, uy, theta, l)
    w_der, w_izq = diferencial(V, W, r, L, w_max)
    return w_der, w_izq, False


# =========================================================
# PRUEBAS
# =========================================================
def _pruebas():
    EPS = 1e-9

    # --- La matriz se invierte bien: ida y vuelta da lo mismo ---
    for theta in (0.0, 0.6, 1.9, -2.4):
        for ux, uy in ((10.0, 0.0), (0.0, 10.0), (-4.0, 7.0)):
            V, W = uniciclo(ux, uy, theta)
            # Reconstruir (ux, uy) desde (V, W)
            rx = V * math.cos(theta) - L_PUNTO * W * math.sin(theta)
            ry = V * math.sin(theta) + L_PUNTO * W * math.cos(theta)
            assert abs(rx - ux) < 1e-9 and abs(ry - uy) < 1e-9

    # --- l = 0 debe fallar, no dar basura ---
    try:
        uniciclo(1.0, 1.0, 0.0, l=0.0)
        assert False, "l=0 deberia rechazarse"
    except ValueError:
        pass

    # --- Robot mirando a +x con destino al frente: avanza sin girar ---
    w_der, w_izq, fin = paso_diferencial((0, 0, 0.0), (500, 0))
    assert not fin
    assert w_der > 0 and w_izq > 0
    assert abs(w_der - w_izq) < 1e-6, (w_der, w_izq)

    # --- Destino JUSTO ATRAS: retrocede en linea recta ---
    # No da la vuelta. El controlador gobierna el punto virtual, no la
    # orientacion, y retroceder es el camino mas corto para ese punto.
    # Ver la nota sobre reversa en uniciclo().
    w_der, w_izq, _ = paso_diferencial((0, 0, 0.0), (-500, 0))
    assert w_der < 0 and w_izq < 0, (w_der, w_izq)
    assert abs(w_der - w_izq) < 1e-6, (w_der, w_izq)

    # --- Destino atras pero fuera de eje: retrocede en curva ---
    w_der, w_izq, _ = paso_diferencial((0, 0, 0.0), (-500, 300))
    assert abs(w_der - w_izq) > 1e-3, (w_der, w_izq)

    # --- Destino perpendicular: gira sobre si mismo casi en el lugar ---
    w_der, w_izq, _ = paso_diferencial((0, 0, 0.0), (0, 400))
    assert w_der * w_izq < 0, (w_der, w_izq)

    # --- Ya en el destino: se detiene ---
    # Se prueba con el PUNTO VIRTUAL sobre el destino, que es la condicion
    # real de llegada. Con theta=0 el centro va l pixeles atras.
    w_der, w_izq, fin = paso_diferencial((100 - L_PUNTO, 100, 0.0), (100, 100))
    assert fin and w_der == 0 and w_izq == 0

    # --- El centro queda l corto: es la propiedad del metodo, no un error ---
    px, py = punto_virtual(100 - L_PUNTO, 100, 0.0)
    assert abs(math.hypot(px - 100, py - 100)) < 1e-9
    assert abs(math.hypot((100 - L_PUNTO) - 100, 0)) - L_PUNTO < 1e-9

    # --- Saturacion: nunca se pasa del maximo, y conserva la relacion ---
    w_der, w_izq, _ = paso_diferencial((0, 0, 0.0), (99999, 12345))
    assert max(abs(w_der), abs(w_izq)) <= W_MAXIMA + 1e-9
    sin_sat = diferencial(*uniciclo(*convergencia(
        *punto_virtual(0, 0, 0.0), 99999, 12345), 0.0), w_max=1e12)
    if abs(sin_sat[1]) > EPS and abs(w_izq) > EPS:
        assert abs(sin_sat[0] / sin_sat[1] - w_der / w_izq) < 1e-6

    # --- Holonomico: rotacion pura, la magnitud se conserva ---
    for theta in (0.0, 0.7, 2.2, -1.1):
        a, lat = holonomico(3.0, 4.0, theta)
        assert abs(math.hypot(a, lat) - 5.0) < 1e-9

    # --- Holonomico sin rotacion: pasa tal cual ---
    a, izq = holonomico(3.0, 4.0, 0.0)
    assert abs(a - 3.0) < EPS and abs(izq - 4.0) < EPS

    # --- El lateral apunta a la IZQUIERDA, no a la derecha ---
    # Robot mirando a +x. Un objetivo en +y esta a su izquierda, asi que la
    # componente lateral debe salir POSITIVA. Si esta prueba falla, quien
    # conecte esto con cinematica.py va a mandar el robot al lado contrario.
    a, izq = holonomico(0.0, 10.0, 0.0)
    assert abs(a) < EPS and izq > 0, (a, izq)

    # Robot mirando a +y (girado 90 grados). Un objetivo en +x esta ahora a
    # su DERECHA, asi que el lateral debe salir NEGATIVO.
    a, izq = holonomico(10.0, 0.0, math.pi / 2)
    assert abs(a) < 1e-9 and izq < 0, (a, izq)

    print("Pruebas OK")
    print()
    _simulacion()


def _simulacion():
    """Simula el lazo cerrado para ver si converge de verdad."""
    print("Simulacion: robot en (0,0) mirando a +x, destino (400, -300)")
    print()
    print("{:>5} {:>8} {:>8} {:>8} {:>9} {:>9} {:>9}".format(
        "paso", "x", "y", "th", "w_der", "w_izq", "distancia"))
    print("-" * 62)

    x, y, theta = 0.0, 0.0, 0.0
    destino = (400.0, -300.0)
    dt = 0.1

    for paso in range(1, 121):
        w_der, w_izq, fin = paso_diferencial((x, y, theta), destino)
        px, py = punto_virtual(x, y, theta)
        d_punto = math.hypot(px - destino[0], py - destino[1])
        d_centro = math.hypot(x - destino[0], y - destino[1])

        if fin:
            print("{:>5} {:>8.1f} {:>8.1f} {:>8.2f} {:>9} {:>9} {:>9.1f}".format(
                paso, x, y, theta, "-", "-", d_punto))
            print()
            print("Llego en {} pasos ({:.1f} s a 10 Hz).".format(paso, paso * dt))
            print("Punto virtual a {:.1f} px del destino; centro a {:.1f} px.".format(
                d_punto, d_centro))
            print("Esa diferencia es l = {:.0f} px, por definicion del metodo.".format(
                L_PUNTO))
            return

        # Modelo directo del uniciclo, para cerrar el lazo en simulacion
        V = RADIO_LLANTA * (w_der + w_izq) / 2.0
        W = RADIO_LLANTA * (w_der - w_izq) / ENTRE_LLANTAS
        x += V * math.cos(theta) * dt
        y += V * math.sin(theta) * dt
        theta += W * dt

        if paso % 10 == 0 or paso == 1:
            print("{:>5} {:>8.1f} {:>8.1f} {:>8.2f} {:>9.1f} {:>9.1f} {:>9.1f}".format(
                paso, x, y, theta, w_der, w_izq, d_punto))

    print()
    print("No convergio en 120 pasos. Revisa K_CONVERGENCIA y TOLERANCIA.")


if __name__ == "__main__":
    _pruebas()
