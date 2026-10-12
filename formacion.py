#!/usr/bin/env python3
"""
formacion.py - Repulsores y control de formacion por grafo completo.

Recuperado y adaptado de robomaster-vicon-integration (ver
_vendor/robomaster-vicon-integration/NOTAS.md para el detalle de que se tomo
de donde). Alla el control corria sobre RoboMaster S1 con drive_speed()
directo; aqui se deja todo en el dominio de "velocidad deseada en el mundo"
(ui), igual que convergencia() en control.py, para que sea agnostico de que
tipo de robot la ejecuta despues (holonomico como Atlas, o uniciclo).

EL LAZO COMPLETO (cuando se conecte a la simulacion)

    posiciones de todos los robots (Z)
    grafo_completo()          -> A, quien ve a quien
    control_formacion()       -> ui de formacion, por robot
    repulsion_entre_robots()  -> ui de anti-colision, por robot
    repulsion_de_obstaculos() -> ui de los repulsores dibujados en el mapa
    (suma de las tres)        -> velocidad deseada en el mundo, por robot
    holonomico() / uniciclo() en control.py -> lo que cada robot puede
                                                ejecutar de verdad

REPULSORES: incluye las 4 formulas que aparecian intercambiadas (comentando
unas y descomentando otra) en 3ForAngRep.py y 3MarAngRepBer.py. Las cuatro
comparten el mismo contrato: cero en d >= r, crecen hacia el centro, y
apuntan siempre alejando del repulsor. Lo que cambia es la forma de la
curva - que tan brusco es el frenazo cerca del borde vs cerca del centro.
"""

import math

import numpy as np


# =========================================================
# REPULSORES
# =========================================================
def _lineal(d, r, eta):
    return eta * (r - d)


def _lineal_normalizado(d, r, eta):
    return eta * (1.0 - d / r)


def _espiral(d, r, eta):
    """
    Mas agresivo cerca del centro que las lineales: crece con 1/d.

    OJO: asume r < 1 (radios tipicos en metros, ej. 0.2-0.6 en el repo
    original). Con r >= 1, r**4 >= r y la formula da negativa (atractiva)
    en todo el dominio d <= r - deja de ser un repulsor. Es una propiedad
    de la formula original, no algo que se pueda arreglar sin cambiar la
    curva; si necesitas radios >= 1, usa "khatib" o "lineal" en su lugar.
    """
    return eta * (1.0 - (r ** 4) / d)


def _khatib(d, r, eta):
    """
    Potencial repulsivo clasico (Khatib, 1986). Es el que se suele ver en
    los libros de campos potenciales; las otras tres son variantes mas
    simples pensadas para ajustarse a mano en campo.
    """
    return (2.0 * eta / d ** 4) * (1.0 / d ** 2 - 1.0 / r ** 2)


FORMULAS_REPULSOR = {
    "lineal": _lineal,
    "lineal_norm": _lineal_normalizado,
    "espiral": _espiral,
    "khatib": _khatib,
}

ETA_DEFECTO = 1.5     # ganancia repulsiva
R_DEFECTO = 0.6        # radio de corte, en las mismas unidades que las posiciones


def fuerza_repulsiva(delta, eta=ETA_DEFECTO, r=R_DEFECTO, tipo="lineal"):
    """
    Un solo repulsor, generico: 'delta' es POSICION_ROBOT - POSICION_REPULSOR
    (no hace falta normalizar antes). Sirve igual para el repulsor siendo
    otro robot (repulsion_entre_robots) o un punto fijo dibujado en el mapa
    (repulsion_de_obstaculos) - mismo contrato en los dos casos.

    Fuera del radio r, o si el repulsor cae justo encima del robot (d~0,
    direccion indefinida), la fuerza es cero: no hay empujon fantasma.
    """
    if tipo not in FORMULAS_REPULSOR:
        raise ValueError("tipo de repulsor desconocido: {!r} (opciones: {})".format(
            tipo, ", ".join(FORMULAS_REPULSOR)))

    d = float(np.linalg.norm(delta))
    if d >= r or d < 1e-6:
        return np.zeros(2)

    direccion = np.asarray(delta, dtype=float) / d
    magnitud = FORMULAS_REPULSOR[tipo](d, r, eta)
    return magnitud * direccion


def repulsion_entre_robots(Z, eta=ETA_DEFECTO, r=R_DEFECTO, tipo="lineal"):
    """
    Cada robot se aleja de cada otro robot que tenga a menos de r. Es el
    caso "todos contra todos": no hace falta una matriz de adyacencia
    aparte, la propia distancia decide quien empuja a quien (a diferencia
    de control_formacion(), donde el grafo SI importa).
    """
    Z = np.asarray(Z, dtype=float)
    n = Z.shape[0]
    R = np.zeros_like(Z)
    for i in range(n):
        for j in range(n):
            if i != j:
                R[i, :] += fuerza_repulsiva(Z[i, :] - Z[j, :], eta, r, tipo)
    return R


def repulsion_de_obstaculos(Z, obstaculos, eta=ETA_DEFECTO, r=R_DEFECTO, tipo="lineal"):
    """
    Igual que repulsion_entre_robots(), pero contra puntos fijos en vez de
    otros robots - los repulsores que se dibujan en el mapa del sandbox.

    obstaculos: lista de dicts, cada uno con al menos {"x", "y"} y
    opcionalmente "eta", "r", "tipo" propios (si faltan, se usan los
    valores por defecto de la funcion). Asi cada repulsor dibujado puede
    tener su propio tipo/fuerza/alcance, no todos igual.
    """
    Z = np.asarray(Z, dtype=float)
    n = Z.shape[0]
    R = np.zeros_like(Z)
    for i in range(n):
        for o in obstaculos:
            centro = np.array([o["x"], o["y"]], dtype=float)
            R[i, :] += fuerza_repulsiva(
                Z[i, :] - centro,
                o.get("eta", eta), o.get("r", r), o.get("tipo", tipo),
            )
    return R


# =========================================================
# FORMACION POR GRAFO COMPLETO
# =========================================================
K_FORMACION = 1.0


def grafo_completo(n):
    """Matriz de adyacencia todos-con-todos: cada robot ve a los otros n-1."""
    return np.ones((n, n)) - np.eye(n)


def control_formacion(Z, Zd, A, k=K_FORMACION, normalizar=True):
    """
    Ley de consenso de formacion, generalizada a N robots:

        ui = -k * SUM_j A[i,j] * ( (Zi - Zj) - (Zdi - Zdj) )

    Zi - Zj es la posicion relativa ACTUAL entre el robot i y el j;
    Zdi - Zdj es la que deberian tener. El error entre ambas, sumado sobre
    cada vecino segun A, es lo que se corrige. Con A = grafo_completo(n)
    cada robot usa a TODOS los demas como referencia (mas informacion, mas
    robusto a que un robot se pierda un frame); con un grafo mas ralo cada
    uno solo mira a sus vecinos directos.

    normalizar=True divide por el grado (numero de vecinos) de cada robot,
    asi la ganancia efectiva no depende de cuantos vecinos tenga - importa
    en grafos no completos o no regulares; en un grafo completo todos los
    grados son iguales (n-1) asi que solo reescala k por esa constante.
    Si necesitas replicar EXACTO el comportamiento de CtrlForm6RobVICON.py
    (que no normaliza), usa normalizar=False.
    """
    Z = np.asarray(Z, dtype=float)
    Zd = np.asarray(Zd, dtype=float)
    A = np.asarray(A, dtype=float)
    n = Z.shape[0]

    U = np.zeros_like(Z)
    for i in range(n):
        grado = 0
        for j in range(n):
            if i != j and A[i, j]:
                U[i, :] -= k * ((Z[i, :] - Z[j, :]) - (Zd[i, :] - Zd[j, :]))
                grado += 1
        if normalizar and grado > 0:
            U[i, :] /= grado
    return U


# =========================================================
# PRUEBAS
# =========================================================
def _pruebas():
    EPS = 1e-9

    # --- Cada formula de repulsor: cero fuera de r, empuja hacia afuera adentro ---
    for tipo in FORMULAS_REPULSOR:
        # justo en el borde: practicamente cero (las 4 formulas se anulan en d=r)
        f_borde = fuerza_repulsiva(np.array([R_DEFECTO, 0.0]), r=R_DEFECTO, tipo=tipo)
        assert np.linalg.norm(f_borde) < 1e-6, (tipo, f_borde)

        # fuera del radio: exactamente cero
        f_fuera = fuerza_repulsiva(np.array([R_DEFECTO * 2, 0.0]), r=R_DEFECTO, tipo=tipo)
        assert np.array_equal(f_fuera, np.zeros(2)), (tipo, f_fuera)

        # a la mitad del radio: apunta en la MISMA direccion que delta (alejando)
        f_cerca = fuerza_repulsiva(np.array([R_DEFECTO / 2, 0.0]), r=R_DEFECTO, eta=1.0, tipo=tipo)
        assert f_cerca[0] > 0 and abs(f_cerca[1]) < EPS, (tipo, f_cerca)

    # --- repulsor pegado encima del robot: no explota, da cero ---
    f_encima = fuerza_repulsiva(np.array([0.0, 0.0]), tipo="khatib")
    assert np.array_equal(f_encima, np.zeros(2))

    # --- repulsion_entre_robots: 2 robots lejos, sin empujon ---
    Z_lejos = np.array([[0.0, 0.0], [10.0, 10.0]])
    assert np.allclose(repulsion_entre_robots(Z_lejos), 0.0)

    # --- repulsion_entre_robots: 2 robots encimados, se separan en direcciones opuestas ---
    Z_cerca = np.array([[0.0, 0.0], [0.1, 0.0]])
    R = repulsion_entre_robots(Z_cerca, eta=1.0, r=R_DEFECTO)
    assert R[0, 0] < 0 and R[1, 0] > 0, R  # cada uno huye del otro

    # --- repulsion_de_obstaculos: un repulsor con su propio tipo/radio ---
    obst = [{"x": 0.0, "y": 0.0, "r": 0.6, "eta": 2.0, "tipo": "espiral"}]
    Ro = repulsion_de_obstaculos(np.array([[0.3, 0.0]]), obst)
    assert Ro[0, 0] > 0  # el robot esta a la derecha del repulsor, lo empuja mas a la derecha

    # --- grafo_completo: todos conectados menos consigo mismo ---
    A3 = grafo_completo(3)
    assert np.array_equal(A3, np.array([[0, 1, 1], [1, 0, 1], [1, 1, 0]]))

    # --- control_formacion: ya en la formacion deseada -> control cero ---
    Zd3 = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, -1.0]])
    assert np.allclose(control_formacion(Zd3, Zd3, A3), 0.0)

    # --- control_formacion: normalizar=True vs False solo difiere en escala (grafo regular) ---
    Z3 = Zd3 + np.array([[0.5, 0.0], [0.0, 0.0], [0.0, 0.0]])
    u_norm = control_formacion(Z3, Zd3, A3, k=1.0, normalizar=True)
    u_sin = control_formacion(Z3, Zd3, A3, k=1.0, normalizar=False)
    assert np.allclose(u_norm * 2, u_sin)  # grado = n-1 = 2 en grafo completo de 3

    print("Pruebas OK")
    print()
    _simulacion()


def _simulacion():
    """3 robots en grafo completo, convergiendo a un triangulo desde posiciones al azar."""
    print("Simulacion: 3 robots -> formacion triangular, grafo completo")
    print()

    rng = np.random.default_rng(0)
    Z = rng.uniform(-2, 2, size=(3, 2))
    Zd = np.array([[1.0, 0.0], [-0.5, 0.87], [-0.5, -0.87]])  # triangulo equilatero
    A = grafo_completo(3)
    dt = 0.1

    print("{:>5} {:>22} {:>22} {:>22}".format("paso", "robot 0", "robot 1", "robot 2"))
    print("-" * 78)

    for paso in range(1, 61):
        U = control_formacion(Z, Zd, A, k=1.5)
        Z = Z + U * dt

        if paso % 10 == 0 or paso == 1:
            fila = " ".join("({:+6.2f},{:+6.2f})".format(x, y) for x, y in Z)
            print("{:>5} {}".format(paso, fila))

    error = np.linalg.norm((Z - Z.mean(axis=0)) - (Zd - Zd.mean(axis=0)))
    print()
    print("Error de forma final (invariante a traslacion): {:.4f}".format(error))


if __name__ == "__main__":
    _pruebas()
