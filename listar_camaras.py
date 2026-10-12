#!/usr/bin/env python3
"""
listar_camaras.py - Encuentra el indice de tu camara externa.

El problema: cv2.VideoCapture(N) toma un indice, pero cual numero
corresponde a cual camara no es predecible. Normalmente la integrada de la
laptop es 0 y la USB es 1, pero no siempre. En Linux una sola camara fisica
suele crear DOS nodos /dev/video*, asi que la segunda camara puede quedar en
el indice 2 o 3.

Uso:
    python3 listar_camaras.py            # lista que indices funcionan
    python3 listar_camaras.py --ver      # muestra cada una para identificarla

Una vez que sepas el indice, ponlo en el script de control:
    camara = cam.initialize(1, resolucionx, resoluciony)
"""

import argparse
import sys

try:
    import cv2
except ImportError:
    print("Falta OpenCV. Instala con:")
    print("  python3 -m pip install opencv-python opencv-contrib-python")
    sys.exit(1)


MAX_INDICE = 6


def backend():
    """
    Elige el backend correcto segun el sistema operativo.

    OJO: el repo usa cv2.CAP_DSHOW, que es DirectShow y solo existe en
    Windows. En Mac o Linux ese parametro impide que la camara abra, y el
    error no menciona el backend por ningun lado. Si estas en Mac o Linux y
    'no jala la camara', esta suele ser la causa real, no el indice.
    """
    if sys.platform == "win32":
        return cv2.CAP_DSHOW, "CAP_DSHOW (Windows)"
    if sys.platform == "darwin":
        return cv2.CAP_AVFOUNDATION, "CAP_AVFOUNDATION (macOS)"
    return cv2.CAP_V4L2, "CAP_V4L2 (Linux)"


def abrir(indice, api):
    try:
        captura = cv2.VideoCapture(indice, api)
    except Exception:
        return None
    if not captura.isOpened():
        captura.release()
        return None
    return captura


def buscar(api, nombre_api):
    print("Sistema: {}".format(sys.platform))
    print("Backend: {}".format(nombre_api))
    print()

    encontradas = []
    for indice in range(MAX_INDICE):
        captura = abrir(indice, api)
        if captura is None:
            print("  indice {}: no disponible".format(indice))
            continue

        ok, frame = captura.read()
        if not ok or frame is None:
            print("  indice {}: abre pero no entrega imagen".format(indice))
            captura.release()
            continue

        alto, ancho = frame.shape[:2]

        # Pedir 1280x720 y ver que devuelve. Si no lo soporta, entrega otra
        # cosa: util para distinguir una webcam buena de la integrada.
        captura.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        captura.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        ancho_hd = int(captura.get(cv2.CAP_PROP_FRAME_WIDTH))
        alto_hd = int(captura.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("  indice {}: OK  nativo {}x{}   pidiendo 720p da {}x{}".format(
            indice, ancho, alto, ancho_hd, alto_hd))
        encontradas.append(indice)
        captura.release()

    print()
    if not encontradas:
        print("No se encontro ninguna camara.")
        print("Revisa que este conectada y que ninguna otra app la este usando")
        print("(Zoom, Teams y el navegador la bloquean).")
    else:
        print("Indices disponibles: {}".format(encontradas))
        print("Corre con --ver para identificar cual es cual.")
    return encontradas


def ver(indices, api):
    """Muestra cada camara una por una para identificarla visualmente."""
    for indice in indices:
        captura = abrir(indice, api)
        if captura is None:
            continue
        captura.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        captura.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        print("Mostrando indice {}. ESC o cualquier tecla para la siguiente.".format(indice))
        titulo = "indice {}".format(indice)
        try:
            while True:
                ok, frame = captura.read()
                if not ok:
                    break
                cv2.putText(frame, "INDICE {}".format(indice), (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.putText(frame, "tapa el lente para confirmar cual es",
                            (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 220, 220), 2)
                cv2.imshow(titulo, frame)
                if cv2.waitKey(1) & 0xFF != 255:
                    break
        finally:
            captura.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ver", action="store_true",
                        help="muestra cada camara para identificarla")
    args = parser.parse_args()

    api, nombre = backend()
    encontradas = buscar(api, nombre)

    if args.ver and encontradas:
        ver(encontradas, api)


if __name__ == "__main__":
    main()
