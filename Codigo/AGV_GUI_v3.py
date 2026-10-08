import csv
import math
import queue
import socket
import threading
import time
import tkinter as tk
from pathlib import Path

import customtkinter as ctk
import ezdxf
import matplotlib.pyplot as plt

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import filedialog, messagebox


# ============================================================
# CONFIGURACION DE RED
# ============================================================

IP_AGV = "192.168.0.75"
PUERTO = 8888

SOCKET_TIMEOUT = 0.25

# El primer punto del DXF se convierte en (0,0).
NORMALIZAR_DXF_AL_PRIMER_PUNTO = True

EPS = 1e-6


# ============================================================
# CONTROL DE CALIDAD - ESP32-CAM
#
# Adaptado de "Linea5mm_15cmaltura_5tolerancia".
# Mide el ancho de la linea negra con Black Hat + perfiles
# perpendiculares. Se agregan: correccion por inclinacion,
# deteccion basica de discontinuidad y veredicto por medicion.
# Requiere: pip install opencv-python numpy requests pillow
# ============================================================
try:
    import cv2
    import numpy as np
    import requests
    from PIL import Image

    QC_DISPONIBLE = True
    QC_ERROR_IMPORT = ""
except ImportError as _qc_err:
    QC_DISPONIBLE = False
    QC_ERROR_IMPORT = str(_qc_err)

# ---- Valores por defecto (editables desde la interfaz) -------
QC_CAMERA_IP = "192.168.1.91"
QC_ANCHO_NOMINAL_MM = 5.0
QC_TOLERANCIA_PCT = 5.0
QC_PIXEL_POR_MM = 3.95          # calibrado a 15 cm de altura (provisional)
QC_OFFSET_MM = 0.0              # mm que debe avanzar el AGV desde que sale tinta
                                # hasta que la linea entra en el campo de la camara
QC_GAP_MAX_MM = 3.0             # hueco maximo aceptado en la linea
QC_CONTINUIDAD = True

# ---- Parametros de procesamiento (del script original) -------
QC_BLUR_SIZE = 5
QC_BLACKHAT_KERNEL = 61
QC_CANTIDAD_PERFILES = 40
QC_MARGEN_PERFIL = 0.15
QC_AREA_MIN = 200

# Contraste minimo (umbral de Otsu sobre Black Hat). Sin este filtro,
# Otsu "encuentra" una linea hasta en ruido puro (piso sin tinta).
# Vacio ~0-5, linea negra ~55, linea tenue ~20. Subir si el piso tiene textura.
QC_OTSU_MIN = 12

# Un tramo se considera OK si al menos este % de sus mediciones esta OK
QC_PORC_OK_TRAMO = 90.0

QC_PREVIEW_ANCHO = 340


def qc_obtener_imagen(session, url):
    """Obtiene un JPEG desde la ESP32-CAM (/capture)."""
    respuesta = session.get(url, timeout=3)
    respuesta.raise_for_status()

    datos = np.frombuffer(respuesta.content, dtype=np.uint8)
    imagen = cv2.imdecode(datos, cv2.IMREAD_COLOR)

    if imagen is None:
        raise ValueError("La camara devolvio una imagen JPEG invalida")

    return imagen


def qc_crear_mascara(imagen):
    """Mascara donde la linea negra queda en blanco (Black Hat + Otsu)."""
    gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    gris = cv2.GaussianBlur(gris, (QC_BLUR_SIZE, QC_BLUR_SIZE), 0)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (QC_BLACKHAT_KERNEL, QC_BLACKHAT_KERNEL)
    )
    blackhat = cv2.morphologyEx(gris, cv2.MORPH_BLACKHAT, kernel)

    otsu, mascara = cv2.threshold(
        blackhat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    kernel_pequeno = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel_pequeno)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel_pequeno)

    return gris, blackhat, mascara, float(otsu)


def qc_candidatos(mascara):
    """Contornos alargados que pueden ser la linea (el mayor primero)."""
    contornos, _ = cv2.findContours(
        mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    candidatos = []

    for contorno in contornos:
        area = cv2.contourArea(contorno)

        if area < QC_AREA_MIN:
            continue

        (_, _), (w, h), _ = cv2.minAreaRect(contorno)

        if max(w, h) < min(w, h) * 3:
            continue

        candidatos.append((area, contorno))

    candidatos.sort(key=lambda par: par[0], reverse=True)

    return [contorno for _, contorno in candidatos]


def _qc_ancho_perfil(perfil):
    """Ancho (px) de la zona oscura en un perfil 1D, o None."""
    if perfil.size < 2:
        return None

    minimo = int(np.min(perfil))
    maximo = int(np.max(perfil))

    umbral = minimo + (maximo - minimo) * 0.35

    indices = np.where(perfil < umbral)[0]

    if len(indices) < 2:
        return None

    ancho = int(indices[-1] - indices[0] + 1)

    return ancho if 5 <= ancho <= 100 else None


def qc_medir_linea(gris, contorno, cfg):
    """Ancho de la linea con multiples perfiles perpendiculares."""
    rect = cv2.minAreaRect(contorno)
    x, y, ancho, alto = cv2.boundingRect(contorno)

    alto_img, ancho_img = gris.shape

    vx, vy, _, _ = cv2.fitLine(
        contorno, cv2.DIST_L2, 0, 0.01, 0.01
    ).flatten()

    mediciones = []

    if alto > ancho:
        # Linea vertical en la imagen: perfiles por filas
        factor = float(np.clip(abs(vy), 0.5, 1.0))
        margen = int(alto * QC_MARGEN_PERFIL)
        pos = np.linspace(
            y + margen, y + alto - margen, QC_CANTIDAD_PERFILES
        ).astype(int)

        i0 = max(0, x - 20)
        i1 = min(ancho_img, x + ancho + 20)

        for yy in pos:
            if 0 <= yy < alto_img:
                a = _qc_ancho_perfil(gris[yy, i0:i1])
                if a is not None:
                    mediciones.append(a)
    else:
        # Linea horizontal en la imagen: perfiles por columnas
        factor = float(np.clip(abs(vx), 0.5, 1.0))
        margen = int(ancho * QC_MARGEN_PERFIL)
        pos = np.linspace(
            x + margen, x + ancho - margen, QC_CANTIDAD_PERFILES
        ).astype(int)

        i0 = max(0, y - 20)
        i1 = min(alto_img, y + alto + 20)

        for xx in pos:
            if 0 <= xx < ancho_img:
                a = _qc_ancho_perfil(gris[i0:i1, xx])
                if a is not None:
                    mediciones.append(a)

    if len(mediciones) < 5:
        return None

    cobertura = len(mediciones) / float(len(pos))

    mediciones = np.array(mediciones, dtype=np.float32)

    q1 = np.percentile(mediciones, 25)
    q3 = np.percentile(mediciones, 75)
    iqr = q3 - q1

    filtradas = mediciones[
        (mediciones >= q1 - 1.5 * iqr) & (mediciones <= q3 + 1.5 * iqr)
    ]

    if len(filtradas) < 5:
        return None

    # La correccion por inclinacion convierte el ancho medido en la fila
    # (o columna) en ancho perpendicular a la linea.
    ancho_px = float(np.median(filtradas)) * factor
    ancho_mm = ancho_px / cfg["px_por_mm"]

    nominal = cfg["ancho_nominal_mm"]
    tol = cfg["tol_pct"] / 100.0

    return {
        "rect": rect,
        "ancho_px": ancho_px,
        "ancho_mm": ancho_mm,
        "desvio_mm": ancho_mm - nominal,
        "error": abs(ancho_mm - nominal) / nominal * 100.0,
        "en_tolerancia": nominal * (1 - tol) <= ancho_mm <= nominal * (1 + tol),
        "cobertura": cobertura,
    }


def qc_continuidad(candidatos, ancho_px, cfg):
    """
    Busca huecos a lo largo de la direccion de la linea principal.
    Considera solo candidatos colineales con la principal.
    """
    principal = candidatos[0]

    vx, vy, x0, y0 = [
        float(v)
        for v in cv2.fitLine(
            principal, cv2.DIST_L2, 0, 0.01, 0.01
        ).flatten()
    ]

    intervalos = []

    for contorno in candidatos:
        p = contorno.reshape(-1, 2).astype(np.float32)

        dx = p[:, 0] - x0
        dy = p[:, 1] - y0

        t = dx * vx + dy * vy        # a lo largo de la linea
        s = -dx * vy + dy * vx       # lateral

        if abs(float(np.median(s))) > max(2.0 * ancho_px, 10.0):
            continue

        intervalos.append((float(t.min()), float(t.max())))

    intervalos.sort()

    gap_px = 0.0
    fin = intervalos[0][1]

    for ini, fn in intervalos[1:]:
        if ini > fin:
            gap_px = max(gap_px, ini - fin)

        fin = max(fin, fn)

    gap_mm = gap_px / cfg["px_por_mm"]

    return {
        "n_segmentos": len(intervalos),
        "gap_mm": gap_mm,
        "discontinua": gap_mm > cfg["gap_max_mm"],
    }


def qc_analizar(imagen, cfg):
    """
    Analiza un frame. Devuelve un dict con el veredicto y una imagen
    anotada (PIL, RGB, reducida) para mostrar en la interfaz.

    estado: OK | FUERA DE TOLERANCIA | DISCONTINUA | SIN MEDICION | SIN LINEA
    """
    gris, _, mascara, otsu = qc_crear_mascara(imagen)

    # Sin contraste suficiente no hay linea (ej. se acabo la tinta)
    candidatos = qc_candidatos(mascara) if otsu >= QC_OTSU_MIN else []

    frame = imagen.copy()

    res = {
        "estado": "SIN LINEA",
        "ok": False,
        "ancho_px": None,
        "ancho_mm": None,
        "error": None,
        "desvio_mm": None,
        "gap_mm": None,
        "cobertura": None,
    }

    if candidatos:
        principal = candidatos[0]
        cv2.drawContours(frame, [principal], -1, (0, 255, 0), 2)

        med = qc_medir_linea(gris, principal, cfg)

        if med is None:
            res["estado"] = "SIN MEDICION"
        else:
            cont = None

            if cfg["continuidad"]:
                cont = qc_continuidad(candidatos, med["ancho_px"], cfg)

            res.update(
                ancho_px=med["ancho_px"],
                ancho_mm=med["ancho_mm"],
                error=med["error"],
                desvio_mm=med["desvio_mm"],
                cobertura=med["cobertura"],
            )

            if cont is not None:
                res["gap_mm"] = cont["gap_mm"]

            if cont is not None and cont["discontinua"]:
                res["estado"] = "DISCONTINUA"
            elif med["en_tolerancia"]:
                res["estado"] = "OK"
                res["ok"] = True
            else:
                res["estado"] = "FUERA DE TOLERANCIA"

            cv2.polylines(
                frame, [np.int32(cv2.boxPoints(med["rect"]))], True, (255, 0, 0), 2
            )

    color = (0, 200, 0) if res["ok"] else (0, 0, 255)

    cv2.putText(frame, res["estado"], (15, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    if res["ancho_mm"] is not None:
        cv2.putText(frame, f"{res['ancho_mm']:.2f} mm ({res['error']:.1f} %)",
                    (15, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    escala = QC_PREVIEW_ANCHO / float(frame.shape[1])
    pequena = cv2.resize(
        frame, (QC_PREVIEW_ANCHO, max(1, int(frame.shape[0] * escala)))
    )

    res["preview"] = Image.fromarray(cv2.cvtColor(pequena, cv2.COLOR_BGR2RGB))

    return res



class AGVApp(ctk.CTk):

    def __init__(self):
        super().__init__()

        self.title("AGV WiFi + OTA - Solo encoders")
        self.geometry("1300x750")
        self.minsize(1100, 650)

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Socket TCP
        self.sock = None
        self.reader_running = False
        self.rx_queue = queue.Queue()

        # DXF
        self.dxf_points = []
        self.dxf_marks = []

        # ---- Control de calidad (ESP32-CAM) ----
        self.last_tel_time = 0.0
        self.qc_queue = queue.Queue()
        self.qc_thread = None
        self.qc_stop_event = threading.Event()
        self.qc_cfg = {
            "ip": QC_CAMERA_IP,
            "ancho_nominal_mm": QC_ANCHO_NOMINAL_MM,
            "tol_pct": QC_TOLERANCIA_PCT,
            "px_por_mm": QC_PIXEL_POR_MM,
            "offset_mm": QC_OFFSET_MM,
            "gap_max_mm": QC_GAP_MAX_MM,
            "continuidad": QC_CONTINUIDAD,
        }
        self.qc_results = []       # todas las mediciones automaticas
        self.qc_points = []        # (x, y, estado) para el grafico
        self.qc_seg_results = []   # mediciones del tramo en curso
        self.qc_seg_count = 0
        self.qc_last = None
        self.qc_state = "DESACTIVADO"
        self.qc_plot_dirty = False
        self.qc_last_plot = 0.0
        self.qc_img_ref = None   # marks[i] = True si el segmento (i-1 -> i) es trazo real del DXF
        self.current_index = -1

        # ====================================================
        # NUEVO:
        # Orden de puntos seleccionado con el mouse
        # ====================================================
        self.selected_route = []

        # Recorrido
        self.route_thread = None
        self.route_cancel = threading.Event()
        self.point_reached = threading.Event()

        # Telemetria
        self.telemetry = {
            "ENC1": 0,
            "ENC2": 0,

            "DL": 0.0,
            "DR": 0.0,

            "X": 0.0,
            "Y": 0.0,
            "YAW": 0.0,
            # NUEVO - yaw medido por el MPU6050
            "YAW_IMU": 0.0,

            # ====================================================
            # NUEVO - DIAGNOSTICO MPU6050
            # ====================================================
            "GYRO_Z_RAW": 0.0,
            "BIAS_Z": 0.0,
            "GYRO_Z": 0.0,
            "DT_MS": 0.0,

            # NUEVO - rumbo / bomba
            "IMU": 0,
            "HERR": 0.0,
            "PUMP": 0,
            "PEN": 0,
            "MARK": 0,
            "ST": 0,
            "PRE": 0,
            "TRAMO": 0.0,

            "E1": 1,
            "E2": 1,

            "M1": 1,
            "M2": 1,
        }

        # Panel derecho con scroll
        self.info_canvas = None

        self.build_ui()

        self.after(
            60,
            self.process_rx_queue
        )

        self.after(150, self.qc_poll)

    # ========================================================
    # INTERFAZ
    # ========================================================
    def build_ui(self):

        self.grid_columnconfigure(
            1,
            weight=1
        )

        self.grid_rowconfigure(
            0,
            weight=1
        )

        # ----------------------------------------------------
        # PANEL IZQUIERDO
        # ----------------------------------------------------

        left = ctk.CTkScrollableFrame(
            self,
            width=330,
            label_text="CONTROLES"
        )

        left.grid(
            row=0,
            column=0,
            padx=10,
            pady=10,
            sticky="nsew"
        )

        ctk.CTkLabel(
            left,
            text="CONTROL AGV",
            font=ctk.CTkFont(
                size=24,
                weight="bold"
            )
        ).pack(
            pady=(14, 8)
        )

        ctk.CTkLabel(
            left,
            text="WiFi + TCP / Solo encoders"
        ).pack(
            pady=(0, 8)
        )

        # ----------------------------------------------------
        # RED
        # ----------------------------------------------------

        net = ctk.CTkFrame(left)

        net.pack(
            fill="x",
            padx=12,
            pady=5
        )

        ctk.CTkLabel(
            net,
            text="IP del AGV"
        ).pack(
            anchor="w",
            padx=8,
            pady=(8, 2)
        )

        self.entry_ip = ctk.CTkEntry(net)

        self.entry_ip.pack(
            fill="x",
            padx=8,
            pady=2
        )

        self.entry_ip.insert(
            0,
            IP_AGV
        )

        ctk.CTkLabel(
            net,
            text="Puerto TCP"
        ).pack(
            anchor="w",
            padx=8,
            pady=(6, 2)
        )

        self.entry_port = ctk.CTkEntry(net)

        self.entry_port.pack(
            fill="x",
            padx=8,
            pady=2
        )

        self.entry_port.insert(
            0,
            str(PUERTO)
        )

        ctk.CTkButton(
            net,
            text="CONECTAR AGV",
            command=self.connect_agv
        ).pack(
            fill="x",
            padx=8,
            pady=(8, 3)
        )

        ctk.CTkButton(
            net,
            text="DESCONECTAR",
            command=self.disconnect_agv
        ).pack(
            fill="x",
            padx=8,
            pady=(3, 8)
        )

        self.lbl_connection = ctk.CTkLabel(
            net,
            text="Estado: DESCONECTADO"
        )

        self.lbl_connection.pack(
            pady=(0, 8)
        )

        # ----------------------------------------------------
        # DXF
        # ----------------------------------------------------

        ctk.CTkButton(
            left,
            text="CARGAR DXF",
            command=self.load_dxf
        ).pack(
            fill="x",
            padx=12,
            pady=(10, 4)
        )

        ctk.CTkButton(
            left,
            text="INICIAR RECORRIDO DXF",
            command=self.start_dxf_route
        ).pack(
            fill="x",
            padx=12,
            pady=4
        )

        self.lbl_points = ctk.CTkLabel(
            left,
            text="Puntos DXF: 0"
        )

        self.lbl_points.pack(
            pady=4
        )

        # ----------------------------------------------------
        # PRUEBA GIRO 90°
        # ----------------------------------------------------

        ctk.CTkButton(
            left,
            text="GIRO +90°",
            command=lambda:
                self.send_command(
                    "TURN:90"
                )
        ).pack(
            fill="x",
            padx=12,
            pady=4
        )

        ctk.CTkButton(
            left,
            text="GIRO -90°",
            command=lambda:
                self.send_command(
                    "TURN:-90"
                )
        ).pack(
            fill="x",
            padx=12,
            pady=4
        )

        # ====================================================
        # NUEVO:
        # BORRAR ORDEN SELECCIONADO
        # ====================================================

        ctk.CTkButton(
            left,
            text="BORRAR SELECCIÓN DE PUNTOS",
            command=self.clear_selected_route
        ).pack(
            fill="x",
            padx=12,
            pady=4
        )

        # ----------------------------------------------------
        # GOTO MANUAL
        # ----------------------------------------------------

        ctk.CTkLabel(
            left,
            text="Movimiento manual",
            font=ctk.CTkFont(
                weight="bold"
            )
        ).pack(
            pady=(12, 4)
        )

        coord_frame = ctk.CTkFrame(
            left,
            fg_color="transparent"
        )

        coord_frame.pack(
            fill="x",
            padx=10
        )

        coord_frame.grid_columnconfigure(
            (0, 1),
            weight=1
        )

        self.entry_x = ctk.CTkEntry(
            coord_frame,
            placeholder_text="X [mm]"
        )

        self.entry_x.grid(
            row=0,
            column=0,
            padx=3,
            sticky="ew"
        )

        self.entry_y = ctk.CTkEntry(
            coord_frame,
            placeholder_text="Y [mm]"
        )

        self.entry_y.grid(
            row=0,
            column=1,
            padx=3,
            sticky="ew"
        )

        ctk.CTkButton(
            left,
            text="IR A (X,Y)",
            command=self.manual_goto
        ).pack(
            fill="x",
            padx=12,
            pady=6
        )

        # ----------------------------------------------------
        # DIAGNOSTICO
        # ----------------------------------------------------

        ctk.CTkLabel(
            left,
            text="Diagnóstico",
            font=ctk.CTkFont(
                weight="bold"
            )
        ).pack(
            pady=(10, 4)
        )

        self.sw_enc1 = ctk.CTkSwitch(
            left,
            text="Encoder 1",
            command=lambda:
                self.toggle(
                    "ENC1",
                    self.sw_enc1.get()
                )
        )

        self.sw_enc1.pack(
            anchor="w",
            padx=20,
            pady=2
        )

        self.sw_enc1.select()

        self.sw_enc2 = ctk.CTkSwitch(
            left,
            text="Encoder 2",
            command=lambda:
                self.toggle(
                    "ENC2",
                    self.sw_enc2.get()
                )
        )

        self.sw_enc2.pack(
            anchor="w",
            padx=20,
            pady=2
        )

        self.sw_enc2.select()

        self.sw_motor1 = ctk.CTkSwitch(
            left,
            text="Motor 1",
            command=lambda:
                self.toggle(
                    "MOTOR1",
                    self.sw_motor1.get()
                )
        )

        self.sw_motor1.pack(
            anchor="w",
            padx=20,
            pady=2
        )

        self.sw_motor1.select()

        self.sw_motor2 = ctk.CTkSwitch(
            left,
            text="Motor 2",
            command=lambda:
                self.toggle(
                    "MOTOR2",
                    self.sw_motor2.get()
                )
        )

        self.sw_motor2.pack(
            anchor="w",
            padx=20,
            pady=2
        )

        self.sw_motor2.select()

        # ----------------------------------------------------
        # NUEVO - MARCADO (BOMBA) Y HERRAMIENTAS
        # ----------------------------------------------------
        ctk.CTkLabel(
            left,
            text="MARCADO (BOMBA)",
            font=ctk.CTkFont(weight="bold")
        ).pack(pady=(12, 2))

        self.sw_pump_en = ctk.CTkSwitch(
            left,
            text="Marcar con bomba (OFF = en seco)",
            command=self.toggle_pump_enable
        )
        self.sw_pump_en.pack(anchor="w", padx=20, pady=2)

        pump_btns = ctk.CTkFrame(left, fg_color="transparent")
        pump_btns.pack(fill="x", padx=10, pady=2)
        pump_btns.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkButton(
            pump_btns,
            text="CEBAR (ON)",
            command=lambda: self.send_command("PUMP_ON")
        ).grid(row=0, column=0, padx=3, sticky="ew")

        ctk.CTkButton(
            pump_btns,
            text="BOMBA OFF",
            command=lambda: self.send_command("PUMP_OFF")
        ).grid(row=0, column=1, padx=3, sticky="ew")

        pwm_row = ctk.CTkFrame(left, fg_color="transparent")
        pwm_row.pack(fill="x", padx=10, pady=2)
        pwm_row.grid_columnconfigure(0, weight=1)

        self.entry_pump_pwm = ctk.CTkEntry(
            pwm_row,
            placeholder_text="PWM bomba 0-255"
        )
        self.entry_pump_pwm.insert(0, "200")
        self.entry_pump_pwm.grid(row=0, column=0, padx=3, sticky="ew")

        ctk.CTkButton(
            pwm_row,
            text="APLICAR PWM",
            width=110,
            command=self.apply_pump_pwm
        ).grid(row=0, column=1, padx=3)

        ctk.CTkButton(
            left,
            text="RECALIBRAR GIRO (AGV quieto)",
            command=lambda: self.send_command("CALIB")
        ).pack(fill="x", padx=12, pady=(8, 2))

        cmd_row = ctk.CTkFrame(left, fg_color="transparent")
        cmd_row.pack(fill="x", padx=10, pady=2)
        cmd_row.grid_columnconfigure(0, weight=1)

        self.entry_cmd = ctk.CTkEntry(
            cmd_row,
            placeholder_text="Comando: GET, SET:KP_HDG=3, MOVE:1000,1"
        )
        self.entry_cmd.grid(row=0, column=0, padx=3, sticky="ew")
        self.entry_cmd.bind("<Return>", lambda e: self.send_manual_command())

        ctk.CTkButton(
            cmd_row,
            text="ENVIAR",
            width=70,
            command=self.send_manual_command
        ).grid(row=0, column=1, padx=3)

        # ----------------------------------------------------
        # NUEVO - CONTROL DE CALIDAD (ESP32-CAM)
        # ----------------------------------------------------
        ctk.CTkLabel(
            left,
            text="CONTROL DE CALIDAD (ESP32-CAM)",
            font=ctk.CTkFont(weight="bold")
        ).pack(pady=(14, 2))

        self.sw_qc = ctk.CTkSwitch(
            left,
            text="QC activo (mide solo si la bomba tira tinta)",
            command=self.qc_toggle
        )
        self.sw_qc.pack(anchor="w", padx=20, pady=2)

        if not QC_DISPONIBLE:
            self.sw_qc.configure(state="disabled")
            ctk.CTkLabel(
                left,
                text="Falta: pip install opencv-python numpy requests pillow",
                text_color="#e67e22",
                wraplength=260
            ).pack(padx=12, pady=2)

        def qc_row(texto, valor):
            fila = ctk.CTkFrame(left, fg_color="transparent")
            fila.pack(fill="x", padx=10, pady=1)
            fila.grid_columnconfigure(1, weight=1)

            ctk.CTkLabel(fila, text=texto, anchor="w", width=130).grid(
                row=0, column=0, padx=3
            )

            entrada = ctk.CTkEntry(fila)
            entrada.insert(0, str(valor))
            entrada.grid(row=0, column=1, padx=3, sticky="ew")

            return entrada

        self.entry_qc_ip = qc_row("IP cámara", QC_CAMERA_IP)
        self.entry_qc_ancho = qc_row("Ancho nominal [mm]", QC_ANCHO_NOMINAL_MM)
        self.entry_qc_tol = qc_row("Tolerancia [%]", QC_TOLERANCIA_PCT)
        self.entry_qc_pxmm = qc_row("Calibración [px/mm]", QC_PIXEL_POR_MM)
        self.entry_qc_offset = qc_row("Offset boquilla→cám [mm]", QC_OFFSET_MM)
        self.entry_qc_gap = qc_row("Hueco máx. [mm]", QC_GAP_MAX_MM)

        qc_btns = ctk.CTkFrame(left, fg_color="transparent")
        qc_btns.pack(fill="x", padx=10, pady=(4, 2))
        qc_btns.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkButton(
            qc_btns,
            text="APLICAR PARÁMETROS",
            command=self.qc_apply_config
        ).grid(row=0, column=0, padx=3, sticky="ew")

        ctk.CTkButton(
            qc_btns,
            text="MEDIR AHORA (prueba)",
            command=self.qc_manual
        ).grid(row=0, column=1, padx=3, sticky="ew")

        ctk.CTkButton(
            left,
            text="RESET ORIGEN (0,0,0°)",
            command=self.reset_origin
        ).pack(
            fill="x",
            padx=12,
            pady=(10, 4)
        )

        # ----------------------------------------------------
        # STOP
        # ----------------------------------------------------

        ctk.CTkButton(
            left,
            text="STOP / PARADA DE EMERGENCIA",
            height=60,
            fg_color="#b00020",
            hover_color="#7c0016",
            font=ctk.CTkFont(
                size=16,
                weight="bold"
            ),
            command=self.emergency_stop
        ).pack(
            fill="x",
            padx=12,
            pady=12
        )

        # ----------------------------------------------------
        # PANEL DERECHO
        # ----------------------------------------------------

        right = ctk.CTkFrame(self)

        right.grid(
            row=0,
            column=1,
            padx=(0, 10),
            pady=10,
            sticky="nsew"
        )

        right.grid_columnconfigure(
            0,
            weight=3,
            minsize=520
        )

        right.grid_columnconfigure(
            1,
            weight=2,
            minsize=380
        )

        right.grid_rowconfigure(
            0,
            weight=1
        )

        # ----------------------------------------------------
        # GRAFICO
        # ----------------------------------------------------

        plot_frame = ctk.CTkFrame(right)

        plot_frame.grid(
            row=0,
            column=0,
            padx=8,
            pady=8,
            sticky="nsew"
        )

        self.fig, self.ax = plt.subplots(
            figsize=(8, 7)
        )

        self.fig.patch.set_facecolor(
            "#202020"
        )

        self.ax.set_facecolor(
            "#202020"
        )

        self.ax.tick_params(
            colors="white"
        )

        self.ax.set_title(
            "Plano DXF",
            color="white"
        )

        self.ax.set_xlabel(
            "X [mm]",
            color="white"
        )

        self.ax.set_ylabel(
            "Y [mm]",
            color="white"
        )

        self.ax.grid(
            True,
            alpha=0.25
        )

        self.ax.set_aspect(
            "equal",
            adjustable="datalim"
        )

        self.canvas = FigureCanvasTkAgg(
            self.fig,
            master=plot_frame
        )

        self.canvas.draw()

        self.canvas.get_tk_widget().pack(
            fill="both",
            expand=True
        )

        # ====================================================
        # NUEVO:
        # CLICK SOBRE LOS PUNTOS DEL DXF
        # ====================================================

        self.canvas.mpl_connect(
            "button_press_event",
            self.on_plot_click
        )

        # ====================================================
        # PANEL INFORMACION CON SCROLL
        # ====================================================

        info_container = ctk.CTkFrame(
            right
        )

        info_container.grid(
            row=0,
            column=1,
            padx=8,
            pady=8,
            sticky="nsew"
        )

        info_container.grid_rowconfigure(
            0,
            weight=1
        )

        info_container.grid_columnconfigure(
            0,
            weight=1
        )

        self.info_canvas = tk.Canvas(
            info_container,
            bg="#2b2b2b",
            highlightthickness=0
        )

        self.info_canvas.grid(
            row=0,
            column=0,
            sticky="nsew"
        )

        # Scroll vertical
        info_scroll_y = ctk.CTkScrollbar(
            info_container,
            orientation="vertical",
            command=self.info_canvas.yview
        )

        info_scroll_y.grid(
            row=0,
            column=1,
            sticky="ns"
        )

        # Scroll horizontal
        info_scroll_x = ctk.CTkScrollbar(
            info_container,
            orientation="horizontal",
            command=self.info_canvas.xview
        )

        info_scroll_x.grid(
            row=1,
            column=0,
            sticky="ew"
        )

        self.info_canvas.configure(
            yscrollcommand=
                info_scroll_y.set,
            xscrollcommand=
                info_scroll_x.set
        )

        info = ctk.CTkFrame(
            self.info_canvas,
            width=440
        )

        self.info_window = (
            self.info_canvas.create_window(
                (0, 0),
                window=info,
                anchor="nw"
            )
        )

        info.grid_columnconfigure(
            0,
            weight=1
        )

        info.bind(
            "<Configure>",
            self.update_info_scrollregion
        )

        self.bind_all(
            "<MouseWheel>",
            self.mousewheel_info,
            add="+"
        )

        self.bind_all(
            "<Shift-MouseWheel>",
            self.shift_mousewheel_info,
            add="+"
        )

        # ----------------------------------------------------
        # ESTADO
        # ----------------------------------------------------

        status = ctk.CTkFrame(info)

        status.grid(
            row=0,
            column=0,
            padx=8,
            pady=8,
            sticky="ew"
        )

        self.lbl_state = ctk.CTkLabel(
            status,
            text="Estado AGV: IDLE"
        )

        self.lbl_state.pack(
            anchor="w",
            padx=8,
            pady=3
        )

        self.lbl_pos = ctk.CTkLabel(
            status,
            text="Posición: X=0.0 Y=0.0 mm"
        )

        self.lbl_pos.pack(
            anchor="w",
            padx=8,
            pady=3
        )

        self.lbl_yaw = ctk.CTkLabel(
            status,
            text="Yaw relativo: 0.0°"
        )

        self.lbl_yaw.pack(
            anchor="w",
            padx=8,
            pady=3
        )

        # ====================================================
        # NUEVO - YAW MPU6050
        # ====================================================
        self.lbl_yaw_imu = ctk.CTkLabel(
            status,
            text="Yaw MPU6050: 0.0°"
        )

        self.lbl_yaw_imu.pack(
            anchor="w",
            padx=8,
            pady=3
        )

        # ====================================================
        # NUEVO - DIAGNOSTICO MPU6050
        # ====================================================
        self.lbl_gyro_z_raw = ctk.CTkLabel(
            status,
            text="GYRO_Z_RAW: 0.00000 °/s"
        )
        self.lbl_gyro_z_raw.pack(anchor="w", padx=8, pady=2)

        self.lbl_bias_z = ctk.CTkLabel(
            status,
            text="BIAS_Z: 0.00000 °/s"
        )
        self.lbl_bias_z.pack(anchor="w", padx=8, pady=2)

        self.lbl_gyro_z = ctk.CTkLabel(
            status,
            text="GYRO_Z: 0.00000 °/s"
        )
        self.lbl_gyro_z.pack(anchor="w", padx=8, pady=2)

        self.lbl_dt_ms = ctk.CTkLabel(
            status,
            text="DT: 0.000 ms"
        )
        self.lbl_dt_ms.pack(anchor="w", padx=8, pady=2)

        # NUEVO - estado IMU / rumbo / bomba
        self.lbl_imu_ok = ctk.CTkLabel(status, text="MPU6050: -")
        self.lbl_imu_ok.pack(anchor="w", padx=8, pady=2)

        self.lbl_herr = ctk.CTkLabel(status, text="Error de rumbo: 0.00°")
        self.lbl_herr.pack(anchor="w", padx=8, pady=2)

        self.lbl_pump = ctk.CTkLabel(status, text="Bomba: OFF | habilitada: NO")
        self.lbl_pump.pack(anchor="w", padx=8, pady=2)

        self.lbl_route = ctk.CTkLabel(
            status,
            text="Recorrido: -"
        )

        self.lbl_route.pack(
            anchor="w",
            padx=8,
            pady=3
        )

        # ----------------------------------------------------
        # TELEMETRIA
        # ----------------------------------------------------

        tele = ctk.CTkFrame(info)

        tele.grid(
            row=1,
            column=0,
            padx=8,
            pady=8,
            sticky="ew"
        )

        ctk.CTkLabel(
            tele,
            text="TELEMETRÍA",
            font=ctk.CTkFont(
                weight="bold"
            )
        ).pack(
            pady=4
        )

        self.lbl_enc1 = ctk.CTkLabel(
            tele,
            text="Encoder 1: 0"
        )

        self.lbl_enc1.pack(
            anchor="w",
            padx=8
        )

        self.lbl_enc2 = ctk.CTkLabel(
            tele,
            text="Encoder 2: 0"
        )

        self.lbl_enc2.pack(
            anchor="w",
            padx=8
        )

        self.lbl_dl = ctk.CTkLabel(
            tele,
            text="Rueda izquierda: 0.00 mm"
        )

        self.lbl_dl.pack(
            anchor="w",
            padx=8
        )

        self.lbl_dr = ctk.CTkLabel(
            tele,
            text="Rueda derecha: 0.00 mm"
        )

        self.lbl_dr.pack(
            anchor="w",
            padx=8
        )

        self.lbl_mean = ctk.CTkLabel(
            tele,
            text="Distancia media: 0.00 mm"
        )

        self.lbl_mean.pack(
            anchor="w",
            padx=8
        )

        self.lbl_diff = ctk.CTkLabel(
            tele,
            text="Diferencia: 0.00 mm"
        )

        self.lbl_diff.pack(
            anchor="w",
            padx=8
        )

        # ----------------------------------------------------
        # CONSOLA
        # ----------------------------------------------------

        # ----------------------------------------------------
        # NUEVO - RESULTADOS DEL CONTROL DE CALIDAD
        # ----------------------------------------------------
        qc_frame = ctk.CTkFrame(info)
        qc_frame.grid(row=2, column=0, padx=8, pady=8, sticky="ew")

        ctk.CTkLabel(
            qc_frame,
            text="CONTROL DE CALIDAD",
            font=ctk.CTkFont(weight="bold")
        ).pack(pady=4)

        self.lbl_qc_state = ctk.CTkLabel(qc_frame, text="QC: DESACTIVADO")
        self.lbl_qc_state.pack(anchor="w", padx=8)

        self.lbl_qc_verdict = ctk.CTkLabel(
            qc_frame,
            text="—",
            text_color="gray70",
            font=ctk.CTkFont(size=22, weight="bold")
        )
        self.lbl_qc_verdict.pack(pady=4)

        self.lbl_qc_ancho = ctk.CTkLabel(
            qc_frame, text="Ancho: -- mm", wraplength=380, justify="left"
        )
        self.lbl_qc_ancho.pack(anchor="w", padx=8)

        self.lbl_qc_cont = ctk.CTkLabel(
            qc_frame, text="Continuidad: --", wraplength=380, justify="left"
        )
        self.lbl_qc_cont.pack(anchor="w", padx=8)

        self.lbl_qc_stats = ctk.CTkLabel(
            qc_frame, text="Mediciones: 0", wraplength=380, justify="left"
        )
        self.lbl_qc_stats.pack(anchor="w", padx=8, pady=(0, 4))

        self.qc_preview = ctk.CTkLabel(
            qc_frame, text="Sin imagen", width=340, height=200
        )
        self.qc_preview.pack(pady=4)

        ctk.CTkLabel(qc_frame, text="Resumen por tramo").pack(anchor="w", padx=8)

        self.qc_hist = ctk.CTkTextbox(qc_frame, width=400, height=130)
        self.qc_hist.pack(padx=6, pady=4)

        qc_res_btns = ctk.CTkFrame(qc_frame, fg_color="transparent")
        qc_res_btns.pack(fill="x", padx=6, pady=(0, 6))
        qc_res_btns.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkButton(
            qc_res_btns, text="LIMPIAR RESULTADOS", command=self.qc_clear
        ).grid(row=0, column=0, padx=3, sticky="ew")

        ctk.CTkButton(
            qc_res_btns, text="EXPORTAR CSV", command=self.qc_export_csv
        ).grid(row=0, column=1, padx=3, sticky="ew")

        console_frame = ctk.CTkFrame(
            info
        )

        console_frame.grid(
            row=3,
            column=0,
            padx=8,
            pady=8,
            sticky="ew"
        )

        console_frame.grid_columnconfigure(
            0,
            weight=1
        )

        ctk.CTkLabel(
            console_frame,
            text="CONSOLA"
        ).grid(
            row=0,
            column=0,
            pady=4
        )

        self.console = ctk.CTkTextbox(
            console_frame,
            width=400,
            height=350
        )

        self.console.grid(
            row=1,
            column=0,
            padx=6,
            pady=6,
            sticky="ew"
        )

        self.protocol(
            "WM_DELETE_WINDOW",
            self.on_close
        )

    # ========================================================
    # SCROLL PANEL DERECHO
    # ========================================================
    def update_info_scrollregion(
        self,
        event=None
    ):

        if self.info_canvas is None:
            return

        self.info_canvas.configure(
            scrollregion=
                self.info_canvas.bbox(
                    "all"
                )
        )

    def cursor_sobre_info(self):

        if self.info_canvas is None:
            return False

        try:
            x = self.winfo_pointerx()
            y = self.winfo_pointery()

            widget = self.winfo_containing(
                x,
                y
            )

            while widget is not None:

                if widget == self.info_canvas:
                    return True

                try:
                    widget = widget.master
                except Exception:
                    break

        except Exception:
            pass

        return False

    def mousewheel_info(
        self,
        event
    ):

        if not self.cursor_sobre_info():
            return

        if event.delta == 0:
            return

        self.info_canvas.yview_scroll(
            int(-event.delta / 120),
            "units"
        )

        return "break"

    def shift_mousewheel_info(
        self,
        event
    ):

        if not self.cursor_sobre_info():
            return

        if event.delta == 0:
            return

        self.info_canvas.xview_scroll(
            int(-event.delta / 120),
            "units"
        )

        return "break"

    # ========================================================
    # WIFI / TCP
    # ========================================================
    def connect_agv(self):

        if self.sock:
            self.log(
                "Ya existe una conexión."
            )
            return

        ip = self.entry_ip.get().strip()

        try:
            port = int(
                self.entry_port.get().strip()
            )

        except ValueError:

            messagebox.showerror(
                "Puerto",
                "Puerto inválido."
            )

            return

        self.log(
            f"Conectando a {ip}:{port}..."
        )

        try:

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            sock.settimeout(
                3.0
            )

            sock.connect(
                (ip, port)
            )

            sock.settimeout(
                SOCKET_TIMEOUT
            )

            self.sock = sock

            self.reader_running = True

            threading.Thread(
                target=self.socket_reader,
                daemon=True
            ).start()

            self.lbl_connection.configure(
                text=
                    f"Estado: CONECTADO "
                    f"({ip}:{port})"
            )

            self.log(
                "AGV conectado."
            )

            self.send_command(
                "PING"
            )

        except Exception as exc:

            self.sock = None

            messagebox.showerror(
                "WiFi",
                str(exc)
            )

            self.log(
                f"ERROR conexión: {exc}"
            )

    def disconnect_agv(self):

        self.route_cancel.set()
        self.reader_running = False

        try:

            if self.sock:

                try:

                    self.send_command(
                        "STOP"
                    )

                    time.sleep(
                        0.05
                    )

                except Exception:
                    pass

                self.sock.close()

        except Exception:
            pass

        self.sock = None

        self.lbl_connection.configure(
            text=
                "Estado: DESCONECTADO"
        )

        self.log(
            "AGV desconectado."
        )

    def socket_reader(self):

        buffer = ""

        while (
            self.reader_running and
            self.sock
        ):

            try:

                data = self.sock.recv(
                    4096
                )

                if not data:

                    self.rx_queue.put(
                        "ERROR:CONNECTION_CLOSED"
                    )

                    break

                buffer += data.decode(
                    "utf-8",
                    errors="replace"
                )

                while "\n" in buffer:

                    line, buffer = (
                        buffer.split(
                            "\n",
                            1
                        )
                    )

                    line = line.strip()

                    if line:

                        self.rx_queue.put(
                            line
                        )

            except socket.timeout:
                continue

            except Exception as exc:

                self.rx_queue.put(
                    f"ERROR_LOCAL:{exc}"
                )

                break

        self.reader_running = False

    def send_command(
        self,
        command
    ):

        if not self.sock:

            self.log(
                f"No conectado: "
                f"{command}"
            )

            return False

        try:

            self.sock.sendall(
                (
                    command.strip()
                    + "\n"
                ).encode(
                    "utf-8"
                )
            )

            self.log(
                f">> {command}"
            )

            return True

        except Exception as exc:

            self.log(
                f"ERROR enviando: "
                f"{exc}"
            )

            return False

    def process_rx_queue(self):

        try:

            while True:

                line = (
                    self.rx_queue
                    .get_nowait()
                )

                self.handle_line(
                    line
                )

        except queue.Empty:
            pass

        self.after(
            60,
            self.process_rx_queue
        )

    def handle_line(
        self,
        line
    ):

        if line.startswith(
            "TEL,"
        ):

            self.parse_telemetry(
                line
            )

            return

        self.log(
            f"<< {line}"
        )

        if line.startswith(
            "MOVING"
        ):

            self.lbl_state.configure(
                text=
                    "Estado AGV: MOVING"
            )

        elif line.startswith(
            "TURNING"
        ):

            self.lbl_state.configure(
                text=
                    "Estado AGV: TURNING"
            )

        elif line == "POINT_REACHED":

            self.lbl_state.configure(
                text=
                    "Estado AGV: "
                    "POINT_REACHED"
            )

            self.point_reached.set()

        elif line == "STOPPED":

            self.lbl_state.configure(
                text=
                    "Estado AGV: STOPPED"
            )

            self.route_cancel.set()

            self.point_reached.set()

        elif line.startswith(
            "ERROR"
        ):

            self.lbl_state.configure(
                text=
                    f"Estado AGV: "
                    f"{line}"
            )

            self.route_cancel.set()

            self.point_reached.set()

    # ========================================================
    # TELEMETRIA
    # ========================================================
    def parse_telemetry(self, line):
        self.last_tel_time = time.time()


        try:

            values = {}

            for item in (
                line.split(",")[1:]
            ):

                if "=" in item:

                    key, value = (
                        item.split(
                            "=",
                            1
                        )
                    )

                    values[
                        key.strip()
                    ] = value.strip()

            for key in (
                "ENC1",
                "ENC2",
                "E1",
                "E2",
                "M1", "M2", "IMU", "PUMP", "PEN", "MARK", "ST", "PRE" ) :

                if key in values:

                    self.telemetry[
                        key
                    ] = int(
                        float(
                            values[key]
                        )
                    )

            for key in (
                "DL",
                "DR",
                "X",
                "Y",
                "YAW",
                # NUEVO - yaw integrado del MPU6050
                "YAW_IMU",
                # ====================================================
                # NUEVO - DIAGNOSTICO MPU6050
                # ====================================================
                "GYRO_Z_RAW",
                "BIAS_Z",
                "GYRO_Z",
                "DT_MS", "HERR", "TRAMO" ) :

                if key in values:

                    self.telemetry[
                        key
                    ] = float(
                        values[key]
                    )

            self.update_telemetry()

        except Exception as exc:

            self.log(
                f"Error telemetría: "
                f"{exc}"
            )

    def update_telemetry(self):

        t = self.telemetry

        self.lbl_enc1.configure(
            text=
                f"Encoder 1: "
                f"{t['ENC1']} "
                f"({'ON' if t['E1'] else 'OFF'})"
        )

        self.lbl_enc2.configure(
            text=
                f"Encoder 2: "
                f"{t['ENC2']} "
                f"({'ON' if t['E2'] else 'OFF'})"
        )

        self.lbl_dl.configure(
            text=
                f"Rueda izquierda: "
                f"{t['DL']:.2f} mm"
        )

        self.lbl_dr.configure(
            text=
                f"Rueda derecha: "
                f"{t['DR']:.2f} mm"
        )

        mean = (
            abs(t["DL"]) +
            abs(t["DR"])
        ) / 2.0

        diff = (
            abs(t["DL"]) -
            abs(t["DR"])
        )

        self.lbl_mean.configure(
            text=
                f"Distancia media: "
                f"{mean:.2f} mm"
        )

        self.lbl_diff.configure(
            text=
                f"Diferencia: "
                f"{diff:.2f} mm"
        )

        self.lbl_pos.configure(
            text=
                f"Posición: "
                f"X={t['X']:.1f} "
                f"Y={t['Y']:.1f} mm"
        )

        self.lbl_yaw.configure(
            text=
                f"Yaw relativo: "
                f"{t['YAW']:.1f}°"
        )

        # NUEVO - mostrar yaw del MPU6050 sin cambiar el yaw existente
        self.lbl_yaw_imu.configure(
            text=
                f"Yaw MPU6050: "
                f"{t['YAW_IMU']:.1f}°"
        )

        # ====================================================
        # NUEVO - DIAGNOSTICO MPU6050
        # ====================================================
        self.lbl_gyro_z_raw.configure(
            text=f"GYRO_Z_RAW: {t['GYRO_Z_RAW']:.5f} °/s"
        )
        self.lbl_bias_z.configure(
            text=f"BIAS_Z: {t['BIAS_Z']:.5f} °/s"
        )
        self.lbl_gyro_z.configure(
            text=f"GYRO_Z: {t['GYRO_Z']:.5f} °/s"
        )
        self.lbl_dt_ms.configure(
            text=f"DT: {t['DT_MS']:.3f} ms"
        )

        self.lbl_imu_ok.configure(
            text="MPU6050: " + ("OK" if t["IMU"] else "NO DETECTADO")
        )
        self.lbl_herr.configure(
            text=f"Error de rumbo: {t['HERR']:.2f}°"
        )
        estado_bomba = (
            f"ON (PWM {t['PUMP']})" if t["PUMP"] > 0 else "OFF"
        )
        self.lbl_pump.configure(
            text=(
                f"Bomba: {estado_bomba} | "
                f"habilitada: {'SÍ' if t['PEN'] else 'NO'} | "
                f"tramo: {'MARCANDO' if t['MARK'] else '-'}"
            )
        )

    # ========================================================
    # DXF
    # ========================================================
    @staticmethod
    def close_points(
        a,
        b
    ):

        return (
            abs(
                a[0] - b[0]
            ) <= EPS
            and
            abs(
                a[1] - b[1]
            ) <= EPS
        )

    @classmethod
    def extract_dxf_path(cls, filename):
        """
        Devuelve (puntos, marcas).
        marcas[i] = True si el segmento (i-1 -> i) es un trazo real del DXF
        (hay que marcar). False si es un traslado entre trazos no conectados
        (el AGV debe viajar con la bomba apagada). marcas[0] = False.
        """
        doc = ezdxf.readfile(filename)
        msp = doc.modelspace()

        chains = []

        for entity in msp:
            tipo = entity.dxftype()

            if tipo == "LINE":
                a = (float(entity.dxf.start.x), float(entity.dxf.start.y))
                b = (float(entity.dxf.end.x), float(entity.dxf.end.y))
                chains.append([a, b])

            elif tipo == "LWPOLYLINE":
                points = [
                    (float(p[0]), float(p[1]))
                    for p in entity.get_points("xy")
                ]

                if len(points) >= 2:
                    if (
                        entity.closed
                        and not cls.close_points(points[0], points[-1])
                    ):
                        points.append(points[0])

                    chains.append(points)

        if not chains:
            raise ValueError("El DXF no contiene LINE o LWPOLYLINE.")

        first = chains.pop(0)
        ordered = list(first)
        marks = [False] + [True] * (len(first) - 1)

        while chains:
            end = ordered[-1]
            found = False

            for i, chain in enumerate(chains):
                if cls.close_points(end, chain[0]):
                    ordered.extend(chain[1:])
                    marks.extend([True] * (len(chain) - 1))
                    chains.pop(i)
                    found = True
                    break

                if cls.close_points(end, chain[-1]):
                    chain = list(reversed(chain))
                    ordered.extend(chain[1:])
                    marks.extend([True] * (len(chain) - 1))
                    chains.pop(i)
                    found = True
                    break

            if not found:
                chain = chains.pop(0)

                if not cls.close_points(ordered[-1], chain[0]):
                    # Traslado sin marcar hasta el inicio del nuevo trazo
                    ordered.append(chain[0])
                    marks.append(False)

                ordered.extend(chain[1:])
                marks.extend([True] * (len(chain) - 1))

        clean = []
        clean_marks = []

        for point, mark in zip(ordered, marks):
            if not clean or not cls.close_points(clean[-1], point):
                clean.append(point)
                clean_marks.append(mark)

        if NORMALIZAR_DXF_AL_PRIMER_PUNTO:
            x0, y0 = clean[0]
            clean = [(x - x0, y - y0) for x, y in clean]

        clean_marks[0] = False

        return clean, clean_marks

    def load_dxf(self):

        filename = (
            filedialog
            .askopenfilename(
                title=
                    "Seleccionar DXF",
                filetypes=[
                    (
                        "DXF",
                        "*.dxf"
                    )
                ]
            )
        )

        if not filename:
            return

        try:

            self.dxf_points, self.dxf_marks = (
                self.extract_dxf_path(
                    filename
                )
            )

            if (
                len(
                    self.dxf_points
                )
                <
                2
            ):

                raise ValueError(
                    "La trayectoria necesita "
                    "al menos dos puntos."
                )

            self.current_index = -1

            # Al cargar un DXF nuevo
            # se borra la selección anterior.
            self.selected_route.clear()

            self.lbl_points.configure(
                text=
                    f"Puntos DXF: "
                    f"{len(self.dxf_points)}"
            )

            self.log(
                f"DXF cargado: "
                f"{Path(filename).name}"
            )

            self.log(
                f"Puntos encontrados: "
                f"{len(self.dxf_points)}"
            )

            for i, (x, y) in enumerate(
                self.dxf_points
            ):

                self.log(
                    f"P{i}: "
                    f"X={x:.2f} "
                    f"Y={y:.2f}"
                )

            self.draw_dxf()

        except Exception as exc:

            messagebox.showerror(
                "DXF",
                str(exc)
            )

            self.log(
                f"ERROR DXF: "
                f"{exc}"
            )

    def draw_dxf(self):

        self.ax.clear()

        self.ax.set_facecolor(
            "#202020"
        )

        self.ax.set_title(
            "Plano DXF / Trayectoria",
            color="white"
        )

        self.ax.set_xlabel(
            "X [mm]",
            color="white"
        )

        self.ax.set_ylabel(
            "Y [mm]",
            color="white"
        )

        self.ax.tick_params(
            colors="white"
        )

        self.ax.grid(
            True,
            alpha=0.25
        )

        self.ax.set_aspect(
            "equal",
            adjustable="datalim"
        )

        if not self.dxf_points:

            self.canvas.draw_idle()

            return

        xs = [
            point[0]

            for point
            in self.dxf_points
        ]

        ys = [
            point[1]

            for point
            in self.dxf_points
        ]

        self.ax.plot(
            xs,
            ys,
            marker="o",
            linewidth=1.5
        )

        for i, (x, y) in enumerate(
            self.dxf_points
        ):

            self.ax.annotate(
                str(i),
                (x, y),
                xytext=(5, 5),
                textcoords=
                    "offset points",
                color="white"
            )

        # ====================================================
        # NUEVO:
        # MOSTRAR ORDEN ELEGIDO
        # ====================================================

        for order, point_index in enumerate(
            self.selected_route,
            start=1
        ):

            x, y = (
                self.dxf_points[
                    point_index
                ]
            )

            self.ax.scatter(
                [x],
                [y],
                s=180,
                marker="o"
            )

            self.ax.annotate(
                f"ORD {order}",
                (x, y),
                xytext=(8, -18),
                textcoords=
                    "offset points",
                color="white",
                fontweight="bold"
            )

        if (
            0 <=
            self.current_index
            <
            len(
                self.dxf_points
            )
        ):

            x, y = (
                self.dxf_points[
                    self.current_index
                ]
            )

            self.ax.scatter(
                [x],
                [y],
                s=120,
                marker="o"
            )

        next_index = (
            self.current_index
            +
            1
        )

        if (
            0 <= next_index
            <
            len(
                self.dxf_points
            )
        ):

            x, y = (
                self.dxf_points[
                    next_index
                ]
            )

            self.ax.scatter(
                [x],
                [y],
                s=140,
                marker="x"
            )

        self.qc_draw_overlay()

        self.canvas.draw_idle()

    # ========================================================
    # NUEVO:
    # SELECCION DE PUNTOS DESDE EL GRAFICO
    # ========================================================
    def on_plot_click(
        self,
        event
    ):

        if not self.dxf_points:
            return

        if (
            event.xdata is None
            or
            event.ydata is None
        ):
            return

        click_x = event.xdata
        click_y = event.ydata

        nearest_index = None

        nearest_distance = (
            float("inf")
        )

        for i, (x, y) in enumerate(
            self.dxf_points
        ):

            distance = math.hypot(
                click_x - x,
                click_y - y
            )

            if (
                distance
                <
                nearest_distance
            ):

                nearest_distance = (
                    distance
                )

                nearest_index = i

        if nearest_index is None:
            return

        xs = [
            p[0]
            for p
            in self.dxf_points
        ]

        ys = [
            p[1]
            for p
            in self.dxf_points
        ]

        width = (
            max(xs)
            -
            min(xs)
        )

        height = (
            max(ys)
            -
            min(ys)
        )

        size = max(
            width,
            height,
            1.0
        )

        tolerance = (
            size * 0.08
        )

        if (
            nearest_distance
            >
            tolerance
        ):
            return

        if (
            nearest_index
            in
            self.selected_route
        ):

            self.log(
                f"P{nearest_index} "
                f"ya está seleccionado."
            )

            return

        self.selected_route.append(
            nearest_index
        )

        order = len(
            self.selected_route
        )

        x, y = (
            self.dxf_points[
                nearest_index
            ]
        )

        self.log(
            f"Selección {order}: "
            f"P{nearest_index} "
            f"X={x:.2f} "
            f"Y={y:.2f}"
        )

        self.draw_dxf()

    def clear_selected_route(self):

        self.selected_route.clear()

        self.log(
            "Selección de puntos borrada."
        )

        self.draw_dxf()

    # ========================================================
    # RECORRIDO DXF
    # ========================================================
    def start_dxf_route(self):

        if not self.dxf_points:

            messagebox.showwarning(
                "DXF",
                "Primero cargue un DXF."
            )

            return

        if not self.sock:

            messagebox.showwarning(
                "AGV",
                "Primero conecte el AGV."
            )

            return

        if (
            self.route_thread
            and
            self.route_thread
            .is_alive()
        ):
            return

        self.route_cancel.clear()

        self.point_reached.clear()

        self.route_thread = (
            threading.Thread(
                target=
                    self.route_worker,
                daemon=True
            )
        )

        self.route_thread.start()

    def route_worker(self):

        self.log(
            "=== INICIO DXF ==="
        )

        self.send_command(
            "RESET"
        )

        time.sleep(
            0.25
        )

        # ====================================================
        # NUEVO:
        # SI HAY PUNTOS ELEGIDOS,
        # UTILIZA ESE ORDEN.
        #
        # SI NO HAY PUNTOS ELEGIDOS,
        # FUNCIONA COMO ANTES.
        # ====================================================

        if self.selected_route:

            route_indices = list(
                self.selected_route
            )

            self.log(
                "Recorrido seleccionado: "
                +
                " -> ".join(
                    f"P{i}"
                    for i
                    in route_indices
                )
            )

        else:

            start_index = 0

            if (
                self.dxf_points
                and
                math.hypot(
                    self.dxf_points[0][0],
                    self.dxf_points[0][1]
                )
                <
                1e-6
            ):

                start_index = 1

            route_indices = list(
                range(
                    start_index,
                    len(
                        self.dxf_points
                    )
                )
            )

        # El AGV parte del primer punto (origen) tras el RESET
        prev_index = 0 if NORMALIZAR_DXF_AL_PRIMER_PUNTO else None

        for i in route_indices:

            if (
                self.route_cancel
                .is_set()
            ):
                return

            x, y = (
                self.dxf_points[i]
            )

            self.current_index = (
                i - 1
            )

            self.after(
                0,
                self.draw_dxf
            )

            self.after(
                0,
                lambda
                i=i,
                x=x,
                y=y:
                    self.lbl_route.configure(
                        text=
                            f"Punto {i}: "
                            f"X={x:.1f} "
                            f"Y={y:.1f}"
                    )
            )

            self.log(
                f"Objetivo: "
                f"X={x:.2f} "
                f"Y={y:.2f}"
            )

            self.point_reached.clear()

            if not self.send_command(
                f"GOTO:"
                f"{x:.3f},"
                f"{y:.3f},"
                f"{1 if self.segment_is_marked(prev_index, i) else 0}"
            ):
                return

            while not (
                self.route_cancel
                .is_set()
            ):

                if (
                    self.point_reached
                    .wait(
                        0.1
                    )
                ):
                    break

            if (
                self.route_cancel
                .is_set()
            ):
                return

            self.current_index = i

            self.after(
                0,
                self.draw_dxf
            )

            self.log(
                f"Punto P{i} "
                f"alcanzado."
            )

            prev_index = i

            time.sleep(
                0.10
            )

        self.log(
            "=== DXF FINALIZADO ==="
        )

        # Reset de orientación / referencia
        # después de terminar el gráfico.
        self.send_command(
            "RESET"
        )

        self.after(
            0,
            lambda:
                self.lbl_route.configure(
                    text=
                        "Recorrido: "
                        "FINALIZADO"
                )
        )

    # ========================================================
    # MOVIMIENTO MANUAL
    # ========================================================
    def manual_goto(self):

        try:

            x = float(
                self.entry_x
                .get()
                .replace(
                    ",",
                    "."
                )
            )

            y = float(
                self.entry_y
                .get()
                .replace(
                    ",",
                    "."
                )
            )

        except ValueError:

            messagebox.showerror(
                "Coordenadas",
                "Ingrese X e Y numéricos."
            )

            return

        if not self.sock:

            messagebox.showwarning(
                "AGV",
                "Primero conecte el AGV."
            )

            return

        self.route_cancel.set()

        self.point_reached.clear()

        self.log(
            f"GOTO manual: "
            f"X={x:.2f} "
            f"Y={y:.2f}"
        )

        self.send_command(
            f"GOTO:"
            f"{x:.3f},"
            f"{y:.3f}"
        )

    # ========================================================
    # DIAGNOSTICO
    # ========================================================
        # ========================================================
    # CONTROL DE CALIDAD (ESP32-CAM)
    #
    # El QC solo mide cuando la bomba esta tirando tinta:
    #   PUMP > 0       bomba encendida
    #   MARK == 1      el tramo en curso marca
    #   PRE == 0       no esta en pre-flujo (todavia no hay linea)
    #   ST == 1        el AGV esta avanzando en recta
    #   TRAMO >= off   la linea ya llego al campo de la camara
    # ========================================================
    def qc_read_config(self):
        """Lee los campos de la interfaz. Devuelve True si son validos."""
        def num(entry):
            return float(entry.get().strip().replace(",", "."))

        try:
            cfg = {
                "ip": self.entry_qc_ip.get().strip(),
                "ancho_nominal_mm": num(self.entry_qc_ancho),
                "tol_pct": num(self.entry_qc_tol),
                "px_por_mm": num(self.entry_qc_pxmm),
                "offset_mm": num(self.entry_qc_offset),
                "gap_max_mm": num(self.entry_qc_gap),
                "continuidad": QC_CONTINUIDAD,
            }
        except ValueError:
            messagebox.showerror("Control de calidad", "Revise los parámetros del QC: deben ser numéricos.")
            return False

        if not cfg["ip"] or cfg["ancho_nominal_mm"] <= 0 or cfg["px_por_mm"] <= 0 or cfg["tol_pct"] < 0:
            messagebox.showerror("Control de calidad", "IP, ancho nominal y px/mm deben ser válidos (> 0).")
            return False

        self.qc_cfg = cfg
        return True

    def qc_apply_config(self):
        if self.qc_read_config():
            self.log(
                f"QC: parámetros aplicados | nominal {self.qc_cfg['ancho_nominal_mm']:.2f} mm "
                f"±{self.qc_cfg['tol_pct']:.1f}% | {self.qc_cfg['px_por_mm']:.3f} px/mm | "
                f"offset {self.qc_cfg['offset_mm']:.0f} mm"
            )
            self.qc_update_labels()

    def qc_toggle(self):
        if self.sw_qc.get():
            if not QC_DISPONIBLE:
                self.sw_qc.deselect()
                return

            if not self.qc_read_config():
                self.sw_qc.deselect()
                return

            self.qc_start()
        else:
            self.qc_stop()

    def qc_start(self):
        if self.qc_thread and self.qc_thread.is_alive():
            return

        self.qc_stop_event = threading.Event()
        self.qc_thread = threading.Thread(
            target=self.qc_worker,
            args=(self.qc_stop_event,),
            daemon=True
        )
        self.qc_thread.start()

        self.qc_state = "ESPERANDO BOMBA"
        self.log("QC activado: solo mide mientras la bomba tira tinta.")
        self.qc_update_labels()

    def qc_stop(self):
        self.qc_stop_event.set()

        # Si habia un tramo medido a medias, cerrarlo
        self.qc_close_segment()

        self.qc_state = "DESACTIVADO"

        try:
            self.qc_update_labels()
        except Exception:
            pass

    def qc_worker(self, stop_event):
        """Hilo del QC: captura y analiza solo mientras la bomba tira tinta."""
        session = requests.Session()
        activo_prev = False
        errores = 0
        estado_prev = None

        def estado(txt):
            nonlocal estado_prev
            if txt != estado_prev:
                estado_prev = txt
                self.qc_queue.put(("state", txt))

        while not stop_event.is_set():
            cfg = dict(self.qc_cfg)
            url = f"http://{cfg['ip']}/capture"

            t = dict(self.telemetry)

            telemetria_fresca = (
                self.sock is not None
                and (time.time() - self.last_tel_time) < 1.0
            )

            activo = (
                telemetria_fresca
                and t["PUMP"] > 0
                and t["MARK"] == 1
                and t["PRE"] == 0
                and t["ST"] == 1
                and t["TRAMO"] >= cfg["offset_mm"]
            )

            if not activo:
                if activo_prev:
                    self.qc_queue.put(("seg_end",))

                activo_prev = False

                if errores == 0:
                    estado("ESPERANDO BOMBA (sin tinta)")

                stop_event.wait(0.05)
                continue

            if not activo_prev:
                self.qc_queue.put(("seg_start",))

            activo_prev = True

            try:
                pose = {"X": t["X"], "Y": t["Y"], "YAW": t["YAW"]}

                imagen = qc_obtener_imagen(session, url)
                res = qc_analizar(imagen, cfg)

                self.qc_queue.put(("result", res, pose))

                errores = 0
                estado("MIDIENDO")

            except Exception as e:
                errores += 1
                estado("CÁMARA SIN CONEXIÓN")

                if errores in (1, 20):
                    self.qc_queue.put(("error", str(e)))

                stop_event.wait(0.5)

        session.close()

    def qc_manual(self):
        """Medicion de prueba/calibracion. NO entra en las estadisticas."""
        if not QC_DISPONIBLE:
            messagebox.showerror("Control de calidad", "Falta instalar: pip install opencv-python numpy requests pillow")
            return

        if not self.qc_read_config():
            return

        cfg = dict(self.qc_cfg)

        def tarea():
            try:
                session = requests.Session()
                imagen = qc_obtener_imagen(session, f"http://{cfg['ip']}/capture")
                res = qc_analizar(imagen, cfg)
                self.qc_queue.put(("manual", res))
            except Exception as e:
                self.qc_queue.put(("error", str(e)))

        threading.Thread(target=tarea, daemon=True).start()

    def qc_poll(self):
        """Procesa los eventos del hilo QC en el hilo de la interfaz."""
        ultimo_preview = None
        cambio = False

        try:
            while True:
                ev = self.qc_queue.get_nowait()
                kind = ev[0]

                if kind == "state":
                    self.qc_state = ev[1]
                    cambio = True

                elif kind == "seg_start":
                    self.qc_close_segment()
                    self.qc_seg_count += 1
                    self.qc_seg_results = []

                elif kind == "seg_end":
                    self.qc_close_segment()

                elif kind == "result":
                    self.qc_handle_result(ev[1], ev[2])
                    ultimo_preview = ev[1]["preview"]
                    cambio = True

                elif kind == "manual":
                    res = ev[1]
                    ultimo_preview = res["preview"]
                    self.qc_last = res
                    cambio = True
                    ancho = f"{res['ancho_mm']:.2f} mm" if res["ancho_mm"] is not None else "-"
                    self.log(f"QC prueba manual: {res['estado']} | ancho {ancho}")

                elif kind == "error":
                    self.log(f"QC: error de cámara: {ev[1]}")

        except queue.Empty:
            pass

        if ultimo_preview is not None:
            ci = ctk.CTkImage(
                light_image=ultimo_preview,
                dark_image=ultimo_preview,
                size=ultimo_preview.size
            )
            self.qc_preview.configure(image=ci, text="")
            self.qc_img_ref = ci

        if cambio:
            self.qc_update_labels()

        # Refresco del grafico como maximo 1 vez por segundo
        if self.qc_plot_dirty and (time.time() - self.qc_last_plot) > 1.0:
            self.qc_plot_dirty = False
            self.qc_last_plot = time.time()

            if self.dxf_points:
                self.draw_dxf()

        self.after(100, self.qc_poll)

    def qc_handle_result(self, res, pose):
        """Registra una medicion del QC automatico."""
        off = self.qc_cfg["offset_mm"]
        yaw = math.radians(pose["YAW"])

        # Posicion de la linea observada: la camara mira "off" mm atras de la boquilla
        x = pose["X"] - off * math.cos(yaw)
        y = pose["Y"] - off * math.sin(yaw)

        rec = {
            "tramo": self.qc_seg_count,
            "t": time.strftime("%H:%M:%S"),
            "x_mm": x,
            "y_mm": y,
            "ancho_mm": res["ancho_mm"],
            "ancho_px": res["ancho_px"],
            "error_pct": res["error"],
            "gap_mm": res["gap_mm"],
            "estado": res["estado"],
            "ok": res["ok"],
        }

        self.qc_results.append(rec)
        self.qc_seg_results.append(rec)
        self.qc_points.append((x, y, res["estado"]))
        self.qc_plot_dirty = True
        self.qc_last = res

        if not res["ok"]:
            ancho = f"{res['ancho_mm']:.2f} mm" if res["ancho_mm"] is not None else "-"
            self.log(
                f"QC ⚠ {res['estado']} | tramo {self.qc_seg_count} | "
                f"({x:.0f}, {y:.0f}) mm | ancho {ancho}"
            )

    def qc_close_segment(self):
        """Resumen de un tramo marcado, al apagarse la bomba."""
        seg = self.qc_seg_results

        if not seg:
            return

        self.qc_seg_results = []

        n = len(seg)
        n_ok = sum(1 for r in seg if r["ok"])
        pct = 100.0 * n_ok / n
        anchos = [r["ancho_mm"] for r in seg if r["ancho_mm"] is not None]
        n_disc = sum(1 for r in seg if r["estado"] == "DISCONTINUA")
        n_sl = sum(1 for r in seg if r["estado"] in ("SIN LINEA", "SIN MEDICION"))

        veredicto = "OK" if pct >= QC_PORC_OK_TRAMO else "NOK"

        if anchos:
            ancho_txt = (
                f"{sum(anchos) / len(anchos):.2f} mm "
                f"(mín {min(anchos):.2f} / máx {max(anchos):.2f})"
            )
        else:
            ancho_txt = "sin medición"

        x0, y0 = seg[0]["x_mm"], seg[0]["y_mm"]
        x1, y1 = seg[-1]["x_mm"], seg[-1]["y_mm"]

        linea = (
            f"Tramo {seg[0]['tramo']}: {veredicto} | {n} med. | OK {pct:.0f}% | "
            f"ancho {ancho_txt} | discont. {n_disc} | sin línea {n_sl} | "
            f"({x0:.0f},{y0:.0f})→({x1:.0f},{y1:.0f})"
        )

        self.qc_hist.insert("end", linea + "\n")
        self.qc_hist.see("end")

        self.log("QC " + linea)

    def qc_update_labels(self):
        if not hasattr(self, "lbl_qc_state"):
            return

        cfg = self.qc_cfg

        self.lbl_qc_state.configure(text=f"QC: {self.qc_state}")

        nominal = cfg["ancho_nominal_mm"]
        tol = cfg["tol_pct"]

        res = self.qc_last

        if res is None:
            self.lbl_qc_verdict.configure(text="—", text_color="gray70")
            self.lbl_qc_ancho.configure(
                text=f"Ancho: -- mm  (nominal {nominal:.2f} ±{tol:.1f}%)"
            )
            self.lbl_qc_cont.configure(text="Continuidad: --")
        else:
            colores = {
                "OK": "#2ecc71",
                "FUERA DE TOLERANCIA": "#e74c3c",
                "DISCONTINUA": "#f39c12",
                "SIN MEDICION": "#e74c3c",
                "SIN LINEA": "#e74c3c",
            }

            self.lbl_qc_verdict.configure(
                text=res["estado"],
                text_color=colores.get(res["estado"], "gray70")
            )

            if res["ancho_mm"] is not None:
                self.lbl_qc_ancho.configure(
                    text=(
                        f"Ancho: {res['ancho_mm']:.2f} mm ({res['ancho_px']:.1f} px) | "
                        f"desvío {res['desvio_mm']:+.2f} mm ({res['error']:.1f}%) | "
                        f"nominal {nominal:.2f} ±{tol:.1f}%"
                    )
                )
            else:
                self.lbl_qc_ancho.configure(
                    text=f"Ancho: -- mm  (nominal {nominal:.2f} ±{tol:.1f}%)"
                )

            if res["gap_mm"] is not None:
                self.lbl_qc_cont.configure(
                    text=(
                        f"Continuidad: hueco máx {res['gap_mm']:.1f} mm "
                        f"(límite {cfg['gap_max_mm']:.1f} mm) | "
                        f"perfiles válidos {100 * res['cobertura']:.0f}%"
                    )
                )
            else:
                self.lbl_qc_cont.configure(text="Continuidad: --")

        total = len(self.qc_results)

        if total:
            n_ok = sum(1 for r in self.qc_results if r["ok"])
            anchos = [r["ancho_mm"] for r in self.qc_results if r["ancho_mm"] is not None]

            if anchos:
                media = sum(anchos) / len(anchos)
                rango = f" | media {media:.2f} | mín {min(anchos):.2f} | máx {max(anchos):.2f} mm"
            else:
                rango = ""

            self.lbl_qc_stats.configure(
                text=(
                    f"Mediciones: {total} | OK {n_ok} ({100 * n_ok / total:.1f}%) | "
                    f"NOK {total - n_ok}{rango}"
                )
            )
        else:
            self.lbl_qc_stats.configure(text="Mediciones: 0")

    def qc_clear(self):
        self.qc_results = []
        self.qc_points = []
        self.qc_seg_results = []
        self.qc_seg_count = 0
        self.qc_last = None
        self.qc_hist.delete("1.0", "end")

        self.qc_update_labels()

        if self.dxf_points:
            self.draw_dxf()

        self.log("QC: resultados borrados.")

    def qc_export_csv(self):
        if not self.qc_results:
            messagebox.showinfo("Control de calidad", "No hay mediciones para exportar.")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=time.strftime("QC_%Y%m%d_%H%M%S.csv")
        )

        if not filename:
            return

        campos = [
            "tramo", "t", "x_mm", "y_mm", "ancho_mm", "ancho_px",
            "error_pct", "gap_mm", "estado", "ok"
        ]

        with open(filename, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=campos)
            w.writeheader()
            w.writerows(self.qc_results)

        self.log(f"QC: {len(self.qc_results)} mediciones exportadas a {filename}")

    def qc_draw_overlay(self):
        """Puntos de medicion sobre el plano: verde OK, rojo NOK, naranja discontinua."""
        if not self.qc_points:
            return

        grupos = {
            "#2ecc71": [],
            "#e74c3c": [],
            "#f39c12": [],
        }

        for x, y, estado in self.qc_points:
            if estado == "OK":
                grupos["#2ecc71"].append((x, y))
            elif estado == "DISCONTINUA":
                grupos["#f39c12"].append((x, y))
            else:
                grupos["#e74c3c"].append((x, y))

        for color, pts in grupos.items():
            if pts:
                self.ax.scatter(
                    [p[0] for p in pts],
                    [p[1] for p in pts],
                    s=18,
                    c=color,
                    marker="s",
                    zorder=6
                )

    def segment_is_marked(self, a, b):
        """True si ir del punto a al punto b recorre un trazo real del DXF."""
        if a is None or abs(a - b) != 1:
            return False

        hi = max(a, b)

        return hi < len(self.dxf_marks) and bool(self.dxf_marks[hi])

    def toggle_pump_enable(self):
        on = bool(self.sw_pump_en.get())
        self.send_command(f"PUMP_EN:{1 if on else 0}")
        self.log(
            "Marcado con bomba: HABILITADO" if on
            else "Marcado con bomba: DESHABILITADO (recorrido en seco)"
        )

    def apply_pump_pwm(self):
        try:
            v = int(float(self.entry_pump_pwm.get().replace(",", ".")))
        except ValueError:
            messagebox.showerror("Bomba", "Ingrese un PWM numérico (0-255).")
            return

        v = max(0, min(255, v))
        self.send_command(f"PUMP_PWM:{v}")

    def send_manual_command(self):
        cmd = self.entry_cmd.get().strip()

        if cmd:
            self.send_command(cmd)

    def reset_origin(self):

        self.route_cancel.set()

        self.current_index = -1

        self.send_command(
            "RESET"
        )

        self.log(
            "Origen = "
            "X0, Y0, yaw0."
        )

        self.draw_dxf()

    def toggle(
        self,
        device,
        state
    ):

        command = (
            f"{device}_"
            f"{'ON' if state else 'OFF'}"
        )

        self.send_command(
            command
        )

    # ========================================================
    # STOP
    # ========================================================
    def emergency_stop(self):

        self.route_cancel.set()

        self.point_reached.set()

        self.send_command(
            "STOP"
        )

        self.lbl_state.configure(
            text=
                "Estado AGV: STOP"
        )

        self.log(
            "!!! STOP / "
            "PARADA DE EMERGENCIA !!!"
        )

    # ========================================================
    # LOG
    # ========================================================
    def log(
        self,
        message
    ):

        timestamp = time.strftime(
            "%H:%M:%S"
        )

        def append():

            self.console.insert(
                "end",
                f"[{timestamp}] "
                f"{message}\n"
            )

            self.console.see(
                "end"
            )

        if (
            threading.current_thread()
            is
            threading.main_thread()
        ):

            append()

        else:

            self.after(
                0,
                append
            )

    # ========================================================
    # CIERRE
    # ========================================================
    def on_close(self):
        self.qc_stop()


        self.route_cancel.set()

        self.reader_running = False

        try:

            if self.sock:

                self.send_command(
                    "STOP"
                )

                time.sleep(
                    0.05
                )

                self.sock.close()

        except Exception:
            pass

        self.sock = None

        self.destroy()


if __name__ == "__main__":

    app = AGVApp()

    app.mainloop()