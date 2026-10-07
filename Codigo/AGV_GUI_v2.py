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
        self.dxf_marks = []   # marks[i] = True si el segmento (i-1 -> i) es trazo real del DXF
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

        console_frame = ctk.CTkFrame(
            info
        )

        console_frame.grid(
            row=2,
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
    def parse_telemetry(
        self,
        line
    ):

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
                "M1", "M2", "IMU", "PUMP", "PEN", "MARK" ) :

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
                "DT_MS", "HERR" ) :

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