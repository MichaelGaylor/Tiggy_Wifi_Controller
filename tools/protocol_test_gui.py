#!/usr/bin/env python3
"""
WiFi CNC Controller - GUI Protocol Test Tool

Graphical interface for testing the ESP32 firmware without Mach3.
Uses tkinter (built into Python, no pip install needed).

Usage:
    python protocol_test_gui.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import socket
import struct
import time
import threading
import queue
from datetime import datetime

# Import protocol functions from the CLI tool
from protocol_test import (
    WCNC_MAGIC, WCNC_VERSION,
    UDP_MOTION_PORT, UDP_STATUS_PORT, TCP_CONTROL_PORT,
    PKT_MOTION_SEGMENT, PKT_JOG_COMMAND, PKT_JOG_STOP,
    PKT_ESTOP, PKT_FEED_HOLD, PKT_FEED_RESUME,
    PKT_RESET, PKT_HOME_COMMAND, PKT_IO_CONTROL,
    PKT_STATUS_REPORT, PKT_HANDSHAKE_REQ, PKT_HANDSHAKE_RESP,
    PKT_PING, PKT_PONG,
    PKT_CONFIG_GET, PKT_CONFIG_SET, PKT_CONFIG_RESP, PKT_CONFIG_SAVE, PKT_ACK,
    PKT_HOME_COMPLETE, PKT_ALARM, PKT_PROBE_RESULT,
    CFG_STEPS_PER_MM_X, CFG_STEPS_PER_MM_Y, CFG_STEPS_PER_MM_Z,
    CFG_STEPS_PER_MM_A, CFG_STEPS_PER_MM_B, CFG_STEPS_PER_MM_C,
    CFG_MAX_RATE_X, CFG_MAX_RATE_Y, CFG_MAX_RATE_Z,
    CFG_MAX_RATE_A, CFG_MAX_RATE_B, CFG_MAX_RATE_C,
    CFG_ACCEL_X, CFG_ACCEL_Y, CFG_ACCEL_Z,
    CFG_ACCEL_A, CFG_ACCEL_B, CFG_ACCEL_C,
    CFG_STEP_PULSE_US, CFG_DIR_SETUP_US, CFG_STEP_IDLE_DELAY_MS, CFG_STATUS_INTERVAL_MS,
    CFG_WIFI_SSID, CFG_WIFI_PASSWORD,
    CFG_INVERT_STEP, CFG_INVERT_DIR, CFG_INVERT_ENABLE,
    CFG_INVERT_LIMIT, CFG_INVERT_HOME, CFG_INVERT_ESTOP, CFG_INVERT_PROBE,
    CFG_HOMING_DIR_MASK, CFG_HOMING_SEEK_RATE, CFG_HOMING_FEED_RATE, CFG_HOMING_PULLOFF,
    CFG_CHARGE_PUMP_FREQ, CFG_SPINDLE_PWM_FREQ, CFG_SPINDLE_MAX_RPM,
    CFG_PIN_STEP_X, CFG_PIN_STEP_Y, CFG_PIN_STEP_Z,
    CFG_PIN_STEP_A, CFG_PIN_STEP_B, CFG_PIN_STEP_C,
    CFG_PIN_DIR_X, CFG_PIN_DIR_Y, CFG_PIN_DIR_Z,
    CFG_PIN_DIR_A, CFG_PIN_DIR_B, CFG_PIN_DIR_C,
    CFG_PIN_ENABLE,
    CFG_PIN_LIMIT_X, CFG_PIN_LIMIT_Y, CFG_PIN_LIMIT_Z,
    CFG_PIN_LIMIT_A, CFG_PIN_LIMIT_B, CFG_PIN_LIMIT_C,
    CFG_PIN_PROBE, CFG_PIN_ESTOP, CFG_PIN_SPINDLE,
    CFG_PIN_LED, CFG_PIN_CHARGE_PUMP,
    CFG_PIN_MISC_OUT0, CFG_PIN_MISC_OUT1,
    CFG_PIN_ENCODER_A, CFG_PIN_ENCODER_B, CFG_PIN_ENCODER_INDEX,
    CFG_ENCODER_PPR, CFG_ENCODER_MODE, CFG_ENCODER_FILTER_NS,
    CFG_PIN_MISC_IN0, CFG_PIN_MISC_IN1, CFG_PIN_MISC_IN2, CFG_PIN_MISC_IN3,
    CFG_DEVICE_MODE,
    CFG_IO_PIN_COUNT, CFG_IO_DIR_MASK, CFG_IO_PULLUP_MASK, CFG_IO_INVERT_MASK,
    CFG_IO_PIN_BASE,
    CFG_PIN_ETH_MOSI, CFG_PIN_ETH_MISO, CFG_PIN_ETH_SCLK,
    CFG_PIN_ETH_INT, CFG_PIN_ETH_SPI_HOST,
    VAL_UINT8, VAL_UINT16, VAL_UINT32, VAL_FLOAT,
    STATES, MAX_AXES, CONFIG_VALUE_LEN,
    crc16_ccitt, build_header, finalize_packet, validate_packet,
    tcp_send, tcp_recv,
    send_config_get, format_config_value,
    send_config_set, send_config_save,
    discover_esp32,
)

# Per-axis config key tables
AXIS_NAMES = ["X", "Y", "Z", "A", "B", "C"]
SPM_KEYS = [CFG_STEPS_PER_MM_X, CFG_STEPS_PER_MM_Y, CFG_STEPS_PER_MM_Z,
            CFG_STEPS_PER_MM_A, CFG_STEPS_PER_MM_B, CFG_STEPS_PER_MM_C]
RATE_KEYS = [CFG_MAX_RATE_X, CFG_MAX_RATE_Y, CFG_MAX_RATE_Z,
             CFG_MAX_RATE_A, CFG_MAX_RATE_B, CFG_MAX_RATE_C]
ACCEL_KEYS = [CFG_ACCEL_X, CFG_ACCEL_Y, CFG_ACCEL_Z,
              CFG_ACCEL_A, CFG_ACCEL_B, CFG_ACCEL_C]

# Per-axis pin config key tables
PIN_STEP_KEYS = [CFG_PIN_STEP_X, CFG_PIN_STEP_Y, CFG_PIN_STEP_Z,
                 CFG_PIN_STEP_A, CFG_PIN_STEP_B, CFG_PIN_STEP_C]
PIN_DIR_KEYS = [CFG_PIN_DIR_X, CFG_PIN_DIR_Y, CFG_PIN_DIR_Z,
                CFG_PIN_DIR_A, CFG_PIN_DIR_B, CFG_PIN_DIR_C]
PIN_LIMIT_KEYS = [CFG_PIN_LIMIT_X, CFG_PIN_LIMIT_Y, CFG_PIN_LIMIT_Z,
                  CFG_PIN_LIMIT_A, CFG_PIN_LIMIT_B, CFG_PIN_LIMIT_C]
PIN_SINGLE_KEYS = [
    ("Enable",      CFG_PIN_ENABLE),
    ("Probe",       CFG_PIN_PROBE),
    ("E-Stop",      CFG_PIN_ESTOP),
    ("Spindle",     CFG_PIN_SPINDLE),
    ("LED",         CFG_PIN_LED),
    ("Charge Pump", CFG_PIN_CHARGE_PUMP),
    ("Misc Out 0",  CFG_PIN_MISC_OUT0),
    ("Misc Out 1",  CFG_PIN_MISC_OUT1),
    ("Misc In 0",   CFG_PIN_MISC_IN0),
    ("Misc In 1",   CFG_PIN_MISC_IN1),
    ("Misc In 2",   CFG_PIN_MISC_IN2),
    ("Misc In 3",   CFG_PIN_MISC_IN3),
    ("Encoder A",   CFG_PIN_ENCODER_A),
    ("Encoder B",   CFG_PIN_ENCODER_B),
    ("Encoder Idx", CFG_PIN_ENCODER_INDEX),
    ("Eth MOSI",    CFG_PIN_ETH_MOSI),
    ("Eth MISO",    CFG_PIN_ETH_MISO),
    ("Eth SCLK",    CFG_PIN_ETH_SCLK),
    ("Eth INT",     CFG_PIN_ETH_INT),
    ("Eth SPI#",    CFG_PIN_ETH_SPI_HOST),
]


class WiFiCNCTester:
    def __init__(self, root):
        self.root = root
        self.root.title("WiFi CNC Protocol Tester")
        self.root.geometry("750x900")
        self.root.resizable(True, True)
        self.root.configure(bg="#2b2b2b")

        # State
        self.tcp_sock = None
        self.udp_send_sock = None
        self.status_thread = None
        self.keepalive_thread = None
        self.keepalive_running = False
        self.status_running = False
        self.connected = False
        self.status_queue = queue.Queue()
        self.device_info = {}
        self.tcp_lock = threading.Lock()  # Protects tcp_sock access

        # Status data
        self.positions = [0] * 6
        self.state = 0
        self.alarm_code = 0
        self.buf_avail = 0
        self.buf_total = 0
        self.limits = 0
        self.home_sw = 0
        self.estop_in = 0
        self.probe = 0
        self.feed_rate = 0
        self.uptime = 0
        self.misc_outputs = 0
        self.misc_inputs = 0
        self.spindle_state = 0
        self.coolant_state = 0
        self.encoder_rpm = 0
        self.encoder_position = 0
        self.encoder_index_count = 0
        self.io_module_inputs = 0
        self.io_module_outputs = 0

        self._build_ui()
        self._poll_status_queue()

    # =================================================================
    # UI Construction
    # =================================================================

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")

        # Dark theme colors
        bg = "#2b2b2b"
        fg = "#e0e0e0"
        self._frame_bg = "#353535"
        accent = "#4a9eff"

        style.configure("TFrame", background=self._frame_bg)
        style.configure("TLabel", background=self._frame_bg, foreground=fg, font=("Consolas", 10))
        style.configure("TButton", font=("Consolas", 10))
        style.configure("Header.TLabel", background=self._frame_bg, foreground=accent,
                         font=("Consolas", 11, "bold"))
        style.configure("Status.TLabel", background=self._frame_bg, foreground=fg,
                         font=("Consolas", 12, "bold"))
        style.configure("Pos.TLabel", background=self._frame_bg, foreground="#80ff80",
                         font=("Consolas", 13, "bold"))
        style.configure("TNotebook", background=bg)
        style.configure("TNotebook.Tab", font=("Consolas", 10), padding=[8, 4])

        pad = {"padx": 8, "pady": 4}

        # --- Connection Frame (always visible) ---
        conn_frame = ttk.LabelFrame(self.root, text=" Connection ", style="TFrame")
        conn_frame.pack(fill="x", padx=8, pady=(8, 2))

        row = ttk.Frame(conn_frame, style="TFrame")
        row.pack(fill="x", **pad)

        ttk.Label(row, text="ESP32 IP:", style="TLabel").pack(side="left")
        self.ip_var = tk.StringVar(value="192.168.4.1")
        self.ip_entry = ttk.Entry(row, textvariable=self.ip_var, width=18,
                                   font=("Consolas", 11))
        self.ip_entry.pack(side="left", padx=(4, 8))

        self.connect_btn = ttk.Button(row, text="Connect", command=self._on_connect)
        self.connect_btn.pack(side="left", padx=2)
        self.disconnect_btn = ttk.Button(row, text="Disconnect", command=self._on_disconnect,
                                          state="disabled")
        self.disconnect_btn.pack(side="left", padx=2)

        self.discover_btn = ttk.Button(row, text="Discover", command=self._on_discover)
        self.discover_btn.pack(side="left", padx=2)

        self.conn_indicator = tk.Canvas(row, width=18, height=18,
                                         bg=self._frame_bg, highlightthickness=0)
        self.conn_indicator.pack(side="left", padx=(8, 0))
        self._indicator_id = self.conn_indicator.create_oval(2, 2, 16, 16, fill="#666666")

        self.conn_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(row, textvariable=self.conn_status_var, style="TLabel").pack(side="left", padx=8)

        # Device info row (shown after connect)
        self.dev_info_var = tk.StringVar(value="")
        self.dev_info_label = ttk.Label(conn_frame, textvariable=self.dev_info_var,
                                         style="TLabel", foreground="#888888")
        self.dev_info_label.pack(fill="x", padx=8, pady=(0, 4))

        # --- Tabbed Notebook ---
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=2)

        control_tab = ttk.Frame(self.notebook, style="TFrame")
        config_tab = ttk.Frame(self.notebook, style="TFrame")
        io_tab = ttk.Frame(self.notebook, style="TFrame")
        pins_tab = ttk.Frame(self.notebook, style="TFrame")
        wifi_tab = ttk.Frame(self.notebook, style="TFrame")

        self.notebook.add(control_tab, text=" Control ")
        self.notebook.add(config_tab, text=" Config ")
        self.notebook.add(io_tab, text=" I/O ")
        self.notebook.add(pins_tab, text=" Pins ")
        self.notebook.add(wifi_tab, text=" WiFi ")

        self._build_control_tab(control_tab)
        self._build_config_tab(config_tab)
        self._build_io_tab(io_tab)
        self._build_pins_tab(pins_tab)
        self._build_wifi_tab(wifi_tab)

        # --- Log Frame (always visible at bottom) ---
        log_frame = ttk.LabelFrame(self.root, text=" Log ", style="TFrame")
        log_frame.pack(fill="x", padx=8, pady=(2, 8))

        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=8, bg="#1e1e1e", fg="#cccccc",
            insertbackground="#cccccc", font=("Consolas", 9),
            state="disabled", wrap="word"
        )
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        self.log_text.tag_configure("info", foreground="#cccccc")
        self.log_text.tag_configure("success", foreground="#44cc44")
        self.log_text.tag_configure("error", foreground="#ff4444")
        self.log_text.tag_configure("warn", foreground="#ffaa44")
        self.log_text.tag_configure("send", foreground="#4a9eff")

        # Keyboard bindings for jog
        self.root.bind("<KeyPress-Left>", lambda e: self._jog_start(0, -1))
        self.root.bind("<KeyPress-Right>", lambda e: self._jog_start(0, 1))
        self.root.bind("<KeyPress-Up>", lambda e: self._jog_start(1, 1))
        self.root.bind("<KeyPress-Down>", lambda e: self._jog_start(1, -1))
        self.root.bind("<KeyPress-Prior>", lambda e: self._jog_start(2, 1))
        self.root.bind("<KeyPress-Next>", lambda e: self._jog_start(2, -1))
        self.root.bind("<KeyRelease-Left>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Right>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Up>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Down>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Prior>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Next>", lambda e: self._on_jog_stop())

        # A/B/C keyboard jog: Home/End=A, Insert/Delete=B, [/]=C
        self.root.bind("<KeyPress-Home>", lambda e: self._jog_start(3, -1))
        self.root.bind("<KeyPress-End>", lambda e: self._jog_start(3, 1))
        self.root.bind("<KeyPress-Insert>", lambda e: self._jog_start(4, -1))
        self.root.bind("<KeyPress-Delete>", lambda e: self._jog_start(4, 1))
        self.root.bind("<KeyPress-bracketleft>", lambda e: self._jog_start(5, -1))
        self.root.bind("<KeyPress-bracketright>", lambda e: self._jog_start(5, 1))
        self.root.bind("<KeyRelease-Home>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-End>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Insert>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-Delete>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-bracketleft>", lambda e: self._on_jog_stop())
        self.root.bind("<KeyRelease-bracketright>", lambda e: self._on_jog_stop())

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._log("WiFi CNC Protocol Tester ready", "info")
        self._log("Keyboard jog: Arrows=X/Y, PgUp/Dn=Z, Home/End=A, Ins/Del=B, [/]=C", "info")

    # ----- Control Tab -----

    def _build_control_tab(self, parent):
        pad = {"padx": 8, "pady": 4}

        # Live Status
        status_frame = ttk.LabelFrame(parent, text=" Live Status ", style="TFrame")
        status_frame.pack(fill="x", padx=4, pady=4)

        r1 = ttk.Frame(status_frame, style="TFrame")
        r1.pack(fill="x", **pad)

        ttk.Label(r1, text="State:", style="TLabel").pack(side="left")
        self.state_var = tk.StringVar(value="---")
        ttk.Label(r1, textvariable=self.state_var, style="Status.TLabel",
                   width=12).pack(side="left", padx=(2, 16))
        ttk.Label(r1, text="Buffer:", style="TLabel").pack(side="left")
        self.buffer_var = tk.StringVar(value="---/---")
        ttk.Label(r1, textvariable=self.buffer_var, style="Status.TLabel",
                   width=10).pack(side="left", padx=(2, 16))
        ttk.Label(r1, text="Uptime:", style="TLabel").pack(side="left")
        self.uptime_var = tk.StringVar(value="--:--:--")
        ttk.Label(r1, textvariable=self.uptime_var, style="Status.TLabel",
                   width=10).pack(side="left")

        r2 = ttk.Frame(status_frame, style="TFrame")
        r2.pack(fill="x", **pad)

        ttk.Label(r2, text="Feed:", style="TLabel").pack(side="left")
        self.feed_var = tk.StringVar(value="0 stp/s")
        ttk.Label(r2, textvariable=self.feed_var, style="Status.TLabel",
                   width=14).pack(side="left", padx=(2, 16))
        ttk.Label(r2, text="Limits:", style="TLabel").pack(side="left")
        self.limits_var = tk.StringVar(value="0x00")
        ttk.Label(r2, textvariable=self.limits_var, style="Status.TLabel",
                   width=6).pack(side="left", padx=(2, 8))
        ttk.Label(r2, text="Home:", style="TLabel").pack(side="left")
        self.home_var = tk.StringVar(value="0x00")
        ttk.Label(r2, textvariable=self.home_var, style="Status.TLabel",
                   width=6).pack(side="left", padx=(2, 8))
        ttk.Label(r2, text="Probe:", style="TLabel").pack(side="left")
        self.probe_var = tk.StringVar(value="Open")
        ttk.Label(r2, textvariable=self.probe_var, style="Status.TLabel",
                   width=6).pack(side="left", padx=(2, 8))
        ttk.Label(r2, text="EStop:", style="TLabel").pack(side="left")
        self.estop_var = tk.StringVar(value="Clear")
        ttk.Label(r2, textvariable=self.estop_var, style="Status.TLabel",
                   width=8).pack(side="left")

        # Encoder status row
        enc_row = ttk.Frame(status_frame, style="TFrame")
        enc_row.pack(fill="x", **pad)
        ttk.Label(enc_row, text="Encoder:", style="TLabel").pack(side="left")
        self.enc_rpm_var = tk.StringVar(value="--- RPM")
        ttk.Label(enc_row, textvariable=self.enc_rpm_var, style="Status.TLabel",
                  width=10).pack(side="left", padx=(2, 12))
        ttk.Label(enc_row, text="Pos:", style="TLabel").pack(side="left")
        self.enc_pos_var = tk.StringVar(value="---")
        ttk.Label(enc_row, textvariable=self.enc_pos_var, style="Status.TLabel",
                  width=8).pack(side="left", padx=(2, 12))
        ttk.Label(enc_row, text="Index:", style="TLabel").pack(side="left")
        self.enc_idx_var = tk.StringVar(value="0")
        ttk.Label(enc_row, textvariable=self.enc_idx_var, style="Status.TLabel",
                  width=8).pack(side="left")

        r3 = ttk.Frame(status_frame, style="TFrame")
        r3.pack(fill="x", **pad)

        self.pos_vars = []
        for name in AXIS_NAMES:
            ttk.Label(r3, text=f"{name}:", style="TLabel").pack(side="left")
            var = tk.StringVar(value="0")
            ttk.Label(r3, textvariable=var, style="Pos.TLabel",
                       width=9, anchor="e").pack(side="left", padx=(0, 10))
            self.pos_vars.append(var)

        # Jog Controls
        jog_frame = ttk.LabelFrame(parent, text=" Jog Controls ", style="TFrame")
        jog_frame.pack(fill="x", padx=4, pady=4)

        jog_settings = ttk.Frame(jog_frame, style="TFrame")
        jog_settings.pack(fill="x", **pad)

        ttk.Label(jog_settings, text="Axis:", style="TLabel").pack(side="left")
        self.jog_axis_var = tk.StringVar(value="X")
        ttk.Combobox(jog_settings, textvariable=self.jog_axis_var,
                      values=AXIS_NAMES, width=4, state="readonly",
                      font=("Consolas", 10)).pack(side="left", padx=(2, 16))

        ttk.Label(jog_settings, text="Speed:", style="TLabel").pack(side="left")
        self.jog_speed_var = tk.StringVar(value="1000")
        ttk.Entry(jog_settings, textvariable=self.jog_speed_var, width=8,
                   font=("Consolas", 10)).pack(side="left", padx=(2, 4))
        ttk.Label(jog_settings, text="steps/sec", style="TLabel").pack(side="left")

        jog_grid = ttk.Frame(jog_frame, style="TFrame")
        jog_grid.pack(pady=(0, 8))

        btn_w = 6
        ttk.Label(jog_grid, text="", style="TLabel", width=btn_w).grid(row=0, column=0)
        self._jog_btn(jog_grid, "Y+", 1, 1, 0, 1)
        ttk.Label(jog_grid, text="", style="TLabel", width=btn_w).grid(row=0, column=2)
        ttk.Label(jog_grid, text="  ", style="TLabel", width=2).grid(row=0, column=3)
        self._jog_btn(jog_grid, "Z+", 2, 1, 0, 4)

        self._jog_btn(jog_grid, "X-", 0, -1, 1, 0)
        tk.Button(jog_grid, text="STOP", width=btn_w, height=1,
                  bg="#cc4444", fg="white", font=("Consolas", 10, "bold"),
                  activebackground="#ff4444",
                  command=self._on_jog_stop).grid(row=1, column=1, padx=2, pady=2)
        self._jog_btn(jog_grid, "X+", 0, 1, 1, 2)
        ttk.Label(jog_grid, text="  ", style="TLabel", width=2).grid(row=1, column=3)
        self._jog_btn(jog_grid, "Z-", 2, -1, 1, 4)

        ttk.Label(jog_grid, text="", style="TLabel", width=btn_w).grid(row=2, column=0)
        self._jog_btn(jog_grid, "Y-", 1, -1, 2, 1)

        # A/B/C jog buttons (to the right of Z)
        ttk.Label(jog_grid, text="  ", style="TLabel", width=2).grid(row=0, column=5)
        self._jog_btn(jog_grid, "A-", 3, -1, 0, 6)
        self._jog_btn(jog_grid, "A+", 3, 1, 0, 7)
        self._jog_btn(jog_grid, "B-", 4, -1, 1, 6)
        self._jog_btn(jog_grid, "B+", 4, 1, 1, 7)
        self._jog_btn(jog_grid, "C-", 5, -1, 2, 6)
        self._jog_btn(jog_grid, "C+", 5, 1, 2, 7)

        # Motion test controls
        mt_frame = ttk.LabelFrame(parent, text=" Motion Test ", style="TFrame")
        mt_frame.pack(fill="x", padx=4, pady=4)
        mt_row = ttk.Frame(mt_frame, style="TFrame")
        mt_row.pack(fill="x", **pad)

        ttk.Label(mt_row, text="Axis:", style="TLabel").pack(side="left")
        self.mt_axis_var = tk.StringVar(value="X")
        ttk.Combobox(mt_row, textvariable=self.mt_axis_var,
                      values=AXIS_NAMES, width=4, state="readonly",
                      font=("Consolas", 10)).pack(side="left", padx=(2, 12))

        ttk.Label(mt_row, text="Steps:", style="TLabel").pack(side="left")
        self.mt_steps_var = tk.StringVar(value="1000")
        ttk.Entry(mt_row, textvariable=self.mt_steps_var, width=8,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 12))

        ttk.Label(mt_row, text="Speed:", style="TLabel").pack(side="left")
        self.mt_speed_var = tk.StringVar(value="500")
        ttk.Entry(mt_row, textvariable=self.mt_speed_var, width=8,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 4))
        ttk.Label(mt_row, text="stp/s", style="TLabel").pack(side="left", padx=(0, 12))

        ttk.Button(mt_row, text="Send Motion",
                    command=self._on_motion_test).pack(side="left", padx=2)

        # Commands
        cmd_frame = ttk.LabelFrame(parent, text=" Commands ", style="TFrame")
        cmd_frame.pack(fill="x", padx=4, pady=4)

        cmd_row1 = ttk.Frame(cmd_frame, style="TFrame")
        cmd_row1.pack(fill="x", **pad)

        ttk.Button(cmd_row1, text="Handshake",
                    command=self._on_handshake).pack(side="left", padx=2)
        ttk.Button(cmd_row1, text="Home All",
                    command=self._on_home_all).pack(side="left", padx=2)
        ttk.Button(cmd_row1, text="Feed Hold",
                    command=self._on_feed_hold).pack(side="left", padx=2)
        ttk.Button(cmd_row1, text="Resume",
                    command=self._on_feed_resume).pack(side="left", padx=2)

        cmd_row2 = ttk.Frame(cmd_frame, style="TFrame")
        cmd_row2.pack(fill="x", padx=8, pady=(0, 4))

        tk.Button(cmd_row2, text="E-STOP", width=14, height=1,
                  bg="#cc0000", fg="white", font=("Consolas", 12, "bold"),
                  activebackground="#ff0000",
                  command=self._on_estop).pack(side="left", padx=2)
        tk.Button(cmd_row2, text="Reset", width=14, height=1,
                  bg="#446644", fg="white", font=("Consolas", 12, "bold"),
                  activebackground="#66aa66",
                  command=self._on_reset).pack(side="left", padx=2)

        # Per-axis home buttons
        cmd_row3 = ttk.Frame(cmd_frame, style="TFrame")
        cmd_row3.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(cmd_row3, text="Home:", style="TLabel").pack(side="left")
        for i, name in enumerate(AXIS_NAMES):
            ttk.Button(cmd_row3, text=name, width=3,
                        command=lambda ax=i: self._on_home_axis(ax)).pack(side="left", padx=1)

    # ----- Config Tab -----

    def _build_config_tab(self, parent):
        pad = {"padx": 8, "pady": 4}

        # Axis Parameters
        axis_frame = ttk.LabelFrame(parent, text=" Axis Parameters ", style="TFrame")
        axis_frame.pack(fill="x", padx=4, pady=4)

        # Header row
        hdr = ttk.Frame(axis_frame, style="TFrame")
        hdr.pack(fill="x", padx=8, pady=(4, 0))
        ttk.Label(hdr, text="Axis", style="Header.TLabel", width=5).pack(side="left")
        ttk.Label(hdr, text="Steps/mm", style="Header.TLabel", width=12).pack(side="left", padx=4)
        ttk.Label(hdr, text="Max Rate", style="Header.TLabel", width=12).pack(side="left", padx=4)
        ttk.Label(hdr, text="Accel", style="Header.TLabel", width=12).pack(side="left", padx=4)

        # Per-axis entries
        self.axis_spm_vars = []   # Steps/mm (float)
        self.axis_rate_vars = []  # Max rate (uint32)
        self.axis_accel_vars = [] # Acceleration (uint32)

        for name in AXIS_NAMES:
            row = ttk.Frame(axis_frame, style="TFrame")
            row.pack(fill="x", padx=8, pady=1)
            ttk.Label(row, text=f"  {name}", style="TLabel", width=5).pack(side="left")

            spm_var = tk.StringVar(value="800.0")
            ttk.Entry(row, textvariable=spm_var, width=12,
                      font=("Consolas", 10)).pack(side="left", padx=4)
            self.axis_spm_vars.append(spm_var)

            rate_var = tk.StringVar(value="20000")
            ttk.Entry(row, textvariable=rate_var, width=12,
                      font=("Consolas", 10)).pack(side="left", padx=4)
            self.axis_rate_vars.append(rate_var)

            accel_var = tk.StringVar(value="5000")
            ttk.Entry(row, textvariable=accel_var, width=12,
                      font=("Consolas", 10)).pack(side="left", padx=4)
            self.axis_accel_vars.append(accel_var)

        axis_btns = ttk.Frame(axis_frame, style="TFrame")
        axis_btns.pack(fill="x", padx=8, pady=(4, 8))
        ttk.Button(axis_btns, text="Read Axis Config",
                    command=self._on_axis_config_read).pack(side="left", padx=2)
        ttk.Button(axis_btns, text="Write & Save",
                    command=self._on_axis_config_write).pack(side="left", padx=2)
        ttk.Label(axis_btns, text="stp/mm | stp/s | stp/s\u00b2",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

        # Timing Parameters
        timing_frame = ttk.LabelFrame(parent, text=" Timing ", style="TFrame")
        timing_frame.pack(fill="x", padx=4, pady=4)

        trow = ttk.Frame(timing_frame, style="TFrame")
        trow.pack(fill="x", **pad)

        timing_items = [
            ("Step Pulse (us):", "3"),
            ("Dir Setup (us):", "2"),
            ("Idle Delay (ms):", "255"),
            ("Status Int (ms):", "50"),
        ]
        self.timing_vars = []
        for label, default in timing_items:
            ttk.Label(trow, text=label, style="TLabel").pack(side="left")
            var = tk.StringVar(value=default)
            ttk.Entry(trow, textvariable=var, width=6,
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))
            self.timing_vars.append(var)

        timing_btns = ttk.Frame(timing_frame, style="TFrame")
        timing_btns.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(timing_btns, text="Read Timing",
                    command=self._on_timing_read).pack(side="left", padx=2)
        ttk.Button(timing_btns, text="Write & Save",
                    command=self._on_timing_write).pack(side="left", padx=2)

        # Homing Parameters
        home_frame = ttk.LabelFrame(parent, text=" Homing ", style="TFrame")
        home_frame.pack(fill="x", padx=4, pady=4)

        hrow = ttk.Frame(home_frame, style="TFrame")
        hrow.pack(fill="x", **pad)

        home_items = [
            ("Dir Mask (hex):", "0x00"),
            ("Seek Rate:", "5000"),
            ("Feed Rate:", "500"),
            ("Pulloff:", "100"),
        ]
        self.homing_vars = []
        for label, default in home_items:
            ttk.Label(hrow, text=label, style="TLabel").pack(side="left")
            var = tk.StringVar(value=default)
            ttk.Entry(hrow, textvariable=var, width=8,
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))
            self.homing_vars.append(var)

        home_btns = ttk.Frame(home_frame, style="TFrame")
        home_btns.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(home_btns, text="Read Homing",
                    command=self._on_homing_read).pack(side="left", padx=2)
        ttk.Button(home_btns, text="Write & Save",
                    command=self._on_homing_write).pack(side="left", padx=2)
        ttk.Label(home_btns, text="Dir mask: bit=1 means positive direction",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

        # Encoder Parameters
        enc_frame = ttk.LabelFrame(parent, text=" Spindle Encoder ", style="TFrame")
        enc_frame.pack(fill="x", padx=4, pady=4)

        enc_row = ttk.Frame(enc_frame, style="TFrame")
        enc_row.pack(fill="x", **pad)

        ttk.Label(enc_row, text="PPR:", style="TLabel").pack(side="left")
        self.enc_ppr_var = tk.StringVar(value="0")
        ttk.Entry(enc_row, textvariable=self.enc_ppr_var, width=6,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        ttk.Label(enc_row, text="Mode:", style="TLabel").pack(side="left")
        self.enc_mode_var = tk.StringVar(value="Quadrature")
        ttk.Combobox(enc_row, textvariable=self.enc_mode_var,
                      values=["Quadrature", "Index-only"], width=12, state="readonly",
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        ttk.Label(enc_row, text="Filter (ns):", style="TLabel").pack(side="left")
        self.enc_filter_var = tk.StringVar(value="1000")
        ttk.Entry(enc_row, textvariable=self.enc_filter_var, width=6,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        enc_btns = ttk.Frame(enc_frame, style="TFrame")
        enc_btns.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(enc_btns, text="Read Encoder",
                    command=self._on_encoder_read).pack(side="left", padx=2)
        ttk.Button(enc_btns, text="Write & Save",
                    command=self._on_encoder_write).pack(side="left", padx=2)
        ttk.Label(enc_btns, text="PPR=0 disables encoder. Pins on Pins tab.",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

        # I/O Module Config
        iomod_cfg_frame = ttk.LabelFrame(parent, text=" I/O Module ", style="TFrame")
        iomod_cfg_frame.pack(fill="x", padx=4, pady=4)

        iomod_r1 = ttk.Frame(iomod_cfg_frame, style="TFrame")
        iomod_r1.pack(fill="x", **pad)

        ttk.Label(iomod_r1, text="Device Mode:", style="TLabel").pack(side="left")
        self.dev_mode_var = tk.StringVar(value="Motion Controller")
        ttk.Combobox(iomod_r1, textvariable=self.dev_mode_var,
                      values=["Motion Controller", "I/O Module"], width=16, state="readonly",
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        ttk.Label(iomod_r1, text="Channels:", style="TLabel").pack(side="left")
        self.io_ch_count_var = tk.StringVar(value="0")
        ttk.Entry(iomod_r1, textvariable=self.io_ch_count_var, width=4,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        iomod_r2 = ttk.Frame(iomod_cfg_frame, style="TFrame")
        iomod_r2.pack(fill="x", **pad)

        ttk.Label(iomod_r2, text="Dir Mask (hex):", style="TLabel").pack(side="left")
        self.io_dir_var = tk.StringVar(value="0000")
        ttk.Entry(iomod_r2, textvariable=self.io_dir_var, width=6,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        ttk.Label(iomod_r2, text="Pullup (hex):", style="TLabel").pack(side="left")
        self.io_pull_var = tk.StringVar(value="FFFF")
        ttk.Entry(iomod_r2, textvariable=self.io_pull_var, width=6,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        ttk.Label(iomod_r2, text="Invert (hex):", style="TLabel").pack(side="left")
        self.io_inv_var = tk.StringVar(value="0000")
        ttk.Entry(iomod_r2, textvariable=self.io_inv_var, width=6,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 10))

        # I/O Module Pin Assignments (GPIO for each channel)
        iomod_pins_frame = ttk.Frame(iomod_cfg_frame, style="TFrame")
        iomod_pins_frame.pack(fill="x", **pad)
        ttk.Label(iomod_pins_frame, text="Channel GPIO pins:", style="TLabel").pack(side="left")
        self.io_pin_vars = []
        for ch in range(16):
            var = tk.StringVar(value="255")
            ttk.Entry(iomod_pins_frame, textvariable=var, width=3,
                      font=("Consolas", 9)).pack(side="left", padx=1)
            self.io_pin_vars.append(var)
        ttk.Label(iomod_pins_frame, text="(255=unused)",
                  style="TLabel", foreground="#888888").pack(side="left", padx=4)

        iomod_btns = ttk.Frame(iomod_cfg_frame, style="TFrame")
        iomod_btns.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(iomod_btns, text="Read I/O Module",
                    command=self._on_iomodule_read).pack(side="left", padx=2)
        ttk.Button(iomod_btns, text="Write & Save",
                    command=self._on_iomodule_write).pack(side="left", padx=2)
        ttk.Label(iomod_btns, text="Dir: 1=output, 0=input per channel. Reboot to apply.",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

    # ----- I/O Tab -----

    def _build_io_tab(self, parent):
        pad = {"padx": 8, "pady": 4}

        # Live I/O Status
        live_frame = ttk.LabelFrame(parent, text=" Live I/O Status ", style="TFrame")
        live_frame.pack(fill="x", padx=4, pady=4)

        # Misc outputs row
        out_row = ttk.Frame(live_frame, style="TFrame")
        out_row.pack(fill="x", **pad)
        ttk.Label(out_row, text="Misc Outputs:", style="TLabel").pack(side="left")
        self.misc_out_indicators = []
        for i in range(5):
            canvas = tk.Canvas(out_row, width=18, height=18,
                               bg=self._frame_bg, highlightthickness=0)
            canvas.pack(side="left", padx=2)
            dot = canvas.create_oval(2, 2, 16, 16, fill="#444444")
            self.misc_out_indicators.append((canvas, dot))
            ttk.Label(out_row, text=str(i), style="TLabel",
                      font=("Consolas", 8)).pack(side="left", padx=(0, 4))

        ttk.Label(out_row, text="    Misc Inputs:", style="TLabel").pack(side="left")
        self.misc_in_indicators = []
        for i in range(5):
            canvas = tk.Canvas(out_row, width=18, height=18,
                               bg=self._frame_bg, highlightthickness=0)
            canvas.pack(side="left", padx=2)
            dot = canvas.create_oval(2, 2, 16, 16, fill="#444444")
            self.misc_in_indicators.append((canvas, dot))
            ttk.Label(out_row, text=str(i), style="TLabel",
                      font=("Consolas", 8)).pack(side="left", padx=(0, 4))

        # Spindle / Coolant row
        sc_row = ttk.Frame(live_frame, style="TFrame")
        sc_row.pack(fill="x", **pad)
        ttk.Label(sc_row, text="Spindle:", style="TLabel").pack(side="left")
        self.spindle_var = tk.StringVar(value="OFF")
        ttk.Label(sc_row, textvariable=self.spindle_var, style="Status.TLabel",
                  width=6).pack(side="left", padx=(2, 16))
        ttk.Label(sc_row, text="Coolant:", style="TLabel").pack(side="left")
        self.coolant_var = tk.StringVar(value="OFF")
        ttk.Label(sc_row, textvariable=self.coolant_var, style="Status.TLabel",
                  width=12).pack(side="left", padx=(2, 16))

        # Output control buttons
        ctrl_frame = ttk.LabelFrame(parent, text=" Output Controls ", style="TFrame")
        ctrl_frame.pack(fill="x", padx=4, pady=4)

        out_ctrl_row = ttk.Frame(ctrl_frame, style="TFrame")
        out_ctrl_row.pack(fill="x", **pad)
        ttk.Label(out_ctrl_row, text="Toggle Misc Out:", style="TLabel").pack(side="left")
        self.misc_out_toggle_state = [False] * 5
        self.misc_out_btns = []
        for i in range(5):
            btn = ttk.Button(out_ctrl_row, text=f"Out {i}", width=6,
                             command=lambda idx=i: self._toggle_misc_output(idx))
            btn.pack(side="left", padx=2)
            self.misc_out_btns.append(btn)

        spn_row = ttk.Frame(ctrl_frame, style="TFrame")
        spn_row.pack(fill="x", **pad)
        ttk.Label(spn_row, text="Spindle:", style="TLabel").pack(side="left")
        ttk.Button(spn_row, text="OFF", width=5,
                   command=lambda: self._send_io_control(spindle=0)).pack(side="left", padx=2)
        ttk.Button(spn_row, text="CW", width=5,
                   command=lambda: self._send_io_control(spindle=1)).pack(side="left", padx=2)
        ttk.Button(spn_row, text="CCW", width=5,
                   command=lambda: self._send_io_control(spindle=2)).pack(side="left", padx=2)
        ttk.Label(spn_row, text="  Coolant:", style="TLabel").pack(side="left")
        ttk.Button(spn_row, text="Flood", width=6,
                   command=lambda: self._toggle_coolant(0)).pack(side="left", padx=2)
        ttk.Button(spn_row, text="Mist", width=6,
                   command=lambda: self._toggle_coolant(1)).pack(side="left", padx=2)
        ttk.Button(spn_row, text="All Off", width=6,
                   command=lambda: self._send_io_control(coolant=0)).pack(side="left", padx=2)

        self._io_spindle_state = 0
        self._io_coolant_state = 0

        # I/O Module Channel Status
        iomod_frame = ttk.LabelFrame(parent, text=" I/O Module Channels ", style="TFrame")
        iomod_frame.pack(fill="x", padx=4, pady=4)

        iomod_in_row = ttk.Frame(iomod_frame, style="TFrame")
        iomod_in_row.pack(fill="x", **pad)
        ttk.Label(iomod_in_row, text="Module Inputs:", style="TLabel").pack(side="left")
        self.io_mod_in_indicators = []
        for i in range(16):
            canvas = tk.Canvas(iomod_in_row, width=14, height=14,
                               bg=self._frame_bg, highlightthickness=0)
            canvas.pack(side="left", padx=1)
            dot = canvas.create_oval(1, 1, 13, 13, fill="#444444")
            self.io_mod_in_indicators.append((canvas, dot))

        iomod_out_row = ttk.Frame(iomod_frame, style="TFrame")
        iomod_out_row.pack(fill="x", **pad)
        ttk.Label(iomod_out_row, text="Module Outputs:", style="TLabel").pack(side="left")
        self.io_mod_out_indicators = []
        for i in range(16):
            canvas = tk.Canvas(iomod_out_row, width=14, height=14,
                               bg=self._frame_bg, highlightthickness=0)
            canvas.pack(side="left", padx=1)
            dot = canvas.create_oval(1, 1, 13, 13, fill="#444444")
            self.io_mod_out_indicators.append((canvas, dot))

        ttk.Label(iomod_frame, text="Shows I/O expansion module state (when connected in I/O module mode)",
                  style="TLabel", foreground="#888888").pack(padx=8, pady=(0, 4))

        # I/O Inversion Config
        io_frame = ttk.LabelFrame(parent, text=" I/O Inversion Config ", style="TFrame")
        io_frame.pack(fill="x", padx=4, pady=4)

        io_row1 = ttk.Frame(io_frame, style="TFrame")
        io_row1.pack(fill="x", **pad)

        self.io_vars = {}
        io_items = [
            ("Limit", CFG_INVERT_LIMIT),
            ("Home", CFG_INVERT_HOME),
            ("E-Stop", CFG_INVERT_ESTOP),
            ("Probe", CFG_INVERT_PROBE),
        ]
        for label, key in io_items:
            ttk.Label(io_row1, text=f"{label}:", style="TLabel").pack(side="left")
            var = tk.StringVar(value="0x00")
            ttk.Entry(io_row1, textvariable=var, width=6,
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))
            self.io_vars[key] = var

        io_row2 = ttk.Frame(io_frame, style="TFrame")
        io_row2.pack(fill="x", padx=8, pady=(0, 8))

        ttk.Button(io_row2, text="Read I/O Config",
                   command=self._on_io_read).pack(side="left", padx=2)
        ttk.Button(io_row2, text="Write I/O Config",
                   command=self._on_io_write).pack(side="left", padx=2)
        ttk.Label(io_row2, text="Bitmask: bit0=X..bit5=C, E-Stop/Probe: 0/1",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

        # Step/Dir/Enable inversion
        sde_frame = ttk.LabelFrame(parent, text=" Step/Dir/Enable Inversion ", style="TFrame")
        sde_frame.pack(fill="x", padx=4, pady=4)

        sde_row = ttk.Frame(sde_frame, style="TFrame")
        sde_row.pack(fill="x", **pad)

        sde_items = [
            ("Step", CFG_INVERT_STEP),
            ("Dir", CFG_INVERT_DIR),
            ("Enable", CFG_INVERT_ENABLE),
        ]
        for label, key in sde_items:
            ttk.Label(sde_row, text=f"{label}:", style="TLabel").pack(side="left")
            var = tk.StringVar(value="0x00")
            ttk.Entry(sde_row, textvariable=var, width=6,
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))
            self.io_vars[key] = var

        sde_btns = ttk.Frame(sde_frame, style="TFrame")
        sde_btns.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(sde_btns, text="Read",
                   command=self._on_io_read).pack(side="left", padx=2)
        ttk.Button(sde_btns, text="Write & Save",
                   command=self._on_io_write).pack(side="left", padx=2)
        ttk.Label(sde_btns, text="Bitmask: bit0=X..bit5=C",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

        # Output config (charge pump, spindle PWM)
        out_cfg_frame = ttk.LabelFrame(parent, text=" Output Config ", style="TFrame")
        out_cfg_frame.pack(fill="x", padx=4, pady=4)

        ocfg_row = ttk.Frame(out_cfg_frame, style="TFrame")
        ocfg_row.pack(fill="x", **pad)

        ttk.Label(ocfg_row, text="Charge Pump (Hz):", style="TLabel").pack(side="left")
        self.cp_freq_var = tk.StringVar(value="10000")
        ttk.Entry(ocfg_row, textvariable=self.cp_freq_var, width=8,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 12))

        ttk.Label(ocfg_row, text="Spindle PWM (Hz):", style="TLabel").pack(side="left")
        self.sp_freq_var = tk.StringVar(value="1000")
        ttk.Entry(ocfg_row, textvariable=self.sp_freq_var, width=8,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 12))

        ttk.Label(ocfg_row, text="Max RPM:", style="TLabel").pack(side="left")
        self.sp_rpm_var = tk.StringVar(value="24000")
        ttk.Entry(ocfg_row, textvariable=self.sp_rpm_var, width=8,
                  font=("Consolas", 10)).pack(side="left", padx=(2, 4))

        ocfg_btns = ttk.Frame(out_cfg_frame, style="TFrame")
        ocfg_btns.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(ocfg_btns, text="Read",
                   command=self._on_output_config_read).pack(side="left", padx=2)
        ttk.Button(ocfg_btns, text="Write & Save",
                   command=self._on_output_config_write).pack(side="left", padx=2)
        ttk.Label(ocfg_btns, text="0 Hz = disabled",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

    # ----- Pins Tab -----

    def _build_pins_tab(self, parent):
        pad = {"padx": 8, "pady": 4}

        # Stepper Pins (per-axis: Step, Dir, Limit)
        stepper_frame = ttk.LabelFrame(parent, text=" Stepper Pins (GPIO Number) ", style="TFrame")
        stepper_frame.pack(fill="x", padx=4, pady=4)

        # Header row
        hdr = ttk.Frame(stepper_frame, style="TFrame")
        hdr.pack(fill="x", padx=8, pady=(4, 0))
        ttk.Label(hdr, text="Axis", style="Header.TLabel", width=5).pack(side="left")
        ttk.Label(hdr, text="Step", style="Header.TLabel", width=8).pack(side="left", padx=4)
        ttk.Label(hdr, text="Dir", style="Header.TLabel", width=8).pack(side="left", padx=4)
        ttk.Label(hdr, text="Limit", style="Header.TLabel", width=8).pack(side="left", padx=4)

        self.pin_step_vars = []
        self.pin_dir_vars = []
        self.pin_limit_vars = []

        for name in AXIS_NAMES:
            row = ttk.Frame(stepper_frame, style="TFrame")
            row.pack(fill="x", padx=8, pady=1)
            ttk.Label(row, text=f"  {name}", style="TLabel", width=5).pack(side="left")

            step_var = tk.StringVar(value="0")
            ttk.Entry(row, textvariable=step_var, width=8,
                      font=("Consolas", 10)).pack(side="left", padx=4)
            self.pin_step_vars.append(step_var)

            dir_var = tk.StringVar(value="0")
            ttk.Entry(row, textvariable=dir_var, width=8,
                      font=("Consolas", 10)).pack(side="left", padx=4)
            self.pin_dir_vars.append(dir_var)

            limit_var = tk.StringVar(value="0")
            ttk.Entry(row, textvariable=limit_var, width=8,
                      font=("Consolas", 10)).pack(side="left", padx=4)
            self.pin_limit_vars.append(limit_var)

        # Single pins (Enable, Probe, E-Stop, Spindle, LED, Charge Pump, Misc Out 0/1)
        single_frame = ttk.LabelFrame(parent, text=" Other Pins (GPIO Number) ", style="TFrame")
        single_frame.pack(fill="x", padx=4, pady=4)

        self.pin_single_vars = {}
        s_row1 = ttk.Frame(single_frame, style="TFrame")
        s_row1.pack(fill="x", **pad)
        s_row2 = ttk.Frame(single_frame, style="TFrame")
        s_row2.pack(fill="x", **pad)
        s_row3 = ttk.Frame(single_frame, style="TFrame")
        s_row3.pack(fill="x", **pad)
        s_row4 = ttk.Frame(single_frame, style="TFrame")
        s_row4.pack(fill="x", **pad)

        pin_rows = [s_row1, s_row2, s_row3, s_row4]
        for i, (label, key) in enumerate(PIN_SINGLE_KEYS):
            row = pin_rows[i // 5]
            ttk.Label(row, text=f"{label}:", style="TLabel").pack(side="left")
            var = tk.StringVar(value="0")
            ttk.Entry(row, textvariable=var, width=4,
                      font=("Consolas", 10)).pack(side="left", padx=(2, 10))
            self.pin_single_vars[key] = var

        # Buttons
        btn_frame = ttk.Frame(parent, style="TFrame")
        btn_frame.pack(fill="x", padx=8, pady=4)
        ttk.Button(btn_frame, text="Read Pin Config",
                   command=self._on_pin_config_read).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="Write & Save",
                   command=self._on_pin_config_write).pack(side="left", padx=2)
        ttk.Label(btn_frame, text="Pin changes require ESP32 reboot to take effect",
                  style="TLabel", foreground="#ffaa44").pack(side="left", padx=8)

    # ----- WiFi Tab -----

    def _build_wifi_tab(self, parent):
        pad = {"padx": 8, "pady": 4}

        wifi_frame = ttk.LabelFrame(parent, text=" WiFi Setup ", style="TFrame")
        wifi_frame.pack(fill="x", padx=4, pady=4)

        wifi_row1 = ttk.Frame(wifi_frame, style="TFrame")
        wifi_row1.pack(fill="x", **pad)

        ttk.Label(wifi_row1, text="SSID:", style="TLabel").pack(side="left")
        self.wifi_ssid_var = tk.StringVar()
        ttk.Entry(wifi_row1, textvariable=self.wifi_ssid_var, width=20,
                  font=("Consolas", 10)).pack(side="left", padx=(4, 12))

        ttk.Label(wifi_row1, text="Password:", style="TLabel").pack(side="left")
        self.wifi_pass_var = tk.StringVar()
        self.wifi_pass_entry = ttk.Entry(wifi_row1, textvariable=self.wifi_pass_var,
                                          width=20, font=("Consolas", 10), show="*")
        self.wifi_pass_entry.pack(side="left", padx=(4, 12))

        self.wifi_show_var = tk.BooleanVar(value=False)
        tk.Checkbutton(wifi_row1, text="Show", variable=self.wifi_show_var,
                        bg=self._frame_bg, fg="#e0e0e0", selectcolor="#404040",
                        activebackground=self._frame_bg, activeforeground="#e0e0e0",
                        font=("Consolas", 9),
                        command=self._toggle_password_visibility).pack(side="left")

        wifi_row2 = ttk.Frame(wifi_frame, style="TFrame")
        wifi_row2.pack(fill="x", padx=8, pady=(0, 8))

        self.wifi_apply_btn = ttk.Button(wifi_row2, text="Apply & Reboot ESP32",
                                          command=self._on_wifi_setup)
        self.wifi_apply_btn.pack(side="left", padx=2)
        ttk.Label(wifi_row2, text="ESP32 reboots to join your network after apply",
                  style="TLabel", foreground="#888888").pack(side="left", padx=8)

    def _jog_btn(self, parent, text, axis, direction, row, col):
        """Create a jog direction button."""
        btn = ttk.Button(parent, text=text, width=6,
                          command=lambda: self._jog_start(axis, direction))
        btn.grid(row=row, column=col, padx=2, pady=2)
        return btn

    # =================================================================
    # Logging
    # =================================================================

    def _log(self, msg, tag="info"):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n", tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # =================================================================
    # Connection
    # =================================================================

    def _on_connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            self._log("Enter an IP address", "error")
            return

        self._log(f"Connecting to {ip}...", "send")
        self.connect_btn.configure(state="disabled")
        threading.Thread(target=self._connect_thread, args=(ip,), daemon=True).start()

    def _connect_thread(self, ip):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((ip, TCP_CONTROL_PORT))

            host_name = b"GUITester\x00" + b'\x00' * 22
            payload = struct.pack('<I', 0x01000000) + host_name
            header = build_header(PKT_HANDSHAKE_REQ, len(payload))
            packet = finalize_packet(header + payload)
            tcp_send(sock, packet)

            resp_data = tcp_recv(sock)
            if not validate_packet(resp_data) or len(resp_data) < 62:
                self.root.after(0, self._connect_failed, "Invalid handshake response")
                sock.close()
                return

            fw_ver = struct.unpack_from('<I', resp_data, 18)[0]
            num_axes = resp_data[22]
            caps = resp_data[23]
            buf_cap = struct.unpack_from('<H', resp_data, 24)[0]
            max_rate = struct.unpack_from('<I', resp_data, 26)[0]
            dev_name = resp_data[30:62].split(b'\x00')[0].decode('ascii', errors='replace')

            # Extended handshake fields (v1.1)
            device_mode = 0
            encoder_ppr = 0
            io_channel_count = 0
            if len(resp_data) >= 18 + 48:
                device_mode = resp_data[62]
                encoder_ppr = (resp_data[63] << 8) | resp_data[64]
                io_channel_count = resp_data[65]

            self.device_info = {
                'firmware_version': f"{(fw_ver >> 24) & 0xFF}.{(fw_ver >> 16) & 0xFF}.{fw_ver & 0xFFFF}",
                'num_axes': num_axes,
                'capabilities': caps,
                'buffer_capacity': buf_cap,
                'max_step_rate': max_rate,
                'device_name': dev_name,
                'device_mode': device_mode,
                'encoder_ppr': encoder_ppr,
                'io_channel_count': io_channel_count,
            }

            # Increase socket timeout for config operations (firmware timeout is 10s)
            sock.settimeout(8.0)
            self.tcp_sock = sock
            self.udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Send UDP registration packet so firmware knows our address
            reg_payload = struct.pack('<Bxxx', 0xFF)
            reg_header = build_header(PKT_JOG_STOP, len(reg_payload))
            reg_packet = finalize_packet(reg_header + reg_payload)
            self.udp_send_sock.sendto(reg_packet, (ip, UDP_MOTION_PORT))

            # Start status listener
            self.status_running = True
            self.status_thread = threading.Thread(target=self._status_listener,
                                                   args=(ip,), daemon=True)
            self.status_thread.start()

            # Mark connected BEFORE starting keepalive so the thread
            # doesn't exit immediately due to self.connected == False
            self.connected = True

            # Start TCP keepalive thread
            self.keepalive_running = True
            self.keepalive_thread = threading.Thread(target=self._keepalive_loop,
                                                      daemon=True)
            self.keepalive_thread.start()

            self.root.after(0, self._connect_success, ip)

        except Exception as e:
            self.root.after(0, self._connect_failed, str(e))

    def _connect_success(self, ip):
        self.conn_indicator.itemconfig(self._indicator_id, fill="#44cc44")
        info = self.device_info
        mode_str = "I/O Module" if info.get('device_mode', 0) == 1 else "Motion Ctrl"
        self.conn_status_var.set(f"{info['device_name']} v{info['firmware_version']}")
        dev_info_parts = [
            f"Mode: {mode_str}",
            f"Axes: {info['num_axes']}",
            f"Buffer: {info['buffer_capacity']}",
            f"Max rate: {info['max_step_rate']:,} stp/s",
        ]
        if info.get('encoder_ppr', 0) > 0:
            dev_info_parts.append(f"Encoder: {info['encoder_ppr']} PPR")
        if info.get('io_channel_count', 0) > 0:
            dev_info_parts.append(f"I/O: {info['io_channel_count']}ch")
        self.dev_info_var.set("  ".join(dev_info_parts))
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")
        self._log(f"Connected to {ip}", "success")
        self._log(f"  Device: {info['device_name']}  "
                  f"FW: {info['firmware_version']}  "
                  f"Mode: {mode_str}  "
                  f"Caps: 0x{info['capabilities']:02X}", "success")

    def _connect_failed(self, reason):
        self.connect_btn.configure(state="normal")
        self._log(f"Connection failed: {reason}", "error")

    def _on_disconnect(self):
        self._disconnect()
        self._log("Disconnected", "warn")

    def _disconnect(self):
        self.keepalive_running = False
        self.status_running = False
        self.connected = False
        if self.tcp_sock:
            try:
                self.tcp_sock.close()
            except Exception:
                pass
            self.tcp_sock = None
        if self.udp_send_sock:
            try:
                self.udp_send_sock.close()
            except Exception:
                pass
            self.udp_send_sock = None

        self.conn_indicator.itemconfig(self._indicator_id, fill="#666666")
        self.conn_status_var.set("Disconnected")
        self.dev_info_var.set("")
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")

    # =================================================================
    # TCP Reconnect + Safe Config Read
    # =================================================================

    def _tcp_with_reconnect(self, fn, label="Config"):
        """Run fn(self.tcp_sock) with tcp_lock, auto-reconnect on failure.

        fn receives the tcp socket and should do its config reads/writes.
        Returns True on success.
        """
        if not self.connected:
            self.root.after(0, self._log, "Not connected", "error")
            return False
        try:
            with self.tcp_lock:
                fn(self.tcp_sock)
            return True
        except Exception:
            self.root.after(0, self._log,
                            f"{label}: connection error, reconnecting...", "warn")
            try:
                with self.tcp_lock:
                    if self._tcp_reconnect():
                        fn(self.tcp_sock)
                        return True
            except Exception as e2:
                self.root.after(0, self._log,
                                f"{label} failed after reconnect: {e2}", "error")
            return False

    def _tcp_reconnect(self):
        """Attempt to re-establish the TCP connection (call with tcp_lock held)."""
        ip = self.ip_var.get().strip()
        if not ip:
            return False
        try:
            if self.tcp_sock:
                try:
                    self.tcp_sock.close()
                except Exception:
                    pass
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((ip, TCP_CONTROL_PORT))

            # Re-handshake
            host_name = b"GUITester\x00" + b'\x00' * 22
            payload = struct.pack('<I', 0x01000000) + host_name
            header = build_header(PKT_HANDSHAKE_REQ, len(payload))
            packet = finalize_packet(header + payload)
            tcp_send(sock, packet)
            resp_data = tcp_recv(sock)
            if not validate_packet(resp_data) or len(resp_data) < 62:
                sock.close()
                return False

            sock.settimeout(8.0)
            self.tcp_sock = sock
            self.root.after(0, self._log, "TCP reconnected", "success")
            return True
        except Exception as e:
            self.root.after(0, self._log, f"TCP reconnect failed: {e}", "error")
            return False

    # =================================================================
    # TCP Keepalive
    # =================================================================

    def _keepalive_loop(self):
        """Send PKT_PING every 2s to prevent firmware TCP timeout (10s)."""
        fail_count = 0
        while self.keepalive_running and self.connected:
            # Try to acquire lock with timeout — don't block forever if
            # a config read thread is holding it (they send their own packets
            # which keep the firmware connection alive)
            acquired = self.tcp_lock.acquire(timeout=3.0)
            if not acquired:
                # Config read is active and sending packets — connection is alive
                time.sleep(2.0)
                continue
            try:
                if not self.tcp_sock:
                    break
                ping_payload = struct.pack('<I', 0)  # ping_id
                header = build_header(PKT_PING, len(ping_payload))
                packet = finalize_packet(header + ping_payload)
                tcp_send(self.tcp_sock, packet)
                tcp_recv(self.tcp_sock)  # Read PKT_PONG to clear buffer
                fail_count = 0
            except Exception:
                fail_count += 1
                if fail_count >= 2 and self.keepalive_running:
                    # Try to reconnect TCP before giving up
                    self.root.after(0, self._log,
                                    "TCP connection lost \u2014 reconnecting...", "warn")
                    if self._tcp_reconnect():
                        fail_count = 0
                    else:
                        self.root.after(0, self._log,
                                        "TCP reconnect failed \u2014 disconnected", "error")
                        self.root.after(0, self._disconnect)
                        break
            finally:
                self.tcp_lock.release()
            time.sleep(2.0)

    # =================================================================
    # Discovery
    # =================================================================

    def _on_discover(self):
        self.discover_btn.configure(state="disabled")
        self._log("Searching for ESP32 on the network...", "send")
        threading.Thread(target=self._discover_thread, daemon=True).start()

    def _discover_thread(self):
        try:
            ip = discover_esp32(timeout=5.0)
            if ip:
                self.root.after(0, self._discover_success, ip)
            else:
                self.root.after(0, self._log,
                                "No ESP32 found. Check power and network.", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Discovery error: {e}", "error")
        self.root.after(0, self.discover_btn.configure, {"state": "normal"})

    def _discover_success(self, ip):
        self.ip_var.set(ip)
        self._log(f"Found ESP32 at {ip}", "success")

    # =================================================================
    # Status Listener Thread
    # =================================================================

    def _status_listener(self, ip):
        """Background thread: listen for UDP status reports."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', UDP_STATUS_PORT))
        sock.settimeout(0.5)

        while self.status_running:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            if not validate_packet(data):
                continue

            pkt_type = data[5]

            # Handle HOME_COMPLETE packet (0x22)
            if pkt_type == PKT_HOME_COMPLETE and len(data) >= 18 + 4:
                axis_mask, success = struct.unpack_from('<BB', data, 18)
                axes = [AXIS_NAMES[i] for i in range(6) if axis_mask & (1 << i)]
                result = "SUCCESS" if success else "FAILED"
                self.status_queue.put({
                    '_home_complete': True,
                    'axes': ','.join(axes) if axes else '?',
                    'success': bool(success),
                    'result': result,
                })
                continue

            if pkt_type != PKT_STATUS_REPORT:
                continue
            if len(data) < 18 + 46:
                continue

            off = 18
            positions = list(struct.unpack_from('<6i', data, off))
            off += 24
            buf_avail, buf_total = struct.unpack_from('<HH', data, off)
            off += 4
            state, alarm, limits, probe, homing, home_sw, estop_in, flags = struct.unpack_from('<8B', data, off)
            off += 8
            seg_id = struct.unpack_from('<H', data, off)[0]
            off += 2
            uptime = struct.unpack_from('<I', data, off)[0]
            off += 4
            feed_rate = struct.unpack_from('<i', data, off)[0]
            off += 4

            # Extended fields (v1.1) - backward compatible with old 46-byte payloads
            misc_out = misc_in = spindle_st = coolant_st = 0
            if len(data) >= 18 + 50:
                misc_out, misc_in, spindle_st, coolant_st = struct.unpack_from('<4B', data, off)
                off += 4

            # Spindle encoder + I/O module fields (v1.1, 12 bytes)
            enc_rpm = enc_pos = 0
            enc_idx_count = 0
            io_mod_in = io_mod_out = 0
            if len(data) >= 18 + 62:
                enc_rpm, enc_pos, enc_idx_count, io_mod_in, io_mod_out = \
                    struct.unpack_from('<HHIHH', data, off)

            self.status_queue.put({
                'positions': positions,
                'buf_avail': buf_avail,
                'buf_total': buf_total,
                'state': state,
                'alarm': alarm,
                'limits': limits,
                'home_sw': home_sw,
                'estop_in': estop_in,
                'probe': probe,
                'feed_rate': feed_rate,
                'uptime': uptime,
                'misc_outputs': misc_out,
                'misc_inputs': misc_in,
                'spindle_state': spindle_st,
                'coolant_state': coolant_st,
                'encoder_rpm': enc_rpm,
                'encoder_position': enc_pos,
                'encoder_index_count': enc_idx_count,
                'io_module_inputs': io_mod_in,
                'io_module_outputs': io_mod_out,
            })

        sock.close()

    def _poll_status_queue(self):
        """Poll the status queue from the main thread and update GUI."""
        latest = None
        try:
            while True:
                msg = self.status_queue.get_nowait()
                if msg.get('_home_complete'):
                    self._log(f"Home {msg['axes']}: {msg['result']}",
                              "recv" if msg['success'] else "error")
                else:
                    latest = msg
        except queue.Empty:
            pass

        if latest:
            self._update_status_display(latest)

        self.root.after(50, self._poll_status_queue)

    def _update_status_display(self, s):
        """Update all status labels from a status dict."""
        state_name = STATES.get(s['state'], f"?{s['state']}")
        self.state_var.set(state_name)
        self.buffer_var.set(f"{s['buf_avail']}/{s['buf_total']}")
        self.feed_var.set(f"{s['feed_rate']} stp/s")
        self.limits_var.set(f"0x{s['limits']:02X}")
        self.home_var.set(f"0x{s['home_sw']:02X}")
        self.probe_var.set("TRIG" if s['probe'] else "Open")
        self.estop_var.set("ACTIVE" if s['estop_in'] else "Clear")

        secs = s['uptime'] // 1000
        h, m, sec = secs // 3600, (secs % 3600) // 60, secs % 60
        self.uptime_var.set(f"{h:02d}:{m:02d}:{sec:02d}")

        for i, var in enumerate(self.pos_vars):
            var.set(str(s['positions'][i]))

        # Update I/O indicators
        misc_out = s.get('misc_outputs', 0)
        misc_in = s.get('misc_inputs', 0)
        for i in range(5):
            canvas, dot = self.misc_out_indicators[i]
            color = "#44cc44" if (misc_out >> i) & 1 else "#444444"
            canvas.itemconfig(dot, fill=color)
        for i in range(5):
            canvas, dot = self.misc_in_indicators[i]
            color = "#44cc44" if (misc_in >> i) & 1 else "#444444"
            canvas.itemconfig(dot, fill=color)

        spindle_st = s.get('spindle_state', 0)
        spindle_names = {0: "OFF", 1: "CW", 2: "CCW"}
        self.spindle_var.set(spindle_names.get(spindle_st, f"?{spindle_st}"))

        coolant_st = s.get('coolant_state', 0)
        parts = []
        if coolant_st & 0x01:
            parts.append("Flood")
        if coolant_st & 0x02:
            parts.append("Mist")
        self.coolant_var.set(" + ".join(parts) if parts else "OFF")

        # Encoder
        enc_rpm = s.get('encoder_rpm', 0)
        enc_pos = s.get('encoder_position', 0)
        enc_idx = s.get('encoder_index_count', 0)
        self.enc_rpm_var.set(f"{enc_rpm} RPM")
        degrees = enc_pos * 360.0 / 65535.0 if enc_pos else 0
        self.enc_pos_var.set(f"{degrees:.1f}\u00b0")
        self.enc_idx_var.set(str(enc_idx))

        # I/O Module channels
        io_in = s.get('io_module_inputs', 0)
        io_out = s.get('io_module_outputs', 0)
        if hasattr(self, 'io_mod_in_indicators'):
            for i, (canvas, dot) in enumerate(self.io_mod_in_indicators):
                on = (io_in >> i) & 1
                canvas.itemconfig(dot, fill="#44cc44" if on else "#444444")
        if hasattr(self, 'io_mod_out_indicators'):
            for i, (canvas, dot) in enumerate(self.io_mod_out_indicators):
                on = (io_out >> i) & 1
                canvas.itemconfig(dot, fill="#ff8800" if on else "#444444")

    # =================================================================
    # UDP Send Helpers
    # =================================================================

    def _send_udp(self, packet):
        """Send a UDP packet to the ESP32."""
        if not self.connected:
            self._log("Not connected", "error")
            return False
        ip = self.ip_var.get().strip()
        try:
            self.udp_send_sock.sendto(packet, (ip, UDP_MOTION_PORT))
            return True
        except Exception as e:
            self._log(f"Send failed: {e}", "error")
            return False

    # =================================================================
    # Jog
    # =================================================================

    def _jog_start(self, axis, direction):
        speed = int(self.jog_speed_var.get())
        payload = struct.pack('<bbbxI', axis, direction, 0, speed)
        header = build_header(PKT_JOG_COMMAND, len(payload))
        packet = finalize_packet(header + payload)

        if self._send_udp(packet):
            dir_str = "+" if direction > 0 else "-"
            self._log(f"Jog {AXIS_NAMES[axis]}{dir_str} at {speed} stp/s", "send")

    def _on_jog_stop(self):
        if not self.connected:
            return
        for axis in range(MAX_AXES):
            payload = struct.pack('<Bxxx', axis)
            header = build_header(PKT_JOG_STOP, len(payload))
            packet = finalize_packet(header + payload)
            self._send_udp(packet)

    # =================================================================
    # Commands
    # =================================================================

    def _on_handshake(self):
        if self.connected:
            self._disconnect()
        self._on_connect()

    def _on_motion_test(self):
        axis_name = self.mt_axis_var.get()
        axis_idx = AXIS_NAMES.index(axis_name) if axis_name in AXIS_NAMES else 0
        step_count = int(self.mt_steps_var.get())
        speed = float(self.mt_speed_var.get())
        if speed <= 0:
            self._log("Speed must be > 0", "error")
            return

        steps = [0] * 6
        steps[axis_idx] = step_count
        duration_us = int(abs(step_count) / speed * 1000000)
        speed_sqr = int(speed * speed * 1000)
        accel = 500000

        seg_data = struct.pack('<6iIIIIHBB',
                                *steps, duration_us, speed_sqr, speed_sqr,
                                accel, 0, 0, 0)
        seg_count_data = struct.pack('<Bxxx', 1)
        payload = seg_count_data + seg_data
        header = build_header(PKT_MOTION_SEGMENT, len(payload))
        packet = finalize_packet(header + payload)

        if self._send_udp(packet):
            self._log(f"Motion test: {axis_name}={step_count} steps at {speed:.0f} stp/s "
                      f"({duration_us / 1000:.0f}ms)", "send")

    def _on_home_all(self):
        payload = struct.pack('<Bxxx', 0x3F)
        header = build_header(PKT_HOME_COMMAND, len(payload))
        packet = finalize_packet(header + payload)
        if self._send_udp(packet):
            self._log("Home all axes", "send")

    def _on_home_axis(self, axis):
        payload = struct.pack('<Bxxx', 1 << axis)
        header = build_header(PKT_HOME_COMMAND, len(payload))
        packet = finalize_packet(header + payload)
        if self._send_udp(packet):
            self._log(f"Home {AXIS_NAMES[axis]}", "send")

    def _on_feed_hold(self):
        header = build_header(PKT_FEED_HOLD, 0)
        packet = finalize_packet(header)
        if self._send_udp(packet):
            self._log("Feed hold", "send")

    def _on_feed_resume(self):
        header = build_header(PKT_FEED_RESUME, 0)
        packet = finalize_packet(header)
        if self._send_udp(packet):
            self._log("Feed resume", "send")

    def _on_estop(self):
        for _ in range(3):
            header = build_header(PKT_ESTOP, 0)
            packet = finalize_packet(header)
            self._send_udp(packet)
        self._log("E-STOP sent (3x)", "error")

    def _on_reset(self):
        header = build_header(PKT_RESET, 0)
        packet = finalize_packet(header)
        if self._send_udp(packet):
            self._log("Reset sent", "send")

    # =================================================================
    # Config Read/Write (Axis, Timing, Homing)
    # =================================================================

    def _on_axis_config_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._axis_config_read_thread, daemon=True).start()

    def _axis_config_read_thread(self):
        try:
            with self.tcp_lock:
                for i in range(MAX_AXES):
                    # Steps/mm (float)
                    vtype, vdata = send_config_get(self.tcp_sock, SPM_KEYS[i])
                    if vtype is not None and vtype == VAL_FLOAT:
                        val = struct.unpack_from('<f', bytes(vdata))[0]
                        self.root.after(0, self.axis_spm_vars[i].set, f"{val:.1f}")
                    # Max rate (uint32)
                    vtype, vdata = send_config_get(self.tcp_sock, RATE_KEYS[i])
                    if vtype is not None:
                        val = struct.unpack_from('<I', bytes(vdata))[0]
                        self.root.after(0, self.axis_rate_vars[i].set, str(val))
                    # Accel (uint32)
                    vtype, vdata = send_config_get(self.tcp_sock, ACCEL_KEYS[i])
                    if vtype is not None:
                        val = struct.unpack_from('<I', bytes(vdata))[0]
                        self.root.after(0, self.axis_accel_vars[i].set, str(val))
            self.root.after(0, self._log, "Axis config read (18 keys)", "success")
        except Exception as e:
            self.root.after(0, self._log, f"Axis config read error: {e}", "error")

    def _on_axis_config_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._axis_config_write_thread, daemon=True).start()

    def _axis_config_write_thread(self):
        try:
            with self.tcp_lock:
                for i in range(MAX_AXES):
                    spm = float(self.axis_spm_vars[i].get())
                    if not send_config_set(self.tcp_sock, SPM_KEYS[i], spm, VAL_FLOAT):
                        self.root.after(0, self._log, f"Failed to set steps/mm {AXIS_NAMES[i]}", "error")
                        return
                    rate = int(self.axis_rate_vars[i].get())
                    if not send_config_set(self.tcp_sock, RATE_KEYS[i], rate, VAL_UINT32):
                        self.root.after(0, self._log, f"Failed to set max rate {AXIS_NAMES[i]}", "error")
                        return
                    accel = int(self.axis_accel_vars[i].get())
                    if not send_config_set(self.tcp_sock, ACCEL_KEYS[i], accel, VAL_UINT32):
                        self.root.after(0, self._log, f"Failed to set accel {AXIS_NAMES[i]}", "error")
                        return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log, "Axis config written & saved to NVS", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Axis config write error: {e}", "error")

    def _on_timing_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._timing_read_thread, daemon=True).start()

    def _timing_read_thread(self):
        keys = [CFG_STEP_PULSE_US, CFG_DIR_SETUP_US, CFG_STEP_IDLE_DELAY_MS, CFG_STATUS_INTERVAL_MS]
        try:
            with self.tcp_lock:
                for idx, key in enumerate(keys):
                    vtype, vdata = send_config_get(self.tcp_sock, key)
                    if vtype is not None:
                        val = struct.unpack_from('<H', bytes(vdata))[0]
                        self.root.after(0, self.timing_vars[idx].set, str(val))
            self.root.after(0, self._log, "Timing config read", "success")
        except Exception as e:
            self.root.after(0, self._log, f"Timing read error: {e}", "error")

    def _on_timing_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._timing_write_thread, daemon=True).start()

    def _timing_write_thread(self):
        keys = [CFG_STEP_PULSE_US, CFG_DIR_SETUP_US, CFG_STEP_IDLE_DELAY_MS, CFG_STATUS_INTERVAL_MS]
        try:
            with self.tcp_lock:
                for idx, key in enumerate(keys):
                    val = int(self.timing_vars[idx].get())
                    if not send_config_set(self.tcp_sock, key, val, VAL_UINT16):
                        self.root.after(0, self._log, f"Failed to set timing key 0x{key:04X}", "error")
                        return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log, "Timing config written & saved to NVS", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Timing write error: {e}", "error")

    def _on_homing_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._homing_read_thread, daemon=True).start()

    def _homing_read_thread(self):
        keys = [CFG_HOMING_DIR_MASK, CFG_HOMING_SEEK_RATE, CFG_HOMING_FEED_RATE, CFG_HOMING_PULLOFF]
        types = [VAL_UINT8, VAL_UINT32, VAL_UINT32, VAL_UINT32]
        try:
            with self.tcp_lock:
                for idx, (key, vt) in enumerate(zip(keys, types)):
                    vtype, vdata = send_config_get(self.tcp_sock, key)
                    if vtype is not None:
                        if vt == VAL_UINT8:
                            val = vdata[0]
                            self.root.after(0, self.homing_vars[idx].set, f"0x{val:02X}")
                        else:
                            val = struct.unpack_from('<I', bytes(vdata))[0]
                            self.root.after(0, self.homing_vars[idx].set, str(val))
            self.root.after(0, self._log, "Homing config read", "success")
        except Exception as e:
            self.root.after(0, self._log, f"Homing read error: {e}", "error")

    def _on_homing_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._homing_write_thread, daemon=True).start()

    def _homing_write_thread(self):
        keys = [CFG_HOMING_DIR_MASK, CFG_HOMING_SEEK_RATE, CFG_HOMING_FEED_RATE, CFG_HOMING_PULLOFF]
        types = [VAL_UINT8, VAL_UINT32, VAL_UINT32, VAL_UINT32]
        try:
            with self.tcp_lock:
                for idx, (key, vt) in enumerate(zip(keys, types)):
                    val = int(self.homing_vars[idx].get(), 0)
                    if not send_config_set(self.tcp_sock, key, val, vt):
                        self.root.after(0, self._log, f"Failed to set homing key 0x{key:04X}", "error")
                        return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log, "Homing config written & saved to NVS", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Homing write error: {e}", "error")

    # =================================================================
    # Encoder Config Read/Write
    # =================================================================

    def _on_encoder_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._encoder_read_thread, daemon=True).start()

    def _encoder_read_thread(self):
        try:
            with self.tcp_lock:
                # PPR (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_ENCODER_PPR)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.enc_ppr_var.set, str(val))
                # Mode (uint8: 0=quadrature, 1=index-only)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_ENCODER_MODE)
                if vtype is not None:
                    mode = vdata[0]
                    self.root.after(0, self.enc_mode_var.set,
                                    "Index-only" if mode == 1 else "Quadrature")
                # Filter (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_ENCODER_FILTER_NS)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.enc_filter_var.set, str(val))
            self.root.after(0, self._log, "Encoder config read", "success")
        except Exception as e:
            self.root.after(0, self._log, f"Encoder read error: {e}", "error")

    def _on_encoder_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._encoder_write_thread, daemon=True).start()

    def _encoder_write_thread(self):
        try:
            with self.tcp_lock:
                ppr = int(self.enc_ppr_var.get())
                if not send_config_set(self.tcp_sock, CFG_ENCODER_PPR, ppr, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set encoder PPR", "error")
                    return
                mode = 1 if self.enc_mode_var.get() == "Index-only" else 0
                if not send_config_set(self.tcp_sock, CFG_ENCODER_MODE, mode, VAL_UINT8):
                    self.root.after(0, self._log, "Failed to set encoder mode", "error")
                    return
                filt = int(self.enc_filter_var.get())
                if not send_config_set(self.tcp_sock, CFG_ENCODER_FILTER_NS, filt, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set encoder filter", "error")
                    return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log,
                                    "Encoder config saved (reboot to apply)", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Encoder write error: {e}", "error")

    # =================================================================
    # I/O Module Config Read/Write
    # =================================================================

    def _on_iomodule_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._iomodule_read_thread, daemon=True).start()

    def _iomodule_read_thread(self):
        try:
            with self.tcp_lock:
                # Device mode (uint8)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_DEVICE_MODE)
                if vtype is not None:
                    mode = vdata[0]
                    self.root.after(0, self.dev_mode_var.set,
                                    "I/O Module" if mode == 1 else "Motion Controller")
                # Channel count (uint8)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_IO_PIN_COUNT)
                if vtype is not None:
                    self.root.after(0, self.io_ch_count_var.set, str(vdata[0]))
                # Dir mask (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_IO_DIR_MASK)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.io_dir_var.set, f"{val:04X}")
                # Pullup mask (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_IO_PULLUP_MASK)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.io_pull_var.set, f"{val:04X}")
                # Invert mask (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_IO_INVERT_MASK)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.io_inv_var.set, f"{val:04X}")
                # Per-channel pin assignments (uint8 each)
                for ch in range(16):
                    vtype, vdata = send_config_get(self.tcp_sock, CFG_IO_PIN_BASE + ch)
                    if vtype is not None:
                        self.root.after(0, self.io_pin_vars[ch].set, str(vdata[0]))
            self.root.after(0, self._log, "I/O module config read (21 keys)", "success")
        except Exception as e:
            self.root.after(0, self._log, f"I/O module read error: {e}", "error")

    def _on_iomodule_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._iomodule_write_thread, daemon=True).start()

    def _iomodule_write_thread(self):
        try:
            with self.tcp_lock:
                mode = 1 if self.dev_mode_var.get() == "I/O Module" else 0
                if not send_config_set(self.tcp_sock, CFG_DEVICE_MODE, mode, VAL_UINT8):
                    self.root.after(0, self._log, "Failed to set device mode", "error")
                    return
                ch_count = int(self.io_ch_count_var.get())
                if not send_config_set(self.tcp_sock, CFG_IO_PIN_COUNT, ch_count, VAL_UINT8):
                    self.root.after(0, self._log, "Failed to set I/O channel count", "error")
                    return
                dir_mask = int(self.io_dir_var.get(), 16)
                if not send_config_set(self.tcp_sock, CFG_IO_DIR_MASK, dir_mask, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set I/O dir mask", "error")
                    return
                pull_mask = int(self.io_pull_var.get(), 16)
                if not send_config_set(self.tcp_sock, CFG_IO_PULLUP_MASK, pull_mask, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set I/O pullup mask", "error")
                    return
                inv_mask = int(self.io_inv_var.get(), 16)
                if not send_config_set(self.tcp_sock, CFG_IO_INVERT_MASK, inv_mask, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set I/O invert mask", "error")
                    return
                # Per-channel pin assignments
                for ch in range(16):
                    val = int(self.io_pin_vars[ch].get())
                    if not send_config_set(self.tcp_sock, CFG_IO_PIN_BASE + ch, val, VAL_UINT8):
                        self.root.after(0, self._log,
                                       f"Failed to set I/O pin {ch}", "error")
                        return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log,
                                    "I/O module config saved (reboot to apply)", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"I/O module write error: {e}", "error")

    # =================================================================
    # I/O Control (UDP IO_CONTROL packets)
    # =================================================================

    def _toggle_misc_output(self, idx):
        """Toggle a misc output bit and send IO_CONTROL packet."""
        self.misc_out_toggle_state[idx] = not self.misc_out_toggle_state[idx]
        self._send_io_control()

    def _toggle_coolant(self, bit):
        """Toggle a coolant bit (0=flood, 1=mist) and send IO_CONTROL packet."""
        self._io_coolant_state ^= (1 << bit)
        self._send_io_control(coolant=self._io_coolant_state)

    def _send_io_control(self, spindle=None, coolant=None):
        """Build and send a WCNC_PKT_IO_CONTROL packet via UDP."""
        if not self.connected:
            self._log("Not connected", "error")
            return

        # Build misc output bitmask from toggle state
        misc_out = 0
        for i in range(5):
            if self.misc_out_toggle_state[i]:
                misc_out |= (1 << i)

        if spindle is not None:
            self._io_spindle_state = spindle
        if coolant is not None:
            self._io_coolant_state = coolant

        payload = struct.pack('<BBHBxxx',
                              misc_out,
                              self._io_spindle_state,
                              0,  # spindle_rpm (TODO: add RPM control)
                              self._io_coolant_state)
        header = build_header(PKT_IO_CONTROL, len(payload))
        packet = finalize_packet(header + payload)
        if self._send_udp(packet):
            parts = []
            parts.append(f"out=0x{misc_out:02X}")
            spn = {0: "OFF", 1: "CW", 2: "CCW"}.get(self._io_spindle_state, "?")
            parts.append(f"spindle={spn}")
            cool = []
            if self._io_coolant_state & 1:
                cool.append("flood")
            if self._io_coolant_state & 2:
                cool.append("mist")
            parts.append(f"coolant={'|'.join(cool) if cool else 'off'}")
            self._log(f"IO Control: {', '.join(parts)}", "send")

    # =================================================================
    # I/O Config
    # =================================================================

    def _on_io_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._io_read_thread, daemon=True).start()

    def _io_read_thread(self):
        try:
            with self.tcp_lock:
                for key, var in self.io_vars.items():
                    vtype, vdata = send_config_get(self.tcp_sock, key)
                    if vtype is not None:
                        val = vdata[0]
                        self.root.after(0, var.set, f"0x{val:02X}")
            self.root.after(0, self._log, "I/O config read", "success")
        except Exception as e:
            self.root.after(0, self._log, f"I/O read error: {e}", "error")

    def _on_io_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._io_write_thread, daemon=True).start()

    def _io_write_thread(self):
        try:
            with self.tcp_lock:
                for key, var in self.io_vars.items():
                    val = int(var.get(), 0)
                    if not send_config_set(self.tcp_sock, key, val, VAL_UINT8):
                        self.root.after(0, self._log,
                                       f"Failed to set config key 0x{key:04X}", "error")
                        return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log,
                                   "I/O config saved to NVS", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"I/O write error: {e}", "error")

    # =================================================================
    # Output Config (charge pump, spindle PWM)
    # =================================================================

    def _on_output_config_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._output_config_read_thread, daemon=True).start()

    def _output_config_read_thread(self):
        try:
            with self.tcp_lock:
                # Charge pump freq (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_CHARGE_PUMP_FREQ)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.cp_freq_var.set, str(val))
                # Spindle PWM freq (uint16)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_SPINDLE_PWM_FREQ)
                if vtype is not None:
                    val = struct.unpack_from('<H', bytes(vdata))[0]
                    self.root.after(0, self.sp_freq_var.set, str(val))
                # Spindle max RPM (uint32)
                vtype, vdata = send_config_get(self.tcp_sock, CFG_SPINDLE_MAX_RPM)
                if vtype is not None:
                    val = struct.unpack_from('<I', bytes(vdata))[0]
                    self.root.after(0, self.sp_rpm_var.set, str(val))
            self.root.after(0, self._log, "Output config read", "success")
        except Exception as e:
            self.root.after(0, self._log, f"Output config read error: {e}", "error")

    def _on_output_config_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._output_config_write_thread, daemon=True).start()

    def _output_config_write_thread(self):
        try:
            with self.tcp_lock:
                cp_freq = int(self.cp_freq_var.get())
                if not send_config_set(self.tcp_sock, CFG_CHARGE_PUMP_FREQ, cp_freq, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set charge pump freq", "error")
                    return
                sp_freq = int(self.sp_freq_var.get())
                if not send_config_set(self.tcp_sock, CFG_SPINDLE_PWM_FREQ, sp_freq, VAL_UINT16):
                    self.root.after(0, self._log, "Failed to set spindle PWM freq", "error")
                    return
                sp_rpm = int(self.sp_rpm_var.get())
                if not send_config_set(self.tcp_sock, CFG_SPINDLE_MAX_RPM, sp_rpm, VAL_UINT32):
                    self.root.after(0, self._log, "Failed to set spindle max RPM", "error")
                    return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log, "Output config written & saved to NVS", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Output config write error: {e}", "error")

    # =================================================================
    # Pin Config Read/Write
    # =================================================================

    def _on_pin_config_read(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._pin_config_read_thread, daemon=True).start()

    def _pin_config_read_thread(self):
        try:
            with self.tcp_lock:
                for i in range(MAX_AXES):
                    # Step pin
                    vtype, vdata = send_config_get(self.tcp_sock, PIN_STEP_KEYS[i])
                    if vtype is not None:
                        self.root.after(0, self.pin_step_vars[i].set, str(vdata[0]))
                    # Dir pin
                    vtype, vdata = send_config_get(self.tcp_sock, PIN_DIR_KEYS[i])
                    if vtype is not None:
                        self.root.after(0, self.pin_dir_vars[i].set, str(vdata[0]))
                    # Limit pin
                    vtype, vdata = send_config_get(self.tcp_sock, PIN_LIMIT_KEYS[i])
                    if vtype is not None:
                        self.root.after(0, self.pin_limit_vars[i].set, str(vdata[0]))
                # Single pins
                for label, key in PIN_SINGLE_KEYS:
                    vtype, vdata = send_config_get(self.tcp_sock, key)
                    if vtype is not None and key in self.pin_single_vars:
                        self.root.after(0, self.pin_single_vars[key].set, str(vdata[0]))
            self.root.after(0, self._log, "Pin config read (26 keys)", "success")
        except Exception as e:
            self.root.after(0, self._log, f"Pin config read error: {e}", "error")

    def _on_pin_config_write(self):
        if not self.connected or not self.tcp_sock:
            self._log("Not connected", "error")
            return
        threading.Thread(target=self._pin_config_write_thread, daemon=True).start()

    def _pin_config_write_thread(self):
        try:
            with self.tcp_lock:
                for i in range(MAX_AXES):
                    val = int(self.pin_step_vars[i].get())
                    if not send_config_set(self.tcp_sock, PIN_STEP_KEYS[i], val, VAL_UINT8):
                        self.root.after(0, self._log,
                                       f"Failed to set Step {AXIS_NAMES[i]} pin", "error")
                        return
                    val = int(self.pin_dir_vars[i].get())
                    if not send_config_set(self.tcp_sock, PIN_DIR_KEYS[i], val, VAL_UINT8):
                        self.root.after(0, self._log,
                                       f"Failed to set Dir {AXIS_NAMES[i]} pin", "error")
                        return
                    val = int(self.pin_limit_vars[i].get())
                    if not send_config_set(self.tcp_sock, PIN_LIMIT_KEYS[i], val, VAL_UINT8):
                        self.root.after(0, self._log,
                                       f"Failed to set Limit {AXIS_NAMES[i]} pin", "error")
                        return
                for label, key in PIN_SINGLE_KEYS:
                    if key in self.pin_single_vars:
                        val = int(self.pin_single_vars[key].get())
                        if not send_config_set(self.tcp_sock, key, val, VAL_UINT8):
                            self.root.after(0, self._log,
                                           f"Failed to set {label} pin", "error")
                            return
                if send_config_save(self.tcp_sock):
                    self.root.after(0, self._log,
                                   "Pin config saved to NVS (reboot to apply)", "success")
                else:
                    self.root.after(0, self._log, "Config save failed", "error")
        except Exception as e:
            self.root.after(0, self._log, f"Pin config write error: {e}", "error")

    # =================================================================
    # WiFi Setup
    # =================================================================

    def _toggle_password_visibility(self):
        self.wifi_pass_entry.configure(show="" if self.wifi_show_var.get() else "*")

    def _on_wifi_setup(self):
        ssid = self.wifi_ssid_var.get().strip()
        password = self.wifi_pass_var.get()
        if not ssid:
            self._log("Enter a WiFi SSID", "error")
            return
        ip = self.ip_var.get().strip()
        if not ip:
            self._log("Enter an IP address first", "error")
            return

        self.wifi_apply_btn.configure(state="disabled")
        threading.Thread(target=self._wifi_setup_thread,
                         args=(ip, ssid, password), daemon=True).start()

    def _wifi_setup_thread(self, ip, ssid, password):
        """Open a fresh TCP connection for config to avoid keepalive timeout."""
        sock = None
        try:
            self.root.after(0, self._log, "Opening config connection...", "send")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((ip, TCP_CONTROL_PORT))

            host_name = b"WiFiSetup\x00" + b'\x00' * 23
            payload = struct.pack('<I', 0x01000000) + host_name
            header = build_header(PKT_HANDSHAKE_REQ, len(payload))
            packet = finalize_packet(header + payload)
            tcp_send(sock, packet)
            resp = tcp_recv(sock)
            if not validate_packet(resp):
                self.root.after(0, self._log, "Handshake failed", "error")
                sock.close()
                self.root.after(0, self.wifi_apply_btn.configure, {"state": "normal"})
                return

            self.root.after(0, self._log, f"Setting WiFi SSID: '{ssid}'", "send")
            if not send_config_set(sock, CFG_WIFI_SSID, ssid):
                self.root.after(0, self._log, "Failed to set SSID", "error")
                sock.close()
                self.root.after(0, self.wifi_apply_btn.configure, {"state": "normal"})
                return

            self.root.after(0, self._log,
                            f"Setting WiFi password: {'*' * len(password)}", "send")
            if not send_config_set(sock, CFG_WIFI_PASSWORD, password):
                self.root.after(0, self._log, "Failed to set password", "error")
                sock.close()
                self.root.after(0, self.wifi_apply_btn.configure, {"state": "normal"})
                return

            self.root.after(0, self._log, "Saving config...", "send")
            if send_config_save(sock):
                self.root.after(0, self._log,
                                "Config saved! ESP32 rebooting to join your network...",
                                "success")
                self.root.after(1500, self._disconnect)
            else:
                self.root.after(0, self._log, "Config save failed", "error")

            sock.close()

        except Exception as e:
            self.root.after(0, self._log, f"WiFi setup error: {e}", "error")
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

        self.root.after(0, self.wifi_apply_btn.configure, {"state": "normal"})

    # =================================================================
    # Cleanup
    # =================================================================

    def _on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = WiFiCNCTester(root)
    root.mainloop()


if __name__ == '__main__':
    main()
