#!/usr/bin/env python3
"""
is27_gui.py: desktop dashboard for the IS27 robot car.

Connects to the MQTT broker, decodes the car's binary frames
(comms_codec.py) and sorts them into one dataset per topic:

  Overview   mode, uptime, fault lamps, live values, event log, commands
  Barcode map  A=L B=R C=U D=S box: sent retained on robot/map, the car
             confirms the map it is using on robot/tlm/map
  Motion     wheel speeds + distance (robot/tlm/motion)
  Line       line position, sensor mode, barcodes (robot/tlm/line)
  Terrain    pitch, roll, compass, humps (robot/tlm/terrain)
  Obstacle   top view of the last obstacle + every sweep (robot/tlm/obstacle)
  Faults     every fault event (robot/event/fault) and status fault changes
  Resources  CPU load per core, control-loop times, kernel objects in use,
             message queue fill (robot/tlm/resources)
  Raw        every frame as hex + decoded, with a topic filter

Each tab has Export CSV.  Charts show the last 30 s .. 5 min.

Run:
    pip install paho-mqtt
    python3 is27_gui.py                         (type the broker IP in the window)
    python3 is27_gui.py --broker 192.168.1.10
    python3 is27_gui.py --demo                  (no car needed: simulated data)

Only Python's built-in tkinter is used for the window and charts.
"""
import argparse
import csv
import math
import queue
import random
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from comms_codec import (DECODERS, CMD_CODES, FAULT_NAMES, encode, encode_cmd, encode_tune,
                         parse_map_text, encode_map, decode_map, map_to_text, MAP_LETTERS)

try:
    import paho.mqtt.client as mqtt
except ImportError:                     # the demo still works without it
    mqtt = None

WINDOWS = {"30 s": 30, "1 min": 60, "2 min": 120, "5 min": 300}
MAX_ROWS = 20000                        # per dataset
TABLE_ROWS = 300                        # rows kept visible in each table

# colours
BG = "#f6f7f9"
CARD = "#ffffff"
INK = "#1f2933"
MUTED = "#6b7785"
GRID = "#e3e7eb"
C1, C2, C3, C4 = "#2563eb", "#ea580c", "#16a34a", "#9333ea"
RED, AMBER, GREEN, GREY = "#dc2626", "#d97706", "#16a34a", "#c3cad2"


# ---------------------------------------------------------------- data
class Dataset:
    def __init__(self, name, columns):
        self.name = name
        self.columns = columns          # keys shown in the table / CSV
        self.rows = []
        self.added = 0                  # rows added since the table last drew

    def add(self, row):
        self.rows.append(row)
        self.added += 1
        if len(self.rows) > MAX_ROWS:
            del self.rows[:len(self.rows) - MAX_ROWS]

    def clear(self):
        self.rows.clear()
        self.added = 0

    def export(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["pc_time"] + self.columns)
            for r in self.rows:
                w.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r["t"])) +
                            ".%03d" % int((r["t"] % 1) * 1000)] + [r.get(c, "") for c in self.columns])


def enc_text(d):
    """which wheel encoder is unhealthy (motion frame bits enc_l / enc_r)"""
    if d.get("enc_ok"):
        return "ok"
    l, r = d.get("enc_l", 0), d.get("enc_r", 0)
    return "LEFT FAULT" if (r and not l) else "RIGHT FAULT" if (l and not r) else "BOTH FAULT"


def fmt_time(t):
    return time.strftime("%H:%M:%S", time.localtime(t))


# ---------------------------------------------------------------- widgets
class Chart(tk.Canvas):
    """Simple time chart on a Canvas: several series, auto or fixed y range."""

    def __init__(self, master, title, series, unit="", yfixed=None, height=210, ymin0=False):
        super().__init__(master, height=height, bg=CARD, highlightthickness=1,
                         highlightbackground=GRID)
        self.title = title + (" (%s)" % unit.strip() if unit.strip() else "")
        self.series, self.yfixed, self.ymin0 = series, yfixed, ymin0

    def draw(self, rows, window, now):
        self.delete("all")
        w, h = max(self.winfo_width(), 200), max(self.winfo_height(), 120)
        L, R, T, B = 56, 12, 28, 24
        pw, ph = w - L - R, h - T - B
        self.create_text(L, 6, anchor="nw", text=self.title, fill=INK, font=("Segoe UI", 10, "bold"))
        x = L + self.measure(self.title) + 16
        for key, label, col in self.series:
            self.create_rectangle(x, 10, x + 14, 14, fill=col, outline=col)
            self.create_text(x + 18, 6, anchor="nw", text=label, fill=MUTED, font=("Segoe UI", 9))
            x += 26 + self.measure(label)

        t0 = now - window
        pts = [r for r in rows if r["t"] >= t0]
        vals = [r[k] for r in pts for k, _, _ in self.series if r.get(k) is not None]
        if self.yfixed:
            lo, hi = self.yfixed
        elif vals:
            lo, hi = min(vals), max(vals)
            if hi - lo < 1e-6:
                lo, hi = lo - 1, hi + 1
            pad = (hi - lo) * 0.1
            lo, hi = lo - pad, hi + pad
            if self.ymin0 and lo < 0 <= min(vals):
                lo = 0
        else:
            lo, hi = 0, 1

        def X(t):
            return L + (t - t0) / window * pw

        def Y(v):
            return T + ph - (v - lo) / (hi - lo) * ph

        for i in range(6):                              # grid + y labels
            v = lo + (hi - lo) * i / 5
            y = Y(v)
            self.create_line(L, y, L + pw, y, fill=GRID)
            self.create_text(L - 6, y, anchor="e", text=self.num(v), fill=MUTED, font=("Segoe UI", 8))
        for i in range(5):                              # x labels
            t = t0 + window * i / 4
            self.create_text(X(t), T + ph + 4, anchor="n", fill=MUTED, font=("Segoe UI", 8),
                             text="now" if i == 4 else "-%ds" % round(now - t))
        if lo < 0 < hi:
            self.create_line(L, Y(0), L + pw, Y(0), fill="#b8c1cb")
        self.create_rectangle(L, T, L + pw, T + ph, outline=GRID)

        for key, _, col in self.series:
            xy = []
            for r in pts:
                v = r.get(key)
                if v is not None:
                    xy += [X(r["t"]), Y(max(lo, min(hi, v)))]
            if len(xy) >= 4:
                self.create_line(*xy, fill=col, width=2)
            elif len(xy) == 2:
                self.create_oval(xy[0] - 2, xy[1] - 2, xy[0] + 2, xy[1] + 2, fill=col, outline=col)
        if not pts:
            self.create_text(L + pw / 2, T + ph / 2, text="no data in this window", fill=MUTED)

    @staticmethod
    def num(v):
        return "%d" % round(v) if abs(v) >= 10 or v == int(v) else "%.1f" % v

    def measure(self, text):
        return 7 * len(text)


class Table(ttk.Frame):
    """Treeview showing a dataset, newest row on top."""

    def __init__(self, master, dataset, headings, widths=None, height=10):
        super().__init__(master)
        self.ds = dataset
        cols = ["time"] + dataset.columns
        self.tv = ttk.Treeview(self, columns=cols, show="headings", height=height)
        for i, c in enumerate(cols):
            self.tv.heading(c, text=(["Time"] + headings)[i])
            self.tv.column(c, width=(widths[i] if widths else 90), anchor="center", stretch=True)
        sb = ttk.Scrollbar(self, orient="vertical", command=self.tv.yview)
        self.tv.configure(yscrollcommand=sb.set)
        self.tv.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    def refresh(self):
        n = self.ds.added
        if n == 0:
            return
        self.ds.added = 0
        for r in self.ds.rows[-min(n, TABLE_ROWS):]:
            self.tv.insert("", 0, values=[fmt_time(r["t"])] + [r.get(c, "") for c in self.ds.columns])
        kids = self.tv.get_children()
        for k in kids[TABLE_ROWS:]:
            self.tv.delete(k)

    def reset(self):
        self.tv.delete(*self.tv.get_children())


# ---------------------------------------------------------------- app
class App:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.q = queue.Queue()
        self.client = None
        self.demo = None
        self.cmd_id = 0
        self.msgs = 0
        self.rate = 0.0
        self.last_rx = 0
        self.last = {}                  # latest decoded value per topic
        self.prev_mode = None
        self.prev_bc = None
        self.map_seen = False           # broker's retained robot/map shown in the box yet?
        self.car_map = None             # last robot/tlm/map from the car
        self.prev_hump_ev = 0
        self.prev_faults = 0
        self.last_sweep = None

        self.ds = {
            "motion": Dataset("motion", ["vl", "vr", "dist_cm", "owner", "enc_ok"]),
            "line": Dataset("line", ["mode", "pos", "sensors", "bc_char", "bc_action", "speed_mms"]),
            "barcode": Dataset("barcode", ["text", "action"]),
            "terrain": Dataset("terrain", ["pitch", "roll", "heading", "hump_mm", "events"]),
            "hump": Dataset("hump", ["height_mm"]),
            "obstacle": Dataset("obstacle", ["stage", "dist_cm", "bearing_deg", "width_cm", "need_left_cm",
                                             "need_right_cm", "action"]),
            "resources": Dataset("resources", ["cpu0", "cpu1", "motion_us", "line_us", "tasks", "flags",
                                               "sems", "mbfs", "cycs", "queue_pct", "drops"]),
            "fault": Dataset("fault", ["source", "faults"]),
            "status": Dataset("status", ["seq", "up_s", "mode", "faults", "tx_drop"]),
            "raw": Dataset("raw", ["topic", "bytes", "hex", "decoded"]),
            "event": Dataset("event", ["what"]),
        }
        self.build()
        if args.demo:
            self.start_demo()
        elif args.broker:
            self.connect()
        self.root.after(100, self.pump)
        self.root.after(500, self.redraw)

    # ------------------------------------------------ layout
    def build(self):
        r = self.root
        r.title("IS27 car dashboard")
        r.geometry("1340x800")
        r.configure(bg=BG)
        st = ttk.Style()
        st.theme_use("clam")
        st.configure(".", background=BG, foreground=INK, font=("Segoe UI", 10))
        st.configure("Card.TFrame", background=CARD)
        st.configure("Card.TLabel", background=CARD)
        st.configure("Muted.TLabel", background=CARD, foreground=MUTED, font=("Segoe UI", 9))
        st.configure("Big.TLabel", background=CARD, font=("Segoe UI", 20, "bold"))
        st.configure("Val.TLabel", background=CARD, font=("Segoe UI", 13, "bold"))
        st.configure("TNotebook.Tab", padding=(14, 6))
        st.configure("Treeview", rowheight=22)

        bar = ttk.Frame(r, padding=(10, 8))
        bar.pack(fill="x")
        ttk.Label(bar, text="Broker").pack(side="left")
        self.e_broker = ttk.Entry(bar, width=16)
        self.e_broker.insert(0, self.args.broker or "")
        self.e_broker.pack(side="left", padx=(4, 10))
        ttk.Label(bar, text="Port").pack(side="left")
        self.e_port = ttk.Entry(bar, width=6)
        self.e_port.insert(0, str(self.args.port))
        self.e_port.pack(side="left", padx=(4, 10))
        ttk.Label(bar, text="Topic root").pack(side="left")
        self.e_root = ttk.Entry(bar, width=14)
        self.e_root.insert(0, self.args.root)
        self.e_root.pack(side="left", padx=(4, 10))
        self.b_conn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.b_conn.pack(side="left")
        ttk.Button(bar, text="Demo data", command=self.toggle_demo).pack(side="left", padx=6)
        self.lamp = tk.Canvas(bar, width=14, height=14, bg=BG, highlightthickness=0)
        self.lamp.pack(side="left", padx=(10, 4))
        self.l_conn = ttk.Label(bar, text="not connected", foreground=MUTED)
        self.l_conn.pack(side="left")

        self.cb_win = ttk.Combobox(bar, values=list(WINDOWS), width=8, state="readonly")
        self.cb_win.set("1 min")
        self.cb_win.pack(side="right")
        ttk.Label(bar, text="Chart window").pack(side="right", padx=(10, 4))
        self.v_freeze = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Freeze charts", variable=self.v_freeze).pack(side="right", padx=10)

        cmd = ttk.Frame(r, padding=(10, 0, 10, 6))
        cmd.pack(fill="x")
        ttk.Label(cmd, text="Send to car:").pack(side="left")
        self.cmd_buttons = []
        for name, label in [("START", "START"), ("STOP", "STOP"), ("PAUSE", "PAUSE"), ("RESUME", "RESUME"),
                            ("UTURN", "U-TURN"), ("REVERSE", "REVERSE"), ("PING", "PING")]:
            b = ttk.Button(cmd, text=label, width=8, command=lambda n=name: self.send(n))
            b.pack(side="left", padx=3)
            self.cmd_buttons.append(b)
        tune = ttk.Frame(r, padding=(10, 0, 10, 6))
        tune.pack(fill="x")
        ttk.Label(tune, text="Line tuning (live):").pack(side="left")
        self.v_speed = tk.IntVar(value=30)
        self.v_bend = tk.IntVar(value=50)
        self.v_kp = tk.IntVar(value=1000)
        for label, var, lo, hi, step, unit in [("speed", self.v_speed, 30, 300, 10, "mm/s"),
                                               ("bends at", self.v_bend, 20, 100, 5, "% of speed"),
                                               ("steering KP", self.v_kp, 300, 3000, 100, "")]:
            ttk.Label(tune, text="   " + label).pack(side="left")
            ttk.Spinbox(tune, from_=lo, to=hi, increment=step, width=5, textvariable=var).pack(side="left", padx=3)
            if unit:
                ttk.Label(tune, text=unit).pack(side="left")
        self.v_pivot = tk.BooleanVar(value=False)
        ttk.Checkbutton(tune, text="stop + pivot at bends", variable=self.v_pivot).pack(side="left", padx=(10, 0))
        self.b_speed = ttk.Button(tune, text="Apply", command=self.send_speed)
        self.b_speed.pack(side="left", padx=8)
        self.cmd_buttons.append(self.b_speed)
        ttk.Label(tune, text="(car resets these on restart)", foreground=MUTED).pack(side="left")

        # barcode letter -> action.  Retained on the broker, so the car gets
        # it again whenever it (re)connects.  Letters not named go straight.
        bmap = ttk.Frame(r, padding=(10, 0, 10, 6))
        bmap.pack(fill="x")
        ttk.Label(bmap, text="Barcode map:").pack(side="left")
        self.e_map = ttk.Entry(bmap, width=34)
        self.e_map.insert(0, "A=L B=R C=U D=S")
        self.e_map.pack(side="left", padx=(4, 4))
        self.e_map.bind("<Return>", lambda e: self.send_map())
        self.b_map = ttk.Button(bmap, text="Send map", command=self.send_map)
        self.b_map.pack(side="left", padx=3)
        self.b_map_clear = ttk.Button(bmap, text="Clear map", command=lambda: self.send_map(clear=True))
        self.b_map_clear.pack(side="left", padx=3)
        self.cmd_buttons += [self.b_map, self.b_map_clear]
        ttk.Label(bmap, text="L left  R right  U U-turn  S stop  (others: straight)",
                  foreground=MUTED).pack(side="left", padx=(8, 0))
        self.l_map = ttk.Label(bmap, text="Car is using: (not heard yet)", foreground=MUTED)
        self.l_map.pack(side="right")
        ttk.Button(cmd, text="Export all CSV", command=self.export_all).pack(side="right")
        ttk.Button(cmd, text="Clear all", command=self.clear_all).pack(side="right", padx=6)

        nb = ttk.Notebook(r)
        nb.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.nb = nb
        self.build_overview(nb)
        self.build_motion(nb)
        self.build_line(nb)
        self.build_terrain(nb)
        self.build_obstacle(nb)
        self.build_faults(nb)
        self.build_resources(nb)
        self.build_raw(nb)
        self.set_conn(False, "not connected")

    def tab(self, nb, title):
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text=title)
        return f

    def card(self, parent, **pack):
        c = ttk.Frame(parent, style="Card.TFrame", padding=10)
        c.pack(**pack)
        return c

    def tools(self, parent, names):
        t = ttk.Frame(parent)
        t.pack(fill="x", pady=(0, 6))
        for n in names:
            ttk.Button(t, text="Export %s CSV" % n, command=lambda n=n: self.export(n)).pack(side="right", padx=3)
        return t

    def build_overview(self, nb):
        f = self.tab(nb, "Overview")
        top = ttk.Frame(f)
        top.pack(fill="x")
        self.ov = {}
        for key, label in [("mode", "Mode"), ("up", "Uptime"), ("rate", "Messages / s"), ("age", "Last status")]:
            c = self.card(top, side="left", fill="x", expand=True, padx=(0, 8))
            ttk.Label(c, text=label, style="Muted.TLabel").pack(anchor="w")
            self.ov[key] = ttk.Label(c, text="-", style="Big.TLabel")
            self.ov[key].pack(anchor="w")

        mid = ttk.Frame(f)
        mid.pack(fill="both", expand=True, pady=8)
        left = ttk.Frame(mid)
        left.pack(side="left", fill="both", expand=True)

        fc = self.card(left, fill="x")
        ttk.Label(fc, text="Faults (shared bitmask)", style="Muted.TLabel").grid(row=0, column=0, columnspan=6, sticky="w")
        self.fault_lamps = []
        for i, n in enumerate(FAULT_NAMES[:11]):
            cv = tk.Canvas(fc, width=14, height=14, bg=CARD, highlightthickness=0)
            cv.grid(row=1 + i // 4, column=(i % 4) * 2, padx=(0, 4), pady=3, sticky="w")
            ttk.Label(fc, text=n, style="Card.TLabel").grid(row=1 + i // 4, column=(i % 4) * 2 + 1, sticky="w", padx=(0, 18))
            self.fault_lamps.append(cv)

        vc = self.card(left, fill="both", expand=True, pady=(8, 0))
        ttk.Label(vc, text="Live values", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        self.live = {}
        names = [("speed", "Wheel speed L / R"), ("dist", "Distance"), ("owner", "Driving"),
                 ("line", "Line"), ("barcode", "Last barcode"), ("tilt", "Pitch / roll"),
                 ("heading", "Heading"), ("humps", "Humps"), ("obstacle", "Last obstacle"),
                 ("cpu", "CPU core 0 / 1"), ("lspeed", "Line tuning on the car")]
        for i, (k, label) in enumerate(names):
            ttk.Label(vc, text=label, style="Muted.TLabel").grid(row=1 + i // 3 * 2, column=i % 3, sticky="w", padx=(0, 30), pady=(8, 0))
            self.live[k] = ttk.Label(vc, text="-", style="Val.TLabel")
            self.live[k].grid(row=2 + i // 3 * 2, column=i % 3, sticky="w", padx=(0, 30))

        right = self.card(mid, side="left", fill="both", padx=(8, 0))
        ttk.Label(right, text="Event log", style="Muted.TLabel").pack(anchor="w")
        self.log = tk.Listbox(right, width=52, height=24, bd=0, highlightthickness=0, bg=CARD,
                              fg=INK, font=("Consolas", 9), activestyle="none")
        self.log.pack(fill="both", expand=True)
        ttk.Button(right, text="Export events CSV", command=lambda: self.export("event")).pack(anchor="e", pady=(6, 0))

    def build_motion(self, nb):
        f = self.tab(nb, "Motion")
        self.tools(f, ["motion"])
        self.ch_speed = Chart(f, "Wheel speed", [("vl", "left", C1), ("vr", "right", C2)], " mm/s")
        self.ch_speed.pack(fill="x")
        self.ch_dist = Chart(f, "Distance driven", [("dist_cm", "distance", C3)], " cm", height=150, ymin0=True)
        self.ch_dist.pack(fill="x", pady=8)
        self.t_motion = Table(f, self.ds["motion"], ["Left mm/s", "Right mm/s", "Distance cm", "Driving", "Encoders"])
        self.t_motion.pack(fill="both", expand=True)

    def build_line(self, nb):
        f = self.tab(nb, "Line + barcode")
        self.tools(f, ["line", "barcode"])
        self.ch_line = Chart(f, "Line position (- = under left sensor, + = right)", [("pos", "position", C1)],
                             "", yfixed=(-1000, 1000))
        self.ch_line.pack(fill="x")
        low = ttk.Frame(f)
        low.pack(fill="both", expand=True, pady=8)
        self.t_line = Table(low, self.ds["line"], ["Mode", "Position", "Sensors", "Barcode", "Action", "Speed mm/s"])
        self.t_line.pack(side="left", fill="both", expand=True)
        bc = ttk.Frame(low)
        bc.pack(side="left", fill="both", padx=(8, 0))
        ttk.Label(bc, text="Barcodes read").pack(anchor="w")
        self.t_bc = Table(bc, self.ds["barcode"], ["Text", "Action"], widths=[80, 80, 90])
        self.t_bc.pack(fill="both", expand=True)

    def build_terrain(self, nb):
        f = self.tab(nb, "Terrain (IMU)")
        self.tools(f, ["terrain", "hump"])
        row = ttk.Frame(f)
        row.pack(fill="x")
        self.ch_tilt = Chart(row, "Pitch and roll", [("pitch", "pitch (nose up +)", C1), ("roll", "roll", C4)], " deg")
        self.ch_tilt.pack(side="left", fill="x", expand=True)
        self.compass = tk.Canvas(row, width=210, height=210, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        self.compass.pack(side="left", padx=(8, 0))
        low = ttk.Frame(f)
        low.pack(fill="both", expand=True, pady=8)
        self.t_terrain = Table(low, self.ds["terrain"], ["Pitch deg", "Roll deg", "Heading", "Last hump mm", "Events"])
        self.t_terrain.pack(side="left", fill="both", expand=True)
        hb = ttk.Frame(low)
        hb.pack(side="left", fill="both", padx=(8, 0))
        ttk.Label(hb, text="Humps crossed").pack(anchor="w")
        self.t_hump = Table(hb, self.ds["hump"], ["Height mm"], widths=[80, 90])
        self.t_hump.pack(fill="both", expand=True)

    def build_obstacle(self, nb):
        f = self.tab(nb, "Obstacle")
        self.tools(f, ["obstacle"])
        row = ttk.Frame(f)
        row.pack(fill="both", expand=True)
        self.topview = tk.Canvas(row, width=440, height=440, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        self.topview.pack(side="left", fill="y")
        self.t_obs = Table(row, self.ds["obstacle"], ["Step", "Dist cm", "Bearing deg", "Width cm", "Move L cm",
                                                      "Move R cm", "Went"],
                           widths=[70, 100, 60, 80, 70, 75, 75, 70], height=18)
        self.t_obs.pack(side="left", fill="both", expand=True, padx=(8, 0))

    def build_faults(self, nb):
        f = self.tab(nb, "Faults")
        self.tools(f, ["fault", "status"])
        ttk.Label(f, text="Fault events (robot/event/fault) and changes seen in robot/status").pack(anchor="w")
        self.t_fault = Table(f, self.ds["fault"], ["Source", "Faults"], widths=[90, 140, 700], height=12)
        self.t_fault.pack(fill="both", expand=True)
        ttk.Label(f, text="Status frames (1 per second)").pack(anchor="w", pady=(8, 0))
        self.t_status = Table(f, self.ds["status"], ["Seq", "Uptime s", "Mode", "Faults", "TX drop"], height=8)
        self.t_status.pack(fill="both", expand=True)

    TASKS = [("motion", 10, "0", "motors, encoders, 20 ms"), ("mission", 11, "1", "buttons, modes, avoidance"),
             ("line", 12, "1", "IR sensors, 20 ms"), ("ranger", 13, "1", "ultrasonic + servo, 60 ms"),
             ("imu", 14, "1", "GY-511, 50 Hz"), ("barcode", 15, "0", "Code 39 decoder"),
             ("comms", 18, "0", "MQTT frames, 100 ms"), ("status", 20, "0", "console + LED"),
             ("idle0 / idle1", 32, "0 / 1", "CPU-load meters (lowest)"),
             ("USB console (kernel)", 3, "0", ""), ("WiFi + MQTT (kernel)", 4, "0", "")]

    def build_resources(self, nb):
        f = self.tab(nb, "Resources")
        self.tools(f, ["resources"])
        row = ttk.Frame(f)
        row.pack(fill="x")
        self.ch_cpu = Chart(row, "CPU load", [("cpu0", "core 0", C1), ("cpu1", "core 1", C2)], " %",
                            yfixed=(0, 100), height=190)
        self.ch_cpu.pack(side="left", fill="x", expand=True)
        self.ch_loop = Chart(row, "Longest loop each second", [("motion_us", "motion", C3), ("line_us", "line", C4)],
                             " us", height=190, ymin0=True)
        self.ch_loop.pack(side="left", fill="x", expand=True, padx=(8, 0))
        low = ttk.Frame(f)
        low.pack(fill="both", expand=True, pady=8)
        self.res_bars = tk.Canvas(low, width=430, height=300, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        self.res_bars.pack(side="left", fill="y")
        tv = ttk.Treeview(low, columns=["task", "pri", "core", "job"], show="headings", height=11)
        for c, w, t in [("task", 160, "Task"), ("pri", 60, "Priority"), ("core", 60, "Core"), ("job", 260, "Job")]:
            tv.heading(c, text=t)
            tv.column(c, width=w, anchor="w")
        for r in self.TASKS:
            tv.insert("", "end", values=r)
        tv.pack(side="left", fill="both", expand=True, padx=(8, 0))

    def draw_resources(self):
        c = self.res_bars
        c.delete("all")
        c.create_text(10, 8, anchor="nw", text="Kernel objects in use (whole firmware)", fill=INK,
                      font=("Segoe UI", 10, "bold"))
        rs = self.last.get("tlm/resources")
        if not rs:
            c.create_text(215, 150, text="no resource data yet", fill=MUTED)
            return
        d = rs[1]
        y = 40
        for key, label in [("tasks", "Tasks"), ("flags", "Event flags"), ("sems", "Semaphores"),
                           ("mbfs", "Message buffers"), ("cycs", "Cyclic handlers")]:
            used, mx = d[key], max(d[key + "_max"], 1)
            frac = used / mx
            col = GREEN if frac < 0.6 else AMBER if frac < 0.85 else RED
            c.create_text(10, y + 8, anchor="w", text=label, fill=INK)
            c.create_rectangle(130, y, 330, y + 16, outline=GRID, fill=BG)
            c.create_rectangle(130, y, 130 + 200 * frac, y + 16, outline="", fill=col)
            c.create_text(340, y + 8, anchor="w", text="%d / %d" % (used, mx), fill=INK)
            y += 30
        y += 6
        for text in ["CPU load: core 0 %d %%, core 1 %d %%" % (d["cpu0"], d["cpu1"]),
                     "Longest motion loop %d us, line loop %d us (period 20000 us)" % (d["motion_us"], d["line_us"]),
                     "MQTT out-queue %d %% full, %d frames dropped" % (d["queue_pct"], d["drops"])]:
            c.create_text(10, y, anchor="nw", text=text, fill=INK)
            y += 22

    def build_raw(self, nb):
        f = self.tab(nb, "Raw frames")
        t = self.tools(f, ["raw"])
        ttk.Label(t, text="Show topic").pack(side="left")
        self.cb_raw = ttk.Combobox(t, values=["all"] + list(DECODERS), width=14, state="readonly")
        self.cb_raw.set("all")
        self.cb_raw.pack(side="left", padx=6)
        self.cb_raw.bind("<<ComboboxSelected>>", lambda e: self.refill_raw())
        cols = ["time", "topic", "bytes", "hex", "decoded"]
        self.tv_raw = ttk.Treeview(f, columns=cols, show="headings")
        for c, w in zip(cols, [80, 110, 50, 150, 760]):
            self.tv_raw.heading(c, text=c.capitalize())
            self.tv_raw.column(c, width=w, anchor="w")
        self.tv_raw.pack(fill="both", expand=True)
        self.raw_shown = 0

    # ------------------------------------------------ connection
    def set_conn(self, ok, text):
        self.lamp.delete("all")
        self.lamp.create_oval(2, 2, 12, 12, fill=GREEN if ok else GREY, outline="")
        self.l_conn.configure(text=text)
        for b in self.cmd_buttons:
            b.state(["!disabled"] if ok else ["disabled"])

    def toggle_connect(self):
        if self.client:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if mqtt is None:
            messagebox.showerror("paho-mqtt missing", "Install it first:\n\npip install paho-mqtt")
            return
        self.stop_demo()
        host = self.e_broker.get().strip()
        if not host:
            messagebox.showinfo("Broker", "Type the broker's IP address (the PC running Mosquitto).")
            return
        root = self.e_root.get().strip() or "robot"
        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="is27-gui-%d" % random.randint(0, 99999))
        except AttributeError:
            c = mqtt.Client(client_id="is27-gui-%d" % random.randint(0, 99999))

        def on_connect(cl, u, flags, rc, props=None):
            self.map_seen = False
            cl.subscribe(root + "/#", qos=1)
            self.q.put(("conn", True, "connected to %s" % host))

        def on_disconnect(cl, u, *a):
            self.q.put(("conn", False, "disconnected (retrying)"))

        def on_message(cl, u, msg):
            if msg.topic.startswith(root + "/"):
                self.q.put(("msg", time.time(), msg.topic[len(root) + 1:], bytes(msg.payload)))

        c.on_connect, c.on_disconnect, c.on_message = on_connect, on_disconnect, on_message
        try:
            c.connect_async(host, int(self.e_port.get() or 1883), 30)
            c.loop_start()
        except Exception as e:      # bad address etc.
            messagebox.showerror("Connect failed", str(e))
            return
        self.client, self.root_topic = c, root
        self.b_conn.configure(text="Disconnect")
        self.set_conn(False, "connecting to %s ..." % host)

    def disconnect(self):
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            self.client = None
        self.b_conn.configure(text="Connect")
        self.set_conn(False, "not connected")

    def send_speed(self):
        try:
            frame = encode_tune(int(self.v_speed.get()), int(self.v_bend.get()), int(self.v_kp.get()),
                                bool(self.v_pivot.get()))
        except (ValueError, tk.TclError):
            return
        if self.demo:
            self.demo.speed, self.demo.bend, self.demo.kp = frame[1] * 10, frame[2], frame[3] * 100
            self.demo.pivot = frame[4] == 2
        elif self.client:
            self.client.publish(self.root_topic + "/cmd", frame, qos=1, retain=False)
        self.event("sent TUNING speed %d mm/s, bends %d %%, KP %d, pivot %s" %
                   (frame[1] * 10, frame[2], frame[3] * 100, "on" if frame[4] == 2 else "off"))

    def send_map(self, clear=False):
        """Publish the barcode map to <root>/map, QoS 1, retained."""
        text = "" if clear else self.e_map.get()
        try:
            act = parse_map_text(text)
        except ValueError as e:
            messagebox.showerror("Barcode map", "%s\n\nExample: A=L B=R C=U D=S" % e)
            return
        if clear:
            self.e_map.delete(0, "end")
        frame = encode_map(act)
        if self.demo:
            self.demo.bc_map = act
            self.handle(time.time(), "tlm/map", frame + b"\x01")     # the demo car confirms at once
        elif self.client:
            self.client.publish(self.root_topic + "/map", frame, qos=1, retain=True)
        self.event("sent barcode map: %s" % map_to_text(act))

    def send(self, name):
        self.cmd_id = (self.cmd_id + 1) % 32
        frame = encode_cmd(name, self.cmd_id)
        if self.demo:
            self.demo.command(name)
        elif self.client:
            self.client.publish(self.root_topic + "/cmd", frame, qos=1, retain=False)
        self.event("sent %s (id %d)" % (name, self.cmd_id))

    # ------------------------------------------------ demo
    def toggle_demo(self):
        if self.demo:
            self.stop_demo()
            self.set_conn(False, "not connected")
        else:
            self.disconnect()
            self.start_demo()

    def start_demo(self):
        self.demo = DemoCar(self.q)
        self.demo.start()
        self.set_conn(True, "DEMO (simulated car)")

    def stop_demo(self):
        if self.demo:
            self.demo.stop()
            self.demo = None

    # ------------------------------------------------ incoming data
    def pump(self):
        n = 0
        try:
            while n < 800:
                item = self.q.get_nowait()
                n += 1
                if item[0] == "conn":
                    self.set_conn(item[1], item[2])
                else:
                    self.handle(*item[1:])
        except queue.Empty:
            pass
        self.root.after(100, self.pump)

    def handle(self, t, sub, payload):
        self.msgs += 1
        self.last_rx = t
        if sub == "map":                # our own retained map, echoed by the broker
            if not self.map_seen and len(payload) == 10:
                self.map_seen = True    # first one after connecting: show what the broker holds
                try:
                    act = decode_map(payload)
                except ValueError:
                    return
                if all(0 <= c <= 4 for c in act):
                    self.e_map.delete(0, "end")
                    self.e_map.insert(0, " ".join("%s=%s" % (chr(65 + i), "-LRUS"[c])
                                                  for i, c in enumerate(act) if c))
            return
        dec = DECODERS.get(sub)
        if dec is None:
            return
        try:
            d = dec(payload)
        except ValueError as e:
            self.ds["raw"].add({"t": t, "topic": sub, "bytes": len(payload), "hex": payload.hex(), "decoded": "BAD: %s" % e})
            return
        self.ds["raw"].add({"t": t, "topic": sub, "bytes": len(payload), "hex": payload.hex(),
                            "decoded": ", ".join("%s=%s" % (k, v) for k, v in d.items() if k != "pad")})
        self.last[sub] = (t, d)

        if sub == "status":
            self.ds["status"].add({"t": t, "seq": d["seq"], "up_s": d["up_s"], "mode": d["mode"],
                                   "faults": ",".join(d["fault_names"]) or "none", "tx_drop": d["tx_drop"]})
            if d["mode"] != self.prev_mode:
                self.event("mode %s" % d["mode"])
                self.prev_mode = d["mode"]
            if d["faults"] != self.prev_faults:
                self.ds["fault"].add({"t": t, "source": "status",
                                      "faults": ",".join(d["fault_names"]) or "all clear"})
                self.prev_faults = d["faults"]
        elif sub == "event/fault":
            self.ds["fault"].add({"t": t, "source": "FAULT EVENT", "faults": "raised " + ",".join(d["raised_names"])})
            self.event("FAULT raised %s" % ",".join(d["raised_names"]))
        elif sub == "tlm/motion":
            self.ds["motion"].add({"t": t, "vl": d["vl"], "vr": d["vr"], "dist_cm": d["dist_cm"],
                                   "owner": d["owner"], "enc_ok": enc_text(d)})
        elif sub == "tlm/line":
            see = ("L" if d["see"] & 1 else "") + ("R" if d["see"] & 2 else "")
            self.ds["line"].add({"t": t, "mode": d["mode"], "pos": d["pos"], "sensors": see or "-",
                                 "bc_char": d["bc_char"], "bc_action": d["bc_action"] if d["bc_char"] else "",
                                 "speed_mms": d["speed_mms"]})
            bc = (d["bc_char"], d["bc_action"])
            if d["bc_char"] and bc != self.prev_bc:
                self.ds["barcode"].add({"t": t, "text": d["bc_char"], "action": d["bc_action"]})
                self.event('barcode "%s" -> %s' % bc)
            self.prev_bc = bc if d["bc_char"] else self.prev_bc
        elif sub == "tlm/terrain":
            self.ds["terrain"].add({"t": t, "pitch": d["pitch_d10"] / 10, "roll": d["roll_d10"] / 10,
                                    "heading": "-" if d["heading"] is None else d["heading"],
                                    "hump_mm": d["hump_mm"], "events": " ".join(d["event_names"])})
            new = d["events"] & ~self.prev_hump_ev      # rising edges only (bits stay set ~1 s)
            if new & 1:
                self.ds["hump"].add({"t": t, "height_mm": d["hump_mm"]})
                self.event("hump %d mm" % d["hump_mm"])
            if new & 2:
                self.event("impact")
            if new & 4:
                self.event("tilt / picked up")
            self.prev_hump_ev = d["events"]
        elif sub == "tlm/obstacle":
            self.ds["obstacle"].add({"t": t, **{k: d[k] for k in self.ds["obstacle"].columns}})
            if d["stage"] == "SWEPT":
                self.last_sweep = (t, d)
                self.event("obstacle swept: %d cm, width %d cm -> going %s" % (d["dist_cm"], d["width_cm"], d["action"]))
            elif d["stage"] == "DETECTED":
                self.event("obstacle: stopped at %d cm" % d["dist_cm"])
            else:
                self.event("obstacle: %s" % d["stage"].lower().replace("_", " "))
        elif sub == "tlm/map":
            src = "from GUI" if d["received"] else "built-in, GUI map not received"
            self.l_map.configure(text="Car is using: %s (%s)" % (d["text"], src),
                                 foreground=INK if d["received"] else AMBER)
            if (d["map"], d["received"]) != self.car_map:
                self.car_map = (d["map"], d["received"])
                self.event("car barcode map: %s (%s)" % (d["text"], src))
        elif sub == "tlm/resources":
            self.ds["resources"].add({"t": t, **{k: d[k] for k in self.ds["resources"].columns}})

    def event(self, text):
        t = time.time()
        self.ds["event"].add({"t": t, "what": text})
        self.log.insert(0, "%s  %s" % (fmt_time(t), text))
        if self.log.size() > 400:
            self.log.delete(400, "end")

    # ------------------------------------------------ drawing
    def redraw(self):
        now = time.time()
        self.rate = self.rate * 0.6 + self.msgs * 2 * 0.4      # called every 0.5 s
        self.msgs = 0
        win = WINDOWS[self.cb_win.get()]
        if not self.v_freeze.get():
            self.ch_speed.draw(self.ds["motion"].rows, win, now)
            self.ch_dist.draw(self.ds["motion"].rows, win, now)
            self.ch_line.draw(self.ds["line"].rows, win, now)
            self.ch_tilt.draw(self.ds["terrain"].rows, win, now)
            self.ch_cpu.draw(self.ds["resources"].rows, win, now)
            self.ch_loop.draw(self.ds["resources"].rows, win, now)
        for tb in (self.t_motion, self.t_line, self.t_bc, self.t_terrain, self.t_hump, self.t_obs,
                   self.t_fault, self.t_status):
            tb.refresh()
        self.refresh_raw()
        self.draw_overview(now)
        self.draw_compass()
        self.draw_topview()
        self.draw_resources()
        self.root.after(500, self.redraw)

    def draw_overview(self, now):
        st = self.last.get("status")
        if st:
            d = st[1]
            col = {"LINE": GREEN, "BARCODE": GREEN, "AVOID": AMBER, "RECOVERY": AMBER,
                   "SAFE_STOP": RED, "COMPLETE": C1}.get(d["mode"], INK)
            self.ov["mode"].configure(text=d["mode"], foreground=col)
            h, rest = divmod(d["up_s"], 3600)
            self.ov["up"].configure(text="%d:%02d:%02d" % (h, rest // 60, rest % 60))
            age = now - st[0]
            self.ov["age"].configure(text="%.0f s ago" % age, foreground=RED if age > 3 else INK)
            for i, cv in enumerate(self.fault_lamps):
                cv.delete("all")
                cv.create_oval(1, 1, 13, 13, fill=RED if d["faults"] & (1 << i) else GREY, outline="")
        self.ov["rate"].configure(text="%.1f" % self.rate)

        def val(k, text):
            self.live[k].configure(text=text)
        m = self.last.get("tlm/motion")
        if m:
            d = m[1]
            val("speed", "%d / %d mm/s" % (d["vl"], d["vr"]))
            val("dist", "%.2f m" % (d["dist_cm"] / 100))
            val("owner", "%s%s" % (d["owner"], "" if d["enc_ok"] else ", encoder " + enc_text(d)))
        ln = self.last.get("tlm/line")
        if ln:
            d = ln[1]
            val("line", "%s, pos %d" % (d["mode"], d["pos"]))
            val("lspeed", "%d mm/s, bends %d %%, KP %d, pivot %s" % (d["speed_mms"], d["bend_pct"], d["kp"],
                                                                 "on" if d.get("pivot") else "off"))
            val("barcode", ('"%s" %s' % (d["bc_char"], d["bc_action"])) if d["bc_char"] else "-")
        tr = self.last.get("tlm/terrain")
        if tr:
            d = tr[1]
            val("tilt", "%.1f / %.1f deg" % (d["pitch_d10"] / 10, d["roll_d10"] / 10))
            val("heading", "-" if d["heading"] is None else "%d deg" % d["heading"])
            val("humps", "%d (last %d mm)" % (len(self.ds["hump"].rows), d["hump_mm"]))
        ob = self.last.get("tlm/obstacle")
        if ob:
            d = ob[1]
            val("obstacle", "%s, %d cm" % (d["stage"].lower().replace("_", " "), d["dist_cm"]))
        rs = self.last.get("tlm/resources")
        if rs:
            val("cpu", "%d %% / %d %%" % (rs[1]["cpu0"], rs[1]["cpu1"]))

    def draw_compass(self):
        c = self.compass
        c.delete("all")
        cx, cy, r = 105, 108, 62
        c.create_text(8, 6, anchor="nw", text="Heading (uncalibrated)", fill=INK, font=("Segoe UI", 10, "bold"))
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=GRID, width=2)
        for a, n in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            x, y = cx + (r + 12) * math.sin(math.radians(a)), cy - (r + 12) * math.cos(math.radians(a))
            c.create_text(x, y, text=n, fill=MUTED, font=("Segoe UI", 9, "bold"))
        tr = self.last.get("tlm/terrain")
        if tr and tr[1]["heading"] is not None:
            a = math.radians(tr[1]["heading"])
            c.create_line(cx, cy, cx + (r - 8) * math.sin(a), cy - (r - 8) * math.cos(a), fill=RED, width=3, arrow="last")
            c.create_text(cx, cy + r + 30, text="%d deg" % tr[1]["heading"], fill=INK,
                          font=("Segoe UI", 10, "bold"))
        else:
            c.create_text(cx, cy, text="no compass", fill=MUTED)

    def draw_topview(self):
        """Car at the bottom, looking up; obstacle drawn from the last sweep."""
        c = self.topview
        c.delete("all")
        w, h = 440, 440
        c.create_text(10, 8, anchor="nw", text="Last obstacle, seen from above", fill=INK, font=("Segoe UI", 10, "bold"))
        s = 6.0                                     # px per cm
        cx, cy = w / 2, h - 70                      # sensor position
        for d in range(10, 60, 10):                 # range rings
            c.create_arc(cx - d * s, cy - d * s, cx + d * s, cy + d * s, start=10, extent=160, style="arc", outline=GRID)
            c.create_text(cx + 4, cy - d * s, anchor="sw", text="%d cm" % d, fill=MUTED, font=("Segoe UI", 8))
        c.create_rectangle(cx - 9 * s, cy, cx + 9 * s, cy + 50, fill="#dbe4f0", outline="#8aa0bd")
        c.create_text(cx, cy + 25, text="car", fill=MUTED)
        ob = self.last_sweep
        if not ob:
            c.create_text(cx, h / 2, text="no obstacle sweep yet", fill=MUTED)
            return
        d = ob[1]
        y = cy - d["dist_cm"] * s
        # edges: the car stores how far it must move; edge = move - half car (9) - margin (5)
        xl = cx - (d["need_left_cm"] - 14) * s
        xr = cx + (d["need_right_cm"] - 14) * s
        if xr - xl < 2 * s:
            mid = cx - math.sin(math.radians(d["bearing_deg"])) * d["dist_cm"] * s
            xl, xr = mid - max(d["width_cm"], 2) * s / 2, mid + max(d["width_cm"], 2) * s / 2
        c.create_rectangle(xl, y - 30, xr, y, fill="#fde2d0", outline=C2, width=2)
        c.create_text((xl + xr) / 2, y - 38, text="%d cm wide" % d["width_cm"], fill=C2)
        if d["action"] in ("LEFT", "RIGHT"):
            sgn = -1 if d["action"] == "LEFT" else 1
            move = (d["need_left_cm"] if sgn < 0 else d["need_right_cm"]) * s
            c.create_line(cx, cy, cx + sgn * move, y + 10, cx + sgn * move, y - 50, smooth=True,
                          fill=C3, width=3, arrow="last")
            c.create_text(cx + sgn * move, y - 60, text="goes %s (%d cm)" % (d["action"].lower(), move / s), fill=C3)
        else:
            c.create_text(cx, y - 50, text="BLOCKED: waiting", fill=RED)
        c.create_text(10, h - 14, anchor="w", fill=MUTED, font=("Segoe UI", 9),
                      text="%d cm away at %+d deg (+ = left)" % (d["dist_cm"], d["bearing_deg"]))

    def refresh_raw(self):
        rows = self.ds["raw"].rows
        new = rows[self.raw_shown:] if self.raw_shown <= len(rows) else rows
        self.raw_shown = len(rows)
        flt = self.cb_raw.get()
        for r in new[-TABLE_ROWS:]:
            if flt == "all" or r["topic"] == flt:
                self.tv_raw.insert("", 0, values=[fmt_time(r["t"]), r["topic"], r["bytes"], r["hex"], r["decoded"]])
        for k in self.tv_raw.get_children()[TABLE_ROWS:]:
            self.tv_raw.delete(k)

    def refill_raw(self):
        self.tv_raw.delete(*self.tv_raw.get_children())
        flt = self.cb_raw.get()
        rows = [r for r in self.ds["raw"].rows if flt == "all" or r["topic"] == flt][-TABLE_ROWS:]
        for r in rows:
            self.tv_raw.insert("", 0, values=[fmt_time(r["t"]), r["topic"], r["bytes"], r["hex"], r["decoded"]])

    # ------------------------------------------------ files
    def export(self, name):
        ds = self.ds[name]
        if not ds.rows:
            messagebox.showinfo("Export", "No %s data yet." % name)
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="is27_%s_%s.csv" %
                                            (name, time.strftime("%Y%m%d_%H%M%S")), filetypes=[("CSV", "*.csv")])
        if path:
            ds.export(path)

    def export_all(self):
        folder = filedialog.askdirectory(title="Folder for the CSV files")
        if not folder:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        n = 0
        for name, ds in self.ds.items():
            if ds.rows:
                ds.export("%s/is27_%s_%s.csv" % (folder, name, stamp))
                n += 1
        messagebox.showinfo("Export", "Saved %d CSV files." % n)

    def clear_all(self):
        for ds in self.ds.values():
            ds.clear()
        for tb in (self.t_motion, self.t_line, self.t_bc, self.t_terrain, self.t_hump, self.t_obs,
                   self.t_fault, self.t_status):
            tb.reset()
        self.tv_raw.delete(*self.tv_raw.get_children())
        self.raw_shown = 0
        self.log.delete(0, "end")
        self.last.clear()
        self.last_sweep = None


# ---------------------------------------------------------------- demo car
class DemoCar(threading.Thread):
    """Makes the same binary frames the car sends, so the whole decode path
    is exercised without a broker: driving with curves, a hump every ~25 s,
    an obstacle every ~30 s, a barcode every ~20 s, an encoder fault now and
    then.  The command buttons change its mode."""

    def __init__(self, q):
        super().__init__(daemon=True)
        self.q, self.run_flag = q, True
        self.mode = 2                           # LINE
        self.t0 = time.time()
        self.back_to_line = 0
        self.speed, self.bend, self.kp = 30, 50, 1000
        self.pivot = False
        self.bc_map = [0] * MAP_LETTERS

    def command(self, name):
        self.mode = {"START": 2, "RESUME": 2, "STOP": 1, "PAUSE": 6, "UTURN": 3, "REVERSE": 5}.get(name, self.mode)
        if name in ("UTURN", "REVERSE"):
            self.back_to_line = time.time() + (3 if name == "UTURN" else 2)
        if name == "PING":
            self.put("status", self.status())

    def stop(self):
        self.run_flag = False

    def put(self, sub, frame):
        self.q.put(("msg", time.time(), sub, frame))

    def status(self, faults=0):
        up = int(time.time() - self.t0)
        return encode("status", seq=up & 15, up_s=up, mode=self.mode, faults=faults)

    def run(self):
        dist, k, faults = 0.0, 0, 0
        bc_char, bc_act = 0, 0
        while self.run_flag:
            t = time.time() - self.t0
            ms = int(t * 1000) & 0xFFFF
            if self.back_to_line and time.time() > self.back_to_line:
                self.mode = 2 if self.mode == 3 else 1          # U-turn -> follow, reverse -> idle
                self.back_to_line = 0
            moving = self.mode in (2, 3, 4, 5)
            curve = math.sin(t / 4) * 0.6 if moving else 0
            base = self.speed if moving else 0
            vl, vr = base * (1 + curve * 0.5), base * (1 - curve * 0.5)
            dist += (vl + vr) / 2 * 0.2 / 10
            pos = int(curve * 900 + random.gauss(0, 40)) if moving else 0
            mode = (7 if self.pivot and abs(pos) > 600 else 1) if abs(pos) > 150 else 2
            if moving and int(t) % 20 == 10 and k % 5 == 0:
                bc_char = random.choice([ord("A"), ord("B"), ord("C"), ord("Z")])
                bc_act = self.bc_map[bc_char - ord("A")]
            ph = (t % 25)
            pitch = 120 * math.sin((ph - 10) / 1.2 * math.pi) if 10 < ph < 12.4 and moving else random.gauss(0, 5)
            hump_ev = 1 if 12.4 < ph < 13.4 and moving else 0
            faults = 0x002 if 40 < (t % 60) < 44 else 0
            self.put("tlm/motion", encode("motion", t_ms=ms, vl=vl, vr=vr, dist_cm=dist, owner=1 if moving else 0,
                                          enc_l=0 if faults else 1, enc_r=1))
            self.put("tlm/line", encode("line", t_ms=ms, mode=mode if moving else 0, pos8=pos // 8,
                                        see=(1 if pos < -150 else 0) | (2 if pos > 150 else 0),
                                        bc_char=bc_char - 31 if bc_char else 0, bc_action=bc_act,
                                        speed_cs=self.speed // 10, bend_pct=self.bend, kp100=self.kp // 100,
                                        pivot=1 if self.pivot else 0))
            self.put("tlm/terrain", encode("terrain", t_ms=ms, pitch_d10=pitch, roll_d10=random.gauss(0, 8),
                                           hump_mm=31 if hump_ev or ph > 12.4 else 0,
                                           heading=int(t * 9) % 360, events=hump_ev))
            if k % 5 == 0:
                self.put("status", self.status(faults))
            if k % 5 == 2:
                busy = 18 if moving else 6
                self.put("tlm/resources", encode("resources", t_ms=ms, cpu0=busy + random.randint(0, 6),
                                                 cpu1=busy + 8 + random.randint(0, 8), tasks=13, tasks_max=32,
                                                 flags=7, flags_max=16, sems=2, sems_max=16, mbfs=2, mbfs_max=12,
                                                 cycs=1, cycs_max=12, queue_pct=random.randint(0, 6), drops=0,
                                                 motion_us=150 + random.randint(0, 60),
                                                 line_us=80 + random.randint(0, 40)))
            ob_phase = k % 150
            if moving and ob_phase in (75, 85, 100, 115):
                if ob_phase == 75:
                    self.ob_w, self.ob_c = random.randint(6, 25), random.randint(-6, 6)
                w, c = self.ob_w, self.ob_c                # obstacle width, centre (cm, + = left)
                left, right = c + w // 2, c - w // 2
                nl, nr = left + 14, -right + 14
                stage = {75: 1, 85: 3, 100: 6, 115: 7}[ob_phase]  # detected, swept, passed, rejoined
                self.put("tlm/obstacle", encode("obstacle", t_ms=ms, dist_cm=25,
                                                bearing_deg=round(math.degrees(math.atan2(c, 25))),
                                                width_cm=w if stage >= 3 else 0,
                                                need_left_cm=nl if stage >= 3 else 0,
                                                need_right_cm=nr if stage >= 3 else 0,
                                                action=(1 if nl <= nr else 2) if stage >= 3 else 0, stage=stage))
            if faults and k % 300 == 201:
                self.put("event/fault", encode("fault", t_ms=ms, raised=0x002))
            k += 1
            time.sleep(0.2)


def main():
    p = argparse.ArgumentParser(description="IS27 car dashboard")
    p.add_argument("--broker", default="")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--root", default="robot")
    p.add_argument("--demo", action="store_true", help="simulated car, no broker needed")
    a = p.parse_args()
    root = tk.Tk()
    App(root, a)
    root.mainloop()


if __name__ == "__main__":
    main()
