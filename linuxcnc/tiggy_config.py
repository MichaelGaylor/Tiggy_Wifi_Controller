#!/usr/bin/env python3
"""
Tiggy motion controller - set-up for LinuxCNC, in windows.

    python3 tiggy_config.py [folder of a configuration]

Finds the controller on the network, and sets up a mill, router or lathe
with it connected: which motor is on which output, motor tuning with the
motor running, switches with their state shown live, the spindle and its
sensor.  It writes the same LinuxCNC configuration as tiggy_setup.py (which
asks the same in a terminal) into ~/linuxcnc/configs/<name>, and opens one it
wrote before to change it.

The controller works for one program at a time: LinuxCNC has to be closed
while this program is connected to it.
"""

import json
import os
import subprocess
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tiggy_link as link       # noqa: E402
import tiggy_setup as gen       # noqa: E402

LETTERS = gen.LETTERS
CONFIGS = os.path.join(os.path.expanduser("~"), "linuxcnc", "configs")
MIN_FIRMWARE = (1 << 24) | (2 << 16)
USES = [("none", "no switch"), ("home", "home switch"), ("limit", "limit switch"), ("both", "home and limit")]
SENSORS = [(link.SENSOR_NONE, "none"), (link.SENSOR_PULSE, "one pulse per revolution"),
           (link.SENSOR_ENCODER, "encoder with index pulse"), (link.SENSOR_ENCODER_NO_INDEX, "encoder without index")]
ON, OFF, UNKNOWN = "#35b34a", "#c9c9c9", "#f2f2f2"


# ------------------------------------------------------------------ the machine

def new_machine(lathe=False):
    gen.QUIET = True
    m = gen.ask_machine(gen.Asker(True, lathe=lathe))
    m["address_fixed"] = ""
    return m


def new_joint(m, letter, output, second=False):
    rotary = letter in "ABC"
    inch = m["inch"]
    j = dict(letter=letter, second=second, rotary=rotary, output=output, reverse=False,
             scale=88.8889 if rotary else (20000.0 if inch else 800.0),
             vel=90.0 if rotary else (2.0 if inch else 50.0),
             acc=900.0 if rotary else (20.0 if inch else 500.0),
             switch_input=output, home_switch=False, limit_switch=False, switch_nc=False,
             home_positive=(letter == "Z" or bool(m.get("lathe"))))
    j["lo"], j["hi"] = (-99999.0, 99999.0) if rotary else gen.travel_default(m, letter)
    return j


def read_ini(path):
    """{section: {key: value}} of an INI file (LinuxCNC's kind: no nesting, last value wins)."""
    out, section = {}, None
    with open(path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("[") and line.endswith("]"):
                section = out.setdefault(line[1:-1], {})
            elif section is not None and "=" in line and not line.startswith(("#", ";")):
                key, value = line.split("=", 1)
                section[key.strip()] = value.strip()
    return out


def number(text, default):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def load_machine(folder):
    """The machine of a configuration folder: its answers (tiggy.json), with the numbers that
    may since have been changed by hand taken from the INI file.  Returns (machine, notes)."""
    notes = []
    inis = sorted(f for f in os.listdir(folder) if f.endswith(".ini"))
    ini = read_ini(os.path.join(folder, inis[0])) if inis else {}
    answers = os.path.join(folder, "tiggy.json")
    if os.path.exists(answers):
        with open(answers) as f:
            m = json.load(f)
    else:
        # written by an older set-up: the kind of machine and its axes are in the INI file,
        # what the switches are used for is not
        lathe = ini.get("DISPLAY", {}).get("LATHE", "0") == "1"
        m = new_machine(lathe)
        m["inch"] = ini.get("TRAJ", {}).get("LINEAR_UNITS", "mm").startswith("in")
        letters = ini.get("TRAJ", {}).get("COORDINATES", "X Y Z").replace(" ", "")
        m["joints"], seen = [], set()
        for n, letter in enumerate(c for c in letters if c in LETTERS):
            output = LETTERS.index(letter) if lathe else n
            m["joints"].append(new_joint(m, letter, output, second=letter in seen))
            seen.add(letter)
        m["name"] = ini.get("EMC", {}).get("MACHINE", os.path.basename(folder))
        notes.append("This configuration was written by an older set-up: check the Switches and Spindle pages, "
                     "which could not be read back from it.")
    m.setdefault("address_fixed", "")
    m.setdefault("lathe", False)
    m.setdefault("diameter", False)
    m.setdefault("buffer_ms", 200)
    for n, j in enumerate(m["joints"]):
        j.setdefault("output", n)
        s = ini.get("JOINT_%d" % n, {})
        j["scale"] = abs(number(s.get("SCALE"), j["scale"])) or j["scale"]
        j["vel"] = number(s.get("MAX_VELOCITY"), j["vel"])
        j["acc"] = number(s.get("MAX_ACCELERATION"), j["acc"])
        j["lo"] = number(s.get("MIN_LIMIT"), j["lo"])
        j["hi"] = number(s.get("MAX_LIMIT"), j["hi"])
        if "DIR_INVERT" in s:
            j["reverse"] = s["DIR_INVERT"].strip() == "1"
    t = ini.get("TIGGY", {})
    if t:
        m["address"] = t.get("ADDRESS", m["address"])
        m["pulse"] = int(number(t.get("STEP_PULSE_US"), m["pulse"]))
        m["dir_setup"] = int(number(t.get("DIR_SETUP_US"), m["dir_setup"]))
        m["max_rpm"] = number(t.get("SPINDLE_MAX_RPM"), m["max_rpm"])
        m["pwm_freq"] = int(number(t.get("SPINDLE_PWM_FREQ"), m["pwm_freq"]))
        m["pump"] = int(number(t.get("CHARGE_PUMP_FREQ"), m["pump"]))
        m["buffer_ms"] = min(400, max(120, int(number(t.get("BUFFER_MS"), m["buffer_ms"]))))
    if m["address"] != "auto":
        m["address_fixed"] = m["address"]
    return m, notes


def existing():
    """Folders in ~/linuxcnc/configs that hold a Tiggy configuration."""
    found = []
    if os.path.isdir(CONFIGS):
        for name in sorted(os.listdir(CONFIGS)):
            folder = os.path.join(CONFIGS, name)
            if os.path.exists(os.path.join(folder, "tiggy.json")) or os.path.exists(os.path.join(folder, "tiggy.hal")):
                found.append(name)
    return found


def linuxcnc_running():
    try:
        return subprocess.run(["pgrep", "-x", "linuxcncsvr"], capture_output=True, timeout=3).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def nice(v):
    return gen.fmt(v)


# ------------------------------------------------------------------ the window

class App(tk.Tk):
    def __init__(self, folder=None):
        super().__init__()
        self.title("Tiggy set-up for LinuxCNC")
        self.minsize(860, 640)
        self.geometry("940x780")
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Title.TLabel", font=("TkDefaultFont", 11, "bold"))
        style.configure("Hint.TLabel", foreground="#555555")
        style.configure("Bad.TLabel", foreground="#b00020")
        style.configure("Good.TLabel", foreground="#1b7a2f")

        self.m = new_machine()
        self.folder = None                  # where the machine was opened from
        self.ctl = None
        self.jog = None                     # (motor output, direction) while a test button is held
        self.jog_sent = 0.0
        self.zero = [0] * 6
        self.loaders = []                   # functions that put the machine's values on screen
        self.selected = 0                   # joint shown on the Motors page

        top = ttk.Frame(self, padding=(10, 8, 10, 2))
        top.pack(fill="x")
        ttk.Label(top, text="Machine:").pack(side="left")
        self.which = tk.StringVar()
        self.chooser = ttk.Combobox(top, textvariable=self.which, state="readonly", width=34)
        self.chooser.pack(side="left", padx=6)
        self.chooser.bind("<<ComboboxSelected>>", self.choose)
        self.note = ttk.Label(top, text="", style="Hint.TLabel")
        self.note.pack(side="left", padx=10)

        self.book = ttk.Notebook(self)
        self.book.pack(fill="both", expand=True, padx=10, pady=6)
        self.pages = {}
        for name, build in (("Controller", self.page_controller), ("Machine", self.page_machine),
                            ("Motors", self.page_motors), ("Switches", self.page_switches),
                            ("Spindle", self.page_spindle), ("Finish", self.page_finish)):
            frame = ttk.Frame(self.book, padding=12)
            self.book.add(frame, text="  %s  " % name)
            self.pages[name] = frame
            build(frame)
        self.book.bind("<<NotebookTabChanged>>", lambda e: self.show())

        bottom = ttk.Frame(self, padding=(10, 2, 10, 10))
        bottom.pack(fill="x", side="bottom", before=self.book)      # (kept in view however small the window)
        self.lamp = tk.Canvas(bottom, width=14, height=14, highlightthickness=0)
        self.lamp.pack(side="left")
        self.lamp_dot = self.lamp.create_oval(2, 2, 12, 12, fill=OFF, outline="#888888")
        self.state = ttk.Label(bottom, text="Controller: not connected")
        self.state.pack(side="left", padx=6)
        ttk.Button(bottom, text="Close", command=self.close).pack(side="right")
        ttk.Button(bottom, text="Save and start LinuxCNC", command=lambda: self.save(start=True)).pack(side="right", padx=6)
        ttk.Button(bottom, text="Save", command=self.save).pack(side="right")

        self.protocol("WM_DELETE_WINDOW", self.close)
        self.fill_chooser()
        names = existing()
        if folder:
            self.open_folder(folder)
        elif names:
            self.open_folder(os.path.join(CONFIGS, names[0]))
        else:
            self.show()
        self.after(50, self.tick)

    # ---- small parts
    def row(self, parent, r, label, widget=None, hint=""):
        ttk.Label(parent, text=label).grid(row=r, column=0, sticky="w", pady=3)
        if widget is not None:
            widget.grid(row=r, column=1, sticky="w", padx=8, pady=3)
        if hint:
            ttk.Label(parent, text=hint, style="Hint.TLabel").grid(row=r, column=2, sticky="w")

    def entry(self, parent, get, put, width=10, kind=float, low=None, high=None):
        """An entry for one value of the machine.  get() reads it, put(value) stores it."""
        var = tk.StringVar()
        e = ttk.Entry(parent, textvariable=var, width=width)

        def load():
            v = get()
            var.set(v if kind is str else nice(v))

        def store(_event=None):
            text = var.get().strip().replace(",", ".")
            try:
                v = text if kind is str else kind(float(text))
                if kind is not str and ((low is not None and v < low) or (high is not None and v > high)):
                    raise ValueError
                if kind is str and not v:
                    raise ValueError
            except ValueError:
                load()                          # not a value this can have: put the old one back
                self.bell()
                return
            if v != get():
                put(v)
                self.show()

        e.bind("<FocusOut>", store)
        e.bind("<Return>", store)
        self.loaders.append(load)
        return e

    def check(self, parent, text, get, put):
        var = tk.BooleanVar()
        c = ttk.Checkbutton(parent, text=text, variable=var, command=lambda: (put(var.get()), self.show()))
        self.loaders.append(lambda: var.set(bool(get())))
        return c

    def radios(self, parent, options, get, put):
        var = tk.StringVar()
        frame = ttk.Frame(parent)
        for value, text in options:
            ttk.Radiobutton(frame, text=text, value=str(value), variable=var,
                            command=lambda: (put(var.get()), self.show())).pack(side="left", padx=(0, 14))
        self.loaders.append(lambda: var.set(str(get())))
        return frame

    def light(self, parent):
        c = tk.Canvas(parent, width=18, height=18, highlightthickness=0)
        c.dot = c.create_oval(3, 3, 15, 15, fill=UNKNOWN, outline="#888888")
        return c

    @staticmethod
    def shine(lamp, on):
        lamp.itemconfigure(lamp.dot, fill=UNKNOWN if on is None else (ON if on else OFF))

    def unit(self, j=None):
        if j is not None and j["rotary"]:
            return "degree"
        return "inch" if self.m["inch"] else "mm"

    def show(self):
        """Puts the machine on screen."""
        if self.selected >= len(self.m["joints"]):
            self.selected = 0
        for load in self.loaders:
            load()

    def joint(self):
        return self.m["joints"][self.selected]

    # ---- which machine
    def fill_chooser(self):
        self.names = existing()
        self.chooser["values"] = self.names + ["(a new mill or router)", "(a new lathe)"]

    def choose(self, _event=None):
        text = self.which.get()
        if text in self.names:
            self.open_folder(os.path.join(CONFIGS, text))
        else:
            self.m = new_machine(lathe="lathe" in text)
            if self.ctl:
                self.m["address"] = self.m["address_fixed"] = self.ctl.address
            self.folder = None
            self.note["text"] = "A new machine: give it a name on the Machine page."
            self.show()

    def open_folder(self, folder):
        try:
            self.m, notes = load_machine(folder)
        except (OSError, ValueError, KeyError) as e:
            messagebox.showerror("Tiggy set-up", "%s could not be opened:\n%s" % (folder, e))
            return
        self.folder = folder
        self.which.set(os.path.basename(folder))
        self.note["text"] = notes[0] if notes else folder
        self.show()

    # =============================================================== Controller
    def page_controller(self, f):
        ttk.Label(f, text="The controller", style="Title.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(f, style="Hint.TLabel", wraplength=760, justify="left",
                  text="The controller works for one program at a time. Close LinuxCNC (and Mach3) while you set "
                       "the machine up here; this program lets go of the controller when it starts LinuxCNC."
                  ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(2, 10))

        bar = ttk.Frame(f)
        bar.grid(row=2, column=0, columnspan=3, sticky="w")
        ttk.Button(bar, text="Find controllers", command=self.find).pack(side="left")
        self.find_note = ttk.Label(bar, text="", style="Hint.TLabel")
        self.find_note.pack(side="left", padx=10)

        cols = ("name", "address", "firmware", "serial", "state")
        self.found = ttk.Treeview(f, columns=cols, show="headings", height=5, selectmode="browse")
        for c, title, width in zip(cols, ("Name", "Address", "Firmware", "Serial number", ""), (150, 130, 80, 150, 200)):
            self.found.heading(c, text=title)
            self.found.column(c, width=width, anchor="w")
        self.found.grid(row=3, column=0, columnspan=3, sticky="we", pady=6)
        self.found.bind("<<TreeviewSelect>>", self.pick)
        self.found.bind("<Double-1>", lambda e: self.connect())

        box = ttk.Frame(f)
        box.grid(row=4, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Label(box, text="Address:").pack(side="left")
        self.address = self.entry(box, lambda: self.m["address_fixed"], self.set_address, width=16, kind=str)
        self.address.pack(side="left", padx=6)
        self.connect_button = ttk.Button(box, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=4)
        ttk.Button(box, text="Disconnect", command=self.disconnect).pack(side="left")

        self.auto = tk.BooleanVar()
        ttk.Checkbutton(f, variable=self.auto, command=self.set_auto,
                        text="Let LinuxCNC look for the controller each time it starts, instead of using this address"
                        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Label(f, style="Hint.TLabel", wraplength=760, justify="left",
                  text="Looking works when computer and controller are on the same network. It does not reach "
                       "through some routers, nor out of Linux running inside Windows (WSL): there, keep the address."
                  ).grid(row=6, column=0, columnspan=3, sticky="w")
        self.loaders.append(lambda: self.auto.set(self.m["address"] == "auto"))

        queue = ttk.Frame(f)
        queue.grid(row=7, column=0, columnspan=3, sticky="w", pady=(14, 0))
        ttk.Label(queue, text="Motion kept queued in the controller:").pack(side="left")
        self.entry(queue, lambda: self.m["buffer_ms"], lambda v: self.m.update(buffer_ms=v), width=5, kind=int,
                   low=120, high=400).pack(side="left", padx=6)
        ttk.Label(queue, text="ms").pack(side="left")
        ttk.Label(f, style="Hint.TLabel", wraplength=760, justify="left",
                  text="A late or lost network packet is covered by what is queued; the machine runs that far behind "
                       "LinuxCNC. 200 suits a good link. Over WiFi a longer stall can still spoil a screw-cutting pass: "
                       "use 400 for screw cutting over WiFi, or better, the controller's Ethernet."
                  ).grid(row=8, column=0, columnspan=3, sticky="w")

        self.about = ttk.Label(f, text="", justify="left", wraplength=760)
        self.about.grid(row=9, column=0, columnspan=3, sticky="w", pady=(16, 0))
        f.columnconfigure(2, weight=1)

    def set_address(self, text):
        self.m["address_fixed"] = text
        if self.m["address"] != "auto":
            self.m["address"] = text

    def set_auto(self):
        self.m["address"] = "auto" if self.auto.get() else (self.m["address_fixed"] or "auto")
        self.show()

    def find(self):
        self.find_note["text"] = "Looking..."
        self.update_idletasks()
        try:
            found = link.discover(also=[self.m["address_fixed"]])
        except OSError as e:
            found = []
            self.find_note["text"] = "Could not look: %s" % e
        self.found.delete(*self.found.get_children())
        for c in found:
            what = "an I/O module" if c["io_module"] else ("in use by a program" if c["busy"] else "free")
            if c["version"] < MIN_FIRMWARE:
                what = "firmware too old: needs 1.2.0"
            self.found.insert("", "end", iid=c["address"], values=(c["name"], c["address"], c["firmware"], c["serial"], what))
        if found:
            self.find_note["text"] = "%d found. Pick one and press Connect." % len(found)
            self.found.selection_set(found[0]["address"])
        elif not self.find_note["text"].startswith("Could"):
            self.find_note["text"] = ("None answered. Is it switched on and on this network? Otherwise type its "
                                      "address below.")

    def pick(self, _event=None):
        chosen = self.found.selection()
        if chosen:
            self.set_address(chosen[0])
            self.show()

    def connect(self):
        self.disconnect()
        address = self.m["address_fixed"]
        if not address:
            messagebox.showinfo("Tiggy set-up", "Press Find controllers, or type the controller's address.")
            return
        if linuxcnc_running():
            messagebox.showwarning("Tiggy set-up", "LinuxCNC is running, and the controller works for one program "
                                                   "at a time.\n\nClose LinuxCNC first.")
            return
        c = link.Controller(address)
        try:
            info = c.open()
            if info["version"] < MIN_FIRMWARE:
                c.close()
                raise link.LinkError("the controller has firmware %s: this needs 1.2.0 or newer" % info["firmware"])
            if info["io_module"]:
                c.close()
                raise link.LinkError("that is an I/O expansion module, not a motion controller")
            c.set("limit_mask", 0)          # (nothing has said yet which inputs are limit switches)
        except link.LinkError as e:
            messagebox.showerror("Tiggy set-up", "Could not connect:\n\n%s" % e)
            return
        self.ctl = c
        self.zero = [0] * 6
        self.zeroed = False
        self.describe()
        self.show()

    def describe(self):
        """What the controller says of itself (after connecting, and after it restarts)."""
        c = self.ctl
        try:
            kind, lines = c.sensor()
            terminals = c.get("terminals")
        except link.LinkError as e:
            self.lost(e)
            return
        self.on_controller = dict(kind=kind, lines=lines, terminals=terminals)
        sensor = dict(SENSORS)[kind] + (", %d lines" % lines if kind.startswith("encoder") else "")
        can = "can cut threads" if c.info["capabilities"] & link.CAP_SPINDLE_SYNC else "cannot cut threads"
        self.about["text"] = (
            "Connected to %s at %s, serial firmware %s.\n"
            "Fastest step rate with its present pulse length: %s steps a second on an axis.\n"
            "Terminals: %s.   Spindle sensor: %s (%s)." % (
                c.info["name"], c.address, c.info["firmware"], format(c.info["max_step_rate"], ","),
                "six axes" if terminals == link.TERMINALS_SIX else "four axes, inputs and outputs", sensor, can)
        ).replace("serial firmware", "firmware")

    def disconnect(self):
        if self.ctl:
            try:
                self.ctl.stop()
                self.ctl.out.update(spindle=0, rpm=0, outputs=0)
                self.ctl._sent_io = 0.0
                self.ctl.poll()
            except link.LinkError:
                pass
            self.ctl.close()
        self.ctl = None
        self.jog = None
        if hasattr(self, "about"):
            self.about["text"] = ""

    def lost(self, error):
        self.disconnect()
        self.state["text"] = "Controller: %s" % error

    # ================================================================== Machine
    def page_machine(self, f):
        ttk.Label(f, text="The machine", style="Title.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        self.row(f, 1, "Name", self.entry(f, lambda: self.m["name"], lambda v: self.m.update(name=v), width=24, kind=str),
                 "its configuration is kept in a folder of this name")
        self.row(f, 2, "Kind", self.radios(f, [("mill", "Mill or router"), ("lathe", "Lathe")],
                                           lambda: "lathe" if self.m["lathe"] else "mill", self.set_kind))
        self.row(f, 3, "Units", self.radios(f, [("mm", "Millimetres"), ("inch", "Inches")],
                                            lambda: "inch" if self.m["inch"] else "mm", self.set_units))
        self.diameter = self.check(f, "X positions are diameters (G7), as on most lathes",
                                   lambda: self.m["diameter"], lambda v: self.m.update(diameter=v))
        self.diameter.grid(row=4, column=1, columnspan=2, sticky="w", padx=8)

        ttk.Label(f, text="Motor outputs", style="Title.TLabel").grid(row=5, column=0, columnspan=3, sticky="w", pady=(16, 2))
        self.outputs_hint = ttk.Label(f, style="Hint.TLabel", wraplength=760, justify="left")
        self.outputs_hint.grid(row=6, column=0, columnspan=3, sticky="w", pady=(0, 6))
        grid = ttk.Frame(f)
        grid.grid(row=7, column=0, columnspan=3, sticky="w")
        self.output_boxes = []
        for o in range(6):
            ttk.Label(grid, text="Output %s  (STEP %s, DIR %s)" % (LETTERS[o], LETTERS[o], LETTERS[o])).grid(
                row=o, column=0, sticky="w", pady=2)
            var = tk.StringVar()
            box = ttk.Combobox(grid, textvariable=var, state="readonly", width=16,
                               values=["not used"] + ["%s axis" % c for c in LETTERS])
            box.grid(row=o, column=1, padx=10)
            box.bind("<<ComboboxSelected>>", lambda e, o=o, var=var: self.set_output(o, var.get()))
            label = ttk.Label(grid, text="", style="Hint.TLabel")
            label.grid(row=o, column=2, sticky="w")
            self.output_boxes.append((box, var, label))
        self.loaders.append(self.load_outputs)

    def load_outputs(self):
        m = self.m
        self.diameter.state(["!disabled"] if m["lathe"] else ["disabled"])
        self.outputs_hint["text"] = (
            "A lathe has X (the cross slide) on output X and Z (the carriage) on output Z." if m["lathe"] else
            "Say which axis each output's motor drives. An axis with two motors (a gantry) is given to two outputs.")
        by_output = {j["output"]: j for j in m["joints"]}
        for o, (box, var, label) in enumerate(self.output_boxes):
            j = by_output.get(o)
            var.set("%s axis" % j["letter"] if j else "not used")
            box.state(["disabled"] if m["lathe"] else ["!disabled", "readonly"])
            label["text"] = "second motor of %s" % j["letter"] if j and j["second"] else ""

    def set_kind(self, kind):
        lathe = kind == "lathe"
        if lathe == self.m["lathe"]:
            return
        keep = {k: self.m[k] for k in ("name", "address", "address_fixed", "inch")}
        self.m = new_machine(lathe)
        self.m.update(keep)
        if keep["inch"]:
            self.reset_joints()

    def set_units(self, unit):
        inch = unit == "inch"
        if inch == self.m["inch"]:
            return
        if not messagebox.askokcancel("Tiggy set-up", "Changing the units puts every axis back to typical steps, "
                                                      "speeds and travels for %s.\n\nGo on?" % ("inches" if inch else "millimetres")):
            return
        self.m["inch"] = inch
        self.reset_joints()

    def reset_joints(self):
        self.m["joints"] = [dict(new_joint(self.m, j["letter"], j["output"], j["second"]), reverse=j["reverse"])
                            for j in self.m["joints"]]

    def set_output(self, output, text):
        """Output 'output' now drives the axis named in text (or nothing)."""
        letter = text[0] if text != "not used" else ""
        joints = [j for j in self.m["joints"] if j["output"] != output]
        old = next((j for j in self.m["joints"] if j["output"] == output), None)
        if letter:
            if old and old["letter"] == letter:
                return
            joints.append(new_joint(self.m, letter, output))
        joints.sort(key=lambda j: j["output"])
        seen = set()
        for j in joints:                    # the motor on the lower output is the axis' first
            j["second"] = j["letter"] in seen
            seen.add(j["letter"])
        if not joints:
            self.bell()
            return
        self.m["joints"] = joints
        self.show()

    # =================================================================== Motors
    def page_motors(self, f):
        left = ttk.Frame(f)
        left.grid(row=0, column=0, sticky="ns", padx=(0, 16))
        ttk.Label(left, text="Motor", style="Title.TLabel").pack(anchor="w")
        self.motor_list = tk.Listbox(left, height=7, width=22, exportselection=False, activestyle="none")
        self.motor_list.pack(pady=4)
        self.motor_list.bind("<<ListboxSelect>>", self.pick_motor)
        self.loaders.append(self.load_motor_list)

        sig = ttk.LabelFrame(left, text="Step signals (all motors)", padding=8)
        sig.pack(fill="x", pady=(14, 0))
        self.row(sig, 0, "Step pulse", self.entry(sig, lambda: self.m["pulse"], lambda v: self.m.update(pulse=v),
                                                 width=5, kind=int, low=1, high=50), "us")
        self.row(sig, 1, "Direction set-up", self.entry(sig, lambda: self.m["dir_setup"],
                                                       lambda v: self.m.update(dir_setup=v), width=5, kind=int, low=0, high=50), "us")
        self.check(sig, "Enable is active low", lambda: self.m["enable_invert"],
                   lambda v: self.m.update(enable_invert=v)).grid(row=2, column=0, columnspan=3, sticky="w")
        ttk.Label(sig, style="Hint.TLabel", wraplength=190, justify="left",
                  text="From the drives' data sheet; 5 suits most.").grid(row=3, column=0, columnspan=3, sticky="w")

        right = ttk.Frame(f)
        right.grid(row=0, column=1, sticky="nsew")
        f.columnconfigure(1, weight=1)
        self.motor_title = ttk.Label(right, text="", style="Title.TLabel")
        self.motor_title.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        j = self.joint
        self.unit_labels = []

        def with_unit(r, label, widget, suffix):
            self.row(right, r, label, widget)
            lab = ttk.Label(right, text="", style="Hint.TLabel")
            lab.grid(row=r, column=2, sticky="w")
            self.unit_labels.append((lab, suffix))

        with_unit(1, "Steps per unit", self.entry(right, lambda: j()["scale"], lambda v: self.set_joint("scale", v), low=0.001), "steps per %s")
        with_unit(2, "Top speed", self.entry(right, lambda: j()["vel"], lambda v: self.set_joint("vel", v), low=0.0001), "%s per second")
        with_unit(3, "Acceleration", self.entry(right, lambda: j()["acc"], lambda v: self.set_joint("acc", v), low=0.0001), "%s per second squared")
        with_unit(4, "Travel, lowest", self.entry(right, lambda: j()["lo"], lambda v: self.set_joint("lo", v)), "%s")
        with_unit(5, "Travel, highest", self.entry(right, lambda: j()["hi"], lambda v: self.set_joint("hi", v)), "%s")
        self.check(right, "Reverse the direction of this motor", lambda: j()["reverse"],
                   lambda v: self.set_joint("reverse", v)).grid(row=6, column=0, columnspan=3, sticky="w", pady=(4, 0))
        self.rate_note = ttk.Label(right, text="", wraplength=520, justify="left")
        self.rate_note.grid(row=7, column=0, columnspan=3, sticky="w", pady=(8, 2))

        self.graph = tk.Canvas(right, width=520, height=100, background="white", highlightthickness=1,
                               highlightbackground="#bbbbbb")
        self.graph.grid(row=8, column=0, columnspan=3, sticky="w", pady=4)

        test = ttk.LabelFrame(right, text="Try it", padding=8)
        test.grid(row=9, column=0, columnspan=3, sticky="we", pady=(8, 0))
        self.minus = ttk.Button(test, text="<<  hold to move -", width=18)
        self.minus.grid(row=0, column=0)
        self.plus = ttk.Button(test, text="hold to move +  >>", width=18)
        self.plus.grid(row=0, column=1, padx=6)
        for button, direction in ((self.minus, -1), (self.plus, 1)):
            button.bind("<ButtonPress-1>", lambda e, d=direction: self.jog_start(d))
            button.bind("<ButtonRelease-1>", lambda e: self.jog_stop())
        ttk.Label(test, text="at").grid(row=0, column=2, padx=(10, 2))
        self.jog_percent = tk.IntVar(value=25)
        ttk.Scale(test, from_=1, to=100, variable=self.jog_percent, length=120,
                  command=lambda v: self.jog_label.configure(text="%d %% of top speed" % float(v))).grid(row=0, column=3)
        self.jog_label = ttk.Label(test, text="25 % of top speed")
        self.jog_label.grid(row=0, column=4, padx=6)
        self.moved = ttk.Label(test, text="")
        self.moved.grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Button(test, text="Call this 0", command=self.jog_zero).grid(row=1, column=3, sticky="w", pady=(8, 0))
        cal = ttk.Frame(test)
        cal.grid(row=2, column=0, columnspan=5, sticky="w", pady=(8, 0))
        ttk.Label(cal, text="If a rule says it really moved").pack(side="left")
        self.really = tk.StringVar()
        ttk.Entry(cal, textvariable=self.really, width=9).pack(side="left", padx=6)
        ttk.Button(cal, text="correct the steps per unit", command=self.calibrate).pack(side="left")
        self.test_note = ttk.Label(test, text="", style="Hint.TLabel", wraplength=500, justify="left")
        self.test_note.grid(row=3, column=0, columnspan=5, sticky="w", pady=(6, 0))
        self.loaders.append(self.load_motor)

    def load_motor_list(self):
        self.motor_list.delete(0, "end")
        for j in self.m["joints"]:
            self.motor_list.insert("end", " %s%s   (output %s)" % (j["letter"], ", second motor" if j["second"] else "",
                                                                   LETTERS[j["output"]]))
        self.motor_list.selection_clear(0, "end")
        self.motor_list.selection_set(self.selected)

    def pick_motor(self, _event=None):
        chosen = self.motor_list.curselection()
        if chosen and chosen[0] != self.selected:
            self.focus_set()                # (an entry being typed in stores its value first)
            self.update()
            self.selected = chosen[0]
            self.show()

    def set_joint(self, key, value):
        j = self.joint()
        j[key] = value
        if key == "hi" and j["hi"] < j["lo"]:
            j["lo"] = j["hi"]
        if key == "lo" and j["lo"] > j["hi"]:
            j["hi"] = j["lo"]
        # the motors of one axis go at one speed and have one travel
        for other in self.m["joints"]:
            if other is not j and other["letter"] == j["letter"]:
                for k in ("scale", "vel", "acc", "lo", "hi"):
                    other[k] = j[k]

    def limit_rate(self):
        """Steps a second the controller can make on one axis with the pulse length set here."""
        pulse = self.m["pulse"]
        return int(1e6 / max(10, 3 * pulse + 4))

    def load_motor(self):
        j = self.joint()
        u = self.unit(j)
        self.motor_title["text"] = "%s axis%s, on output %s" % (j["letter"], ", second motor" if j["second"] else "", LETTERS[j["output"]])
        for lab, suffix in self.unit_labels:
            lab["text"] = suffix % u
        rate, limit = abs(j["vel"] * j["scale"]), self.limit_rate()
        per_minute = "%s %s a minute" % (nice(round(j["vel"] * 60, 3)), u)
        if rate > limit:
            self.rate_note.configure(style="Bad.TLabel", text=(
                "Top speed is %s: that needs %s steps a second, and with a %d us pulse the controller makes %s. "
                "Lower the speed or the steps per %s, or shorten the pulse." % (
                    per_minute, format(int(rate), ","), self.m["pulse"], format(limit, ","), u)))
        else:
            self.rate_note.configure(style="TLabel", text=(
                "Top speed is %s, which is %s steps a second (the controller makes up to %s with a %d us pulse). "
                "It is reached in %.2f seconds." % (per_minute, format(int(rate), ","), format(limit, ","),
                                                    self.m["pulse"], j["vel"] / j["acc"])))
        self.draw_graph(j)
        if not self.ctl:
            self.test_note["text"] = "Connect to the controller (first page) to run this motor from here."
        else:
            self.test_note["text"] = ("The motor runs with the values above, as LinuxCNC will run it. Mind the ends "
                                      "of the travel: nothing stops it here but letting go.")
        self.show_moved()

    def draw_graph(self, j):
        """Speed against time for a move that reaches top speed: how long the ramps are."""
        g = self.graph
        g.delete("all")
        w, h, pad = 520, 100, 26
        ramp = j["vel"] / j["acc"]
        total = max(2 * ramp + max(ramp, 0.2), 0.001)
        x = lambda t: pad + (w - 2 * pad) * t / total       # noqa: E731
        y = lambda v: h - pad + (2 * pad - h + 8) * v       # noqa: E731
        g.create_line(pad, h - pad, w - pad + 6, h - pad, fill="#888888")
        g.create_line(pad, h - pad, pad, 10, fill="#888888")
        g.create_line(x(0), y(0), x(ramp), y(1), x(total - ramp), y(1), x(total), y(0), fill="#1565c0", width=2)
        g.create_text(pad + 4, 12, anchor="w", text="%s %s/s" % (nice(j["vel"]), self.unit(j)), fill="#333333")
        g.create_text(x(ramp), h - pad + 10, text="%.2f s" % ramp, fill="#333333")
        g.create_text(w - pad, h - pad + 10, anchor="e", text="time", fill="#888888")
        g.create_line(x(ramp), y(1), x(ramp), h - pad, fill="#bbbbbb", dash=(2, 3))

    def push_motor(self, j):
        """Gives the controller this motor's values, as LinuxCNC's driver will."""
        c, o = self.ctl, j["output"]
        c.set("step_pulse_us", self.m["pulse"])
        c.set("dir_setup_us", self.m["dir_setup"])
        c.set("invert_enable", 1 if self.m["enable_invert"] else 0)
        mask = c.get("invert_dir")
        c.set("invert_dir", (mask | (1 << o)) if j["reverse"] else (mask & ~(1 << o)))
        c.set("max_rate_x", abs(j["vel"] * j["scale"]), axis=o)
        c.set("accel_x", abs(j["acc"] * j["scale"]), axis=o)

    def jog_start(self, direction):
        if not self.ctl:
            self.bell()
            return
        j = self.joint()
        if abs(j["vel"] * j["scale"]) > self.limit_rate():
            self.bell()
            return
        try:
            s = self.ctl.status
            if s and s["state"] in (6, 7, 8):
                self.ctl.reset()
            self.push_motor(j)
        except link.LinkError as e:
            self.lost(e)
            return
        self.jog = (j["output"], direction, abs(j["vel"] * j["scale"]) * self.jog_percent.get() / 100.0)
        self.jog_sent = 0.0

    def jog_stop(self):
        if self.jog and self.ctl:
            self.ctl.stop(self.jog[0])
        self.jog = None

    def jog_zero(self):
        if self.ctl and self.ctl.status:
            self.zero = list(self.ctl.status["position"])
        self.show_moved()

    def show_moved(self):
        j = self.joint()
        if not (self.ctl and self.ctl.status):
            self.moved["text"] = ""
            return None
        steps = self.ctl.status["position"][j["output"]] - self.zero[j["output"]]
        if j["reverse"]:
            pass                            # (the controller counts the way the motor is told to turn)
        travel = steps / j["scale"]
        self.moved["text"] = "Moved %s %s  (%d steps)" % (nice(round(travel, 4)), self.unit(j), steps)
        return travel

    def calibrate(self):
        j = self.joint()
        shown = self.show_moved()
        try:
            real = abs(float(self.really.get().replace(",", ".")))
        except ValueError:
            real = 0.0
        if not shown or real <= 0.0 or abs(shown) < 1e-9:
            messagebox.showinfo("Tiggy set-up", "Press \"Call this 0\", move the axis a good distance with the "
                                                "buttons, measure how far it really went and type that in.")
            return
        new = j["scale"] * abs(shown) / real
        if messagebox.askokcancel("Tiggy set-up", "It was told to move %s and moved %s.\n\nSteps per %s: %s now, %s corrected. "
                                                  "Use the corrected value?" % (nice(round(abs(shown), 4)), nice(real), self.unit(j),
                                                                              nice(j["scale"]), nice(round(new, 4)))):
            self.set_joint("scale", round(new, 4))
            self.really.set("")
            self.jog_zero()
            self.show()

    # ================================================================= Switches
    def page_switches(self, f):
        ttk.Label(f, text="Switches", style="Title.TLabel").grid(row=0, column=0, columnspan=6, sticky="w")
        ttk.Label(f, style="Hint.TLabel", wraplength=760, justify="left",
                  text="Each motor output has a switch input of the same name. A switch connects its input to 0 V; "
                       "the lamp shows the input as the controller sees it now, so operate each switch by hand and "
                       "watch its lamp."
                  ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(2, 10))
        self.switch_frame = ttk.Frame(f)
        self.switch_frame.grid(row=2, column=0, columnspan=6, sticky="w")
        self.switch_rows = []
        self.loaders.append(self.load_switches)

        other = ttk.Frame(f)
        other.grid(row=3, column=0, columnspan=6, sticky="w", pady=(18, 0))
        ttk.Label(other, text="Probe and E-stop", style="Title.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 4))
        self.probe_lamp = self.light(other)
        self.probe_lamp.grid(row=1, column=0)
        ttk.Label(other, text="Probe input").grid(row=1, column=1, sticky="w", padx=6)
        self.check(other, "opens when it touches", lambda: self.m["probe_nc"],
                   lambda v: self.m.update(probe_nc=v)).grid(row=1, column=2, sticky="w", padx=10)
        self.estop_lamp = self.light(other)
        self.estop_lamp.grid(row=2, column=0)
        self.check(other, "An E-stop button is wired to the controller's E-stop input", lambda: self.m["estop"],
                   lambda v: self.m.update(estop=v, estop_nc=v and self.m["estop_nc"])).grid(row=2, column=1, sticky="w", padx=6)
        self.check(other, "opens when pressed (the usual kind)", lambda: self.m["estop_nc"],
                   lambda v: self.m.update(estop_nc=v, estop=v or self.m["estop"])).grid(row=2, column=2, sticky="w", padx=10)
        aux = ttk.Frame(other)
        aux.grid(row=3, column=0, columnspan=4, sticky="w", pady=(10, 0))
        ttk.Label(aux, text="Other inputs:").pack(side="left")
        self.aux_lamps = []
        for n in range(4):
            lamp = self.light(aux)
            lamp.pack(side="left", padx=(10, 2))
            ttk.Label(aux, text="%d" % (n + 1)).pack(side="left")
            self.aux_lamps.append(lamp)
        ttk.Label(aux, text="  (free for your own use in LinuxCNC: tiggy.input.0 to .3)", style="Hint.TLabel").pack(side="left")

    def load_switches(self):
        for child in self.switch_frame.winfo_children():
            child.destroy()
        self.switch_rows = []
        f = self.switch_frame
        for c, title in enumerate(("Now", "Axis", "Input", "The switch is", "", "Homes at")):
            ttk.Label(f, text=title, style="Hint.TLabel").grid(row=0, column=c, sticky="w", padx=(0, 12))
        for r, j in enumerate(self.m["joints"], 1):
            lamp = self.light(f)
            lamp.grid(row=r, column=0, pady=3)
            ttk.Label(f, text="%s%s" % (j["letter"], " (second motor)" if j["second"] else "")).grid(row=r, column=1, sticky="w", padx=(0, 12))
            ttk.Label(f, text=LETTERS[j["output"]]).grid(row=r, column=2, sticky="w", padx=(0, 12))
            use = "both" if j["home_switch"] and j["limit_switch"] else ("home" if j["home_switch"] else
                                                                          ("limit" if j["limit_switch"] else "none"))
            var = tk.StringVar(value=dict(USES)[use])
            box = ttk.Combobox(f, textvariable=var, state="readonly", width=16, values=[t for _k, t in USES])
            box.grid(row=r, column=3, padx=(0, 12))
            box.bind("<<ComboboxSelected>>", lambda e, j=j, var=var: self.set_switch(j, var.get()))
            nc = tk.BooleanVar(value=j["switch_nc"])
            ttk.Checkbutton(f, text="opens when operated", variable=nc,
                            command=lambda j=j, nc=nc: self.set_switch_nc(j, nc.get())).grid(row=r, column=4, sticky="w", padx=(0, 12))
            end = tk.StringVar(value="the high end" if j["home_positive"] else "the low end")
            where = ttk.Combobox(f, textvariable=end, state="readonly", width=12, values=["the low end", "the high end"])
            where.grid(row=r, column=5)
            where.bind("<<ComboboxSelected>>", lambda e, j=j, end=end: j.update(home_positive=end.get() == "the high end"))
            if not j["home_switch"]:
                where.state(["disabled"])
            self.switch_rows.append((lamp, j))

    def set_switch(self, j, text):
        use = next(k for k, t in USES if t == text)
        j["home_switch"] = use in ("home", "both")
        j["limit_switch"] = use in ("limit", "both")
        j["switch_input"] = j["output"]
        if use == "none":
            j["switch_nc"] = False
        self.show()

    def set_switch_nc(self, j, nc):
        j["switch_nc"] = nc
        self.push_inversions()

    def push_inversions(self):
        """Lets the controller show a normally-closed switch as off while it is closed."""
        if not self.ctl:
            return
        mask = 0
        for j in self.m["joints"]:
            if j["switch_nc"]:
                mask |= 1 << j["switch_input"]
        try:
            self.ctl.set("invert_limit", mask)
            self.ctl.set("invert_home", mask)
            self.ctl.set("invert_probe", 1 if self.m["probe_nc"] else 0)
            self.ctl.set("invert_estop", 1 if self.m["estop_nc"] else 0)
        except link.LinkError as e:
            self.lost(e)

    # ================================================================== Spindle
    def page_spindle(self, f):
        ttk.Label(f, text="Spindle", style="Title.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 4))
        self.row(f, 1, "Switched by the controller", self.radios(
            f, [("n", "not at all"), ("o", "on and off"), ("p", "speed by PWM")],
            lambda: self.m["spindle"], lambda v: self.m.update(spindle=v)))
        self.max_rpm = self.entry(f, lambda: self.m["max_rpm"], lambda v: self.m.update(max_rpm=v), low=1.0)
        self.row(f, 2, "Speed at full PWM", self.max_rpm, "rpm")
        self.pwm_freq = self.entry(f, lambda: self.m["pwm_freq"], lambda v: self.m.update(pwm_freq=v), kind=int, low=50, high=50000)
        self.row(f, 3, "PWM frequency", self.pwm_freq, "Hz")
        run = ttk.Frame(f)
        run.grid(row=4, column=1, columnspan=2, sticky="w", padx=8, pady=(2, 0))
        self.spin = tk.BooleanVar()
        ttk.Checkbutton(run, text="Run it now at", variable=self.spin, command=self.run_spindle).pack(side="left")
        self.spin_rpm = tk.StringVar(value="1000")
        e = ttk.Entry(run, textvariable=self.spin_rpm, width=7)
        e.pack(side="left", padx=6)
        e.bind("<Return>", lambda ev: self.run_spindle())
        ttk.Label(run, text="rpm").pack(side="left")

        ttk.Label(f, text="Spindle sensor, for screw cutting", style="Title.TLabel").grid(row=5, column=0, columnspan=3, sticky="w", pady=(18, 4))
        self.sensor_kind = tk.StringVar(value=link.SENSOR_NONE)
        kinds = ttk.Frame(f)
        kinds.grid(row=6, column=0, columnspan=3, sticky="w")
        for value, text in SENSORS:
            ttk.Radiobutton(kinds, text=text[0].upper() + text[1:], value=value, variable=self.sensor_kind).pack(side="left", padx=(0, 14))
        self.sensor_lines = tk.StringVar(value="1024")
        lines = ttk.Frame(f)
        lines.grid(row=7, column=0, columnspan=3, sticky="w", pady=4)
        ttk.Label(lines, text="Lines per revolution of the encoder:").pack(side="left")
        ttk.Entry(lines, textvariable=self.sensor_lines, width=7).pack(side="left", padx=6)
        self.terminals = tk.IntVar(value=link.TERMINALS_SIX)
        term = ttk.Frame(f)
        term.grid(row=8, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Label(term, text="The controller's terminals:").pack(side="left")
        ttk.Radiobutton(term, text="six axes", value=link.TERMINALS_SIX, variable=self.terminals).pack(side="left", padx=10)
        ttk.Radiobutton(term, text="four axes; B and C are the encoder, inputs, outputs and charge pump",
                        value=link.TERMINALS_FOUR, variable=self.terminals).pack(side="left")
        ttk.Label(f, style="Hint.TLabel", wraplength=760, justify="left",
                  text="These two are kept in the controller itself. A sensor giving one pulse per revolution has a "
                       "terminal with either choice; an encoder needs the four-axis terminals. The controller "
                       "restarts to take a change."
                  ).grid(row=9, column=0, columnspan=3, sticky="w", pady=(4, 4))
        send = ttk.Frame(f)
        send.grid(row=10, column=0, columnspan=3, sticky="w")
        ttk.Button(send, text="Send to the controller", command=self.send_sensor).pack(side="left")
        self.sensor_now = ttk.Label(send, text="")
        self.sensor_now.pack(side="left", padx=12)
        self.check(f, "Wire LinuxCNC for screw cutting (G33, G76) with this sensor", lambda: self.m["threading"],
                   lambda v: self.m.update(threading=v)).grid(row=11, column=0, columnspan=3, sticky="w", pady=(10, 0))

        ttk.Label(f, text="Other", style="Title.TLabel").grid(row=12, column=0, columnspan=3, sticky="w", pady=(18, 4))
        self.row(f, 13, "Charge pump", self.entry(f, lambda: self.m["pump"], lambda v: self.m.update(pump=v),
                                                  kind=int, low=0, high=50000), "Hz while the machine is on (0: not used)")
        self.check(f, "Ask on screen at a tool change", lambda: self.m["manual_toolchange"],
                   lambda v: self.m.update(manual_toolchange=v)).grid(row=14, column=0, columnspan=3, sticky="w")
        self.loaders.append(self.load_spindle)

    def load_spindle(self):
        pwm = self.m["spindle"] == "p"
        for w in (self.max_rpm, self.pwm_freq):
            w.state(["!disabled"] if pwm else ["disabled"])
        if self.ctl and getattr(self, "on_controller", None) and not getattr(self, "sensor_shown", False):
            now = self.on_controller        # show what the controller has, once after connecting
            self.sensor_kind.set(now["kind"])
            if now["kind"].startswith("encoder"):
                self.sensor_lines.set(str(now["lines"]))
            self.terminals.set(now["terminals"])
            self.sensor_shown = True
        if not self.ctl:
            self.sensor_shown = False
            self.sensor_now["text"] = "(connect to the controller to read and set these)"

    def run_spindle(self):
        if not self.ctl:
            self.spin.set(False)
            self.bell()
            return
        try:
            rpm = max(0, int(float(self.spin_rpm.get())))
        except ValueError:
            rpm = 0
        try:
            c = self.ctl
            c.set("pwm", 1 if self.m["spindle"] == "p" else 0)
            c.set("pwm_hz", self.m["pwm_freq"])
            c.set("pwm_max_rpm", self.m["max_rpm"])
            s = c.status
            if s and s["state"] in (6, 7, 8):
                c.reset()
            c.out.update(spindle=1 if self.spin.get() else 0, rpm=min(rpm, 65535))
        except link.LinkError as e:
            self.lost(e)

    def send_sensor(self):
        if not self.ctl:
            messagebox.showinfo("Tiggy set-up", "Connect to the controller first (first page).")
            return
        kind = self.sensor_kind.get()
        try:
            lines = int(float(self.sensor_lines.get()))
        except ValueError:
            lines = 0
        if kind.startswith("encoder") and not 1 <= lines <= 20000:
            messagebox.showinfo("Tiggy set-up", "Give the encoder's lines per revolution (as printed on it), 1 to 20000.")
            return
        if any(j["output"] >= 4 for j in self.m["joints"]) and self.terminals.get() == link.TERMINALS_FOUR:
            messagebox.showinfo("Tiggy set-up", "This machine has a motor on output B or C (Machine page): the "
                                                "four-axis terminals have no such outputs.")
            return
        self.spin.set(False)
        self.config(cursor="watch")
        self.sensor_now["text"] = "The controller is restarting..."
        self.update()
        try:
            c = self.ctl
            c.out.update(spindle=0, rpm=0)
            c.set_sensor(kind, lines)
            c.set("terminals", self.terminals.get())
            c.save()
            c.restarted()
            c.set("limit_mask", 0)
        except link.LinkError as e:
            self.config(cursor="")
            self.lost(e)
            messagebox.showerror("Tiggy set-up", str(e))
            return
        self.config(cursor="")
        self.describe()
        now = self.on_controller
        if now["kind"] != link.SENSOR_NONE and not (self.ctl.info["capabilities"] & link.CAP_SENSOR):
            messagebox.showwarning("Tiggy set-up", "The controller has no terminals for that sensor with these "
                                                   "terminals. An encoder needs the four-axis terminals.")
        if now["kind"] != link.SENSOR_NONE and self.ctl.info["capabilities"] & link.CAP_SPINDLE_SYNC:
            self.m["threading"] = True
        self.show()

    # =================================================================== Finish
    def page_finish(self, f):
        ttk.Label(f, text="What will be written", style="Title.TLabel").pack(anchor="w")
        self.summary = tk.Text(f, height=20, width=96, wrap="word", relief="flat", background=self.cget("background"),
                               font=("TkFixedFont", 10))
        self.summary.pack(fill="both", expand=True, pady=6)
        self.loaders.append(self.load_summary)

    def problems(self):
        m, found = self.m, []
        if not m["joints"]:
            found.append("No motor output has an axis (Machine page).")
        limit = self.limit_rate()
        for j in m["joints"]:
            if abs(j["vel"] * j["scale"]) > limit:
                found.append("%s: top speed needs %s steps a second; the controller makes %s with a %d us pulse (Motors page)."
                             % (j["letter"], format(int(abs(j["vel"] * j["scale"])), ","), format(limit, ","), m["pulse"]))
        if m["threading"] and m["spindle"] == "n":
            found.append("Screw cutting needs LinuxCNC to know the spindle is running: let the controller switch it "
                         "(Spindle page).")
        return found

    def load_summary(self):
        m = self.m
        u = "inch" if m["inch"] else "mm"
        lines = ["%s: a %s, in %s" % (m["name"], "lathe" + (" (X as diameter)" if m["diameter"] else "") if m["lathe"]
                                    else "mill or router", "inches" if m["inch"] else "millimetres"),
                 "Controller: %s; %d ms of motion queued in it" % (
                     "looked for each time LinuxCNC starts" if m["address"] == "auto" else "at " + m["address"], m["buffer_ms"]),
                 ""]
        for j in m["joints"]:
            uj = "degree" if j["rotary"] else u
            use = ("home and limit switch" if j["home_switch"] and j["limit_switch"] else "home switch" if j["home_switch"]
                   else "limit switch" if j["limit_switch"] else "no switch")
            lines.append("%s%-2s output %s  %9s steps/%s  %7s %s/s  %7s %s/s2  travel %s to %s  %s%s" % (
                j["letter"], "2" if j["second"] else "", LETTERS[j["output"]], nice(j["scale"]), uj, nice(j["vel"]), uj,
                nice(j["acc"]), uj, nice(j["lo"]), nice(j["hi"]), use, ", reversed" if j["reverse"] else ""))
        lines += ["",
                  "Step pulse %d us, direction set-up %d us%s" % (m["pulse"], m["dir_setup"], ", enable active low" if m["enable_invert"] else ""),
                  "Spindle: %s%s" % ({"n": "not switched by the controller", "o": "on and off",
                                     "p": "PWM, %s rpm at full, %d Hz" % (nice(m["max_rpm"]), m["pwm_freq"])}[m["spindle"]],
                                    "; wired for screw cutting (G33, G76)" if m["threading"] else ""),
                  "E-stop button on the controller: %s.  Charge pump: %s." % (
                      ("yes" + (", opens when pressed" if m["estop_nc"] else "")) if m["estop"] else "no",
                      "%d Hz" % m["pump"] if m["pump"] else "not used"),
                  "",
                  "Folder: %s" % os.path.join(CONFIGS, gen.folder_name(m["name"]))]
        bad = self.problems()
        if bad:
            lines += ["", "TO PUT RIGHT FIRST:"] + ["  - " + b for b in bad]
        self.summary.configure(state="normal")
        self.summary.delete("1.0", "end")
        self.summary.insert("1.0", "\n".join(lines))
        self.summary.configure(state="disabled")

    def save(self, start=False):
        self.focus_set()                    # (an entry being typed in stores its value first)
        self.update()
        m = self.m
        bad = self.problems()
        if bad:
            messagebox.showwarning("Tiggy set-up", "Not saved yet:\n\n" + "\n\n".join(bad))
            return
        folder = os.path.join(CONFIGS, gen.folder_name(m["name"]))
        ini = os.path.join(folder, gen.folder_name(m["name"]) + ".ini")
        if os.path.exists(ini) and os.path.abspath(folder) != os.path.abspath(self.folder or ""):
            if not messagebox.askokcancel("Tiggy set-up", "There is already a machine called %s.\n\nReplace it?" % m["name"]):
                return
        if os.path.exists(ini):             # keep what was there, once per run of LinuxCNC's own files
            for name in (os.path.basename(ini), "tiggy.hal"):
                old = os.path.join(folder, name)
                if os.path.exists(old):
                    with open(old, "rb") as a, open(old + ".before", "wb") as b:
                        b.write(a.read())
        try:
            ini = gen.write_config(m, folder)
        except OSError as e:
            messagebox.showerror("Tiggy set-up", "Could not write the configuration:\n%s" % e)
            return
        self.folder = folder
        self.fill_chooser()
        self.which.set(os.path.basename(folder))
        self.note["text"] = "Saved in %s" % folder
        if start:
            self.disconnect()
            time.sleep(0.5)
            try:
                subprocess.Popen(["linuxcnc", ini], start_new_session=True,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError as e:
                messagebox.showerror("Tiggy set-up", "LinuxCNC could not be started:\n%s" % e)
                return
            self.destroy()

    def close(self):
        self.disconnect()
        self.destroy()

    # ---- twenty times a second
    def tick(self):
        c = self.ctl
        if c:
            try:
                s = c.poll()
                if self.jog and time.monotonic() - self.jog_sent > 0.1:
                    self.jog_sent = time.monotonic()
                    c.jog(self.jog[0], self.jog[1], self.jog[2])
            except link.LinkError as e:
                self.lost(e)
                s = None
                c = None
        if c:
            if s and not getattr(self, "zeroed", True):
                self.zero = list(s["position"])
                self.zeroed = True
            self.lamp.itemconfigure(self.lamp_dot, fill=ON if s else "#e0a030")
            self.state["text"] = "Controller: %s at %s, firmware %s - %s" % (
                c.info["name"], c.address, c.info["firmware"], c.state_text())
        elif not self.state["text"].startswith("Controller: the") and not self.state["text"].startswith("Controller: no answer"):
            self.lamp.itemconfigure(self.lamp_dot, fill=OFF)
            if not self.state["text"].startswith("Controller: not") and "lost" not in self.state["text"]:
                self.state["text"] = "Controller: not connected"
        s = c.status if c else None
        for lamp, j in getattr(self, "switch_rows", []):
            self.shine(lamp, None if not s else bool(s["switches"] >> j["switch_input"] & 1))
        self.shine(self.probe_lamp, None if not s else s["probe"])
        self.shine(self.estop_lamp, None if not s else s["estop"])
        for n, lamp in enumerate(self.aux_lamps):
            self.shine(lamp, None if not s else bool(s["inputs"] >> n & 1))
        if s:
            self.show_moved()
            if getattr(self, "on_controller", None):
                now = self.on_controller
                text = "On the controller: %s" % dict(SENSORS)[now["kind"]]
                if now["kind"] != link.SENSOR_NONE:
                    text += " - now %d rpm, %d turns counted" % (s["rpm"], s["turns"])
                self.sensor_now["text"] = text
        self.after(50, self.tick)


def main():
    folder = None
    if len(sys.argv) > 1:
        folder = os.path.abspath(sys.argv[1])
        if os.path.isfile(folder):
            folder = os.path.dirname(folder)
    App(folder).mainloop()


if __name__ == "__main__":
    main()
