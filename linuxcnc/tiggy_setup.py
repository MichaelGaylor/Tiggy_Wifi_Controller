#!/usr/bin/env python3
"""
Tiggy motion controller - LinuxCNC configuration wizard

Asks about the machine and writes a complete LinuxCNC configuration for it:

    python3 tiggy_setup.py               answer the questions
    python3 tiggy_setup.py --defaults    a plain 3-axis machine without questions
    python3 tiggy_setup.py --defaults --lathe      a plain lathe (X and Z)

(tiggy_config.py asks the same in windows, with the controller connected.)

The result is a folder in ~/linuxcnc/configs with
    <name>.ini      the machine: speeds, travels, homing, and in its [TIGGY]
                    section everything about the controller
    tiggy.hal       how LinuxCNC and the controller are connected
    postgui.hal     for your own additions
    tool.tbl        tool table
    tiggy.json      the answers, for tiggy_config.py to open again

Everything can be changed afterwards by editing the INI file; the wizard is
only the quick way to a working start.
"""

import argparse
import json
import os
import sys

LETTERS = "XYZABC"


# ------------------------------------------------------------------ asking

class Asker:
    def __init__(self, use_defaults, lathe=False, sensor=False):
        self.use_defaults = use_defaults
        self.lathe = lathe                  # what is offered as the kind of machine...
        self.sensor = sensor                # ...and for the spindle sensor

    def text(self, prompt, default):
        if self.use_defaults:
            return default
        try:
            answer = input("  %s [%s]: " % (prompt, default)).strip()
        except EOFError:
            answer = ""
        return answer if answer else default

    def number(self, prompt, default, lowest=None, highest=None):
        while True:
            answer = self.text(prompt, "%g" % default)
            try:
                value = float(answer)
            except ValueError:
                print("    Please give a number.")
                continue
            if (lowest is not None and value < lowest) or (highest is not None and value > highest):
                print("    Please give a number from %g to %g." % (lowest, highest))
                continue
            return value

    def whole(self, prompt, default, lowest, highest):
        return int(round(self.number(prompt, default, lowest, highest)))

    def yes(self, prompt, default):
        while True:
            answer = self.text(prompt + " (y/n)", "y" if default else "n").lower()
            if answer in ("y", "yes"):
                return True
            if answer in ("n", "no"):
                return False
            print("    Please answer y or n.")

    def choice(self, prompt, options, default):
        """options: list of (key, description)."""
        if not self.use_defaults:
            print("  " + prompt)
            for key, description in options:
                print("      %s  %s" % (key, description))
        keys = [k for k, _ in options]
        while True:
            answer = self.text("Your choice", default).lower()
            if answer in keys:
                return answer
            print("    Please give one of: " + ", ".join(keys))


QUIET = False


def say(text=""):
    if not QUIET:
        print(text)


def heading(text):
    say("\n" + text)
    say("-" * len(text))


# --------------------------------------------------------------- questions

def travel_default(m, letter):
    """Lowest and highest position offered for a linear axis."""
    inch = m["inch"]
    if m.get("lathe"):
        if letter == "X":                   # (as a radius, from a little beyond the centre line)
            return (-0.4, 6.0) if inch else (-10.0, 150.0)
        return (-16.0, 0.4) if inch else (-400.0, 10.0)
    if letter == "Z":
        return (-4.0, 0.0) if inch else (-100.0, 0.0)
    return (0.0, 12.0) if inch else (0.0, 300.0)


def outputs(m):
    """The controller's motor output each joint is on, in the order of the joints."""
    return [j.get("output", n) for n, j in enumerate(m["joints"])]


def ask_machine(ask):
    m = {}
    heading("The machine")
    m["name"] = ask.text("Name of the machine", "Tiggy")
    say("  The controller is found on the network by itself. Give its address")
    say("  (for example 192.168.1.50) only if the search does not find it.")
    m["address"] = ask.text("Address of the controller", "auto")
    m["inch"] = ask.choice("Units", [("mm", "millimetres"), ("inch", "inches")], "mm") == "inch"
    unit = "inch" if m["inch"] else "mm"
    m["lathe"] = ask.choice("Kind of machine", [("m", "mill or router"), ("l", "lathe")],
                            "l" if ask.lathe else "m") == "l"
    m["diameter"] = False

    if m["lathe"]:
        say("  A lathe has X (the cross slide) on the controller's X output and Z (the carriage) on its Z output.")
        m["diameter"] = ask.yes("Are X positions given as diameters (G7), as on most lathes", True)
        letters = "XZ"
    else:
        letters = ask.text("Axes, in the order of the controller's motor outputs", "XYZ").upper().replace(" ", "")
        letters = "".join(c for c in letters if c in LETTERS)[:6] or "XYZ"
    gantry = ""
    if not m["lathe"] and len(letters) < 6 and ask.yes("Is there a second motor on one axis (a gantry)", False):
        gantry = ask.text("Which axis has the second motor", "Y").upper()[:1]
        if gantry not in letters:
            gantry = ""
        else:
            say("  The second %s motor goes on the controller's next free output (%s)."
                % (gantry, LETTERS[len(letters)]))
    m["joints"] = []
    for letter in letters + gantry:
        second = (letter == gantry and len(m["joints"]) >= len(letters))
        rotary = letter in "ABC"
        j = {"letter": letter, "second": second, "rotary": rotary}
        u = "degree" if rotary else unit
        # The motor output the joint is on: in the order asked, except that a lathe's Z is on the Z output
        j["output"] = LETTERS.index(letter) if m["lathe"] else len(m["joints"])
        heading("%s axis%s (controller output %s)" % (letter, ", second motor" if second else "",
                                                     LETTERS[j["output"]]))
        if second:
            first = next(x for x in m["joints"] if x["letter"] == letter)
            j.update({k: first[k] for k in ("scale", "vel", "acc", "lo", "hi")})
            j["reverse"] = ask.yes("Does this motor turn the other way round", True)
        else:
            j["scale"] = ask.number("Steps per %s" % u, 20000.0 if m["inch"] and not rotary else (88.8889 if rotary else 800.0), 0.001)
            j["vel"] = ask.number("Top speed, %s per second" % u, (2.0 if m["inch"] else 50.0) if not rotary else 90.0, 0.0001)
            j["acc"] = ask.number("Acceleration, %s per second squared" % u,
                                  (20.0 if m["inch"] else 500.0) if not rotary else 900.0, 0.0001)
            if rotary:
                j["lo"], j["hi"] = -99999.0, 99999.0
            else:
                lo, hi = travel_default(m, letter)
                j["lo"] = ask.number("Travel: lowest position, %s" % u, lo)
                j["hi"] = ask.number("Travel: highest position, %s" % u, hi, j["lo"])
            j["reverse"] = ask.yes("Reverse the direction of this motor", False)
        index = j["output"]                 # each motor output has a limit input of the same name
        j["switch_input"] = index
        if second and (first["home_switch"] or first["limit_switch"]) and not ask.yes(
                "Does this motor have a switch of its own, on the %s limit input (to square the gantry)"
                % LETTERS[index], False):
            # One switch for both motors: they are homed together on it.
            for k in ("home_switch", "limit_switch", "switch_nc", "home_positive", "switch_input"):
                j[k] = first[k]
            m["joints"].append(j)
            continue
        use = ask.choice("Switch on the controller's %s limit input:" % LETTERS[index],
                         [("n", "none"), ("h", "home switch"), ("l", "limit switch"),
                          ("b", "both: one switch for homing and as a limit")],
                         ("b" if first["limit_switch"] else "h") if second and first["home_switch"] else "n")
        j["home_switch"] = use in ("h", "b")
        j["limit_switch"] = use in ("l", "b")
        j["switch_nc"] = False
        j["home_positive"] = first["home_positive"] if second else (letter == "Z" or m["lathe"])
        if use != "n":
            j["switch_nc"] = ask.yes("Is it a normally-closed switch (opens when operated)", False)
        if j["home_switch"] and not second:
            j["home_positive"] = ask.yes("Is the home switch at the high end of the travel", letter == "Z" or m["lathe"])
        m["joints"].append(j)

    heading("Step signals")
    say("  Check your motor drivers' data sheet; 5 suits most.")
    m["pulse"] = ask.whole("Step pulse length, microseconds", 5, 1, 50)
    m["dir_setup"] = ask.whole("Direction set-up time, microseconds", 5, 0, 50)
    m["enable_invert"] = ask.yes("Is the drivers' enable signal active low", False)

    heading("Spindle")
    kind = ask.choice("How is the spindle switched?",
                      [("n", "not by the controller"), ("o", "on and off"), ("p", "speed by PWM")], "o")
    m["spindle"] = kind
    m["max_rpm"] = 24000.0
    m["pwm_freq"] = 1000
    if kind == "p":
        m["max_rpm"] = ask.number("Spindle speed at full PWM, rpm", 24000.0, 1.0)
        m["pwm_freq"] = ask.whole("PWM frequency, Hz", 1000, 1, 50000)
    say("  For screw cutting (G33, G76) the controller needs a sensor on the spindle: an encoder,")
    say("  or one pulse per revolution.  It is set up on the controller (terminal layout, kind of sensor).")
    m["threading"] = ask.yes("Is there such a sensor, for screw cutting", ask.sensor)

    heading("Safety")
    m["estop"] = ask.yes("Is an E-stop button wired to the controller's E-stop input", False)
    m["estop_nc"] = m["estop"] and ask.yes("Is it normally closed (opens when pressed)", True)
    m["pump"] = 0
    if ask.yes("Does your breakout board need a charge pump signal", False):
        m["pump"] = ask.whole("Charge pump frequency, Hz", 12500, 1, 50000)
    m["probe_nc"] = False

    heading("Extras")
    m["io_module"] = ""
    if ask.yes("Is there a Tiggy I/O expansion module", False):
        m["io_module"] = ask.text("Its address (auto to search for it)", "auto")
    m["manual_toolchange"] = ask.yes("Show a prompt on screen at tool changes", True)
    return m


# ------------------------------------------------------------------ output

def fmt(v):
    return ("%.6f" % v).rstrip("0").rstrip(".") if "." in ("%.6f" % v) else "%d" % v


def coordinates(m):
    return "".join(j["letter"] for j in m["joints"])


def write_ini(m, path, buffer_ms=200):
    unit = "inch" if m["inch"] else "mm"
    joints = m["joints"]
    linear = [j for j in joints if not j["rotary"]] or joints
    top = max(j["vel"] for j in linear)
    small = 0.04 if m["inch"] else 1.0
    lag = (buffer_ms + 150) / 1000.0        # the machine runs this far behind LinuxCNC, with margin
    axes_done = set()
    out = []
    w = out.append

    w("# LinuxCNC configuration for the Tiggy motion controller")
    w("# Written by tiggy_setup.py - edit freely.")
    w("")
    w("[EMC]")
    w("MACHINE = %s" % m["name"])
    w("VERSION = 1.1")
    w("DEBUG = 0")
    w("")
    w("[DISPLAY]")
    w("DISPLAY = axis")
    w("CYCLE_TIME = 0.100")
    w("POSITION_OFFSET = RELATIVE")
    w("POSITION_FEEDBACK = ACTUAL")
    w("MAX_FEED_OVERRIDE = 1.5")
    w("MAX_SPINDLE_OVERRIDE = 1.5")
    w("MIN_SPINDLE_OVERRIDE = 0.5")
    w("DEFAULT_LINEAR_VELOCITY = %s" % fmt(top * 0.25))
    w("MAX_LINEAR_VELOCITY = %s" % fmt(top))
    w("MIN_LINEAR_VELOCITY = 0")
    if any(j["rotary"] for j in joints):
        w("DEFAULT_ANGULAR_VELOCITY = %s" % fmt(max(j["vel"] for j in joints if j["rotary"]) * 0.25))
        w("MAX_ANGULAR_VELOCITY = %s" % fmt(max(j["vel"] for j in joints if j["rotary"])))
    if m.get("lathe"):
        w("LATHE = 1")
    w("INTRO_GRAPHIC = linuxcnc.gif")
    w("INTRO_TIME = 3")
    w("PROGRAM_PREFIX = ~/linuxcnc/nc_files")
    w("INCREMENTS = %s" % ("0.1in 0.05in 0.01in 0.005in 0.001in" if m["inch"] else "10mm 1mm 0.1mm 0.01mm"))
    w("")
    w("[KINS]")
    kins = "trivkins coordinates=%s" % coordinates(m)
    if any(j["second"] for j in joints):
        kins += " kinstype=BOTH"
    w("KINEMATICS = %s" % kins)
    w("JOINTS = %d" % len(joints))
    w("")
    w("[FILTER]")
    w("# Programs written for other controls (Mach3's .tap ...) are offered when opening a file")
    w("PROGRAM_EXTENSION = .tap,.nc,.cnc,.txt,.gcode G-code from other programs")
    w("PROGRAM_EXTENSION = .py Python Script")
    w("py = python3")
    w("")
    w("[TASK]")
    w("TASK = milltask")
    w("CYCLE_TIME = 0.010")
    w("")
    w("[RS274NGC]")
    w("PARAMETER_FILE = linuxcnc.var")
    plane = " G18 %s" % ("G7" if m.get("diameter") else "G8") if m.get("lathe") else ""
    w("RS274NGC_STARTUP_CODE = %s%s G40 G90 G94 G97 G64 P%s" % ("G20" if m["inch"] else "G21", plane,
                                                                "0.001" if m["inch"] else "0.025"))
    w("")
    w("[EMCMOT]")
    w("EMCMOT = motmod")
    w("COMM_TIMEOUT = 1.0")
    w("SERVO_PERIOD = 1000000")
    w("")
    w("[EMCIO]")
    w("EMCIO = io")
    w("CYCLE_TIME = 0.100")
    w("TOOL_TABLE = tool.tbl")
    w("")
    w("[HAL]")
    w("HALFILE = tiggy.hal")
    w("POSTGUI_HALFILE = postgui.hal")
    w("HALUI = halui")
    w("")
    w("[TIGGY]")
    w("# The controller.  These values are passed on to it every time LinuxCNC starts.")
    w("#")
    w("# Where it is on the network: auto finds it; otherwise its address, e.g. 192.168.1.50")
    w("ADDRESS = %s" % m["address"])
    if m["io_module"]:
        w("# The I/O expansion module, the same way")
        w("IO_MODULE = %s" % m["io_module"])
    w("# Length of a step pulse, and how long the direction signal is steady before a step,")
    w("# in microseconds (1 to 50).  A longer pulse lowers the highest step rate:")
    w("# 5 us gives 52,000 steps a second, 3 us 76,000, 2 us 100,000.")
    w("STEP_PULSE_US = %d" % m["pulse"])
    w("DIR_SETUP_US = %d" % m["dir_setup"])
    w("# 1 if the motor drivers are enabled by a low level")
    w("ENABLE_INVERT = %d" % (1 if m["enable_invert"] else 0))
    w("# Switch the motor drivers off after standing still for this many milliseconds (0 = never)")
    w("IDLE_RELEASE_MS = 0")
    w("# Motion kept queued in the controller, milliseconds (120 to 400).  The machine runs")
    w("# this far behind LinuxCNC.  More rides out a poorer WiFi link; less makes jogging and")
    w("# feed hold answer sooner.  If you raise it, raise FERROR and MIN_FERROR of every joint")
    w("# to at least (this + 150) / 1000 x the joint's MAX_VELOCITY.")
    w("BUFFER_MS = %d" % buffer_ms)
    w("# How long a switch, the probe or the E-stop input must be steady to count, milliseconds (1 to 100)")
    w("DEBOUNCE_MS = 5")
    w("# Inputs are on when connected to ground (a switch that closes).  Put 1 for an input")
    w("# that is on when the connection opens (a normally-closed switch).")
    for i in range(6):
        nc = any(j["switch_input"] == i and j["switch_nc"] for j in joints)
        w("SWITCH_%d_INVERT = %d" % (i, 1 if nc else 0))
    w("PROBE_INVERT = %d" % (1 if m["probe_nc"] else 0))
    w("ESTOP_INVERT = %d" % (1 if m["estop_nc"] else 0))
    w("# Spindle: 1 = speed by PWM, 0 = just on and off")
    w("SPINDLE_PWM = %d" % (1 if m["spindle"] == "p" else 0))
    w("SPINDLE_PWM_FREQ = %d" % m["pwm_freq"])
    w("SPINDLE_MAX_RPM = %s" % fmt(m["max_rpm"]))
    w("SPINDLE_MIN_DUTY = 0")
    w("# Charge pump output, Hz (0 = not used).  It runs while the machine is switched on.")
    w("CHARGE_PUMP_FREQ = %d" % m["pump"])
    w("")
    w("[TRAJ]")
    w("COORDINATES = %s" % " ".join(coordinates(m)))
    w("LINEAR_UNITS = %s" % unit)
    w("ANGULAR_UNITS = degree")
    w("DEFAULT_LINEAR_VELOCITY = %s" % fmt(top * 0.25))
    w("MAX_LINEAR_VELOCITY = %s" % fmt(top))
    w("")

    outs = outputs(m)
    for n, j in enumerate(joints):
        if j["letter"] not in axes_done:
            axes_done.add(j["letter"])
            w("[AXIS_%s]" % j["letter"])
            w("MAX_VELOCITY = %s" % fmt(j["vel"]))
            w("MAX_ACCELERATION = %s" % fmt(j["acc"]))
            w("MIN_LIMIT = %s" % fmt(j["lo"]))
            w("MAX_LIMIT = %s" % fmt(j["hi"]))
            w("")
        ferror = max(j["vel"] * lag, small)
        w("[JOINT_%d]" % n)
        w("# %s%s, on the controller's %s output" % (j["letter"], " (second motor)" if j["second"] else "", LETTERS[outs[n]]))
        w("TYPE = %s" % ("ANGULAR" if j["rotary"] else "LINEAR"))
        w("SCALE = %s" % fmt(j["scale"]))
        w("STEP_INVERT = 0")
        w("DIR_INVERT = %d" % (1 if j["reverse"] else 0))
        w("MAX_VELOCITY = %s" % fmt(j["vel"]))
        w("MAX_ACCELERATION = %s" % fmt(j["acc"]))
        w("MIN_LIMIT = %s" % fmt(j["lo"]))
        w("MAX_LIMIT = %s" % fmt(j["hi"]))
        w("# The controller plays the motion a fraction of a second after LinuxCNC commands it")
        w("# ([TIGGY]BUFFER_MS), so the following error allowed must cover that much travel.")
        w("FERROR = %s" % fmt(ferror))
        w("MIN_FERROR = %s" % fmt(ferror))
        if j["home_switch"]:
            sign = 1.0 if j["home_positive"] else -1.0
            edge = j["hi"] if j["home_positive"] else j["lo"]
            search = min(j["vel"] * 0.25, 10.0 * small)
            w("# Homing.  The axis runs on for about a quarter of a second after the switch")
            w("# operates: keep HOME_SEARCH_VEL low enough for the switch to survive that.")
            w("HOME_SEARCH_VEL = %s" % fmt(sign * search))
            w("HOME_LATCH_VEL = %s" % fmt(sign * min(small, search)))
            w("HOME_OFFSET = %s" % fmt(edge))
            w("HOME = %s" % fmt(edge - sign * 2.0 * small))
            w("HOME_IGNORE_LIMITS = NO")
        else:
            w("# No home switch: homing takes the present position as %s." % fmt(0.0))
            w("HOME_SEARCH_VEL = 0")
            w("HOME_LATCH_VEL = 0")
            w("HOME_OFFSET = 0")
            w("HOME = 0")
        twin = any(o is not j and o["letter"] == j["letter"] for o in joints)
        # Z first on a mill (up, out of the work); X first on a lathe (out from the work)
        sequence = 0 if j["letter"] == ("X" if m.get("lathe") else "Z") else 1
        w("HOME_SEQUENCE = %d" % (-sequence if twin and sequence else sequence))
        w("")

    with open(path, "w", newline="\n") as f:
        f.write("\n".join(out))


def write_hal(m, path):
    joints = m["joints"]
    outs = outputs(m)
    in_order = outs == list(range(len(joints)))
    out = []
    w = out.append
    w("# Tiggy motion controller - HAL connections")
    w("# Written by tiggy_setup.py.  The settings themselves are in the INI file.")
    w("")
    w("loadrt [KINS]KINEMATICS")
    w("loadrt [EMCMOT]EMCMOT servo_period_nsec=[EMCMOT]SERVO_PERIOD num_joints=[KINS]JOINTS")
    if not in_order:
        w("# The driver's joints are the controller's motor outputs (%s here); each of LinuxCNC's"
          % ", ".join("%d = %s" % (o, LETTERS[o]) for o in range(max(outs) + 1)))
        w("# joints is connected to the output its motor is on (%s)."
          % ", ".join("%s on %s" % (j["letter"], LETTERS[o]) for j, o in zip(joints, outs)))
    w("loadrt tiggy ip=[TIGGY]ADDRESS joints=%s" % ("[KINS]JOINTS" if in_order else "%d" % (max(outs) + 1))
      + (" io_ip=[TIGGY]IO_MODULE" if m["io_module"] else ""))
    w("loadrt estop_latch")
    w("")
    w("addf tiggy.read             servo-thread")
    w("addf estop-latch.0          servo-thread")
    w("addf motion-command-handler servo-thread")
    w("addf motion-controller      servo-thread")
    w("addf tiggy.write            servo-thread")
    w("")
    w("# --- the controller ---")
    w("setp tiggy.step-pulse-us     [TIGGY]STEP_PULSE_US")
    w("setp tiggy.dir-setup-us      [TIGGY]DIR_SETUP_US")
    w("setp tiggy.enable-invert     [TIGGY]ENABLE_INVERT")
    w("setp tiggy.idle-release-ms   [TIGGY]IDLE_RELEASE_MS")
    w("setp tiggy.buffer-target-ms  [TIGGY]BUFFER_MS")
    w("setp tiggy.debounce-ms       [TIGGY]DEBOUNCE_MS")
    for i in range(6):
        w("setp tiggy.switch.%d-invert   [TIGGY]SWITCH_%d_INVERT" % (i, i))
    w("setp tiggy.probe-invert      [TIGGY]PROBE_INVERT")
    w("setp tiggy.estop-invert      [TIGGY]ESTOP_INVERT")
    w("setp tiggy.spindle-pwm       [TIGGY]SPINDLE_PWM")
    w("setp tiggy.spindle-pwm-freq  [TIGGY]SPINDLE_PWM_FREQ")
    w("setp tiggy.spindle-max-rpm   [TIGGY]SPINDLE_MAX_RPM")
    w("setp tiggy.spindle-min-duty  [TIGGY]SPINDLE_MIN_DUTY")
    w("setp tiggy.charge-pump-freq  [TIGGY]CHARGE_PUMP_FREQ")
    w("")
    w("# --- E-stop chain, and machine on ---")
    w("# LinuxCNC is held in E-stop by its own E-stop button and by tiggy.fault, which is")
    w("# true while the controller cannot be reached, while its E-stop input is operated,")
    w("# and after it has stopped by itself (a limit switch, lost contact).  After a fault,")
    w("# E-stop has to be reset in LinuxCNC before the machine can be switched on again.")
    w("net estop-button iocontrol.0.user-enable-out     => estop-latch.0.ok-in")
    w("net estop-reset  iocontrol.0.user-request-enable => estop-latch.0.reset")
    w("net tiggy-fault  tiggy.fault                     => estop-latch.0.fault-in")
    w("net estop-ok     estop-latch.0.ok-out            => iocontrol.0.emc-enable-in")
    w("net machine-on   motion.motion-enabled           => tiggy.enable")
    w("")
    for n, j in enumerate(joints):
        tag = j["letter"].lower() + ("2" if j["second"] else "")
        o = outs[n]
        w("# --- joint %d: %s%s%s ---" % (n, j["letter"], ", second motor" if j["second"] else "",
                                         "" if in_order else ", on motor output %s" % LETTERS[o]))
        w("setp tiggy.%d.position-scale [JOINT_%d]SCALE" % (o, n))
        w("setp tiggy.%d.maxvel         [JOINT_%d]MAX_VELOCITY" % (o, n))
        w("setp tiggy.%d.maxaccel       [JOINT_%d]MAX_ACCELERATION" % (o, n))
        w("setp tiggy.%d.step-invert    [JOINT_%d]STEP_INVERT" % (o, n))
        w("setp tiggy.%d.dir-invert     [JOINT_%d]DIR_INVERT" % (o, n))
        w("net %s-pos-cmd joint.%d.motor-pos-cmd => tiggy.%d.position-cmd" % (tag, n, o))
        w("net %s-pos-fb  tiggy.%d.position-fb   => joint.%d.motor-pos-fb" % (tag, o, n))
        w("net %s-homing  joint.%d.homing        => tiggy.%d.homing" % (tag, n, o))
        if j["home_switch"] or j["limit_switch"]:
            # A switch used for homing is not also given to LinuxCNC as a limit:
            # the machine runs a moment behind LinuxCNC, so it is still leaving
            # the switch when LinuxCNC has finished homing and would then report
            # a limit.  The controller guards such a switch itself instead.
            if j["home_switch"]:
                users = ["joint.%d.home-sw-in" % n]
            else:
                users = ["joint.%d.neg-lim-sw-in" % n, "joint.%d.pos-lim-sw-in" % n]
            own = j["switch_input"] == o
            source = "tiggy.switch.%d => " % o if own else ""       # a shared switch is already on its net
            name = tag if own else j["letter"].lower()
            w("net %s-switch  %s%s" % (name, source, " ".join(users)))
            if j["limit_switch"] and own:
                if j["home_switch"]:
                    w("# Also the limit switch: the controller itself stops, within milliseconds, when")
                    w("# it operates outside homing, and LinuxCNC then goes into E-stop.  (LinuxCNC is")
                    w("# not given it as a limit: see the note on limit switches in the driver's README.)")
                else:
                    w("# the controller itself stops, within milliseconds, when this switch operates")
                w("setp tiggy.switch.%d-is-limit 1" % o)
        w("")
    w("# --- spindle and coolant ---")
    if m["spindle"] != "n":
        w("net spindle-forward spindle.0.forward       => tiggy.spindle-forward")
        w("net spindle-reverse spindle.0.reverse       => tiggy.spindle-reverse")
        w("net spindle-speed   spindle.0.speed-out-abs => tiggy.spindle-speed")
    if m.get("threading"):
        w("# Screw cutting: LinuxCNC follows a smooth copy of the spindle that the driver")
        w("# makes; the controller cuts by the real one (see the README).")
        w("net spindle-revs         tiggy.spindle-revs         => spindle.0.revs")
        w("net spindle-speed-in     tiggy.spindle-speed-fb     => spindle.0.speed-in")
        w("net spindle-index-enable tiggy.spindle-index-enable <=> spindle.0.index-enable")
    w("net coolant-flood iocontrol.0.coolant-flood => tiggy.coolant-flood")
    w("net coolant-mist  iocontrol.0.coolant-mist  => tiggy.coolant-mist")
    w("")
    w("# --- probe ---")
    w("# The machine travels on for about a quarter of a second after the probe touches:")
    w("# probe slowly (60 mm/min runs on 0.25 mm).")
    w("net probe-in tiggy.probe => motion.probe-input")
    w("")
    w("# --- tool change ---")
    if m["manual_toolchange"]:
        w("loadusr -W hal_manualtoolchange")
        w("net tool-change      iocontrol.0.tool-change      => hal_manualtoolchange.change")
        w("net tool-changed     iocontrol.0.tool-changed     <= hal_manualtoolchange.changed")
        w("net tool-number      iocontrol.0.tool-prep-number => hal_manualtoolchange.number")
    else:
        w("net tool-change      iocontrol.0.tool-change      => iocontrol.0.tool-changed")
    w("net tool-prepare     iocontrol.0.tool-prepare     => iocontrol.0.tool-prepared")
    w("")
    w("# --- free for your own use ---")
    w("# tiggy.output.0 to .4    the controller's auxiliary outputs, e.g.")
    w("#     net light motion.digital-out-00 => tiggy.output.0        (M64 P0 / M65 P0)")
    w("# tiggy.input.0 to .3     its auxiliary inputs (and -not), e.g.")
    w("#     net door tiggy.input.0 => motion.digital-in-00           (M66 P0)")
    w("# tiggy.estop-input       its E-stop input; tiggy.spindle-speed-fb the spindle sensor (rev/s),")
    w("#                         tiggy.spindle-turns and tiggy.spindle-angle what it counts")
    w("# tiggy.buffer-ms, tiggy.underruns, tiggy.segments-resent      how the link is doing")
    with open(path, "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")


def write_postgui(m, path):
    out = ["# Loaded after the screen has started: connections to on-screen panels and to halui go here."]
    if m["io_module"]:
        out += ["",
                "# I/O expansion module: tiggy.io.in-00 to in-15 (and -not), tiggy.io.out-00 to out-15.",
                "# For example a pendant:",
                "#   net jog-x-plus   tiggy.io.in-00 => halui.axis.x.plus",
                "#   net jog-x-minus  tiggy.io.in-01 => halui.axis.x.minus",
                "#   net cycle-start  tiggy.io.in-06 => halui.program.run",
                "#   net feed-hold    tiggy.io.in-07 => halui.program.pause",
                "# and a relay:",
                "#   net relay-1      motion.digital-out-01 => tiggy.io.out-00"]
    with open(path, "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")


def folder_name(name):
    """The name of a machine as the name of its folder and INI file."""
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in name) or "Tiggy"


def write_config(m, folder):
    """Writes the whole configuration for machine m into folder.  Returns the INI file's path."""
    safe = folder_name(m["name"])
    ini = os.path.join(folder, safe + ".ini")
    os.makedirs(folder, exist_ok=True)
    write_ini(m, ini)
    write_hal(m, os.path.join(folder, "tiggy.hal"))
    postgui = os.path.join(folder, "postgui.hal")
    if not os.path.exists(postgui):             # (the place for the user's own additions: never replaced)
        write_postgui(m, postgui)
    tool = os.path.join(folder, "tool.tbl")
    if not os.path.exists(tool):
        with open(tool, "w", newline="\n") as f:
            if m.get("lathe"):
                f.write("T1 P1 X0 Z0 D%s Q2 ;example turning tool\n" % ("0.03" if m["inch"] else "0.8"))
            else:
                f.write("T1 P1 Z0 D%s ;example tool\n" % ("0.125" if m["inch"] else "3"))
    with open(os.path.join(folder, "tiggy.json"), "w", newline="\n") as f:
        json.dump(m, f, indent=1, sort_keys=True)
        f.write("\n")
    return ini


def main():
    ap = argparse.ArgumentParser(description="Writes a LinuxCNC configuration for the Tiggy motion controller")
    ap.add_argument("--defaults", action="store_true", help="do not ask: a plain 3-axis machine")
    ap.add_argument("--lathe", action="store_true", help="with --defaults: a plain lathe, X and Z")
    ap.add_argument("--name", help="name of the machine (with --defaults)")
    ap.add_argument("--address", help="address of the controller (with --defaults)")
    ap.add_argument("--dir", help="folder to write to (default ~/linuxcnc/configs/<name>)")
    ap.add_argument("--no-toolchange-prompt", action="store_true", help="tool changes need no confirmation on screen")
    ap.add_argument("--spindle-sensor", action="store_true", help="with --defaults: wire up screw cutting (G33, G76)")
    args = ap.parse_args()

    global QUIET
    QUIET = args.defaults
    if not args.defaults:
        print("Tiggy motion controller - LinuxCNC set-up")
        print("Press Enter to accept the value shown in [brackets].")
    m = ask_machine(Asker(args.defaults, lathe=args.lathe, sensor=args.spindle_sensor))
    if args.name:
        m["name"] = args.name
    if args.address:
        m["address"] = args.address
    if args.no_toolchange_prompt:
        m["manual_toolchange"] = False
    if args.spindle_sensor:
        m["threading"] = True

    safe = folder_name(m["name"])
    folder = args.dir or os.path.join(os.path.expanduser("~"), "linuxcnc", "configs", safe)
    ini = os.path.join(folder, safe + ".ini")
    if os.path.exists(ini) and not args.defaults:
        if not Asker(False).yes("\n%s exists. Replace it" % ini, False):
            sys.exit("Nothing written.")
    write_config(m, folder)

    print("\nConfiguration written to %s" % folder)
    print("Start LinuxCNC with:\n    linuxcnc %s" % ini)
    print("\nBefore the first move: check each axis' direction with a slow jog, and that")
    print("the travel limits in the INI file match the machine.")


if __name__ == "__main__":
    main()
