#!/usr/bin/env python3
"""
Tests the set-up program (tiggy_config.py) by working it: with the firmware
simulator as the controller it finds it, connects, runs a motor from the
Motors page, corrects the steps per unit from a measurement, watches the
switch lamps follow the inputs, runs the spindle, sends a spindle sensor to
the controller, saves the configuration and opens it again; then a lathe and
a machine with two motors on one axis.

It needs a screen to draw on, visible or not, and the simulator's library
(firmware/test/pc/build.sh):

    xvfb-run -a python3 linuxcnc/test/config_test.py [--pictures FOLDER]

Nothing of the user's is touched: the configurations go to a temporary home.
"""

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))

ap = argparse.ArgumentParser()
ap.add_argument("--pictures", help="folder for a picture of each page (needs xwd and ImageMagick's convert)")
args = ap.parse_args()

home = tempfile.mkdtemp(prefix="tiggy-config-test-")
os.environ["HOME"] = home                   # before the program works out where configurations live
sys.path.insert(0, os.path.join(ROOT, "linuxcnc"))
import tiggy_config as tc       # noqa: E402
import tiggy_setup as gen       # noqa: E402

failed, said = [], []


def box(kind):
    def show(_title, text, **_kw):
        said.append(text)
        return True                         # (every question is answered with yes)
    return show


for kind in ("showinfo", "showwarning", "showerror", "askokcancel"):
    setattr(tc.messagebox, kind, box(kind))


class Sim:
    """The simulator's own control channel: what the 'machine' really did."""

    def __init__(self, port=58500):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3.0)
        self.f = self.s.makefile("rwb")

    def __call__(self, **req):
        self.f.write((json.dumps(req) + "\n").encode())
        self.f.flush()
        return json.loads(self.f.readline().decode())


def expect(what, cond):
    print("%s  %s" % ("ok  " if cond else "FAIL", what))
    if not cond:
        failed.append(what)


def step(what, fn):
    try:
        return fn()
    except Exception:
        failed.append(what)
        print("FAIL  %s:\n%s" % (what, traceback.format_exc()))


def run(app, seconds):
    end = time.time() + seconds
    while time.time() < end:
        app.update()
        time.sleep(0.01)


def picture(app, name):
    if not args.pictures:
        return
    app.update()
    os.makedirs(args.pictures, exist_ok=True)
    subprocess.run("xwd -silent -root | convert xwd:- '%s'" % os.path.join(args.pictures, name + ".png"),
                   shell=True, capture_output=True)


def colour(lamp):
    return lamp.itemcget(lamp.dot, "fill")


def main():
    simulator = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "firmware_sim.py"), "--bind", "127.0.0.1", "--quiet"])
    try:
        for _ in range(50):
            try:
                sim = Sim()
                break
            except OSError:
                time.sleep(0.1)
        else:
            sys.exit("the firmware simulator did not start (firmware/test/pc/build.sh builds its library)")
        sim(cmd="set", estop=0, limits=0, probe=0, lim_lo=[-4000000] * 6, lim_hi=[4000000] * 6)

        # a configuration from the terminal wizard, to be opened
        gen.QUIET = True
        m = gen.ask_machine(gen.Asker(True))
        m.update(name="Bench", address="127.0.0.1")
        gen.write_config(m, os.path.join(tc.CONFIGS, "Bench"))

        app = tc.App()
        app.geometry("1000x780+0+0")
        app.report_callback_exception = lambda *a: (failed.append("an error in the program"),
                                                    print("FAIL  error in the program:\n" + "".join(traceback.format_exception(*a))))
        app.update()
        expect("opens the configuration that is there", app.m["name"] == "Bench" and [j["letter"] for j in app.m["joints"]] == list("XYZ"))

        # ---- find and connect
        step("find", app.find)
        found = [app.found.item(i)["values"] for i in app.found.get_children()]
        expect("finds the controller", any(row[1] == "127.0.0.1" and row[4] == "free" for row in found))
        step("connect", app.connect)
        run(app, 0.6)
        expect("connects, and status reports arrive", app.ctl is not None and app.ctl.status is not None)
        expect("says what the controller is", "firmware 1.2" in app.about["text"] and "six axes" in app.about["text"])
        picture(app, "1_controller")
        app.book.select(1)
        picture(app, "2_machine")

        # ---- Motors: run X from the page
        app.book.select(2)
        run(app, 0.2)
        p0 = sim(cmd="get")["pos"][0]
        step("hold +", lambda: app.jog_start(1))
        run(app, 1.0)
        step("let go", app.jog_stop)
        run(app, 0.8)
        p1 = sim(cmd="get")["pos"][0]
        expect("holding + runs X forwards (%d steps)" % (p1 - p0), p1 - p0 > 500)
        expect("the travel shown is the travel made", ("(%d steps)" % (p1 - p0)) in app.moved["text"])
        expect("at a quarter of top speed, as set", 0.6 * 10000 < (p1 - p0) < 1.4 * 10000)
        step("hold -", lambda: app.jog_start(-1))
        run(app, 0.5)
        step("let go", app.jog_stop)
        run(app, 0.8)
        p2 = sim(cmd="get")["pos"][0]
        expect("holding - runs it back (%d steps)" % (p2 - p1), p2 < p1)
        run(app, 0.6)
        p3 = sim(cmd="get")["pos"][0]
        expect("and it stands still when let go", p3 == p2)
        picture(app, "3_motors")
        shown = app.show_moved()
        app.really.set("%g" % (abs(shown) * 2.0))           # "it really went twice as far": half the steps
        step("correct the steps", app.calibrate)
        expect("steps per unit corrected from a measurement (800 -> %s)" % gen.fmt(app.m["joints"][0]["scale"]),
               abs(app.m["joints"][0]["scale"] - 400.0) < 0.01)
        step("too fast", lambda: (app.set_joint("vel", 200.0), app.show()))
        expect("a speed the controller cannot step is pointed out", "needs" in app.rate_note["text"] and app.problems())
        before = sim(cmd="get")["pos"][0]
        step("hold + at that speed", lambda: app.jog_start(1))
        run(app, 0.3)
        step("let go", app.jog_stop)
        expect("...and is not run", sim(cmd="get")["pos"][0] == before)
        step("back to 50", lambda: (app.set_joint("vel", 50.0), app.show()))

        # ---- Switches: the lamps are the inputs
        app.book.select(3)
        step("a switch on X", lambda: app.set_switch(app.m["joints"][0], "home and limit"))
        run(app, 0.3)
        lamp = app.switch_rows[0][0]
        expect("X lamp off while its switch is open", colour(lamp) == tc.OFF)
        sim(cmd="set", limits=1)
        run(app, 0.4)
        expect("X lamp on when it is operated", colour(lamp) == tc.ON)
        expect("the controller has not stopped for it (nothing here is a limit yet)", app.ctl.status["state"] != 6)
        sim(cmd="set", limits=0, probe=1, estop=1)
        run(app, 0.4)
        expect("probe and E-stop lamps follow their inputs", colour(app.probe_lamp) == tc.ON and colour(app.estop_lamp) == tc.ON)
        picture(app, "4_switches")
        sim(cmd="set", probe=0, estop=0)
        run(app, 0.3)

        # ---- Spindle: run it, and send a sensor to the controller
        app.book.select(4)
        step("PWM", lambda: (app.m.update(spindle="p", max_rpm=3000.0, pwm_freq=1000), app.show()))
        app.spin.set(True)
        app.spin_rpm.set("1500")
        step("run the spindle", app.run_spindle)
        run(app, 0.8)
        expect("the spindle runs at the speed asked (%s rpm)" % sim(cmd="get")["spindle"]["rpm"],
               abs(sim(cmd="get")["spindle"]["rpm"] - 1500.0) < 30.0)
        app.spin.set(False)
        step("stop the spindle", app.run_spindle)
        run(app, 0.5)
        app.sensor_kind.set(tc.link.SENSOR_PULSE)
        step("send the sensor", app.send_sensor)
        run(app, 0.5)
        expect("a one-pulse sensor is on the controller after its restart, and threads can be cut",
               app.ctl is not None and app.on_controller["kind"] == tc.link.SENSOR_PULSE and "can cut threads" in app.about["text"])
        expect("screw cutting is wired for LinuxCNC", app.m["threading"])
        picture(app, "5_spindle")

        # ---- save, and open again
        app.book.select(5)
        step("name", lambda: (app.m.update(name="Sim Test"), app.show()))
        picture(app, "6_finish")
        step("save", app.save)
        folder = os.path.join(tc.CONFIGS, "Sim_Test")
        files = sorted(os.listdir(folder)) if os.path.isdir(folder) else []
        expect("writes the configuration (%s)" % ", ".join(files),
               files == ["Sim_Test.ini", "postgui.hal", "tiggy.hal", "tiggy.json", "tool.tbl"])
        ini = tc.read_ini(os.path.join(folder, "Sim_Test.ini")) if files else {}
        expect("with what was set", ini.get("JOINT_0", {}).get("SCALE") == "400" and ini.get("TIGGY", {}).get("ADDRESS") == "127.0.0.1"
               and ini.get("TIGGY", {}).get("SPINDLE_PWM") == "1")
        hal = open(os.path.join(folder, "tiggy.hal")).read() if files else ""
        expect("X's switch for homing and as a limit, and the spindle's copy for G33",
               "tiggy.switch.0 => joint.0.home-sw-in" in hal and "setp tiggy.switch.0-is-limit 1" in hal and "spindle.0.revs" in hal)
        # a number changed by hand in the INI file since
        if files:
            head, joint = open(os.path.join(folder, "Sim_Test.ini")).read().split("[JOINT_0]", 1)
            with open(os.path.join(folder, "Sim_Test.ini"), "w") as f:
                f.write(head + "[JOINT_0]" + joint.replace("MAX_VELOCITY = 50", "MAX_VELOCITY = 42", 1))
        step("open again", lambda: app.open_folder(folder))
        j = app.m["joints"][0]
        expect("opens again as saved", abs(j["scale"] - 400.0) < 0.01 and j["home_switch"] and j["limit_switch"] and app.m["address"] == "127.0.0.1")
        expect("with a number that was changed by hand in the INI file since", j["vel"] == 42.0)
        step("save again", app.save)
        expect("what it replaced is kept beside it", os.path.exists(os.path.join(folder, "Sim_Test.ini.before")))

        # ---- a lathe
        step("new lathe", lambda: (app.which.set("(a new lathe)"), app.choose()))
        expect("a lathe has X on output X and Z on output Z",
               [(j["letter"], j["output"]) for j in app.m["joints"]] == [("X", 0), ("Z", 2)] and app.m["diameter"])
        expect("a new machine takes the connected controller's address", app.m["address"] == "127.0.0.1")
        app.book.select(1)
        picture(app, "7_lathe")
        step("name", lambda: app.m.update(name="Lathe"))
        step("save", app.save)
        lathe = tc.read_ini(os.path.join(tc.CONFIGS, "Lathe", "Lathe.ini")) if os.path.isdir(os.path.join(tc.CONFIGS, "Lathe")) else {}
        expect("its configuration is a lathe's", lathe.get("DISPLAY", {}).get("LATHE") == "1"
               and lathe.get("KINS", {}).get("KINEMATICS") == "trivkins coordinates=XZ" and "G7" in lathe.get("RS274NGC", {}).get("RS274NGC_STARTUP_CODE", ""))

        # ---- two motors on one axis
        step("new mill", lambda: (app.which.set("(a new mill or router)"), app.choose()))
        step("Y's second motor on output A", lambda: app.set_output(3, "Y axis"))
        expect("the motor on the higher output is the axis' second",
               [(j["letter"], j["output"], j["second"]) for j in app.m["joints"]] == [("X", 0, False), ("Y", 1, False), ("Z", 2, False), ("Y", 3, True)])
        step("select Y", lambda: (setattr(app, "selected", 1), app.show(), app.set_joint("scale", 1000.0)))
        expect("both motors of an axis take its values", app.m["joints"][3]["scale"] == 1000.0)
        step("drop Z", lambda: app.set_output(2, "not used"))
        expect("an output can be taken out of use", [j["letter"] for j in app.m["joints"]] == ["X", "Y", "Y"])
        step("name", lambda: app.m.update(name="Gantry"))
        step("save", app.save)
        g = tc.read_ini(os.path.join(tc.CONFIGS, "Gantry", "Gantry.ini")) if os.path.isdir(os.path.join(tc.CONFIGS, "Gantry")) else {}
        expect("its configuration has both motors (%s)" % g.get("KINS", {}).get("KINEMATICS"),
               g.get("KINS", {}).get("KINEMATICS") == "trivkins coordinates=XYY kinstype=BOTH")
        ghal = open(os.path.join(tc.CONFIGS, "Gantry", "tiggy.hal")).read() if g else ""
        expect("...on the outputs given (Y's second on A)", "joint.2.motor-pos-cmd => tiggy.3.position-cmd" in ghal and "joints=4" in ghal)

        step("close", app.close)
        run_after = sim(cmd="get")
        expect("the controller is left at rest", not run_after.get("moving", False))
    finally:
        simulator.terminate()
        shutil.rmtree(home, ignore_errors=True)
    print("\n%s" % ("ALL PASSED (set-up program)" if not failed else "%d FAILED" % len(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
