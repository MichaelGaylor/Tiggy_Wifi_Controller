#!/usr/bin/env python3
"""
Arranges a controller on the bench for run_on_board.sh --threading, and puts
it back afterwards.

    board_prepare.py prepare <address> <file>
    board_prepare.py restore <address> <file>

prepare sets the spindle sensor to one pulse per revolution and exchanges the
X step and spindle terminals in the controller's settings, so that a wire
from the X step terminal to the sensor's terminal carries the spindle PWM to
the sensor: a spindle that turns when LinuxCNC switches the spindle on.  The
PWM is set to the 53 Hz of the test as well, so that the controller starts
with it.  What was set before is kept in <file>; restore puts it back.  The
controller restarts each time.
"""

import json
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from spindle_loopback_test import (Board, NAME, PIN_INDEX, PIN_SPINDLE, PIN_STEP_X, PWM_HZ, PWM_ON,  # noqa: E402
                                   SENSOR_MODE, SENSOR_PPR)

KEYS = (SENSOR_MODE, SENSOR_PPR, PIN_STEP_X, PIN_SPINDLE, PWM_ON, PWM_HZ)


def main():
    if len(sys.argv) != 4 or sys.argv[1] not in ("prepare", "restore"):
        sys.exit(__doc__)
    what, address, path = sys.argv[1:4]
    board = Board(address)
    if what == "prepare":
        was = board.read(KEYS)
        step, spindle = was[PIN_STEP_X], was[PIN_SPINDLE]
        if 0xFF in (step, spindle):
            sys.exit("This controller has no spindle output to stand in for a spindle.")
        with open(path, "w") as f:
            json.dump({str(k): v for k, v in was.items()}, f)
        board.write({SENSOR_MODE: 1, SENSOR_PPR: 1, PIN_STEP_X: spindle, PIN_SPINDLE: step, PWM_ON: 1, PWM_HZ: 53})
        print("The controller's spindle output is now on GPIO %d and its sensor input on GPIO %d: "
              "these two are to be linked." % (step, board.c.cfg_get(PIN_INDEX)))
    else:
        with open(path) as f:
            was = {int(k): v for k, v in json.load(f).items()}
        try:
            board.write(was)
            print("The controller's settings are as they were.")
        except Exception as e:
            print("THE CONTROLLER'S SETTINGS WERE NOT PUT BACK (%s).  They were:" % e)
            for k, v in was.items():
                print("    %-32s (0x%04X) %s" % (NAME[k], k, v))
            sys.exit(1)
    board.close()


if __name__ == "__main__":
    main()
