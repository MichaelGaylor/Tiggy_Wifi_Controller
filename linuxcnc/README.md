# Tiggy motion controller with LinuxCNC

The `tiggy` driver lets LinuxCNC run a machine through the Tiggy controller
over WiFi or Ethernet. LinuxCNC plans the motion; the controller makes the
step pulses. No realtime kernel is needed, and all six axes are supported.

It needs LinuxCNC 2.8 or newer in its standard "uspace" form (the
`linuxcnc-uspace` package, or the LinuxCNC live image) and controller
firmware 1.2.0 or newer.

## Installing

```bash
sudo apt install linuxcnc-uspace linuxcnc-uspace-dev    # not needed on the LinuxCNC live image
tar xzf Tiggy-LinuxCNC.tar.gz
cd linuxcnc-package
./install.sh
```

`install.sh` builds the driver, installs it among LinuxCNC's HAL components,
puts **Tiggy set-up for LinuxCNC** in the menu and starts it.
`./install.sh --driver` only rebuilds the driver, for example after an update.

## Setting up a machine

The set-up program (`tiggy_config.py`) does what the Config menu does in
Mach3, with the controller connected:

| Page | |
|---|---|
| Controller | Finds the controllers on the network. Pick yours and connect; its address goes into the configuration. |
| Machine | Its name, mill/router or lathe, millimetres or inches, and which axis the motor on each output drives (two outputs for an axis with two motors). A lathe has X on output X and Z on output Z. |
| Motors | Steps per unit, top speed, acceleration and travel of each axis. The ramp is drawn, and the step rate checked against what the controller can make. **Try it** runs the motor from the page with those values; measure how far it really went, and the steps per unit are corrected for you. |
| Switches | What the switch on each input is - home, limit, both - with a lamp that shows the input as it is now, so a switch can be operated by hand and watched. Probe and E-stop likewise. |
| Spindle | On and off, or speed by PWM, with a test. The spindle sensor for screw cutting and what the controller's terminals are used for (six axes, or four and the encoder's inputs): these two are kept in the controller itself and are sent from here. |
| Finish | What will be written. |

**Save** writes the configuration to `~/linuxcnc/configs/<name>`; **Save and
start LinuxCNC** starts it as well:

```bash
linuxcnc ~/linuxcnc/configs/Tiggy/Tiggy.ini
```

The program opens a configuration again to change it. Numbers changed in the
INI file by hand since - steps, speeds, travels, the controller's address -
are kept; the files it replaces are left beside the new ones with `.before`
added, and `postgui.hal`, the place for your own additions, is never touched.
A controller works for one program at a time, so LinuxCNC has to be closed
while the set-up program is connected to it.

Without a screen, `python3 tiggy_setup.py` asks the same questions in a
terminal (`--defaults` writes a plain 3-axis machine without asking,
`--defaults --lathe` a plain lathe).

Everything can also be changed in the INI file. The controller's settings are
in its `[TIGGY]` section and are passed on to the controller each time
LinuxCNC starts; only the spindle sensor and the use of the terminals are set
on the controller itself, from the Spindle page.

### A lathe

X and Z go on the controller's X and Z outputs, with their switches on the
X and Z inputs. X positions are diameters (G7) unless you say otherwise, the
plane is G18, and with a spindle sensor G33 and the G76 threading cycle work
(see below). The screen is LinuxCNC's lathe screen.

### Finding the controller

With `ADDRESS = auto` the driver looks for the controller on every network
the computer is connected to, and at 192.168.4.1 (the controller's own access
point). If the computer cannot broadcast to the controller's network (some
routers, virtual machines), give the address instead: `ADDRESS = 192.168.1.50`.

Until the controller is found LinuxCNC stays in E-stop, and after a few
seconds says so on screen.

## What to expect

**The machine runs a fraction of a second behind LinuxCNC.** The controller
keeps 200 ms of motion queued (`[TIGGY]BUFFER_MS`) so that a late or lost
network packet never reaches the motors: it is sent again before its turn
comes. Everything LinuxCNC does arrives exactly, to the step, but that much
later. In practice:

- Jogging, feed hold and stop take effect about a quarter of a second after
  the key. E-stop and machine-off do not wait: they stop the controller at once.
- `FERROR` and `MIN_FERROR` of each joint must allow for the delay. The wizard
  sets them to 0.35 s of travel at top speed; following error therefore no
  longer detects a stalled motor (with steppers it never could).
- **Homing**: an axis runs on for about a quarter of a second after its switch
  operates. The wizard sets a search speed of at most 10 mm/s (2.5 mm of run-on).
  The position latched is the controller's own count, reported together with
  the switch, so homing repeats to within the distance covered in 10 ms at
  `HOME_LATCH_VEL` (0.01 mm at 1 mm/s).
- **Probing** (G38.x): the same applies. Probe slowly: at 60 mm/min the machine
  travels on 0.25 mm after contact and the result repeats to about 0.01 mm.
- **Limit switches** given to the wizard as limits stop the controller itself
  within milliseconds; LinuxCNC is then told and goes into E-stop. While an
  axis stands on its switch, or one is being homed, the controller's limit
  stop stands aside, so that the axis can be homed and jogged off again.
- **A switch that is both home and limit** is wired to LinuxCNC as the home
  switch only (do not also connect it to `joint.N.neg-lim-sw-in` /
  `pos-lim-sw-in`). The machine is still leaving the switch when LinuxCNC has
  finished homing, and LinuxCNC would report a limit at that moment. The
  controller's own stop guards the limit instead.
- **Screw cutting (G33, G76)** works with a spindle sensor on the controller:
  an encoder, with or without an index pulse, or a single sensor giving one
  pulse per revolution (set on the Spindle page of the set-up program, which
  sends it to the controller).  LinuxCNC
  follows the spindle's position, but the motors are a fifth of a second
  behind LinuxCNC, so the real spindle will not do for that.  The driver
  shows LinuxCNC a smooth copy of the spindle instead (`tiggy.spindle-revs`),
  and sends on what LinuxCNC then commands marked to be played by the
  spindle's angle; the controller, which has the real spindle, begins each
  pass as it passes its index and keeps the axes in place on the thread
  whatever the speed does under the cut.  What this means in practice:
  - Every pass begins at the same place on the work, as LinuxCNC's own
    encoder handling would have it.  As always with LinuxCNC, begin the pass
    clear of the work - the axis needs a moment to reach the spindle's speed -
    and begin every pass at the same spindle speed: the distance that takes
    grows with the speed, and with it moves the place where the thread
    begins.  (What the speed does once the pass is under way does not
    matter.)
  - The thread itself is played by the spindle: from the start of a pass
    until LinuxCNC goes on to a rapid move (it says so on
    `motion.motion-type`, which the set-up connects to the driver) or the
    machine stands still.  The rapid moves out of the thread, back and in
    for the next pass go by the clock like any other move, at whatever
    speed the axes have.  If LinuxCNC switches the spindle off straight
    after a pass, it is kept running for the quarter second until the pass
    has been played.
  - While it follows the spindle the controller makes fewer steps a second
    than otherwise: keep the feed of the thread itself (pitch x spindle
    speed x steps per unit) under about 20,000 steps a second.
  - If a pass cannot be kept in step (the PC or the network stalls in the
    middle of it), the controller stops and abandons it rather than cut
    across the thread, and LinuxCNC is stopped with the reason.
  - Rigid tapping (G33.1) is not supported: the controller does not follow
    a spindle that reverses.

**If the computer or the network hesitates** the queue covers it. If it
hesitates for longer than the queue, the controller slows the machine to a
stop along the programmed path, waits, and picks up again: no position is
lost, and `tiggy.underruns` counts the event. The machine is then further
behind LinuxCNC than usual until the next pause in the motion, and at high
speed LinuxCNC may stop with a following error. If LinuxCNC is not heard
from for a whole second while the machine is working, the controller stops
and switches its outputs off.

**If the controller is lost** while the machine is on (power, network),
LinuxCNC goes into E-stop. The driver finds the controller again by itself
and LinuxCNC's position is kept, but nothing moves and no output comes back
on until the machine has been switched off and on in LinuxCNC.

## Reference

Loaded with `loadrt tiggy ip=<address or auto> joints=<1-6> [io_ip=<address or auto>]`.
`tiggy.read` goes before the motion controller in the servo thread and
`tiggy.write` after it. Joint N uses the controller's motor output N
(0 = X ... 5 = C).

### Pins

| Pin | | |
|---|---|---|
| `tiggy.N.position-cmd` | float in | commanded position, from `joint.N.motor-pos-cmd` |
| `tiggy.N.position-fb` | float out | where the controller really is, to `joint.N.motor-pos-fb` |
| `tiggy.N.counts` | s32 out | the same in steps |
| `tiggy.N.homing` | bit in | from `joint.N.homing`: lets the joint run onto a limit switch |
| `tiggy.enable` | bit in | machine on (`motion.motion-enabled`). While false the controller is held stopped, drivers and outputs off |
| `tiggy.fault` | bit out | for the E-stop chain (the `fault-in` of an `estop_latch`). True while the controller cannot be reached, its E-stop input is operated, or it has stopped by itself |
| `tiggy.ready` | bit out | the opposite of `tiggy.fault` |
| `tiggy.connected` | bit out | the controller is answering |
| `tiggy.switch.0` ... `.5` (and `-not`) | bit out | the six limit / home inputs |
| `tiggy.probe` | bit out | probe input |
| `tiggy.estop-input` | bit out | the controller's E-stop input |
| `tiggy.input.0` ... `.3` (and `-not`) | bit out | auxiliary inputs |
| `tiggy.output.0` ... `.4` | bit in | auxiliary outputs |
| `tiggy.spindle-forward`, `-reverse` | bit in | spindle |
| `tiggy.spindle-speed` | float in | spindle speed, rpm |
| `tiggy.spindle-speed-fb`, `-fb-rpm` | float out | spindle encoder, rev/s and rpm |
| `tiggy.spindle-revs` | float out | for `spindle.0.revs`: the copy of the spindle LinuxCNC follows in G33 and G76 |
| `tiggy.spindle-index-enable` | bit io | for `spindle.0.index-enable` |
| `tiggy.motion-type` | s32 in | from `motion.motion-type`: a move in step with the spindle ends when LinuxCNC goes on to a rapid move (1). Left unconnected, it ends when the machine next stands still |
| `tiggy.spindle-turns`, `tiggy.spindle-angle` | u32, float out | the controller's own reading of its spindle sensor: turns counted, and how far round the turn it is (0 to 1). They come from the same status report as the joints' `counts`: for checking a sensor, or how a thread lies on the spindle |
| `tiggy.coolant-flood`, `-mist` | bit in | coolant |
| `tiggy.buffer-ms` | float out | motion queued at this moment |
| `tiggy.underruns` | u32 out | times the queue ran short |
| `tiggy.segments-resent` | u32 out | motion sent again after 40 ms without acknowledgement |
| `tiggy.position-corrections` | u32 out | times the driver's count had to be put right from the controller's. Stays 0 unless the queue ran completely dry in mid-move |
| `tiggy.io.connected`, `tiggy.io.in-00` ... `in-15` (and `-not`), `tiggy.io.out-00` ... `out-15` | | I/O expansion module (with `io_ip=`) |

### Parameters

| Parameter | Default | |
|---|---|---|
| `tiggy.N.position-scale` | 1 | steps per unit; negative reverses the joint |
| `tiggy.N.maxvel`, `tiggy.N.maxaccel` | 0 | the joint's limits. The controller uses them for its own stops, the driver to refuse a command that jumps |
| `tiggy.N.step-invert`, `tiggy.N.dir-invert` | 0 | signal polarity |
| `tiggy.step-pulse-us`, `tiggy.dir-setup-us` | 5 | step pulse length and direction set-up time, 1-50 us |
| `tiggy.enable-invert` | 0 | drivers enabled by a low level |
| `tiggy.idle-release-ms` | 0 | release the drivers after standing still this long (0 = never) |
| `tiggy.buffer-target-ms` | 200 | motion kept queued, 120-400 ms |
| `tiggy.debounce-ms` | 5 | time an input must be steady, 1-100 ms |
| `tiggy.switch.N-invert` | 0 | the input is on when it opens (normally-closed switch) |
| `tiggy.switch.N-is-limit` | 0 | the controller itself stops when this input operates |
| `tiggy.probe-invert`, `tiggy.estop-invert` | 0 | as for the switches |
| `tiggy.spindle-pwm` | 0 | spindle output is a PWM speed signal |
| `tiggy.spindle-pwm-freq`, `-max-rpm`, `-min-duty` | 1000, 24000, 0 | PWM frequency (Hz), speed at full duty, lowest duty (%) |
| `tiggy.charge-pump-freq` | 0 | charge pump output, Hz; runs while the machine is on |

### Messages

| Message | Meaning |
|---|---|
| no controller found on the network | Not switched on, on another network, or broadcasts do not reach it: set `ADDRESS` |
| ... has firmware x.y.z: update it to 1.2.0 or newer | Flash the current firmware |
| joint N needs ... steps a second | `MAX_VELOCITY` x `SCALE` is more than the controller can step with this pulse length |
| the controller stopped: limit switch input N operated | A switch marked as a limit was hit. Switch the machine on again and, with LinuxCNC's "override limits", jog off it |
| the controller stopped: it heard nothing from LinuxCNC for a second | The network or the computer stopped for that long |
| joint N: the position LinuxCNC commands jumped ... | A HAL connection or setting is wrong; the driver does not follow jumps |
| joint N: LinuxCNC's position and the controller's differ ... | The machine was switched on again in the instant after being stopped in mid-move. Switch it off and on once more |
| ... accepts a connection but does not answer as a Tiggy controller | Another program (Mach3, a second LinuxCNC) is using the controller, or the address belongs to something else |
| another program has taken over the controller | The same, while the machine was running |
| connection to the controller lost | The link dropped. LinuxCNC goes into E-stop; the driver reconnects by itself and the position is kept |

## Testing without a machine

`tools/firmware_sim.py` runs the controller's firmware on a PC. Start it, set
`ADDRESS = 127.0.0.1`, and LinuxCNC drives a simulated machine.
`tools/linuxcnc_driver_test.py` tests the driver itself against the simulator
(moves to the step, packet loss, a stalling PC, E-stop, limits, reconnection)
without needing LinuxCNC; `linuxcnc/test/run_in_linuxcnc.sh` does the same
inside a real LinuxCNC.

With a controller on the bench, `linuxcnc/test/run_on_board.sh <address> --motion`
has LinuxCNC drive it without a screen - homing, MDI, a program, jogging,
abort, E-stop, some minutes of fast moves - and compares LinuxCNC's position
with the controller's own step count after each.  (These test programs are
in the source tree, not in the download.)

LinuxCNC and this driver also run inside Windows, under WSL with Debian 13:
the controller makes the step pulses, so the computer needs no realtime
kernel.  WSL is behind Windows' network, where `ADDRESS = auto` does not
reach: give the controller's address.
