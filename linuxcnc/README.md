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

`install.sh` builds the driver, installs it among LinuxCNC's HAL components
and then starts the set-up wizard, which asks about the machine (axes, steps
per unit, speeds, travel, switches, spindle) and writes a complete
configuration to `~/linuxcnc/configs/<name>`. Start LinuxCNC with it:

```bash
linuxcnc ~/linuxcnc/configs/Tiggy/Tiggy.ini
```

- `./install.sh --driver` only rebuilds the driver, for example after an update.
- `python3 tiggy_setup.py` runs the wizard again; `--defaults` writes a plain
  3-axis machine without asking.

Everything the wizard asked can be changed later in the INI file. The
controller's own settings are in its `[TIGGY]` section and are passed on to
the controller each time LinuxCNC starts, so there is nothing to set up on
the controller itself beyond joining it to your network.

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
  within milliseconds; LinuxCNC is then told and goes into E-stop. (While an
  axis stands on its switch, or one is being homed, the controller leaves all
  the limits to LinuxCNC, which reacts about a quarter of a second later.)
- Spindle-synchronised motion (G33, G76 threading, rigid tapping) is not
  supported.

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
| `tiggy.ready` | bit out | for the E-stop chain (`iocontrol.0.emc-enable-in`). False while the controller cannot be reached, its E-stop input is operated, or it has stopped by itself |
| `tiggy.connected` | bit out | the controller is answering |
| `tiggy.switch.0` ... `.5` (and `-not`) | bit out | the six limit / home inputs |
| `tiggy.probe` | bit out | probe input |
| `tiggy.estop-input` | bit out | the controller's E-stop input |
| `tiggy.input.0` ... `.3` (and `-not`) | bit out | auxiliary inputs |
| `tiggy.output.0` ... `.4` | bit in | auxiliary outputs |
| `tiggy.spindle-forward`, `-reverse` | bit in | spindle |
| `tiggy.spindle-speed` | float in | spindle speed, rpm |
| `tiggy.spindle-speed-fb`, `-fb-rpm` | float out | spindle encoder, rev/s and rpm |
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
