# Tiggy WiFi CNC Motion Controller

Open-source ESP32 **6-axis** CNC motion controller firmware with LinuxCNC and GRBL support. Free and fully functional for all 6 axes with LinuxCNC, GRBL senders, or any G-code host. Mach3 support available separately.

Replaces parallel port drivers with a WiFi or Ethernet-connected motion controller supporting up to 6 axes, spindle encoder feedback, and expanded I/O. Works standalone (GRBL-compatible serial/telnet), with LinuxCNC, or with Mach3 (plugin sold separately). **All 6 axes are free and open source** -- the only limitation is in the Mach3 plugin, which requires a Pro Mach3 license for axes 4-6.

## Downloads

| Component | Download | Description |
|-----------|----------|-------------|
| **ESP32 Firmware** | [Flash from browser](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/) | One-click web flasher (Chrome/Edge) |
| **LinuxCNC** | [Tiggy-LinuxCNC.tar.gz](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/Tiggy-LinuxCNC.tar.gz) | Driver, installer and set-up wizard |
| **Mach3 Plugin** | [www.tiggyengineering.com](https://www.tiggyengineering.com) | Windows plugin (free: 3-axis, Pro: 6-axis) |
| **Protocol Tester** | [TiggyProtocolTester.exe](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/TiggyProtocolTester.exe) | Windows GUI for testing and configuring |

Firmware and LinuxCNC builds are created automatically via GitHub Actions.

## Features

- Up to **6 axes** (X, Y, Z, A, B, C) with independent step/direction control
- **WiFi or Ethernet** (W5500) connectivity -- auto-detected, no configuration needed
- Motion streamed from the host as timed segments with acknowledgement: evenly spaced pulses, and lost or repeated packets can never lose or repeat a step
- Safe when the host goes away: host watchdog, jog deadman, outputs off on E-stop
- GRBL-compatible serial/telnet interface for standalone use
- Homing, limit switches, E-stop and probe inputs, handled in the controller
- Spindle on/off or PWM, charge pump, auxiliary outputs and inputs (count varies by board)
- Spindle speed feedback from an encoder or a one-pulse-per-revolution sensor
- **Screw cutting on lathes**: the controller itself keeps the tool in step with the spindle, so the pitch holds when the spindle slows in the cut and every pass follows the last
- Optional **I/O expansion module** (pendant buttons, relay outputs)
- Auto-discovery and automatic reconnection

## Supported Hardware

| Board | MCU | Wired Axes | Notes |
|-------|-----|------------|-------|
| Tiggy Standard board | ESP32-S3-Zero (FH4R2) | 3 (X/Y/Z) | Compact, Quad SPI PSRAM |
| Tiggy Pro board (Octal) | ESP32-S3-DevKitC-1 (N16R8/N8R8) | 6 (X/Y/Z/A/B/C) | Octal SPI PSRAM, limited I/O |
| Tiggy Pro board (Quad) | ESP32-S3-DevKitC-1 (N8/N8R2) | 6 (X/Y/Z/A/B/C) | Quad SPI PSRAM, more I/O |
| Classic board | ESP32-WROOM-32 | 6 (limited I/O) | Legacy support |

All boards run the same 6-axis firmware. "Wired Axes" refers to the number of step/dir GPIO pairs physically routed on the board. The firmware supports 6 axes regardless of board -- unused axes simply have no GPIO assigned.

One setting trades the B and C axes of the six-axis boards for inputs and outputs - a spindle sensor, a charge pump, more auxiliary I/O: see [Four axes and more I/O](#four-axes-and-more-io-spindle-sensor-and-screw-cutting).

## Quick Start

### Flash Firmware

**Easiest:** Open the [Web Flasher](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/) in Chrome or Edge, plug in your ESP32 via USB, select your board, and click Flash. No software install needed.

**From source** (requires [PlatformIO](https://platformio.org/)):

```bash
cd firmware
pio run -e esp32s3-devkitc -t upload    # Tiggy Pro board (6 wired axes)
pio run -e esp32s3-zero -t upload       # Tiggy Standard board (3 wired axes)
```

### LinuxCNC

```bash
sudo apt install linuxcnc-uspace linuxcnc-uspace-dev   # already there on the LinuxCNC live image
tar xzf Tiggy-LinuxCNC.tar.gz
cd linuxcnc-package
./install.sh    # builds the driver, then starts the set-up program
```

Or double-click `install.desktop` from your file manager.

The set-up program ("Tiggy set-up for LinuxCNC" in the menu afterwards) finds the controller on the network and sets up a mill, router or lathe with it connected, in the manner of Mach3's Config menu: which axis each motor output drives, motor tuning with the motor running from the page, switches with a lamp for each input, the spindle and its sensor. It writes a complete configuration to `~/linuxcnc/configs` and starts LinuxCNC with it. No realtime kernel is needed: LinuxCNC plans the motion and the controller makes the step pulses. What to expect, and every pin and setting of the `tiggy` driver, is in [linuxcnc/README.md](linuxcnc/README.md).

### Mach3 (Windows)

Download the Mach3 plugin from [www.tiggyengineering.com](https://www.tiggyengineering.com), then:

1. Copy `Tiggy.dll` to `C:\Mach3\PlugIns\` and `M62-M65.m1s` to `C:\Mach3\macros\Mach3Mill\`
2. Start Mach3 and choose **Tiggy Motion Controller** as the motion device
3. Set the machine up in Mach3's own Config menu (Motor Tuning, Ports and Pins, Homing/Limits) -- the plugin finds the controller by itself

Plugin 2.0 needs controller firmware 1.2.0 or newer. See the [User Manual](Tiggy_Plugin_Manual.md).

**Free tier**: 3 axes (X/Y/Z). **Pro Mach3 license**: 6 axes, slaved axes, I/O expansion module, screw cutting (G32/G76 in Mach3 Turn). Note: this axis limit applies **only to the Mach3 plugin**. LinuxCNC, GRBL senders, and all other host software get full 6-axis support for free.

## Four axes and more I/O: spindle sensor and screw cutting

Six axes use nearly every terminal of a Tiggy Pro board. A lathe that cuts threads, and many a mill, would rather have four axes and the inputs and outputs. Setting the controller's *machine layout* to **four axes** (configuration key `WCNC_CFG_MACHINE_LAYOUT`; in Mach3 it is on the plugin's Terminals tab) keeps X, Y, Z and A - and Ethernet - and turns the terminals of B and C into inputs and outputs. The same set-up serves a lathe and a mill: the spindle sensor takes only the inputs its kind needs, and every sensor terminal it leaves free is an auxiliary input.

With four axes ("or" = what the terminal is when the sensor does not use it):

| | Tiggy Pro (octal memory) | Tiggy Pro (quad memory) |
|---|---|---|
| Encoder A | GPIO 16, or auxiliary input 3 | GPIO 16, or auxiliary input 4 |
| Encoder B | GPIO 17, or auxiliary input 4 | GPIO 17 |
| Index pulse / single sensor | GPIO 21, or auxiliary input 2 | GPIO 37 |
| Other auxiliary inputs | 1: GPIO 18 | 1 to 3: GPIO 18, 21, 36 |
| Auxiliary outputs | 1 to 3: GPIO 11, 12, 40 | 1 to 5: GPIO 11, 12, 40, 39, 34 |
| Charge pump | GPIO 39 | GPIO 10 |

So an octal-memory Pro board with four axes, Ethernet and an encoder with index still has a limit/home input per axis, probe, E-stop, an auxiliary input, the spindle output, a charge pump and three auxiliary outputs; without a sensor it has four auxiliary inputs. No terminal changes between input and output with the sensor setting - the sensor only ever shares a terminal with an input - so a sensor left wired while the setting is changed is never driven against. A single sensor has its terminal even with six axes on both Pro boards (GPIO 21 and GPIO 37). The Tiggy Standard board has its sensor terminals in any case (encoder A / B / index on GPIO 15 / 16 / 17; those the sensor does not use are auxiliary inputs 1 to 3). `firmware/test/test_pin_layout.c` checks every combination for each board and prints it.

The spindle sensor is either an **encoder with A and B channels**, with or without an index pulse (100 lines or more), or a **single sensor giving one pulse per revolution**, which goes on the index terminal. The encoder gives the better thread: with one pulse per turn the controller cannot see a change of spindle speed until the turn is over. (A single sensor giving several pulses per revolution serves the speed display only: nothing tells the pulses apart, so no thread can be started by them.) The inputs have the chip's weak pull-ups: a single sensor that switches to ground connects directly, but an open-collector encoder needs pull-up resistors of its own (a few kilohms to 3.3 V) to be read at speed, or push-pull outputs.

A threading pass is played in step with the spindle or not at all. If something breaks into one - a feed hold or stop from the host in the middle of it, the host leaving the controller without motion, a spindle the axes cannot keep up with - the controller brings the axes to rest, drops what was left and says so (`WCNC_STREAM_SYNC_LOST`), rather than carry on out of step with the thread already cut.

Screw cutting is available to both hosts: G32 and G76 in Mach3 Turn through the plugin, G33 and G76 in LinuxCNC through its driver (`linuxcnc/README.md`).

For screw cutting the host streams the threading pass like any other move, marked to be played *by the spindle* and to begin at its index (`WCNC_SEG_FLAG_SPINDLE_SYNC`, `WCNC_SEG_FLAG_INDEX_START` in `protocol/wifi_cnc_protocol.h`). The controller then advances through the move by the angle the spindle really turns, so the pitch does not depend on the spindle holding its speed, a spindle that stalls stops the axes with it, and every pass begins at the same place on the work. The Mach3 plugin does this for G32 and G76 in Mach3 Turn, the LinuxCNC driver for G33 and G76.

### A carrier board for either layout

[`hardware/devkitc-carrier`](hardware/devkitc-carrier) is a KiCad project for a board that an ESP32-S3-DevKitC-1 plugs into. It has plugs for six drives, and four jumpers that give the pins of B and C either to their drives (six axes) or to three relays and the charge pump (four axes). Also on it: a socket for a W5500 Ethernet module, seven opto-isolated inputs (limits, E-stop, probe), a separate plug for the spindle sensor and a 0 to 10 V spindle speed output. It is revision A and has not been built yet - see its README before ordering one.

## Architecture

```
Mach3 / LinuxCNC / Serial Terminal
              |
         [TCP:58429 + UDP:58427/58428]
              |
        ESP32-S3 Firmware (this repo)
              |
        Step/Dir/Enable/Spindle/IO GPIO
```

- **TCP** (port 58429): Handshake, configuration, keep-alive
- **UDP** (port 58427): Motion stream, jog, home, outputs, E-stop; status reports go back to the host
- **Protocol**: Defined in `protocol/wifi_cnc_protocol.h` (version 1.2)

## Testing without hardware

The firmware's motion code (`firmware/main/motion`) is plain C that also builds on a PC, where it runs behind the real network ports with a simulated machine attached:

```bash
firmware/test/pc/build.sh                 # or build.bat on Windows: builds the test library, runs the unit test
python tools/firmware_sim.py              # the firmware's motion code as a controller on this PC
python tools/protocol_conformance.py --sim --motion 127.0.0.1    # protocol tests
python tools/firmware_selftest.py         # G-code mode moves, feed hold, homing, older hosts, starved stream, pulse spacing
python tools/threading_test.py            # screw cutting: every step of a pass against the angle of a simulated spindle
python tools/encoder_bench_test.py 127.0.0.1 --lines 1024 --sim    # the bench test of a real encoder (below), the simulator turning one
python tools/encoder_loopback_test.py 127.0.0.1 --sim              # the board's test of its own encoder inputs (below), on the simulator
python tools/ugs_compat_test.py           # TiggyUGS's own connection, planner and sender code against the firmware
                                          # (needs TiggyUGS's source and its Python packages: see the file)
```

Host software (the Mach3 plugin, LinuxCNC, TiggyUGS) connects to it like to a board. Hosts written for protocol 1.0 or 1.1 keep working with firmware 1.2: their speed-described segments are played as before, the status report shows them the room in the queue those wait in, and their reset packet stops and drops what they sent. `tools/tiggy_sim.py` is an independent model of the same protocol in Python; the conformance tests hold both to one behaviour, and can be run against a real board too (`protocol_conformance.py <ip>`, add `--motion` only when the motors are free to move).

The LinuxCNC driver is tested the same way. It builds, unchanged, around a stand-in for LinuxCNC and is run against the firmware simulator (it starts its own simulators, so close one you started by hand first):

```bash
linuxcnc/test/build.sh                    # or build.bat on Windows
python tools/linuxcnc_driver_test.py      # moves to the step, packet loss, a stalling PC, E-stop, limits, reconnection
linuxcnc/test/run_in_linuxcnc.sh          # the same driver inside a real LinuxCNC (Linux with linuxcnc-uspace-dev):
                                          # a mill, home switches, G33, and a lathe with G33 and a G76 cycle (also at 150 rpm by a one-pulse sensor)
xvfb-run -a python3 linuxcnc/test/config_test.py     # the set-up program, worked from its pages against the simulator
```


### With a board on the bench

A bare dev board is enough; nothing needs to be wired to it.

```bash
python tools/protocol_conformance.py <ip> --motion    # the binary protocol over the network (the axes "move": no motors, or a free machine)
python tools/grbl_serial_test.py COM14                # the G-code interface on the USB port: moves, arcs, hold, jog and jog cancel, reset, no drift
python tools/grbl_serial_test.py <ip>                 # the same interface over the network (telnet, port 23)
python tools/ugs_serial_test.py COM14                 # TiggyUGS's own serial connection and sender against it (needs TiggyUGS's source)
python tools/latency_test.py <ip>                     # how promptly the controller answers over the network: median and worst case
linuxcnc/test/run_on_board.sh <ip> --motion           # LinuxCNC itself driving the board (on Linux, or in Windows under WSL): homing, MDI,
                                                      # a program, jogging, abort, E-stop and a long run, each checked to the step
```

With one wire on it - X step linked to the single spindle sensor's terminal, GPIO 1 to GPIO 21 on the DevKitC - the board tests its own step output, its spindle sensor input and screw cutting:

```bash
python tools/spindle_loopback_test.py <ip> --bench    # counts its own step pulses, then cuts threading passes against its spindle output
```

It changes settings (the spindle sensor, and which terminal is X step and which the spindle output) and puts them back at the end, so nothing but the wire may be connected to those two terminals. The file says what is measured and how.

With three links - spindle output to encoder A, charge pump to encoder B, Z step to the index input: GPIO 9 to 16, 39 to 17 and 42 to 21 on the DevKitC with octal memory - the board tests its encoder inputs with no encoder at all:

```bash
python tools/encoder_loopback_test.py <ip> --bench    # A, B and index made by the controller itself: the count at speed, the index pulse, screw cutting
```

The spindle PWM and the charge pump, set to one frequency, are an encoder's A and B; Z stepped by the clock is its index pulse, and a measure of the count as well, since all three come from one crystal. The test checks that the real pulse counter neither loses nor gains a count at up to 50 000 lines a second (200 000 counts, two and a half million in all; the spindle output makes no more than 50 kHz), that every index pulse is taken and found at one place, and that threading passes are played by A and B as they are by a single sensor. What it cannot check is an encoder itself and the wiring to it - for that:

A spindle encoder can be tested the same way before it is on a machine: wired to the controller, turned by hand, then spun with a cordless drill on its shaft.

```bash
python tools/encoder_bench_test.py <ip> --lines 600 --bench    # A, B and the index pulse read by hand, then threading passes by the drill-spun encoder
```

It says what to do next and beeps each time (`--say`: it speaks), and goes by what the controller sees, so there is no hurry. By hand it checks that A and B are counted both ways and that the index pulse comes once a turn at the same place in the count - which is the number of lines being right; if it is not, it says what the encoder looks like, and it names the wire if one is missing. With the drill it checks that no counts are lost at speed, that a pass lies where it should on the spindle and a second pass on the first, that Z stops with the spindle, and that a pass waits for a spindle at rest. It sets the four-axis terminal layout and the sensor settings and puts them back (`--keep`: leaves them), and steps the passes out on Z: no drive may be connected there. The inputs take 3.3 V and no more - the file says how each kind of encoder output is wired.

## Documentation

- [User Manual](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/plugin_manual.html) -- full plugin configuration guide ([markdown](Tiggy_Plugin_Manual.md))
- [Engineering Reference](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/engineering_reference.html) -- protocol, wiring diagrams, and internals

## License

MIT License -- see [LICENSE](LICENSE)

The ESP32 firmware, LinuxCNC HAL component, protocol definition, and tools are all open source and provide **full 6-axis support at no cost**. The Mach3 plugin is distributed separately -- its Free tier supports 3 axes; a Pro Mach3 license unlocks all 6 axes and additional features.
