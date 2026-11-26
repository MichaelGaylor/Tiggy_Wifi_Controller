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
- Spindle encoder speed feedback
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
./install.sh    # builds the driver, then asks about your machine
```

Or double-click `install.desktop` from your file manager.

The installer writes a complete configuration to `~/linuxcnc/configs`. No realtime kernel is needed: LinuxCNC plans the motion and the controller makes the step pulses. What to expect, and every pin and setting of the `tiggy` driver, is in [linuxcnc/README.md](linuxcnc/README.md).

### Mach3 (Windows)

Download the Mach3 plugin from [www.tiggyengineering.com](https://www.tiggyengineering.com), then:

1. Copy `Tiggy.dll` to `C:\Mach3\PlugIns\` and `M62-M65.m1s` to `C:\Mach3\macros\Mach3Mill\`
2. Start Mach3 and choose **Tiggy Motion Controller** as the motion device
3. Set the machine up in Mach3's own Config menu (Motor Tuning, Ports and Pins, Homing/Limits) -- the plugin finds the controller by itself

Plugin 2.0 needs controller firmware 1.2.0 or newer. See the [User Manual](Tiggy_Plugin_Manual.md).

**Free tier**: 3 axes (X/Y/Z). **Pro Mach3 license**: 6 axes, slaved axes, I/O expansion module. Note: this axis limit applies **only to the Mach3 plugin**. LinuxCNC, GRBL senders, and all other host software get full 6-axis support for free.

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
python tools/firmware_selftest.py         # G-code mode moves, feed hold, homing, starved stream, pulse spacing
```

Host software (the Mach3 plugin, LinuxCNC) connects to it like to a board. `tools/tiggy_sim.py` is an independent model of the same protocol in Python; the conformance tests hold both to one behaviour, and can be run against a real board too (`protocol_conformance.py <ip>`, add `--motion` only when the motors are free to move).

The LinuxCNC driver is tested the same way. It builds, unchanged, around a stand-in for LinuxCNC and is run against the firmware simulator (it starts its own simulators, so close one you started by hand first):

```bash
linuxcnc/test/build.sh                    # or build.bat on Windows
python tools/linuxcnc_driver_test.py      # moves to the step, packet loss, a stalling PC, E-stop, limits, reconnection
linuxcnc/test/run_in_linuxcnc.sh          # the same driver inside a real LinuxCNC (Linux with linuxcnc-uspace-dev)
```


## Documentation

- [User Manual](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/plugin_manual.html) -- full plugin configuration guide ([markdown](Tiggy_Plugin_Manual.md))
- [Engineering Reference](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/engineering_reference.html) -- protocol, wiring diagrams, and internals

## License

MIT License -- see [LICENSE](LICENSE)

The ESP32 firmware, LinuxCNC HAL component, protocol definition, and tools are all open source and provide **full 6-axis support at no cost**. The Mach3 plugin is distributed separately -- its Free tier supports 3 axes; a Pro Mach3 license unlocks all 6 axes and additional features.
