# Tiggy WiFi CNC Motion Controller

Open-source ESP32 CNC motion controller firmware with LinuxCNC support.

Replaces parallel port drivers with a WiFi or Ethernet-connected motion controller supporting up to 6 axes, spindle encoder feedback, and expanded I/O. Works standalone (GRBL-compatible serial/telnet), with LinuxCNC, or with Mach3 (plugin sold separately).

## Downloads

| Component | Download | Description |
|-----------|----------|-------------|
| **ESP32 Firmware** | [Flash from browser](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/) | One-click web flasher (Chrome/Edge) |
| **LinuxCNC** | [Tiggy-LinuxCNC.tar.gz](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/Tiggy-LinuxCNC.tar.gz) | HAL component + interactive installer |
| **Mach3 Plugin** | [www.tiggyengineering.com](https://www.tiggyengineering.com) | Windows plugin (free: 3-axis, Pro: 6-axis) |

Firmware and LinuxCNC builds are created automatically via GitHub Actions.

## Features

- Up to **6 axes** (X, Y, Z, A, B, C) with independent step/direction control
- **WiFi or Ethernet** (W5500) connectivity -- auto-detected, no configuration needed
- GRBL-compatible serial/telnet interface for standalone use
- Real-time position feedback and DRO display
- Limit switches, home switches, E-stop, and probe inputs
- Spindle PWM (CW/CCW), coolant (flood/mist), charge pump outputs
- **Spindle encoder** feedback for threading (G33/G76) and CSS (G96)
- 5 misc outputs and 4 misc inputs with function mapping
- Optional **I/O expansion module** (pendant buttons, relay outputs)
- Board pin map profiles for different hardware configurations
- Auto-discovery and automatic reconnection

## Supported Hardware

| Board | MCU | Axes | Notes |
|-------|-----|------|-------|
| Tiggy Standard | ESP32-S3-Zero (FH4R2) | 3 (X/Y/Z) | Compact, Quad SPI PSRAM |
| Tiggy Pro | ESP32-S3-DevKitC-1 (N16R8) | 6 (X/Y/Z/A/B/C) | Full I/O, Octal SPI PSRAM |
| Classic | ESP32-WROOM-32 | 6 (limited I/O) | Legacy support |

## Quick Start

### Flash Firmware

**Easiest:** Open the [Web Flasher](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/) in Chrome or Edge, plug in your ESP32 via USB, select your board, and click Flash. No software install needed.

**From source** (requires [PlatformIO](https://platformio.org/)):

```bash
cd firmware
pio run -e esp32s3-devkitc -t upload    # 6-axis DevKitC
pio run -e esp32s3-zero -t upload       # 3-axis S3-Zero
```

### LinuxCNC

```bash
tar xzf Tiggy-LinuxCNC.tar.gz
cd linuxcnc-package
chmod +x install.sh
./install.sh    # Interactive setup wizard
```

Or double-click `install.desktop` from your file manager.

**Prerequisites:** `sudo apt install linuxcnc-uspace linuxcnc-uspace-dev`

### Mach3 (Windows)

Download the Mach3 plugin from [www.tiggyengineering.com](https://www.tiggyengineering.com), then:

1. Extract `Tiggy.dll` and `*.pinmap` to `C:\Mach3\PlugIns\`
2. Extract `M62-M65.m1s` to `C:\Mach3\macros\Mach3Mill\`
3. Config > Select Motion Device > Tiggy Motion Controller
4. Restart Mach3

**Free tier**: 3 axes (X/Y/Z). **Pro license**: 6 axes, I/O module, threading.

## Architecture

```
Mach3 / LinuxCNC / Serial Terminal
              |
         [TCP:8080 + UDP:8081]
              |
        ESP32-S3 Firmware (this repo)
              |
        Step/Dir/Enable/Spindle/IO GPIO
```

- **TCP** (port 8080): Handshake, configuration, commands
- **UDP** (port 8081): Real-time motion packets and status reports
- **Protocol**: Defined in `protocol/wifi_cnc_protocol.h`

## Documentation

- [User Manual](Tiggy_Plugin_Manual.md) -- full plugin configuration guide
- [Engineering Reference](docs/engineering_reference.html) -- protocol and internals

## License

MIT License -- see [LICENSE](LICENSE)

The ESP32 firmware, LinuxCNC HAL component, protocol definition, and tools are all open source. The Mach3 plugin is distributed separately.
