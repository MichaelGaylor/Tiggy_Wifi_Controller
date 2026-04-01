# Tiggy WiFi CNC Motion Controller

A multi-platform wireless CNC motion controller for Mach3 (Windows) and LinuxCNC (Linux), powered by ESP32-S3 firmware.

Replaces Mach3's parallel port driver or LinuxCNC's parallel port HAL with a WiFi or Ethernet-connected motion controller supporting up to 6 axes, spindle encoder feedback, and expanded I/O.

## Downloads

| Platform | Download | Description |
|----------|----------|-------------|
| **Windows (Mach3)** | [Tiggy-Windows-Mach3.zip](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/Tiggy-Windows-Mach3.zip) | Plugin DLL, pin maps, M-code macros |
| **Linux (LinuxCNC)** | [Tiggy-LinuxCNC.tar.gz](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/Tiggy-LinuxCNC.tar.gz) | HAL component source + installer |

Builds are created automatically on every push to `main` via GitHub Actions.

## Features

- Up to **6 axes** (X, Y, Z, A, B, C) with independent step/direction control
- **WiFi or Ethernet** (W5500) connectivity -- auto-detected, no configuration needed
- Real-time position feedback and DRO display
- Limit switches, home switches, E-stop, and probe inputs
- Spindle PWM (CW/CCW), coolant (flood/mist), charge pump outputs
- **Spindle encoder** feedback for threading (G33/G76) and CSS (G96)
- Signal inversion via Mach3's native **Ports and Pins** dialog
- 5 misc outputs (M62-M65) and 4 misc inputs with function mapping
- Optional **I/O expansion module** (pendant buttons, relay outputs, B/C axis jog)
- Board pin map profiles for different hardware configurations
- Auto-discovery and automatic reconnection

## Supported Hardware

| Board | MCU | Axes | Notes |
|-------|-----|------|-------|
| Tiggy Standard | ESP32-S3-Zero (FH4R2) | 3 (X/Y/Z) | Compact, Quad SPI PSRAM |
| Tiggy Pro | ESP32-S3-DevKitC-1 (N16R8) | 6 (X/Y/Z/A/B/C) | Full I/O, Octal SPI PSRAM |
| Classic | ESP32-WROOM-32 | 6 (limited I/O) | Legacy support |

## Installation

### Windows (Mach3)

1. Download [Tiggy-Windows-Mach3.zip](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/Tiggy-Windows-Mach3.zip)
2. Close Mach3 completely
3. Extract `Tiggy.dll` and `*.pinmap` files to `C:\Mach3\PlugIns\`
4. Extract `M62.m1s` through `M65.m1s` to `C:\Mach3\macros\Mach3Mill\`
5. Launch Mach3
6. Go to **Config > Select Motion Device** and choose **Tiggy Motion Controller**
7. Restart Mach3

The plugin will auto-connect to the controller. Configure your ESP32's IP in **Config > Tiggy Motion Controller...**

### Linux (LinuxCNC)

1. Download [Tiggy-LinuxCNC.tar.gz](https://github.com/MichaelGaylor/Tiggy_Wifi_Controller/releases/latest/download/Tiggy-LinuxCNC.tar.gz)
2. Extract to any folder
3. Double-click `install.desktop` (or run `./install.sh` from a terminal)
4. Follow the interactive setup wizard
5. Launch LinuxCNC and select your new configuration

**Prerequisites:** LinuxCNC must be installed (`sudo apt install linuxcnc-uspace linuxcnc-uspace-dev`).

### ESP32 Firmware

Firmware is built with [PlatformIO](https://platformio.org/):

```bash
cd firmware
pio run                      # Compile (default: esp32s3-devkitc)
pio run -t upload            # Upload to connected board
pio run -e esp32s3-zero -t upload   # Upload to S3-Zero board
```

## Configuration

### Mach3 Configuration

All settings are in the plugin's own dialog -- you do **not** need to configure Mach3's Motor Tuning or Ports and Pins.

Open the settings via **Config > Tiggy Motion Controller...** in the Mach3 menu.

### Plugin Settings Dialog

Open via **Config > Tiggy Motion Controller...** in the Mach3 menu. Tabs:

1. **Connection** -- IP address, connect/disconnect, live spindle RPM
2. **Axis Config** -- Steps/unit, velocity, acceleration per axis
3. **Inputs/Outputs** -- Live signal status, misc input function mapping
4. **Advanced** -- Step timing, homing params, spindle PWM, charge pump, laser mode
5. **I/O Module** -- Expansion module config (pendant jog, relay outputs)
6. **Pin Map** -- Board GPIO pin assignments

## Architecture

```
Mach3 / LinuxCNC
       |
   [TCP:8080 + UDP:8081]
       |
  ESP32-S3 Firmware
       |
  Step/Dir/Enable/Spindle/IO GPIO
```

- **TCP** (port 8080): Handshake, configuration, commands
- **UDP** (port 8081): Real-time motion packets and status reports
- **Protocol**: Defined in `protocol/wifi_cnc_protocol.h` (shared between all platforms)

## Documentation

Full user manual: [Tiggy_Plugin_Manual.md](Tiggy_Plugin_Manual.md)

## Building from Source

### Windows Plugin (Tiggy.dll)

Requires Visual Studio 2022 Build Tools:

```
cd plugin\WiFiCNC
build.bat
```

The DLL is output to `plugin\WiFiCNC\bin\Tiggy.dll` and auto-deployed to `C:\Mach3\PlugIns\` if Mach3 is installed.

### LinuxCNC HAL Component

Requires LinuxCNC development headers:

```bash
cd linuxcnc
chmod +x install.sh
./install.sh
```

### ESP32 Firmware

Requires PlatformIO:

```bash
cd firmware
pio run -e esp32s3-devkitc    # 6-axis DevKitC
pio run -e esp32s3-zero       # 3-axis S3-Zero
pio run -e esp32-wroom32      # Classic ESP32
```

## License

This is a private repository. All rights reserved.
