# Tiggy Motion Controller Plugin - User Manual

## 1. Overview

The Tiggy Motion Controller plugin replaces Mach3's parallel port driver with a wireless (or wired Ethernet) motion controller. It communicates over TCP (control/config) and UDP (real-time motion/status) to deliver step pulses, read inputs, and control spindle/coolant outputs.

**Key features:**

- Up to 6 axes (X, Y, Z, A, B, C) with independent step/direction control -- Pro Mach3 license required for A/B/C axes; Free tier supports X/Y/Z
- Axis configuration (steps/unit, velocity, accel) in the plugin dialog
- Signal inversion configured in the plugin's own settings dialog
- Real-time position feedback and DRO display
- Limit switches, home switches, E-stop, and probe inputs
- Spindle PWM (CW/CCW), coolant (flood/mist), charge pump outputs
- Spindle encoder feedback for threading (G33/G76) and CSS (G96)
- Up to 5 misc outputs (EXTACT1-5) and up to 4 misc inputs (ACTIVATION1-4) -- actual count depends on board variant
- Input function mapping: assign physical buttons to Mach3 actions
- I/O expansion module with B/C axis jog support
- Board pin map profiles for different hardware configurations
- Auto-discovery of controllers on the network
- Automatic reconnection with exponential backoff

**Supported boards:**

| Board | MCU | Wired Axes | Misc Inputs |
|-------|-----|------------|-------------|
| Tiggy Standard board (ESP32-S3-Zero) | Waveshare ESP32-S3-Zero (FH4R2) | 3 (X/Y/Z) | 0 |
| Tiggy Pro board (ESP32-S3-DevKitC, Octal) | ESP32-S3-DevKitC-1 N16R8 / N8R8 | 6 (X/Y/Z/A/B/C) | 2 |
| Tiggy Pro board (ESP32-S3-DevKitC, Quad) | ESP32-S3-DevKitC-1 N8 / N8R2 | 6 (X/Y/Z/A/B/C) | 3 |
| Classic board (ESP32-WROOM-32) | ESP32-WROOM-32 DevKit | 6 (limited I/O) | 0 |

**Mach3 Plugin Licensing:** The Free tier supports 3 axes (X/Y/Z). The Pro Mach3 license unlocks 6 axes, I/O expansion module, and spindle encoder threading. Note: the ESP32 firmware is open source and supports all 6 axes with no restrictions. If you use LinuxCNC, GRBL senders, or any other host software, all 6 axes work for free. The 3-axis limit applies **only** to the Mach3 plugin. Download from [www.tiggyengineering.com](https://www.tiggyengineering.com).

---

## 2. Installation

### Files

| File | Location | Purpose |
|------|----------|---------|
| `Tiggy.dll` | `C:\Mach3\PlugIns\` | The plugin DLL |
| `*.pinmap` | `C:\Mach3\PlugIns\` | Board pin map files (one per board type) |
| `M62.m1s` - `M65.m1s` | `C:\Mach3\macros\Mach3Mill\` | VBScript macros for digital output control |

### Steps

1. Close Mach3 completely.
2. Copy `Tiggy.dll` to `C:\Mach3\PlugIns\`.
3. Copy the `.pinmap` files (e.g. `tiggy_standard.pinmap`, `tiggy_pro.pinmap`) to the same `C:\Mach3\PlugIns\` folder.
4. Copy `M62.m1s`, `M63.m1s`, `M64.m1s`, and `M65.m1s` to `C:\Mach3\macros\Mach3Mill\` (or your active profile's macro folder).
5. Launch Mach3.
6. Go to **Config > Select Motion Device** and choose **Tiggy Motion Controller**.
7. Restart Mach3. The plugin will load and attempt to connect to the controller.

### First Run

On first run the plugin creates default settings in the Windows registry under `HKEY_CURRENT_USER\Software\Tiggy`. It will try to connect to `192.168.4.1` (the controller's default access-point IP). If that fails, it runs a 5-second UDP auto-discovery broadcast.

---

## 3. Configuration Dialog

Open the plugin settings via **Config > Tiggy Motion Controller...** in the Mach3 menu bar.

The dialog has **6 tabs**:

### Tab 1: Connection

| Control | Description |
|---------|-------------|
| Controller IP Address | The IP address of your controller |
| Connect | Manually connect to the entered IP |
| Disconnect | Disconnect from the current controller |
| Auto-Discover | Broadcast a discovery packet to find the controller on your network |
| Status | Shows CONNECTED or DISCONNECTED |
| Spindle RPM | Live RPM reading from the spindle encoder (updated every 500ms) |

The IP address is saved to the registry. On each Mach3 startup the plugin tries:
1. The saved IP
2. UDP auto-discovery (5 second timeout)
3. Default AP IP `192.168.4.1`

**Spindle Encoder:** The motion controller board has a PCNT-based spindle encoder input. The RPM is read from the controller's status reports and displayed live on this tab. If no encoder is connected, the RPM reads 0.

### Tab 2: Axis Config

Replaces Mach3's Motor Tuning panel. The plugin owns all motion parameters.

| Column | Description |
|--------|-------------|
| On | Enable/disable the axis |
| Steps/Unit | Steps per user unit (mm or inch, matches your Mach3 units setting) |
| Velocity (u/min) | Maximum velocity in user units per minute |
| Accel (u/s^2) | Acceleration in user units per second squared |

**All 6 axes** (X, Y, Z, A, B, C) are listed. Only enable the axes your machine physically has. Axes A, B, and C require a Pro Mach3 license.

These values are written into Mach3's internal MainPlanner structure so that DRO display works correctly. You do **not** need to configure Motor Tuning in Mach3 separately.

### Tab 3: Inputs / Outputs

This tab has three sections:

#### Input Status (live from controller)

Displays real-time state of all input signals, updated every 200ms while the tab is open:

| Signal | Source |
|--------|--------|
| X/Y/Z/A/B/C Limit | `status.limit_switches` bitmask |
| X/Y/Z/A/B/C Home | `status.home_switches` bitmask |
| E-Stop | `status.estop_input` |
| Probe | `status.probe_state` |

Each input signal has an **Inv** (invert) checkbox. E-Stop and Probe also have inversion checkboxes.

**Inversion behaviour:**

| Signal | Raw Source | Inversion Applied By |
|--------|-----------|---------------------|
| Limit switches | Firmware debounce callback | Plugin sends inversion bitmask to firmware |
| Home switches | Firmware homing cycle | Plugin sends inversion bitmask to firmware |
| E-Stop | Firmware sends raw GPIO state | Plugin applies inversion |
| Probe | Firmware sends raw GPIO state | Plugin applies inversion |

**Tip:** If a signal shows ON with nothing connected, tick its Inv checkbox.

#### Misc Input Functions (button press -> action)

Four dropdown selectors, one for each misc input (Input 1 through Input 4). Each dropdown maps a physical button press on the controller's GPIO to a Mach3 action:

| Function | DoButton Code | Description |
|----------|--------------|-------------|
| None | 0 | No action (disabled) |
| Cycle Start | 1000 | Starts G-code execution |
| Feed Hold | 1001 | Pauses motion (resumable) |
| Stop | 1003 | Stops the current program |
| E-Stop | 1021 | Triggers emergency stop |
| Reset | 1022 | Resets Mach3 after E-stop |
| Rewind | 1010 | Rewinds to start of program |
| Single Block | 1002 | Toggles single-block mode |

**How it works:** The plugin monitors `ACTIVATION1` through `ACTIVATION4` input signals (mapped from the controller's misc input GPIOs). On a **rising edge** (0 to 1 transition), it calls Mach3's `DoButton()` function with the configured code. This means pressing a physical button once triggers the action once -- holding the button does not repeat the action.

**Example setup:**
- Input 1 = Cycle Start (wire a green "Go" button to Misc In 1 on the controller)
- Input 2 = Feed Hold (wire a yellow "Pause" button to Misc In 2)
- Input 3 = E-Stop (wire a red mushroom button to Misc In 3)
- Input 4 = Reset (wire a momentary "Reset" button to Misc In 4)

**Note:** The Tiggy Standard board has no misc input GPIOs assigned (all show `n/a` in Pin Map). The Tiggy Pro board (Octal, default) has 2 misc inputs on GPIO 18 and 21. The Tiggy Pro board (Quad) has 3 misc inputs on GPIO 18, 21, and 36. (GPIO 47 is the W5500 Ethernet interrupt pin.)


### Tab 4: Advanced

#### Timing

| Setting | Default | Description |
|---------|---------|-------------|
| Step Pulse | 5 us | Width of each step pulse in microseconds |
| Dir Setup | 5 us | Time between direction change and first step pulse |

#### Step/Dir Inversion

Per-axis checkboxes to invert the step and direction signals. Use these if your stepper drivers expect opposite polarity.

#### Homing

| Setting | Default | Description |
|---------|---------|-------------|
| Direction (+) | All unchecked | Per-axis: tick if the axis should home toward the positive end |
| Seek rate | 500 steps/sec | Fast approach speed during homing |
| Feed rate | 50 steps/sec | Slow approach speed (fine positioning) |
| Pulloff | 200 steps | Distance to back off after finding the switch |

**How homing works:** When you press "Ref All Home" in Mach3, the plugin sends a home command to the controller with a bitmask of all enabled axes. The controller firmware handles the entire homing sequence (seek, find switch, back off, re-approach slowly, set zero). On completion, the plugin zeros the axis positions and sets the `Referenced` flag.

**Homing tips:**
- If an axis homes the wrong direction, tick its Direction (+) checkbox
- If homing completes instantly without touching the switch, the home switch inversion is probably wrong -- check the Inputs tab
- Increase Seek rate for faster homing on long-travel machines
- Increase Pulloff if you need more clearance after homing

#### Spindle

| Setting | Default | Description |
|---------|---------|-------------|
| PWM Freq | 1000 Hz | Spindle PWM frequency |
| Max RPM | 24000 | Maximum spindle RPM (maps to 100% PWM duty) |

#### Charge Pump / Step Idle / Laser

| Setting | Default | Description |
|---------|---------|-------------|
| Charge Pump | 0 Hz | Charge pump PWM frequency. 0 = disabled. Set to e.g. 10000 for a 10kHz charge pump signal |
| Step Idle | 0 ms | Time after last motion before disabling stepper drivers. 0 = never disable |
| Laser Mode | Unchecked | When enabled, spindle PWM output tracks the current feed rate proportionally. Used for laser cutting/engraving where power must vary with speed |

### Tab 5: I/O Module

Configures the optional I/O expansion module -- a second controller board running in I/O module mode, used for pendant buttons and external relay outputs.

| Control | Description |
|---------|-------------|
| Enable I/O Module | Connect to the I/O module on startup |
| IP | IP address of the I/O expansion module |
| Status | Shows Connected or Not connected |

#### Input Functions

Eight input channels (In 0 - In 7) can each be assigned a function:

| Function | Code | Description |
|----------|------|-------------|
| None | 0 | Disabled |
| Jog X+/X-/Y+/Y-/Z+/Z-/A+/A-/B+/B-/C+/C- | 101-112 | Jog the specified axis in the specified direction |
| Cycle Start | 1000 | Start G-code execution |
| Feed Hold | 1001 | Pause motion |
| Stop | 1003 | Stop the current program |
| E-Stop | 1021 | Emergency stop |
| Reset | 1022 | Reset after E-stop |

The I/O module sends button press events to the plugin. Jog speed is taken from the axis configuration (Axis Config tab velocity setting), not set separately on the I/O module.

#### Output Functions

Eight output channels (Out 0 - Out 7) can each mirror a Mach3 signal:

| Function | Code | Description |
|----------|------|-------------|
| None | 0 | Disabled |
| Spindle CW | 1 | Mirrors spindle clockwise state |
| Spindle CCW | 2 | Mirrors spindle counter-clockwise state |
| Flood | 3 | Mirrors flood coolant state |
| Mist | 4 | Mirrors mist coolant state |
| Output 1-4 | 5-8 | Mirrors misc output states |

**How to set up an I/O module (firmware side):**

The I/O module is a second ESP32 board running the same firmware, with one setting changed. No separate download needed.

1. Flash the same firmware onto a second ESP32 board using the [Web Flasher](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/)
2. Connect to the board's WiFi hotspot (`WiFiCNC-XXXX`) and open the Protocol Tester at `192.168.4.1`
3. Go to the **WiFi** tab, enter your network name and password, click **Save**. The board will reboot and join your network
4. Reconnect to your normal network, use **Discover** to find the board's new IP address
5. In the **Config** tab, I/O Module section: change **Device Mode** to "I/O Module"
6. Set **Channels** to the number of buttons/outputs you need (e.g. 8)
7. Set the **Dir Mask** -- each bit controls whether that channel is an input (0) or output (1). For example: `000F` makes channels 0-3 outputs and 4-15 inputs
8. Set GPIO pin numbers for each channel in the pin fields (255 = unused)
9. Click **Write & Save**, then reboot the board
10. Come back to the Mach3 plugin's I/O Module tab (above) and enter the I/O module's IP address

See the [Engineering Reference](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/engineering_reference.html#io-module-setup) for a detailed walkthrough with examples.

**How it works:** The I/O module connects via its own TCP/UDP channels and reports input states. The plugin sends output states based on the function mapping above. The module does not run stepper or motion code -- it only handles digital I/O. The plugin tells them apart by the device mode field in the connection handshake. Each device has its own IP address.

### Tab 6: Pin Map

Displays the GPIO pin assignments for the selected controller board.

#### Board Selector

A dropdown listing all board profiles found in the `PlugIns` folder (loaded from `.pinmap` files). Select your board to see its pin assignments.

#### Pin Display

Read-only display of all GPIO assignments, organised into groups:

- **Step / Direction Pins** -- 6 axes, step and direction GPIO for each
- **Limit / Home Switch Pins** -- 6 axes, limit and home GPIO for each
- **Output Pins** -- Enable, Spindle, Charge Pump, Misc Out 1-2
- **Input Pins** -- E-Stop, Probe, Misc In 1-4, Status LED

A pin value of `n/a` means that function is not available on the selected board.

---

## 4. Pin Map Files

Pin map files (`.pinmap`) are plain text files that define the GPIO assignments for a specific controller board. They use a simple INI-style format.

### File Location

Place `.pinmap` files in the same folder as the plugin DLL: `C:\Mach3\PlugIns\`

The plugin scans this folder on startup and when the config dialog opens, finding all files with the `.pinmap` extension.

### File Format

```ini
# Comment lines start with #

[Board]
Name=Tiggy Standard Motion Controller
Description=Compact 3-axis controller

[Step]
X=1
Y=2
Z=42
A=-1
B=-1
C=-1

[Direction]
X=3
Y=4
Z=41
A=-1
B=-1
C=-1

[Limits]
X=5
Y=6
Z=7
A=-1
B=-1
C=-1

[Home]
X=5
Y=6
Z=7
A=-1
B=-1
C=-1

[Outputs]
Enable=8
Spindle=9
ChargePump=10
MiscOut1=11
MiscOut2=12
LED=48

[Inputs]
EStop=13
Probe=14
MiscIn1=-1
MiscIn2=-1
MiscIn3=-1
MiscIn4=-1
```

### Sections

| Section | Keys | Description |
|---------|------|-------------|
| `[Board]` | `Name`, `Description` | Board identity. Name is used in the dropdown selector |
| `[Step]` | `X`, `Y`, `Z`, `A`, `B`, `C` | GPIO numbers for step pulse outputs |
| `[Direction]` | `X`, `Y`, `Z`, `A`, `B`, `C` | GPIO numbers for direction outputs |
| `[Limits]` | `X`, `Y`, `Z`, `A`, `B`, `C` | GPIO numbers for limit switch inputs |
| `[Home]` | `X`, `Y`, `Z`, `A`, `B`, `C` | GPIO numbers for home switch inputs |
| `[Outputs]` | `Enable`, `Spindle`, `ChargePump`, `MiscOut1`, `MiscOut2`, `LED` | GPIO numbers for output pins |
| `[Inputs]` | `EStop`, `Probe`, `MiscIn1`-`MiscIn4` | GPIO numbers for input pins |

A pin value of `-1` means the function is not available on this board.

### Creating a Custom Pin Map

1. Copy an existing `.pinmap` file and rename it (e.g. `my_custom_board.pinmap`)
2. Edit the `[Board]` section with your board's name and description
3. Update all GPIO numbers to match your hardware wiring
4. Place the file in `C:\Mach3\PlugIns\`
5. Open the plugin config dialog -- your board will appear in the Pin Map tab dropdown

---

## 5. Outputs

### Spindle

The plugin monitors `Engine->SpindleCW` and `Engine->SpindleCCW` and sends spindle state to the controller in real-time. Supported G-codes:

| G-Code | Action |
|--------|--------|
| M3 Sxxxx | Spindle CW at xxxx RPM |
| M4 Sxxxx | Spindle CCW at xxxx RPM |
| M5 | Spindle stop |

RPM is mapped to PWM duty cycle based on the Max RPM setting.

### Coolant

| G-Code | Action |
|--------|--------|
| M7 | Mist coolant ON |
| M8 | Flood coolant ON |
| M9 | All coolant OFF |

### Misc Outputs (EXTACT1-5)

Five general-purpose digital outputs controlled via Mach3's external activation signals. These map to the controller's `misc_outputs` bitmask:

| Signal | Bit | Immediate On/Off | Motion-Synced On/Off |
|--------|-----|------------------|---------------------|
| EXTACT1 (Output #1) | 0 | M64 P0 / M65 P0 | M62 P0 / M63 P0 |
| EXTACT2 (Output #2) | 1 | M64 P1 / M65 P1 | M62 P1 / M63 P1 |
| EXTACT3 (Output #3) | 2 | M64 P2 / M65 P2 | M62 P2 / M63 P2 |
| EXTACT4 (Output #4) | 3 | M64 P3 / M65 P3 | M62 P3 / M63 P3 |
| EXTACT5 (Output #5) | 4 | M64 P4 / M65 P4 | M62 P4 / M63 P4 |

**Immediate vs motion-synced:**
- **M64/M65** (immediate): The output changes instantly when the M-code is executed, even if motion is still in progress from a previous line.
- **M62/M63** (motion-synced): The macro waits for all queued motion to complete before changing the output. Use these when timing matters (e.g. activating a clamp after a move completes).

**Hardware availability:** The Tiggy Standard board has 2 misc output GPIOs (Output #1 on GPIO 11, Output #2 on GPIO 12). The Tiggy Pro board (Quad) has 2 misc output GPIOs (GPIO 11, GPIO 12) plus a charge pump on GPIO 10. The Tiggy Pro board (Octal, default) has no misc output GPIOs and no charge pump -- these pins are used for A/B/C direction outputs. Outputs #3-#5 are reserved for future expansion.

### M-Code Macros (Required)

Mach3 requires VBScript macro files for M62-M65 to work with external motion controllers. These files **must** be in your Mach3 macro folder (e.g. `C:\Mach3\macros\Mach3Mill\`).

| File | M-Code | Function |
|------|--------|----------|
| `M62.m1s` | M62 Pn | Synchronized digital output ON (waits for motion to finish) |
| `M63.m1s` | M63 Pn | Synchronized digital output OFF (waits for motion to finish) |
| `M64.m1s` | M64 Pn | Immediate digital output ON |
| `M65.m1s` | M65 Pn | Immediate digital output OFF |

The `P` parameter selects which output: P0 = Output #1, P1 = Output #2, etc.

**How the macros work:** Each macro calls `ActivateSignal()` or `DeActivateSignal()` with the Mach3 signal index for the corresponding EXTACT output (EXTACT1 = signal 7, EXTACT2 = signal 8, etc.). The plugin monitors these signals and forwards the state to the controller.

**Example G-code:**
```gcode
G0 X10 Y10          ; Rapid to position
M64 P0              ; Turn on Output #1 immediately
G1 X50 F500         ; Linear move
M62 P1              ; Turn on Output #2 after move completes
G4 P2               ; Dwell 2 seconds
M65 P0              ; Turn off Output #1
M63 P1              ; Turn off Output #2 after any pending motion
```

### Spindle Encoder (Lathe Features)

The controller supports a quadrature spindle encoder with index pulse for lathe operations:

| Feature | G-Code | Description |
|---------|--------|-------------|
| Threading | G33 / G76 | Spindle-synchronized Z moves for thread cutting |
| CSS | G96 | Constant Surface Speed -- RPM auto-adjusts with X position |
| RPM mode | G97 | Fixed RPM mode (standard) |

**Setup:**
1. Connect encoder A, B, and Index signals to the GPIO pins shown in Pin Map
2. The encoder PPR (pulses per revolution) is detected at handshake
3. Spindle position is fed to Mach3's `Engine->CurrentSpindleCount` at 50ms intervals
4. Index pulse edges are reported to Mach3 for thread start synchronization
5. Set Mach3 to **Lathe mode** in Config > General Config (this is a Mach3 setting, not plugin-controlled)

Live spindle RPM is displayed on the Connection tab.

### Charge Pump

When enabled (frequency > 0), the plugin sends a PWM frequency to the controller's charge pump GPIO. The charge pump activates when:
- Mach3's "Charge Pump Always On" is enabled, OR
- The CHARGE output signal is activated

Setting the frequency to 0 disables the charge pump.

### Stepper Enable

The shared stepper enable pin is controlled by the controller firmware. The Step Idle setting determines how long after the last motion before steppers are disabled (0 = never disable).

---

## 6. Inputs

### Limit Switches

One limit switch input per axis. A single GPIO serves both the positive and negative limit (the plugin sets both `LIMITPLUS` and `LIMITMINUS` from the same bit). Inversion is applied by the plugin.

When a limit switch triggers during motion, Mach3 will E-stop.

### Home Switches

One home switch input per axis. Often shared with the limit switch GPIO (same physical switch). Inversion is applied by the **firmware** (sent via `WCNC_CFG_INVERT_HOME`).

### E-Stop

Single GPIO input. Inversion applied by the plugin. When triggered, the plugin:
1. Sets `Engine->InSigs[EMERGENCY].Activated = true`
2. Mach3 enters E-stop state
3. Plugin sends E-stop command to controller

If the controller reports ESTOP or ALARM state, the plugin also triggers Mach3's E-stop via `DoButton(OEM_ESTOP)`.

### Probe

Single GPIO input for touch probes and tool length sensors. Inversion applied by the plugin. Used by G38.2 probing cycles.

### Misc Inputs (ACTIVATION1-4)

Four general-purpose digital inputs from controller GPIO. These show up in Mach3 as Input #1 through Input #4. Each can be assigned a function via the Input Function Mapping dropdowns (see Tab 3: Inputs).

---

## 7. Motion Control

### Trajectory Processing

The plugin reads Mach3's trajectory buffer (`Engine->TrajBuffer`) at ~10-50Hz, converts trajectory points into motion segments, and sends them to the controller over UDP. The data path:

```
Mach3 Trajectory Buffer -> SegmentBuilder -> BufferManager -> NetworkClient -> controller
```

Flow control is managed by monitoring the controller's buffer occupancy in its status reports.

### Jogging

Jogging is handled directly by the plugin. Keyboard jog (without Shift) uses the plugin's configured velocity. Shift+arrow uses Mach3's computed jog speed. The plugin sends jog commands to the controller as continuous motion with acceleration.

### Dwell

G4 (dwell) commands are sent to the controller as zero-motion segments with the specified duration.

### Feed Hold / Resume

Feed hold (pressing "Feed Hold" or via G-code) sends a hold command to the controller, which decelerates to a stop. Resume sends a continue command.

---

## 8. Connection & Protocol

### Network Architecture

| Channel | Protocol | Port | Purpose |
|---------|----------|------|---------|
| Control | TCP | 58429 | Handshake, config, commands |
| Motion | UDP | 58427 | Motion segments, jog, E-stop, home, I/O (PC to controller) |
| Status | UDP | 58428 | Status reports (controller to PC, 50ms interval) |

### WiFi vs Ethernet (W5500)

The controller supports both WiFi and wired Ethernet via a W5500 SPI Ethernet module (such as the USR-ES1 breakout, available cheaply on Amazon/eBay). **No software configuration is needed** -- the firmware automatically detects the W5500 at power-on. If a W5500 module is wired up and an Ethernet cable is plugged in, the controller uses Ethernet. If no W5500 is detected, WiFi starts as normal.

The plugin does not need to know whether the controller is using WiFi or Ethernet -- it connects by IP address either way.

**Default wiring (Tiggy Pro board):**

| W5500 Module Pin | Connect to | Notes |
|------------------|-----------|-------|
| 3.3V | ESP32 3V3 pin | Power supply |
| GND | ESP32 GND pin | Ground |
| MOSI | GPIO 45 | SPI data out |
| MISO | GPIO 46 | SPI data in |
| SCLK | GPIO 0 | SPI clock |
| INT | GPIO 47 | Interrupt |
| CS | Tie to GND | Always selected (saves a pin) |
| RST | Tie to 3.3V | Always running (saves a pin) |

Pins can be changed using the Protocol Tester's Pins tab if your hardware uses different GPIOs. See the [Engineering Reference](https://michaelgaylor.github.io/Tiggy_Wifi_Controller/engineering_reference.html#ethernet-setup) for a full wiring diagram.

### USB Serial (G-code Only)

The USB port on the ESP32 provides a serial connection at 115,200 baud for GRBL-compatible G-code senders (UGS, CNCjs, bCNC, LaserGRBL). This gives you full 6-axis control over USB without needing WiFi or Ethernet. However, the Mach3 plugin and LinuxCNC HAL require WiFi or Ethernet -- they use the binary protocol which is not available over USB.

### Connection Sequence

1. TCP connect to controller
2. Handshake (exchange version, capabilities, buffer size)
3. Sync all configuration to controller (timing, inversion, axis params, homing, spindle, etc.)
4. Start UDP channel
5. Send RESET to clear any stale state
6. Begin normal operation

### Auto-Discovery

The plugin broadcasts a UDP discovery packet on port 8082. Any controller running the Tiggy firmware will respond with its IP address. Discovery has a 5-second timeout.

### Reconnection

If the connection drops, the plugin automatically attempts to reconnect with exponential backoff (1s, 2s, 4s, 8s, 16s, up to 30s maximum).

---

## 9. Registry Settings

All plugin settings are stored in `HKEY_CURRENT_USER\Software\Tiggy`. These are loaded on startup and saved when the config dialog is closed with OK.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| ESPAddress | String | 192.168.4.1 | controller IP address |
| X_Enabled .. C_Enabled | DWORD | X/Y/Z=1, A/B/C=0 | Per-axis enable |
| X_StepsPerUnit .. C_StepsPerUnit | String | 200.0 | Steps per user unit |
| X_VelPerMin .. C_VelPerMin | String | 1000.0 | Max velocity (units/min) |
| X_Accel .. C_Accel | String | 100.0 | Acceleration (units/s^2) |
| StepPulseUs | DWORD | 5 | Step pulse width (us) |
| DirSetupUs | DWORD | 5 | Dir setup time (us) |
| InvertStep | DWORD | 0 | Step inversion bitmask |
| InvertDir | DWORD | 0 | Direction inversion bitmask |
| InvertLimit | DWORD | 0 | Limit switch inversion bitmask |
| InvertHome | DWORD | 0 | Home switch inversion bitmask |
| InvertEstop | DWORD | 0 | E-stop inversion (0 or 1) |
| InvertProbe | DWORD | 0 | Probe inversion (0 or 1) |
| HomingDirMask | DWORD | 0 | Homing direction bitmask |
| HomingSeekRate | DWORD | 500 | Homing seek rate (steps/sec) |
| HomingFeedRate | DWORD | 50 | Homing feed rate (steps/sec) |
| HomingPulloff | DWORD | 200 | Homing pulloff (steps) |
| SpindlePwmFreq | DWORD | 1000 | Spindle PWM frequency (Hz) |
| SpindleMaxRpm | DWORD | 24000 | Spindle max RPM |
| ChargePumpFreq | DWORD | 0 | Charge pump frequency (Hz) |
| StepIdleDelayMs | DWORD | 0 | Step idle delay (ms) |
| LaserMode | DWORD | 0 | Laser mode (0/1) |
| MiscInputFunc1 .. MiscInputFunc4 | DWORD | 0 | Input function mapping codes |
| BoardName | String | Tiggy Standard... | Selected board profile name |
| IOModuleAddress | String | (empty) | I/O expansion module IP |
| IOModuleEnabled | DWORD | 0 | Enable I/O module (0/1) |
| IOInputFunc0 .. IOInputFunc15 | DWORD | 0 | I/O module input function codes |
| IOOutputFunc0 .. IOOutputFunc15 | DWORD | 0 | I/O module output function codes |
| IOJogSpeed | DWORD | 5000 | I/O module jog speed (steps/sec) |

---

## 10. Debug Log

The plugin writes a debug log to `Tiggy_debug.log` in the same folder as the DLL (`C:\Mach3\PlugIns\Tiggy_debug.log`). This log includes:

- Startup/shutdown events
- Connection attempts and results
- Handshake details (firmware version, buffer capacity)
- Configuration sync
- Spindle/coolant state changes
- Homing and probing events
- E-stop transitions
- Misc output changes
- Input function mapping triggers
- Error conditions

The log appends on each Mach3 session (not overwritten).

---

## 11. Troubleshooting

### DROs show #INF or wrong values on startup

The plugin writes axis configuration (steps/unit, velocity, acceleration) into Mach3's MainPlanner during `piPostInitControl()`. If the DROs show incorrect values, open the plugin config dialog and click OK -- this forces a re-apply.

### E-stop triggers with nothing connected

The E-stop GPIO pin may be floating high. Tick the **Inv** checkbox next to E-Stop in the Inputs tab. This inverts the interpretation so an open (unconnected) pin reads as "not triggered".

### Homing completes instantly without touching switches

The firmware thinks the home switches are already triggered. Check the Inputs tab:
- If home switches show **ON** with nothing connected, tick the corresponding **Home switch inversion** checkboxes
- The firmware applies home inversion internally, so these checkboxes directly affect the firmware's interpretation during homing

### Homing only moves one axis

The firmware homes axes sequentially (typically Z first for safety). If axes appear to home instantly, see "Homing completes instantly" above.

### Cannot connect to controller

1. Verify the controller is powered and running the Tiggy firmware
2. Check that your PC and controller are on the same network
3. Try the Auto-Discover button in the Connection tab
4. Check the debug log for connection error details
5. Try the default AP IP: `192.168.4.1`

### Limit switches not working

The plugin enables limit switch signals for all enabled axes. If limits don't trigger:
1. Check the Inputs tab live display - does the switch show ON when physically pressed?
2. If it shows the wrong state, tick the Inv checkbox for that axis
3. Verify the limit GPIO is correctly wired to the pin shown in the Pin Map tab

### Input function buttons don't trigger

1. Verify the board has misc input GPIOs assigned (check Pin Map tab - Tiggy Standard has none)
2. Check the Inputs tab live display to confirm the input state changes when you press the button
3. Ensure a function is selected in the dropdown (not "None")
4. The plugin only triggers on **rising edge** (button press, not release or hold)

### Plugin DLL locked / cannot update

Close Mach3 completely before copying a new `Tiggy.dll` to the PlugIns folder. Mach3 loads the DLL at startup and holds it open.

---

## 12. Firmware Compatibility

The plugin communicates using the Tiggy Protocol (defined in `wifi_cnc_protocol.h`). Both the firmware and plugin must use the same version of this file.

### Protocol Version

The handshake exchange includes firmware version information. The plugin logs this at connection time:

```
Handshake OK! FW v1.0.0, 6 axes, buffer=128, maxRate=200000
```

### Configuration Keys Sent to controller

On connection (and when settings change), the plugin sends:

| Config Key | Value |
|-----------|-------|
| Step pulse width | `WCNC_CFG_STEP_PULSE_US` |
| Dir setup time | `WCNC_CFG_DIR_SETUP_US` |
| Invert step/dir/limit/home/estop/probe | `WCNC_CFG_INVERT_*` |
| Per-axis max rate | `WCNC_CFG_MAX_RATE_X` + axis |
| Per-axis acceleration | `WCNC_CFG_ACCEL_X` + axis |
| Per-axis steps/unit | `WCNC_CFG_STEPS_PER_MM_X` + axis |
| Homing parameters | `WCNC_CFG_HOMING_*` |
| Spindle PWM freq/max RPM | `WCNC_CFG_SPINDLE_*` |
| Charge pump frequency | `WCNC_CFG_CHARGE_PUMP_FREQ` |
| Step idle delay | `WCNC_CFG_STEP_IDLE_DELAY_MS` |
