# Tiggy Motion Controller for Mach3 - User Manual

Plugin 2.0 · controller firmware 1.2 or newer

## 1. Overview

The Tiggy Motion Controller plugin lets Mach3 drive a Tiggy controller over WiFi or wired Ethernet in place of a parallel port. The controller generates the step pulses, reads the switches and drives the outputs; Mach3 does everything else exactly as it always has.

**You set the machine up in Mach3's own menus** - Motor Tuning, Ports and Pins, Homing/Limits - the same way as with any other motion device. The plugin reads those settings and passes them to the controller, and follows at once when you change one.

**What you get**

- Up to 6 axes (X, Y, Z, A, B, C), slaved axes for gantries
- Smooth, evenly spaced step pulses up to about 100,000 steps per second per axis
- Motion that survives a poor WiFi connection: lost data is repeated automatically and can never lose or repeat a step
- Home and limit switches, probe (G31), E-stop input, four auxiliary inputs
- Spindle on/off or PWM speed control, charge pump, up to five auxiliary outputs
- Screw cutting on lathes (G32 and G76 in Mach3 Turn) with a spindle encoder or a one-pulse-per-revolution sensor
- Optional I/O expansion module with 16 more inputs and outputs
- The controller is found on the network automatically
- Safe behaviour when something goes wrong: if the PC stops talking to it the controller stops the machine by itself within a second and switches its outputs off

**Controller boards**

| Board | Module | Axes | Aux inputs | Aux outputs |
|-------|--------|------|------------|-------------|
| Tiggy Standard | Waveshare ESP32-S3-Zero | 3 (X/Y/Z) | - | 2 + charge pump |
| Tiggy Pro (quad memory) | ESP32-S3-DevKitC-1 N8 / N8R2 | 6 | 3 | 2 + charge pump |
| Tiggy Pro (octal memory) | ESP32-S3-DevKitC-1 N16R8 / N8R8 | 6 | 2 | - |
| Classic | ESP32-WROOM-32 DevKit | 6 | - | - |

**Licence.** The plugin is free for three axes (X, Y, Z) with every other feature. A Pro Mach3 licence adds the A, B and C axes, slaved axes, the I/O expansion module and screw cutting. The controller firmware itself is open source and always has all six axes: the limit applies only to the Mach3 plugin, not to LinuxCNC or G-code senders. Licences: [www.tiggyengineering.com](https://www.tiggyengineering.com).

---

## 2. What you need

- Mach3 R3.042 or R3.043 on Windows
- A Tiggy controller with firmware **1.2.0 or newer**, on the same network as the PC
- `Tiggy.dll`, and the `M62`-`M65` macros if you want to switch outputs from G-code

Plugin 2.0 does not work with firmware older than 1.2 (and tells you so on its Status tab). Update the controller first: see the firmware page of the project.

---

## 3. Installation

### 3.1 The controller

A new controller has no network set up, so it opens its own:

1. Power the controller. It creates a WiFi network called **TiggyCNC-xxxx** (password `12345678`).
2. Join that network with a phone or the PC. A set-up page opens (or browse to `192.168.4.1`).
3. Choose your own WiFi network and enter its password. The controller restarts and joins it.

If the controller cannot join your network when it is switched on - the router is still starting, say - it opens the TiggyCNC-xxxx network again so that you can reach it, keeps trying your network in the background, and closes its own network once it is connected. You never need to power-cycle it to get it back on line.

**Wired Ethernet.** With a W5500 Ethernet module fitted and a cable plugged in, the controller uses Ethernet instead of WiFi with no settings to change (see section 8).

### 3.2 The plugin

1. Close Mach3.
2. Copy `Tiggy.dll` to `C:\Mach3\PlugIns\`.
3. Copy `M62.m1s`, `M63.m1s`, `M64.m1s` and `M65.m1s` to your profile's macro folder, for example `C:\Mach3\macros\Mach3Mill\`.
4. Start Mach3. It asks which motion device to use: choose **Tiggy Motion Controller** (tick "Don't ask me this again").
   If Mach3 does not ask, use **Function Cfg's > Reset Device Sel...**, restart Mach3 and choose it then.

The plugin now looks for the controller by itself. Open **PlugIn Control > Tiggy Motion Controller...** : the Status tab shows "Connected to ..." in green when it has found it. Nothing has to be typed in.

The first time it runs with a profile, the plugin fills in the Port and Pin numbers of the controller's inputs and outputs in Ports and Pins (section 4.3) and saves them, so most of the set-up is already done.

---

## 4. Setting up the machine

Everything in this section is done in Mach3's **Config** menu. The Tiggy window (section 5) has buttons that open the right dialogs, shows the settings as the controller is using them, and has the same steps on its "Setting up" tab.

### 4.1 Axes

**Config > Ports and Pins > Motor Outputs**

- Tick **Enabled** for each axis you use.
- The Step Pin, Dir Pin and Port numbers in this table are not used: each motor driver is wired to its own terminals on the controller (shown in the "Step, Dir" column of the Tiggy window's Machine tab).
- Tick **Dir LowActive** if an axis moves the wrong way. **Step Low Active** selects step pulses that are low instead of high.

**Config > Motor Tuning**

For each axis enter **Steps per**, **Velocity** and **Acceleration**, then press **SAVE AXIS SETTINGS** before selecting the next axis.

The highest velocity Mach3 accepts is the controller's fastest step rate (section 9). If the Velocity box will not go as high as you expect, that is the limit.

**Step Pulse** and **Dir Pulse**, in the same dialog, are the width of the step pulse and the time the direction signal is set before a step, in microseconds. They are one setting for all axes (press SAVE AXIS SETTINGS after changing them). Most stepper drivers need 2 to 5; check the driver's data sheet. The plugin enters 5 for both the first time it runs, and the controller accepts 1 to 50 whatever range the dialog shows next to the boxes. A longer step pulse lowers the fastest step rate (section 9).

**Slaved axes** (Pro): Config > Slave Axis, as usual. The slave follows its master step for step.

### 4.2 Units

Config > Select Native Units, as usual. Motor tuning is in those units.

### 4.3 Switches, probe and E-stop

**Config > Ports and Pins > Input Signals.** The controller's inputs appear to Mach3 as **Port 1**:

| Port 1, Pin | Controller input |
|-------------|------------------|
| 1 - 6 | X, Y, Z, A, B, C limit / home switch |
| 7 | Probe |
| 8 | E-stop |
| 9 - 12 | Auxiliary input 1 - 4 |
| 13 | Spindle index |

These numbers are already filled in. What you do:

- Tick **Enabled** for the signals you have wired: `X Home`, `X ++`, `X --`, `Probe`, `EStop` and so on.
- One switch per axis can serve as home switch and as limit switch: `X ++`, `X --` and `X Home` all have Pin 1.
- **Active Low** is for a switch that connects its input to ground when it operates. This is the usual wiring and how the inputs are set to begin with. Untick it for a switch that *opens* when it operates - for example an E-stop button wired normally-closed, which is the safer way to wire one.
- An input with nothing connected reads "not operated" when it is set Active Low.

The **Inputs & Outputs** tab of the Tiggy window shows every input live. Operate each switch by hand and watch it change before you rely on it.

**Electrical noise.** The controller ignores any change on an input that lasts less than 5 ms. If a switch input still trips by itself, raise **Debounce Interval** in Config > General Config. Mach3 counts it in steps of 40 microseconds, so 500 means 20 ms; the controller uses at least 5 ms and at most 100 ms. Homing and probing are not slowed down by it: they act on the first edge of the switch.

**Emulated** and the HotKey column work as in any Mach3 set-up.

### 4.4 Homing

**Config > Homing/Limits**

- **Home Neg**: ticked = the axis homes in the negative direction.
- **Speed %**: the search speed, as a percentage of the axis' tuned velocity.
- **Home Off.** and **Auto Zero**: as in any Mach3 set-up.
- **Soft Max / Soft Min / Slow Zone**: soft limits, as usual.
- **Reversed**: swaps the direction of the axis.

In the Tiggy window, on the **Controller** tab: the distance each axis backs off its switch after finding it, and the speed of the slow final approach.

How an axis homes: fast towards the switch; back off; slowly towards the switch again, stopping within one step of the point where it operates; back off by the set distance. That last point is the home position. The controller does this by itself, so the accuracy does not depend on the network.

An axis with no home switch enabled is simply set as referenced where it stands when you press its Ref button.

### 4.5 Limit switches

With `X ++` / `X --` (and so on) enabled, the controller itself stops everything the moment a limit switch operates while the machine is moving, and Mach3 goes into Reset with the reason shown.

To get off the switch: press Reset, switch on **OverRide Limits** (or use Auto LimitOverRide), jog the axis off the switch, switch the override off again.

### 4.6 Spindle

**Config > Ports and Pins > Spindle Setup**

- M3 and M4 switch the controller's spindle output on, M5 switches it off. There is nothing to set for that.
- **Use Spindle Motor Output** with **PWM Control** ticked: the spindle output becomes a PWM speed signal. **PWMBase Freq.** sets its frequency and **Minimum PWM** the lowest duty, in per cent, it gives while the spindle runs. **Config > Spindle Pulleys** sets the speed that corresponds to 100 %.
- **Flood Mist Control**: choose the Output # signals for M7 and M8 and give those signals a controller output (below).
- With a spindle sensor fitted (section 4.10), Mach3's true spindle speed display shows the measured speed.

### 4.7 Outputs

**Config > Ports and Pins > Output Signals**

| Port, Pin | Controller output |
|-----------|-------------------|
| Port 1, Pin 1 - 5 | Auxiliary output 1 - 5 |

`Output #1` to `Output #5` are set to these to begin with. How many of them exist depends on the board (see the table in section 1); the Inputs & Outputs tab shows which terminals are fitted.

- **Charge Pump**: tick Enabled and the controller's charge pump output runs while Mach3 is out of Reset (always, with "Charge Pump On in EStop"). Its frequency is on the Controller tab of the Tiggy window. Use it with a safety board that cuts the power when the signal stops.
- **Enable1**: ticking **Active Low** inverts the controller's motor driver enable output.

**Switching outputs from G-code.** With the supplied macros:

| M-code | Effect |
|--------|--------|
| `M64 P0` ... `M64 P4` | Output #1 ... #5 on, at once |
| `M65 P0` ... `M65 P4` | Output #1 ... #5 off, at once |
| `M62 Pn` / `M63 Pn` | The same, but after the moves before it have finished |

All outputs are switched off by the controller when Mach3 goes into Reset, on E-stop, and if contact with the PC is lost.

### 4.8 General Config

Everything in General Config works as usual. Two things to know:

- Tick **G04 Dwell in ms** only if your programs give dwell times in milliseconds. Fractions of a second (G4 P0.5) work either way.
- **Debounce Interval** sets how long a switch input must be steady (section 4.3).

### 4.9 Backlash

**Config > Backlash** has no effect: backlash compensation was a function of Mach3's parallel port driver and the controller does not do it. The Machine tab of the Tiggy window tells you if it is switched on.

---

### 4.10 Four axes and more inputs and outputs; spindle sensor and screw cutting

**The Terminals tab of the Tiggy window** holds two settings of the controller itself. They say how it is wired, so they are kept in the controller and are the same for every Mach3 profile that uses it.

**What the axis terminals are used for.** Six axes use nearly every terminal of a Tiggy Pro board. Choose **Four axes** and the controller keeps X, Y, Z and A - and Ethernet - and turns the terminals of B and C into inputs and outputs: a spindle sensor, a charge pump, auxiliary inputs and outputs. The same set-up suits a lathe and a mill: the spindle sensor takes only the inputs its kind needs, and a terminal it does not need is an auxiliary input. The Inputs & Outputs tab always shows what each terminal does at the moment.

With four axes ("or" = what the terminal is when the sensor does not use it):

| | Tiggy Pro (octal memory) | Tiggy Pro (quad memory) |
|---|---|---|
| Encoder A | GPIO 16, or auxiliary input 3 | GPIO 16, or auxiliary input 4 |
| Encoder B | GPIO 17, or auxiliary input 4 | GPIO 17 |
| Index pulse / single sensor | GPIO 21, or auxiliary input 2 | GPIO 37 |
| Other auxiliary inputs | 1: GPIO 18 | 1 to 3: GPIO 18, 21, 36 |
| Auxiliary outputs | 1 to 3: GPIO 11, 12, 40 | 1 to 5: GPIO 11, 12, 40, 39, 34 |
| Charge pump | GPIO 39 | GPIO 10 |

No terminal changes between input and output with the sensor setting: the sensor only ever shares a terminal with an auxiliary input. A single sensor has its terminal even with six axes on both Pro boards (GPIO 21 and GPIO 37). The Tiggy Standard board has three motor outputs and its sensor terminals already, so the choice makes no difference to it: encoder A, B and index are GPIO 15, 16 and 17, and those the sensor does not use are auxiliary inputs 1 to 3. The Classic board has no terminals for a sensor.

**The sensor on the spindle** (needed only for screw cutting, which is part of Tiggy Pro).

- *Encoder with A and B channels*, or *with A, B and an index pulse*: enter the number of lines marked on it (50 to 10000; 100 or more is recommended). Without an index pulse, the position the spindle had when the controller was switched on serves instead. An encoder is the better sensor.
- *One sensor: a pulse per revolution* (a slotted disc or a magnet, as used with Mach3's parallel port): it goes on the index terminal. It follows a steady spindle exactly, but cannot see a change of speed until the turn is over, so while the speed is changing - as the tool enters the cut - the thread is out by about what the speed changed by in one turn. A sensor that gives several pulses per revolution can be entered too, but only for the speed display: nothing tells its pulses apart, so no thread can be started by them.
- *None*: the terminals are auxiliary inputs.

Use 3.3 V signals. The inputs have weak pull-ups: a single sensor that switches its output to ground (open collector, NPN) connects directly. An open-collector encoder needs pull-up resistors of its own - 2.2 to 4.7 kilohms from each output to 3.3 V - or the controller cannot follow it at speed; an encoder with push-pull outputs needs nothing. The controller reads encoders up to about 250 000 lines a second (2500 lines at 6000 rpm).

Changing either setting restarts the controller, which takes a few seconds; Mach3 goes into Reset meanwhile and the axes must be referenced again. Some terminals then do a different job, so check the Inputs & Outputs tab against your wiring before you power the machine. With a sensor, the Terminals tab shows it working and the spindle speed.

**Cutting a thread.** Use **G32** and **G76** in Mach3 Turn as described in the Mach3 Turn manual. There is nothing to set for it in Mach3. The controller itself keeps the tool in step with the spindle: the pass begins as the spindle passes its index, and from there the tool advances by the angle the spindle really turns. So the pitch holds if the spindle slows in the cut, every pass follows the last, and if the spindle stalls the tool stops with it.

- Have the spindle running at a steady speed before the thread starts (Mach3 plans the thread for the speed the spindle has at that moment), and allow a little distance before the work for the tool to get up to speed.
- The **feed override must be at 100 %**. At any other setting Mach3 would scale the pitch, so the thread is not started: Mach3 goes into Reset saying so, with the override put back to 100 %.
- If the spindle is not turning, or the controller has no spindle sensor, the thread is likewise not started. If the spindle stops after that, the controller waits for it, and the Tiggy note says what it is waiting for.
- A pass is cut in step with the spindle or not at all. If this PC leaves the controller without motion in the middle of a pass (Windows or the network stalling for longer than the quarter second the controller has in hand), or the spindle runs far faster than the pass was planned for, the controller brings the axes to rest and abandons the pass; Mach3 is put into Reset and the Tiggy note says why. The thread already cut is not cut across - but check it, and the tool, before going on.
- **Feed hold** in the middle of a pass stops it; Mach3 does not continue a threading pass, so run the thread again. **Stop** and **Reset** work as always.
- While it supplies the spindle speed, the plugin switches Mach3's *Use Spindle Feedback in Sync Modes* on and *Spindle Speed Averaging* off (Ports and Pins > Spindle Setup) - threads come out wrong otherwise - and puts both back as you had them when Mach3 closes.
- The true spindle speed display is steadied: while the machine moves it follows only a real change of speed, and during a thread it stays at the speed the thread was planned for.

## 5. The Tiggy window

**PlugIn Control > Tiggy Motion Controller...** (it can stay open while you work).

| Tab | What it is for |
|-----|----------------|
| **Status** | Whether the controller is connected, what it is doing, the quality of the network link, and - in red - the reason Mach3 last went into Reset by itself. The controller's address is here too; normally leave "Find the controller on the network automatically" ticked. |
| **Machine** | The settings Mach3 has, as the controller is using them: each axis with the terminals its motor is wired to, the step pulse timing, the input debounce time and the minimum PWM. Buttons open Motor Tuning, Ports and Pins, Homing/Limits, Spindle Pulleys and General Config. A note tells you if an axis still needs tuning. |
| **Inputs & Outputs** | Every controller input and output: its pin number for Ports and Pins, its terminal, its state right now, and which Mach3 signals use it. "Restore the standard pin numbers" puts the numbers of section 4.3 and 4.7 back. |
| **Controller** | The few settings Mach3 has no place for: when to release the motor drivers, charge pump frequency, homing back-off distances, diagnostics log. |
| **Terminals** | Six axes, or four with more inputs and outputs, and the sensor on the spindle (section 4.10).  These two are kept in the controller, not in the Mach3 profile. |
| **I/O Module** | The optional I/O expansion module (section 7). |
| **Licence** | Your e-mail address and Pro licence key. |
| **Setting up** | The steps of section 4 in brief. |

Settings on these tabs take effect when you press **Apply**. They are kept per Mach3 profile.

A small yellow note appears at the bottom of the Mach3 screen whenever the plugin has put Mach3 into Reset, saying why.

---

## 6. Running the machine

Use Mach3 as you always do: MDI, programs, jogging, feed hold, feed rate override, Stop, Reset.

- **Jogging** is ramped by the controller at the axis' acceleration, at Mach3's Slow Jog Rate (Shift for full speed). If the PC stops asking for a jog - Mach3 freezes, the network drops - the axis ramps to a stop by itself within half a second.
- **Feed hold** slows to a stop along the path; **Cycle Start** continues.
- **Stop** brings the machine to a quick, controlled stop. As with any Mach3 set-up, use Feed Hold first if you want to carry on afterwards.
- **Reset / E-stop** stops at once and switches the outputs off. While the E-stop input is active the controller cannot be taken out of Reset.
- **Probing**: G31 stops within one step of contact; the contact position is in #2000 - #2005 as usual. If the probe input is already active when G31 starts, the move is not made and Mach3 goes into Reset saying so.
- **Referencing**: Ref All Home or the individual Ref buttons (section 4.4).
- **Soft limits** work as usual, for programs and for jogging.

**If Mach3 goes into Reset by itself**, the reason is on the Status tab and in the yellow note: a limit switch, the E-stop input, homing that did not find its switch, or the connection to the controller.

**After the controller has been switched off or restarted**, the motors have been without power, so the axes are no longer referenced: reference the machine again.

---

## 7. I/O expansion module (Pro)

A second Tiggy board set up as an I/O module adds 16 inputs and outputs for buttons, a pendant, relays or indicator lamps.

1. In the Tiggy window, **I/O Module** tab: tick "Use an I/O expansion module", enter the module's address, press Apply. The tab shows when it is connected and the state of all its channels.
2. The module appears to Mach3 as **Port 2, Pins 1 to 16**. Give any input or output signal Port 2 and a pin number in Ports and Pins.

Examples:

- A Cycle Start button: `OEM Trig #1` on Port 2 Pin 1, then Config > System Hotkeys > OEM code for trigger 1 = `1000`.
- A jog pendant: `Jog X++` on Port 2 Pin 3, `Jog X--` on Pin 4, and so on.
- A relay: `Output #6` on Port 2 Pin 1.

Tick Active Low for an input that is connected to ground when its button is pressed.

---

## 8. Network

- The PC and the controller must be on the same network. The plugin finds the controller by itself; if you have more than one, or the network blocks broadcasts, untick the automatic search on the Status tab and enter the address.
- Give the controller a fixed address in your router (a "DHCP reservation") so that it is found at once every time.
- **WiFi quality** matters less than you might expect: motion is buffered about a quarter of a second ahead and anything lost is sent again. The Status tab shows how much had to be repeated; a few per cent is harmless. If the link stalls for longer than the buffer, the controller slows the machine to a smooth stop along the programmed path and carries on when the data arrives - no steps are lost, but the pause may mark the work, so a reasonable signal is still worth having.
- **If the connection is lost completely**, the controller stops the machine by itself within a second, switches its outputs off, and Mach3 goes into Reset. The plugin reconnects automatically; you cannot leave Reset until it has.
- Only one PC at a time can control a controller. While it is in use, a second connection is refused.

**Wired Ethernet.** For a machine in an electrically noisy workshop a cable is the robust choice. The Tiggy Pro board accepts a W5500 SPI Ethernet module:

| W5500 pin | Controller |
|-----------|-----------|
| 3.3V, GND | 3V3, GND |
| MOSI | GPIO 45 |
| MISO | GPIO 46 |
| SCLK | GPIO 0 |
| INT | GPIO 47 |
| CS | GND |
| RST | 3.3V |

With the module fitted and a cable plugged in at power-on, the controller uses Ethernet.

**Ports**, should a firewall need them: TCP 58429 and UDP 58427, both from the PC to the controller. Nothing has to be opened for incoming traffic on the PC.

**USB.** The controller's USB port is a GRBL-compatible G-code interface for senders such as UGS or CNCjs. Mach3 does not use it.

---

## 9. Limits

**Step rate.** The fastest step rate of an axis depends on the step pulse width (Step Pulse in Config > Motor Tuning):

| Step pulse width | Fastest step rate per axis |
|------------------|----------------------------|
| 2 µs | 100,000 steps/s |
| 3 µs | 76,000 steps/s |
| 5 µs (default) | 52,000 steps/s |
| 10 µs | 29,000 steps/s |

Mach3's Motor Tuning will not accept a velocity above this. When several axes run close to the limit at the same time, the controller plays the move slightly slower rather than produce pulses its timing cannot guarantee; no steps are lost. If you need more speed, use fewer microsteps on the driver.

**Screw cutting.** Reference spindle speeds from 4 rpm. The spindle may run up to half as fast again as it did when the thread was planned and the tool still follows; beyond that it falls behind and catches up when it can. Rigid tapping and feed per revolution (G95) are not synchronised to the spindle.

**Not supported:** backlash compensation (Config > Backlash), and the parallel-port-specific options (Sherline mode, kernel speed and so on, which have no meaning here).


---

## 10. Troubleshooting

**The Status tab says "Controller not found".** Is the controller powered and on the same network as the PC? Look for its TiggyCNC-xxxx network: if that is visible, the controller could not join your WiFi (section 3.1). On a network that blocks broadcasts, enter the controller's address by hand.

**"Controller ... has firmware 1.1.x - update it".** Plugin 2.0 needs firmware 1.2.0 or newer.

**"Mach3 is not using this plugin to drive the machine".** Function Cfg's > Reset Device Sel..., restart Mach3 and choose Tiggy Motion Controller.

**Mach3 will not come out of Reset: "The E-stop input is active".** The controller sees its E-stop input as operated. With an E-stop button wired normally-closed, untick Active Low for `EStop`; with nothing wired, tick it (section 4.3). Watch the E-stop row on the Inputs & Outputs tab while you press the button.

**An axis does not move.** Machine tab: is it enabled, and does its Status say "ready"? "not tuned" means Motor Tuning has no values for it; "needs Pro" means A, B or C without a Pro licence.

**An axis moves the wrong way.** Ports and Pins > Motor Outputs > Dir LowActive, or Homing/Limits > Reversed.

**Homing: "switch not found".** The axis travelled for ten minutes without reaching its home switch, or the switch did not release when the axis backed off. Check the direction (Home Neg), that `X Home` has the right pin, and the switch itself on the Inputs & Outputs tab.

**Mach3 goes into Reset as soon as an axis moves: "Limit switch on input n tripped".** That switch reads as operated. Check its Active Low setting and its wiring on the Inputs & Outputs tab.

**Motion stutters.** Look at the Network line on the Status tab. A high percentage of repeated data means a weak or crowded WiFi link: move the access point, change channel, or use Ethernet.

**Asking for help.** Tick "Write detailed diagnostics to the log file" on the Controller tab, reproduce the problem, press "Show the log", and send that file.

---

## 11. Coming from plugin 1.x

Plugin 1.x kept the whole machine set-up in its own dialog. Plugin 2.0 uses Mach3's own settings instead.

- The first time 2.0 runs it copies your steps per unit, velocity and acceleration into Mach3's Motor Tuning for every axis that has no tuning in Mach3 yet, together with the controller address and licence. Your step pulse width and direction set-up time go into Step Pulse and Dir Pulse in the same dialog. Check them in Config > Motor Tuning.

- Input polarity, homing direction and speed, spindle and charge pump settings are now Mach3's own (section 4): go through section 4.3 to 4.7 once.
- The functions that 1.x assigned to buttons in its own dialog are done with Mach3's OEM triggers (section 7).
- The pin map files are no longer needed: the plugin reads the terminal numbers from the controller.
- The controller must be updated to firmware 1.2.0 or newer.
