# Tiggy DevKitC carrier (revision A)

A carrier board for an **ESP32-S3-DevKitC-1** running the Tiggy firmware (environment `esp32s3-devkitc`, octal-memory modules such as N16R8) with the terminal layout set to **four axes**. KiCad 10 project, two layers, 145 x 95 mm.

![The board, component side](tiggy-devkitc-carrier-top.png)

> **Status: not yet built.** The schematic passes KiCad's electrical rule check and the routed board its design rule check (no violations, nothing unconnected, board and schematic in agreement), but no board has been made or powered. Check it against your own parts before ordering - above all the Ethernet module's pin order (below) and the relays.

## What is on it

| | |
|---|---|
| Controller | ESP32-S3-DevKitC-1 on two 22-pin sockets. A second row of holes takes the 25.4 mm wide boards sold under the same name. |
| Supply | 12 to 24 V DC (28 V is the regulator's absolute maximum), fused (resettable) and protected against reverse connection. A Recom R-78E5.0-1.0 makes 5 V; the DevKitC is fed through a diode, so its USB port never supplies the relays and the two supplies cannot fight. |
| Drives | X, Y, Z, A: step, direction and enable as 5 V push-pull outputs (74AHCT541 / 74AHCT125), one four-way plug per drive: STEP, DIR, EN, 0 V. |
| Inputs | Seven opto-isolated inputs with an LED each: limit/home X, Y, Z, A, auxiliary input 1, E-stop, probe. A switch or an NPN sensor between the terminal and 0 V operates the input; the supply voltage is brought out for sensors. |
| Spindle sensor | Its own six-way plug: 5 V, 0 V, A, B, Z. Encoder with or without index, or a single sensor on Z. 5 V pull-ups (open-collector sensors connect directly), Schmitt triggers, good for the 250 000 lines a second the firmware reads. Without a sensor the three are auxiliary inputs 2 to 4. |
| Outputs | Three relays (Omron G5LE-1, 5 V coil) on auxiliary outputs 1 to 3, each switched by a MOSFET so the coil gets its full 5 V; changeover contacts on three-way plugs, an LED each. |
| Spindle speed | The spindle PWM as 0 to 10 V for a frequency converter (needs 12 V or more at the supply terminal), and as a 5 V PWM signal. The charge pump output is on the same plug. |
| Ethernet | Socket for a **USR-ES1** W5500 module (the "W5500 Lite", WIZ820io pin order). |

Plugs are Phoenix MC 1,5/..-G-3,81 (signals) and MSTBA 2,5/..-G-5,08 (supply, relays), or the many compatible "15EDG" parts.

## Which pin does what

| DevKitC pin | Function | | DevKitC pin | Function |
|---|---|---|---|---|
| GPIO 1 / 3 | X step / direction | | GPIO 5, 6, 7, 15 | Limit/home X, Y, Z, A |
| GPIO 2 / 4 | Y step / direction | | GPIO 18 | Auxiliary input 1 |
| GPIO 42 / 38 | Z step / direction | | GPIO 13 | E-stop |
| GPIO 41 / 10 | A step / direction | | GPIO 14 | Probe |
| GPIO 8 | Drive enable | | GPIO 16, 17 | Encoder A, B (or auxiliary inputs 3, 4) |
| GPIO 9 | Spindle PWM | | GPIO 21 | Index / single sensor (or auxiliary input 2) |
| GPIO 39 | Charge pump | | GPIO 11, 12, 40 | Relays 1, 2, 3 (auxiliary outputs 1 to 3) |
| GPIO 45, 46, 0, 47 | Ethernet MOSI, MISO, clock, interrupt | | | |

This is the firmware's own map (`firmware/main/pin_config.h`, `pin_layout.c`) for the **four axes** terminal layout, so no pin numbers have to be entered - but that layout has to be chosen once, because a new controller starts with six axes: in the Mach3 plugin on the Terminals tab, otherwise with the protocol tester (`tools/protocol_test_gui.py`). The spindle sensor is chosen in the same place. Until it is, GPIO 11, 12, 39 and 40 are the B and C axes' outputs, so set the layout before wiring the relays to anything.

## Three things done on purpose

- **The Ethernet module is selected only after the ESP32 has left reset** (Q1, some 20 to 80 ms after the reset line goes high - long after the boot options are read, long before the firmware first talks to the module). The firmware has no pin for the module's chip select and expects it tied to 0 V. Tied there directly, the module drives its data output onto GPIO 46 all the time, and GPIO 46 is one of the pins the ESP32 reads as a boot option: left high it can stop the board entering download mode through the UART port. Solder jumper JP1 ties chip select to 0 V directly if you prefer the plain arrangement. The module's reset follows the ESP32's.
- **GPIO 0 has a pull-up** (4.7 k). It is the Ethernet clock, and the DevKitC has no pull-up of its own on it - only the one inside the chip - while it must be high at reset.
- **The outputs to the drives are held low while the ESP32 is in reset** (10 k on every buffer input), so no stray steps at power-on.

## Limits of this revision

- Relay contacts are laid out for **low voltage only** (tracks 1.2 mm, 1 mm clearance). Switch mains with an external relay or contactor.
- The spindle sensor inputs need a **74LVC14** (U4). A 74HC14 fits the footprint but is not 5 V tolerant on a 3.3 V supply.
- A short on the 5 V sensor supply resets the board (the regulator limits the current); there is no separate fuse for it.
- With USB alone the ESP32 runs, but the drive outputs, relays and inputs need the 12-24 V supply.

## How the project is made

Everything is generated from [`gen/design.py`](gen/design.py), which holds the circuit (every part, every net) and where each part sits:

```
python gen/build.py --tracklayer path/to/Tracklayer
```

writes the project's own library (the two module sockets), the schematic, runs KiCad's electrical rule check, exports the netlist, places the parts on the board from that netlist, routes it with [Tracklayer](https://github.com/MichaelGaylor/Tracklayer), fills the copper pours and runs KiCad's design rule check with the schematic compared against the board. Reports are written to `build/`. It needs KiCad 10 (`kicad-cli` and the Python that comes with it; where KiCad is not in `C:\Program Files\KiCad\10.0`, set `KICAD_BIN` and `KICAD_SHARE` to its `bin` and `share/kicad` directories) and takes about a quarter of an hour, nearly all of it routing. `--keep-routing` reuses the last routing when only text or values have changed.

Because the schematic is generated, its connections are by net name on each pin rather than drawn wires, block by block. Hand edits to the `.kicad_sch` or `.kicad_pcb` are lost at the next build: change `design.py`, or stop using the generator and carry on in KiCad.

Dimensions used for the sockets: Espressif's drawing of the ESP32-S3-DevKitC-1 v1.1 (25.40 x 62.74 mm, rows 22.86 mm apart, last pin 8.00 mm from the USB end) and USR IOT's data sheet of the USR-ES1 (two rows of six pins 20.32 mm apart; left row seen from above with the socket away from you: GND, GND, MOSI, SCLK, SCS, INT; right row: GND, 3.3 V, 3.3 V, NC, RST, MISO).
