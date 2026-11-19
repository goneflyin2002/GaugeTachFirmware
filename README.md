# GaugeTach Firmware

Stepper-driven needle gauge for classic cars, set up from a phone over
Bluetooth. The first gauge is a 0–7000 RPM tachometer with a 345° sweep for a
Cobra; the same firmware and app are meant to grow into a universal gauge kit.

- **App:** [gauge_app](https://github.com/goneflyin2002/gauge_app) (Flutter, Android)
- **Current version:** 0.7.1
- **Installing in a car:** see [INSTALL.md](INSTALL.md)

---

## Hardware

| Part | Notes |
|---|---|
| Seeed XIAO ESP32-C5 | BLE 5, native USB |
| TB6612FNG dual H-bridge | Drives the motor at 5 V from 3.3 V logic |
| X27-series gauge stepper | Internal stop modified for ~345° travel |
| PC817 optocoupler + parts | Tach signal from coil negative |
| Automotive 12 V → 5 V buck | Fused, TVS-protected input |

### Wiring

| TB6612FNG | Connect to |
|---|---|
| AIN1 / AIN2 | XIAO D0 / D1 |
| BIN1 / BIN2 | XIAO D6 / D7 |
| PWMA, PWMB, STBY, VCC | 3.3 V |
| VM | 5 V |
| GND | GND (common with the XIAO) |
| AO1 / AO2 | X27 pin 1 / pin 2 |
| BO1 / BO2 | X27 pin 3 / pin 4 |

**Tach input (D10):** coil negative → 10 kΩ 1 W → 18 V zener to ground → 1 kΩ →
PC817 LED, with a 1N4148 reverse-parallel across the LED. The PC817 output
pulls D10 low on each spark. At D10: a **4.7 kΩ pull-up to 3.3 V** and a
**2.2 nF capacitor to GND**, both right at the pin, with **no series
resistor** (it would stop the PC817 pulling D10 cleanly low). The capacitor
stops edge ringing and interference from being counted as pulses.
Never connect coil negative directly to the board.

The firmware adds more layers on top: a check that each edge settles LOW
(sampled every 20 µs for up to 100 µs), a max-RPM lockout, rejection of
intervals far shorter than the recent typical one, treating a ~2× interval as
a missed pulse, and a reading that ignores the 2 highest and 2 lowest of the
last 8 intervals. The ESP32-C5's hardware glitch filter is available but off
(it dropped real edges from the optocoupler in 0.7.0). Rejected
pulses are counted (`tach`, and in the app's Live tab) so you can see whether
noise is getting in.

**Ignition sense (D9, optional):** 12 V ignition (switched) → 2.2 kΩ → D9,
with 560 Ω from D9 to ground (about 2.4–2.9 V at D9 with the key on). A
100 nF capacitor across the 560 Ω filters noise. With "Park needle when
ignition turns off" enabled in the app, turning the key off drives the needle
to 0 and switches the coils off while the supercapacitor keeps the gauge
alive. It only parks once sparks have stopped too, so battery sag while
cranking can't trigger it. Use an ignition feed that stays on in START if you
can.

To drive the X27 straight from the GPIO pins instead (3.3 V, less torque),
set `USE_TB6612 = false` at the top of the sketch.

---

## Building (Arduino IDE)

- **Board:** Tools → Board → esp32 → XIAO_ESP32C5 (esp32 core 3.3.x)
- **USB CDC On Boot:** Enabled
- **Partition Scheme:** `8M with spiffs (3MB APP/1.5MB SPIFFS)` (the default).
  It has the two app slots that Bluetooth updates need; don't pick a "No OTA" scheme.
- No extra libraries.

### First run

1. Flash over USB. The gauge homes against its stop, then sweeps.
2. In the app (or the Serial Monitor at 115200, "Newline"), nudge the needle
   onto the 0 mark, then **Set zero** and **Save**.
3. Check for lost steps with a 10-sweep test (`sweep 10`); the needle must
   finish exactly on 0.

---

## Serial / app commands

The app sends exactly these commands, so the Serial Monitor can do everything
the app can.

| Command | Does |
|---|---|
| `show` | Print all settings |
| `mon` | Toggle live readout (includes the rejected-pulse count) |
| `tach` / `tach reset` | Tach edge count, rejects by type, layer states / clear counters |
| `tach hw\|confirm\|adaptive on\|off` | Switch a tach filter layer for testing (not saved) |
| `rpm <n>` / `live` | Simulate an input / back to the real signal |
| `sweep [n]` | Sweep 0 → full → 0, n times |
| `home` | Re-zero against the stop |
| `goto <deg>` / `jog <deg>` | Hold the needle at / nudge it by an angle |
| `setzero` | The needle is on the 0 mark now: store it |
| `motor off` / `motor on` | De-energize / re-energize the coils |
| `set <key> <value>` | `sweep max ppr maxrpm filter speed accel deadband drive invert zerooffset homespeed overtravel wake ign` |
| `cal clear` / `cal add <rpm> <deg>` | Non-linear dial calibration (up to 8 points) |
| `save` / `defaults` | Store settings in flash / load defaults (not saved) |

---

## Bluetooth protocol

Advertises as `GaugeTach-XXXX` (last 4 hex digits of the chip ID).
UUIDs are `6a1f000x-3c2d-4b8e-9f10-7e5a2c9d4b01`:

| x | Type | Contents |
|---|---|---|
| 1 | Service | GaugeTach |
| 2 | Read | JSON of all settings, plus `fw` and `rev` |
| 3 | Write | Text commands from the table above, one per line |
| 4 | Notify | Live JSON 10×/s: `rpm raw needle target homed busy hold sim unsaved rev ota otapct otaerr ignon parked rej` |
| 5 | Write | OTA control: `begin <bytes>`, `end`, `abort` |
| 6 | Write | OTA data: raw firmware chunks, in order |

---

## Releasing a firmware update (OTA)

The app checks `version.json` in this repo every time it connects to a gauge,
and offers the update if it's newer than the firmware on the gauge.

1. Bump `#define FW_VERSION` in `GaugeTachX27.ino`.
2. **Sketch → Export Compiled Binary.** The `.bin` appears in the sketch's
   `build/…` folder. Use the plain `GaugeTachX27.ino.bin`, not the
   `.bootloader.bin`, `.partitions.bin` or `.merged.bin`.
3. Rename it to `GaugeTach_vX.Y.Z.bin` and put it in `releases/`.
4. Update `version.json`: `version`, `url`, `size` (exact bytes: right-click →
   Properties → "Size", not "Size on disk"), and `notes`.
5. Commit and push. Phones see it within a minute.

This repo must be **public** so the app can download without logging in.
The gauge verifies each image before switching to it, so a bad or partial
transfer is rejected and the old firmware keeps running.

The first OTA-capable firmware (0.5.0) has to be flashed over USB once.

---

## Version history

| Version | Summary |
|---|---|
| 0.7.1 | Fixes 0.7.0 tach dropouts: hardware glitch filter off by default, edge check samples up to 100 µs (slow optocoupler edges pass), missed pulses no longer dip the needle, layer on/off switches and edges/s in `mon`. |
| 0.7.0 | Tach input noise protection: hardware glitch filter, edge confirmation, adaptive interval rejection, trimmed 2-revolution reading, noise counters. Fixes erratic readings from edge ringing at low RPM. |
| 0.6.0 | Ignition sense on D9: needle parks at 0 when the key is turned off (switch in the app). |
| 0.5.1 | OTA test release. |
| 0.5.0 | Firmware updates over Bluetooth from the app. |
| 0.4.1 | USB serial can no longer stall the gauge (fixed slow nudges). |
| 0.4.0 | Bluetooth setup service for the app. |
| 0.3.0 | TB6612FNG drive at 5 V. |
| 0.2.0 | X27 gauge stepper, sine microstepping, homing against the stop. |
| 0.1.0 | First prototype (NEMA 8 + TMC2209, hall-sensor homing). |
