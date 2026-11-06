# Installing GaugeTach in a car

Written for the first install: a 0–7000 RPM tach in a Cobra (V8, 12 V
negative ground). The same steps apply to any classic car.

**Before going to the car:** the gauge should already home, sweep, pass the
10-sweep step-loss test on the bench, and connect to the app.

---

## 1. Power

```
 12 V (switched or constant)
   │
 [1 A fuse]   ← as close to the source as practical
   │
   ├──[TVS SMBJ18A]──┐   ← clamps load dumps / alternator spikes
   │                 │
 [12 V → 5 V buck] GND
 (automotive, ≥24 V input rating)
   │
  5 V ──┬── XIAO 5V pin
        ├── TB6612FNG VM
        └── supercapacitor (+)
```

- **Ground:** one good wire from the gauge to the engine block or a main
  chassis ground. On the gauge board, bring every ground to one point: XIAO,
  TB6612, optocoupler output side, ignition divider, buck output and supercap.
- The supercapacitor keeps the gauge alive for a few seconds after the key is
  turned off, long enough to park the needle (section 3).

---

## 2. Tach signal (D10)

Where to connect:

| Ignition | Connect the tach wire to |
|---|---|
| Points, Pertronix or other coil-driving module | **Coil negative (–)** |
| MSD / capacitive discharge box | The box's **tach output** terminal. Never coil – on an MSD. |

The signal always goes through the optocoupler. Coil negative spikes to
several hundred volts, so it must **never** connect directly to the board.

```
 coil –  ──[10 kΩ 1 W]──┬──────[1 kΩ]──┬──►|── (PC817 LED, pins 1→2) ──┐
                        │              │                             │
                    [18 V zener]       └──|◄── 1N4148 (reverse) ─────┤
                    (cathode up)                                     │
                        │                                            │
 GND  ──────────────────┴────────────────────────────────────────────┘

 PC817 output:  collector (pin 4) → D10      emitter (pin 3) → GND
                (D10 has an internal pull-up; add 4.7–10 kΩ to 3.3 V for extra margin)
```

- Use a **shielded or twisted pair** for the tach wire, and route it **away
  from the plug leads and the coil's high-tension lead**.
- Ground the shield at the gauge end only.

---

## 3. Ignition sense (D9, optional)

Lets the needle drop to 0 straight away when the key is turned off.

```
 12 V ignition (key on) ──[2.2 kΩ]──┬── D9
                                    │
                                 [560 Ω] ‖ [100 nF]
                                    │
 GND ───────────────────────────────┘
```

- Use a feed that is only live with the key on. If possible, choose one that
  **stays live in START**.
- D9 sees about 2.4–2.9 V with the key on. Battery sag while cranking can make
  it read "off", but the gauge only parks once sparks have also stopped, so it
  can't park with the engine running.
- In the app, turn on **Setup → Ignition → Park needle when ignition turns off**.

---

## 4. Mounting

- **Strain-relieve the wires** where they reach the board. Vibration breaks
  wires at the solder joint, not on the board.
- Keep it **out of direct sun** behind the glass, and away from the heater core.
- Fasten the supercapacitor and the buck converter down; don't let them hang
  on their leads.
- Keep the USB port reachable if you can, for recovery flashing.

---

## 5. First power-up (key on, engine off)

1. The gauge powers up, homes against the stop, and does its wake sweep.
2. The app connects from the driver's seat.
3. **Setup → Cylinders = 8** (4 pulses per revolution).
4. **Setup → Ignition** reads **ON** (if ignition sense is wired and enabled).

---

## 6. First start

1. **Idle:** the needle should sit steady at idle speed. If it's jumpy, raise
   **Setup → Smoothing** to about 100–150 ms.
2. **Accuracy:** compare with a known reference (timing light with a tach,
   handheld inductive tach). Exactly double or half means the cylinders /
   pulses-per-rev setting is wrong.
3. **Blip the throttle:** the needle should follow smoothly, without lag or
   overshoot. Adjust **Needle → Top speed / Acceleration** if needed, and
   re-run the 10-sweep test after any change.
4. **Key off:** the needle runs to 0 within a couple of seconds and the coils
   go quiet.
5. Tap **Save to gauge** once it's all right.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Needle twitches, or reads high at idle | Ignition noise on the tach wire: re-route it away from the HT leads, check the ground, use shielded wire. Lower **Ignore pulses above** (e.g. 8000). |
| Reads exactly 2× or ½× | Wrong cylinder / pulses-per-rev setting. |
| No reading at all | Tach wire on the wrong terminal (MSD: use the tach output), optocoupler polarity, or the 1N4148 fitted the wrong way round. |
| Needle doesn't return exactly to 0 | Steps lost: lower Top speed / Acceleration, then re-run the 10-sweep test. |
| Needle parks while driving | Ignition sense on a feed that drops out; move it to a solid ignition feed. |
| Needle doesn't park at key-off | Check the switch in the app, and that D9 reads OFF with the key off (Setup → Ignition). |
| Gauge resets when the starter engages | Voltage dip: check the buck's minimum input voltage and the supercap. |
