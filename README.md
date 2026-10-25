# fs_wheel

Barebones CNC-style control for the v2 friction surfacing rig: a wheel
mechtrode in a vertical spindle, a substrate standing in the Y-Z plane,
force applied along X.

```
X  force axis   (+X engages the wheel rim against the substrate)
Y  traverse     (deposits a line; the rim's surface speed is also along Y)
Z  stepover     (between lines; signed)
```

One line of the cycle: **engage** +X at a slow feed until the load cell
reads the target force, **traverse** Y at the feed while a P controller
on X holds the force, **retract** −X by the back-off, **step** Z, repeat.
Lines alternate direction (serpentine) or all run the same way.

Hardware: SKR 1.4 Turbo running Klipper (Moonraker on :7125), SparkFun
NAU7802 on the Pi's I2C reading a 5 kg cell mounted on the spindle,
H100 VFD over Modbus RTU on /dev/ttyUSB0 (speed only; RUN/STOP stay on
the keypad).

## Run

```
python -m venv venv && venv/bin/pip install -r requirements.txt
venv/bin/python app.py          # http://<pi>:5000
```
`fswheel.service` is a user-level systemd unit for the Pi.

## Page

- **Position** - work and machine DRO, Zero X/Y/Z (G92), Home.
- **Jog** - step jog on each axis.
- **Force** - smoothed grams, Tare, one-point Calibrate with a known mass,
  EMA alpha. No automatic tare happens anywhere.
- **Spindle** - rpm setpoint and readback from the VFD.
- **MDI** - type g-code, Enter sends it (work frame).
- **Surfacing cycle** - parameters, Cycle Start / Feed Hold / Stop /
  Engage / Retract / Reset / E-STOP. Spindle rpm is derived from wheel
  diameter and surface speed. Stop and faults both retract −X first.
  Feed Hold retracts; resume re-engages where it stopped.
- **Force loop** - kp, per-slice clamp, deadband, max force, slice time.

Every cycle writes `logs/run_<timestamp>.csv` (parameters in `#` header
lines, then ~7 Hz rows of force, position, commanded X and spindle Hz/A).
`config.json` keeps calibration, loop tuning and the last parameters.

## Files

| file | purpose |
|---|---|
| `app.py` | Flask API + serves `static/index.html` |
| `machine.py` | Moonraker client: g-code, live position, homing, e-stop |
| `loadcell.py` | NAU7802 thread, calibration, smoothing |
| `spindle.py` | H100 Modbus speed control (from v1) |
| `cycle.py` | engage / traverse / retract / step state machine, CSV log |
