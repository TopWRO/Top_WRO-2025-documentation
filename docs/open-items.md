# Open Items

[← back to README](../README.md)

Information that cannot be derived from the source code and must come from the team or from
measurements on the car. Each item shows where the value is used. Until it is filled in,
the documentation shows **TODO** rather than an estimate.

## Hardware facts (team)

| # | Item | Used in |
|---|---|---|
| H1 | Chassis construction (LEGO Technic / 3D-printed / mixed) and CAD files | README §4.3, `models/` |
| H2 | Length, width, height, wheelbase, track, wheel diameter, mass | README §4.1–4.3, FoV diagram |
| H3 | How the 1 : 3 gear-up is built; differential yes/no | README §4.1, drivetrain photo |
| H4 | Steering linkage type (Ackermann / parallel) and real wheel angle at ±1 | README §4.2 |
| H5 | Battery (chemistry, cells, capacity), regulators, switch, fuse; how the XL330s and the Pi 5 are powered | README §5.5, topology diagram |
| H6 | Camera model, mounting height and tilt | README §5.2, BOM |
| H7 | Exact mounting position and orientation of each ToF and the camera | `sensor_layout_fov` diagram |
| H8 | Sensor supply wiring (3V3 / 5 V / GND) on the Pico | `pico_wiring_pinout` diagram |
| H9 | Confirm the colour sensor is a TCS34725 (firmware checks chip ID 0x44) | README §5.2 |

## Measurements (car)

| # | Measurement | How |
|---|---|---|
| M1 | Current draw: Pi 5, Pico + sensors, each XL330 (typical / peak) | USB / inline power meter during a run |
| M2 | ToF refresh rate per sensor | `stats` command via `pico_hub_test.py` |
| M3 | AI loop rate and inference time | `[ai_run PERF]` log lines |
| M4 | Success rate over N runs, Open and Obstacle; parking success | test log sheet |
| M5 | Heading correction applied by each wall calibration | `[gyro_wall_cal]` log lines |
| M6 | Minimum turning radius | drive a full circle at steering ±1 |

## Team and media

| # | Item |
|---|---|
| T1 | 2026 team members, roles, coach, home city |
| T2 | Competition(s) entered in 2026 and results |
| T3 | Performance video links (Open + Obstacle) → `video/video.md` |
| T4 | Photos → [`photo-checklist.md`](photo-checklist.md) |

## Software

| # | Item |
|---|---|
| S1 | Apply and test the Pi-side serial adapter ([software.md §2.3](software.md#23-compatibility-note-the-pi-parser-needs-an-adapter)) |
| S2 | Pin the installed package versions (`pip freeze` on the Pi) in `src/pi5/requirements.txt` |
| S3 | Publish the four competition `.h5` files as a GitHub Release |
| S4 | Reason for removing the heading condition from the parking gate (V04 → V05) |
