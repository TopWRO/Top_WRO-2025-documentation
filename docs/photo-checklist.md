# Photo Checklist

[← back to README](../README.md)

Every image below currently exists as a **generated placeholder** at its final path. Replace
the file with a real photo **under the same name**, and the README picks it up automatically.
`tools/figures/placeholders.py` never overwrites a real photo: it only rewrites files that
carry its placeholder marker.

**General rules:** 4:3 landscape, ≥ 2000 px wide, plain light background for vehicle shots,
good diffuse light, wheels straight unless stated, cables tidy. Export as JPEG and keep each
file under 1 MB.

## Required by WRO

| ☐ | File | Shot |
|---|---|---|
| ☐ | `v-photos/front.jpg` | Camera at chassis height, lens centred on the front ToF (F) |
| ☐ | `v-photos/back.jpg` | Camera at chassis height; rear ToF (B) and drive gear visible |
| ☐ | `v-photos/left.jpg` | Full side profile; L1 / L2 visible |
| ☐ | `v-photos/right.jpg` | Full side profile; R1 / R2 visible |
| ☐ | `v-photos/top.jpg` | Straight down from ~60 cm; whole car in frame |
| ☐ | `v-photos/bottom.jpg` | Straight up; colour sensor and drivetrain visible |
| ☐ | `t-photos/official.jpg` | All members + coach with the vehicle, team name readable |
| ☐ | `t-photos/funny.jpg` | One informal team photo |

## Engineering details (used in the README)

| ☐ | File | Shot |
|---|---|---|
| ☐ | `docs/assets/photos/drivetrain.jpg` | DXL ID 1 + 1:3 gear train + rear axle (differential if present) |
| ☐ | `docs/assets/photos/steering.jpg` | DXL ID 2 + linkage, wheels at full lock |
| ☐ | `docs/assets/photos/sensor_hub.jpg` | Pico 2 + BNO085 + wiring, pin labels readable |
| ☐ | `docs/assets/photos/tof_side_pair.jpg` | R1 / R2 mounts **with a ruler** showing 14.8 cm |
| ☐ | `docs/assets/photos/tof_front_back.jpg` | F and B mounts, height above the floor visible |
| ☐ | `docs/assets/photos/color_sensor.jpg` | TCS34725 under the chassis, LED on |
| ☐ | `docs/assets/photos/camera_mount.jpg` | Side view **with a ruler**: camera height and tilt |
| ☐ | `docs/assets/photos/power.jpg` | Battery, switch, fuse, regulators and wiring |
| ☐ | `docs/assets/photos/u2d2.jpg` | U2D2 and the daisy-chained DYNAMIXEL cables |
| ☐ | `docs/assets/photos/wiring_overview.jpg` | Top deck removed, all cables visible |
| ☐ | `docs/assets/photos/start_ui.jpg` | Sense HAT showing the CW / CCW choice |

## Action shots

| ☐ | File | Shot |
|---|---|---|
| ☐ | `docs/assets/photos/on_track_open.jpg` | Car mid-corner in the Open Challenge |
| ☐ | `docs/assets/photos/on_track_obstacle.jpg` | Car passing a red or green pillar |
| ☐ | `docs/assets/photos/parking.jpg` | Car parked inside the magenta parking lot |

## Measurements to take during the photo session

While the car is out, also measure: length × width × height, wheelbase, track width, wheel
diameter, mass, camera height and tilt, the height and position of every ToF, and the real
front-wheel angle at steering command ±1. These values complete the dimension tables and the
FoV diagram (see [`open-items.md`](open-items.md)).
