# Source Code

Everything that runs on the vehicle. There are two targets:

| Target | Language | Runs | Folder |
|---|---|---|---|
| **Raspberry Pi 5** | Python 3 | Camera, CNN inference, motion control, DYNAMIXEL bus | [`pi5/`](pi5) |
| **Raspberry Pi Pico 2** | MicroPython | BNO085 IMU, 6 × ToF, TCS34725 colour sensor, USB serial stream | [`pico2/`](pico2) |

> **Provenance.** These files were copied verbatim from our development repository
> [`TopWRO/wro-fe-2026`](https://github.com/TopWRO/wro-fe-2026). The only changes are
> added `#` comment lines: an English header in each file and an English API note above
> each important class and function. We checked this with Python's `ast` module: every
> migrated file has exactly the same syntax tree as its original, so the code here is
> the code that runs on the car. The team's original Chinese inline comments are kept.

## Layout

```text
src/
├── pi5/
│   ├── competition/                  # competition entry points + core library
│   │   ├── AFI_CUP_V05.py            # core library, Obstacle build (models, speed limit 0.35)
│   │   ├── AFI_CUP_V05f.py           # core library, Open build (free-run models, speed limit 0.70)
│   │   ├── AFI_CUP_ORun.py           # ▶ Obstacle Challenge: unpark → 3 AI laps → re-calibrate → park
│   │   ├── AFI_CUP_FRun.py           # ▶ Open Challenge: 3 AI laps → gyro-held stop in start section
│   │   └── AFI_CUP_ORun_main_CW_CCW_V01.py   # parking-only tuning script
│   ├── data_collection/
│   │   └── Recording_N02.py          # PS4 driving → DonkeyCar tub (training data) / model bench drive
│   ├── manual_control/
│   │   └── rc_with_dxl_error_check_recover.py  # RC driving with DYNAMIXEL fault monitoring
│   ├── tools/                        # bench tools (see the table below)
│   └── requirements.txt
└── pico2/                            # copy ALL files to the Pico root (flat layout is required)
    ├── main.py                       # entry file: core 0 = USB / IMU / colour, 20 Hz output
    ├── sensor_hub_tof.py             # core 1 = six-ToF round-robin worker
    ├── vl53l4cd.py · vl53l4cx.py · i2c_device.py   # third-party drivers (Adafruit, MIT)
    └── vl53l4cd_patch.py · vl53l4cx_patch.py       # our driver extensions (thresholds, full measurement)
```

The core library and the run scripts must stay in the **same folder**, because the run
scripts use `import AFI_CUP_V05 as R` (or `AFI_CUP_V05f`).

## Mapping to the development repository

| Here | `wro-fe-2026` | Notes |
|---|---|---|
| `pi5/competition/*` | `AFI_CUP_*.py` (repo root) | unchanged names |
| `pi5/data_collection/Recording_N02.py` | `Recording_N02.py` | supersedes `Recording_N01.py` (same tub format) |
| `pi5/manual_control/…` | `rc_with_dxl_error_check_recover.py` | supersedes `rc_with_dxl_error_check.py`, `ControlRCcarwithPSControler.py` |
| `pi5/tools/analyze_orange_image.py` | `analyze_orange_image.py` (root) | the root copy is newer than `Models/analyze_orange_image.py` |
| `pico2/*` | `Pi Pico Scripts/*` | `vl53l4cd_patch (1).py` renamed to `vl53l4cd_patch.py`, the name `sensor_hub_tof.py` imports. `pico_sensor_hub_20hz.py` is a byte-identical duplicate of `main.py` and was not copied |

These were not copied: `face_detect.py` (camera demo), `usb3.0_serial04.py` (reader for the
legacy 7-field protocol), trained models (see [Releases](#trained-models)) and tub data.

## Tools

| Script | Moves motors? | Purpose |
|---|:---:|---|
| `tools/camera_view.py` | no | Live preview with the exact competition camera settings |
| `tools/line_color_detect.py` | no | Floor-line HSV detection and calibration |
| `tools/analyze_orange_image.py` | no | Interactive HSV colour sampler (drag a box) |
| `tools/pico_hub_test.py` | no | Pico hub monitor: sets ToF timing budgets, `stats`, `--test`, `--soft-reset` |
| `tools/dxl_recover_test.py` | **steering** (centres) | Print DYNAMIXEL status, reboot faulted motors, drive velocity 0, steering to centre (2047) |
| `tools/steer_45_test.py` | **steering** | Swing steering ±45° three times |
| `tools/spin_motor2.py` | **drive** | Spin drive motor at ~50 rpm for 3 s (lift the wheels first) |

## Running

```bash
# Raspberry Pi 5 (inside the venv)
cd src/pi5/competition
python3 AFI_CUP_FRun.py        # Open Challenge      ⚠ motors move
python3 AFI_CUP_ORun.py        # Obstacle Challenge  ⚠ motors move

# Pico 2: copy every file in src/pico2/ to the Pico root, e.g. with mpremote
mpremote cp src/pico2/*.py :
mpremote reset
```

Model paths are set at the top of the core library (`MODEL_PATH_CW` / `MODEL_PATH_CCW`)
and expect the models in `~/Top WRO FE 2026/Models/`.

## Trained models

The four competition networks are published as **GitHub Release assets** (≈ 9.9 MB each)
and are not committed to git:

| Challenge | Direction | File | Loaded by |
|---|---|---|---|
| Open | CW | `TransferFcwmypilot.h5` | `AFI_CUP_V05f.py` |
| Open | CCW | `TransferFccwmypilot.h5` | `AFI_CUP_V05f.py` |
| Obstacle | CW | `ocw11mypilot.h5` | `AFI_CUP_V05.py` |
| Obstacle | CCW | `occw10mypilot.h5` | `AFI_CUP_V05.py` |

## Serial protocol note

The current Pico firmware sends the labelled 11-field line. The Pi parser in
`UnifiedRobotIO._read_latest_serial_frame()` still expects the legacy 7-field CSV.
The adapter that is needed is described in
[`docs/software.md`](../docs/software.md#2-serial-protocol).
