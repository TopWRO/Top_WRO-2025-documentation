# Setup Guide

## Raspberry Pi 5

1. Install Raspberry Pi OS (64-bit), enable the camera, and connect the Pico (USB), U2D2 (USB)
   and Sense HAT.
2. Create the virtual environment and install the dependencies:
   ```bash
   python3 -m venv ~/env
   source ~/env/bin/activate
   pip install -r src/pi5/requirements.txt
   ```
3. Check the devices: `/dev/ttyACM0` (Pico 2), `/dev/ttyUSB0` (U2D2).
4. Bench checks that do **not** move the motors:
   `python3 src/pi5/tools/pico_hub_test.py` and `python3 src/pi5/tools/camera_view.py`.

## Raspberry Pi Pico 2

1. Flash the MicroPython firmware for RP2350 (hold BOOTSEL, copy the `.uf2`).
2. Copy **all** files from `src/pico2/` to the root of the board (flat, no sub-folders):
   ```bash
   mpremote cp src/pico2/*.py :
   mpremote reset
   ```
3. The board prints `# INIT F OK …` for each ToF, then one data line every 50 ms.

## DYNAMIXEL

- U2D2 at 57,600 bps, Protocol 2.0. ID 1 = drive (velocity mode), ID 2 = steering (position mode).
- `python3 src/pi5/tools/dxl_recover_test.py` prints the status and reboots motors that report a
  hardware error (the steering moves to its centre position).
- The software never changes IDs or baud rates (EEPROM). Set them once with ROBOTIS DYNAMIXEL Wizard.

## Safety

Every script that moves a motor must be started with the wheels off the table or with the car
on the mat. `Ctrl+C` always triggers a safe stop (`car.shutdown()` in `finally:`).
