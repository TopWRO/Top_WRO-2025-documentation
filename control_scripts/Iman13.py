#!/usr/bin/env python3
"""manage_combined.py – Manual recording **or** autonomous driving
-----------------------------------------------------------------
Flags (choose exactly one):
    --recording   ️Use PS4 pad, store images + user controls in a Tub
    --driving     Load a trained `.h5` model, predict angle/throttle, drive

In **driving** mode the script now prints the predicted angle and throttle
values every loop so you can watch them live in the terminal.
"""

# ---------------------------------------------------------------------------
# Standard library
# ---------------------------------------------------------------------------
import os
import sys
import time
import signal
from pathlib import Path
from typing import Tuple
from sense_hat import SenseHat
from time import sleep

# Third‑party
from buildhat import Motor
import donkeycar as dk
import numpy as np
import pygame
from donkeycar.parts.transform import Lambda
from donkeycar.parts.tub_v2 import TubWriter, TubWiper

try:
    from donkeycar.parts.keras import KerasInterpreter, KerasLinear
except ImportError:
    sys.exit("Keras/TensorFlow not available — install DonkeyCar with AI support.")


FREERUN_MODEL_PATH_CW    = "~/projectbuildhat/models/fcwm/fcwm008/mypilot.h5"
FREERUN_MODEL_PATH_CCW   = "~/projectbuildhat/models/fccwm/fccwm007/mypilot.h5"
OBSTACLE_MODEL_PATH_CW   = "~/projectbuildhat/models/ocwm/ocwm001-008/mypilot.h5"
OBSTACLE_MODEL_PATH_CCW  = "~/projectbuildhat/models/occwm/occwm001-010/mypilot.h5"



sense = SenseHat()
sense.set_rotation(180)
sense.low_light = True
sense.clear()

def flash(msg, seconds=0.7):
    sense.show_message(msg, scroll_speed=0.10, text_colour=[255, 255, 255])
    sleep(seconds)

sense.clear()

DRIVE_MODE = "FCW"
flash(DRIVE_MODE)
print("Move joystick, press middle to confirm.")
while True:
    for ev in sense.stick.get_events():
        if ev.action != "pressed":
            continue
        if ev.direction == "left":
            DRIVE_MODE = "FCW";  flash("FCW")
        elif ev.direction == "right":
            DRIVE_MODE = "FCCW"; flash("FCCW")
        elif ev.direction == "up":
            DRIVE_MODE = "OCW";  flash("OCW")
        elif ev.direction == "down":
            DRIVE_MODE = "OCCW"; flash("OCCW")
        elif ev.direction == "middle":
            flash(DRIVE_MODE)
            sense.clear()
            print("Selection finished.")
            break
    else:
        continue
    break

if DRIVE_MODE == "FCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(FREERUN_MODEL_PATH_CW)
elif DRIVE_MODE == "FCCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(FREERUN_MODEL_PATH_CCW)
elif DRIVE_MODE == "OCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(OBSTACLE_MODEL_PATH_CW)
elif DRIVE_MODE == "OCCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(OBSTACLE_MODEL_PATH_CCW)
else:
    sys.exit("Invalid DRIVE_MODE")

print("Model:", MODEL_PATH_DEFAULT)






# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CAPTURE_W, CAPTURE_H = (176, 132)      # camera capture
CROP_W, CROP_H       = (160, 120)      # crop sent to network / tub

DATA_PATH          = os.path.expanduser("~/projectbuildhat/data")


DRIVE_LOOP_HZ      = 20
JOYSTICK_DEADZONE  = 0.05
RECORD_THRESHOLD   = 0.05               # throttle magnitude to start recording
MAX_SPEED_PERCENT  = 25
STEERING_MAX_SPEED = 100
angle_offset       = 0.7

TUB_INPUTS = [
    "cam/image_array",
    "user/angle",
    "user/throttle",
    "user/mode",
]
TUB_TYPES  = ["image_array", "float", "float", "str"]


# ---------------------------------------------------------------------------
# 摄像头参数设置 —— 用于禁用自动曝光/白平衡，并手动锁定参数
# ---------------------------------------------------------------------------
CAMERA_FRAMERATE             = 30        # 帧率
PICAMERA_AWB_MODE            = 'off'     # 关闭自动白平衡
PICAMERA_EXPOSURE_MODE       = 'off'     # 关闭自动曝光
PICAMERA_ISO                 = 100       # 手动 ISO
PICAMERA_SHUTTER_SPEED       = 15000     # 快门速度 (微秒)，例如 15000≈1/15s
PICAMERA_AWB_GAINS           = (1.5, 1.2)# 自定义白平衡增益 (红, 蓝)
PICAMERA_EXPOSURE_COMPENSATION = 0       # 曝光补偿（自动模式生效，可留0）





# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def center_crop(img, tw=CROP_W, th=CROP_H):
    h, w = img.shape[:2]
    x0 = (w - tw) // 2
    y0 = (h - th) // 2 - 6 
    return img[y0:y0 + th, x0:x0 + tw]

# ---------------------------------------------------------------------------
# Hardware parts
# ---------------------------------------------------------------------------
class LegoSteering:
    """Single steering motor on Build HAT port A."""
    def __init__(self, port="A", left=-60, right=60):
        self.motor = Motor(port)
        self.left, self.right = left, right
        self.prev_pos = None
        time.sleep(0.1)
        self.motor.run_to_position(0.0); self.motor.stop()

    def run(self, angle: float):
        angle = angle* angle_offset
        angle = max(min(angle, 1.0), -1.0)
        raw   = self.left + (angle + 1) * (self.right - self.left) / 2
        pos   = int(round(raw / 10) * 10)
#         print(round(angle,2),round(raw,2),round(pos,2))
#         if pos == 0:
#             pass
#         elif pos > 0 :
#             pos = pos +10
#         else:
#             pos = pos-10

        speed = int(angle * STEERING_MAX_SPEED)


        if pos ==0 and pos != self.prev_pos:
            self.motor.stop()
            self.motor.run_to_position(-3, speed=100, blocking=False)            

        if pos!= 0 and pos != self.prev_pos and -40<=pos<=40 :
            self.motor.stop()
            self.motor.run_to_position(pos, speed=100, blocking=False)

        if pos!= 0 and pos != self.prev_pos and ((-40>pos)or (pos>40)) :
            self.motor.stop()
            self.motor.start(speed=speed)        
        
        if pos!= 0 and pos == self.prev_pos:
            return

        self.prev_pos = pos
        return
    def shutdown(self):
        self.motor.run_to_position(0); self.motor.stop()
        try:
            self.motor.close()
        except AttributeError:
            pass


class LegoThrottle:
    """Drive Build HAT motors on ports B and C."""
    def __init__(self, left_port="B", right_port="C", max_speed=MAX_SPEED_PERCENT):
        self.ml, self.mr = Motor(left_port), Motor(right_port)
        self.max_speed = int(max(min(max_speed, 100), 0))
        self._stop()
        self.last_speed = None

    def _stop(self):
        print('run stop!')
        for m in (self.ml, self.mr):
            m.stop_action = "brake"
            m.stop()
            m.float()


    def run(self, throttle: float):
        throttle = max(min(throttle, 1.0), -1.0)
        speed = int(throttle * self.max_speed)
        speed = int(round(speed / 10.0) * 10)
#         print(speed)
        if self.last_speed ==0 and speed ==0:
            return
        if self.last_speed!= 0 and speed == 0:

            self._stop()
            
        if speed !=0 and speed == self.last_speed:
            return
        
        if speed !=0 and speed != self.last_speed:
#             self._stop()
            self.ml.start(speed=speed)
            self.mr.start(speed=speed)

            
        
#         
#         if speed == self.last_speed:
#             return                       # nothing to do
#         else:
#             if speed == 0:
#                 self._stop()
#             else:
#                 self.ml.start(speed=speed)
#                 self.mr.start(speed=speed)
#             
        
        
        
        self.last_speed = speed

    def shutdown(self):
        self._stop()
        # Release Build HAT ports so script can restart in same interpreter
        for m in (self.ml, self.mr):
            try:
                m.close()
            except AttributeError:
                pass


import numpy as np
import time

class StuckDetector:
    """
    run(img, throttle, angle) -> (throttle_out, angle_out)
    """
    def __init__(self,
                 window_sec=2.0,
                 sim_thresh=0.9,
                 diff_thresh=10,
                 reverse_time=5.0,
                 reverse_speed=-0.5,
                 warmup_initial=5.0,
                 warmup_post_rev=3.0):
        self.window   = float(window_sec)
        self.sim_th   = float(sim_thresh)
        self.diff_th  = int(diff_thresh)
        self.rev_time = float(reverse_time)
        self.rev_spd  = float(reverse_speed)
        self.warm_i   = float(warmup_initial)
        self.warm_p   = float(warmup_post_rev)

        now = time.monotonic()
        self.start_t    = now
        self.post_rev_t = now

        self.history = []
        self.state   = "NORMAL"
        self.rev_until = 0.0

    def _similarity(self, img0, img1):
        diff = np.abs(img0.astype(int) - img1.astype(int))
        mask = ((diff[:,:,0] < self.diff_th) &
                (diff[:,:,1] < self.diff_th) &
                (diff[:,:,2] < self.diff_th))
        return mask.sum() / mask.size

    def run(self, img, throttle, angle):
        now = time.monotonic()
        if now - self.start_t < self.warm_i:
            return throttle, angle

        if now < self.post_rev_t:
            return throttle, angle

        self.history.append((now, img.copy()))
        cutoff = now - self.window
        while self.history and self.history[0][0] < cutoff:
            self.history.pop(0)

        if self.state == "REVERSING":
            if now < self.rev_until:
                self.history.clear()
                return self.rev_spd, 0.0
            else:

                self.state = "NORMAL"
                self.history.clear()
                self.post_rev_t = now + self.warm_p
                return throttle, angle

        if len(self.history) >= 2:
            _, img0 = self.history[0]
            sim = self._similarity(img0, img)
#             print(sim)
            if sim >= self.sim_th:
                print(f"\nStuck detected (sim={sim:.2f}), reversing for {self.rev_time}s")
                self.state    = "REVERSING"
                self.rev_until= now + self.rev_time
                return self.rev_spd, 0.0

        return throttle, angle






class PS4Joystick:
    def __init__(self, deadzone=JOYSTICK_DEADZONE):
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No PS4 controller detected.")
        self.js = pygame.joystick.Joystick(0); self.js.init()
        self.dz = deadzone
        print(f"Connected joystick: {self.js.get_name()}")

    def _dz(self, v):
        return 0.0 if abs(v) < self.dz else float(v)

    def run(self) -> Tuple[float, float, str]:
        pygame.event.pump()
        angle = self._dz(self.js.get_axis(0))
        throttle = -self._dz(self.js.get_axis(4))
        # Triangle 键 (button 2) → 标记要删除
        if self.js.get_button(2):
            return angle, throttle, "erase"

        # L1 键 (button 4) → 退出程序
        if self.js.get_button(4):
            raise KeyboardInterrupt

        # 正常驾驶模式
        return angle, throttle, "user"

    def shutdown(self):
        pygame.quit()

# ---------------------------------------------------------------------------
# Display part for driving mode
# ---------------------------------------------------------------------------
class ConsoleDisplay:
    def __init__(self):
        self.last_t = 0
    def run(self, angle: float, throttle: float):
        t = time.monotonic()
        if t - self.last_t >= 1.0:              # one message per second
            print(f"Pred → angle {angle:+.2f}  thr {throttle:+.2f}")
            self.last_t = t


# ---------------------------------------------------------------------------
# 可复用的摄像头初始化（recording & driving 都会调用）
# ---------------------------------------------------------------------------
def add_camera(car: dk.vehicle.Vehicle):
    from donkeycar.parts.camera import PiCamera

    # 1) 创建 PiCamera，并关闭自动曝光/白平衡
    cam = PiCamera(
        image_w = CAPTURE_W,
        image_h = CAPTURE_H,
        image_d = 3
    )

    # 2) 等待相机内部自动测光／白平衡稳定（尤其在 off 模式下要给 hardware 一点时间）
    time.sleep(2)


    cam.camera.framerate           = CAMERA_FRAMERATE
    cam.camera.exposure_mode       = PICAMERA_EXPOSURE_MODE
    cam.camera.awb_mode            = PICAMERA_AWB_MODE
    cam.camera.iso                 = PICAMERA_ISO
    cam.camera.shutter_speed       = PICAMERA_SHUTTER_SPEED
    cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
    cam.camera.awb_gains           = PICAMERA_AWB_GAINS

    # 4) 加入车辆管线
    car.add(cam, outputs=["cam/raw"], threaded=True)
    car.add(Lambda(center_crop), inputs=["cam/raw"], outputs=["cam/image_array"])

# ---------------------------------------------------------------------------
# Vehicle builders
# ---------------------------------------------------------------------------

def build_vehicle_recording() -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    # Controller and actuators
    car.add(PS4Joystick(), outputs=["user/angle", "user/throttle", "user/mode"])
    car.add(LegoSteering(), inputs=["user/angle"])
    car.add(LegoThrottle(), inputs=["user/throttle"])

    # Write only when driver pushes throttle stick beyond threshold
    car.add(Lambda(lambda t: abs(t) > RECORD_THRESHOLD),
            inputs=["user/throttle"], outputs=["recording"])

    tub = TubWriter(base_path=DATA_PATH, inputs=TUB_INPUTS, types=TUB_TYPES)
    car.add(tub, inputs=TUB_INPUTS, outputs=["tub/num_records"], run_condition="recording")

    # Print count every 10 records
    class RecordCounter:
        """Display total sample count every 10 new records."""
        def __init__(self):
            self.last_ten = -1
        def run(self, n):
            if n is None:
                return  # no samples yet
            ten = n // 10
            if ten != self.last_ten:
                print(f"Recorded samples: {n}")
                self.last_ten = ten
    
    car.add(RecordCounter(), inputs=["tub/num_records"], outputs=[])
    # ———— 这里开始插入 PromptWiper ————
    class PromptWiper:
        def __init__(self, tub, num_records=100):
            self.tub = tub
            self.num = num_records
            self.erased = False

        def run(self, mode):
            if mode == "erase" and not self.erased:
                self.tub.delete_last_n_records(self.num)
                print(f"Deleted last {self.num} records.")
                self.erased = True
            if mode != "erase":
                self.erased = False

    wiper = PromptWiper(tub.tub, num_records=100)
    car.add(wiper, inputs=["user/mode"], outputs=[])
    # ———— PromptWiper 插入结束 ————
    return car


def build_vehicle_driving(model_path: str) -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    interpreter = KerasInterpreter()
    pilot = KerasLinear(interpreter=interpreter, input_shape=(CROP_H, CROP_W, 3))
    pilot.load(model_path)

    car.add(pilot, inputs=["cam/image_array"], outputs=["pilot/angle", "pilot/throttle"])

    # Live console read‑out of predictions
#     car.add(ConsoleDisplay(), inputs=["pilot/angle", "pilot/throttle"], outputs=[])
    
    
    stuck = StuckDetector(
        window_sec=2.0,
        sim_thresh=0.9,
        diff_thresh=10,
        reverse_time=8.0,
        reverse_speed=-0.5,
        warmup_initial=7.0,
        warmup_post_rev=7.0
    )
    car.add(
        stuck,
        inputs=["cam/image_array","pilot/throttle","pilot/angle"],
        outputs=["final/throttle","final/angle"]
    )


    car.add(LegoSteering(), inputs=["final/angle"])
    car.add(LegoThrottle(), inputs=["final/throttle"])
    return car

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Record data or drive autonomously")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--recording", action="store_true", help="Manual driving & data logging")
    group.add_argument("--driving",   action="store_true", help="Autonomous driving with model")
    parser.add_argument("--model", default=MODEL_PATH_DEFAULT, help=".h5 model path for --driving mode")
    args = parser.parse_args()

    if args.recording:
        vehicle = build_vehicle_recording()
    else:
        mpath = Path(args.model).expanduser()
        if not mpath.is_file():
            sys.exit(f"Model file not found: {mpath}")
        vehicle = build_vehicle_driving(str(mpath))

    def _sigterm(_s, _f):
        print("\nSIGTERM – shutting down…")
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
        sys.exit(0)
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        vehicle.start(rate_hz=DRIVE_LOOP_HZ)
    except KeyboardInterrupt:
        print("\nCtrl‑C – shutting down…")
    finally:
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
