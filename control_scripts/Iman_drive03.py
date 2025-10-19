#!/usr/bin/env python3
# drive_with_gyro_guard.py
# -------------------------------------------------------------
# 1.  Sense HAT joystick lets you pick one of four model paths
#     (FCW / FCCW / OCW / OCCW).
# 2.  Donkey Car vehicle starts in *driving* (autonomous) mode
#     using that model.
# 3.  A gyroscope part integrates cumulative yaw.
#     If abs(yaw) exceeds 3 turns (1080 deg) the throttle stops
#     and the program exits cleanly.

import os
import sys
import time
import signal
from pathlib import Path
from typing import Tuple

# ------------------ Sense HAT selection -----------------------------------
from sense_hat import SenseHat
from time import sleep

# ------------------ Donkey Car imports ------------------------------------
from buildhat import Motor
import donkeycar as dk
import pygame
from donkeycar.parts.transform import Lambda
from donkeycar.parts.keras import KerasInterpreter, KerasLinear

FREERUN_MODEL_PATH_CW    = "~/projectbuildhat/freeruncwmodels13nat/mypilot.h5"
FREERUN_MODEL_PATH_CCW   = "~/projectbuildhat/freerunccwmodels15/mypilot.h5"
OBSTACLE_MODEL_PATH_CW   = "~/projectbuildhat/obstacleruncwmodels50nat/mypilot.h5"
OBSTACLE_MODEL_PATH_CCW  = "~/projectbuildhat/models/occwm/occwm001-010/mypilot.h5"



sense = SenseHat()
sense.set_rotation(180)
sense.low_light = True
sense.clear()

def flash(msg, seconds=0.7):
    sense.show_message(msg, scroll_speed=0.10, text_colour=[255, 255, 255])
    sleep(seconds)

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
DRIVE_LOOP_HZ      = 20
JOYSTICK_DEADZONE  = 0.05
RECORD_THRESHOLD   = 0.05               # throttle magnitude to start recording
MAX_SPEED_PERCENT  = 30
STEERING_MAX_SPEED = 100
angle_offset       = 0.7

YAW_LIMIT_DEG = 1030.0

# 3 turns
GYRO_CAL_SEC  = 3.0
GYRO2DEG      = 57.2957795  # rad/s -> deg/s

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


# ------------------ Helpers ----------------------------------------------
def center_crop(img, tw=CROP_W, th=CROP_H):
    h, w = img.shape[:2]
    x0 = (w - tw) // 2
    y0 = (h - th) // 2 - 6 
    return img[y0:y0 + th, x0:x0 + tw]

# ------------------ Gyro parts -------------------------------------------
from sense_hat import SenseHat as SH

# ---------------------------------------------------------------------------
# GyroYaw - 仅使用 Z 轴陀螺仪积分，提供可重置的 offset
# ---------------------------------------------------------------------------
from sense_hat import SenseHat

GYRO_CAL_SEC  = 3.0        # 校准时长 (秒)
GYRO2DEG      = 57.2957795 # rad/s -> deg/s (180 / pi)

class GyroYaw:
    """
    * 启动时先平均 GYRO_CAL_SEC 秒得到 bias
    * yaw 累计所有增量
    * set_offset(val) 可把当前 yaw 设为新的零点
    * run() 返回 (yaw - offset)
    """
    def __init__(self):
        self.sh = SenseHat()
        # 只开陀螺仪, 关掉加速计/磁力计减少干扰
        self.sh.set_imu_config(True, False, False)
        self.bias = self._calibrate_bias()
        self.yaw = 0.0
        self.offset = 0.0
        self.last_time = time.time()

    # ---------------------------------------------------------------------
    def _calibrate_bias(self):
        print("GyroYaw: calibrating bias for {} s...".format(GYRO_CAL_SEC))
        s = 0.0
        n = 0
        t0 = time.time()
        while time.time() - t0 < GYRO_CAL_SEC:
            s += self.sh.get_gyroscope_raw()["z"]
            n += 1
            time.sleep(0.01)
        bias = s / n if n else 0.0
        print("GyroYaw: bias =", bias, "rad/s")
        return bias

    # ---------------------------------------------------------------------
    def set_offset(self, val):
        """将当前累积 yaw 记为新的零点"""
        self.offset = val

    # ---------------------------------------------------------------------
    def run(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # 积分角速度 (扣除 bias) -> 累加到 yaw
        z_rad_s = self.sh.get_gyroscope_raw()["z"] - self.bias
        self.yaw += z_rad_s * GYRO2DEG * dt

        # 返回减去 offset 后的角度
        return self.yaw - self.offset

class YawGuard:
    """Stops throttle and raises KeyboardInterrupt if |yaw| > limit."""
    def __init__(self, limit_deg=YAW_LIMIT_DEG):
        self.limit = limit_deg
    def run(self, throttle, yaw):
        if abs(yaw) > self.limit:
            print("\nYaw limit reached ({:.1f} deg) - stopping.".format(yaw))
            raise KeyboardInterrupt
        return throttle  # safe throttle passes through

# ---------------------------------------------------------------------------
# Hardware parts
# ---------------------------------------------------------------------------
class LegoSteering:
    """Single steering motor on Build HAT port A."""
    def __init__(self, port="A", left=-70, right=70):
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



# ---------------------------------------------------------------------------
# ColorLineCounterThreaded - RGBI 動態基線 + 滯回 + 去抖 (非阻塞)
# 讀取放在自建線程中 (rate_hz)，主循環只取匯總計數
# ---------------------------------------------------------------------------
from buildhat import ColorSensor
import threading, time

class ColorLineCounterThreaded:
    def __init__(self,
                 rate_hz=50,
                 base_i_alpha=0.05,     # 白底 I 基線的 EMA 系數
                 base_i_min=240,        # I >= 此值才認定為白底可更新基線
                 blue_enter=0.78,       # i_rel < 0.78 視為進入藍線
                 blue_hold=0.85,        # i_rel < 0.85 視為仍在藍線(滯回)
                 orange_margin=0.18,    # r_n - max(g_n,b_n) > 0.18 判定橙線
                 orange_i_rel_min=0.85, # 橙線要求 i_rel 夠亮，避免深紅誤判
                 min_hold_s=0.03,       # 進入後至少駐留 30ms 才記一次
                 min_gap_s=0.08,        # 離開後至少間隔 80ms 才允許下一次
                 debug=False):
        self.sensor = ColorSensor("D")
        self.orange = 0
        self.blue   = 0

        # 動態基線與參數
        self.I_base = None
        self.base_i_alpha = float(base_i_alpha)
        self.base_i_min   = int(base_i_min)
        self.blue_enter   = float(blue_enter)
        self.blue_hold    = float(blue_hold)
        self.orange_margin= float(orange_margin)
        self.orange_i_rel_min = float(orange_i_rel_min)

        # 去抖/滯回狀態
        self.state = None          # None / "blue" / "orange"
        self.enter_t = 0.0
        self.exit_t  = 0.0
        self.min_hold_s = float(min_hold_s)
        self.min_gap_s  = float(min_gap_s)

        # 執行緒
        self._dt   = 1.0 / float(rate_hz)
        self._stop = False
        self.debug = bool(debug)
        t = threading.Thread(target=self._worker, daemon=True)
        t.start()

    def _update_base(self, i):
        # 僅在高亮白底時更新基線
        if i >= self.base_i_min:
            if self.I_base is None:
                self.I_base = float(i)
            else:
                a = self.base_i_alpha
                self.I_base = (1.0 - a) * self.I_base + a * float(i)

    def _classify_with_hysteresis(self, r, g, b, i):
        """
        返回 None / "blue" / "orange"
        藍線：以 i_rel 變暗為主，帶滯回
        橙線：R 通道相對占優，且 i_rel 足夠亮
        """
        # 更新白底基線
        self._update_base(i)
        if self.I_base is None:
            return None  # 基線未建立，先不分類

        # 相對亮度 (避免絕對 I 受光源影響)
        i_rel = float(i) / max(self.I_base, 1.0)
        i_rel = min(max(i_rel, 0.0), 2.0)

        # 歸一化通道 (相對亮度)
        den = float(max(i, 1))
        r_n, g_n, b_n = r/den, g/den, b/den

        # --- blue by darkness (with hysteresis) ---
        if self.state == "blue":
            blue_now = (i_rel < self.blue_hold)
        else:
            blue_now = (i_rel < self.blue_enter)

        # 輕度藍色優勢作為輔助 (可放寬至 >= 0.02)
        if blue_now and not (b_n >= r_n + 0.02 and b_n >= g_n + 0.02):
            # 不強制，但若無藍優勢且僅短暫變暗，易誤觸 -> 仍允許，以 I 為主
            pass

        # --- orange by red dominance (and bright enough) ---
        orange_now = ( (r_n - max(g_n, b_n)) > self.orange_margin and
                       i_rel >= self.orange_i_rel_min )

        # 互斥：藍與橙同時成立時，按變暗優先(多見於藍線)
        if blue_now and orange_now:
            return "blue"
        if blue_now:
            return "blue"
        if orange_now:
            return "orange"
        return None

    def _worker(self):
        while not self._stop:
            # 讀取 (兼容某些庫版本只回 RGB)
            data = self.sensor.get_color_rgbi()
            if len(data) == 4:
                r, g, b, i = data
            else:
                r, g, b = data
                i = self.sensor.get_reflected_light()

            kind = self._classify_with_hysteresis(r, g, b, i)
            now = time.monotonic()

            if self.state is None:
                # 嘗試進入線條狀態
                if kind is not None and (now - self.exit_t) >= self.min_gap_s:
                    self.state = kind
                    self.enter_t = now
                    if self.debug:
                        print("enter", kind, "RGBI=", r, g, b, i)
            else:
                # 仍在某條線
                if kind != self.state:
                    # 退出：判定駐留時間是否足夠
                    hold = now - self.enter_t
                    if hold >= self.min_hold_s:
                        if self.state == "orange":
                            self.orange += 1
                        elif self.state == "blue":
                            self.blue += 1
                        if self.debug:
                            print("count", self.state, "hold={:.3f}s".format(hold))
                    # 重置
                    self.state = None
                    self.exit_t = now

            time.sleep(self._dt)

    # DonkeyCar 調用：取當前計數
    def run(self):
        return self.orange, self.blue

    def shutdown(self):
        self._stop = True



# ---------------------------------------------------------------------------
# StopGuard - 同时满足 yaw 和 颜色计数后延时 3 秒停车
# ---------------------------------------------------------------------------
class StopGuard:
    def __init__(self, yaw_lim=YAW_LIMIT_DEG,
                 need_orange=12, need_blue=12, delay_sec=3.0):
        self.yaw_lim = yaw_lim
        self.need_o  = need_orange
        self.need_b  = need_blue
        self.delay   = delay_sec
        self.line_met_time = None

    def run(self, throttle, yaw, orange_cnt, blue_cnt):
        now = time.monotonic()

        # 已满足线条数量条件
        if (orange_cnt + blue_cnt) >= (self.need_o +  self.need_b):
            if self.line_met_time is None:
                self.line_met_time = now          # 首次满足, 开始计时
        else:
            self.line_met_time = None             # 条件被打破, 重新计

        # 两条件都满足并且延时达到
        if (abs(yaw) > self.yaw_lim or
            self.line_met_time is not None and
            now - self.line_met_time >= self.delay):
            print("\nStopGuard: 条件满足, 停车.")
            raise KeyboardInterrupt


        return throttle  # 正常输出





# ---------------------------------------------------------------------------
# ConsoleTelemetry - 每 period 秒打印一行状态
# ---------------------------------------------------------------------------
class ConsoleTelemetry:
    def __init__(self, period=0.5):
        self.period = period
        self.next_time = time.monotonic()

    def run(self, angle, throttle, yaw_deg, orange_cnt, blue_cnt):
        now = time.monotonic()
        if now >= self.next_time:
            msg = (
                "Angle {:+6.2f}  Thr {:+6.2f}  "
                "Yaw {:+8.2f} deg  "
                "Orange {:2d}  Blue {:2d}"
            ).format(angle, throttle, yaw_deg, orange_cnt, blue_cnt)
            print(msg)
            self.next_time = now + self.period
        # 无输出



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

            if sim >= self.sim_th:
                print(f"\nStuck detected (sim={sim:.2f}), reversing for {self.rev_time}s")
                self.state    = "REVERSING"
                self.rev_until= now + self.rev_time
                return self.rev_spd, 0.0

        return throttle, angle










def alignment_sequence(drive_mode, gyro):
    """
    OCW / OCCW 对齐动作。现在在终端实时打印 yaw 角度。
    """
    if drive_mode not in ("OCW", "OCCW"):
        return

    steer = LegoSteering()
    
    throttle = LegoThrottle(max_speed=40)

    # 简易打印函数：覆盖同一行
    def print_yaw():
        sys.stdout.write("\rYaw:{:+8.2f} deg".format(gyro.yaw))
        sys.stdout.flush()

    # 带打印的等待封装
    def wait_until(cond):
        while True:
            yaw_val = gyro.run()   # 更新 yaw
            print_yaw()
            if cond(yaw_val):
                break
            time.sleep(0.01)
        print()  # 换行

    def settle(duration, step=0.02):
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            gyro.run()
            print_yaw()
            time.sleep(step)


    # 转向/驱动 lambda 与原逻辑保持一致
    turn_r = lambda: steer.run(+0.6)
    turn_l = lambda: steer.run(-0.6)
    stop_m = lambda: throttle.run(0.0)
    fwd    = lambda: throttle.run(+0.5)
    back   = lambda: throttle.run(-0.5)

    try:
        if drive_mode == "OCW":
            turn_r();  settle(0.4)
            fwd();  wait_until(lambda y: y >  15);  stop_m()

            turn_l();  settle(0.4)
            back(); wait_until(lambda y: y >  35);  stop_m()

            turn_r();  settle(0.4)
            fwd();  wait_until(lambda y: y >  50);  stop_m()

            turn_l();  settle(0.4)
            fwd();  wait_until(lambda y: y <  10);  stop_m()

        else:  # OCCW
            turn_l();  settle(0.4)
            fwd();  wait_until(lambda y: y < -20);  stop_m()

            turn_r();  settle(0.4)
            back(); wait_until(lambda y: y < -35);  stop_m()

            turn_l();  settle(0.4)
            fwd();  wait_until(lambda y: y < -50);  stop_m()

            turn_r();  settle(0.4)
            fwd();  wait_until(lambda y: y > -10);  stop_m()

        
        print("car will stop and Gyro settle 3 s ...")
        stop_m()
        steer.shutdown()
        throttle.shutdown()
        t0 = time.time()
        while time.time() - t0 < 3.0:
            yaw_now = gyro_global.run()
            sys.stdout.write("\rIdle yaw:{:+.2f} ".format(yaw_now))
            sys.stdout.flush()
            time.sleep(0.02)
        print()
        
        






    except KeyboardInterrupt:
        pass
    finally:
        stop_m()
        steer.shutdown()
        throttle.shutdown()




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

# ------------------ Vehicle builder --------------------------------------
def build_vehicle(model_path):
    car = dk.vehicle.Vehicle()
    add_camera(car)

    # AI pilot
    interp = KerasInterpreter()
    pilot  = KerasLinear(interpreter=interp, input_shape=(CROP_H, CROP_W, 3))
    pilot.load(model_path)
    car.add(pilot, inputs=["cam/image_array"], outputs=["pilot/angle","pilot/throttle"])

#     # Gyro yaw part
# 
#     car.add(gyro, outputs=["yaw"])
# 
#  
#     # NEW: 线程版颜色计数器
#     line_counter = ColorLineCounterThreaded(rate_hz=25)
#     car.add(
#         line_counter,
#         outputs=["cnt/orange", "cnt/blue"],
#         threaded=False      # 重点: 放线程里
#     )
# 
#     # NEW: 复合守卫 (取代原先 YawGuard )
#     guard = StopGuard()
#     car.add(
#         guard,
#         inputs=["pilot/throttle", "yaw", "cnt/orange", "cnt/blue"],
#         outputs=["safe/throttle"]
#     )

#     car.add(
#         ConsoleTelemetry(),
#         inputs=[
#             "pilot/angle",
#             "pilot/throttle"            
#         ],
#         outputs=[]
#     )
#     # Yaw guard filters throttle
# #     guard = YawGuard()
# #     car.add(guard, inputs=["pilot/throttle","yaw"], outputs=["safe/throttle"])
# 
#     stuck = StuckDetector(
#         window_sec=2.0,
#         sim_thresh=0.9,
#         diff_thresh=10,
#         reverse_time=7.0,
#         reverse_speed=-0.5,
#         warmup_initial=7.0,
#         warmup_post_rev=7.0
#     )
#     car.add(
#         stuck,
#         inputs=["cam/image_array","safe/throttle","pilot/angle"],
#         outputs=["final/throttle","final/angle"]
#     )

    car.add(LegoSteering(), inputs=["pilot/angle"])
    car.add(LegoThrottle(), inputs=["pilot/throttle"])
    
    
    return car


# ---------------------------------------------------------------
# one global gyro instance: zero here, then reused everywhere
# ---------------------------------------------------------------
# gyro_global = GyroYaw()


#alignment_sequence(DRIVE_MODE, gyro_global)
# ------------------ Main --------------------------------------------------

vehicle = build_vehicle(MODEL_PATH_DEFAULT)

def shutdown(_sig=None, _frm=
             None):
    print("Shutting down...")
    if hasattr(vehicle, "shutdown"):
        vehicle.shutdown()
    else:
        vehicle.stop()
    sense.clear()
    sys.exit(0)

signal.signal(signal.SIGTERM, shutdown)

try:
    vehicle.start(rate_hz=DRIVE_LOOP_HZ)
except KeyboardInterrupt:
    print("KeyboardInterrupt received.")
finally:
    
    shutdown()







  

 