#!/usr/bin/env python3
# 4x VL53L0X + Sense HAT 连续航向角(可正可负) + Motor A 转向闭环保持目标角
# 前进: 读front传感器；后退: 读back传感器
# 流程：面向0°前进→前方<=250mm停车→转向+30°→后退直到与初始方向成-90°停车

import time, atexit
import RPi.GPIO as GPIO
import board, busio
import adafruit_vl53l0x
from sense_hat import SenseHat
from buildhat import Motor, MotorPair

# ====== 你的接线保持不变 ======
XSHUT  = [21, 27, 22, 26]               # front, right, back, left  (BCM)
NAMES  = ["front", "right", "back", "left"]
ADDRS  = [0x2A, 0x2B, 0x2C, 0x2D]
DEFAULT = 0x29

# ====== 行为参数（可按车体调整） ======
STEER_LIMIT_DEG = 50                    # 转向电机左右极限（±50°）
STEER_SPEED     = 80                    # 转向电机速度
Kp_steer        = 0.9                   # 转向比例增益：命令角 = clip(-Kp*误差)
STEER_DEADBAND  = 2.0                   # 转向死区（度），误差小于此不动
LOOP_HZ         = 25                    # 主循环频率（Hz）
DT              = 1.0 / LOOP_HZ

SPEED_FWD_LR    = (-50, -50)            # 前进速度(B,C)。若方向相反，把两数取反
SPEED_REV_LR    = ( 50,  50)            # 后退速度(B,C)
STOP_GAP_MM     = 250                   # 前向停车阈值（mm）
DIST_DEBOUNCE   = 0.06                  # 距离二次确认延时（秒）

TARGET_FWD_DEG  = 0.0                   # 段1目标航向（0°）
TURN_ANGLE_DEG  = 30                    # 打轮角度（+30°）
TARGET_REV_DEG  = -90.0                 # 段2相对初始目标航向（-90°）

USE_COMPASS     = False                 # 磁力计是否参与融合（Sense HAT此处只用陀螺+加计）

# ====== 低层工具：改地址 ======
class _RawI2CDev:
    def __init__(self, i2c, addr): self.i2c, self.addr = i2c, addr
    def write(self, bts):
        while not self.i2c.try_lock(): pass
        try: self.i2c.writeto(self.addr, bytes(bts))
        finally:
            try: self.i2c.unlock()
            except: pass

def _scan(i2c):
    while not i2c.try_lock(): pass
    try: lst = i2c.scan()
    finally:
        try: self.i2c.unlock()
        except: pass
    return lst

def _wait(i2c, addr, present, tmax=2.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < tmax:
        ok = (addr in _scan(i2c))
        if ok == present: return True
        time.sleep(0.02)
    return False

def _set_addr(i2c, old_addr, new_addr):
    _RawI2CDev(i2c, old_addr).write([0x8A, new_addr & 0x7F])
    time.sleep(0.02)

def assign_addresses(i2c):
    GPIO.setmode(GPIO.BCM)
    for p in XSHUT:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    time.sleep(0.05)

    for pin, new_addr, name in zip(XSHUT, ADDRS, NAMES):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.12)
        if not _wait(i2c, DEFAULT, True):
            print("ERROR: default 0x29 not seen for", name); return False
        try:
            _set_addr(i2c, DEFAULT, new_addr)
        except Exception as e:
            print("ERROR set addr:", name, e); return False
        if not _wait(i2c, DEFAULT, False): return False
        if not _wait(i2c, new_addr, True): return False
        print("assigned", name, "-> 0x%02X" % new_addr)
    return True

# ====== 连续角度解包器 ======
class YawUnwrapper:
    def __init__(self):
        self.inited   = False
        self.last_raw = 0.0
        self.cum      = 0.0
        self.zero     = 0.0

    def reset_zero(self, raw_deg):
        self.zero = raw_deg
        self.last_raw = raw_deg
        self.cum  = 0.0
        self.inited = True

    def update(self, raw_deg):
        if not self.inited:
            self.reset_zero(raw_deg)
            return 0.0
        rel = raw_deg - self.zero
        while rel >= 180.0: rel -= 360.0
        while rel <  -180.0: rel += 360.0

        prev_rel = self.last_raw - self.zero
        while prev_rel >= 180.0: prev_rel -= 360.0
        while prev_rel <  -180.0: prev_rel += 360.0

        delta = rel - prev_rel
        if   delta >  180.0: delta -= 360.0
        elif delta < -180.0: delta += 360.0

        self.cum      += delta
        self.last_raw  = raw_deg
        return self.cum

# ====== 全局对象 ======
sense   = None
unwrap  = None
sensors = {}
steer   = None
pair    = None

# ====== 工具函数 ======
def clip(x, lo, hi):
    return hi if x > hi else lo if x < lo else x

# ---- (1) 全局读陀螺仪：返回连续角度（右正左负） ----
def read_yaw_cont():
    """随处可调。程序启动处会置零为0°。"""
    raw = sense.get_orientation_degrees()['yaw']  # 0..360
    return unwrap.update(raw)

def zero_heading_now():
    """把当前朝向设为0°基准。"""
    raw = sense.get_orientation_degrees()['yaw']
    unwrap.reset_zero(raw)

# ---- (2) 距离读取（随方向切换front/back） ----
def read_distance_mm(forward=True):
    """forward=True读front，False读back。"""
    key = "front" if forward else "back"
    return int(sensors[key].range)

# ---- (3) 转向闭环到目标航向 ----
def steer_hold_to_heading(target_deg):
    """
    根据当前连续角度对齐目标角（右正左负）。
    采用P控制 + 死区 + 限幅。返回实际给定的目标转角（电机位置）。
    """
    yaw = read_yaw_cont()
    err = target_deg - yaw
    # 死区：误差很小时不动，避免抖动
    if abs(err) < STEER_DEADBAND:
        cmd = 0.0
    else:
        cmd = clip(-Kp_steer * err, -STEER_LIMIT_DEG, STEER_LIMIT_DEG)
    # 非阻塞设置目标角（电机角度取负号是因为几何/装配方向，与你原程序一致）
    steer.run_to_position(int(-cmd), speed=STEER_SPEED)
    return cmd, yaw, err

# ---- (4) 运动原语 ----
def start_drive(forward=True):
    """按方向起步"""
    if forward:
        pair.start(speedl=SPEED_FWD_LR[0], speedr=SPEED_FWD_LR[1])
    else:
        pair.start(speedl=SPEED_REV_LR[0], speedr=SPEED_REV_LR[1])

def stop_drive(float_steer=True):
    """停车，并将转向电机浮动避免顶住"""
    pair.stop()
    if float_steer:
        try:
            steer.stop(action="float")
        except TypeError:
            steer.float(); steer.stop()

def steer_to_angle(angle_deg, speed=STEER_SPEED):
    """把转向轮打到某个固定角（几何正向=右打），不做航向闭环"""
    angle_deg = clip(angle_deg, -STEER_LIMIT_DEG, STEER_LIMIT_DEG)
    steer.run_to_position(int(angle_deg), speed=speed)

# ---- (5) 段落动作 ----
def segment_forward_until_front(threshold_mm=STOP_GAP_MM):
    """
    段1：保持航向0°前进，直到前方距离<=threshold_mm停车。
    """
    print("[Seg1] zero->forward, stop at front <= %d mm" % threshold_mm)
    start_drive(forward=True)
    while True:
        # 航向保持在0°
        cmd, yaw, err = steer_hold_to_heading(TARGET_FWD_DEG)

        # 前向避障（前进读front）
        d = read_distance_mm(forward=True)
        if d <= threshold_mm:
            time.sleep(DIST_DEBOUNCE)
            if read_distance_mm(forward=True) <= threshold_mm:
                stop_drive()
                print("[Seg1] STOP: front=%d mm, yaw=%.1f, steer_cmd=%.1f" % (d, yaw, cmd))
                break
        # 调试输出（可注释）
        # print("yaw= %7.2f | steer_cmd= %5.1f | front= %4d mm" % (yaw, cmd, d))
        time.sleep(DT)

def segment_reverse_until_heading(target_deg=TARGET_REV_DEG, fixed_steer_deg=TURN_ANGLE_DEG):
    """
    段2：把转向轮打到fixed_steer_deg（例如+30°），后退，
         直到连续角达到target_deg（例如-90°）停车。
    """
    print("[Seg2] steer to %+g°, reverse until heading %.1f°" % (fixed_steer_deg, target_deg))
    steer_to_angle(+fixed_steer_deg, speed=STEER_SPEED)
    time.sleep(0.3)

    start_drive(forward=False)
    while True:
        yaw = read_yaw_cont()

        # 后退时持续读back传感器（便于你必要时调试/扩展）
        d_back = read_distance_mm(forward=False)

        # 角度判据：到达或越过目标角
        if (target_deg <= 0 and yaw <= target_deg) or (target_deg > 0 and yaw >= target_deg):
            stop_drive()
            print("[Seg2] STOP: yaw=%.1f° (target=%.1f°), back=%d mm" % (yaw, target_deg, d_back))
            break

        # 保持固定打角，不做航向闭环（如需带闭环改为：steer_hold_to_heading(target_deg)）
        # 这里每次循环轻触一次位置，避免“跑偏”
        steer_to_angle(+fixed_steer_deg, speed=STEER_SPEED)
        time.sleep(DT)

# ====== 初始化 ======
def init_everything():
    global sense, unwrap, sensors, steer, pair

    i2c = busio.I2C(board.SCL, board.SDA)
    if not assign_addresses(i2c):
        print("address assignment failed")
        try: GPIO.cleanup()
        except: pass
        raise SystemExit(1)

    # init 4 sensors
    for nm, addr in zip(NAMES, ADDRS):
        sensors[nm] = adafruit_vl53l0x.VL53L0X(i2c, address=addr)
        print("init ok:", nm, "@0x%02X" % addr)

    # IMU
    sense = SenseHat()
    # set_imu_config(compass, gyro, accel)
    sense.set_imu_config(USE_COMPASS, True, True)
    time.sleep(0.2)

    # Motors
    print("Init Build HAT motors...")
    steer = Motor('A')                  # 转向
    pair  = MotorPair('B', 'C')         # 行走

    # 转向归中
    steer.run_to_position(0, speed=100)
    time.sleep(0.6)

    # 航向置零（当前朝向=0°）
    global unwrap
    unwrap = YawUnwrapper()
    zero_heading_now()
    print("Heading zeroed.")

# ====== 主程序 ======
def main():
    try:
        init_everything()

        # 段1：0°前进到前方<=250mm
        segment_forward_until_front(STOP_GAP_MM)

        # 段2：打+30°后退到-90°
        segment_reverse_until_heading(TARGET_REV_DEG, TURN_ANGLE_DEG)

    except KeyboardInterrupt:
        print("KeyboardInterrupt -> safe stop")
        try:
            stop_drive(float_steer=True)
        except Exception:
            pass
    finally:
        # 归中 & float & 清GPIO
        try:
            steer.run_to_position(0, speed=100)
            try:
                steer.stop(action="float")
            except TypeError:
                steer.float(); steer.stop()
        except Exception:
            pass
        try: pair.stop()
        except Exception: pass
        try: GPIO.cleanup()
        except Exception: pass

if __name__ == "__main__":
    main()
