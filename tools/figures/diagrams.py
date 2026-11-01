"""Block diagrams: hardware topology, Pico pinout, software architecture,
control loop and vehicle layout / ToF field of view.

Every number drawn here is taken from src/ (pins, addresses, rates, gains).
Things the code cannot tell us (power system, exact sensor positions,
chassis dimensions) are drawn dashed and labelled TODO.
"""
import math

from matplotlib.patches import Circle, FancyBboxPatch, Rectangle, Wedge

from style import (AQUA, BLUE, INK, INK2, MUTED, ORANGE, RULE, SURFACE, VIOLET, arrow, box,
                   canvas, legend_roles, save)


# --------------------------------------------------------------------------
def system_topology():
    fig, ax = canvas(132, 74, "System hardware topology",
                     "Signal paths of the 2026 vehicle. Interfaces, addresses and rates come from src/.")
    box(ax, 0, 52, 26, 13, "BNO085 IMU", ["UART-RVC, 115200 baud", "Pico GP0 / GP1", "continuous yaw"], "sensor")
    box(ax, 0, 31, 26, 17, "6 × ToF distance", ["F: VL53L4CX", "R1 R2 B L2 L1: VL53L4CD", "I2C1 400 kHz, 0x2A–0x2F",
                                                 "XSHUT GP16–GP21"], "sensor")
    box(ax, 0, 14, 26, 13, "TCS34725 colour", ["I2C0 100 kHz @ 0x29", "Pico GP4 / GP5", "LED on GP6"], "sensor")
    box(ax, 34, 30, 28, 21, "Raspberry Pi Pico 2", ["RP2350 · MicroPython", "core 0: USB, IMU, colour", "core 1: ToF round-robin",
                                                     "20 Hz labelled frames"], "compute")
    box(ax, 34, 10, 28, 12, "Power distribution", ["TODO: battery, regulators,", "switch and fuse"], "todo", dashed=True)
    box(ax, 70, 28, 30, 25, "Raspberry Pi 5", ["DonkeyCar CNN @ 15 Hz", "UnifiedRobotIO @ 20 Hz", "motion primitives @ 20 Hz",
                                                "Python 3 · venv"], "compute", title_size=11)
    box(ax, 70, 59, 30, 11, "Camera (CSI)", ["320 × 240 → 160 × 120", "exposure / AWB locked"], "sensor")
    box(ax, 70, 8, 30, 12, "Sense HAT", ["joystick: CW/CCW confirm", "8 × 8 LED: status"], "ui")
    box(ax, 108, 35, 24, 11, "ROBOTIS U2D2", ["USB ↔ TTL, 57600 bps"], "actuator")
    box(ax, 108, 55, 24, 15, "XL330-M288-T  ID 1", ["drive · velocity mode", "1 : 3 to the wheels", "odometry 4096 /rev"], "actuator")
    box(ax, 108, 12, 24, 15, "XL330-M288-T  ID 2", ["steering · position mode", "centre 2016, ±40°"], "actuator")

    for y0, y1 in ((58.5, 46), (39.5, 40.5), (20.5, 35)):
        arrow(ax, (26, y0), (34, y1), color=AQUA)
    arrow(ax, (62, 40.5), (70, 40.5), "USB CDC", both=True, color=BLUE)
    arrow(ax, (85, 59), (85, 53), "CSI", color=AQUA, label_offset=(4.5, -1.2))
    arrow(ax, (85, 20), (85, 28), "40-pin header", both=True, color=VIOLET, label_offset=(7.5, -1.2))
    arrow(ax, (100, 40.5), (108, 40.5), "USB", color=ORANGE)
    arrow(ax, (120, 46), (120, 55), "TTL bus", both=True, color=ORANGE, label_offset=(5, -1.2))
    arrow(ax, (120, 35), (120, 27), "Protocol 2.0", both=True, color=ORANGE, label_offset=(6.5, -1.2))
    legend_roles(ax, 0, 1, ["compute", "sensor", "actuator", "ui", "todo"],
                 ["compute", "sensing", "actuation", "user interface", "to be documented"])
    save(fig, "system_topology")


# --------------------------------------------------------------------------
PICO_LEFT = ["GP0", "GP1", "GND", "GP2", "GP3", "GP4", "GP5", "GND", "GP6", "GP7",
             "GP8", "GP9", "GND", "GP10", "GP11", "GP12", "GP13", "GND", "GP14", "GP15"]
PICO_RIGHT = ["VBUS", "VSYS", "GND", "3V3_EN", "3V3(OUT)", "ADC_VREF", "GP28", "GND", "GP27", "GP26",
              "RUN", "GP22", "GND", "GP21", "GP20", "GP19", "GP18", "GND", "GP17", "GP16"]

LEFT_USE = {
    "GP0": ("UART0 TX (configured, unused by RVC)", MUTED),
    "GP1": ("UART0 RX  ←  BNO085 TX · 115200 RVC", AQUA),
    "GP2": ("I2C1 SDA  ↔  ToF bus (6 sensors)", BLUE),
    "GP3": ("I2C1 SCL  ↔  ToF bus · 400 kHz", BLUE),
    "GP4": ("I2C0 SDA  ↔  TCS34725 @ 0x29", ORANGE),
    "GP5": ("I2C0 SCL  ↔  TCS34725 · 100 kHz", ORANGE),
    "GP6": ("Colour-sensor LED enable (high)", ORANGE),
}
RIGHT_USE = {
    "GP21": ("XSHUT R1 · VL53L4CD · 0x2B", BLUE),
    "GP20": ("XSHUT F  · VL53L4CX · 0x2A", BLUE),
    "GP19": ("XSHUT B  · VL53L4CD · 0x2D", BLUE),
    "GP18": ("XSHUT R2 · VL53L4CD · 0x2C", BLUE),
    "GP17": ("XSHUT L1 · VL53L4CD · 0x2F", BLUE),
    "GP16": ("XSHUT L2 · VL53L4CD · 0x2E", BLUE),
}


def pico_pinout():
    fig, ax = canvas(132, 64, "Pico 2 sensor-hub wiring",
                     "Signal pins used by src/pico2 (main.py, sensor_hub_tof.py). Sensor 3V3/GND supply rails: TODO verify on the car.")
    bx, bw, top, step = 56, 20, 56, 2.55
    ax.add_patch(FancyBboxPatch((bx, top - 20 * step - 1.2), bw, 20 * step + 3.4,
                                boxstyle="round,pad=0,rounding_size=1.2", fc="#2f6b3a", ec="#1e4a27", lw=1.5))
    ax.add_patch(Rectangle((bx + bw / 2 - 3, top + 1.0), 6, 2.6, fc="#c9c9c4", ec=INK2, lw=1))
    ax.text(bx + bw / 2, top + 4.5, "USB  →  Raspberry Pi 5  (/dev/ttyACM0)", ha="center", va="bottom",
            fontsize=8.5, color=INK, fontweight="bold")
    ax.text(bx + bw / 2, top - 10 * step, "Raspberry Pi\nPico 2\n(RP2350)", ha="center", va="center",
            fontsize=9.5, color="white", fontweight="bold")
    for i in range(20):
        y = top - i * step
        for side, names, use in ((0, PICO_LEFT, LEFT_USE), (1, PICO_RIGHT, RIGHT_USE)):
            name = names[i]
            px = bx if side == 0 else bx + bw
            used = name in use
            colour = use[name][1] if used else "#9fb8a3"
            ax.add_patch(Circle((px, y), 0.62, fc=colour if used else "#d8e2d9", ec="#1e4a27", lw=0.8, zorder=3))
            pin_no = i + 1 if side == 0 else 40 - i
            ax.text(px + (1.3 if side == 0 else -1.3), y, f"{name}", ha="left" if side == 0 else "right",
                    va="center", fontsize=6.6, color="white" if used else "#d8e2d9", zorder=4,
                    fontweight="bold" if used else "normal")
            ax.text(px + (-1.4 if side == 0 else 1.4), y, str(pin_no), ha="right" if side == 0 else "left",
                    va="center", fontsize=6, color=MUTED)
            if used:
                label, c = use[name]
                x_end = bx - 7 if side == 0 else bx + bw + 7
                ax.plot([px - 3.4 if side == 0 else px + 3.4, x_end], [y, y], color=c, lw=1.6, zorder=2)
                ax.text(x_end + (-0.8 if side == 0 else 0.8), y, label, ha="right" if side == 0 else "left",
                        va="center", fontsize=7.8, color=INK, bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec=c, lw=1))
    # device boxes
    box(ax, 0, 14, 30, 10, "ToF bus (I2C1)", ["F, R1, R2, B, L2, L1 share SDA/SCL;", "one XSHUT line each"], "sensor", line_size=7.5)
    box(ax, 0, 2, 30, 10, "Start-up addressing", ["all XSHUT low → wake one sensor at", "0x29 → move to 0x2A … 0x2F"], "neutral", line_size=7.5)
    box(ax, 100, 38, 32, 12, "Output frame (20 Hz)", ["degree, F, R1, R2, B, L2, L1,", "C1R, C1G, C1B, C1C"], "compute", line_size=7.5)
    save(fig, "pico_wiring_pinout")


# --------------------------------------------------------------------------
def software_architecture():
    fig, ax = canvas(132, 80, "Software architecture and threads",
                     "Who owns which resource. Arrows are data flow; the main thread never touches a serial port.")
    # (x, y, w, h, name, label position) - labels sit where no arrow crosses
    lanes = [(0, 58, 132, 18, "Raspberry Pi 5 · main thread", (130.8, 75.4, "right", "top")),
             (0, 34, 132, 21, "Raspberry Pi 5 · UnifiedRobotIO thread (20 Hz)", (68, 54.4, "left", "top")),
             (0, 2, 93, 28, "Raspberry Pi Pico 2 (MicroPython, dual-core)", (25, 29.4, "left", "top")),
             (95, 2, 37, 28, "USB peripheral of the Pi 5", (96.2, 2.8, "left", "bottom"))]
    for x, y, w, h, name, (lx, ly, ha, va) in lanes:
        ax.add_patch(Rectangle((x, y), w, h, fc="#f6f6f4", ec=RULE, lw=1))
        ax.text(lx, ly, name, fontsize=8.5, color=INK2, va=va, ha=ha, fontweight="bold")
    box(ax, 4, 62, 30, 10, "Run script", ["AFI_CUP_ORun / FRun", "challenge sequence"], "software")
    box(ax, 40, 62, 50, 10, "CarController primitives (20 Hz loops)",
        ["gyro_run · gyro_turn · gyro_run_tof · motor_run", "gyro_single_tof_wall_run · calibrate_gyro_from_side_wall"], "software")
    box(ax, 96, 62, 32, 10, "ai_run (15 Hz)", ["camera → 160×120 → KerasLinear", "→ steering, throttle × limit"], "software")
    box(ax, 4, 38, 34, 13, "Pico frame parser", ["latest complete line only", "+ heading offset / scale",
                                                  "+ gyro reset request"], "actuator", line_size=7.6)
    box(ax, 44, 38, 40, 13, "Target mailbox → DXL writes", ["steering goal position (ID 2)", "drive goal velocity (ID 1)",
                                                             "latest value, no queue"], "actuator", line_size=7.6)
    box(ax, 90, 38, 38, 13, "Odometry", ["Present Position (ID 1)", "→ wheel-equivalent degrees"], "actuator", line_size=7.6)
    box(ax, 4, 6, 40, 18, "Core 0 (main.py)", ["USB serial owner + command parser", "BNO085 RVC decode, yaw unwrap",
                                               "TCS34725 read", "print frame every 50 ms"], "sensor", line_size=7.6)
    box(ax, 52, 6, 38, 18, "Core 1 (sensor_hub_tof.py)", ["owns I2C1 + XSHUT", "6 ToF strict round-robin",
                                                          "40 ms budget each (~3.5 Hz)", "publishes via lock"], "sensor", line_size=7.6)
    box(ax, 99, 6, 29, 18, "U2D2 → DYNAMIXEL", ["ID 1 drive", "ID 2 steering", "TTL, Protocol 2.0"], "neutral", line_size=7.6)
    arrow(ax, (34, 67), (40, 67), color=BLUE)
    arrow(ax, (90, 67), (96, 67), color=BLUE)
    arrow(ax, (21, 51), (21, 62), "get_sensor_data()", color=ORANGE, label_offset=(0, 1.0))
    arrow(ax, (64, 62), (64, 51), "set_steering / set_throttle", color=BLUE, label_offset=(0, 1.0))
    arrow(ax, (109, 51), (109, 62), "get_wheel_degrees()", color=ORANGE, label_offset=(0, 1.0))
    arrow(ax, (21, 24), (21, 38), "USB CDC frames 20 Hz", color=AQUA, label_offset=(0, 2.0))
    arrow(ax, (44, 15), (52, 15), color=AQUA, both=True)
    ax.text(48, 17.2, "lock", ha="center", fontsize=7.4, color=INK2)
    arrow(ax, (64, 38), (104, 24), "TTL bus", color=ORANGE, rad=0.15, label_offset=(10, -0.5))
    arrow(ax, (110, 24), (110, 38), color=ORANGE)
    save(fig, "software_architecture")


# --------------------------------------------------------------------------
def control_loop():
    fig, ax = canvas(132, 44, "Cascade controller of gyro_single_tof_wall_run",
                     "Outer loop: side-ToF wall distance shifts the heading target. Inner loop: heading P-control drives the steering motor.")

    def summer(x, y, second="−", from_top=False):
        """Summing junction: '+' for the left input, `second` beside the top/bottom input."""
        ax.add_patch(Circle((x, y), 2.0, fc=SURFACE, ec=INK, lw=1.4, zorder=3))
        ax.text(x - 3.4, y + 0.7, "+", fontsize=9, color=INK2)
        ax.text(x + 1.0, y + 2.4 if from_top else y - 4.4, second, fontsize=9, color=INK2)

    y = 24
    ax.text(0, y + 1.4, "d*", fontsize=10, fontweight="bold", va="bottom")
    ax.text(0, y - 1.8, "wall target", fontsize=7.4, color=INK2, va="top")
    arrow(ax, (5, y), (12, y))
    summer(14, y, "−", from_top=True)
    arrow(ax, (16, y), (22, y), "e_d", label_offset=(0, 0.6))
    box(ax, 22, y - 6, 22, 12, "Wall gain g(|e|)", ["deadband 0.7 cm", "1.5 / 2.5 / 4.0 °/cm", "clamp ±22°"], "software", line_size=7.2)
    arrow(ax, (44, y), (50, y), "Δψ", label_offset=(0, 0.6))
    summer(52, y, "+", from_top=True)
    ax.text(52, y + 9.5, "ψ_ref (lap heading, e.g. −1080°)", ha="center", fontsize=7.4, color=INK2)
    arrow(ax, (52, y + 9), (52, y + 2))
    arrow(ax, (54, y), (59, y), "ψ*", label_offset=(0, 0.6))
    summer(61, y, "−")
    arrow(ax, (63, y), (68, y), "e_ψ", label_offset=(0, 0.6))
    box(ax, 68, y - 6, 18, 12, "Heading P", ["u = k_p · e_ψ", "k_p = 0.03", "clamp ±1, × sign(v)"], "software", line_size=7.2)
    arrow(ax, (86, y), (91, y), "u", label_offset=(0, 0.6))
    box(ax, 91, y - 6, 18, 12, "Steering motor", ["× 1.1, ±40°", "DXL ID 2", "position mode"], "actuator", line_size=7.2)
    arrow(ax, (109, y), (114, y))
    box(ax, 114, y - 6, 18, 12, "Vehicle", ["kinematics", "(bicycle model)"], "neutral", line_size=7.2)
    # feedback paths
    ax.plot([123, 123, 61, 61], [y - 6, 6, 6, y - 2], color=AQUA, lw=1.4)
    ax.text(92, 6.6, "ψ  ←  BNO085 → Pico (unwrap) → Pi (sign, scale, wall offset)", ha="center", fontsize=7.6, color=INK2,
            bbox=dict(boxstyle="round,pad=0.15", fc=SURFACE, ec="none"))
    ax.plot([127, 127, 14, 14], [y + 6, y + 14, y + 14, y + 2], color=AQUA, lw=1.4)
    ax.text(86, y + 14.6, "d  ←  trailing side ToF (R2 / L2 forward) → EMA α = 0.5, hold 0.5 s on dropout",
            ha="center", fontsize=7.6, color=INK2, bbox=dict(boxstyle="round,pad=0.15", fc=SURFACE, ec="none"))
    arrow(ax, (61, 7), (61, y - 2), color=AQUA)
    arrow(ax, (14, y + 13), (14, y + 2), color=AQUA)
    save(fig, "control_cascade")


# --------------------------------------------------------------------------
def vehicle_layout():
    """Top view: module placement and ToF fields of view (schematic)."""
    fig, ax = canvas(132, 112, "Sensor layout and ToF field of view (top view, schematic)",
                     "Sensor names, roles and the 14.8 cm side-pair spacing are from src/. Body size and exact mounts: TODO measure. Not to scale.")
    cx, cy = 66, 58
    L, W = 34, 20   # schematic body (no real dimensions yet)
    ax.add_patch(FancyBboxPatch((cx - W / 2, cy - L / 2), W, L, boxstyle="round,pad=0,rounding_size=3",
                                fc="#f2f2f0", ec=INK2, lw=1.6, zorder=2))
    for sx in (-1, 1):
        for sy, c in ((1, ORANGE), (-1, ORANGE)):
            ax.add_patch(Rectangle((cx + sx * (W / 2) - (1.6 if sx > 0 else 2.4) + (0.8 if sx > 0 else 0),
                                    cy + sy * (L / 2 - 6) - 3), 2.4, 6, fc="#3b3b38", ec="none", zorder=1))
    ax.text(cx, cy + L / 2 - 4.2, "steering\n(DXL ID 2)", ha="center", va="center", fontsize=6.8, color=INK2, zorder=3)
    ax.text(cx, cy - L / 2 + 4.2, "drive (DXL ID 1)\n1 : 3 gear-up", ha="center", va="center", fontsize=6.8, color=INK2, zorder=3)
    ax.text(cx, cy + 3, "Pi 5 + Pico 2", ha="center", va="center", fontsize=7.6, color=INK, fontweight="bold", zorder=3)
    ax.text(cx, cy - 1.4, "BNO085 · camera", ha="center", va="center", fontsize=6.8, color=INK2, zorder=3)
    ax.text(cx - W / 2 - 13, cy + L / 2 + 1, "▲ forward", ha="center", va="center", fontsize=8.5,
            color=INK, fontweight="bold")

    half = 9.0          # VL53L4CD / VL53L4CX: 18° full field of view (datasheet)
    rng = 24.0          # drawn cone length (schematic; firmware gate is 2..120 cm)
    pair = 14.8 / 34 * L  # side pair spacing drawn in proportion to the body
    sensors = [("F", cx, cy + L / 2, 90, "VL53L4CX"), ("B", cx, cy - L / 2, 270, "VL53L4CD"),
               ("R1", cx + W / 2, cy + pair / 2, 0, "right-front"), ("R2", cx + W / 2, cy - pair / 2, 0, "right-rear"),
               ("L1", cx - W / 2, cy + pair / 2, 180, "left-front"), ("L2", cx - W / 2, cy - pair / 2, 180, "left-rear")]
    for name, x, y, ang, sub in sensors:
        ax.add_patch(Wedge((x, y), rng, ang - half, ang + half, fc=BLUE, alpha=0.13, ec=BLUE, lw=0.8, zorder=1))
        ax.add_patch(Circle((x, y), 1.1, fc=BLUE, ec="white", lw=1.0, zorder=4))
        tx = x + (rng + 3.5) * math.cos(math.radians(ang))
        ty = y + (rng + 3.5) * math.sin(math.radians(ang))
        ax.text(tx, ty, f"{name}\n{sub}", ha="center", va="center", fontsize=7.6, color=INK, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec=RULE))
    # spacing dimension on the right side
    xd = cx + W / 2 + 3.2
    ax.annotate("", xy=(xd, cy + pair / 2), xytext=(xd, cy - pair / 2),
                arrowprops=dict(arrowstyle="<->", color=INK2, lw=1))
    ax.text(xd + 0.8, cy, "d = 14.8 cm", fontsize=7.4, color=INK2, va="center", rotation=90)
    ax.text(0, 4, "Roles in gyro_single_tof_wall_run: the trailing side sensor holds the wall distance,\n"
                  "the leading one watches for the parking-lot wall; R1/R2 or L1/L2 together give the wall angle.",
            fontsize=7.8, color=INK2, va="bottom")
    ax.text(132, 4, "Cones: 18° FoV (datasheet), length schematic.\nValid range used in code: 2–120 cm.",
            fontsize=7.8, color=INK2, va="bottom", ha="right")
    save(fig, "sensor_layout_fov")


ALL = [system_topology, pico_pinout, software_architecture, control_loop, vehicle_layout]
