"""Data and math figures. Inputs: constants copied from src/ and the JSON
summaries in docs/data/ (real recorded data and real git history)."""
import json
import math
from collections import Counter
from datetime import date, timedelta

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from style import AQUA, BLUE, INK, INK2, MUTED, ORANGE, REPO, RULE, SURFACE, YELLOW, save

DATA = REPO / "docs" / "data"


# --------------------------------------------------------------------------
# gyro_single_tof_wall_run constants (src/pi5/competition/AFI_CUP_V05.py)
DEADBAND, MID, FAR, CLAMP = 0.7, 2.0, 8.0, 22.0
GAINS = {"forward (speed > 0)": (1.5, 2.5, 4.0), "reverse (speed < 0)": (0.8, 1.0, 1.5)}


def wall_correction(e, gains):
    a = abs(e)
    if a <= DEADBAND:
        return 0.0
    g = gains[2] if a >= FAR else gains[1] if a >= MID else gains[0]
    return max(-CLAMP, min(CLAMP, e * g))


def gain_schedule():
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    e = np.linspace(-15, 15, 3001)
    for (name, gains), c in zip(GAINS.items(), (BLUE, ORANGE)):
        ax.plot(e, [wall_correction(x, gains) for x in e], color=c, lw=2, label=name)
    ax.axvspan(-DEADBAND, DEADBAND, color="#ecebe8", lw=0)
    for x in (MID, FAR):
        ax.axvline(x, color=RULE, lw=1, ls="--")
        ax.axvline(-x, color=RULE, lw=1, ls="--")
    ax.axhline(CLAMP, color=MUTED, lw=1, ls=":")
    ax.axhline(-CLAMP, color=MUTED, lw=1, ls=":")
    ax.text(14.8, CLAMP + 0.8, "clamp ±22°", ha="right", fontsize=8.5, color=INK2)
    ax.text(0, -26.5, "deadband\n±0.7 cm", ha="center", fontsize=8, color=INK2)
    ax.text(MID + 0.15, -26.5, "2 cm", fontsize=8, color=INK2)
    ax.text(FAR + 0.15, -26.5, "8 cm", fontsize=8, color=INK2)
    ax.set_ylim(-28, 28)
    ax.set_xlabel("wall-distance error  e = d* − d   [cm]")
    ax.set_ylabel("heading-target correction  Δψ  [°]")
    ax.set_title("Gain-scheduled wall correction (gyro_single_tof_wall_run)")
    ax.legend(loc="upper left")
    ax.text(15, -18, "then × side_sign × motion_sign", ha="right", fontsize=8, color=INK2)
    save(fig, "wall_gain_schedule")


# --------------------------------------------------------------------------
def wall_angle_geometry():
    """Two parallel side rays separated by d along the body: R2 − R1 = d · tan θ."""
    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.set_aspect("equal")
    ax.axis("off")
    wall_x = 10.0
    ax.add_patch(Rectangle((wall_x, -3), 1.2, 30, fc="#3b3b38", ec="none"))
    ax.text(wall_x + 1.8, 25.5, "straight wall", fontsize=9, color=INK2, va="top")
    th = math.radians(12)        # illustrative body yaw relative to the wall
    d = 14.8
    c = np.array([-6.0, 11.0])   # body centre
    u = np.array([math.sin(th), math.cos(th)])       # body forward axis
    n = np.array([math.cos(th), -math.sin(th)])      # right-hand normal (ray direction)
    L, W = 26, 13
    corners = [c + sx * W / 2 * n + sy * L / 2 * u for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    ax.add_patch(Polygon(corners, closed=True, fc="#f2f2f0", ec=INK2, lw=1.6))
    p1 = c + W / 2 * n + d / 2 * u   # R1 (front)
    p2 = c + W / 2 * n - d / 2 * u   # R2 (rear)
    for p, name in ((p1, "R1"), (p2, "R2")):
        t = (wall_x - p[0]) / n[0]
        q = p + t * n
        ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.8))
        ax.plot(*p, "o", color=BLUE, ms=7, mec="white")
        ax.text(*(p + t / 2 * n + np.array([0, 0.9])), f"{name}", color=INK, fontsize=10,
                fontweight="bold", ha="center")
    ax.annotate("", xy=p1 - 1.2 * n, xytext=p2 - 1.2 * n, arrowprops=dict(arrowstyle="<->", color=INK2))
    ax.text(*(c + (W / 2 - 3.2) * n + np.array([-0.6, 0])), "d = 14.8 cm", rotation=math.degrees(-th) + 90,
            fontsize=8.5, color=INK2, ha="center", va="center")
    ax.annotate("", xy=c + 9 * u, xytext=c, arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.4))
    ax.plot([c[0], c[0]], [c[1], c[1] + 9], color=MUTED, lw=1, ls="--")
    ax.text(c[0] + 0.3, c[1] + 10.2, "θ", fontsize=12, color=INK)
    ax.text(-44, 26, "Right wall:\n  θ = atan2(R2 − R1, d)\n\nLeft wall:\n  θ = atan2(L1 − L2, d)",
            fontsize=10.5, color=INK, va="top", family="monospace")
    ax.text(-44, 4, "Both rays are perpendicular\nto the body, so their lengths\ndiffer by exactly d·tan θ,\n"
                    "independent of the distance\nto the wall.", fontsize=8.6, color=INK2, va="bottom")
    ax.set_xlim(-45, 16)
    ax.set_ylim(-3, 27)
    ax.set_title("Wall angle from a same-side ToF pair", loc="left")
    save(fig, "wall_angle_geometry")


# --------------------------------------------------------------------------
def tof_timing():
    """Round-robin schedule of sensor_hub_tof.ToFController with the default 40 ms budget."""
    budget, read, guard = 40, 1, 4          # ms (budget, approx. read time, SWITCH_GUARD_MS)
    names = ["F", "R1", "R2", "B", "L2", "L1"]
    fig, ax = plt.subplots(figsize=(9.2, 3.8))
    t, cycle = 0.0, budget + read + guard
    while t < 600:
        for i, n in enumerate(names):
            if t >= 600:
                break
            ax.add_patch(Rectangle((t, 6 - i - 0.35), budget, 0.7, fc=BLUE, ec="none"))
            ax.add_patch(Rectangle((t + budget, 6 - i - 0.35), read, 0.7, fc=ORANGE, ec="none"))
            t += cycle
    for k in range(0, 601, 50):
        ax.plot([k, k], [-0.4, 0.4], color=AQUA, lw=2)
    ax.set_yticks([6 - i for i in range(6)] + [0], names + ["USB out"])
    ax.set_xlim(0, 600)
    ax.set_ylim(-0.8, 6.8)
    ax.set_xlabel("time [ms]")
    ax.grid(axis="y", visible=False)
    ax.set_title("Pico 2 timing: ToF round-robin (core 1) vs. 20 Hz output (core 0)", pad=22)
    ax.text(0, 1.02, f"≈ {cycle} ms per sensor → {6 * cycle} ms per round → ≈ {1000 / (6 * cycle):.1f} Hz per ToF "
            "(model of the scheduler; measure on the car with the `stats` command)",
            transform=ax.transAxes, ha="left", va="bottom", fontsize=8.5, color=INK2)
    h = [Rectangle((0, 0), 1, 1, fc=c) for c in (BLUE, ORANGE, AQUA)]
    ax.legend(h, ["ranging (timing budget 40 ms)", "read + clear (≈1 ms) then 4 ms guard",
                  "frame printed every 50 ms (latest values)"],
              loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3, fontsize=8)
    save(fig, "pico_tof_timing")


# --------------------------------------------------------------------------
def _dataset():
    return json.loads((DATA / "dataset_summary.json").read_text())


def dataset_composition():
    d = _dataset()
    order = ["Obstacle CCW", "Obstacle CW", "Open CCW", "Open CW", "Unlabelled CW"]
    vals = [d["groups"][k]["records"] for k in order]
    tubs = [d["groups"][k]["tubs"] for k in order]
    fig, ax = plt.subplots(figsize=(8.2, 3.6))
    y = np.arange(len(order))[::-1]
    ax.barh(y, vals, height=0.62, color=BLUE)
    for yi, v, t in zip(y, vals, tubs):
        ax.text(v + 1200, yi, f"{v:,}  ({t} tub{'s' if t > 1 else ''})", va="center", fontsize=9, color=INK)
    ax.set_yticks(y, order)
    ax.set_xlim(0, max(vals) * 1.25)
    ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v / 1000:.0f}k"))
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("recorded frames (deleted frames excluded)")
    ax.set_title(f"Training data: {d['total_records']:,} frames recorded with Recording_N02.py")
    save(fig, "dataset_composition")


def steering_throttle_distribution():
    d = _dataset()
    n = d["bins"]["count"]
    centres = np.linspace(-1, 1, n)
    fig, axes = plt.subplots(2, 2, figsize=(9.2, 5.6), sharex=True)
    for col, ch in enumerate(("Open", "Obstacle")):
        for row, key, lab in ((0, "angle_hist", "steering  user/angle"), (1, "throttle_hist", "throttle  user/throttle")):
            ax = axes[row][col]
            for off, (direction, c) in zip((-0.013, 0.013), (("CW", BLUE), ("CCW", ORANGE))):
                g = d["groups"].get(f"{ch} {direction}")
                h = np.array(g[key], float)
                pct = 100 * h / h.sum()
                m = pct > 0
                ax.bar(centres[m] + off, pct[m], width=0.024, bottom=0.01, color=c, label=direction)
            if row == 0:
                ax.set_title(f"{ch} Challenge", fontsize=11)
            if col == 0:
                ax.set_ylabel(f"{lab}\n% of frames", fontsize=8.5)
            ax.set_yscale("log")
            ax.set_ylim(0.01, 100)
    axes[1][0].set_xlabel("command value")
    axes[1][1].set_xlabel("command value")
    axes[0][1].legend(loc="upper right")
    fig.suptitle("Driver command distributions in the training data (log scale)", x=0.06, ha="left",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    save(fig, "dataset_command_distribution")


def recording_sessions():
    d = _dataset()
    dates = list(d["by_date"])
    order = ["Obstacle CW", "Obstacle CCW", "Open CW", "Open CCW", "Unlabelled CW"]
    colours = [BLUE, ORANGE, AQUA, YELLOW, MUTED]
    fig, ax = plt.subplots(figsize=(8.2, 3.8))
    bottom = np.zeros(len(dates))
    for k, c in zip(order, colours):
        v = np.array([d["by_date"][x].get(k, 0) for x in dates], float)
        ax.bar(dates, v, bottom=bottom, color=c, width=0.6, label=k, edgecolor=SURFACE, linewidth=1.5)
        bottom += v
    for i, tot in enumerate(bottom):
        ax.text(i, tot + 1500, f"{int(tot):,}", ha="center", fontsize=9, color=INK)
    ax.set_ylim(0, bottom.max() * 1.15)
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v / 1000:.0f}k"))
    ax.grid(axis="x", visible=False)
    ax.set_ylabel("frames")
    ax.set_title("Recording sessions (DonkeyCar session dates)")
    ax.legend(ncol=3, fontsize=8, loc="upper left")
    save(fig, "dataset_sessions")


# --------------------------------------------------------------------------
def dev_timeline():
    hist = json.loads((DATA / "dev_history.json").read_text())
    fig, ax = plt.subplots(figsize=(9.6, 4.0))
    start, end = date(2025, 6, 1), date(2026, 12, 10)
    labels = {"Top_WRO-2025-documentation": ("2025 documentation repo", BLUE),
              "wro-fe-2026": ("2026 development repo", ORANGE)}
    weeks = {}
    for repo, commits in hist.items():
        c = Counter()
        for cm in commits:
            dd = date.fromisoformat(cm["date"])
            c[dd - timedelta(days=dd.weekday())] += 1
        weeks[repo] = c
    for i, (repo, (lab, col)) in enumerate(labels.items()):
        c = weeks.get(repo, {})
        xs = sorted(c)
        ax.bar([x + timedelta(days=1 + 2.5 * i) for x in xs], [c[x] for x in xs], width=2.5, color=col, label=lab)
    events = [(date(2025, 7, 15), "US National\nChampionship (Jul 2025)"),
              (date(2025, 11, 15), "World Finals,\nSingapore (Nov 2025)"),
              (date(2026, 7, 31), "training data\nrecorded (Jul 29–Aug 3)"),
              (date(2026, 10, 11), "wro-fe-2026 repo\ncreated (Oct 11)")]
    ymax = max(max(v.values()) for v in weeks.values()) * 1.6
    for x, t in events:
        ax.axvline(x, color=MUTED, lw=1, ls="--", zorder=0)
        ax.text(x, ymax * 0.97, t, ha="center", va="top", fontsize=7.8, color=INK2,
                bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec="none"))
    ax.set_xlim(start, end)
    ax.set_ylim(0, ymax)
    ax.xaxis.set_major_locator(mdates.MonthLocator(bymonth=(1, 3, 5, 7, 9, 11)))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%b %Y"))
    ax.grid(axis="x", visible=False)
    ax.set_ylabel("commits per week")
    ax.set_title("Development activity from git history")
    ax.legend(loc="upper left", bbox_to_anchor=(0, -0.12), ncol=2, fontsize=8.5)
    save(fig, "dev_timeline")


ALL = [gain_schedule, wall_angle_geometry, tof_timing, dataset_composition,
       steering_throttle_distribution, recording_sessions, dev_timeline]
