"""Shared look for every generated figure (palette, fonts, box/arrow helpers).

Palette = the validated reference categorical order (blue, orange, aqua,
yellow, magenta, green, violet, red). Text uses neutral ink, never series
colours. Every figure is saved on an explicit light surface so it reads the
same in GitHub light and dark themes.
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
FIG_DIR = REPO / "docs" / "assets" / "figures"
SCHEME_DIR = REPO / "schemes"

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#8a8984"
RULE = "#d9d8d4"

BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
MAGENTA, GREEN, VIOLET, RED = "#e87ba4", "#008300", "#6250d6", "#e34948"
SERIES = [BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED]

# role -> (border, tint fill) for block diagrams
ROLE = {
    "compute": (BLUE, "#e8f1fc"),
    "sensor": (AQUA, "#e4f6ef"),
    "actuator": (ORANGE, "#fdeee7"),
    "power": (YELLOW, "#fdf3dc"),
    "ui": (VIOLET, "#eeecfb"),
    "software": (BLUE, "#e8f1fc"),
    "neutral": (MUTED, "#f2f2f0"),
    "todo": (RED, "#fdecec"),
}

plt.rcParams.update({
    "font.family": ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 10,
    "text.color": INK,
    "axes.edgecolor": RULE,
    "axes.labelcolor": INK2,
    "axes.titlecolor": INK,
    "axes.titlesize": 12,
    "axes.titleweight": "bold",
    "axes.titlelocation": "left",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "axes.axisbelow": True,
    "grid.color": "#ecebe8",
    "grid.linewidth": 0.8,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "legend.frameon": False,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "svg.fonttype": "none",
    "svg.hashsalt": "topwro-2026",   # deterministic SVG ids -> clean git diffs
})


def save(fig, name, folder=FIG_DIR, svg=True):
    folder.mkdir(parents=True, exist_ok=True)
    fig.savefig(folder / f"{name}.png", dpi=200, bbox_inches="tight", pad_inches=0.25,
                metadata={"Software": None})
    if svg:
        fig.savefig(folder / f"{name}.svg", bbox_inches="tight", pad_inches=0.25, metadata={"Date": None})
    plt.close(fig)
    print("  wrote", (folder / name).relative_to(REPO), "(.png" + (", .svg)" if svg else ")"))


def canvas(w, h, title=None, subtitle=None):
    """Blank diagram canvas in data units (0..w, 0..h)."""
    fig, ax = plt.subplots(figsize=(w / 10, h / 10))
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.axis("off")
    ax.set_aspect("equal")
    if title:
        ax.text(0, h + 3.2, title, fontsize=14, fontweight="bold", va="bottom")
    if subtitle:
        ax.text(0, h + 0.8, subtitle, fontsize=9.5, color=INK2, va="bottom")
    return fig, ax


def box(ax, x, y, w, h, title, lines=(), role="neutral", title_size=10, line_size=8.2,
        dashed=False, align="center"):
    """Rounded node with a bold title and optional detail lines. (x, y) = lower-left."""
    edge, fill = ROLE[role]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.2",
                                linewidth=1.6, edgecolor=edge, facecolor=fill,
                                linestyle="--" if dashed else "-", zorder=2))
    cx = x + w / 2 if align == "center" else x + 1.5
    ha = "center" if align == "center" else "left"
    n = len(lines)
    top = y + h / 2 + (n * 1.55) / 2 + 0.2
    ax.text(cx, top if n else y + h / 2, title, ha=ha, va="center" if not n else "top",
            fontsize=title_size, fontweight="bold", color=INK, zorder=3)
    for i, ln in enumerate(lines):
        ax.text(cx, top - 2.3 - i * 1.75, ln, ha=ha, va="top", fontsize=line_size,
                color=INK2, zorder=3)
    return (x, y, w, h)


def arrow(ax, p, q, label=None, color=INK2, both=False, style="-|>", rad=0.0, lw=1.4,
          label_offset=(0, 0.9), label_size=7.8, dashed=False, label_ha="center"):
    a = FancyArrowPatch(p, q, arrowstyle=("<|-|>" if both else style), mutation_scale=11,
                        color=color, linewidth=lw, connectionstyle=f"arc3,rad={rad}",
                        linestyle="--" if dashed else "-", zorder=1, shrinkA=0, shrinkB=0)
    ax.add_patch(a)
    if label:
        mx, my = (p[0] + q[0]) / 2 + label_offset[0], (p[1] + q[1]) / 2 + label_offset[1]
        ax.text(mx, my, label, ha=label_ha, va="bottom", fontsize=label_size, color=INK2,
                zorder=4, bbox=dict(boxstyle="round,pad=0.15", fc=SURFACE, ec="none"))


def legend_roles(ax, x, y, roles, labels):
    for i, (r, lab) in enumerate(zip(roles, labels)):
        edge, fill = ROLE[r]
        ax.add_patch(FancyBboxPatch((x + i * 16, y), 2.6, 2.0, boxstyle="round,pad=0,rounding_size=0.5",
                                    linewidth=1.4, edgecolor=edge, facecolor=fill))
        ax.text(x + i * 16 + 3.6, y + 1.0, lab, va="center", fontsize=8.5, color=INK2)
