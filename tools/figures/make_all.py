#!/usr/bin/env python3
"""Regenerate every documentation figure.

    pip install matplotlib numpy pillow
    python3 tools/figures/extract_dataset_stats.py <path to wro-fe-2026>   # optional: refresh data
    python3 tools/figures/extract_git_history.py ../wro-fe-2026 ../Top_WRO-2025-documentation
    python3 tools/figures/make_all.py

Figures go to docs/assets/figures/. Electromechanical diagrams are also copied
to schemes/ (WRO folder convention).
"""
import shutil

import diagrams
import placeholders
import plots
from style import FIG_DIR, SCHEME_DIR

SCHEMES = ["system_topology", "pico_wiring_pinout", "sensor_layout_fov", "control_cascade"]

if __name__ == "__main__":
    for f in diagrams.ALL + plots.ALL:
        f()
    placeholders.main()
    SCHEME_DIR.mkdir(exist_ok=True)
    for name in SCHEMES:
        for ext in (".png", ".svg"):
            shutil.copy2(FIG_DIR / f"{name}{ext}", SCHEME_DIR / f"{name}{ext}")
    print("  copied", len(SCHEMES), "diagrams to schemes/")
