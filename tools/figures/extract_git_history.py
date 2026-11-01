#!/usr/bin/env python3
"""Collect commit dates of our repositories into docs/data/dev_history.json.

Usage:
    python3 tools/figures/extract_git_history.py <repo> [<repo> ...]
Each repo is labelled by its folder name. Only commit DATES are stored (no
messages, authors or hashes), so private repositories leak nothing.
"""
import json
import subprocess
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parents[2] / "docs" / "data" / "dev_history.json"


def history(repo):
    out = subprocess.run(["git", "-C", str(repo), "log", "--format=%ad", "--date=short"],
                         capture_output=True, text=True, check=True).stdout
    return [{"date": ln} for ln in out.splitlines() if ln]


def main():
    data = {Path(r).resolve().name: history(r) for r in sys.argv[1:]}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1, ensure_ascii=False))
    for k, v in data.items():
        print(f"{k}: {len(v)} commits ({v[-1]['date']} .. {v[0]['date']})")


if __name__ == "__main__":
    main()
