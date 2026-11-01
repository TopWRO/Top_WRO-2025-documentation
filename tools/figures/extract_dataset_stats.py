#!/usr/bin/env python3
"""Summarise DonkeyCar tub data into a small JSON file for the documentation.

The raw tubs (~204k frames, 1.7 GB of JPEGs) are never committed. This script
reads only the *.catalog / manifest.json files of every tub folder, and
writes docs/data/dataset_summary.json with:
  * records per tub (deleted indexes excluded) and per challenge/direction
  * records per recording date (from DonkeyCar session ids, e.g. 26-08-02_0)
  * 41-bin histograms of user/angle and user/throttle per group

Usage:
    python3 tools/figures/extract_dataset_stats.py <dir-containing-data_*> [out.json]
"""
import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

BINS = 41  # steering / throttle in [-1, 1] -> 0.05-wide bins


def group_of(name):
    """data_TSP_Ocw05 -> ('Obstacle', 'CW'); data_Fccw01 -> ('Open', 'CCW')."""
    n = name.lower().replace("data_", "").replace("tsp_", "")
    m = re.match(r"([fo]?)(ccw|cw)", n)
    if not m:
        return None
    challenge = {"f": "Open", "o": "Obstacle", "": "Unlabelled"}[m.group(1)]
    return challenge, m.group(2).upper()


def bin_index(v):
    v = max(-1.0, min(1.0, float(v)))
    return min(BINS - 1, int(round((v + 1.0) / 2.0 * (BINS - 1))))


def read_tub(tub):
    manifest = (tub / "manifest.json").read_text().splitlines()
    meta = json.loads(manifest[-1]) if manifest else {}
    deleted = set(meta.get("deleted_indexes", []))
    for cat in sorted(tub.glob("catalog_*.catalog")):
        for line in cat.read_text(errors="ignore").splitlines():
            if not line.strip():
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if rec.get("_index") in deleted:
                continue
            yield rec


def main():
    root = Path(sys.argv[1]).expanduser()
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else \
        Path(__file__).resolve().parents[2] / "docs" / "data" / "dataset_summary.json"

    tubs = {}
    groups = defaultdict(lambda: {"records": 0, "tubs": 0,
                                  "angle_hist": [0] * BINS, "throttle_hist": [0] * BINS})
    by_date = defaultdict(Counter)
    for tub in sorted(p for p in root.glob("data_*") if p.is_dir()):
        g = group_of(tub.name)
        if g is None:
            continue
        key = f"{g[0]} {g[1]}"
        n = 0
        for rec in read_tub(tub):
            n += 1
            groups[key]["angle_hist"][bin_index(rec.get("user/angle", 0.0))] += 1
            groups[key]["throttle_hist"][bin_index(rec.get("user/throttle", 0.0))] += 1
            sid = str(rec.get("_session_id", ""))
            m = re.match(r"(\d\d)-(\d\d)-(\d\d)", sid)
            if m:
                by_date[f"20{m.group(1)}-{m.group(2)}-{m.group(3)}"][key] += 1
        tubs[tub.name] = {"group": key, "records": n}
        groups[key]["records"] += n
        groups[key]["tubs"] += 1

    summary = {
        "source": "TopWRO/wro-fe-2026 data_* tubs (catalog files only)",
        "bins": {"count": BINS, "min": -1.0, "max": 1.0},
        "total_records": sum(t["records"] for t in tubs.values()),
        "groups": dict(sorted(groups.items())),
        "by_date": {d: dict(c) for d, c in sorted(by_date.items())},
        "tubs": tubs,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=1))
    print(f"{summary['total_records']} records in {len(tubs)} tubs -> {out}")
    for k, v in summary["groups"].items():
        print(f"  {k:16s} {v['records']:7d} records, {v['tubs']} tubs")


if __name__ == "__main__":
    main()
