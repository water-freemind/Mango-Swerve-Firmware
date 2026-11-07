"""Summarize the synthetic CSV emitted by test_clearance_replay.c.

Uses recorded canonical C results; does not implement a second braking model.
No device access. Standard library, Python 3.10+.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path


def summarize(path: Path) -> dict:
    groups: dict[tuple[str, int], list[dict]] = defaultdict(list)
    numeric = ("edge_offset_m", "clearance_m", "required_m", "sample_age_s",
               "response_budget_s", "speed_mps", "wz_rad_s")
    for row in csv.DictReader(path.read_text(encoding="utf-8-sig").splitlines()):
        parsed = dict(row)
        for key in ("channel", "t_ms", "raw_mm", "protective_stop"):
            parsed[key] = int(row[key])
        if parsed["channel"] not in range(4) or parsed["protective_stop"] not in (0, 1):
            raise ValueError("Invalid channel or stop flag")
        for key in numeric:
            parsed[key] = float(row[key])
            if not math.isfinite(parsed[key]):
                raise ValueError(f"Nonfinite {key}")
        truth = float(row["ground_truth_gap_m"])
        parsed["ground_truth_gap_m"] = truth if math.isfinite(truth) else None
        key = (row["scenario"], parsed["channel"])
        if groups[key] and parsed["t_ms"] <= groups[key][-1]["t_ms"]:
            raise ValueError(f"Non-increasing scenario time: {key}")
        groups[key].append(parsed)
    if not groups:
        raise ValueError("Empty clearance replay")
    scenes = []
    for (name, channel), rows in groups.items():
        valid = [r for r in rows if r["result"] == "OK"]
        truth = [r["ground_truth_gap_m"] for r in rows if r["ground_truth_gap_m"] is not None]
        first = next((r for r in rows if r["first_reason"] != "NONE"), None)
        scenes.append({
            "scenario": name, "channel": channel, "samples": len(rows),
            "first_reason": first["first_reason"] if first else None,
            "first_reason_t_ms": first["t_ms"] if first else None,
            "result_counts": {k: sum(r["result"] == k for r in rows) for k in sorted({r["result"] for r in rows})},
            "min_ground_truth_gap_m": min(truth) if truth else None,
            "min_valid_clearance_m": min((r["clearance_m"] for r in valid), default=None),
            "max_required_m": max((r["required_m"] for r in valid), default=None),
            "max_observed_sample_age_s": max((r["sample_age_s"] for r in valid), default=None),
            "max_response_budget_s": max((r["response_budget_s"] for r in valid), default=None),
        })
    return {
        "simulation_only": True,
        "producer": "firmware/tests/test_clearance_replay.c",
        "source": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "model": "canonical auto_laser_clearance; max observed sample age plus additional delay",
        "caveats": [
            "Synthetic mounts, envelopes, velocity and deceleration are not hardware calibration.",
            "Ground truth is fixture data, not measured vehicle position; null means not supplied.",
            "Rotation samples are not continuous sweep validation or measured angular braking.",
            "S21C receipt timestamps do not prove individual optical sample ages.",
            "Invalid geometry outputs are excluded from clearance/budget minima and maxima.",
            "Two-dimensional bottom rays cannot establish high-body obstacle coverage.",
        ],
        "scenarios": scenes,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.csv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"CLEARANCE_REPORT_OK scenarios={len(result['scenarios'])} simulation_only=1 output={args.output}")


if __name__ == "__main__":
    main()
