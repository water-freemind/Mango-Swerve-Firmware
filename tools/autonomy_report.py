"""Offline reports from shared-C traces, and measured-vs-command ramp analysis.

No device access. Reports replay recorded samples; they do not reimplement the
controller. Python 3.10+, standard library only. See PROJECT_GUIDE 9.16.
"""
from __future__ import annotations
import argparse
import csv
from collections import defaultdict
import hashlib
import html
import json
import math
from pathlib import Path
import re

from competition_route import field_geometry


def number(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"nonfinite sample: {value}")
    return result


def read_trace(path: Path) -> list[dict]:
    integers = "t_ms segment kind aisle coverage actions qualified range_valid opening stop_mask".split()
    floats = "true_x true_y pose_x pose_y yaw cmd_vx cmd_vy cmd_wz measured_vx measured_vy measured_wz progress cross_error".split()
    rows = []
    with path.open(encoding="utf-8-sig", newline="") as handle:
        for raw in csv.DictReader(handle):
            row = {k: int(raw[k]) for k in integers}
            row.update({k: number(raw[k]) for k in floats})
            row.update({k: raw[k] for k in ("state", "first_fault")})
            if row["t_ms"] <= (rows[-1]["t_ms"] if rows else 0):
                raise ValueError("trace times must increase from simulation t=0")
            if not 0 <= row["segment"] < 17 or not 0 <= row["coverage"] <= 127:
                raise ValueError("invalid route index or coverage")
            if row["state"] == "FAULT" and any(row[k] != 0 for k in ("cmd_vx", "cmd_vy", "cmd_wz")):
                raise ValueError("fault row contains a motion command")
            rows.append(row)
    if not rows:
        raise ValueError("empty trace")
    return rows


def summarize_trace(rows: list[dict]) -> dict:
    states, segments = defaultdict(int), defaultdict(int)
    transitions, coverage = [], []
    previous_t, previous_key, previous_coverage = 0, None, 0
    first_fault = None
    for row in rows:
        dt = row["t_ms"] - previous_t
        states[row["state"]] += dt
        segments[row["segment"]] += dt
        key = (row["state"], row["segment"])
        if key != previous_key:
            transitions.append({k: row[k] for k in ("t_ms", "state", "segment", "aisle", "coverage", "actions")})
        if row["coverage"] != previous_coverage:
            if row["coverage"] & previous_coverage != previous_coverage:
                raise ValueError("coverage bits disappeared")
            added = row["coverage"] & ~previous_coverage
            coverage.append({"t_ms": row["t_ms"], "added_aisles": [i + 1 for i in range(7) if added & (1 << i)]})
        if row["first_fault"] != "OK" and first_fault is None:
            first_fault = {k: row[k] for k in ("t_ms", "state", "segment", "first_fault")}
        previous_t, previous_key, previous_coverage = row["t_ms"], key, row["coverage"]
    last = rows[-1]
    return {
        "simulation_only": True, "deployment_ready": False, "samples": len(rows),
        "elapsed_s": last["t_ms"] / 1000, "terminal": last["state"],
        "coverage_mask": last["coverage"], "actions": last["actions"],
        "local_qualified": bool(last["qualified"]), "first_fault": first_fault,
        "state_seconds": {k: v / 1000 for k, v in states.items()},
        "segment_seconds": {str(k): v / 1000 for k, v in segments.items()},
        "transitions": transitions, "coverage_events": coverage,
        "peak_pose_disagreement_m": max(math.hypot(r["true_x"] - r["pose_x"], r["true_y"] - r["pose_y"]) for r in rows),
        "peak_command_mps": max(math.hypot(r["cmd_vx"], r["cmd_vy"]) for r in rows),
        "opening_sample_count": sum(r["opening"] != 0 for r in rows),
        "time_attribution": "Each 10ms fixture tick is attributed to its executed segment and returned state; transitions have one-tick resolution.",
    }


def trace_svg(rows: list[dict], title: str, scale: float = 1) -> str:
    # Background follows fixture metadata, not an inferred real field survey.
    def point(x, y):
        return 55 + 115 * x / scale, 610 - 100 * (y / scale + 1)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 470 660">',
             '<rect width="470" height="660" fill="white"/>',
             f'<text x="20" y="22" font-size="16">{html.escape(title)}</text>',
             '<text x="20" y="42" font-size="12">SIMULATION ONLY - map X right, Y up</text>']
    geo = field_geometry(scale)
    for rect in [geo["field"], geo["start_area"], *geo["rows"]]:
        x, y = point(rect.xmin, rect.ymax)
        color = "#b98951" if rect.name.startswith("R") else "#f5f7fa"
        parts.append(f'<rect x="{x}" y="{y}" width="{(rect.xmax-rect.xmin)*115/scale}" height="{(rect.ymax-rect.ymin)*100/scale}" fill="{color}" stroke="#555"/>')
    for name, color in (("true", "#16784e"), ("pose", "#be3969")):
        points = " ".join(f"{point(r[name+'_x'],r[name+'_y'])[0]:.2f},{point(r[name+'_x'],r[name+'_y'])[1]:.2f}" for r in rows[::10] + [rows[-1]])
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.4" opacity=".75"/>')
    x, y = point(rows[-1]["true_x"], rows[-1]["true_y"])
    parts += [f'<circle cx="{x}" cy="{y}" r="5" fill="#c22"/>',
              '<text x="20" y="640" font-size="12">Green: plant, pink: estimated pose, red: end</text>', '</svg>']
    return "\n".join(parts)


def mission_reports(directory: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    reports = {}
    fixture = json.loads((directory / 'fixture.json').read_text(encoding='utf-8'))
    if fixture.get('simulation_only') is not True or not .5 <= number(str(fixture['map_scale'])) <= 2:
        raise ValueError('not a supported simulation fixture')
    with (directory / "scenarios.csv").open(encoding="utf-8-sig", newline="") as handle:
        scenarios = list(csv.DictReader(handle))
    for scenario in scenarios:
        name = scenario["scenario"]
        if not re.fullmatch(r"[a-z0-9-]+", name):
            raise ValueError("invalid scenario filename")
        path = directory / (name + ".csv")
        rows = read_trace(path)
        report = summarize_trace(rows)
        if report["terminal"] != scenario["terminal"] or rows[-1]["first_fault"] != scenario["first_fault"]:
            raise ValueError("scenario summary and trace disagree")
        report["trace_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        report["work_fault_code"] = int(scenario["work_fault"])
        report["fixture"] = fixture
        reports[name] = report
        (output / (name + ".json")).write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
        (output / (name + ".svg")).write_text(trace_svg(rows, name, fixture['map_scale']), encoding="utf-8")
    index = {"simulation_only": True, "deployment_ready": False, "scenarios": reports}
    (output / "index.json").write_text(json.dumps(index, indent=2, allow_nan=False), encoding="utf-8")
    return index


RAMP = re.compile(r"\bTEST_RAMP\s+t_ms=(\d+)\s+target=([^ ]+)\s+output=([^ ]+)\s+reversing=([01])(?:\s|$)")
AXES = ("vx", "vy", "wz")


def ramp_records(path: Path) -> tuple[list[dict], list[str]]:
    warnings, result = [], []
    if path.suffix.lower() == ".csv":
        with path.open(encoding="utf-8-sig", newline="") as handle:
            for raw in csv.DictReader(handle):
                row = {"t_ms": int(raw["t_ms"]), "reversing": int(raw.get("reversing", "0"))}
                for group in ("target", "output", "measured"):
                    vals = [raw.get(f"{group}_{a}", "") for a in AXES]
                    if group == "measured" and not any(vals):
                        row[group] = None
                    else:
                        row[group] = tuple(number(v) for v in vals)
                result.append(row)
    else:
        for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
            if "cached" in line.lower():
                continue  # Snapshot replay is not a new sample.
            if "TEST_RAMP" not in line:
                continue
            match = RAMP.search(line)
            if not match:
                warnings.append("incomplete TEST_RAMP line ignored")
                continue
            t, target, output, reversing = match.groups()
            vectors = [tuple(number(v) for v in value.split(",")) for value in (target, output)]
            if any(len(v) != 3 for v in vectors):
                raise ValueError("TEST_RAMP needs three components per vector")
            result.append(dict(t_ms=int(t), target=vectors[0], output=vectors[1], measured=None, reversing=int(reversing)))
    if len(result) < 2:
        raise ValueError("at least two non-cached ramp samples are required")
    return result, warnings


def ramp_report(records: list[dict], warnings: list[str]) -> dict:
    command_accel, command_angular, actual_accel, actual_angular, gaps, stops = [], [], [], [], [], []
    sessions, duplicates, pending_stop = 1, 0, None
    for before, after in zip(records, records[1:]):
        if after["t_ms"] < before["t_ms"]:
            sessions += 1; pending_stop = None; continue
        if after["t_ms"] == before["t_ms"]:
            duplicates += 1; continue
        dt = (after["t_ms"] - before["t_ms"]) / 1000
        gaps.append(dt)
        for group, linear, angular in (("output", command_accel, command_angular), ("measured", actual_accel, actual_angular)):
            if before[group] is not None and after[group] is not None:
                dv = [(b-a)/dt for a,b in zip(before[group], after[group])]
                linear.append(math.hypot(*dv[:2])); angular.append(abs(dv[2]))
        zero = lambda v: v is not None and math.hypot(*v[:2]) <= .005 and abs(v[2]) <= .01
        if zero(after["target"]) and not zero(before["target"]):
            pending_stop = dict(start_ms=after["t_ms"], command_stop_ms=None, measured_stop_ms=None)
            stops.append(pending_stop)
        if pending_stop is not None:
            if not zero(after["target"]):
                pending_stop = None
            else:
                if pending_stop["command_stop_ms"] is None and zero(after["output"]):
                    pending_stop["command_stop_ms"] = after["t_ms"]
                if pending_stop["measured_stop_ms"] is None and zero(after["measured"]):
                    pending_stop["measured_stop_ms"] = after["t_ms"]
    if not actual_accel:
        warnings.append("No timestamped measured chassis velocity. Actual acceleration/stopping distance and anti-tip performance are UNKNOWN.")
    if gaps and max(gaps) > .1:
        warnings.append("Sparse samples: finite differences are interval averages and can miss peaks; stop times are sample-bounded.")
    return {
        "hardware_validation": False, "samples": len(records), "sessions": sessions,
        "duplicate_timestamps": duplicates, "max_gap_s": max(gaps, default=None),
        "peak_command_average_accel_mps2": max(command_accel, default=None),
        "peak_command_average_angular_accel_radps2": max(command_angular, default=None),
        "peak_measured_average_accel_mps2": max(actual_accel, default=None),
        "peak_measured_average_angular_accel_radps2": max(actual_angular, default=None),
        "stops": stops, "warnings": warnings,
        "note": "Command/feedback derivatives are not IMU acceleration. One near-zero feedback sample is not a certified stable stop. No physical braking distance is inferred.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    mission = sub.add_parser("mission")
    mission.add_argument("traces", type=Path); mission.add_argument("--output", required=True, type=Path)
    ramp = sub.add_parser("ramp")
    ramp.add_argument("input", type=Path); ramp.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.command == "mission":
        result = mission_reports(args.traces, args.output)
        print(f"MISSION_REPORT_OK scenarios={len(result['scenarios'])} simulation_only=true")
    else:
        records, warnings = ramp_records(args.input)
        result = ramp_report(records, warnings)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        print(f"RAMP_REPORT_OK samples={len(records)} actual_feedback={result['peak_measured_average_accel_mps2'] is not None}")


if __name__ == "__main__":
    main()
