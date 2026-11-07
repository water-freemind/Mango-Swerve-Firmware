"""Compare real C command generators; no hardware or measured-motion claim.

Run after compiling firmware/tests/manual_ramp_trace.c twice (candidate/legacy).
Only Python standard library is required. See report metadata for limitations.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import html
import json
import math
from pathlib import Path


def load(path):
    groups = {}
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            name = row.pop("scenario")
            r = {key: float(value) for key, value in row.items()}
            if not all(math.isfinite(v) for v in r.values()) or r["dt_s"] <= 0:
                raise ValueError("Non-finite sample or invalid dt")
            samples = groups.setdefault(name, [])
            if samples and r["t_s"] <= samples[-1]["t_s"]:
                raise ValueError("Non-monotonic time")
            prior = samples[-1] if samples else None
            r["speed"] = math.hypot(r["cmd_vx"], r["cmd_vy"])
            r["demand"] = r["speed"] * abs(r["cmd_wz"])
            continuous = (bool(r["drive_allowed"]) and not r["protective_reset"] and
                          (not prior or prior["drive_allowed"] and not prior["protective_reset"]))
            if continuous:
                r["ax"] = (r["cmd_vx"] - (prior["cmd_vx"] if prior else 0)) / r["dt_s"]
                r["ay"] = (r["cmd_vy"] - (prior["cmd_vy"] if prior else 0)) / r["dt_s"]
                r["aw"] = (r["cmd_wz"] - (prior["cmd_wz"] if prior else 0)) / r["dt_s"]
                r["a"] = math.hypot(r["ax"], r["ay"])
            else:
                r.update(ax=None, ay=None, aw=None, a=None)
            if continuous and prior and prior["a"] is not None:
                interval = .5 * (r["dt_s"] + prior["dt_s"])
                r["j"] = math.hypot(r["ax"] - prior["ax"], r["ay"] - prior["ay"]) / interval
                r["jw"] = abs(r["aw"] - prior["aw"]) / interval
            else:
                r["j"] = r["jw"] = None
            samples.append(r)
    return groups


def metrics(samples):
    peak = lambda key: max((abs(r[key]) for r in samples if r[key] is not None), default=0)
    result = {"peak_cmd_speed_mps": peak("speed"), "peak_cmd_wz_rad_s": peak("cmd_wz"),
              "peak_cmd_accel_mps2": peak("a"), "peak_cmd_angular_accel_rad_s2": peak("aw"),
              "peak_cmd_jerk_mps3": peak("j"), "peak_cmd_angular_jerk_rad_s3": peak("jw"),
              "peak_cmd_lateral_demand_mps2": peak("demand"),
              "final_cmd": [samples[-1][k] for k in ("cmd_vx", "cmd_vy", "cmd_wz")]}
    stop_start = None
    distance = 0.0
    for i, r in enumerate(samples):
        stopped_request = all(r[k] == 0 for k in ("input_vx", "input_vy", "input_wz"))
        if stop_start is None and i and stopped_request and any(samples[i-1][k] != 0 for k in ("input_vx", "input_vy", "input_wz")):
            stop_start = r["t_s"] - r["dt_s"]
        if stop_start is not None:
            # Trapezoidal integral of commanded speed, not measured stop distance.
            distance += .5 * (r["speed"] + samples[i-1]["speed"]) * r["dt_s"]
            if r["speed"] == 0 and r["cmd_wz"] == 0:
                result.update(normal_stop_time_s=r["t_s"] - stop_start,
                              normal_stop_command_distance_m=distance)
                break
    initial = samples[0]
    for r in samples:
        if any(abs(r[k]-initial[k]) > .00001 for k in ("input_vx", "input_vy", "input_wz")):
            break
        if all(abs(r["cmd_" + axis] - r["limited_" + axis]) < .001 for axis in ("vx", "vy", "wz")):
            result["initial_target_reached_s"] = r["t_s"]
            break
    return result


def plot(name, legacy, candidate, path):
    width, height = 1050, 920
    result = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
              '<rect width="100%" height="100%" fill="white"/>',
              '<g font-family="sans-serif" fill="#222">',
              f'<text x="45" y="28" font-size="20">{html.escape(name)} — command-only comparison</text>',
              '<text x="45" y="50">Blue: S-curve candidate; orange: legacy slew. Not measured vehicle motion.</text>']
    duration = max(legacy[-1]["t_s"], candidate[-1]["t_s"])
    for panel, (key, title) in enumerate((("speed", "Translation command (m/s)"), ("cmd_wz", "Yaw-rate command (rad/s)"),
                                        ("a", "Command acceleration magnitude (m/s^2)"), ("j", "Command jerk magnitude (m/s^3)"))):
        top, bottom = 90 + panel * 198, 240 + panel * 198
        vals = [r[key] for rows in (legacy, candidate) for r in rows if r[key] is not None]
        lo, hi = min(0, min(vals)), max(.001, max(vals))
        hi += (hi-lo)*.05
        result += [f'<text x="45" y="{top-12}">{title}; range {lo:.3g}..{hi:.3g}</text>',
                   f'<path d="M60,{top} V{bottom} H1010" fill="none" stroke="#aaa"/>']
        for rows, color in ((legacy, "#ce6e20"), (candidate, "#196dc7")):
            # Keep all points: decimation would hide short jerk peaks.
            sections, points = [], []
            for r in rows:
                if r[key] is None:
                    if points: sections.append(points); points=[]
                else:
                    points.append(f'{60+950*r["t_s"]/duration:.2f},{bottom-150*(r[key]-lo)/(hi-lo):.2f}')
            if points: sections.append(points)
            for points in sections:
                result.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="1.3"/>')
        result += [f'<text x="60" y="{bottom+18}">0 s</text>', f'<text x="940" y="{bottom+18}">{duration:.1f} s</text>']
    result += ['<text x="45" y="905">Gate-close / protective-reset discontinuities excluded from smooth-motion derivatives.</text>', '</g></svg>']
    Path(path).write_text("\n".join(result), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    old, new = load(args.legacy), load(args.candidate)
    if set(old) != set(new): raise ValueError("Scenario mismatch")
    args.out.mkdir(parents=True, exist_ok=True)
    report = {"kind": "command_generator_comparison", "legacy_source": "1a4345b manual_ramp.c frozen test fixture",
              "inputs": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.legacy, args.candidate)},
              "limitations": ["Command curves only; feedback stop is synthetic and wheel tracking/slip/CG are not simulated.",
                              "Target coupling budget 0.30 m/s^2 is not an output invariant; spin-to-translation transient may exceed it.",
                              "Smooth derivative metrics exclude steering-gate and protective-reset transitions.",
                              "Stop distance is integrated command, not a physical stopping-distance guarantee."],
              "scenarios": {}}
    for name in old:
        report["scenarios"][name] = {"legacy": metrics(old[name]), "candidate": metrics(new[name])}
        plot(name, old[name], new[name], args.out / f"{name}.svg")
    (args.out / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    for name, pair in report["scenarios"].items():
        print(name, json.dumps(pair, ensure_ascii=False))


if __name__ == "__main__":
    main()
