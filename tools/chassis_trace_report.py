"""Offline chassis trace CSV analysis; never opens a device or sends commands.

CSV comes from chassis_trace_csv_row, with wheel angles in the internal IK frame.
All velocity reconstruction below is wheel-based, not independent ground truth.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
import math
import re
from pathlib import Path

WHEELS = ("fl", "fr", "rr", "rl")
FIELDS = (
    "sequence,t_ms,length_m,width_m,radius_m,target_vx,target_vy,target_wz,"
    "limited_vx,limited_vy,limited_wz,omega_fl,omega_fr,omega_rr,omega_rl,"
    "angle_fl,angle_fr,angle_rr,angle_rl,yaw_rad,gyro_z_rad_s,imu_age_us,imu_valid,"
    "steer_age_fl,steer_age_fr,steer_age_rr,steer_age_rl,"
    "drive_age_fl,drive_age_fr,drive_age_rr,drive_age_rl,control_dt_ms,"
    "steer_valid_mask,drive_valid_mask,mode,permitted,blocker,fault"
).split(",")
INTEGER_FIELDS = set(FIELDS[:2] + FIELDS[21:])


def dump_metadata(lines: list[str]) -> dict | None:
    meta_pattern = re.compile(r"TEST_TRACE_META (?:dump=([0-9]+) )?expected_count=([0-9]+) first=([0-9]+) next=([0-9]+) overwritten=([0-9]+)(?: reason=([^\s]+))?")
    end_pattern = re.compile(r"TEST_TRACE_END (?:dump=([0-9]+) )?sent_count=([0-9]+)")
    metas = [m for line in lines if (m := meta_pattern.search(line))]
    ends = [m for line in lines if (m := end_pattern.search(line))]
    if not metas and not ends:
        return None
    if len(metas) != 1 or len(ends) != 1:
        raise ValueError("Expected exactly one complete trace dump; isolate a capture session")
    raw = metas[0].groups()
    meta = dict(zip(("dump", "expected_count", "first", "next", "overwritten"), (int(v or 0) for v in raw[:5])))
    meta["stop_reason"] = raw[5] or "UNSPECIFIED"
    end_dump, sent = (int(v or 0) for v in ends[0].groups())
    if meta["dump"] != end_dump or meta["expected_count"] != sent:
        raise ValueError("Trace dump end does not match declared capture")
    if (meta["next"]-meta["first"]) & 0xffffffff != meta["expected_count"]:
        raise ValueError("Invalid capture sequence range")
    return meta


def reassemble_chunks(lines: list[str]) -> list[str]:
    pattern = re.compile(r"TEST_TRACE seq=(HEADER|[0-9]+) part=([0-9]+) total=([0-9]+) data=(.*)$")
    groups = {}
    for line in lines:
        match = pattern.search(line)
        if not match:
            continue
        key, part, total, payload = match.groups()
        part, total = int(part), int(total)
        if not 1 <= part <= total <= 64:
            raise ValueError("Invalid trace chunk index/count")
        group = groups.setdefault(key, {"total": total, "parts": {}})
        if group["total"] != total or (part in group["parts"] and group["parts"][part] != payload):
            raise ValueError(f"Conflicting trace chunks for {key}; isolate one capture/session")
        group["parts"][part] = payload
    if "HEADER" not in groups:
        raise ValueError("Missing chunked trace header")
    rows = []
    for key, group in groups.items():
        if len(group["parts"]) != group["total"]:
            raise ValueError(f"Incomplete trace row {key}: missing chunks; no successful report generated")
        row = "".join(group["parts"][part] for part in range(1, group["total"]+1))
        if key != "HEADER" and row.split(",", 1)[0] != key:
            raise ValueError("Trace chunk sequence does not match row sequence")
        if key == "HEADER":
            header = row
        else:
            rows.append(row)
    return [header] + rows


def read_csv(path: Path) -> list[dict]:
    """Accept raw CSV, TRACE, rows, or bounded TEST_TRACE chunks inside logs.

    Chunk protocol: TEST_TRACE seq=HEADER|<uint32> part=1 total=N data=<text>
    Root exporter uses at most150 payload chars so each log fits its256byte buffer.
    Duplicate identical chunks are okay; missing/conflicting chunks reject the file.
    """
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    metadata = dump_metadata(lines)
    if any("TEST_TRACE seq=" in line for line in lines):
        lines = reassemble_chunks(lines)
    elif any("TRACE," in line for line in lines):
        lines = [line.split("TRACE,", 1)[1] for line in lines if "TRACE," in line]
    reader = csv.DictReader(io.StringIO("\n".join(lines)))
    if reader.fieldnames != FIELDS:
        raise ValueError("Expected chassis_trace CSV v1 header; do not combine unrelated text samples")
    result = []
    for line_number, row in enumerate(reader, 2):
        if None in row or any(v is None for v in row.values()):
            raise ValueError(f"Malformed trace row {line_number}")
        try:
            result.append({key: int(value) if key in INTEGER_FIELDS else float(value)
                           for key, value in row.items()})
        except ValueError as exc:
            raise ValueError(f"Bad trace field at row {line_number}: {exc}") from exc
    if metadata:
        expected = [(metadata["first"]+i) & 0xffffffff for i in range(metadata["expected_count"])]
        if len(result) != len(expected) or {r["sequence"] for r in result} != set(expected):
            raise ValueError("Capture missing whole rows or contains unexpected sequences")
        by_sequence = {r["sequence"]: r for r in result}
        result = [by_sequence[n] for n in expected]
    return result


def observe(row: dict, max_age_ms: int = 100) -> dict:
    issues = []
    numeric = [row[k] for k in FIELDS if k not in INTEGER_FIELDS]
    if not all(math.isfinite(v) for v in numeric):
        issues.append("NONFINITE")
    length, width, radius = (row[k] for k in ("length_m", "width_m", "radius_m"))
    geometry_ok = (all(math.isfinite(v) for v in (length, width, radius)) and
                   .001 <= length <= 100 and .001 <= width <= 100 and .0001 <= radius <= 10)
    if not geometry_ok:
        issues.append("GEOMETRY")
    if row["steer_valid_mask"] != 15 or row["drive_valid_mask"] != 15:
        issues.append("MISSING_WHEEL")
    if any(row[f"{kind}_age_{wheel}"] > max_age_ms or row[f"{kind}_age_{wheel}"] < 0
           for kind in ("steer", "drive") for wheel in WHEELS):
        issues.append("STALE_WHEEL")
    if not row["imu_valid"]:
        issues.append("MISSING_IMU")
    if not 0 <= row["imu_age_us"] <= 20000:
        issues.append("STALE_IMU")
    if not 0 < row["control_dt_ms"] <= 50:
        issues.append("CONTROL_PERIOD")
    if row["blocker"]:
        issues.append("BLOCKED")
    if row["fault"]:
        issues.append("FAULT")
    result = {"issues": issues, "wheel_fit_external": None, "residual_mps": None,
              "residual_max_mps": None, "gyro_wheel_error_rad_s": None}
    # Stale/partial/nonfinite wheels must not be presented as a current measurement.
    if any(k in issues for k in ("NONFINITE", "GEOMETRY", "MISSING_WHEEL", "STALE_WHEEL")):
        return result
    xy = ((length/2, -width/2), (length/2, width/2),
          (-length/2, width/2), (-length/2, -width/2))
    vectors = []
    for wheel in WHEELS:
        speed = row[f"omega_{wheel}"] * radius
        angle = row[f"angle_{wheel}"]
        if abs(speed) > 100 or abs(angle) > 1e6:
            issues.append("WHEEL_RANGE")
            return result
        vectors.append((speed*math.cos(angle), speed*math.sin(angle)))
    vx = sum(v[0] for v in vectors)/4
    vy = sum(v[1] for v in vectors)/4
    wz = sum(-y*u+x*v for (x, y), (u, v) in zip(xy, vectors))/sum(x*x+y*y for x, y in xy)
    residual = [math.hypot(u-(vx-wz*y), v-(vy+wz*x))
                for (x, y), (u, v) in zip(xy, vectors)]
    result.update(wheel_fit_external=[vx, -vy, -wz], residual_mps=residual,
                  residual_max_mps=max(residual), gyro_wheel_error_rad_s=(row["gyro_z_rad_s"]+wz if row["imu_valid"] and 0 <= row["imu_age_us"] <= 20000 else None))
    return result


def analyze_rows(rows: list[dict], source_kind: str, max_age_ms: int = 100) -> dict:
    if source_kind not in ("synthetic", "hardware"):
        raise ValueError("Explicit source_kind must be synthetic or hardware")
    if not rows:
        raise ValueError("No trace samples")
    if max_age_ms < 0:
        raise ValueError("max_age_ms must be nonnegative")
    samples = []
    events = []
    gaps = []
    elapsed_ms = 0
    previous = None
    for row in rows:
        if not 0 <= row["sequence"] <= 0xffffffff or not 0 <= row["t_ms"] <= 0xffffffff:
            raise ValueError("sequence and t_ms must be uint32")
        observation = observe(row, max_age_ms)
        if previous is not None:
            ds = (row["sequence"]-previous["sequence"]) & 0xffffffff
            dt = (row["t_ms"]-previous["t_ms"]) & 0xffffffff
            if ds != 1:
                gaps.append({"sequence": row["sequence"], "sequence_step": ds})
            if dt > 5000:
                observation["issues"].append("TIME_GAP_OR_REORDER")
            elapsed_ms += dt
            changed = [k for k in ("mode", "permitted", "blocker", "fault", "steer_valid_mask", "drive_valid_mask", "imu_valid")
                       if row[k] != previous[k]]
            if changed:
                events.append({"sequence": row["sequence"], "t_ms": row["t_ms"], "changed": changed})
        sample = {"sequence": row["sequence"], "t_ms": row["t_ms"], "elapsed_s": elapsed_ms/1000,
                  "target": [row[k] if math.isfinite(row[k]) else None for k in ("target_vx", "target_vy", "target_wz")],
                  "limited": [row[k] if math.isfinite(row[k]) else None for k in ("limited_vx", "limited_vy", "limited_wz")],
                  **observation}
        samples.append(sample)
        previous = row
    residuals = [s["residual_max_mps"] for s in samples if s["residual_max_mps"] is not None]
    return {"schema": "chassis-trace-report-v1", "source_kind": source_kind, "samples": samples,
            "sample_count": len(samples), "max_age_ms": max_age_ms, "imu_max_age_us": 20000, "events": events,
            "sequence_gaps": gaps, "first_issue": next((s for s in samples if s["issues"]), None),
            "peak_wheel_residual_mps": max(residuals, default=None),
            "limits": ["Wheel fit is diagnostic, not independent ground truth or automatic wheel rejection.",
                       "Coherent all-wheel slip can have zero residual and cannot be identified from wheels alone.",
                       "Snapshot timestamp does not imply simultaneous motor/IMU samples; wheel ages are retained.",
                       "Command slopes do not measure physical acceleration; no stability or stopping-distance certification.",
                       "This bounded capture may only cover a transient; never infer a whole run from it.",
                       "Missing overwritten history cannot be reconstructed from this CSV."]}


def write_svg(report: dict, path: Path) -> None:
    samples = report["samples"]
    end = max(samples[-1]["elapsed_s"], .02)
    values = [v for s in samples for v in (s["limited"][0],
              s["wheel_fit_external"][0] if s["wheel_fit_external"] else None) if v is not None]
    amplitude = max([abs(v) for v in values] + [.1])
    def points(which: str) -> list[str]:
        # Separate paths at invalid measurements rather than joining across gaps.
        paths, active = [], []
        for s in samples:
            v = s["limited"][0] if which == "limited" else (s["wheel_fit_external"][0] if s["wheel_fit_external"] else None)
            if v is None:
                if active: paths.append(" ".join(active)); active=[]
                continue
            active.append(f'{50+880*s["elapsed_s"]/end:.2f},{175-120*v/amplitude:.2f}')
        if active: paths.append(" ".join(active))
        return paths
    lines = ['<svg xmlns="http://www.w3.org/2000/svg" width="980" height="360" viewBox="0 0 980 360">',
             '<rect width="980" height="360" fill="white"/>',
             f'<text x="45" y="24">Chassis trace ({report["source_kind"]}): external vx, m/s</text>',
             '<path d="M50 50V300H930 M50 175H930" fill="none" stroke="#aaa"/>',
             '<text x="50" y="325" fill="#2862ae">Blue: limited command</text>',
             '<text x="350" y="325" fill="#b94c26">Orange: measured-wheel fit (not ground truth)</text>',
             f'<text x="750" y="345">Duration: {end:.3f}s</text>',
             f'<text x="5" y="58">+{amplitude:.2f}</text>', f'<text x="5" y="298">-{amplitude:.2f}</text>']
    for kind, color in (("limited", "#2862ae"), ("wheel", "#b94c26")):
        lines.extend(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>' for pts in points(kind))
    path.write_text("\n".join(lines+["</svg>"]), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--source-kind", required=True, choices=("synthetic", "hardware"))
    parser.add_argument("--max-age-ms", type=int, default=100)
    parser.add_argument("--output", type=Path, required=True, help="Output basename, produces .json and .svg")
    args = parser.parse_args()
    report = analyze_rows(read_csv(args.input), args.source_kind, args.max_age_ms)
    report["input_sha256"] = hashlib.sha256(args.input.read_bytes()).hexdigest()
    report["input_path"] = str(args.input)
    report["capture_metadata"] = dump_metadata(args.input.read_text(encoding="utf-8-sig").splitlines())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    write_svg(report, args.output.with_suffix(".svg"))
    print(f'CHASSIS_TRACE_REPORT_OK samples={report["sample_count"]} source={args.source_kind}')

if __name__ == "__main__":
    main()
