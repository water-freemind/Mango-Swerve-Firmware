"""Offline empty-field route / future tree-coordinate preparation. No device I/O."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape

from competition_route import build_route


def new_profile() -> dict:
    return {
        "schema": "mango-full-map-preparation-v1",
        "mode": "empty-field-debug",
        "field_m": {"width": 3.0, "depth": 4.6, "gate_x": [0.0, 0.4]},
        "start": {"x_m": 0.2, "y_m": -0.5, "yaw_deg": 90.0},
        "start_zone_has_fences": False,
        "physical_ridges_present": False,
        "trees": [],
        "stations": [],
        "work_aisles": {str(i): i for i in range(1, 7)},
        "firmware_tree_upload_supported": False,
        "hardware_full_map_validated": False,
        "note": "树位仅保存在上位机；不自动上传、不改变空场定位、不发送作业或运动指令。",
    }


def finite_number(value: object) -> bool:
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def validate(profile: dict) -> None:
    template = new_profile()
    if not isinstance(profile, dict) or any(profile.get(k) != template[k] for k in (
        "schema", "mode", "field_m", "start", "start_zone_has_fences",
        "physical_ridges_present", "firmware_tree_upload_supported", "hardware_full_map_validated", "work_aisles",
    )):
        raise ValueError("Only the nominal empty field with an unfenced start zone is supported")
    trees = profile.get("trees")
    if not isinstance(trees, list) or len(trees) > 12:
        raise ValueError("trees must be a list of at most 12 calibrated coordinates")
    ids: set[int] = set()
    counts = [0] * 6
    for tree in trees:
        if not isinstance(tree, dict):
            raise ValueError("tree must be an object")
        ident, row = tree.get("id"), tree.get("row")
        if type(ident) is not int or ident not in range(1, 13) or ident in ids:
            raise ValueError("Tree id must be unique and between 1 and 12")
        if type(row) is not int or row not in range(1, 7):
            raise ValueError("Tree row must be between 1 and 6")
        x, y = tree.get("x_m"), tree.get("y_m")
        if not finite_number(x) or not finite_number(y):
            raise ValueError("Tree coordinates must be finite metres")
        x0, x1 = (0.6, 2.4) if row == 4 else (0.4, 2.6)
        y0 = 0.4 + 0.7 * (row - 1)
        if not x0 <= x <= x1 or not y0 - 1e-9 <= y <= y0 + 0.3 + 1e-9:
            raise ValueError("Tree coordinate is outside its nominal ridge footprint")
        ids.add(ident)
        counts[row - 1] += 1
        if counts[row - 1] > 2:
            raise ValueError("At most two tree coordinates per ridge")
    for i, tree in enumerate(trees):
        for other in trees[i + 1:]:
            if math.hypot(tree["x_m"] - other["x_m"], tree["y_m"] - other["y_m"]) < 0.05:
                raise ValueError("Tree centres are duplicated or less than 50mm apart")
    stations = profile.get("stations", [])
    if not isinstance(stations, list) or len(stations) > 12:
        raise ValueError("stations must contain at most 12 separately recorded vehicle poses")
    seen = set()
    station_counts = [0] * 6
    for station in stations:
        if not isinstance(station, dict):
            raise ValueError("station must be an object")
        ident, aisle = station.get("tree_id"), station.get("aisle")
        if type(ident) is not int or ident not in range(1, 13) or ident in seen:
            raise ValueError("Station tree id must be unique and between 1 and 12")
        if type(aisle) is not int or aisle not in range(1, 7) or type(station.get("row")) is not int or station.get("row") != aisle:
            raise ValueError("Only G1..G6 may calibrate R1..R6; G7 is transit only")
        station_counts[aisle - 1] += 1
        if station_counts[aisle - 1] > 2:
            raise ValueError("At most two calibration stations per work aisle")
        pose = station.get("vehicle_pose", {})
        x, y, yaw = (pose.get(k) for k in ("x_m", "y_m", "yaw_deg"))
        if not all(finite_number(v) for v in (x, y, yaw)):
            raise ValueError("Station vehicle pose must be finite")
        if not .2 <= x <= 2.8 or abs(y - (.2 + .7 * (aisle - 1))) > .08 or abs(yaw - 90) > 5:
            raise ValueError("Station must lie near its aisle centre, facing +Y")
        tree = next((t for t in trees if t["id"] == ident), None)
        if tree is not None and tree["row"] != aisle:
            raise ValueError("Tree row and calibration aisle disagree")
        seen.add(ident)


def save(path: Path, profile: dict) -> None:
    validate(profile)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def set_tree(profile: dict, ident: int, row: int, x: float, y: float) -> dict:
    result = json.loads(json.dumps(profile))
    result["trees"] = [t for t in result["trees"] if t["id"] != ident]
    result["trees"].append({"id": ident, "row": row, "x_m": x, "y_m": y, "source": "user-calibrated"})
    result["trees"].sort(key=lambda t: t["id"])
    validate(result)
    return result


def set_station(profile: dict, ident: int, aisle: int, x: float, y: float, yaw: float = 90) -> dict:
    result = json.loads(json.dumps(profile))
    result["stations"] = [s for s in result.get("stations", []) if s["tree_id"] != ident]
    result["stations"].append({"tree_id": ident, "aisle": aisle, "row": aisle,
                               "vehicle_pose": {"x_m": x, "y_m": y, "yaw_deg": yaw}})
    result["stations"].sort(key=lambda s: s["tree_id"])
    validate(result)
    return result


def write_preview(path: Path, profile: dict) -> None:
    validate(profile)
    scale = 100
    def point(x: float, y: float) -> tuple[float, float]:
        return 90 + x * scale, 530 - y * scale
    lines = ['<svg xmlns="http://www.w3.org/2000/svg" width="640" height="720" viewBox="0 0 640 720">',
             '<rect width="640" height="720" fill="white"/>',
             '<g font-family="Microsoft YaHei, sans-serif" font-size="14" fill="#172033">',
             '<text x="30" y="28">全图空场调试：先直达远端，再G7→G1，回门停车</text>',
             '<text x="30" y="52">坐标：米；+X向右，+Y向场内；车头始终朝+Y</text>']
    fence = [(0, 0, 0, 4.6), (0, 4.6, 3, 4.6), (3, 4.6, 3, 0), (0.4, 0, 3, 0)]
    for x0, y0, x1, y1 in fence:
        a, b = point(x0, y0), point(x1, y1)
        lines.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#18243c" stroke-width="4"/>')
    for row in range(1, 7):
        x0, x1 = (0.6, 2.4) if row == 4 else (0.4, 2.6)
        y0 = 0.4 + 0.7 * (row - 1)
        x, y = point(x0, y0 + 0.3)
        lines.append(f'<rect x="{x}" y="{y}" width="{(x1-x0)*scale}" height="30" fill="#f8f2e6" stroke="#b7a789" stroke-dasharray="4 4"/>')
        lines.append(f'<text x="{x+8}" y="{y+20}" fill="#83745c">R{row}（虚拟垄）</text>')
    for index, segment in enumerate(build_route()):
        a, b = point(*segment.start), point(*segment.end)
        color = "#16a368" if index < 2 else "#266bcc"
        lines.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="{color}" stroke-width="2.5"/>')
        lines.append(f'<text x="{(a[0]+b[0])/2+4}" y="{(a[1]+b[1])/2-4}" font-size="10">{index+1}</text>')
    for tree in profile["trees"]:
        x, y = point(tree["x_m"], tree["y_m"])
        lines += [f'<circle cx="{x}" cy="{y}" r="6" fill="#d46b26"/>',
                  f'<text x="{x+8}" y="{y-5}">{escape("T"+str(tree["id"]))}</text>']
    for station in profile.get("stations", []):
        pose = station["vehicle_pose"]
        x, y = point(pose["x_m"], pose["y_m"])
        lines += [f'<rect x="{x-4}" y="{y-4}" width="8" height="8" fill="#ae42a3"/>',
                  f'<text x="{x+7}" y="{y+13}" font-size="10">工位T{station["tree_id"]}</text>']
    x, y = point(0.2, -0.5)
    lines += [f'<circle cx="{x}" cy="{y}" r="5" fill="#df365c"/>',
              '<text x="145" y="585">起点(0.20, -0.50)，无围栏</text>',
              '<text x="30" y="642">灰色虚线垄仅辅助看路线；空场定位不会使用不存在的垄。</text>',
              '<text x="30" y="668">G1→R1 … G6→R6；G7只通行。本工具不上传、不启动电机。</text>',
              '<text x="30" y="694">未做实车全图验收；图中线宽不代表车体扫掠或定位误差。</text>', '</g></svg>']
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=Path("firmware/build/full-map/profile.json"))
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("init")
    sub.add_parser("check")
    tree = sub.add_parser("tree")
    tree.add_argument("--id", type=int, required=True)
    tree.add_argument("--row", type=int, required=True)
    tree.add_argument("--x", type=float, required=True)
    tree.add_argument("--y", type=float, required=True)
    preview = sub.add_parser("preview")
    preview.add_argument("--output", type=Path, default=Path("firmware/build/full-map/preview.svg"))
    station = sub.add_parser("station")
    station.add_argument("--id", type=int, required=True)
    station.add_argument("--aisle", type=int, required=True)
    station.add_argument("--vehicle-x", type=float, required=True)
    station.add_argument("--vehicle-y", type=float, required=True)
    station.add_argument("--yaw", type=float, default=90)
    args = parser.parse_args()
    if args.action == "init":
        if args.profile.exists():
            parser.error("Profile already exists; initialization does not overwrite calibration")
        save(args.profile, new_profile())
    else:
        profile = json.loads(args.profile.read_text(encoding="utf-8"))
        validate(profile)
        if args.action == "tree":
            save(args.profile, set_tree(profile, args.id, args.row, args.x, args.y))
        elif args.action == "station":
            save(args.profile, set_station(profile, args.id, args.aisle, args.vehicle_x, args.vehicle_y, args.yaw))
        elif args.action == "preview":
            write_preview(args.output, profile)
        else:
            print(f"PROFILE_OK calibrated_trees={len(profile['trees'])}/12 hardware_validated=False")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
