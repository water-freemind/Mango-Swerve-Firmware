"""Offline competition route, continuous footprint checks, and time scenarios.

Python 3.10+, standard library only. This is NOT a controller or a hardware
qualification. All body, tracking, acceleration, and tree-stop values are
unmeasured assumptions. World +X is right; world +Y points into the field.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


EPS = 1e-9
Point = tuple[float, float]


@dataclass(frozen=True)
class Config:
    map_scale: float = 1.0
    body_width_m: float = 0.27
    body_length_m: float = 0.27
    yaw_deg: float = 90.0
    yaw_error_deg: float = 5.0
    tracking_error_m: float = 0.02
    safety_margin_m: float = 0.01
    speed_m_s: float = 0.30
    accel_m_s2: float = 0.10
    decel_m_s2: float = 0.10
    net_tree_dwell_s: float = 8.0
    startup_s: float = 8.0
    segment_settle_s: float = 1.0
    reserve_s: float = 30.0
    deadline_s: float = 300.0

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        for name in ("map_scale", "body_width_m", "body_length_m", "speed_m_s",
                     "accel_m_s2", "decel_m_s2", "deadline_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("yaw_error_deg", "tracking_error_m", "safety_margin_m",
                     "net_tree_dwell_s", "startup_s", "segment_settle_s", "reserve_s"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative")


@dataclass(frozen=True)
class Rect:
    name: str
    xmin: float
    ymin: float
    xmax: float
    ymax: float


@dataclass(frozen=True)
class Segment:
    name: str
    start: Point
    end: Point
    phase: str
    ditch: int | None = None
    north_row: int | None = None

    @property
    def length_m(self) -> float:
        return math.dist(self.start, self.end)


def field_geometry(scale: float = 1.0) -> dict:
    """Uniform scale is ONE tolerance scenario, not all independent +/-5% cases."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("scale must be finite and positive")
    rows = []
    for row in range(1, 7):
        x0, x1 = (0.6, 2.4) if row == 4 else (0.4, 2.6)
        y = 0.4 + 0.7 * (row - 1)
        rows.append(Rect(f"R{row}", x0 * scale, y * scale,
                         x1 * scale, (y + 0.3) * scale))
    return {
        "field": Rect("field", 0, 0, 3.0 * scale, 4.6 * scale),
        "start_area": Rect("start", 0, -scale, 0.4 * scale, 0),
        "rows": rows,
        "gate_x_m": [0, 0.4 * scale],
        "ditch_centers_y_m": [(0.2 + 0.7 * i) * scale for i in range(7)],
        "side_centers_x_m": [0.2 * scale, 2.8 * scale],
    }


def build_route(scale: float = 1.0) -> list[Segment]:
    geo = field_geometry(scale)
    left, right = geo["side_centers_x_m"]
    ys = geo["ditch_centers_y_m"]
    parking = (left, -0.5 * scale)
    segments = [Segment("enter", parking, (left, ys[0]), "enter"),
                Segment("outbound_left", (left, ys[0]), (left, ys[-1]), "outbound")]
    for ditch in range(7, 0, -1):
        y = ys[ditch - 1]
        x0, x1 = (left, right) if ditch % 2 else (right, left)
        segments.append(Segment(f"G{ditch}", (x0, y), (x1, y), "survey",
                                ditch, ditch if ditch <= 6 else None))
        if ditch > 1:
            segments.append(Segment(f"shift_G{ditch}_G{ditch - 1}",
                                    (x1, y), (x1, ys[ditch - 2]), "shift"))
    segments.extend([
        Segment("return_G1", (right, ys[0]), (left, ys[0]), "return"),
        Segment("park", (left, ys[0]), parking, "exit"),
    ])
    return segments


def _max_projection(a: float, b: float, low: float, high: float) -> float:
    """Maximum of a*abs(cos(t)) + b*abs(sin(t)) on a closed interval."""
    if high - low >= 2 * math.pi:
        return math.hypot(a, b)
    candidates = [low, high]
    phi = math.atan2(b, a)
    for k in range(math.floor(low / math.pi) - 1,
                   math.ceil(high / math.pi) + 2):
        for theta in (phi + k * math.pi, -phi + k * math.pi):
            if low <= theta <= high:
                candidates.append(theta)
    return max(a * abs(math.cos(t)) + b * abs(math.sin(t)) for t in candidates)


def footprint_half_extents(config: Config) -> Point:
    """Worst AABB over fixed yaw +/- error, then tracking + clearance margin.

    Body length is parallel to vehicle heading; width is perpendicular. Wheel
    spacing (for example 160 mm) must NOT be substituted for either body size.
    """
    yaw = math.radians(config.yaw_deg % 360)
    error = math.radians(config.yaw_error_deg)
    pad = config.tracking_error_m + config.safety_margin_m
    hx = _max_projection(config.body_length_m / 2, config.body_width_m / 2,
                         yaw - error, yaw + error) + pad
    hy = _max_projection(config.body_width_m / 2, config.body_length_m / 2,
                         yaw - error, yaw + error) + pad
    return hx, hy


def _segment_hits_rect(start: Point, end: Point, rect: Rect) -> bool:
    """Closed segment / closed AABB intersection using a continuous slab test."""
    t0, t1 = 0.0, 1.0
    for p, q, lower, upper in zip(start, end,
                                   (rect.xmin, rect.ymin), (rect.xmax, rect.ymax)):
        delta = q - p
        if abs(delta) <= EPS:
            if p < lower - EPS or p > upper + EPS:
                return False
        else:
            a, b = sorted(((lower - p) / delta, (upper - p) / delta))
            t0, t1 = max(t0, a), min(t1, b)
            if t0 > t1 + EPS:
                return False
    return True


def segment_clearance_errors(segment: Segment, config: Config) -> list[str]:
    """Check EVERY position of a segment, including partial gate crossings.

    Obstacles are inflated by the full worst-case footprint. The permitted
    floor is exactly field UNION start rectangle, with a 0.4 m opening at y=0.
    The AABB enclosure is conservative for yaw; endpoint-only tests are not used.
    """
    if not all(math.isfinite(v) for p in (segment.start, segment.end) for v in p):
        return ["nonfinite_coordinate"]
    geo = field_geometry(config.map_scale)
    field, start = geo["field"], geo["start_area"]
    hx, hy = footprint_half_extents(config)
    x0, y0 = segment.start
    x1, y1 = segment.end
    errors = []
    if (min(x0, x1) - hx < field.xmin - EPS or
            max(x0, x1) + hx > field.xmax + EPS or
            min(y0, y1) - hy < start.ymin - EPS or
            max(y0, y1) + hy > field.ymax + EPS):
        errors.append("outer_boundary")

    # At every t with body bottom below y=0, the entire body must fit the gate.
    # x(t) is linear, so its extrema on that clipped t interval are endpoints.
    if min(y0, y1) < hy - EPS:
        if abs(y1 - y0) <= EPS:
            low, high = 0.0, 1.0
        else:
            crossing = (hy - y0) / (y1 - y0)
            low, high = ((0.0, min(1.0, crossing)) if y1 > y0
                         else (max(0.0, crossing), 1.0))
        xs = [x0 + (x1 - x0) * t for t in (low, high)]
        if min(xs) - hx < start.xmin - EPS or max(xs) + hx > start.xmax + EPS:
            errors.append("gate_or_start_boundary")

    for row in geo["rows"]:
        inflated = Rect(row.name, row.xmin - hx, row.ymin - hy,
                        row.xmax + hx, row.ymax + hy)
        if _segment_hits_rect(segment.start, segment.end, inflated):
            errors.append(f"row_collision:{row.name}")
    return errors


def coverage_report(segments: list[Segment], scale: float = 1.0) -> list[dict]:
    """Coverage means a complete survey centerline; NOT visibility/picking proof."""
    geo = field_geometry(scale)
    left, right = geo["side_centers_x_m"]
    report = []
    for ditch in range(7, 0, -1):
        y = geo["ditch_centers_y_m"][ditch - 1]
        intervals = sorted(
            (min(s.start[0], s.end[0]), max(s.start[0], s.end[0]))
            for s in segments if s.phase == "survey"
            and abs(s.start[1] - y) < EPS and abs(s.end[1] - y) < EPS)
        reached = left
        for a, b in intervals:
            if a <= reached + EPS and b >= reached:
                reached = b
        report.append({"ditch": f"G{ditch}", "center_y_m": y,
                       "survey_centerline_complete": reached >= right - EPS,
                       "planned_north_row": f"R{ditch}" if ditch <= 6 else None})
    return report


def validate_route(segments: list[Segment], config: Config) -> dict:
    errors = []
    for index, segment in enumerate(segments):
        errors.extend(f"{segment.name}:{error}"
                      for error in segment_clearance_errors(segment, config))
        if index and math.dist(segments[index - 1].end, segment.start) > EPS:
            errors.append(f"{segment.name}:disconnected")
    parking = (0.2 * config.map_scale, -0.5 * config.map_scale)
    if not segments:
        errors.append("empty_route")
    elif math.dist(segments[0].start, parking) > EPS or math.dist(segments[-1].end, parking) > EPS:
        errors.append("start_or_finish_not_at_parking_center")
    coverage = coverage_report(segments, config.map_scale)
    errors.extend(f"missing_survey:{item['ditch']}" for item in coverage
                  if not item["survey_centerline_complete"])
    return {"passed_for_assumed_geometry": not errors, "errors": errors,
            "coverage": coverage, "deployment_ready": False}


def stop_to_stop_time(distance_m: float, speed_m_s: float,
                      accel_m_s2: float, decel_m_s2: float) -> float:
    """Seconds for a triangular/trapezoidal speed profile, both endpoints at rest."""
    values = (distance_m, speed_m_s, accel_m_s2, decel_m_s2)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("motion parameters must be finite")
    if distance_m < 0 or min(speed_m_s, accel_m_s2, decel_m_s2) <= 0:
        raise ValueError("distance must be nonnegative; speed/accelerations positive")
    ramp_distance = speed_m_s ** 2 * (1 / accel_m_s2 + 1 / decel_m_s2) / 2
    if distance_m < ramp_distance:
        peak = math.sqrt(2 * distance_m / (1 / accel_m_s2 + 1 / decel_m_s2))
        return peak / accel_m_s2 + peak / decel_m_s2
    return (speed_m_s / accel_m_s2 + speed_m_s / decel_m_s2
            + (distance_m - ramp_distance) / speed_m_s)


def motion_segments(segments: list[Segment], assume_tree_stops: bool) -> list[Segment]:
    """Optional stops at thirds of G1..G6: fabricated locations for sensitivity."""
    result = []
    for segment in segments:
        n = 3 if assume_tree_stops and segment.phase == "survey" and segment.north_row else 1
        points = [tuple(a + (b - a) * i / n for a, b in zip(segment.start, segment.end))
                  for i in range(n + 1)]
        result.extend(Segment(f"{segment.name}.{i + 1}" if n > 1 else segment.name,
                              points[i], points[i + 1], segment.phase,
                              segment.ditch, segment.north_row) for i in range(n))
    return result


def _minimum_symmetric_accel(lengths: list[float], speed: float,
                             available_motion_s: float) -> float | None:
    # Infinite acceleration is itself insufficient: there is no feasible a.
    if available_motion_s <= sum(lengths) / speed:
        return None
    low, high = 0.0, 1.0
    while sum(stop_to_stop_time(d, speed, high, high) for d in lengths) > available_motion_s:
        high *= 2
    for _ in range(70):
        mid = (low + high) / 2
        if sum(stop_to_stop_time(d, speed, mid, mid) for d in lengths) > available_motion_s:
            low = mid
        else:
            high = mid
    return high


def time_budget(segments: list[Segment], config: Config, *,
                speed: float | None = None, tree_dwell: float | None = None,
                accel: float | None = None, decel: float | None = None,
                assume_tree_stops: bool = True) -> dict:
    speed = config.speed_m_s if speed is None else speed
    tree_dwell = config.net_tree_dwell_s if tree_dwell is None else tree_dwell
    accel = config.accel_m_s2 if accel is None else accel
    decel = config.decel_m_s2 if decel is None else decel
    if not math.isfinite(tree_dwell) or tree_dwell < 0:
        raise ValueError("tree dwell must be finite and nonnegative")
    moves = motion_segments(segments, assume_tree_stops)
    lengths = [s.length_m for s in moves]
    driving = sum(stop_to_stop_time(d, speed, accel, decel) for d in lengths)
    settle = len(moves) * config.segment_settle_s
    tree_count = 12
    fixed = config.startup_s + settle + config.reserve_s
    elapsed = config.startup_s + driving + settle + tree_count * tree_dwell
    with_reserve = elapsed + config.reserve_s
    available_dwell = config.deadline_s - fixed - driving
    minimum_accel = _minimum_symmetric_accel(
        lengths, speed, config.deadline_s - fixed - tree_count * tree_dwell)
    return {
        "stop_model": "assumed_two_stops_at_thirds_per_G1_to_G6" if assume_tree_stops
                      else "segment_endpoints_only_plus_dwell",
        "speed_m_s": speed, "accel_m_s2": accel, "decel_m_s2": decel,
        "net_dwell_per_tree_s": tree_dwell, "assumed_tree_stop_count": tree_count,
        "motion_segment_count": len(moves), "distance_m": sum(lengths),
        "cruise_only_lower_bound_s": sum(lengths) / speed,
        "driving_stop_to_stop_s": driving, "startup_s": config.startup_s,
        "segment_settle_s": settle, "net_tree_dwell_total_s": tree_count * tree_dwell,
        "elapsed_estimate_s": elapsed, "reserve_s": config.reserve_s,
        "budget_including_reserve_s": with_reserve, "deadline_s": config.deadline_s,
        "scenario_fits_deadline": with_reserve <= config.deadline_s + EPS,
        "remaining_net_tree_dwell_budget_s": available_dwell,
        "max_mean_net_tree_dwell_s": available_dwell / tree_count,
        "minimum_symmetric_accel_for_this_speed_and_dwell_m_s2": minimum_accel,
        "minimum_accel_condition": "no_finite_acceleration_suffices" if minimum_accel is None
                                   else "assumes_accel_equals_decel_and_all_other_scenario_values_hold",
        "is_upper_bound": False,
    }


def make_report(config: Config) -> dict:
    route = build_route(config.map_scale)
    geo = field_geometry(config.map_scale)
    hx, hy = footprint_half_extents(config)
    budgets = [time_budget(route, config, speed=speed, tree_dwell=dwell,
                           assume_tree_stops=stops)
               for stops in (False, True) for speed in (0.1, 0.2, 0.3, 0.5)
               for dwell in (5.0, 8.0, 12.0)]
    return {
        "schema_version": 1, "purpose": "OFFLINE SCENARIO ONLY - NOT FOR HARDWARE",
        "deployment_ready": False, "config_assumptions": asdict(config),
        "coordinate_system": "metres; +X right, +Y up into field; yaw 0 toward +X",
        "route_strategy": "enter; straight left-side outbound to L7; descending G7-to-G1 snake; return across G1; park",
        "survey_order": [s.ditch for s in route if s.phase == "survey"],
        "assumptions_and_limits": [
            "Body 0.27 x 0.27 m is an unconfirmed example, not a measured envelope; include all tools and payload.",
            "160 mm wheel spacing is not the whole vehicle footprint.",
            "Go straight to the far end before any survey; outbound passes ditch mouths but earns no coverage.",
            "The default yaw=90 degree plan assumes a front working mechanism facing +Y; survey G7 first, then G6..G1 process north rows R6..R1.",
            "Other --yaw-deg values are footprint scenarios only and do not validate the working mechanism's direction or reach toward the north rows.",
            "Yaw error, tracking error, safety margin, acceleration/deceleration and dwell must be measured.",
            "Uniform map scale 0.95 gives 0.38 m nominal corridors; it is only one case, not all independent +/-5% geometries.",
            "Continuous conservative AABB sweep checks rows/fences/gate; it omits trees, branches and tool swept motion.",
            "Coverage marks planned complete ditch centerlines, not camera visibility, classification or successful work.",
            "All motion segment endpoints stop; endpoint-only timing adds 12 net dwells but omits their extra acceleration.",
            "The second stop model splits G1..G6 into thirds: its 12 stop locations are assumptions, not measured tree coordinates.",
            "Both timing models are scenario estimates, not guaranteed upper bounds; relocation, retries, tool motion and slip are unknown.",
            "Twelve net dwells exclude segment settle; two purely yellow trees might need no stop, but all twelve are budgeted.",
            "Timing starts at release, includes startup plus whole start-area exit/return, and ends fully stopped; this counting scope is conservative relative to a gate-only timer.",
            "Acceleration 0.10 m/s^2 is a protection-budget assumption, not verified performance; 0.20/0.30 are sensitivity cases only.",
        ],
        "geometry": {"field": asdict(geo["field"]), "start_area": asdict(geo["start_area"]),
                     "rows": [asdict(row) for row in geo["rows"]],
                     "gate_x_m": geo["gate_x_m"],
                     "ditch_centers_y_m": geo["ditch_centers_y_m"],
                     "side_centers_x_m": geo["side_centers_x_m"],
                     "inflated_footprint_full_x_y_m": [2 * hx, 2 * hy]},
        "route": [dict(asdict(s), length_m=s.length_m, fixed_yaw_deg=config.yaw_deg,
                       stop_at_end=True) for s in route],
        "distance_m": sum(s.length_m for s in route),
        "distance_survey_and_return_loop_m": 29.2 * config.map_scale,
        "distance_entry_and_exit_legs_m": 1.4 * config.map_scale,
        "distance_inside_field_m": 29.6 * config.map_scale,
        "distance_inside_start_area_m": 1.0 * config.map_scale,
        "validation": validate_route(route, config),
        "selected_scenario": time_budget(route, config),
        "speed_dwell_sensitivity": budgets,
        "symmetric_acceleration_sensitivity": [
            time_budget(route, config, accel=a, decel=a, assume_tree_stops=stops)
            for stops in (False, True) for a in (0.1, 0.2, 0.3)],
        "assumed_tree_stop_positions": [
            {"ditch": s.ditch, "north_row": s.north_row,
             "position_m": [a + (b - a) * fraction for a, b in zip(s.start, s.end)],
             "is_measured_tree_position": False}
            for s in route if s.phase == "survey" and s.north_row for fraction in (1 / 3, 2 / 3)],
    }


def write_svg(path: Path, config: Config, route: list[Segment]) -> None:
    """Reproducible engineering diagram; no remote resources or drawing package."""
    geo = field_geometry(config.map_scale)
    px = 600 / (5.6 * config.map_scale)
    xoff, yoff = 55, 80 + 4.6 * config.map_scale * px

    def point(p: Point) -> tuple[float, float]:
        return xoff + p[0] * px, yoff - p[1] * px

    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="980" height="760" viewBox="0 0 980 760">',
             '<rect width="980" height="760" fill="#fff"/>',
             '<defs><marker id="arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><path d="M0 0 L7 3.5 L0 7" fill="#1368b5"/></marker><marker id="outbound-arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><path d="M0 0 L7 3.5 L0 7" fill="#287e35"/></marker><marker id="return-arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><path d="M0 0 L7 3.5 L0 7" fill="#d06a22"/></marker></defs>',
             '<g font-family="Microsoft YaHei, Arial, sans-serif" font-size="14">',
             '<text x="30" y="28" font-size="20" font-weight="bold">先直行到尽头，再蛇形返回 — NOT FOR HARDWARE</text>']

    def rect(r: Rect, fill: str, stroke: str = "#333") -> None:
        x, y = point((r.xmin, r.ymax))
        parts.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{(r.xmax-r.xmin)*px:.2f}" height="{(r.ymax-r.ymin)*px:.2f}" fill="{fill}" stroke="{stroke}"/>')

    rect(geo["field"], "#fafaf6")
    rect(geo["start_area"], "#dbf1da")
    # Erase the shared boundary at the open gate, preserving the two jambs.
    gx0, gy = point((0, 0))
    gx1, _ = point((0.4 * config.map_scale, 0))
    parts.append(f'<path d="M{gx0+1:.2f},{gy:.2f} H{gx1-1:.2f}" stroke="#fafaf6" stroke-width="3"/>')
    for row in geo["rows"]:
        rect(row, "#d8c2a2")
        x, y = point(((row.xmin + row.xmax) / 2, (row.ymin + row.ymax) / 2))
        parts.append(f'<text x="{x:.2f}" y="{y+5:.2f}" text-anchor="middle">{row.name}</text>')
    for segment in route:
        x0, y0 = point(segment.start)
        x1, y1 = point(segment.end)
        returning = segment.phase in ("return", "exit")
        outbound = segment.phase in ("enter", "outbound")
        if returning:
            style = 'stroke="#d06a22" stroke-dasharray="7 5" marker-end="url(#return-arrow)"'
        elif outbound:
            style = 'stroke="#287e35" marker-end="url(#outbound-arrow)"'
        else:
            style = 'stroke="#1368b5" marker-end="url(#arrow)"'
        width = 5 if outbound else 2.5
        parts.append(f'<path d="M{x0:.2f},{y0:.2f} L{x1:.2f},{y1:.2f}" fill="none" stroke-width="{width}" {style}/>')
    for i, y in enumerate(geo["ditch_centers_y_m"], 1):
        x, sy = point((1.5 * config.map_scale, y))
        parts.append(f'<text x="{x:.2f}" y="{sy-7:.2f}" text-anchor="middle" fill="#114c86">G{i}</text>')
    sx, sy = point(route[0].start)
    parts.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="6" fill="#287e35"/><text x="{sx+13:.2f}" y="{sy+5:.2f}">起点 / 终点 P</text>')
    notes = [
        "世界坐标：X 向右、Y 向上；长度单位为米。",
        f"地图等比例系数：{config.map_scale:g}；中心路线：{sum(s.length_m for s in route):.2f} m",
        "① 绿粗线：入场后沿左通道直达最远端 L7。",
        "② 蓝实线：从 G7 开始，向下蛇形返回至 G1。",
        "③ 橙虚线箭头：沿 G1 回左侧，再出门停车。",
        f"包络朝向 {config.yaw_deg:g}°；默认 90° 方案车头为 +Y。",
        "先行经过沟口不计覆盖；G7 无北侧作业垄。",
        "默认前向机构：G6 → G1 处理北侧 R6 → R1。",
        "R4 为短垄，名义 x = 0.6 .. 2.4 m。",
        "图线表示车体中心；检查使用整车连续扫掠。",
        f"演示车体 {config.body_width_m:g} × {config.body_length_m:g} m：尚未确认",
        f"航向误差 ±{config.yaw_error_deg:g}°；跟踪误差 {config.tracking_error_m:g} m",
        f"每侧安全余量：{config.safety_margin_m:g} m",
        "尚未建模树枝、树体及作业机构的运动包络。",
        "本图未声称任何真实树位置。",
        "等比例缩放不能覆盖全部现场尺寸误差。",
        "其他朝向只验包络，不证明机构作业方向。",
    ]
    for i, note in enumerate(notes):
        parts.append(f'<text x="430" y="{90+i*29}">{note}</text>')
    parts.extend(['<text x="30" y="735" fill="#a12828">仅供离线核查 | deployment_ready = false | 未连接 STM32 或执行机构</text>', '</g></svg>'])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write full JSON report (otherwise stdout)")
    parser.add_argument("--svg", type=Path, help="write route drawing")
    parser.add_argument("--speed", type=float, default=0.30, help="m/s; assumed maximum")
    parser.add_argument("--accel", type=float, default=0.10, help="m/s^2; unmeasured assumption")
    parser.add_argument("--decel", type=float, help="m/s^2; defaults to --accel")
    parser.add_argument("--body-width", type=float, default=0.27, help="m; whole body envelope")
    parser.add_argument("--body-length", type=float, default=0.27, help="m; includes tools/payload")
    parser.add_argument("--map-scale", type=float, default=1.0, help="uniform geometry scaling only")
    parser.add_argument("--yaw-deg", type=float, default=90.0)
    parser.add_argument("--yaw-error-deg", type=float, default=5.0)
    parser.add_argument("--tracking-error", type=float, default=0.02, help="m per axis")
    parser.add_argument("--margin", type=float, default=0.01, help="m per side")
    parser.add_argument("--tree-dwell", type=float, default=8.0, help="net seconds/tree, excludes settle")
    args = parser.parse_args()
    try:
        config = Config(map_scale=args.map_scale, body_width_m=args.body_width,
                        body_length_m=args.body_length, yaw_deg=args.yaw_deg,
                        yaw_error_deg=args.yaw_error_deg, tracking_error_m=args.tracking_error,
                        safety_margin_m=args.margin, speed_m_s=args.speed,
                        accel_m_s2=args.accel, decel_m_s2=args.accel if args.decel is None else args.decel,
                        net_tree_dwell_s=args.tree_dwell)
    except ValueError as exc:
        parser.error(str(exc))
    report = make_report(config)
    output = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
        scenario = report["selected_scenario"]
        print(f"{report['distance_m']:.2f} m | assumed geometry: {report['validation']['passed_for_assumed_geometry']} | "
              f"budget incl. reserve: {scenario['budget_including_reserve_s']:.1f} s / {config.deadline_s:g} s | "
              f"scenario fits: {scenario['scenario_fits_deadline']} | deployment_ready: false")
    else:
        print(output, end="")
    if args.svg:
        write_svg(args.svg, config, build_route(config.map_scale))
    # Nonzero means geometry/coverage failed; deadline is separately scenario data.
    return 0 if report["validation"]["passed_for_assumed_geometry"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
