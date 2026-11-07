"""Run: py -3 -B -m unittest discover -s tools -p test_competition_route.py -v"""

import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from competition_route import (Config, Segment, build_route, coverage_report, field_geometry,
                               footprint_half_extents, make_report, motion_segments,
                               segment_clearance_errors, stop_to_stop_time,
                               time_budget, validate_route, write_svg)


class GeometryTests(unittest.TestCase):
    def test_nominal_route_is_continuous_covers_seven_ditches_and_returns(self):
        route = build_route()
        result = validate_route(route, Config())
        self.assertTrue(result["passed_for_assumed_geometry"], result["errors"])
        self.assertEqual(len(result["coverage"]), 7)
        self.assertEqual(route[0].start, route[-1].end)
        self.assertAlmostEqual(sum(s.length_m for s in route), 30.6)
        self.assertFalse(result["deployment_ready"])

    def test_goes_straight_to_far_end_before_first_survey(self):
        route = build_route()
        geo = field_geometry()
        first_survey = next(i for i, segment in enumerate(route) if segment.phase == "survey")
        self.assertEqual(first_survey, 2)
        self.assertEqual([s.phase for s in route[:first_survey]], ["enter", "outbound"])
        self.assertTrue(all(s.start[0] == s.end[0] for s in route[:first_survey]))
        self.assertAlmostEqual(route[first_survey].start[1], max(geo["ditch_centers_y_m"]))
        self.assertEqual(route[first_survey].ditch, 7)
        self.assertAlmostEqual(route[1].length_m, 4.2)

    def test_passing_ditch_mouths_on_outbound_does_not_earn_coverage(self):
        outbound = [s for s in build_route() if s.phase in ("enter", "outbound")]
        self.assertTrue(all(not item["survey_centerline_complete"]
                            for item in coverage_report(outbound)))

    def test_snake_descends_seven_to_one_then_returns_only_across_bottom(self):
        route = build_route()
        surveys = [s for s in route if s.phase == "survey"]
        self.assertEqual([s.ditch for s in surveys], [7, 6, 5, 4, 3, 2, 1])
        self.assertEqual([s.north_row for s in surveys], [None, 6, 5, 4, 3, 2, 1])
        shifts = [s for s in route if s.phase == "shift"]
        self.assertEqual(len(shifts), 6)
        self.assertTrue(all(s.end[1] < s.start[1] for s in shifts))
        last_survey = max(i for i, s in enumerate(route) if s.phase == "survey")
        self.assertEqual([s.name for s in route[last_survey + 1:]], ["return_G1", "park"])
        self.assertEqual([item["ditch"] for item in coverage_report(route)],
                         ["G7", "G6", "G5", "G4", "G3", "G2", "G1"])

    def test_038_m_corridor_uniform_scale_and_actual_body_check(self):
        config = Config(map_scale=0.95)
        route = build_route(config.map_scale)
        self.assertTrue(validate_route(route, config)["passed_for_assumed_geometry"])
        too_large = Config(map_scale=0.95, body_width_m=0.38, body_length_m=0.38)
        self.assertFalse(validate_route(route, too_large)["passed_for_assumed_geometry"])

    def test_038_square_45_degree_heading_cannot_fit_040_corridor(self):
        config = Config(body_width_m=0.38, body_length_m=0.38, yaw_deg=45,
                        yaw_error_deg=0, tracking_error_m=0, safety_margin_m=0)
        hx, hy = footprint_half_extents(config)
        self.assertAlmostEqual(2 * hx, 0.38 * math.sqrt(2))
        self.assertAlmostEqual(hx, hy)
        self.assertFalse(validate_route(build_route(), config)["passed_for_assumed_geometry"])

    def test_027_square_45_degrees_cannot_fit_038_corridor_even_without_margin(self):
        config = Config(map_scale=0.95, body_width_m=0.27, body_length_m=0.27,
                        yaw_deg=45, yaw_error_deg=0, tracking_error_m=0, safety_margin_m=0)
        hx, hy = footprint_half_extents(config)
        self.assertAlmostEqual(2 * hx, 0.27 * math.sqrt(2))
        self.assertGreater(2 * hx, 0.38)
        self.assertGreater(2 * hy, 0.38)
        self.assertFalse(validate_route(build_route(config.map_scale), config)["passed_for_assumed_geometry"])

    def test_yaw_interval_contains_interior_maximum(self):
        config = Config(body_length_m=0.6, body_width_m=0.2, yaw_deg=15,
                        yaw_error_deg=15, tracking_error_m=0, safety_margin_m=0)
        hx, _ = footprint_half_extents(config)
        self.assertAlmostEqual(hx, math.hypot(0.3, 0.1))

    def test_body_length_is_checked_even_when_width_fits(self):
        config = Config(body_width_m=0.20, body_length_m=0.45, yaw_error_deg=0,
                        tracking_error_m=0, safety_margin_m=0)
        errors = segment_clearance_errors(next(s for s in build_route() if s.name == "G1"), config)
        self.assertIn("row_collision:R1", errors)

    def test_safe_endpoints_do_not_allow_crossing_through_row(self):
        config = Config()
        diagonal = Segment("bad_crossing", (0.2, 0.2), (2.8, 0.9), "survey")
        for p in (diagonal.start, diagonal.end):
            self.assertEqual(segment_clearance_errors(Segment("point", p, p, "test"), config), [])
        self.assertIn("row_collision:R1", segment_clearance_errors(diagonal, config))

    def test_partial_body_outside_gate_is_rejected(self):
        segment = Segment("entry", (0.35, -0.5), (0.35, 0.2), "enter")
        self.assertIn("gate_or_start_boundary", segment_clearance_errors(segment, Config()))

    def test_diagonal_exits_gate_before_whole_body_has_entered_field(self):
        # Both endpoint bodies fit the floor union; the diagonal sweeps across
        # the right gate jamb before the rear clears y=0.
        segment = Segment("cut_gate", (0.2, -0.5), (0.8, 0.2), "enter")
        config = Config(body_width_m=0.1, body_length_m=0.1, yaw_error_deg=0,
                        tracking_error_m=0, safety_margin_m=0)
        for p in (segment.start, segment.end):
            self.assertEqual(segment_clearance_errors(Segment("point", p, p, "test"), config), [])
        self.assertIn("gate_or_start_boundary", segment_clearance_errors(segment, config))

    def test_omitted_ditch_is_reported_even_if_other_paths_are_safe(self):
        route = [s for s in build_route() if s.name != "G4"]
        result = validate_route(route, Config())
        self.assertIn("missing_survey:G4", result["errors"])
        self.assertFalse(result["passed_for_assumed_geometry"])

    def test_label_does_not_fake_full_centerline_coverage(self):
        short = Segment("G1", (0.2, 0.2), (1.5, 0.2), "survey", 1, 1)
        self.assertFalse(coverage_report([short])[-1]["survey_centerline_complete"])
        rest = Segment("G1_cont", (1.5, 0.2), (2.8, 0.2), "survey", 1, 1)
        self.assertTrue(coverage_report([short, rest])[-1]["survey_centerline_complete"])


class TimeTests(unittest.TestCase):
    def test_triangle_and_trapezoid_in_seconds(self):
        self.assertAlmostEqual(stop_to_stop_time(1, 2, 1, 1), 2)
        self.assertAlmostEqual(stop_to_stop_time(5, 2, 1, 1), 4.5)
        self.assertAlmostEqual(stop_to_stop_time(3, 1, 0.5, 1), 4.5)
        self.assertEqual(stop_to_stop_time(0, 1, 1, 1), 0)

    def test_tree_stops_increase_motion_time_but_not_distance_or_dwell(self):
        route, config = build_route(), Config()
        base = time_budget(route, config, assume_tree_stops=False)
        stops = time_budget(route, config, assume_tree_stops=True)
        self.assertAlmostEqual(base["distance_m"], stops["distance_m"])
        self.assertEqual(stops["motion_segment_count"] - base["motion_segment_count"], 12)
        self.assertGreater(stops["driving_stop_to_stop_s"], base["driving_stop_to_stop_s"])
        self.assertEqual(base["net_tree_dwell_total_s"], stops["net_tree_dwell_total_s"])
        self.assertEqual(stops["segment_settle_s"] - base["segment_settle_s"], 12)

    def test_test_speed_cannot_meet_five_minutes_even_at_infinite_accel(self):
        budget = time_budget(build_route(), Config(), speed=0.1)
        self.assertAlmostEqual(budget["cruise_only_lower_bound_s"], 306)
        self.assertFalse(budget["scenario_fits_deadline"])
        self.assertIsNone(budget["minimum_symmetric_accel_for_this_speed_and_dwell_m_s2"])

    def test_default_does_not_claim_five_minute_success(self):
        budget = time_budget(build_route(), Config())
        self.assertGreater(budget["budget_including_reserve_s"], 300)
        self.assertFalse(budget["scenario_fits_deadline"])
        self.assertFalse(budget["is_upper_bound"])

    def test_budget_counts_startup_dwell_settle_and_reserve_once(self):
        config = Config()
        budget = time_budget(build_route(), config)
        self.assertAlmostEqual(budget["budget_including_reserve_s"],
                               budget["driving_stop_to_stop_s"] + 29 + 12 * 8 + 8 + 30)
        self.assertAlmostEqual(budget["remaining_net_tree_dwell_budget_s"],
                               300 - budget["driving_stop_to_stop_s"] - 29 - 8 - 30)

    def test_minimum_acceleration_condition_reconstructs_deadline(self):
        route, config = build_route(), Config()
        budget = time_budget(route, config, speed=0.5, tree_dwell=5)
        accel = budget["minimum_symmetric_accel_for_this_speed_and_dwell_m_s2"]
        self.assertIsNotNone(accel)
        at_limit = time_budget(route, config, speed=0.5, tree_dwell=5, accel=accel, decel=accel)
        self.assertAlmostEqual(at_limit["budget_including_reserve_s"], 300, places=6)
        below = time_budget(route, config, speed=0.5, tree_dwell=5, accel=accel * 0.99, decel=accel * 0.99)
        self.assertFalse(below["scenario_fits_deadline"])

    def test_assumed_stop_motion_remains_connected_and_same_coverage(self):
        route = motion_segments(build_route(), True)
        self.assertTrue(validate_route(route, Config())["passed_for_assumed_geometry"])

    def test_report_keeps_offline_and_fabricated_stop_labels(self):
        report = make_report(Config())
        self.assertFalse(report["deployment_ready"])
        self.assertEqual(len(report["assumed_tree_stop_positions"]), 12)
        self.assertTrue(all(not p["is_measured_tree_position"] for p in report["assumed_tree_stop_positions"]))
        self.assertAlmostEqual(report["distance_survey_and_return_loop_m"]
                               + report["distance_entry_and_exit_legs_m"], report["distance_m"])
        self.assertAlmostEqual(report["distance_inside_field_m"]
                               + report["distance_inside_start_area_m"], report["distance_m"])

    def test_svg_is_standalone_parseable_and_marks_hardware_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.svg"
            write_svg(path, Config(), build_route())
            root = ET.parse(path).getroot()
            texts = [node.text for node in root.iter("{http://www.w3.org/2000/svg}text")]
            self.assertTrue(any("NOT FOR HARDWARE" in text for text in texts))
            self.assertIn("G7", texts)
            self.assertIn("R4", texts)
            self.assertTrue(any("绿粗线：入场后沿左通道直达最远端" in text for text in texts))
            outbound = [node for node in root.iter("{http://www.w3.org/2000/svg}path")
                        if node.attrib.get("marker-end") == "url(#outbound-arrow)"]
            self.assertEqual(len(outbound), 2)
            self.assertTrue(all(node.attrib.get("stroke") == "#287e35"
                                and float(node.attrib["stroke-width"]) > 2.5 for node in outbound))
            returns = [node for node in root.iter("{http://www.w3.org/2000/svg}path")
                       if node.attrib.get("stroke-dasharray")]
            self.assertEqual(len(returns), 2)
            self.assertTrue(all(node.attrib.get("marker-end") == "url(#return-arrow)"
                                for node in returns))

    def test_invalid_parameters_do_not_create_plausible_results(self):
        for changes in ({"map_scale": 0}, {"body_width_m": -1}, {"speed_m_s": float("nan")},
                        {"accel_m_s2": 0}, {"yaw_error_deg": -1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Config(**changes)
        with self.assertRaises(ValueError):
            stop_to_stop_time(1, 0, 1, 1)


if __name__ == "__main__":
    unittest.main()
