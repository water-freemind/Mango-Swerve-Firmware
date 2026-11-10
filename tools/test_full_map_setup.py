import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from full_map_setup import new_profile, save, set_tree, set_station, validate, write_preview


class FullMapSetupTests(unittest.TestCase):
    def test_empty_profile_has_no_fabricated_tree_locations(self):
        p = new_profile()
        validate(p)
        self.assertEqual(p["trees"], [])
        self.assertFalse(p["start_zone_has_fences"])
        self.assertFalse(p["firmware_tree_upload_supported"])

    def test_tree_update_does_not_mutate_original(self):
        p = new_profile()
        q = set_tree(p, 1, 1, 1.0, 0.55)
        q = set_tree(q, 1, 1, 1.2, 0.55)
        self.assertEqual(len(q["trees"]), 1)
        self.assertEqual(q["trees"][0]["x_m"], 1.2)
        self.assertEqual(p["trees"], [])

    def test_bad_coordinates_and_row_are_rejected(self):
        for ident, row, x, y in [(0, 1, 1, .5), (1, 7, 1, .5), (1, 4, .5, 2.65),
                                  (1, 1, float("nan"), .5), (1, 1, 1, float("inf")),
                                  (True, 1, 1, .5), (1, 1, 3, .5)]:
            with self.subTest(values=(ident, row, x, y)), self.assertRaises(ValueError):
                set_tree(new_profile(), ident, row, x, y)

    def test_conflicting_trees_are_rejected(self):
        p = set_tree(new_profile(), 1, 1, 1, .55)
        with self.assertRaises(ValueError):
            set_tree(p, 2, 1, 1.01, .55)
        p = set_tree(p, 2, 1, 2, .55)
        with self.assertRaises(ValueError):
            set_tree(p, 3, 1, 1.5, .55)

    def test_stations_only_g1_to_g6_and_not_tree_coordinates(self):
        p = set_station(new_profile(), 11, 6, 1.2, 3.7)
        self.assertEqual(p["trees"], [])
        self.assertEqual(p["stations"][0]["row"], 6)
        with self.assertRaises(ValueError):
            set_station(p, 12, 7, 1.2, 4.4)
        with self.assertRaises(ValueError):
            set_station(p, 12, 6, 1.2, 3.0)
        with self.assertRaises(ValueError):
            set_tree(p, 11, 5, 1.2, 3.35)

    def test_saved_svg_and_unknown_map_do_not_claim_hardware_success(self):
        p = new_profile()
        with tempfile.TemporaryDirectory() as d:
            save(Path(d)/"profile.json", p)
            out = Path(d)/"preview.svg"
            write_preview(out, p)
            ET.parse(out)
            text = out.read_text(encoding="utf-8")
            self.assertIn("无围栏", text)
            self.assertIn("未做实车全图验收", text)
        p["field_m"]["width"] = 4.0
        with self.assertRaises(ValueError):
            validate(p)


if __name__ == "__main__":
    unittest.main()
