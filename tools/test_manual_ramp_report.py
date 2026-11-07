"""Regression for comparison evidence: rejected input and discontinuity semantics."""
import csv
import tempfile
import unittest
from pathlib import Path
import manual_ramp_report as report

FIELDS = "scenario,t_s,dt_s,drive_allowed,protective_reset,stationary,input_vx,input_vy,input_wz,limited_vx,limited_vy,limited_wz,cmd_vx,cmd_vy,cmd_wz,accel_vx,accel_vy,accel_wz,reversing".split(",")


class RampReportTests(unittest.TestCase):
    def load(self, samples):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "trace.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, FIELDS)
                writer.writeheader()
                for sample in samples:
                    row = dict.fromkeys(FIELDS, 0)
                    row.update(scenario="test", dt_s=.1, drive_allowed=1)
                    row.update(sample)
                    writer.writerow(row)
            return report.load(path)["test"]

    def test_command_stop_integral_and_time(self):
        rows = self.load([dict(t_s=.1, input_vx=1, limited_vx=1, cmd_vx=1),
                          dict(t_s=.2, cmd_vx=.5), dict(t_s=.3, cmd_vx=0)])
        metrics = report.metrics(rows)
        self.assertAlmostEqual(metrics["normal_stop_time_s"], .2)
        self.assertAlmostEqual(metrics["normal_stop_command_distance_m"], .1)
        self.assertEqual(metrics["final_cmd"], [0, 0, 0])

    def test_protective_stop_is_not_smoothed_away_in_data(self):
        rows = self.load([dict(t_s=.1, cmd_vx=1), dict(t_s=.2, cmd_vx=0, protective_reset=1)])
        self.assertEqual(rows[-1]["cmd_vx"], 0)
        self.assertIsNone(rows[-1]["a"])
        self.assertIsNone(rows[-1]["j"])

    def test_invalid_or_nonmonotonic_trace_is_rejected(self):
        for rows in ([dict(t_s=.1, cmd_vx="nan")], [dict(t_s=.1, dt_s=0)],
                     [dict(t_s=.2), dict(t_s=.1)]):
            with self.assertRaises(ValueError):
                self.load(rows)


if __name__ == "__main__":
    unittest.main()
