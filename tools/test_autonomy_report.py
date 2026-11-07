import copy
import tempfile
import unittest
from pathlib import Path
from autonomy_report import ramp_records, ramp_report, summarize_trace, trace_svg


class Reports(unittest.TestCase):
    def records(self):
        return [dict(t_ms=t, target=target, output=output, measured=feedback, reversing=0) for t,target,output,feedback in (
            (0,(1,0,0),(0,0,0),(0,0,0)),
            (1000,(1,0,0),(.2,0,0),(.1,0,0)),
            (2000,(0,0,0),(.1,0,0),(.08,0,0)),
            (3000,(0,0,0),(0,0,0),(.04,0,0)),
            (4000,(0,0,0),(0,0,0),(0,0,0))) ]

    def test_command_vs_feedback(self):
        r = ramp_report(self.records(), [])
        self.assertAlmostEqual(r['peak_command_average_accel_mps2'], .2)
        self.assertAlmostEqual(r['peak_measured_average_accel_mps2'], .1)
        self.assertEqual(r['stops'], [dict(start_ms=2000,command_stop_ms=3000,measured_stop_ms=4000)])

    def test_no_feedback_not_fabricated(self):
        records = self.records()
        for r in records: r['measured'] = None
        result = ramp_report(records, [])
        self.assertIsNone(result['peak_measured_average_accel_mps2'])
        self.assertIsNone(result['stops'][0]['measured_stop_ms'])

    def test_reset_does_not_create_acceleration(self):
        records = self.records()[:2] + self.records()[:1]
        result = ramp_report(records, [])
        self.assertEqual(result['sessions'], 2)
        self.assertAlmostEqual(result['peak_command_average_accel_mps2'], .2)

    def test_duplicate_ignored(self):
        records = self.records()
        records.insert(1, copy.deepcopy(records[0]))
        self.assertEqual(ramp_report(records, [])['duplicate_timestamps'], 1)

    def test_cached_and_control_not_feedback(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'log.txt'
            p.write_text('[INFO] TEST_RAMP t_ms=0 target=1,0,0 output=0,0,0 reversing=0\n'
                         '[INFO] cached TEST_RAMP t_ms=100 target=1,0,0 output=9,0,0 reversing=0\n'
                         '[INFO] TEST_CONTROL t_ms=100 source=MANUAL vx=9 vy=0 wz=0\n'
                         '[INFO] TEST_RAMP t_ms=1000 target=1,0,0 output=.2,0,0 reversing=0\n')
            rows, warnings = ramp_records(p)
            self.assertEqual(len(rows), 2)
            self.assertIsNone(ramp_report(rows,warnings)['peak_measured_average_accel_mps2'])

    def test_nan_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'log.txt'
            p.write_text('TEST_RAMP t_ms=0 target=nan,0,0 output=0,0,0 reversing=0\n')
            with self.assertRaises(ValueError): ramp_records(p)

    def test_partial_feedback_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'log.csv'
            p.write_text('t_ms,target_vx,target_vy,target_wz,output_vx,output_vy,output_wz,measured_vx,measured_vy,measured_wz\n0,1,0,0,0,0,0,.1,,\n')
            with self.assertRaises(ValueError): ramp_records(p)

    def test_trace_totals_and_first_fault(self):
        rows = [dict(t_ms=10,state='NAVIGATE',segment=0,aisle=0,coverage=0,actions=0,qualified=0,first_fault='OK',
                     true_x=.2,true_y=-.5,pose_x=.2,pose_y=-.5,cmd_vx=.1,cmd_vy=0,opening=1),
                dict(t_ms=20,state='FAULT',segment=0,aisle=0,coverage=0,actions=0,qualified=0,first_fault='POSE',
                     true_x=.2,true_y=-.5,pose_x=.3,pose_y=-.5,cmd_vx=0,cmd_vy=0,opening=1)]
        result = summarize_trace(rows)
        self.assertEqual(result['elapsed_s'],sum(result['state_seconds'].values()))
        self.assertEqual(result['first_fault']['first_fault'],'POSE')
        self.assertFalse(result['deployment_ready'])
        self.assertIn('SIMULATION ONLY',trace_svg(rows,'test'))


if __name__ == '__main__': unittest.main()
