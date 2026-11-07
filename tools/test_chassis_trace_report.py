import csv
import math
from pathlib import Path
import tempfile
import unittest
from chassis_trace_report import FIELDS, read_csv, observe, analyze_rows, write_svg, reassemble_chunks, dump_metadata


def row(sequence=1,t_ms=100):
    r=dict.fromkeys(FIELDS,0)
    r.update(sequence=sequence,t_ms=t_ms,length_m=.2,width_m=.2,radius_m=.05,
             target_vx=.5,limited_vx=.5,control_dt_ms=2,steer_valid_mask=15,drive_valid_mask=15,mode=2,permitted=1,imu_valid=1,imu_age_us=1000)
    for wheel in ('fl','fr','rr','rl'): r[f'omega_{wheel}']=10
    return r

class TraceReportTest(unittest.TestCase):
    def test_forward_and_left_frame(self):
        r=row();o=observe(r)
        self.assertEqual(o['issues'],[])
        self.assertEqual(o['wheel_fit_external'],[.5,0,0])
        self.assertEqual(o['residual_max_mps'],0)
        for wheel in ('fl','fr','rr','rl'): r[f'angle_{wheel}']=-math.pi/2
        self.assertAlmostEqual(observe(r)['wheel_fit_external'][1],.5)

    def test_rotation_sign(self):
        r=row();r['gyro_z_rad_s']=1
        for wheel,(x,y) in zip(('fl','fr','rr','rl'),((.1,-.1),(.1,.1),(-.1,.1),(-.1,-.1))):
            # Internal wz=-1 corresponds to external CCW +1.
            vx,vy=y,-x
            r[f'omega_{wheel}']=math.hypot(vx,vy)/.05
            r[f'angle_{wheel}']=math.atan2(vy,vx)
        o=observe(r)
        self.assertAlmostEqual(o['wheel_fit_external'][2],1)
        self.assertLess(o['residual_max_mps'],1e-12)
        self.assertAlmostEqual(o['gyro_wheel_error_rad_s'],0)

    def test_injected_delay_and_drop(self):
        delayed=row();delayed['drive_age_rr']=150
        dropped=row(2,120);dropped['steer_valid_mask']=7
        report=analyze_rows([delayed,dropped], 'synthetic')
        self.assertIn('STALE_WHEEL',report['samples'][0]['issues'])
        self.assertIn('MISSING_WHEEL',report['samples'][1]['issues'])
        self.assertIsNone(report['samples'][0]['wheel_fit_external'])
        self.assertIsNone(report['samples'][1]['wheel_fit_external'])

    def test_single_wheel_anomaly_and_coherent_slip_limit(self):
        faulty=row();faulty['omega_fl']=18
        self.assertGreater(observe(faulty)['residual_max_mps'],.15)
        slipping=row()
        # All wheels can agree while the body slides or the tires spin.
        for wheel in ('fl','fr','rr','rl'): slipping[f'omega_{wheel}']=20
        self.assertEqual(observe(slipping)['residual_max_mps'],0)
        self.assertIn('Coherent all-wheel slip',analyze_rows([slipping],'synthetic')['limits'][1])

    def test_mode_fault_gap_and_rollover(self):
        a=row(0xffffffff,0xfffffff0);b=row(0,4);b['mode']=1;b['permitted']=0
        c=row(2,24);c['fault']=5
        report=analyze_rows([a,b,c],'synthetic')
        self.assertAlmostEqual(report['samples'][-1]['elapsed_s'],.04)
        self.assertEqual(len(report['events']),2)
        self.assertEqual(report['sequence_gaps'][0]['sequence_step'],2)
        self.assertIn('FAULT',report['first_issue']['issues'])

    def test_invalid_numeric_is_not_zero_motion(self):
        r=row();r['omega_fr']=float('nan')
        o=observe(r)
        self.assertIn('NONFINITE',o['issues']);self.assertIsNone(o['wheel_fit_external'])
        r=row();r['length_m']=0
        self.assertIn('GEOMETRY',observe(r)['issues'])
        r=row();r['imu_age_us']=20001
        self.assertIn('STALE_IMU',observe(r)['issues'])
        self.assertIsNone(observe(r)['gyro_wheel_error_rad_s'])
        r=row();r['imu_valid']=0
        self.assertIn('MISSING_IMU',observe(r)['issues'])
        r=row();r['control_dt_ms']=0
        self.assertIn('CONTROL_PERIOD',observe(r)['issues'])

    def test_chunk_reassembly_and_missing_data(self):
        import io
        output=io.StringIO();writer=csv.DictWriter(output,fieldnames=FIELDS)
        writer.writeheader();writer.writerow(row())
        original=output.getvalue().splitlines()
        chunks=[]
        for key,line in zip(('HEADER','1'),original):
            parts=[line[i:i+150] for i in range(0,len(line),150)]
            for index,part in reversed(list(enumerate(parts,1))):
                text=f'[INFO] TEST_TRACE seq={key} part={index} total={len(parts)} data={part}'
                self.assertLess(len(text),256)
                chunks.append(text)
        self.assertEqual(reassemble_chunks(chunks+chunks[:1]),original)
        with self.assertRaises(ValueError):reassemble_chunks(chunks[1:])
        with self.assertRaises(ValueError):reassemble_chunks(chunks+[chunks[0]+'corrupt'])
        with self.assertRaises(ValueError):reassemble_chunks([c.replace('seq=1 ','seq=2 ') for c in chunks])
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'raw.log';p.write_text('unrelated log\n'+'\n'.join(chunks))
            self.assertEqual(read_csv(p)[0]['sequence'],1)
            meta='[INFO] TEST_TRACE_META dump=7 expected_count=1 first=1 next=2 overwritten=25'
            end='[INFO] TEST_TRACE_END dump=7 sent_count=1'
            p.write_text('\n'.join([meta]+chunks+[end]))
            self.assertEqual(read_csv(p)[0]['sequence'],1)
            meta_no_id=meta.replace('dump=7 ','')+' reason=COMPLETE'
            end_no_id=end.replace('dump=7 ','')
            self.assertEqual(dump_metadata([meta_no_id,end_no_id])['stop_reason'],'COMPLETE')
            p.write_text('\n'.join([meta]+[c for c in chunks if 'seq=HEADER ' in c]+[end]))
            with self.assertRaises(ValueError):read_csv(p)
            p.write_text('\n'.join([meta]+chunks+[end.replace('sent_count=1','sent_count=0')]))
            with self.assertRaises(ValueError):read_csv(p)

    def test_csv_svg_and_explicit_provenance(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'trace.csv'
            with p.open('w',newline='',encoding='utf-8') as f:
                writer=csv.DictWriter(f,fieldnames=FIELDS);writer.writeheader();writer.writerow(row())
            parsed=read_csv(p)
            report=analyze_rows(parsed,'synthetic')
            self.assertEqual(report['source_kind'],'synthetic')
            svg=Path(d)/'trace.svg';write_svg(report,svg)
            self.assertIn('(synthetic)',svg.read_text())
            p.write_text('unrelated log\n'+''.join('TRACE,'+line+'\n' for line in p.read_text().splitlines()))
            self.assertEqual(read_csv(p),parsed)
            p.write_text('sequence,t_ms\n1,0\n')
            with self.assertRaises(ValueError):read_csv(p)
        with self.assertRaises(ValueError):analyze_rows([row()],'unspecified')
        with self.assertRaises(ValueError):analyze_rows([],'synthetic')

if __name__=='__main__': unittest.main()
