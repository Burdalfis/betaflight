"""Synthetic screens distinguish oscillation, transient response and limits."""
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
from stress_metrics import classify_yaw_series,entry_statistics,stress_metrics,analyze_trial


def signal(fn,duration=5,step=.01):
    return [(i*step,fn(i*step)) for i in range(round(duration/step)+1)]


def trace(duration=6,rate=lambda t:0,pitch=0,vx=lambda t:10,vz=lambda t:0,pwm=1715):
    rows=[]
    for t,yaw in signal(rate,duration):
        theta=math.radians(pitch);psi=math.radians(yaw)
        row={'sim_time_s':t,'roll_deg':0,'pitch_deg':pitch,'yaw_deg':yaw*t,
            'gyro_frd_x':-psi*math.sin(theta),'gyro_frd_y':0,'gyro_frd_z':-psi*math.cos(theta),
            'vx_m_s':vx(t),'vy_m_s':0,'vz_m_s':vz(t),'servo_pwm_us':1509,'actuator_valid':1}
        row.update({f'pwm{i}_us':pwm for i in range(3)});rows.append(row)
    return rows


def case():
    return {'name':'hold','end_sim_time_s':6,'stages':[
        {'name':'baseline','actual_start_sim_time_s':0,'channels':[1500,1500,1715,1500],'requested_yaw_rate_dps':0},
        {'name':'operating_lead','actual_start_sim_time_s':1,'channels':[1500,1500,1715,1500],'requested_yaw_rate_dps':0},
        {'name':'pulse','actual_start_sim_time_s':3,'channels':[1500,1500,1715,1500],'requested_yaw_rate_dps':0}]}


class PatternTests(unittest.TestCase):
    def test_constant_oscillation_requires_cycles_and_persistent_envelope(self):
        result=classify_yaw_series(signal(lambda t:12*math.sin(2*math.pi*2*t)))
        self.assertTrue(result['sustained_oscillation'])
        self.assertGreaterEqual(result['full_cycle_count'],4)
        self.assertGreaterEqual(result['oscillation_span_s'],1)
        self.assertAlmostEqual(result['last_two_to_first_two_cycle_amplitude_ratio'],1)
        self.assertFalse(result['divergent_growth_candidate'])

    def test_four_cycles_require_nine_alternating_peaks(self):
        values=[(i*.125,10*(-1)**i) for i in range(8)]
        self.assertFalse(classify_yaw_series(values,config={'maximum_sample_gap_s':.2})['sustained_oscillation'])
        values.append((1.,10))
        # Endpoint extrema require an observed rising and falling neighbor.
        bracketed=[(-.125,0)]+values+[(1.125,0)]
        result=classify_yaw_series(bracketed,config={'maximum_sample_gap_s':.2})
        self.assertEqual(result['full_cycle_count'],4)
        self.assertTrue(result['sustained_oscillation'])

    def test_rising_censored_final_lobe_cannot_close_a_fourth_cycle(self):
        values=[(.2+i*.01,10*math.sin(2*math.pi*(.2+i*.01))) for i in range(391)]
        result=classify_yaw_series(values)
        self.assertEqual(result['candidate_full_cycle_count'],4)
        self.assertEqual(result['full_cycle_count'],3)
        self.assertFalse(result['peak_confirmation'][-1]['right_reversal_observed'])
        self.assertFalse(result['sustained_oscillation'])

    def test_declining_censored_first_lobe_cannot_start_a_fourth_cycle(self):
        values=[(.4+i*.01,10*math.sin(2*math.pi*(.4+i*.01))) for i in range(391)]
        result=classify_yaw_series(values)
        self.assertEqual(result['candidate_full_cycle_count'],4)
        self.assertEqual(result['full_cycle_count'],3)
        self.assertFalse(result['peak_confirmation'][0]['left_reversal_observed'])
        self.assertFalse(result['sustained_oscillation'])

    def test_complete_flat_peak_plateaus_are_confirmed(self):
        values=[]
        for i in range(9):values.extend([0,10*(-1)**i,10*(-1)**i,0])
        result=classify_yaw_series([(i*.1,v) for i,v in enumerate(values)],config={'maximum_sample_gap_s':.11})
        self.assertEqual(result['full_cycle_count'],4)
        self.assertTrue(all(p['confirmed'] for p in result['peak_confirmation']))
        self.assertTrue(result['sustained_oscillation'])

    def test_decaying_ringing_and_quiet_tail_are_not_sustained(self):
        result=classify_yaw_series(signal(lambda t:40*math.exp(-t)*math.sin(2*math.pi*3*t)))
        self.assertTrue(result['decaying_ringing']);self.assertFalse(result['sustained_oscillation'])
        quiet=signal(lambda t:10*math.sin(2*math.pi*3*t) if t<2 else 0)
        self.assertFalse(classify_yaw_series(quiet)['sustained_oscillation'])

    def test_slow_persistent_tone_near_zero_crossing_uses_last_above_band_sample(self):
        result=classify_yaw_series(signal(lambda t:10*math.sin(2*math.pi*.25*t),duration=18))
        self.assertEqual(result['full_cycle_count'],4)
        self.assertTrue(result['oscillation_active_at_tail']);self.assertTrue(result['sustained_oscillation'])

    def test_single_overshoot_is_not_instability(self):
        result=classify_yaw_series(signal(lambda t:100*math.exp(-t*3)))
        for name in ('sustained_oscillation','divergent_growth_candidate','unwanted_world_yaw_spin'):
            self.assertFalse(result[name])

    def test_divergent_growth_reports_its_evidence_windows(self):
        result=classify_yaw_series(signal(lambda t:3*math.exp(t)))
        self.assertTrue(result['divergent_growth_candidate']);self.assertTrue(result['divergence_windows'])
        self.assertFalse(result['sustained_oscillation'])

    def test_spin_is_world_frame_zero_command_and_duration_limited(self):
        error=signal(lambda t:0)
        self.assertTrue(classify_yaw_series(error,signal(lambda t:100))['unwanted_world_yaw_spin'])
        self.assertFalse(classify_yaw_series(error,signal(lambda t:100),zero_yaw_command=False)['unwanted_world_yaw_spin'])
        self.assertFalse(classify_yaw_series(error,signal(lambda t:120 if t<.3 else 0))['unwanted_world_yaw_spin'])

    def test_sampling_gaps_and_bad_values_are_not_stable_evidence(self):
        result=classify_yaw_series([(0,10),(1,-10),(2,10)])
        self.assertFalse(result['classification_available']);self.assertIsNone(result['sustained_oscillation'])
        with self.assertRaises(ValueError):classify_yaw_series([(0,0),(.01,float('nan'))])
        with self.assertRaises(ValueError):classify_yaw_series([(0,0),(0,0)])
        with self.assertRaises(ValueError):classify_yaw_series(signal(lambda t:0),config={'minimum_full_cycles':3})


class TrialTests(unittest.TestCase):
    def test_identical_inputs_merge_and_limits_stay_separate(self):
        result=stress_metrics(trace(rate=lambda t:12*math.sin(2*math.pi*2*t),pwm=2000),case())
        self.assertEqual(len(result['holds']),1)
        self.assertTrue(result['flags']['sustained_oscillation'])
        self.assertTrue(result['holds'][0]['authority_limit_observed'])
        self.assertEqual(result['holds'][0]['saturation']['motor_upper']['sample_fraction'],1)
        self.assertIn('do not label',result['holds'][0]['interpretation'])

    def test_high_pitch_world_spin_is_not_lost_in_body_yaw(self):
        result=stress_metrics(trace(rate=lambda t:100,pitch=75),case())
        self.assertLess(result['holds'][0]['yaw']['native_yaw_error_peak_abs_dps'],30)
        self.assertTrue(result['flags']['unwanted_world_yaw_spin'])

    def test_body_target_is_used_instead_of_raw_stick(self):
        definition=case()
        for stage in definition['stages']:
            stage['channels'][3]=1368;stage['requested_yaw_rate_dps']=60*math.cos(math.radians(75))
        result=stress_metrics(trace(rate=lambda t:-60,pitch=75),definition)
        self.assertAlmostEqual(result['holds'][0]['yaw']['native_yaw_error_rms_dps'],0,places=10)
        self.assertFalse(result['flags']['unwanted_world_yaw_spin'])

    def test_missing_held_target_is_reconstructed_from_native_high_angle_profile(self):
        import struct
        rates=bytearray(24)
        for i in (0,11,12):rates[i]=7
        for i in (2,3,4):rates[i]=67
        rates[16:22]=struct.pack('<3H',1998,1998,1998);rates[22]=3
        advanced=bytearray(61);advanced[17]=80
        record={'angle_earth_ref':100,'common_msp_readback':{
            '111':rates.hex(),'94':advanced.hex(),'125':bytes(5).hex()}}
        definition=case()
        for stage in definition['stages']:
            stage.pop('requested_yaw_rate_dps');stage['channels'][1]=1984;stage['channels'][3]=1368
        result=stress_metrics(trace(rate=lambda t:-60.2976,pitch=75.22082388059701),definition,record=record)
        self.assertAlmostEqual(result['holds'][0]['yaw']['native_yaw_error_rms_dps'],0,places=9)
        self.assertFalse(result['flags']['unwanted_world_yaw_spin'])

    def test_entry_uses_motor_duty_velocity_and_acceleration(self):
        result=entry_statistics(trace(vx=lambda t:10+2*t,vz=lambda t:.3),1,3,channels=[1500,1500,1715,1500])
        self.assertAlmostEqual(result['throttle_demand']['normalized_above_mincheck'],.7)
        self.assertAlmostEqual(result['motors']['0']['modeled_duty_mean'],.715)
        self.assertAlmostEqual(result['finite_difference_3d_acceleration_rms_m_s2'],2)
        self.assertAlmostEqual(result['horizontal_speed_m_s']['mean'],14)
        self.assertAlmostEqual(result['vz_m_s']['mean'],.3)

    def test_prefix_is_censored_without_extrapolation(self):
        result=stress_metrics(trace(),case(),observed_end_s=2)
        self.assertTrue(result['observed_end_censored']);self.assertEqual(result['observed_end_s'],2)
        self.assertEqual(len(result['holds']),1)
        with self.assertRaises(ValueError):stress_metrics(trace(),case(),observed_end_s=-1)

    def test_failed_prefix_stops_at_first_disarm_before_later_spin(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary);rows=trace(rate=lambda t:0 if t<2 else 200)
            with (path/'plant.csv').open('w') as stream:
                writer=csv.DictWriter(stream,list(rows[0]));writer.writeheader();writer.writerows(rows)
            definition=case();definition['partial_program']=True
            (path/'comparison-run.json').write_text(json.dumps({'pass':False,'cases':[definition]}))
            packets=[{'sim_time_s':1,'channels':[1500]*4+[1900]},
                     {'sim_time_s':2,'channels':[1500]*4+[1000]},
                     {'sim_time_s':3,'channels':[1500]*4+[1000]}]
            (path/'rc-packets.json').write_text(json.dumps(packets))
            result=analyze_trial(path);observed=result['cases'][0]
            self.assertEqual(observed['observed_end_s'],2)
            self.assertTrue(observed['observed_end_censored'])
            self.assertEqual(observed['observed_end_reason'],'first_recorded_disarm_cap')
            self.assertIsNone(observed['entry_statistics'])
            self.assertIn('beyond',observed['entry_statistics_reason'])
            self.assertFalse(result['flags']['unwanted_world_yaw_spin'])

    def test_unreached_pulse_has_no_fabricated_operating_point_entry(self):
        definition=case();definition['stages']=definition['stages'][:2]
        definition['partial_program']=True;definition['end_sim_time_s']=2
        definition['stages'][0]['channels'][2]=1345
        definition['stages'][1]['channels']=[1440,1986,1715,1500]
        result=stress_metrics(trace(),definition)
        self.assertIsNone(result['entry_statistics'])
        self.assertIn('not reached',result['entry_statistics_reason'])
        self.assertTrue(result['observed_end_censored'])

    def test_post_disarm_euler_singularity_cannot_erase_valid_prefix(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary);rows=trace()
            for row in rows:
                if row['sim_time_s']>2:row['pitch_deg']=90
            with (path/'plant.csv').open('w') as stream:
                writer=csv.DictWriter(stream,list(rows[0]));writer.writeheader();writer.writerows(rows)
            definition=case();definition['partial_program']=True
            (path/'comparison-run.json').write_text(json.dumps({'pass':False,'cases':[definition]}))
            (path/'rc-packets.json').write_text(json.dumps([{'sim_time_s':2,'channels':[1500]*4+[1000]}]))
            result=analyze_trial(path)
            self.assertTrue(result['available']);self.assertEqual(result['cases'][0]['observed_end_s'],2)
            self.assertTrue(result['cases'][0]['observed_end_censored'])

    def test_partial_marker_stays_censored_even_without_earlier_end(self):
        definition=case();definition['partial_program']=True
        self.assertTrue(stress_metrics(trace(),definition)['observed_end_censored'])

    def test_trial_directory_and_failed_no_marker_boundary(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary);rows=trace()
            with (path/'plant.csv').open('w') as stream:
                writer=csv.DictWriter(stream,list(rows[0]));writer.writeheader();writer.writerows(rows)
            (path/'comparison-run.json').write_text(json.dumps({'pass':True,'cases':[case()]}))
            self.assertTrue(analyze_trial(path)['available'])
            (path/'comparison-run.json').write_text(json.dumps({'pass':False,'cases':[]}))
            self.assertFalse(analyze_trial(path)['available'])


if __name__=='__main__':unittest.main()
