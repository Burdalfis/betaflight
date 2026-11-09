"""Focused envelope math checks: frames, actual operation and matched clocks."""
import math
import unittest

from envelope_metrics import (body_to_euler_rates,failed_prefix_metrics,firmware_attitude_metrics,
                              matched_control_metrics,operating_metrics)


def definition(direction='cw',point='forward',shift=0,lead=1,pitch=20):
    target=60*math.cos(math.radians(pitch)) if direction=='cw' else -60*math.cos(math.radians(pitch)) if direction=='ccw' else 0
    stages=[]
    for name,start,duration in [('baseline',.5,1),('operating_lead',1.5,lead),('pulse',1.5+lead,1),('recovery',2.5+lead,1)]:
        angle=0 if name=='baseline' else pitch
        rate=target if name=='pulse' else 0
        stages.append({'name':name,'duration_s':duration,'actual_start_sim_time_s':start+shift,
            'channels':[1500,1500 if name=='baseline' else 1900,1345 if name=='baseline' else 1380,
                        1368 if rate>0 else 1632 if rate<0 else 1500],
            'requested_roll_deg':0,'requested_pitch_deg':angle,
            'requested_yaw_rate_dps':rate,'yaw_pid_target_frd_dps':rate,
            'stick_requested_yaw_rate_dps':60 if rate>0 else -60 if rate<0 else 0})
    return {'name':point+'_'+direction,'direction':direction,'operating_point':point,
            'kind':'control' if direction=='control' else 'yaw','stages':stages,
            'end_sim_time_s':3.5+lead+shift}


def trace(duration=6,shift=0,pitch=20,rate=lambda t:0,roll=lambda t:0,vx=lambda t:3,vz=lambda t:2):
    result=[];heading=0;x=z=0;previous_x=vx(0);previous_z=vz(0)
    for i in range(round(duration/.01)+1):
        time=i*.01;angle=roll(time);yaw_rate=rate(time)
        if i:
            heading+=yaw_rate*.01;x+=(vx(time)+previous_x)*.005;z+=(vz(time)+previous_z)*.005
        previous_x=vx(time);previous_z=vz(time)
        # Exact constant-Euler-RP yaw kinematics, converted FLU -> FRD.
        phi,theta=math.radians(angle),math.radians(pitch)
        psi=math.radians(yaw_rate)
        p=-psi*math.sin(theta);q=psi*math.sin(phi)*math.cos(theta);r=psi*math.cos(phi)*math.cos(theta)
        row={'sim_time_s':time+shift,'x_m':x,'y_m':0,'z_m':10+z,
            'vx_m_s':vx(time),'vy_m_s':0,'vz_m_s':vz(time),
            'roll_deg':angle,'pitch_deg':pitch,'yaw_deg':(heading+180)%360-180,
            'gyro_frd_x':p,'gyro_frd_y':-q,'gyro_frd_z':-r,
            'servo_pwm_us':1509,'joint_angle_deg':0,'joint_velocity_dps':0,
            'servo_target_deg':0,'servo_torque_nm':0,'actuator_valid':1,'pb0':1490,'pc0':0}
        for motor in range(3):
            row.update({f'pwm{motor}_us':1400,f'thrust{motor}_n':2,
                f'rotor{motor}_omega_rad_s':1000,f'rotor{motor}_terminal_current_a':5,
                f'rotor{motor}_battery_current_a':2})
        result.append(row)
    return result


class KinematicsTests(unittest.TestCase):
    def test_level_axes_and_bridge_pitch_sign(self):
        self.assertEqual(body_to_euler_rates(0,0,(0,0,0)),(0,0,0))
        self.assertAlmostEqual(body_to_euler_rates(0,0,(math.radians(5),0,0))[0],5)
        self.assertAlmostEqual(body_to_euler_rates(0,0,(0,math.radians(-7),0))[1],7)
        self.assertAlmostEqual(body_to_euler_rates(0,0,(0,0,math.radians(60)))[2],-60)

    def test_tilted_yaw_body_roll_is_intentional_not_euler_roll_motion(self):
        theta=math.radians(25)
        gyro=(math.radians(60)*math.sin(theta),0,math.radians(60)*math.cos(theta))
        roll,pitch,yaw=body_to_euler_rates(0,25,gyro)
        self.assertAlmostEqual(roll,0,places=12);self.assertAlmostEqual(pitch,0,places=12)
        self.assertAlmostEqual(yaw,-60,places=12)
        self.assertGreater(math.degrees(gyro[0]),25)

    def test_nonzero_roll_and_pitch_inverse_rotation_and_singularity(self):
        phi,theta=math.radians(20),math.radians(-15)
        fd,pd,yd=map(math.radians,(3,-7,40))
        flu=(fd-yd*math.sin(theta),pd*math.cos(phi)+yd*math.sin(phi)*math.cos(theta),
             -pd*math.sin(phi)+yd*math.cos(phi)*math.cos(theta))
        actual=body_to_euler_rates(20,-15,(flu[0],-flu[1],-flu[2]))
        for value,expected in zip(actual,(3,-7,40)):self.assertAlmostEqual(value,expected,places=11)
        with self.assertRaises(ValueError):body_to_euler_rates(0,90,(0,0,1))


class OperatingTests(unittest.TestCase):
    def test_achieved_speeds_distance_and_corrected_yaw_target(self):
        case=definition();rows=trace(rate=lambda t:-60 if 2.5<=t<=3.5 else 0)
        result=operating_metrics(rows,case)
        pre=result['windows']['pre_pulse']['kinematics'];pulse=result['windows']['pulse']
        self.assertAlmostEqual(pre['horizontal_speed_m_s']['mean'],3)
        self.assertAlmostEqual(pre['vz_m_s']['mean'],2)
        self.assertAlmostEqual(pre['body_forward_ground_speed_m_s']['mean'],
                               3*math.cos(math.radians(20))-2*math.sin(math.radians(20)),places=5)
        self.assertAlmostEqual(pulse['kinematics']['horizontal_path_m'],3)
        self.assertAlmostEqual(pulse['kinematics']['height_change_m'],2)
        self.assertAlmostEqual(pulse['kinematics']['path_3d_m'],math.sqrt(13))
        self.assertAlmostEqual(result['yaw']['target_yaw_rate_frd_dps'],60*math.cos(math.radians(20)))
        self.assertAlmostEqual(pulse['attitude_tracking']['euler_roll_pitch_rate_norm_dps']['sampled_peak_abs'],0,places=11)
        self.assertGreater(pulse['kinematics']['body_roll_rate_frd_dps']['sampled_peak_abs'],20)
        self.assertEqual(result['full_program_observations']['actuator_valid_sample_fraction'],1)

    def test_nonsteady_operation_reports_actual_change_not_desired_speed(self):
        result=operating_metrics(trace(vx=lambda t:t),definition('control'))
        pre=result['windows']['pre_pulse']['kinematics']['horizontal_speed_m_s']
        self.assertAlmostEqual(pre['mean'],2.35)
        self.assertAlmostEqual(pre['net_change_rate_m_s2'],1)
        self.assertIsNone(result['yaw'])

    def test_signed_current_and_servo_authority_occupancy(self):
        rows=trace();case=definition('control')
        for row in rows:
            if 2.5<=row['sim_time_s']<=3.5:
                row['rotor0_terminal_current_a']=-35.22;row['rotor0_battery_current_a']=-2
                row['servo_pwm_us']=1700;row['pwm1_us']=1050
        pulse=operating_metrics(rows,case)['windows']['pulse']['authority']
        self.assertEqual(pulse['motors'][0]['brake_limit_sample_fraction'],1)
        self.assertEqual(pulse['motors'][0]['drive_limit_sample_fraction'],0)
        self.assertEqual(pulse['servo']['endpoint_sample_fraction'],1)
        self.assertEqual(pulse['motor_lower_sample_fraction'],1)
        self.assertAlmostEqual(pulse['modeled_motor_bus_current_a']['integral_as'],2)

    def test_estimator_bias_is_separate_from_estimated_target_tracking(self):
        rows=trace(pitch=30);case=definition('control',pitch=20)
        telemetry=[{'sim_time_s':i*.1,'firmware_roll_deg':0,'firmware_pitch_deg':20} for i in range(5,51)]
        result=firmware_attitude_metrics(rows,telemetry,case)
        pulse=result['windows']['pulse']['observations']
        self.assertTrue(result['available'])
        self.assertAlmostEqual(pulse['pitch_firmware_minus_ground_truth_deg']['sample_mean'],-10)
        self.assertAlmostEqual(pulse['pitch_firmware_minus_commanded_deg']['sample_mean'],0)
        self.assertAlmostEqual(pulse['pitch_ground_truth_minus_commanded_deg']['sample_mean'],10)
        self.assertFalse(firmware_attitude_metrics(rows,[],case)['available'])

    def test_invalid_data_and_incomplete_program_rejected(self):
        rows=trace();rows[200]['vx_m_s']=float('nan')
        with self.assertRaises(ValueError):operating_metrics(rows,definition())
        case=definition();case['stages'][2]['duration_s']=float('nan')
        with self.assertRaises(ValueError):operating_metrics(trace(),case)


class MatchedTests(unittest.TestCase):
    def test_distinct_trace_clocks_noop_jitter_and_tilted_yaw_cancellation(self):
        target=definition();control=definition('control',shift=100,lead=1.07)
        rows=trace(rate=lambda t:-60 if 2.5<=t<=3.5 else 0)
        base=trace(shift=100)
        result=matched_control_metrics(rows,target,base,control,common_duration_s=1.8)
        self.assertAlmostEqual(result['alignment']['control_equivalent_onset_s'],102.5)
        self.assertAlmostEqual(result['alignment']['control_age_alignment_delta_s'],-.07)
        self.assertAlmostEqual(result['phases']['pulse']['roll_pitch_norm_deg']['sampled_peak_abs'],0)
        self.assertAlmostEqual(result['phases']['pulse']['euler_roll_pitch_rate_norm_dps']['sampled_peak_abs'],0,places=11)
        self.assertGreater(result['phases']['pulse']['body_roll_rate_frd_dps']['sampled_peak_abs'],20)
        self.assertEqual(result['common_duration_s'],1.8)

    def test_preexisting_drift_is_exposed_and_non_yaw_command_mismatch_rejected(self):
        target=definition();control=definition('control')
        result=matched_control_metrics(trace(roll=lambda t:.2*t),target,trace(),control)
        self.assertAlmostEqual(result['pre_event_diagnostics']['roll_deg']['pre_differential_slope_per_s'],.2)
        control['stages'][1]['channels'][2]+=1
        with self.assertRaises(ValueError):matched_control_metrics(trace(),target,trace(),control)


class FailedPrefixTests(unittest.TestCase):
    def test_abort_uses_rc_markers_disarm_bound_and_has_no_full_step_metrics(self):
        case=definition();rows=trace(roll=lambda t:0 if t<2.5 else -40*(t-2.5)/.6)
        packets=[{'case':case['name'],'stage':stage['name'],'sim_time_s':stage['actual_start_sim_time_s'],
                  'wall_monotonic':100+stage['actual_start_sim_time_s'],'channels':[1500]*4+[1900]}
                 for stage in case['stages'][:3]]
        packets.append({'case':case['name'],'stage':'pulse','sim_time_s':3.1,'wall_monotonic':103.1,'channels':[1500]*4+[1900]})
        packets.append({'case':'disarmed','stage':'disarmed','sim_time_s':3.12,'wall_monotonic':103.12,'channels':[1500]*4+[1000]})
        result=failed_prefix_metrics(rows,case,packets)
        self.assertTrue(result['available']);self.assertFalse(result['completed_program'])
        self.assertAlmostEqual(result['observed_yaw_duration_s'],.62)
        self.assertAlmostEqual(result['observed_end_s'],3.12)
        self.assertIsNone(result['yaw']);self.assertFalse(result['full_step_metrics_available'])
        self.assertIsNotNone(result['observed_angle_limit_first_time_s'])
        self.assertLess(result['windows']['observed_yaw_prefix']['attitude_tracking']['roll_actual_deg']['sampled_min'],-40)

    def test_failed_before_program_is_retained_without_invented_metrics(self):
        result=failed_prefix_metrics(trace(),definition(),[])
        self.assertFalse(result['available']);self.assertIsNone(result['yaw'])


if __name__=='__main__':unittest.main()
