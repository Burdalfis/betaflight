"""Actual-C finite-horizon priority tests plus independent active-face oracles."""
import copy
import tempfile
import unittest
import numpy as np
from dynamic_native import NativeDynamic,load_protocol,dynamic_parameters,run_configuration,legacy
from dynamic_audit import snapshot,validate_priority_every_row
from dynamic_reference import box_primary_oracle,lexicographic_oracle,analytic_motor_endpoint,static_request_classification
from dynamic_legacy import verify_legacy


class DynamicReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix='tricopter-dynamic-');cls.native=NativeDynamic(cls.tmp.name)
        cls.p=load_protocol();cls.base=legacy.load_protocol();cls.d=dynamic_parameters(cls.p)
        _,cls.c,cls.q,cls.inputs=run_configuration(cls.p,cls.base['airframes'][0],cls.p['scenarios'][0],cls.p['experiments'][2])
        cls.inputs={k:v[:1501].copy() for k,v in cls.inputs.items()}
        cls.result=cls.native.run(cls.c,cls.q,cls.d,**cls.inputs)

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def test_original_abi_and_default_callbacks_are_bit_exact_to_frozen_evidence(self):
        result=verify_legacy(self.native,self.d)
        self.assertTrue(result['original_and_optin_callback_default_modes_bit_exact']);self.assertEqual(len(result['mode_traces']),3)

    def test_every_available_row_obeys_global_priority_bands_and_issued_commands(self):
        report=validate_priority_every_row(self.result,self.d)
        self.assertGreater(report['priority_available_rows'],1000);self.assertTrue(report['global_staged_selection_pass'])
        self.assertFalse(self.result['dynamic_integer'][:,6:8].any())

    def test_independent_faces_motor_forward_and_costs_for_all_sampled_candidates(self):
        for row in (100,300,340,1300,1340,1500):
            audit=snapshot(self.result,row,int(self.inputs['time_us'][row]),self.c,self.d,'unit')
            self.assertTrue(audit['priority_available'])
            self.assertTrue(any(c['valid'] for c in audit['candidates']))

    def test_global_tolerance_chain_cannot_ratchet_primary_priority(self):
        tolerance=1e-6;high=np.asarray([0,.75*tolerance,1.5*tolerance]);yaw=np.asarray([1,.1,0]);move=np.zeros(3)
        result=lexicographic_oracle(high,yaw,move,[1,1,1],tolerance,0,0)
        self.assertEqual(result['selected_indices'].tolist(),[1])
        order=[2,0,1];reversed_result=lexicographic_oracle(high[order],yaw[order],move[order],[1,1,1],tolerance,0,0)
        self.assertEqual(order[int(reversed_result['selected_indices'][0])],1)
        a=np.diag([1.,2.,3.]);oracle=box_primary_oracle(a,[2,-1,1],[0,0,0],[1,1,1],[1,1,1])
        np.testing.assert_allclose(oracle['effort'],[1,0,1/3],atol=1e-12)

    def test_per_airframe_pure_actuator_static_classification_and_high3_proofs(self):
        expected={'synthetic_A':[(True,.3539,None),(False,-.40693,'tail_direction_below_minimum')],
                  'synthetic_B':[(False,.58371,'tail_direction_above_maximum'),(False,-.60224,'tail_direction_below_minimum')]}
        for frame in self.base['airframes']:
            _,controller,_,_=run_configuration(self.p,frame,self.p['scenarios'][0],self.p['experiments'][2])
            effective=controller.astype(np.float32).astype(float);matrix=effective[:16].reshape(4,4)
            for yaw,(feasible,direction,violation) in zip((.18,-.18),expected[frame['name']]):
                target=np.asarray([1.15,0,0,yaw],np.float32).astype(float)
                result=static_request_classification(matrix,target,effective[16:19],effective[19],effective[20])
                self.assertEqual(result['full4_actuator_feasible'],feasible)
                self.assertAlmostEqual(result['required_mathematical_direction_rad'],direction,delta=2e-5)
                if violation:self.assertIn(violation,result['violated_constraints'])
                self.assertEqual(result['high3_static_feasibility']['status'],'proven_feasible')
                np.testing.assert_allclose(matrix@result['required_controls'],target,atol=1e-14)
            high=static_request_classification(matrix,[3,.8,-.6,1.2],effective[16:19],effective[19],effective[20])
            self.assertEqual(high['high3_static_feasibility']['status'],'proven_infeasible')
            self.assertLess(high['high3_static_feasibility']['collective_upper_bound'],3)
        unknown=static_request_classification(np.eye(4),[0,0,1.1,0],[1,1,1],-.4,.55)
        self.assertEqual(unknown['high3_static_feasibility']['status'],'unknown')
        self.assertTrue(all('static_request_feasible' not in w and not w['name'].startswith('feasible_') for w in self.p['transition_windows']))

    def test_pending_servo_copy_does_not_mutate_live_predictor_and_future_suffix_is_causal(self):
        np.testing.assert_array_equal(self.result['numeric'][:,14],self.result['numeric'][:,35])
        changed=copy.deepcopy(self.inputs);cut=1300;changed['requested_response'][cut:]=[1.4,.2,-.1,-.25]
        changed['drop_generation'][cut:,0]=1
        other=self.native.run(self.c,self.q,self.d,**changed)
        for key in self.result:np.testing.assert_array_equal(self.result[key][:cut],other[key][:cut])
        np.testing.assert_array_equal(self.result['numeric'][cut,4:26],other['numeric'][cut,4:26])

    def test_sterile_servo_horizon_matches_independent_delay_play_lag_oracle(self):
        row=100;candidates=self.result['candidate_numeric'][row];valid=self.result['candidate_integer'][row,:,2].astype(bool)
        pwm=self.c[29:32].astype(np.float32).astype(float);counts=self.c[32:35].astype(np.float32).astype(float)
        current=1600.;played=1420.;halfplay=float(np.float32(self.c[35]))/2
        dt=(self.d[0]-self.c[36])/1e6
        for candidate in candidates[valid]:
            internal=np.clip(played,candidate[0]-halfplay,candidate[0]+halfplay)
            target=np.interp(internal,pwm,counts)
            expected=analytic_motor_endpoint(current,target,dt,float(np.float32(self.c[38])),float(np.float32(self.c[37])),
                                            float(np.float32(self.c[40])),float(np.float32(self.c[39])))
            self.assertLess(abs(expected-candidate[1]),.0005)

    def test_hidden_plant_truth_cannot_change_dynamic_commands_with_same_rpm_stream(self):
        plant=self.q.copy();plant[:16]*=np.repeat([1.12,.8,1.1,.75],4);plant[24:26]*=1.8;plant[23]=15000
        changed=self.native.run(self.c,plant,self.d,**self.inputs)
        np.testing.assert_array_equal(changed['numeric'][:,26:83],self.result['numeric'][:,26:83])
        for key in ('dynamic_numeric','dynamic_integer','candidate_numeric','candidate_integer'):
            np.testing.assert_array_equal(changed[key],self.result[key])
        self.assertGreater(np.max(abs(changed['numeric'][:,4:8]-self.result['numeric'][:,4:8])),.01)

    def test_state_loss_uses_explicit_static_fallback_not_zero_rpm(self):
        _,c,q,inputs=run_configuration(self.p,self.base['airframes'][0],self.p['scenarios'][2],self.p['experiments'][2])
        inputs={k:v[:1901] for k,v in inputs.items()};out=self.native.run(c,q,self.d,**inputs)
        mask=(inputs['time_us']>=1520000)&(inputs['time_us']<1800000)
        self.assertFalse(out['dynamic_integer'][mask,8].any());self.assertTrue(np.all(out['dynamic_integer'][mask,0]&4))
        self.assertTrue(np.any(out['numeric'][mask,69:72]>0));self.assertTrue(out['flags'][mask,0].all())

    def test_no_direction_authority_and_all_singular_candidates_are_flagged_fallbacks(self):
        inputs={k:v[:101].copy() for k,v in self.inputs.items()};inputs['requested_response'][:]=[1.15,0,0,.18]
        c=self.c.copy();c[39:41]=1e-6
        no_authority=self.native.run(c,self.q,self.d,**inputs)
        unavailable=(no_authority['dynamic_integer'][:,0]&128)!=0
        self.assertTrue(unavailable.any());self.assertFalse(no_authority['dynamic_integer'][unavailable,8].any())
        c=self.c.copy();c[:16]=np.asarray([[1,1,0,0],[1,-1,0,0],[0,0,0,1],[0,0,1,0]]).ravel()
        inputs['requested_response'][:]=[.8,0,0,0]
        d=self.d.copy();d[31]=0
        singular=self.native.run(c,self.q,d,**inputs)
        self.assertFalse(singular['dynamic_integer'][:,8].any());self.assertTrue(np.any(singular['dynamic_integer'][:,0]&32))

    def test_candidate_order_and_lag_slew_endpoint_oracles(self):
        d=self.d.copy();d[14:31]=d[14:31][::-1]
        changed=self.native.run(self.c,self.q,d,**self.inputs);validate_priority_every_row(changed,d)
        np.testing.assert_allclose(changed['dynamic_numeric'][:,2:4],self.result['dynamic_numeric'][:,2:4],atol=2e-8)
        self.assertAlmostEqual(analytic_motor_endpoint(0,1000,.1,.1,.1,0,0),1000*(1-np.exp(-1)),places=10)
        self.assertAlmostEqual(analytic_motor_endpoint(0,10000,.01,.1,.1,1000,1000),10,places=10)

    def test_unavailable_horizon_and_malformed_original_transport_fail_before_memory_access(self):
        d=self.d.copy();d[0]=self.c[36] # Servo fixed delay; horizon must exceed it.
        with self.assertRaises(ValueError):self.native.run(self.c,self.q,d,**self.inputs)
        for method in (self.native.original,):
            with self.assertRaises(ValueError):method([1,2],self.q,**self.inputs)
            changed=copy.deepcopy(self.inputs);changed['requested_response']=np.zeros((1,4))
            with self.assertRaises(ValueError):method(self.c,self.q,**changed)
            changed=copy.deepcopy(self.inputs);changed['modes'][:]=4
            with self.assertRaises(ValueError):method(self.c,self.q,**changed)
        changed=copy.deepcopy(self.inputs);changed['requested_response'][:]=np.nan
        with self.assertRaises(ValueError):self.native.run(self.c,self.q,self.d,**changed)


if __name__=='__main__':unittest.main()
