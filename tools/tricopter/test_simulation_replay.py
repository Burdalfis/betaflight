"""Causal integration tests for the actual C coordinator and synthetic plant."""
import copy
import tempfile
import unittest
import numpy as np
from simulation_native import NativeSimulation,load_protocol,pack_run,tick_inputs
from simulation_replay import fixture_inputs


class SimulationReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix='tricopter-simulation-');cls.native=NativeSimulation(cls.tmp.name);cls.p=load_protocol()
        cls.c,cls.q,mode=pack_run(cls.p,cls.p['airframes'][0],cls.p['scenarios'][0],cls.p['experiments'][1])
        cls.inputs={k:v[:1001].copy() for k,v in tick_inputs(cls.p,cls.p['scenarios'][0],mode).items()}

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def run_case(self,inputs=None,c=None,q=None):return self.native.run(self.c if c is None else c,self.q if q is None else q,**(self.inputs if inputs is None else inputs))

    def test_commands_only_affect_next_interval_and_capture_delay_is_aged(self):
        out=self.run_case();n,u,b=out['numeric'],out['integer'],out['flags']
        self.assertTrue((n[200,17:20]==0).all());self.assertTrue((n[200,26:29]>0).all());self.assertTrue((n[201,17:20]>0).all())
        self.assertTrue((n[:200,4:8]==0).all())
        for motor in range(3):
            delivered=b[:,31+motor].astype(bool)
            self.assertTrue(np.all(u[delivered,9+motor]==self.q[62]))
            self.assertTrue(np.all((u[delivered,18+motor]-u[delivered,15+motor])==self.q[62]))
            self.assertGreater(np.count_nonzero(b[:200,16+motor]),20) # New equal-zero frames refresh by identity.

    def test_future_request_and_fault_suffix_cannot_change_past_or_same_tick_truth(self):
        before=self.run_case();changed=copy.deepcopy(self.inputs);cut=600
        changed['requested_response'][cut:]=[2.2,.3,-.4,.8];changed['drop_generation'][cut:,0]=1;changed['outlier_generation'][cut:,1]=1
        after=self.run_case(changed)
        for key in before:np.testing.assert_array_equal(before[key][:cut],after[key][:cut])
        np.testing.assert_array_equal(before['numeric'][cut,4:26],after['numeric'][cut,4:26])
        self.assertGreater(np.max(abs(before['numeric'][cut:,26:33]-after['numeric'][cut:,26:33])),.1)

    def test_plant_truth_does_not_leak_with_identical_sensor_stream(self):
        before=self.run_case();plant=self.q.copy()
        plant[:16]*=np.repeat([1.15,.8,1.1,.7],4)
        plant[24:26]*=2 # Independent truth-servo lag constants.
        plant[23]=17000 # Independent truth-servo command delay.
        plant[44:47]*=1.4;plant[47:50]*=.7;plant[59]*=1.25
        after=self.run_case(q=plant)
        np.testing.assert_array_equal(before['numeric'][:,17:23],after['numeric'][:,17:23])
        np.testing.assert_array_equal(before['numeric'][:,26:83],after['numeric'][:,26:83])
        np.testing.assert_array_equal(before['integer'],after['integer'])
        np.testing.assert_array_equal(before['flags'],after['flags'])
        self.assertGreater(np.max(abs(before['numeric'][:,4:8]-after['numeric'][:,4:8])),.01)
        self.assertGreater(np.max(abs(before['numeric'][:,14]-after['numeric'][:,14])),1)

    def test_matched_servo_instances_and_mismatch_are_separate(self):
        out=self.run_case();np.testing.assert_array_equal(out['numeric'][:,14],out['numeric'][:,35])
        plant=self.q.copy();plant[24:26]*=1.8
        mismatch=self.run_case(q=plant)
        self.assertGreater(np.max(abs(mismatch['numeric'][:,14]-mismatch['numeric'][:,35])),1)

    def test_packet_loss_outlier_reacquisition_reset_derivative_gate(self):
        c,q,inputs=fixture_inputs(self.p);out=self.native.run(c,q,**inputs);u,b=out['integer'],out['flags']
        stale=inputs['time_us']>=360000;stale&=inputs['time_us']<520000
        self.assertFalse(b[stale,5].any());self.assertFalse(b[stale,10].any())
        reacquired=(u[:,6]&1024)!=0
        self.assertTrue(reacquired.any());self.assertFalse(b[reacquired,10].any())
        self.assertFalse(b[b[:,31+1].astype(bool)&(out['numeric'][:,76]>20000),17].any())
        self.assertTrue(np.any(u[:,7]&16))
        self.assertTrue(b[-1,5])

    def test_analytic_motor_step_and_scalar_reaction_oracle(self):
        inputs=copy.deepcopy(self.inputs);inputs['modes'][:]=0
        controls=np.asarray([.4,.35,.3,0]);target=self.c[:16].reshape(4,4)@controls
        inputs['requested_response'][:]=target
        plant=self.q.copy();plant[29:32]=.1;plant[35:38]=0
        out=self.run_case(inputs,q=plant);n=out['numeric'];time=inputs['time_us']/1e6
        target_rpm=n[0,26:29]
        oracle=target_rpm[None,:]*(1-np.exp(-time[:,None]/.1))
        np.testing.assert_allclose(n[:,17:20],oracle,rtol=1e-10,atol=1e-9)
        omega=n[:,17:20]*np.pi/30;alpha=n[:,20:23]*np.pi/30
        reaction=np.sum(np.asarray([-1,1,-1])*(plant[44:47]*omega**2+plant[47:50]*alpha),axis=1)
        np.testing.assert_allclose(n[:,12],reaction,rtol=3e-7,atol=2e-8)

    def test_zero_reaction_makes_baseline_and_reaction_only_commands_identical(self):
        c,q=self.c.copy(),self.q.copy();c[49:55]=0;q[44:50]=0
        baseline=copy.deepcopy(self.inputs);baseline['modes'][:]=0
        comp=copy.deepcopy(self.inputs);comp['modes'][:]=1
        a,b=self.run_case(baseline,c,q),self.run_case(comp,c,q)
        np.testing.assert_array_equal(a['numeric'][:,26:34],b['numeric'][:,26:34])
        np.testing.assert_array_equal(a['numeric'][:,4:8],b['numeric'][:,4:8])

    def test_source_timestamp_wrap_and_uint64_plant_clock(self):
        base=self.run_case();inputs=copy.deepcopy(self.inputs);inputs['time_us']+=np.uint64(2**32-20000)
        shifted=self.run_case(inputs)
        np.testing.assert_array_equal(base['numeric'],shifted['numeric'])
        np.testing.assert_array_equal(base['integer'][:,:15],shifted['integer'][:,:15])
        np.testing.assert_array_equal(base['flags'],shifted['flags'])

    def test_lower_plant_rpm_cap_is_exposed_and_enforced(self):
        plant=self.q.copy();plant[60]=1000;out=self.run_case(q=plant)
        self.assertTrue(out['flags'][:,44:47].any());self.assertLessEqual(out['numeric'][:,17:20].max(),1000)
        self.assertGreater(out['numeric'][:,26:29].max(),1000)

    def test_malformed_transport_and_double_count_declarations_are_rejected(self):
        for key,value in [('time_us',np.array([])),('requested_response',np.zeros((2,4))),('modes',np.full(1001,4)),
                          ('drop_generation',np.ones((1001,2))),('requested_response',np.full((1001,4),np.nan))]:
            inputs=copy.deepcopy(self.inputs);inputs[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.run_case(inputs)
        inputs=copy.deepcopy(self.inputs);inputs['time_us'][2]+=1
        with self.assertRaises(ValueError):self.run_case(inputs)
        controller=self.c.copy();controller[70]=0
        with self.assertRaises(ValueError):self.run_case(c=controller)
        plant=self.q.copy();plant[67]=0
        with self.assertRaises(ValueError):self.run_case(q=plant)
        controller=self.c.copy();controller[71]=np.nan
        with self.assertRaises(ValueError):self.run_case(c=controller)


if __name__=='__main__':unittest.main()
