"""Measured-map C transport and opt-in actual allocation integration tests."""
import copy
import tempfile
import unittest
import numpy as np
from dynamic_native import NativeDynamic, dynamic_parameters, load_protocol, run_configuration, legacy


def profile():
    return {'schema_version':1,'kind':'tricopter_basket_measured_angle_mean_map',
        'source_position':'signed PB0-PC0 only','angle_units':'radians','operator_defined_reference':True,
        'knots':[{'corrected_counts':c,'angle_rad':a,'half_branch_separation_rad':h}
                 for c,a,h in [(1000,.9,.01),(1500,.1,.03),(2000,-.6,.02)]]}


class MeasuredAngleNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix='tricopter-measured-angle-')
        cls.native=NativeDynamic(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def test_real_c_vector_map_and_unavailable_mask(self):
        p=profile();out=self.native.angles(p,[1000,1250,1500,1750,2000,999,2001,np.nan])
        np.testing.assert_allclose(out['angle_rad'][:5],[.9,.5,.1,-.25,-.6],atol=1e-7)
        np.testing.assert_allclose(out['half_branch_separation_rad'][:5],[.01,.02,.03,.025,.02],atol=1e-7)
        np.testing.assert_array_equal(out['valid'],[1,1,1,1,1,0,0,0])
        np.testing.assert_array_equal(out['angle_rad'][5:],0)

    def test_malformed_or_unreferenced_profiles_fail_instead_of_pwm_fallback(self):
        for kind in ('reference','counts','angle','envelope','source'):
            p=profile()
            if kind=='reference':p['operator_defined_reference']=False
            if kind=='counts':p['knots'][1]['corrected_counts']=1000
            if kind=='angle':p['knots'][1]['angle_rad']=2
            if kind=='envelope':p['knots'][1]['half_branch_separation_rad']=-.1
            if kind=='source':p['source_position']='PWM'
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.native.angles(p,[1500])

    def test_measured_curve_is_consumed_by_current_and_horizon_state_in_c(self):
        p=load_protocol();base=legacy.load_protocol();d=dynamic_parameters(p)
        _,c,q,inputs=run_configuration(p,base['airframes'][0],p['scenarios'][0],p['experiments'][2])
        inputs={k:v[:1501].copy() for k,v in inputs.items()}
        d[31]=0 # Only held/current and nominal candidates; avoids legacy grid assumptions.
        result=self.native.run(c,q,d,angle_profile=profile(),**inputs)
        available=result['angle_flags'][:,0].astype(bool)
        mapped=self.native.angles(profile(),result['numeric'][:,35])
        np.testing.assert_array_equal(available,mapped['valid'])
        np.testing.assert_allclose(result['numeric'][available,34],mapped['angle_rad'][available],rtol=0,atol=0)
        np.testing.assert_allclose(result['angle_numeric'][available,0],mapped['half_branch_separation_rad'][available],rtol=0,atol=0)
        used=result['dynamic_integer'][:,8].astype(bool)
        self.assertGreater(used.sum(),100)
        np.testing.assert_array_equal(result['angle_flags'][used,1],1)
        expected=self.native.angles(profile(),result['dynamic_numeric'][used,36])
        np.testing.assert_allclose(result['dynamic_numeric'][used,1],expected['angle_rad'],rtol=0,atol=0)
        actual=self.native.angles(profile(),result['numeric'][:,14])
        np.testing.assert_allclose(result['numeric'][:,15],actual['angle_rad'],rtol=0,atol=0)
        self.assertFalse(result['flags'][:,41:43].any())
        self.assertFalse(result['dynamic_integer'][:,6:8].any())
        for row in np.flatnonzero(used)[::100]:
            valid=result['candidate_integer'][row,:,2].astype(bool)
            costs=result['candidate_numeric'][row,valid,27]
            self.assertLessEqual(result['dynamic_numeric'][row,4],costs.min()+d[33]+1e-12)
        # Command-only prediction remains causal: changing future requests cannot
        # change any earlier controller or map output.
        changed=copy.deepcopy(inputs);changed['requested_response'][1300:]=[1.1,.1,-.1,-.15]
        other=self.native.run(c,q,d,angle_profile=profile(),**changed)
        for key in result:np.testing.assert_array_equal(result[key][:1300],other[key][:1300])

    def test_unsupported_measured_range_is_rejected(self):
        p=load_protocol();base=legacy.load_protocol();d=dynamic_parameters(p);d[31]=0
        _,c,q,inputs=run_configuration(p,base['airframes'][0],p['scenarios'][0],p['experiments'][2])
        inputs={k:v[:10] for k,v in inputs.items()}
        a=profile();a['knots'][0]['angle_rad']=.4
        with self.assertRaises(ValueError):self.native.run(c,q,d,angle_profile=a,**inputs)

if __name__=='__main__':unittest.main()
