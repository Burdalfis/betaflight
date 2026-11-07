"""Exact default-path regression against immutable e7608a372 simulation evidence."""
import gzip
import json
import numpy as np
from simulation_native import ROOT,sha

PACKAGE=ROOT/'docs/validation/tricopter-simulation-2026-11-07'


def packaged(path):
    expected={name:digest for digest,name in (line.split('  ',1) for line in (PACKAGE/'checksums.sha256').read_text().splitlines())}
    relative=str(path.relative_to(ROOT))
    if relative not in expected or sha(path)!=expected[relative]:raise ValueError('Legacy evidence package hash mismatch: '+relative)
    return sha(path)


def verify_legacy(native,dynamic):
    fixture_path=PACKAGE/'response/matlab-parity-fixture.json';fixture_hash=packaged(fixture_path)
    f=json.loads(fixture_path.read_text());c,q=np.asarray(f['controller_parameters']),np.asarray(f['plant_parameters'])
    inputs={key:np.asarray(f[key]) for key in ('time_us','requested_response','modes','drop_generation','outlier_generation')}
    expected={key:np.asarray(f['expected_'+key]) for key in ('numeric','integer','flags')}
    original=native.original(c,q,**inputs);callback=native.run(c,q,dynamic,**inputs)
    for key in expected:
        np.testing.assert_array_equal(original[key],expected[key]);np.testing.assert_array_equal(callback[key],expected[key])
        if key=='numeric':
            np.testing.assert_array_equal(original[key].view(np.uint64),expected[key].astype(np.float64).view(np.uint64))
            np.testing.assert_array_equal(callback[key].view(np.uint64),expected[key].astype(np.float64).view(np.uint64))
    modes=[]
    for mode,name in [(0,'baseline'),(1,'reaction_only'),(3,'reaction_and_tail')]:
        path=PACKAGE/f'response/synthetic_A-matched-{name}.csv.gz';digest=packaged(path)
        with gzip.open(path,'rt') as stream:data=np.loadtxt(stream,delimiter=',',skiprows=1,max_rows=701)
        inp={'time_us':data[:,0],'requested_response':data[:,2:6],'modes':data[:,1],
             'drop_generation':np.zeros((len(data),3),np.uint8),'outlier_generation':np.zeros((len(data),3),np.uint8)}
        old=native.original(c,q,**inp);new=native.run(c,q,dynamic,**inp)
        wanted={'numeric':data[:,2:87],'integer':data[:,87:114],'flags':data[:,114:161]}
        for key in wanted:
            np.testing.assert_array_equal(old[key],wanted[key]);np.testing.assert_array_equal(new[key],wanted[key])
            if key=='numeric':
                np.testing.assert_array_equal(old[key].view(np.uint64),wanted[key].astype(np.float64).copy().view(np.uint64))
                np.testing.assert_array_equal(new[key].view(np.uint64),wanted[key].astype(np.float64).copy().view(np.uint64))
        modes.append({'mode':mode,'rows':len(data),'archive':str(path.relative_to(ROOT)),'sha256':digest,'bit_exact':True})
    return {'original_commit':'e7608a372','fixture_rows':len(inputs['time_us']),'fixture_sha256':fixture_hash,
            'original_and_optin_callback_default_modes_bit_exact':True,'mode_traces':modes}
