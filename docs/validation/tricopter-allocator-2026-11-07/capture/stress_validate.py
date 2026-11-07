import hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'src/main/flight/tricopter_allocator.c').is_file())
sys.path.insert(0,str(ROOT/'tools/tricopter'))
from allocator_native import NativeAllocator
from allocator_cases import load_protocol, virtual_problem
from allocator_reference import certificate, project_controls
OUT=ROOT/'.local-artifacts/tricopter/allocator-2026-11-07/stress'
OUT.mkdir(parents=True,exist_ok=True)
native=NativeAllocator(OUT/'build')
p=load_protocol()['direct_effectiveness']
rng=np.random.default_rng(701107)
records=[]
for mode in ['identity','coupled']:
    cfg=json.loads(json.dumps(p))
    if mode=='identity':
        cfg['effectiveness_matrix']=np.eye(4).tolist()
        cfg['response_scales']=[1,1,1,1]
    cfg['solver']['projected_gradient_tolerance']=1e-5
    problem=virtual_problem(cfg)
    requests=rng.uniform(-3,5,(512,4)).astype(np.float32)
    out=native.virtual_run(cfg,requests=requests)
    assert out['valid'].all()
    assert not out['calibrated'].any() and not out['for_flight'].any()
    gaps=[]; corrections=[]; costs=[]
    for i in range(len(requests)):
        c=certificate(problem['matrix'],requests[i],problem['scales'],out['controls'][i],problem['maximum'],problem['minimum_angle'],problem['maximum_angle'])
        gaps.append(c['global_objective_gap_upper_bound'])
        corrections.append(c['published_control_projection_correction'])
        costs.append(c['cost_at_certified_point'])
        assert np.allclose(out['achieved'][i],problem['matrix']@out['controls'][i],rtol=2e-6,atol=2e-6)
        if mode=='identity':
            expected=project_controls(requests[i],problem['maximum'],problem['minimum_angle'],problem['maximum_angle'])
            assert np.allclose(out['controls'][i],expected,rtol=2e-6,atol=2e-6)
    gaps=np.asarray(gaps)
    converged=out['converged']
    assert np.max(gaps[converged]) < 1e-3
    assert np.all(out['effort']>=0) and np.all(out['effort']<=np.asarray(cfg['maximum_effort'],np.float32))
    angle=out['tail_direction_rad']
    assert np.all(angle>=np.float32(cfg['tail_direction_min_rad'])) and np.all(angle<=np.float32(cfg['tail_direction_max_rad']))
    records.append({'name':mode,'requests':len(requests),'valid':int(out['valid'].sum()),'converged':int(converged.sum()),'maximum_certificate_gap':float(np.max(gaps)),
        'maximum_converged_certificate_gap':float(np.max(gaps[converged])),'maximum_publication_projection_correction':max(corrections),
        'maximum_cost':max(costs),'identity_euclidean_projection_oracle':mode=='identity'})
report={'seed':701107,'cases':records,'native_build':native.provenance,'calibrated':False,'for_flight':False,'hardware_accessed':False}
(OUT/'stress-validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(records))
