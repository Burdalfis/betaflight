# Offline dynamic allocator validation — 2026-11-07

The [algorithm and validation boundary](../../tricopter_dynamic_allocator.md) describe the opt-in sampled finite-horizon allocator. It forecasts copied servo-model state, including its delayed command queue, and motor lag/slew from fresh RPM-observer state. Configurable collective/roll/pitch scales define a protected primary group; yaw and then movement are separate objectives within explicit numerical bands. Configuration uses the existing virtual-tail effectiveness map and actuator characterization, without CAD, CG or body inertia.

This package is offline synthetic evidence. **No flight-control output, hardware configuration, flashing or actuator test is involved. PB0−PC0 remains the sole position measurement source.** Direction/count relationships, maps, motor coefficients and dynamics are synthetic; physical angle calibration and loaded characterization remain open.

`response/` preserves all **24 compressed traces / 120,024 rows**, 24 plots, parameter/protocol/source fingerprints, metrics, sampled optimization audits and its own checksum manifest. Two synthetic airframes × four scenarios × three policies compare soft weighted, earlier vertical heuristic and dynamic priority allocation. The scenarios are ideal zero-reaction, matched dynamics, telemetry faults and model mismatch. All use the predeclared 40 ms horizon and 1 ms tick. The primary, yaw and movement numerical tie bands are each `1e-8`.

Every actual-response row is scored, including saturation and explicit fallback. Dynamic allocation is available on 38,754 of 40,008 dynamic-policy rows (87.622–100% per run). Its 1,254 fallback rows comprise 619 per telemetry-fault run plus four startup rows in each matched/mismatched run. Dynamic status and the old nominal static planner diagnostics are distinct. Plots distinguish the static direction goal, issued PWM, actual current direction and selected predicted horizon direction.

The NumPy oracle independently checks **1,336 accepted candidate problems at 80 sampled times**, using SVD/active-face enumeration, box-gap certificates, independent motor forward equations and global priority selection. This verifies accepted sampled candidate primary optima, not continuous-PWM optimality, rank-deficient maps or yaw minimization across the entire primary-tolerance motor sublevel set. Exported C servo forecasts are inputs; using the same model family for the plant is not independent physical validation.

The [response report](response/README.md) retains all policy metrics and static feasibility classifications. The positive-yaw actuator-only request is full-four-axis feasible on synthetic_A; its reverse request and both requests on synthetic_B violate direction limits. Collective/roll/pitch alone remain statically feasible in those windows. Classification excludes the separately added reaction surrogate and does not establish total-response or finite-horizon feasibility. The high-demand window exceeds both maps' collective support bounds.

The results include a transient tradeoff: synthetic_A's ideal positive-yaw window has normalized collective/roll/pitch RMS 0.013528 for the dynamic policy versus 0.010868 for soft weighted and 0.023688 for the earlier heuristic. The reverse window improves to 0.018642 versus 0.069800 and 0.037471. Endpoint priority with held commands does not guarantee improved actual tracking when the controller replans each tick. The scalar reaction remains a held additive yaw surrogate; candidate-dependent future inertial/gyroscopic torque and body-flight dynamics are outside this test.

`capture/` contains **177 passing host C tests**, **59 passing Python tests**, strict and fast-math logs/XML for the 14 dynamic and 18 coordinator tests, and six-module Cortex-M4F compile-only evidence. No execution-time, stack-use or hardware qualification is claimed. `execution-validation.json` pins sources, tools/tests and 25 unchanged original-model/active-path files relative to base `e7608a372`. Only the offline coordinator changes among existing `src/main` files; the new core has no active source-list entry or live caller.

`package-reproduction.json` verifies a fresh full replay: every decompressed trace byte, parameter, metric, sampled audit result and source pin matches. The compact audit's build command contains an output-library path, so frozen and reproduced audit JSON byte hashes differ; both own hashes and all semantic fields are checked. `classification-correction-validation.json` records correction of earlier blanket feasibility labels. That correction changes metadata only: all 24 decompressed traces, parameters, metrics and compiled C source hashes remain identical.

`matlab/matlab-dynamic-validation.json` records an independent MATLAB R2026b Optimization Toolbox audit: **88 candidate problems at five snapshots**, maximum continuous effort difference `2.04e-8`, continuous primary-cost difference `3.00e-7` (including float32 publication of the continuous effort solution), published-command forward-cost difference `1.43e-14`, and passing global priority selection. These oracle tolerances are distinct from the runtime tie bands. The raw log also proves a malformed null reference cost is rejected. This audit consumes the exact `response/candidate-audit.json` hash.

`matlab/legacy/` records compatibility separately: all **701 rows × 85 numeric / 27 integer / 47 flag columns** match the original fixture exactly; six malformed MEX transport cases are rejected; the saved/reopened Simulink signal replay reproduces every sample. Native/Python checks also compare original modes 0/1/3 to the previous frozen traces bit-for-bit, including signed zero. The existing MEX executes the original simulation ABI; this does not execute the new dynamic API in Simulink. Binaries and caches are excluded.

From the repository root:

```sh
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 -m unittest discover -s tools/tricopter -p 'test_*replay.py' -v
python3 tools/tricopter/dynamic_replay.py --output .local-artifacts/tricopter/dynamic-reproduction
sha256sum --check docs/validation/tricopter-dynamic-2026-11-07/checksums.sha256
```

In MATLAB, from the repository root:

```matlab
root = pwd;
addpath(fullfile(root,'tools','tricopter','matlab'));
package = fullfile(root,'docs','validation','tricopter-dynamic-2026-11-07');
validate_tricopter_dynamic_allocator( ...
    fullfile(package,'response','candidate-audit.json'), ...
    fullfile(root,'.local-artifacts','tricopter','dynamic-matlab-reproduction'));
run_tricopter_simulation( ...
    fullfile(package,'matlab','legacy','fixture-input.json'), ...
    fullfile(root,'.local-artifacts','tricopter','dynamic-legacy-matlab'),true);
```

Python requires a C11 compiler, NumPy and Matplotlib. The independent MATLAB audit requires Optimization Toolbox; the legacy MEX check requires a C MEX toolchain and its optional signal replay requires Simulink. See [Python usage](../../../tools/tricopter/README.md) and [MATLAB usage](../../../tools/tricopter/matlab/README.md).
