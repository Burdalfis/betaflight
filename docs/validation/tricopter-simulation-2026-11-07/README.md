# Integrated synthetic simulation validation — 2026-11-07

See [the architecture and model boundary](../../tricopter_simulation.md), [Python usage](../../../tools/tricopter/README.md) and [MATLAB adapter](../../../tools/tricopter/matlab/README.md). This package connects the actual portable C servo predictor, RPM observer, reaction equations and allocator through an offline coordinator. It identifies no aircraft parameters and enables no flight outputs.

The controller uses a configurable virtual-tail effectiveness map without CAD, CG, mass or body inertia. Synthetic motor lag/slew, an independent servo state, packet queues and plant-specific parameters provide the actuator/response plant. The scalar reaction is a declared additive yaw surrogate independent of virtual direction; both maps exclude that separately added contribution. This is not a body-flight-dynamics, gyroscopic or tilted-rotor model. Shared servo-model-family agreement validates integration, not independent actuator physics.

`response/` preserves the frozen protocol, source/compiler/script fingerprints, all **18 compressed traces**, four-axis metrics, plots and the MATLAB parity fixture. Two maps × three scenarios × three modes give **81,018 ticks**. Every command is valid and bounded; no allocation is unconverged. Actual-response errors include every row, including faults and infeasible requests. Prediction errors have explicit availability masks and **85.892–99.822% coverage**. Missing estimates cannot silently become successful zeros.

Counterfactual tests verify prior-command timing, delayed observation age, source-clock wrap, future-suffix invariance, hidden-truth separation, zero-reaction baseline equivalence, lower plant RPM caps and stale/horizon recovery. Plant derivatives are analytic truth inputs only to the plant reaction calculation; the controller uses qualified observer derivatives. Cross-axis tradeoffs are retained: the sample compensation reduces yaw error modestly, while other errors can increase. These are synthetic response-proxy results, not flight-performance claims.

`capture/` records **160 host/regression tests**, **47 Python/native-C tests**, strict/fast-math integration checks, five-module Cortex-M4F compile-only checks, unchanged active/model source fingerprints and a fresh replay reproduction. All 18 reproduced traces match after decompression. The reproduced parity fixture has identical inputs/outputs/source pins; its native build command contains the new output-library path, so each fixture's own byte hash is checked separately.

`matlab/` contains MATLAB R2026b output and a self-contained Simulink replay model. All **701 rows × 85 numeric / 27 integer / 47 flag columns** match the Python fixture exactly, and six malformed MEX transport cases are rejected. The consumed fixture byte hash matches `response/matlab-parity-fixture.json`. Every shared native C/header hash is compared before execution. Simulink is **signal playback only**, with a fixed discrete tick, interpolation off and final-value hold; the saved/reopened model reproduces all samples exactly. The MEX runner executes the actual C simulation.

From the repository root:

```sh
CC=gcc CXX=g++ src/test/run_tricopter_checks.sh
python3 -m unittest discover -s tools/tricopter -p 'test_*replay.py' -v
python3 tools/tricopter/simulation_replay.py --output .local-artifacts/tricopter/simulation-reproduction
sha256sum --check docs/validation/tricopter-simulation-2026-11-07/checksums.sha256
```

In MATLAB, from the repository root:

```matlab
root = pwd;
addpath(fullfile(root,'tools','tricopter','matlab'));
run_tricopter_simulation( ...
    fullfile(root,'docs','validation','tricopter-simulation-2026-11-07','response','matlab-parity-fixture.json'), ...
    fullfile(root,'.local-artifacts','tricopter','simulation-matlab-reproduction'), true);
```

Python needs a C11 compiler, NumPy and Matplotlib. MATLAB needs its C MEX toolchain; omit Simulink by passing `false`. Generated binaries/caches remain outside this evidence package. All maps, rotor coefficients, dynamics, limits and counts/direction relationships are explicit synthetic placeholders. PB0−PC0 remains the sole position source and real angle calibration remains unavailable. No hardware, flash, configuration, active source-list, actuator or measurement path was changed.
