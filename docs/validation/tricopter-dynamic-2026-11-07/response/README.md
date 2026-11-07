# Offline dynamic priority allocation

The opt-in controller evaluates copied servo-predictor state at a declared horizon for bounded PWM samples. Explicit motor lag/slew gives reachable effort boxes; each accepted direction has a full-rank primary three-axis box least-squares solution over 27 active faces. Bounded RPM commands are inverted and forwarded again, and the resulting published-command costs determine selection.

Selection uses global passes: high3 collective/roll/pitch cost first, yaw within the global primary tolerance band, then movement within both bands. This guarantee covers accepted sampled candidate primary optima; it is not a continuous-PWM optimum, rank-deficient-map solution or yaw optimization over the entire primary-tolerance motor sublevel set. Numerical tie bands are explicit.

All configurations remain synthetic and need no CAD/CG/body-inertia model. Mathematical direction is not a calibrated rotor angle. The current observer state and copied predictor history are the only dynamic state inputs; future plant truth is never supplied. State/authority loss degrades explicitly to the static nominal policy. Original soft-planner flags remain nominal diagnostics, distinct from dynamic used/priority/status fields.

Whole-wave errors include high demands and every actual row. Predeclared yaw transition windows have no blanket feasibility label. Each frame is classified independently by its unique controller-B inverse of the raw actuator-only request, with the separate reaction contribution excluded. This is not actual total-response feasibility under reaction or plant-map mismatch, and it does not establish motor/servo horizon reachability. High3 static feasibility is separately proved by a witness or a collective support bound; otherwise it is unknown. Actual performance under mismatch/faults is descriptive and is not guaranteed by the predicted sampled-priority contract. Lower predicted horizon primary cost does not imply lower actual primary RMS at every transition or every intermediate instant; the tables retain those tradeoffs.

| Controller map | Predeclared yaw transition | Pure actuator full4 feasible | Required direction (rad) | Required tail / FR / FL effort | High3 static proof |
|---|---|---|---:|---|---|
| synthetic_A | yaw_positive_transition | True | 0.353915 | 0.42783 / 0.45695 / 0.349381 | proven_feasible |
| synthetic_A | yaw_reverse_transition | False | -0.406929 | 0.456914 / 0.481612 / 0.338646 | proven_feasible |
| synthetic_B | yaw_positive_transition | False | 0.583708 | 0.396191 / 0.35356 / 0.489922 | proven_feasible |
| synthetic_B | yaw_reverse_transition | False | -0.602237 | 0.367785 / 0.3224 / 0.522166 | proven_feasible |

| Airframe / scenario / policy | Whole high3 RMS | Positive-yaw window high3 RMS | Reverse-yaw window high3 RMS | Dynamic used coverage |
|---|---:|---:|---:|---:|
| synthetic_A / ideal_zero_reaction / soft_weighted | 1.3595 | 0.010868 | 0.0698 | off |
| synthetic_A / ideal_zero_reaction / vertical_heuristic | 1.3426 | 0.023688 | 0.037471 | off |
| synthetic_A / ideal_zero_reaction / dynamic_priority | 0.90582 | 0.013528 | 0.018642 | 100.0% |
| synthetic_A / matched / soft_weighted | 1.3663 | 0.012127 | 0.061048 | off |
| synthetic_A / matched / vertical_heuristic | 1.3504 | 0.034531 | 0.045886 | off |
| synthetic_A / matched / dynamic_priority | 0.90447 | 0.017131 | 0.021046 | 99.9% |
| synthetic_A / telemetry_faults / soft_weighted | 1.3621 | 0.011463 | 0.060941 | off |
| synthetic_A / telemetry_faults / vertical_heuristic | 1.3459 | 0.035704 | 0.043561 | off |
| synthetic_A / telemetry_faults / dynamic_priority | 1.0911 | 0.017414 | 0.020792 | 87.6% |
| synthetic_A / model_mismatch / soft_weighted | 1.4008 | 0.095986 | 0.13207 | off |
| synthetic_A / model_mismatch / vertical_heuristic | 1.3827 | 0.092247 | 0.11224 | off |
| synthetic_A / model_mismatch / dynamic_priority | 0.94799 | 0.098989 | 0.10732 | 99.9% |
| synthetic_B / ideal_zero_reaction / soft_weighted | 1.8506 | 0.068064 | 0.34256 | off |
| synthetic_B / ideal_zero_reaction / vertical_heuristic | 1.8208 | 0.07054 | 0.31197 | off |
| synthetic_B / ideal_zero_reaction / dynamic_priority | 1.0445 | 0.021806 | 0.019065 | 100.0% |
| synthetic_B / matched / soft_weighted | 1.8547 | 0.40417 | 0.22752 | off |
| synthetic_B / matched / vertical_heuristic | 1.8251 | 0.3841 | 0.20252 | off |
| synthetic_B / matched / dynamic_priority | 1.0436 | 0.012828 | 0.021759 | 99.9% |
| synthetic_B / telemetry_faults / soft_weighted | 1.8522 | 0.32972 | 0.23314 | off |
| synthetic_B / telemetry_faults / vertical_heuristic | 1.8227 | 0.30396 | 0.20797 | off |
| synthetic_B / telemetry_faults / dynamic_priority | 1.4096 | 0.040209 | 0.021759 | 87.6% |
| synthetic_B / model_mismatch / soft_weighted | 1.9281 | 0.49762 | 0.34519 | off |
| synthetic_B / model_mismatch / vertical_heuristic | 1.8948 | 0.47323 | 0.31626 | off |
| synthetic_B / model_mismatch / dynamic_priority | 1.0695 | 0.10325 | 0.1062 | 99.9% |

Full-time traces contain original state/response fields plus dynamic summaries. Full candidate arrays are retained only at predeclared audit times. Independent NumPy SVD/face enumeration, box-gap certificates, motor forward equations and global selection checks audit accepted candidates; the compact `candidate-audit.json` additionally supports independent MATLAB lsqlin checks.

The original simulation ABI and default callback paths reproduce the frozen 701-row fixture and mode0/1/3 traces bit-for-bit, including signed zero. The earlier e7608a372 evidence is preserved; this opt-in version has separate source/protocol pins. The old nominal direction goal is labeled separately from issued PWM and the selected predicted horizon direction.

Reproduce with `python3 tools/tricopter/dynamic_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_dynamic_replay.py -v`. `--audit-only` exports the compact independent optimization audit. No hardware, PID, mixer, scheduler or active actuator integration is involved.
