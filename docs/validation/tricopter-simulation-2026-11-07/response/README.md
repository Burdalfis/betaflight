# Offline integrated actuator-to-response simulation

This time-stepped simulation executes the actual C servo predictor, RPM observer, reaction equations, allocator and offline coordinator. A separate synthetic plant has its own C servo instance, analytic motor lag/slew, effectiveness map and coefficient values. It integrates no vehicle attitude or body dynamics and contains no PID, mixer, hardware or active control path.

The controller requires no CAD, CG, vehicle mass or body inertia. Its effectiveness map uses a declared global virtual-tail effort basis, not an arbitrary local motor/servo Jacobian. Mathematical direction and corrected PB0-PC0-equivalent counts are linked only by an explicit synthetic mapping, with no physical-angle calibration. Both servo instances use the same model family; matched agreement tests integration, not independent servo physics.

At each boundary the plant advances under prior commands, captures RPM and delivers queued observations, then the coordinator emits commands for the following interval. Decoder capture timestamps survive delayed software delivery, so observer age includes that queue delay. The capture is not an ESC-internal timing measurement. Cached/equal-zero packets, dropouts, outliers and reacquisition retain explicit identity/quality flags.

Plant actual response, direction, counts, derivative, map and reaction never enter the controller after its initial corrected-position reset; RPM packet metadata is the sole simulated measurement stream. Suffix perturbation and hidden-truth counterfactual tests verify that separation.

The full Q+Jalpha law is an explicitly synthetic additive yaw disturbance, independent of mathematical tail direction. Both maps must declare that they exclude it, preventing double counting. It is a scalar surrogate, without tilted-rotor, gyroscopic or aircraft torque-accuracy claims. The plant uses its analytic RPM derivative; the controller uses qualified observer acceleration. Reaction-only compensation and optional bounded virtual-tail-vertical coordination are separate experiments. The latter does not preserve arbitrary matrix collective response.

Every actual-response row is scored, including telemetry faults, saturation and estimator unavailability. Prediction errors are availability-masked with explicit counts/coverage. All four response errors are reported; improvement is not assumed, especially under mismatch. Plant RPM limits are real caps in this synthetic plant, with separate clip flags.

| Airframe / scenario | Experiment | Actual RMSE collective / roll / pitch / yaw | Predicted total coverage |
|---|---|---|---:|
| synthetic_A / matched | baseline | 0.3329 / 0.2046 / 0.174 / 0.3254 | 99.8% |
| synthetic_A / matched | reaction_only | 0.3276 / 0.2097 / 0.1708 / 0.3203 | 99.8% |
| synthetic_A / matched | reaction_and_tail | 0.3291 / 0.2098 / 0.1693 / 0.3207 | 99.8% |
| synthetic_A / telemetry_faults | baseline | 0.3329 / 0.2046 / 0.174 / 0.3254 | 85.9% |
| synthetic_A / telemetry_faults | reaction_only | 0.3305 / 0.2072 / 0.1725 / 0.3209 | 85.9% |
| synthetic_A / telemetry_faults | reaction_and_tail | 0.3319 / 0.2073 / 0.171 / 0.3213 | 85.9% |
| synthetic_A / model_mismatch | baseline | 0.3171 / 0.2157 / 0.1705 / 0.3625 | 99.8% |
| synthetic_A / model_mismatch | reaction_only | 0.3129 / 0.2201 / 0.1667 / 0.3549 | 99.8% |
| synthetic_A / model_mismatch | reaction_and_tail | 0.3129 / 0.2202 / 0.1651 / 0.3553 | 99.8% |
| synthetic_B / matched | baseline | 0.4302 / 0.2041 / 0.3095 / 0.3837 | 99.8% |
| synthetic_B / matched | reaction_only | 0.449 / 0.2047 / 0.3168 / 0.3804 | 99.8% |
| synthetic_B / matched | reaction_and_tail | 0.4477 / 0.2046 / 0.3139 / 0.3806 | 99.8% |
| synthetic_B / telemetry_faults | baseline | 0.4302 / 0.2041 / 0.3095 / 0.3837 | 85.9% |
| synthetic_B / telemetry_faults | reaction_only | 0.4548 / 0.2061 / 0.3159 / 0.3803 | 85.9% |
| synthetic_B / telemetry_faults | reaction_and_tail | 0.4534 / 0.2059 / 0.3129 / 0.3805 | 85.9% |
| synthetic_B / model_mismatch | baseline | 0.4307 / 0.2153 / 0.3175 / 0.4179 | 99.8% |
| synthetic_B / model_mismatch | reaction_only | 0.4555 / 0.2169 / 0.3261 / 0.4143 | 99.8% |
| synthetic_B / model_mismatch | reaction_and_tail | 0.4532 / 0.2168 / 0.3227 / 0.4145 | 99.8% |

All numerical coefficients, maps, lags, limits and counts mappings are arbitrary synthetic placeholders. Error numbers describe this defined plant only and establish neither flight stability nor real-aircraft performance.

The MATLAB parity fixture executes the same C batch bridge; matching it is an interface/parity check, not independent model validation. MATLAB/Simulink analysis is maintained separately. `simulation-summary.json`, `native-build.json`, compressed traces and source hashes preserve reproduction details.

Reproduce with `python3 tools/tricopter/simulation_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_simulation_replay.py -v`. `--fixture-only` exports the compact shared-C parity input/output fixture.
