# Offline tricopter allocation

The primary direct-effectiveness interface needs a response matrix, actuator effort bounds and explicit response priorities. It requires no rotor positions, CG, mass, inertia or kT. Its outputs are effort/response proxies, without thrust, force, moment or RPM claims. The tail direction is a mathematical effort coordinate until separately calibrated.

The matrix acts on [front-right effort, front-left effort, tail effort*cos(direction), tail effort*sin(direction)]. It is a global virtual-tail basis, not an interchangeable local [delta motors, delta servo] Jacobian. A future characterization process must establish that basis and its usable range. The included noiseless least-squares demonstration is entirely synthetic; no aircraft map is identified.

An optional geometry adapter checks SI thrust and r×F equations using arbitrary synthetic rotor positions, nonzero CG offsets and kT. Known reaction yaw is exogenous and applied once. The separate forward API demonstrates that a requested tail angle and a held actual angle produce different wrenches at the same thrusts. Servo angle is never inferred from ADC counts or PWM.

Both interfaces use the actual portable C kernel. Exact inversion handles feasible requests; a bounded convex projected-gradient solve handles saturation. Residual means achieved minus requested. Priorities are the frozen response scales in the protocol. Nonzero saturated residual does not mean solver nonconvergence.

Independent validation first checks feasibility, then forms the convex objective-gap bound g·x−min(g·z), with an independently computed linear support point over front intervals and the tail sector. The report states the dimensionless objective-gap cutoff separately from the C gradient-mapping tolerance. Tiny float32 publication rounding is projected to a feasible float64 certificate point; correction and response error are recorded. This is a numerical certificate, not an interval-arithmetic rounding proof.

| Configuration | Static cases + sweep points | Maximum objective-gap bound | All declared checks pass |
|---|---:|---:|---|
| direct-effectiveness | 86 | 7.99202e-06 | True |
| physical-adapter | 92 | 5.01498e-05 | True |

Gradient mapping tolerance 1e-5 explicitly accommodates float32 publication; it is distinct from the dimensionless independent objective-gap cutoff 1e-4. These numerical settings are not physical coefficients.

Plots are synthetic static request sweeps, not elapsed time or servo tracking. Every geometry, map, coefficient and limit is a placeholder, uncalibrated and unavailable for flight. Fixed-iteration nonconvergence and invalid maps/reaction metadata are stored explicitly. No PID, mixer, scheduler, actuator, hardware or flash path is connected.

Reproduce with `python3 tools/tricopter/allocator_replay.py --output /path/to/output` and `python3 -m unittest discover -s tools/tricopter -p test_allocator_replay.py -v`. Source/build/protocol provenance and per-case certificates are in `allocator-replay-summary.json`; output hashes are in `checksums.sha256`.
