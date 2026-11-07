# MATLAB and Simulink adapter

The MATLAB MEX entry point executes the **same C batch simulation** as the Python runner, including its synthetic plant and all four portable modules. It adds no MATLAB implementation of the controller or actuator dynamics. Source fingerprints are checked before and after compilation. All airframes, coefficients and angle/effort mappings remain synthetic placeholders.

First generate the simulation report and `matlab-parity-fixture.json` with the Python runner. In MATLAB, from the repository root:

```matlab
root = pwd;
addpath(fullfile(root,'tools','tricopter','matlab'));
report = run_tricopter_simulation( ...
    fullfile(root,'.local-artifacts','tricopter','simulation-replay','matlab-parity-fixture.json'), ...
    fullfile(root,'.local-artifacts','tricopter','simulation-matlab'), true);
```

This requires MATLAB's C MEX toolchain. Set the last argument to `false` to omit Simulink. The runner compiles the actual C sources, compares every numeric/status/quality output to the Python fixture, and saves a validation report, MAT trace, CSV and plot. Numeric comparison uses an explicit `2e-6*(1+abs(expected))` tolerance per element; integers and quality flags must match exactly. Unavailable predictions are gaps in plots, with their flags retained.

When requested, it also builds and runs a self-contained `tricopter_offline_signal_replay.slx`. That model is **signal playback and analysis**, using a discrete fixed tick, interpolation off and the saved synthetic request/actual trace. It does not execute a separate Simulink plant, add a PID, or establish independent physical-model validation. The saved model is closed, reopened and checked against the source trace.

The replay vector's first four channels are requested collective/roll/pitch/yaw proxies; the next four are actual surrogate responses in the same order. The MAT file retains all native numeric/status/flag columns; their labels are in the fixture and `simulation_bridge.h`.

## Independent dynamic-allocation audit

The dynamic allocator has a separate opt-in C/Python entry point. Its compact `candidate-audit.json` can be checked with MATLAB's Optimization Toolbox:

```matlab
validate_tricopter_dynamic_allocator( ...
    fullfile(root,'.local-artifacts','tricopter','dynamic-replay','candidate-audit.json'), ...
    fullfile(root,'.local-artifacts','tricopter','dynamic-matlab-audit'));
```

This uses [MathWorks `lsqlin`](https://www.mathworks.com/help/optim/ug/lsqlin.html) as an independent bounded least-squares solver for the protected collective/roll/pitch problem. It also forwards published RPM commands through an independent motor equation and checks the global primary/yaw/movement selection bands. Exported servo-horizon directions are inputs to this algebra audit; it does not independently validate the servo or airframe physics. Source/protocol hashes and explicit numerical differences are saved in the report.

The original MEX runner retains its original flat ABI and modes. Its regenerated fixture and Simulink replay check compatibility; they are separate from the new dynamic-candidate audit.

MEX transport uses [MathWorks' C gateway API](https://www.mathworks.com/help/matlab/matlab_external/gateway-routine.html); signal playback follows the [From Workspace data contract](https://www.mathworks.com/help/simulink/ug/load-data-using-the-from-workspace-block.html). Generated binaries and caches belong in `.local-artifacts`, not the source tree. The current validation boundary and limits are in [the simulation documentation](../../../docs/tricopter_simulation.md).
