# Offline RPM observer replay

The real portable C observer is exercised independently for all three motors. These historical recordings establish compatibility and stale handling; they do not establish true acceleration accuracy or identify motor, voltage, load or torque coefficients.

Settings were frozen before evaluation. Tracking tau is 20 ms; acceleration decays after a 10 ms freshness limit with a 30 ms time constant. Prediction stops at a 100 ms horizon. RPM and acceleration limits are software guards.

Blackbox ages are quantized and their within-loop phase is unmeasured. Replay uses interval-overlap cache identities and conservative representative timestamps. Phase allowances 0/250/500 us are sensitivity assumptions, not measured bounds. Equal-valued new receptions, including zero, can refresh when their plausible intervals separate; cached zeros retain their old source time.

A cached representative timestamp can precede a later narrowed interval intersection. Keeping its original timestamp makes age more conservative; it does not establish an exact source age.

| Record / phase allowance | Motor | Accepted proxy inputs | Not effectively fresh rows | Horizon exceeded rows | Maximum logged age (ms) |
|---|---:|---:|---:|---:|---:|
| record-02-aborted-motor / 0 us | 0 | 2959 | 1329 | 1239 | 1335.6 |
| record-02-aborted-motor / 0 us | 1 | 4297 | 0 | 0 | 0.1 |
| record-02-aborted-motor / 0 us | 2 | 4297 | 0 | 0 | 0.1 |
| record-02-aborted-motor / 250 us | 0 | 2959 | 1329 | 1239 | 1335.6 |
| record-02-aborted-motor / 250 us | 1 | 4297 | 0 | 0 | 0.1 |
| record-02-aborted-motor / 250 us | 2 | 4297 | 0 | 0 | 0.1 |
| record-02-aborted-motor / 500 us | 0 | 2958 | 1330 | 1240 | 1335.6 |
| record-02-aborted-motor / 500 us | 1 | 4297 | 0 | 0 | 0.1 |
| record-02-aborted-motor / 500 us | 2 | 4297 | 0 | 0 | 0.1 |
| record-03-low-power-motors / 0 us | 0 | 19722 | 144 | 22 | 121.6 |
| record-03-low-power-motors / 0 us | 1 | 19785 | 144 | 22 | 121.3 |
| record-03-low-power-motors / 0 us | 2 | 19801 | 149 | 22 | 121.3 |
| record-03-low-power-motors / 250 us | 0 | 19720 | 146 | 23 | 121.6 |
| record-03-low-power-motors / 250 us | 1 | 19783 | 145 | 22 | 121.3 |
| record-03-low-power-motors / 250 us | 2 | 19801 | 152 | 22 | 121.3 |
| record-03-low-power-motors / 500 us | 0 | 19644 | 146 | 23 | 121.6 |
| record-03-low-power-motors / 500 us | 1 | 19722 | 146 | 23 | 121.3 |
| record-03-low-power-motors / 500 us | 2 | 19744 | 153 | 23 | 121.3 |

Counts above are proxy-adapter outcomes, not exact packet counts. Under the zero-phase sensitivity, the aborted gap contains a few contradictory or shifted ancient candidate intervals; they do not reset age to the current time. No observations are accepted in rows whose native age exceeds the prediction horizon.

An independent exact-timestamp synthetic ramp checks the executable C derivative state. Its tail speed/acceleration errors are recorded in `rpm-replay-summary.json`; these errors do not apply to the historical bench trace.

Plots show all three motors, predicted speed, estimated acceleration and stale gaps. The historical raw PB0 channel is unused. Host command times lack FC alignment, so no command-to-RPM delay is inferred.

See `input-provenance.json`, `native-build.json`, `rpm-replay-summary.json` and `checksums.sha256` for pinned sources, compiler, raw/decoded data and outputs. No firmware integration, flashing or actuator action is performed.
