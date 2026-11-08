# Saved tricopter servo neutral — 2026-11-07

Servo record **5** on the connected MATEKF405TE now has **1100 / 1700 / 1509 µs**, rate **100**, no channel forwarding and unchanged reversal flags. The new middle rounds the measured command-regression zero, 1508.805109 µs; it is an **89 µs increase** from the previous 1420 µs setting. The motor remains absent and the ADXL occupies its place.

This is a persistent configuration change on the existing firmware, not a firmware flash or a new control law. The FC still reports Betaflight 2026.6.2, revision `f6e496fc1`, MSP API 1.48. PB0−PC0 remains the sole servo position measurement. Sensor acquisition, PID, motor resources/reordering, receiver/arming, servo rate/forwarding/reversal and startup `servo_center_pulse=1500` are unchanged.

The tested hard PWM limits remain **1100–1700 µs**. The sweep exercised those values; it did not establish new mechanical end stops. Regression training bounds and common ADC-map bounds are not replacement PWM limits. With the stock TRI mixer's existing ±300 µs contribution limit and rate 100, the normal mixer output at this new middle spans **1209–1700 µs**. Its asymmetric saturation is unchanged in implementation; the full configured lower endpoint is not reached through that stock mixer scaling.

The incremental CLI setting is:

```text
servo 5 1100 1700 1509 100 -1
save
```

Application used bounded center increments of at most 20 µs, preserving the other raw servo-record fields, followed by an EEPROM save and normal reboot. The USB connection dropped before the reboot acknowledgement arrived; independent reconnection/readback verified persistence. Uptime changed from 13,334 seconds before application to 23 seconds in the subsequent CLI snapshot.

Verification compares all **1473 complete-dump commands**: the servo5 middle is the only configuration change. Active resources/timers/DMA and firmware version match. The exact full 96-byte servo table matches the intended single-byte change. Thirty post-reboot samples report final PWM **1509 µs**, fresh canonical PB0−PC0, disarmed status and three motor outputs at **1000**.

Full before/after CLI backups, original servo table, the planned byte change, application transcript and post-reboot checks are retained in `.local-artifacts/tricopter/servo-neutral-2026-11-07/`. The archived calibration/sweep recordings retain their original 1420 µs configuration. Their historical capture helpers deliberately require that original center and will reject this new live setting until a future capture explicitly supports it.

To restore the previous neutral while retaining the same limits and rate:

```text
servo 5 1100 1700 1420 100 -1
save
```
