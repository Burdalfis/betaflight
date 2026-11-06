# Tricopter M0 baseline — 2026-11-06

The unchanged local Betaflight `2026.6.2` build for `MATEKF405TE` is reproducible. Both clean builds produced byte-identical firmware HEX, raw BIN and ELF files. The local images have **not** been flashed. No flight-control law was added or validated during M0.

The existing upstream commit `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16` is the unchanged firmware checkpoint. All local baseline builds completed before instrumentation source edits began. The user separately confirmed their working cloud build reports firmware `e0b7bb01b`, configuration `3a406c5`, and board `MATEKF405TE`, and confirmed the aircraft's motor/servo mapping is correct. The [complete CLI text supplied in chat](tricopter_cloud_cli_2026-11-06.txt) preserves that modern baseline evidence. That hardware report does not establish that either locally built image has run on the aircraft.

## Build provenance

| Input | Recorded value |
|---|---|
| Firmware | `2026.6.2`, `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16` |
| Board configuration | `MATEKF405TE`, `3a406c5c30c20d7c3bc7dd5ed547cca961baf0a9` |
| Config header | `src/config/configs/MTKS/MATEKF405TE/config.h` |
| MCU build target | `STM32F405`, Cortex-M4F, hard-float |
| libcanard dependency | `601ed35467e0ac38819df17cd7c918de19f62d58` |
| Toolchain | Arm GNU Toolchain 13.3.Rel1, GCC `13.3.1 20240614` |
| Build options | Standard local configuration; no `OPTIONS` or `EXTRA_FLAGS` overrides |
| Reproducible timestamp | `SOURCE_DATE_EPOCH=1789594937`, the baseline commit timestamp |

The repository's board configuration selects the STM32F405 target. This records the build mapping; it is not independent qualification of the fitted STM32F415 hardware.

From the unchanged baseline checkout, with the pinned submodules and toolchain installed:

```sh
git submodule update --init --checkout -- src/config lib/modules/dronecan/libcanard
make arm_sdk_install
SOURCE_DATE_EPOCH=1789594937 make -j8 CONFIG=MATEKF405TE V=1 CCACHE= fwo
make CONFIG=MATEKF405TE CCACHE= clean
SOURCE_DATE_EPOCH=1789594937 make -j8 CONFIG=MATEKF405TE V=1 CCACHE= fwo
```

The fixed epoch controls GCC's embedded build date/time. `CCACHE=` avoids depending on a user cache outside the workspace. The toolchain installer verified its pinned archive checksum; the archive SHA-256 is also recorded in the [machine-readable manifest](../.local-artifacts/tricopter/m0/manifest.json).

## Results and preserved evidence

Both clean builds passed with the firmware's normal warning-as-error settings. Preprocessor evidence confirms `USE_SERVOS`, `USE_ADC`, `ADC_EXTERNAL1_PIN PC0`, `USE_DSHOT`, `USE_DSHOT_TELEMETRY`, `USE_DSHOT_TELEMETRY_STATS`, `USE_BLACKBOX`, and `USE_FLASHFS`. Linked symbols include `mixerTricopter` and `servoMixerTri`.

GNU size reports **527457 bytes text, 6620 bytes data, and 99900 bytes bss**. These are linked section sizes, not measured runtime memory headroom.

| Artifact | SHA-256, identical across both clean builds |
|---|---|
| HEX | `215d927005ece15e4541dd7a4dc216bf597132e737508b80dbb2136633af9d19` |
| Raw BIN | `2082579546445af6cadb702698c3eaae78964da820dfc4619a79a3b6b58102d8` |
| ELF | `eef2991143d592eadad745c49c439d8f144cbf27e96ad3a7644ec7b0e98dbb03` |

The linker maps differ only in GCC's randomized `/tmp/cc...` LTO filenames; they compare equal after normalizing those paths. Their raw hashes differ.

Local evidence is preserved under [`.local-artifacts/tricopter/m0/`](../.local-artifacts/tricopter/m0/README.md):

- [First build images and map](../.local-artifacts/tricopter/m0/first/), and [clean repeat](../.local-artifacts/tricopter/m0/repeat/).
- [First build log](../.local-artifacts/tricopter/m0/build-first.log), [repeat build log](../.local-artifacts/tricopter/m0/build-repeat.log), and [clean log](../.local-artifacts/tricopter/m0/clean.log).
- [Target preprocessor macros](../.local-artifacts/tricopter/m0/feature-macros.txt), [preprocessor invocation](../.local-artifacts/tricopter/m0/preprocess-command.json), [linked symbols](../.local-artifacts/tricopter/m0/symbols.txt), and [pinned board config](../.local-artifacts/tricopter/m0/MATEKF405TE.config.h).

The relevant existing host tests also passed: motor output reordering, Blackbox, Blackbox encoding, and PID (**28 tests total**). The preferred Clang compiler was unavailable, so the repository's supported experimental GCC/G++ fallback was used. An equivalent invocation is:

```sh
make -C src/test -j4 CC=gcc CXX=g++ \
  test_motor_output_unittest test_blackbox_unittest \
  test_blackbox_encoding_unittest test_pid_unittest
```

The [preserved host test log](../.local-artifacts/tricopter/m0/host-tests.log) contains all four passing suites and the GCC fallback warning.

## Boundary before M1

M0 establishes the local build and recovery reference while retaining the legacy mixer. The user's current cloud firmware and mapping confirmation supply separate aircraft evidence. M1 can proceed as logging-only instrumentation; measured PC0 loading/noise, actual output timing, continuous EDT/RPM reception, logging throughput and runtime jitter remain bench-validation work for the instrumented image.

## Subsequent hardware follow-up

M1 `bd6a5d727` was subsequently flashed and tested with the user's revised PB0 feedback wiring; see [the bench record](tricopter_m1_bench_2026-11-06.md). The untouched M0 images above remain recovery references. The standard local build has additional configuration groups compared with the cloud build, so matching source/config revisions did not guarantee configuration retention: first boot reset settings, and the fresh current-version full dump was restored and verified before testing. The M0 reproducibility result establishes repeatability of the stated local build, not binary/feature equivalence to the cloud image.
