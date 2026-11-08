#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Reuse Betaflight's vendored host framework. Honor CC/CXX; installations
# without the preferred clang can run CC=gcc CXX=g++ ./run_tricopter_checks.sh.
compiler_args=()
if [[ -n "${CC:-}" ]]; then compiler_args+=("CC=$CC"); fi
if [[ -n "${CXX:-}" ]]; then compiler_args+=("CXX=$CXX"); fi
make -j"${TRICOPTER_TEST_JOBS:-4}" "${compiler_args[@]}" \
    test_dshot_telemetry_unittest test_tricopter_diagnostics_unittest \
    test_tricopter_servo_model_unittest \
    test_tricopter_angle_calibration_unittest \
    test_tricopter_rpm_observer_unittest \
    test_tricopter_reaction_torque_unittest \
    test_tricopter_allocator_unittest \
    test_tricopter_simulation_unittest \
    test_tricopter_dynamic_allocator_unittest \
    test_adc_pair_snapshot_unittest \
    test_motor_output_unittest test_blackbox_unittest \
    test_blackbox_encoding_unittest test_pid_unittest
cd ../..
sha256sum --check docs/tricopter_originals.sha256
