/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#define TRICOPTER_SERVO_POSITION_MAX_AGE_US 1000U
#define TRICOPTER_SERVO_POSITION_UNAVAILABLE INT16_MIN

// Canonical tricopter position measurement: signed PB0 wiper minus PC0 servo
// power-ground sense, in ADC counts. This is not a calibrated shaft/rotor angle
// or a measurement relative to the potentiometer's internal low terminal.
typedef struct tricopterServoPositionSample_s {
    bool supported;
    bool ready;
    bool sampleSeen;
    bool fresh;
    int16_t correctedCounts;
    uint32_t ageUs;
    // Diagnostics only. Position consumers must use correctedCounts and fresh.
    uint16_t rawWiperCounts;
    uint16_t rawGroundCounts;
} tricopterServoPositionSample_t;

// No single-channel fallback exists. Before a coherent pair, or with invalid
// raw range, correctedCounts is UNAVAILABLE. A stale cached correction is kept
// for diagnostics but fresh=false. Freshness describes acquisition, not wiring
// health or angle calibration. This accessor does not change actuator outputs.
tricopterServoPositionSample_t servosTricopterGetPosition(timeUs_t currentTimeUs);
