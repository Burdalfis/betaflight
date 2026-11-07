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

#define TRICOPTER_SERVO_MODEL_MAX_KNOTS 8U
#define TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY 32U

// Offline command-driven prediction only. Position is signed PB0-PC0 ADC
// counts, not calibrated shaft angle. PC0 senses the servo power GND input,
// not the potentiometer's internal low terminal. No actuator calls or sensor
// reads are made here. Fitted parameters describe the supplied bench data;
// they are not physical bounds or a loaded-servo calibration.
typedef struct tricopterServoModelParams_s {
    uint8_t knotCount;
    // Strictly increasing PWM knots; monotonic counts, in [-4095, 4095].
    float pwmUs[TRICOPTER_SERVO_MODEL_MAX_KNOTS];
    float positionCounts[TRICOPTER_SERVO_MODEL_MAX_KNOTS];
    // Full width of a continuous play operator in the PWM domain.
    float backlashUs;
    uint32_t delayUs;
    // Positive lag is at least 1 us and at most 60 s; zero disables lag.
    float tauIncreasingPwmS;
    float tauDecreasingPwmS;
    // Zero means unlimited slew. Nonzero limits are counts/s, <= 1e9.
    float rateIncreasingPwmCountsS;
    float rateDecreasingPwmCountsS;
    // Maximum interval between calls, before delayed-event subdivision.
    // This is command-history continuity, not ADC measurement freshness.
    uint32_t maxGapUs;
} tricopterServoModelParams_t;

typedef enum tricopterServoModelStatus_e {
    TRICOPTER_SERVO_MODEL_OK = 0,
    TRICOPTER_SERVO_MODEL_NOT_INITIALIZED = 1U << 0,
    TRICOPTER_SERVO_MODEL_INVALID_CONFIG = 1U << 1,
    TRICOPTER_SERVO_MODEL_INVALID_POSITION = 1U << 2,
    TRICOPTER_SERVO_MODEL_INVALID_PWM = 1U << 3,
    TRICOPTER_SERVO_MODEL_INVALID_TIME = 1U << 4,
    TRICOPTER_SERVO_MODEL_COMMAND_GAP = 1U << 5,
    TRICOPTER_SERVO_MODEL_QUEUE_FULL = 1U << 6,
    TRICOPTER_SERVO_MODEL_INVALID_DIRECTION = 1U << 7,
} tricopterServoModelStatus_e;

typedef struct tricopterServoModelOutput_s {
    float positionCounts;
    // Instantaneous model derivative at the returned timestamp. An ideal
    // static-map jump has no finite derivative and is reported as zero.
    float velocityCountsS;
    uint32_t status;
    bool valid;
    bool angleCalibrated; // Always false, including valid predictions.
} tricopterServoModelOutput_t;

typedef struct tricopterServoModelCommand_s {
    uint64_t dueUs;
    float pwmUs;
} tricopterServoModelCommand_t;

typedef struct tricopterServoModelState_s {
    tricopterServoModelParams_t params;
    tricopterServoModelCommand_t commands[TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY];
    uint64_t timeUs;
    float positionCounts;
    float velocityCountsS;
    float targetCounts;
    float playedPwmUs;
    float lastCommandPwmUs;
    uint32_t status;
    uint8_t commandHead;
    uint8_t commandCount;
    bool initialized;
} tricopterServoModelState_t;

bool tricopterServoModelValidateParams(const tricopterServoModelParams_t *params);

// Reset is the only measured-position input. The caller must ensure that this
// is a valid corrected-count sample; this model does not assess ADC quality.
// The initial PWM is already active (no initial pending delay). Direction is
// -1, 0, or +1 in PWM units. A known settled approach starts the play operator
// at initialPWM-direction*backlash/2; unknown direction uses its center.
// Replay should initialize once per independently declared recording unit,
// from a pre-step hold, then preserve state through every later transition.
// Invalid reset clears state and returns false with a finite invalid output.
bool tricopterServoModelReset(tricopterServoModelState_t *state,
    const tricopterServoModelParams_t *params, uint64_t nowUs,
    float initialPwmUs, float initialCorrectedCounts, int initialDirection);

// Timestamps are absolute microseconds, strictly nondecreasing. Parameters
// are copied at reset and cannot change mid-history. Repeated unchanged PWM
// is coalesced. Changed commands at identical timestamps retain call order;
// zero-delay commands act at that timestamp. Delayed events are integrated
// at their exact due times, independently of the caller's sample intervals.
// Invalid input, backward/overflowing time, an excessive gap, or a full fixed
// queue latches invalid until reset. The last finite prediction is retained,
// with velocity zero. No held-out measurement is consumed by this function.
tricopterServoModelOutput_t tricopterServoModelStep(tricopterServoModelState_t *state,
    uint64_t nowUs, float commandPwmUs);
