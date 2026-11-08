/* Pure, asynchronous, disarmed-only tricopter servo ADC/speed calibration.
 * This module owns no hardware or persistent configuration. Angles are supplied
 * by the operator; only ADC means and an observed movement-speed estimate are
 * learned. Zero-initialize state before its first Start call.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TRICOPTER_SERVO_CALIBRATION_TRACE_CAPACITY 128U
#define TRICOPTER_SERVO_CALIBRATION_SAMPLE_US 5000U
#define TRICOPTER_SERVO_CALIBRATION_SETTLE_US 1000000U
#define TRICOPTER_SERVO_CALIBRATION_AVERAGE_US 200000U
#define TRICOPTER_SERVO_CALIBRATION_STAGE_TIMEOUT_US 2000000U
#define TRICOPTER_SERVO_CALIBRATION_FEEDBACK_TIMEOUT_US 100000U

typedef enum tricopterServoCalibrationStage_e {
    TRICOPTER_SERVO_CALIBRATION_IDLE = 0,
    TRICOPTER_SERVO_CALIBRATION_CENTER,
    TRICOPTER_SERVO_CALIBRATION_LOW,
    TRICOPTER_SERVO_CALIBRATION_HIGH,
    TRICOPTER_SERVO_CALIBRATION_RETURN_CENTER,
    TRICOPTER_SERVO_CALIBRATION_COMPLETE,
    TRICOPTER_SERVO_CALIBRATION_FAILED,
    TRICOPTER_SERVO_CALIBRATION_ABORTED,
} tricopterServoCalibrationStage_t;

typedef enum tricopterServoCalibrationStatus_e {
    TRICOPTER_SERVO_CALIBRATION_OK = 0,
    TRICOPTER_SERVO_CALIBRATION_INVALID_PARAMS,
    TRICOPTER_SERVO_CALIBRATION_ARMED,
    TRICOPTER_SERVO_CALIBRATION_USER_ABORT,
    TRICOPTER_SERVO_CALIBRATION_INVALID_TIME,
    TRICOPTER_SERVO_CALIBRATION_FEEDBACK_TIMEOUT,
    TRICOPTER_SERVO_CALIBRATION_STAGE_TIMEOUT,
    TRICOPTER_SERVO_CALIBRATION_INSUFFICIENT_SPAN,
    TRICOPTER_SERVO_CALIBRATION_NONMONOTONIC,
    TRICOPTER_SERVO_CALIBRATION_SPEED_UNAVAILABLE,
} tricopterServoCalibrationStatus_t;

typedef struct tricopterServoCalibrationParams_s {
    uint16_t pwmUs[3]; // Existing minimum, middle and maximum; strictly ascending.
    float angleDeg[3]; // Operator-provided corresponding angles, strictly monotone.
} tricopterServoCalibrationParams_t;

typedef struct tricopterServoCalibrationOutput_s {
    tricopterServoCalibrationStage_t stage;
    tricopterServoCalibrationStatus_t status;
    uint16_t commandedPwmUs; // Always bounded; terminal states request middle.
    bool commandValid; // False for invalid configuration or any armed call.
    bool active;
    bool complete;
    bool failed;
    bool resultValid;
    int16_t learnedAdc[3]; // Fresh signed PB0-PC0 means at configured PWM points.
    int16_t initialCenterAdc; // Diagnostic; final return-center mean is learned.
    float observedSpeedDegPerSecond; // 10–90% observed move, not physical maximum.
    bool speedEstimateValid;
    uint32_t observed10To90Us;
    uint16_t traceSamples;
} tricopterServoCalibrationOutput_t;

typedef struct tricopterServoCalibrationState_s {
    tricopterServoCalibrationParams_t params;
    tricopterServoCalibrationOutput_t output;
    uint32_t stageStartUs, lastStepUs, lastFeedbackUs, lastSampleUs, lastRampUs;
    uint32_t averageStartUs, traceStartUs, lastTraceUs, traceIntervalUs;
    int32_t averageSum, firstHalfSum, secondHalfSum;
    uint16_t averageSamples, firstHalfSamples, secondHalfSamples;
    int16_t averageMin, averageMax;
    bool averageActive;
    uint16_t traceCount;
    uint32_t traceTimeUs[TRICOPTER_SERVO_CALIBRATION_TRACE_CAPACITY];
    int16_t traceCounts[TRICOPTER_SERVO_CALIBRATION_TRACE_CAPACITY];
} tricopterServoCalibrationState_t;

bool tricopterServoCalibrationValidateParams(const tricopterServoCalibrationParams_t *params);
bool tricopterServoCalibrationStart(tricopterServoCalibrationState_t *state,
    const tricopterServoCalibrationParams_t *params, uint32_t nowUs, bool disarmed);
// rawPairValid means a coherent, nonrailed pair within the caller's freshness
// bound is available, not a one-tick new-sample pulse. Step must run at least
// once per 100ms; sampling/averaging is bounded internally to one per 5ms.
tricopterServoCalibrationOutput_t tricopterServoCalibrationStep(tricopterServoCalibrationState_t *state,
    uint32_t nowUs, bool armed, bool rawPairValid, int16_t correctedCounts);
tricopterServoCalibrationOutput_t tricopterServoCalibrationAbort(tricopterServoCalibrationState_t *state);
const tricopterServoCalibrationOutput_t *tricopterServoCalibrationGetOutput(const tricopterServoCalibrationState_t *state);
