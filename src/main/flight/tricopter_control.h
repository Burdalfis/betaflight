/* Three-point feedback tricopter allocation. No motor/servo I/O or scheduler.
 * Corrected PB0-PC0 is the only measured position input. PWM is command only.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define TRICOPTER_CONTROL_MAP_POINTS 3U
#define TRICOPTER_CONTROL_MAX_ANGLE_DEG 80.0f
#define TRICOPTER_CONTROL_MIN_COSINE 0.1f

typedef struct tricopterControlParams_s {
    float pwmUs[TRICOPTER_CONTROL_MAP_POINTS];
    float angleDeg[TRICOPTER_CONTROL_MAP_POINTS]; // Increasing min, zero, max.
    float adcCounts[TRICOPTER_CONTROL_MAP_POINTS]; // Either monotone direction.
    float positionGainUsPerDeg;
    float yawGain;
    int8_t yawDirection; // +1 or -1; independent of physical calibration.
    float neutralTrimDeg;
    uint32_t feedbackTimeoutUs;
    float adcMarginCounts;
    float minVerticalEffort;
    float pwmMinUs; // Existing hardware limits; must contain physical zero.
    float pwmMaxUs;
} tricopterControlParams_t;

typedef struct tricopterControlInput_s {
    float verticalEffort; // Normalized desired tail vertical effort, 0..1.
    float normalizedYaw; // Unflipped PID yaw sum /1000; caller applies rate/sign.
    float correctedCounts; // Signed PB0-PC0 only; never raw ADC/PWM fallback.
    uint32_t feedbackAgeUs;
    bool feedbackValid; // Caller establishes coherent, supported, non-railed ADC.
} tricopterControlInput_t;

typedef struct tricopterControlOutput_s {
    float tailEffort;
    float targetAngleDeg;
    float actualAngleDeg; // Available only when feedbackValid.
    float pwmUs;
    float currentVertical;
    float currentLateral;
    bool validConfig;
    bool validInput;
    bool feedbackValid;
    bool currentVectorAvailable;
    bool yawSaturated;
    bool verticalSaturated;
} tricopterControlOutput_t;

tricopterControlParams_t tricopterControlDefaultParams(void);
bool tricopterControlValidateParams(const tricopterControlParams_t *params);
// ADC excursions within adcMarginCounts are clamped to the calibrated endpoint;
// beyond that margin they are unavailable. Out-of-domain PWM/angle is rejected.
bool tricopterControlAngleFromAdc(const tricopterControlParams_t *params,
    float correctedCounts, float *angleDeg);
bool tricopterControlPwmFromAngle(const tricopterControlParams_t *params,
    float angleDeg, float *pwmUs);
tricopterControlOutput_t tricopterControlEvaluate(const tricopterControlParams_t *params,
    const tricopterControlInput_t *input);
