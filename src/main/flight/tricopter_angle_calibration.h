/* Offline settled basket angle map. Sole position input is signed PB0-PC0.
 * Mean major-approach map is continuous through command reversals. The envelope
 * describes observed major-branch separation, not a confidence/error bound.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define TRICOPTER_ANGLE_CALIBRATION_MAX_KNOTS 64U

typedef struct tricopterAngleCalibration_s {
    uint8_t knotCount;
    float correctedCounts[TRICOPTER_ANGLE_CALIBRATION_MAX_KNOTS]; // Strictly ascending.
    float angleRad[TRICOPTER_ANGLE_CALIBRATION_MAX_KNOTS]; // Strictly monotone, either sign.
    float halfBranchSeparationRad[TRICOPTER_ANGLE_CALIBRATION_MAX_KNOTS];
    bool operatorDefinedReference; // Mounting reference; not independent metrology.
} tricopterAngleCalibration_t;

typedef struct tricopterAngleCalibrationOutput_s {
    float angleRad;
    float halfBranchSeparationRad;
    bool valid;
    bool operatorDefinedReference;
} tricopterAngleCalibrationOutput_t;

bool tricopterAngleCalibrationValidate(const tricopterAngleCalibration_t *map);
tricopterAngleCalibrationOutput_t tricopterAngleCalibrationEvaluate(
    const tricopterAngleCalibration_t *map, float correctedCounts);
// Out-of-domain requests fail; no endpoint clipping or extrapolation.
bool tricopterAngleCalibrationInverse(const tricopterAngleCalibration_t *map,
    float angleRad, float *correctedCounts);
