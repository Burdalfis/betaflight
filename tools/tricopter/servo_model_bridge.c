/* Offline batch adapter. Every prediction calls the portable flight C model. */
#include <math.h>
#include <float.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "flight/tricopter_servo_model.h"

int tricopterServoReplay(const double *values, double initialPosition, double initialCommand,
                        int initialDirection, size_t count, const double *timeUs,
                        const double *commandUs, double *position, double *velocity,
                        uint32_t *status, uint8_t *valid)
{
    if (!values || !timeUs || !commandUs || !position || !velocity || !status || !valid || !count ||
        !isfinite(initialPosition) || !isfinite(initialCommand) ||
        fabs(initialPosition) > FLT_MAX || fabs(initialCommand) > FLT_MAX) {
        return 0;
    }
    for (unsigned i = 0; i < 9; i++) {
        if (!isfinite(values[i]) || fabs(values[i]) > FLT_MAX) {
            return 0;
        }
    }
    if (values[4] < 0 || values[4] > UINT32_MAX) {
        return 0;
    }
    for (size_t i = 0; i < count; i++) {
        if (!isfinite(timeUs[i]) || timeUs[i] < 0 || timeUs[i] > 9007199254740991. ||
            !isfinite(commandUs[i]) || fabs(commandUs[i]) > FLT_MAX) {
            return 0;
        }
    }
    tricopterServoModelParams_t params = {0};
    params.knotCount = 3;
    params.pwmUs[0] = 1100;
    params.pwmUs[1] = 1420;
    params.pwmUs[2] = 1700;
    for (unsigned i = 0; i < 3; i++) {
        params.positionCounts[i] = (float)values[i];
    }
    params.backlashUs = (float)values[3];
    params.delayUs = (uint32_t)llround(values[4]);
    params.tauIncreasingPwmS = (float)values[5];
    params.tauDecreasingPwmS = (float)values[6];
    params.rateIncreasingPwmCountsS = (float)values[7];
    params.rateDecreasingPwmCountsS = (float)values[8];
    params.maxGapUs = 5000;
    tricopterServoModelState_t state;
    if (!count || !tricopterServoModelReset(&state, &params, (uint64_t)llround(timeUs[0]),
                                           (float)initialCommand, (float)initialPosition, initialDirection)) {
        return 0;
    }
    for (size_t i = 0; i < count; i++) {
        const tricopterServoModelOutput_t out = tricopterServoModelStep(&state, (uint64_t)llround(timeUs[i]), (float)commandUs[i]);
        position[i] = out.positionCounts;
        velocity[i] = out.velocityCountsS;
        status[i] = out.status;
        valid[i] = out.valid;
        if (out.angleCalibrated) {
            return -1; /* This adapter never carries angle calibration. */
        }
    }
    return 1;
}
