/* Offline batch transport to the actual geometric allocator and forward API. */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "flight/tricopter_allocator.h"

int tricopterAllocatorReplay(const double *values, uint32_t source, uint32_t mode, size_t count,
    const float *request, const float *thrust, const float *angle, const float *reaction,
    const uint8_t *useReaction, const uint8_t *reactionValid,
    float *numeric, uint32_t *integer, uint8_t *flags)
{
    if (!values || source > 1 || mode > 1 || !count || count > SIZE_MAX/32 || !request || !thrust || !angle
        || !reaction || !useReaction || !reactionValid || !numeric || !integer || !flags) {
        return 0;
    }
    for (unsigned i = 0; i < 25; ++i) {
        if (!isfinite(values[i]) || fabs(values[i]) > FLT_MAX
            || (i == 24 && (values[i] < 0 || values[i] > UINT16_MAX || values[i] != floor(values[i])))) {
            return 0;
        }
    }
    tricopterAllocatorParams_t params = {0};
    for (unsigned motor = 0; motor < 3; ++motor) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            params.motorPositionM[motor][axis] = (float)values[3*motor+axis];
        }
        params.maxThrustN[motor] = (float)values[9+motor];
        params.kT[motor] = (float)values[12+motor];
    }
    params.tailAngleMinRad = (float)values[15];
    params.tailAngleMaxRad = (float)values[16];
    for (unsigned axis = 0; axis < 4; ++axis) {
        params.wrenchScale[axis] = (float)values[17+axis];
    }
    params.lowTailThrustN = (float)values[21];
    params.projectedGradientTolerance = (float)values[22];
    params.maxConditionNumber = (float)values[23];
    params.maxIterations = (uint16_t)values[24];
    params.source = (tricopterAllocatorSource_e)source;
    for (size_t row = 0; row < count; ++row) {
        tricopterAllocatorOutput_t out;
        if (mode) {
            out = tricopterAllocatorForward(&params, thrust+3*row, angle[row], reaction[row], useReaction[row], reactionValid[row]);
        } else {
            tricopterAllocatorRequest_t target = {
                .reactionYawNm = reaction[row], .useReactionYaw = useReaction[row] != 0,
                .reactionYawValid = reactionValid[row] != 0,
            };
            for (unsigned axis = 0; axis < 4; ++axis) {
                target.wrench[axis] = request[4*row+axis];
            }
            out = tricopterAllocatorAllocate(&params, &target);
        }
        float *f = numeric+32*row;
        for (unsigned axis = 0; axis < 4; ++axis) {
            f[axis] = out.requestedWrench[axis];
            f[4+axis] = out.achievedWrench[axis];
            f[8+axis] = out.residualWrench[axis];
            f[24+axis] = out.controlsN[axis];
        }
        for (unsigned axis = 0; axis < 3; ++axis) {
            f[12+axis] = out.forceN[axis];
            f[15+axis] = out.thrustN[axis];
            f[18+axis] = out.omegaRadS[axis];
            f[21+axis] = out.mechanicalRpm[axis];
        }
        f[28] = out.tailAngleRad;
        f[29] = out.cost;
        f[30] = out.projectedGradientNorm;
        f[31] = out.conditionNumber;
        integer[3*row] = out.status;
        integer[3*row+1] = out.constraints;
        integer[3*row+2] = out.iterations;
        flags[7*row] = out.valid;
        flags[7*row+1] = out.converged;
        flags[7*row+2] = out.exact;
        flags[7*row+3] = out.tailAngleDefined;
        flags[7*row+4] = out.tailThrustLow;
        flags[7*row+5] = out.calibrated;
        flags[7*row+6] = out.forFlight;
    }
    return 1;
}

int tricopterAllocatorVirtualReplay(const double *values, uint32_t source, uint32_t mode, size_t count,
    const float *request, const float *effort, const float *direction,
    float *numeric, uint32_t *integer, uint8_t *flags)
{
    if (!values || source > 1 || mode > 1 || !count || count > SIZE_MAX/23 || !request || !effort || !direction
        || !numeric || !integer || !flags) {
        return 0;
    }
    for (unsigned i = 0; i < 29; ++i) {
        if (!isfinite(values[i]) || fabs(values[i]) > FLT_MAX
            || (i == 28 && (values[i] < 0 || values[i] > UINT16_MAX || values[i] != floor(values[i])))) {
            return 0;
        }
    }
    tricopterAllocatorVirtualParams_t params = {0};
    for (unsigned i = 0; i < 16; ++i) {
        params.effectivenessMatrix[i/4][i%4] = (float)values[i];
    }
    for (unsigned i = 0; i < 3; ++i) {
        params.maxEffort[i] = (float)values[16+i];
    }
    params.tailDirectionMinRad = (float)values[19];
    params.tailDirectionMaxRad = (float)values[20];
    for (unsigned i = 0; i < 4; ++i) {
        params.responseScale[i] = (float)values[21+i];
    }
    params.lowTailEffort = (float)values[25];
    params.projectedGradientTolerance = (float)values[26];
    params.maxConditionNumber = (float)values[27];
    params.maxIterations = (uint16_t)values[28];
    params.source = (tricopterAllocatorSource_e)source;
    for (size_t row = 0; row < count; ++row) {
        tricopterAllocatorVirtualOutput_t out;
        if (mode) {
            out = tricopterAllocatorVirtualForward(&params, effort+3*row, direction[row]);
        } else {
            tricopterAllocatorVirtualRequest_t target;
            for (unsigned axis = 0; axis < 4; ++axis) {
                target.response[axis] = request[4*row+axis];
            }
            out = tricopterAllocatorVirtualAllocate(&params, &target);
        }
        float *f = numeric+23*row;
        for (unsigned axis = 0; axis < 4; ++axis) {
            f[axis] = out.requestedResponse[axis];
            f[4+axis] = out.achievedResponse[axis];
            f[8+axis] = out.residualResponse[axis];
            f[15+axis] = out.controls[axis];
        }
        for (unsigned motor = 0; motor < 3; ++motor) {
            f[12+motor] = out.effort[motor];
        }
        f[19] = out.tailDirectionRad;
        f[20] = out.cost;
        f[21] = out.projectedGradientNorm;
        f[22] = out.conditionNumber;
        integer[3*row] = out.status;
        integer[3*row+1] = out.constraints;
        integer[3*row+2] = out.iterations;
        flags[7*row] = out.valid;
        flags[7*row+1] = out.converged;
        flags[7*row+2] = out.exact;
        flags[7*row+3] = out.tailDirectionDefined;
        flags[7*row+4] = out.tailEffortLow;
        flags[7*row+5] = out.calibrated;
        flags[7*row+6] = out.forFlight;
    }
    return 1;
}
