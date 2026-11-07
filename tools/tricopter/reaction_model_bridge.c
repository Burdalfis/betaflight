/* Offline batch transport to the actual portable C reaction-torque core. */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "flight/tricopter_reaction_torque.h"

int tricopterReactionReplay(const double *values, uint32_t coefficientSource, size_t count,
    const float *rpm, const float *accelerationRpmS, const uint32_t *ageUs,
    const uint8_t *seen, const uint8_t *fresh, const uint8_t *valid, const uint8_t *accelerationValid,
    float *omega, float *alpha, float *aerodynamicNm, float *inertialNm, float *totalNm, float *summedNm,
    uint32_t *status, uint8_t *invalidMotorMask, uint8_t *outputValid, uint8_t *calibrated, uint8_t *forFlight)
{
    if (!values || !count || count > SIZE_MAX / 3 || coefficientSource > 1 || !rpm || !accelerationRpmS
        || !ageUs || !seen || !fresh || !valid || !accelerationValid || !omega || !alpha || !aerodynamicNm
        || !inertialNm || !totalNm || !summedNm || !status || !invalidMotorMask || !outputValid || !calibrated || !forFlight) {
        return 0;
    }
    for (unsigned i = 0; i < 13; ++i) {
        if (!isfinite(values[i]) || fabs(values[i]) > FLT_MAX
            || (i >= 6 && i < 9 && (values[i] < INT8_MIN || values[i] > INT8_MAX || values[i] != floor(values[i])))
            || (i == 12 && (values[i] < 0 || values[i] > UINT32_MAX || values[i] != floor(values[i])))) {
            return 0;
        }
    }
    tricopterReactionTorqueParams_t params = {
        .coefficientSource = (tricopterReactionTorqueCoefficientSource_e)coefficientSource,
        .maxMechanicalRpm = (float)values[9],
        .maxAbsAccelerationRpmS = (float)values[10],
        .maxAbsTorqueNm = (float)values[11],
        .maxRpmAgeUs = (uint32_t)values[12],
    };
    for (unsigned motor = 0; motor < 3; ++motor) {
        params.kQ[motor] = (float)values[motor];
        params.inertiaKgM2[motor] = (float)values[3+motor];
        params.reactionSign[motor] = (int8_t)values[6+motor];
    }
    for (size_t row = 0; row < count; ++row) {
        tricopterReactionTorqueMotorState_t motors[3];
        for (unsigned motor = 0; motor < 3; ++motor) {
            const size_t i = 3*row+motor;
            motors[motor] = (tricopterReactionTorqueMotorState_t) {
                .mechanicalRpm = rpm[i], .accelerationRpmS = accelerationRpmS[i], .ageUs = ageUs[i],
                .seen = seen[i] != 0, .fresh = fresh[i] != 0, .valid = valid[i] != 0,
                .accelerationValid = accelerationValid[i] != 0,
            };
        }
        const tricopterReactionTorqueOutput_t out = tricopterReactionTorqueEvaluate(&params, motors);
        for (unsigned motor = 0; motor < 3; ++motor) {
            const size_t i = 3*row+motor;
            omega[i] = out.omegaRadS[motor];
            alpha[i] = out.alphaRadS2[motor];
            aerodynamicNm[i] = out.aerodynamicNm[motor];
            inertialNm[i] = out.inertialNm[motor];
            totalNm[i] = out.totalNm[motor];
        }
        summedNm[3*row] = out.summedAerodynamicNm;
        summedNm[3*row+1] = out.summedInertialNm;
        summedNm[3*row+2] = out.summedTotalNm;
        status[row] = out.status;
        invalidMotorMask[row] = out.invalidMotorMask;
        outputValid[row] = out.valid;
        calibrated[row] = out.coefficientsCalibrated;
        forFlight[row] = out.forFlight;
    }
    return 1;
}
