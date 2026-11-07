/* Offline near-vertical yaw reaction equations; no firmware I/O. */
#include "tricopter_reaction_torque.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128 || DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024
#error "Reaction torque requires IEEE-754 binary32 float and binary64 double"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "Reaction torque requires 32-bit float");
_Static_assert(sizeof(double) == sizeof(uint64_t), "Reaction torque requires 64-bit double");

#define ALL_MOTORS_MASK ((1U << TRICOPTER_REACTION_TORQUE_MOTORS) - 1U)

static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

bool tricopterReactionTorqueValidateParams(const tricopterReactionTorqueParams_t *params)
{
    if (!params || params->coefficientSource != TRICOPTER_REACTION_TORQUE_COEFFICIENTS_SYNTHETIC_PLACEHOLDER
        || !finiteFloat(params->maxMechanicalRpm) || params->maxMechanicalRpm <= 0.0f || params->maxMechanicalRpm > 1e7f
        || !finiteFloat(params->maxAbsAccelerationRpmS) || params->maxAbsAccelerationRpmS <= 0.0f || params->maxAbsAccelerationRpmS > 1e10f
        || !finiteFloat(params->maxAbsTorqueNm) || params->maxAbsTorqueNm <= 0.0f
        || !params->maxRpmAgeUs || params->maxRpmAgeUs >= UINT32_C(0x80000000)) {
        return false;
    }
    for (unsigned i = 0; i < TRICOPTER_REACTION_TORQUE_MOTORS; ++i) {
        if (!finiteFloat(params->kQ[i]) || params->kQ[i] < 0.0f
            || !finiteFloat(params->inertiaKgM2[i]) || params->inertiaKgM2[i] < 0.0f
            || (params->reactionSign[i] != -1 && params->reactionSign[i] != 1)) {
            return false;
        }
    }
    return true;
}

tricopterReactionTorqueOutput_t tricopterReactionTorqueEvaluate(
    const tricopterReactionTorqueParams_t *params,
    const tricopterReactionTorqueMotorState_t motors[TRICOPTER_REACTION_TORQUE_MOTORS])
{
    tricopterReactionTorqueOutput_t out = {0};
    if (!tricopterReactionTorqueValidateParams(params)) {
        out.status = TRICOPTER_REACTION_TORQUE_INVALID_CONFIG;
        if (params && params->coefficientSource != TRICOPTER_REACTION_TORQUE_COEFFICIENTS_SYNTHETIC_PLACEHOLDER) {
            out.status |= TRICOPTER_REACTION_TORQUE_INVALID_COEFFICIENT_SOURCE;
        }
        out.invalidMotorMask = ALL_MOTORS_MASK;
        return out;
    }
    if (!motors) {
        out.status = TRICOPTER_REACTION_TORQUE_INVALID_INPUT;
        out.invalidMotorMask = ALL_MOTORS_MASK;
        return out;
    }
    for (unsigned i = 0; i < TRICOPTER_REACTION_TORQUE_MOTORS; ++i) {
        uint32_t status = 0;
        if (!motors[i].seen) {
            status |= TRICOPTER_REACTION_TORQUE_RPM_UNSEEN;
        }
        if (!motors[i].fresh || motors[i].ageUs > params->maxRpmAgeUs) {
            status |= TRICOPTER_REACTION_TORQUE_RPM_STALE;
        }
        if (!motors[i].valid) {
            status |= TRICOPTER_REACTION_TORQUE_RPM_INVALID;
        }
        if (!motors[i].accelerationValid) {
            status |= TRICOPTER_REACTION_TORQUE_ACCELERATION_UNAVAILABLE;
        }
        if (!finiteFloat(motors[i].mechanicalRpm) || !finiteFloat(motors[i].accelerationRpmS)) {
            status |= TRICOPTER_REACTION_TORQUE_INVALID_INPUT;
        } else if (motors[i].mechanicalRpm < 0.0f || motors[i].mechanicalRpm > params->maxMechanicalRpm
            || fabsf(motors[i].accelerationRpmS) > params->maxAbsAccelerationRpmS) {
            status |= TRICOPTER_REACTION_TORQUE_INPUT_BOUNDS;
        }
        if (status) {
            out.status |= status;
            out.invalidMotorMask |= 1U << i;
        }
    }
    if (out.status) {
        return out;
    }

    // Every input is a finite float. With omega<=1.1e6 and |alpha|<=1.1e9,
    // even FLT_MAX coefficients produce <1e52 products/sums: binary64 cannot
    // overflow. Check every torque against the explicit software bound before
    // any float conversion; opposing huge terms must not hide a guard fault.
    const double rpmToRadians = 3.14159265358979323846264338327950288 / 30.0;
    double omega[TRICOPTER_REACTION_TORQUE_MOTORS];
    double alpha[TRICOPTER_REACTION_TORQUE_MOTORS];
    double aerodynamic[TRICOPTER_REACTION_TORQUE_MOTORS];
    double inertial[TRICOPTER_REACTION_TORQUE_MOTORS];
    double total[TRICOPTER_REACTION_TORQUE_MOTORS];
    double sumAerodynamic = 0.0, sumInertial = 0.0, sumTotal = 0.0;
    const double bound = params->maxAbsTorqueNm;
    for (unsigned i = 0; i < TRICOPTER_REACTION_TORQUE_MOTORS; ++i) {
        omega[i] = (double)motors[i].mechanicalRpm * rpmToRadians;
        alpha[i] = (double)motors[i].accelerationRpmS * rpmToRadians;
        aerodynamic[i] = (double)params->reactionSign[i] * params->kQ[i] * omega[i] * omega[i];
        inertial[i] = (double)params->reactionSign[i] * params->inertiaKgM2[i] * alpha[i];
        total[i] = aerodynamic[i] + inertial[i];
        if (fabs(aerodynamic[i]) > bound || fabs(inertial[i]) > bound || fabs(total[i]) > bound) {
            out.status |= TRICOPTER_REACTION_TORQUE_TORQUE_BOUNDS;
            out.invalidMotorMask |= 1U << i;
        }
        sumAerodynamic += aerodynamic[i];
        sumInertial += inertial[i];
        sumTotal += total[i];
    }
    if (fabs(sumAerodynamic) > bound || fabs(sumInertial) > bound || fabs(sumTotal) > bound) {
        out.status |= TRICOPTER_REACTION_TORQUE_TORQUE_BOUNDS;
        out.invalidMotorMask = ALL_MOTORS_MASK;
    }
    if (out.status) {
        return out;
    }
    for (unsigned i = 0; i < TRICOPTER_REACTION_TORQUE_MOTORS; ++i) {
        out.omegaRadS[i] = (float)omega[i];
        out.alphaRadS2[i] = (float)alpha[i];
        out.aerodynamicNm[i] = (float)aerodynamic[i];
        out.inertialNm[i] = (float)inertial[i];
        out.totalNm[i] = (float)total[i];
    }
    out.summedAerodynamicNm = (float)sumAerodynamic;
    out.summedInertialNm = (float)sumInertial;
    out.summedTotalNm = (float)sumTotal;
    out.valid = true;
    // Zero-initialized coefficientsCalibrated/forFlight deliberately stay false.
    return out;
}
