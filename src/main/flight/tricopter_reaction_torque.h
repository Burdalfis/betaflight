/* Offline near-vertical yaw reaction equations; no control or actuator I/O. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TRICOPTER_REACTION_TORQUE_MOTORS 3U

typedef enum tricopterReactionTorqueCoefficientSource_e {
    TRICOPTER_REACTION_TORQUE_COEFFICIENTS_UNSPECIFIED = 0,
    TRICOPTER_REACTION_TORQUE_COEFFICIENTS_SYNTHETIC_PLACEHOLDER = 1,
} tricopterReactionTorqueCoefficientSource_e;

typedef struct tricopterReactionTorqueParams_s {
    float kQ[TRICOPTER_REACTION_TORQUE_MOTORS]; // N*m/(rad/s)^2; nonnegative.
    float inertiaKgM2[TRICOPTER_REACTION_TORQUE_MOTORS]; // J; nonnegative.
    // Reaction on body: -1 for CCW rotor, +1 for CW, viewed from above.
    // Aircraft logical order rear/front-right/front-left is [-1,+1,-1].
    int8_t reactionSign[TRICOPTER_REACTION_TORQUE_MOTORS];
    tricopterReactionTorqueCoefficientSource_e coefficientSource;
    // Explicit software guards, not identified physical bounds. RPM <=1e7,
    // |RPM/s| <=1e10; the torque guard applies to every component and sum.
    float maxMechanicalRpm;
    float maxAbsAccelerationRpmS;
    float maxAbsTorqueNm;
    uint32_t maxRpmAgeUs;
} tricopterReactionTorqueParams_t;

typedef struct tricopterReactionTorqueMotorState_s {
    float mechanicalRpm; // Rotation magnitude, already converted from eRPM.
    float accelerationRpmS; // Signed derivative of the magnitude, RPM/s.
    uint32_t ageUs;
    bool seen;
    bool fresh;
    bool valid;
    // An initialized/reacquired/coalesced observer's zero derivative is not
    // a known acceleration. Caller must explicitly establish validity.
    bool accelerationValid;
} tricopterReactionTorqueMotorState_t;

typedef enum tricopterReactionTorqueStatus_e {
    TRICOPTER_REACTION_TORQUE_OK = 0,
    TRICOPTER_REACTION_TORQUE_INVALID_CONFIG = 1U << 0,
    TRICOPTER_REACTION_TORQUE_INVALID_COEFFICIENT_SOURCE = 1U << 1,
    TRICOPTER_REACTION_TORQUE_INVALID_INPUT = 1U << 2,
    TRICOPTER_REACTION_TORQUE_RPM_UNSEEN = 1U << 3,
    TRICOPTER_REACTION_TORQUE_RPM_STALE = 1U << 4,
    TRICOPTER_REACTION_TORQUE_RPM_INVALID = 1U << 5,
    TRICOPTER_REACTION_TORQUE_ACCELERATION_UNAVAILABLE = 1U << 6,
    TRICOPTER_REACTION_TORQUE_INPUT_BOUNDS = 1U << 7,
    TRICOPTER_REACTION_TORQUE_TORQUE_BOUNDS = 1U << 8,
} tricopterReactionTorqueStatus_e;

typedef struct tricopterReactionTorqueOutput_s {
    float omegaRadS[TRICOPTER_REACTION_TORQUE_MOTORS];
    float alphaRadS2[TRICOPTER_REACTION_TORQUE_MOTORS];
    float aerodynamicNm[TRICOPTER_REACTION_TORQUE_MOTORS];
    float inertialNm[TRICOPTER_REACTION_TORQUE_MOTORS];
    float totalNm[TRICOPTER_REACTION_TORQUE_MOTORS];
    float summedAerodynamicNm;
    float summedInertialNm;
    float summedTotalNm;
    uint32_t status;
    uint8_t invalidMotorMask;
    bool valid; // Synthetic equation evaluation only, not physical accuracy.
    bool coefficientsCalibrated; // Always false.
    bool forFlight; // Always false.
} tricopterReactionTorqueOutput_t;

bool tricopterReactionTorqueValidateParams(const tricopterReactionTorqueParams_t *params);

// Stateless, allocation-free equations in the plan's body-yaw convention:
// omega=mechanicalRPM*pi/30, alpha=RPM/s*pi/30,
// aero_i=sign_i*kQ_i*omega_i^2, inertia_i=sign_i*J_i*alpha_i.
// This is frame/mount reaction for near-vertical axes and negligible body
// motion, not total external torque of the frame+rotors. Future angular-
// momentum bookkeeping must not add the same J*alpha contribution twice.
// No angle, thrust, gyroscopic or 3D model. There are
// no coefficient defaults or calibrated provenance accepted by this API.
// Every motor must be seen/fresh/valid with known acceleration, even if J=0.
// Numeric validation survives fast-math. Binary64 intermediate products/sums
// are checked before float32 output conversion. Any invalid input/component/
// per-motor sum/grand sum clears ALL numerical outputs; no partial torque.
tricopterReactionTorqueOutput_t tricopterReactionTorqueEvaluate(
    const tricopterReactionTorqueParams_t *params,
    const tricopterReactionTorqueMotorState_t motors[TRICOPTER_REACTION_TORQUE_MOTORS]);
