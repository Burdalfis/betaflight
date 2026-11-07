/* Offline finite-horizon virtual-tail allocation. No firmware/actuator I/O. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tricopter_allocator.h"
#include "tricopter_servo_model.h"

#define TRICOPTER_DYNAMIC_ALLOCATOR_MAX_GRID 17U
#define TRICOPTER_DYNAMIC_ALLOCATOR_MAX_CANDIDATES 19U
#define TRICOPTER_DYNAMIC_ALLOCATOR_MAX_SERVO_STEPS 256U

// Shared model inputs, inherited from the offline coordinator: no duplicate
// geometry/airframe config, physical angle, or newly inferred coefficients.
typedef struct tricopterDynamicAllocatorModel_s {
    tricopterAllocatorVirtualParams_t allocator;
    float correctedCountsAtDirectionMin;
    float correctedCountsAtDirectionMax;
    float effortPerOmegaSquared[3];
} tricopterDynamicAllocatorModel_t;

typedef struct tricopterDynamicAllocatorParams_s {
    uint32_t horizonUs; // 1..1e6; must exceed predictor command delay.
    uint32_t maxRpmAgeUs;
    float motorTauIncreasingS[3];
    float motorTauDecreasingS[3];
    float motorSlewIncreasingRpmS[3]; // Zero disables slew limit.
    float motorSlewDecreasingRpmS[3];
    float pwmGridUs[TRICOPTER_DYNAMIC_ALLOCATOR_MAX_GRID];
    uint8_t gridCount; // 0..17; current and nominal PWM are always appended.
    float maxHighMatrixCondition; // [1,1e8], weighted high 3x3 matrix.
    // Absolute NUMERICAL tie tolerances, each in [0,1e-6]. No yaw weight can
    // trade a larger high-group cost outside this explicitly declared band.
    float highCostTolerance;
    float yawCostTolerance;
    float movementCostTolerance;
} tricopterDynamicAllocatorParams_t;

typedef struct tricopterDynamicAllocatorMotorState_s {
    float mechanicalRpm;
    uint32_t ageUs;
    bool seen;
    bool fresh;
    bool valid;
} tricopterDynamicAllocatorMotorState_t;

typedef struct tricopterDynamicAllocatorInput_s {
    uint64_t nowUs;
    float requestedResponse[4]; // Net response, held constant over horizon.
    float currentPwmUs;
    float nominalPwmUs;
    tricopterDynamicAllocatorMotorState_t motors[3];
} tricopterDynamicAllocatorInput_t;

typedef enum tricopterDynamicAllocatorStatus_e {
    TRICOPTER_DYNAMIC_ALLOCATOR_OK = 0,
    TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_CONFIG = 1U << 0,
    TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_INPUT = 1U << 1,
    TRICOPTER_DYNAMIC_ALLOCATOR_STATE_UNAVAILABLE = 1U << 2,
    TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_HORIZON = 1U << 3,
    TRICOPTER_DYNAMIC_ALLOCATOR_SERVO_PREDICTION_UNAVAILABLE = 1U << 4,
    TRICOPTER_DYNAMIC_ALLOCATOR_HIGH_MATRIX_UNAVAILABLE = 1U << 5,
    TRICOPTER_DYNAMIC_ALLOCATOR_EMPTY_REACHABLE_BOX = 1U << 6,
    TRICOPTER_DYNAMIC_ALLOCATOR_NO_DIRECTION_AUTHORITY = 1U << 7,
    TRICOPTER_DYNAMIC_ALLOCATOR_NO_VALID_CANDIDATE = 1U << 8,
    TRICOPTER_DYNAMIC_ALLOCATOR_PWM_TARGET_CLIPPED = 1U << 9,
} tricopterDynamicAllocatorStatus_e;

typedef struct tricopterDynamicAllocatorCandidate_s {
    float pwmUs;
    float horizonCorrectedCounts;
    float horizonDirectionRad;
    float continuousOptimalEffort[3]; // Logical tail/front-right/front-left.
    float commandMechanicalRpm[3]; // Authoritative published commands.
    float predictedMechanicalRpm[3];
    float predictedEffort[3];
    float predictedControls[4];
    float predictedResponse[4];
    float residualResponse[4];
    double highCost; // 0.5 sum high3 normalized residual squared.
    double yawCost; // 0.5 normalized yaw residual squared.
    double movementCost; // Sum normalized effort-change² + PWM-change².
    float highMatrixCondition;
    uint32_t status;
    uint8_t evaluatedFaces;
    bool valid;
} tricopterDynamicAllocatorCandidate_t;

typedef struct tricopterDynamicAllocatorOutput_s {
    float minimumPwmUs;
    float maximumPwmUs;
    float reachableMinEffort[3];
    float reachableMaxEffort[3];
    float minimumCommandRpm[3];
    float maximumCommandRpm[3];
    tricopterDynamicAllocatorCandidate_t candidates[TRICOPTER_DYNAMIC_ALLOCATOR_MAX_CANDIDATES];
    tricopterDynamicAllocatorCandidate_t selected;
    double minimumHighCost;
    double minimumYawCostWithinHighTolerance;
    uint32_t status;
    uint8_t candidateCount;
    uint8_t validCandidateCount;
    uint8_t selectedCandidate;
    bool valid;
    bool priorityAvailable;
    bool calibrated; // Always false.
    bool forFlight; // Always false.
} tricopterDynamicAllocatorOutput_t;

bool tricopterDynamicAllocatorValidateParams(const tricopterDynamicAllocatorModel_t *model,
    const tricopterDynamicAllocatorParams_t *params, const tricopterServoModelParams_t *servo);

// Copies the CURRENT predictor (its time must equal nowUs), preserves queued
// command history, queues each candidate now, and advances only the copy under
// a held command in <=256 substeps. No future truth/measurement is accepted.
// Motor min/max reachable H efforts come from explicit monotone first-order/
// slew dynamics under command bounds; future efforts also respect static max.
// Each accepted candidate requires a well-conditioned full-rank high3 matrix.
// Its unique primary box optimum is found over 27 active faces, then inverted
// to bounded float32 RPM commands and FORWARDED again for all reported costs.
// Global selection: min published high cost, min yaw within high tolerance,
// then movement within both tolerances; deterministic order resolves final tie.
// Guarantee is over these sampled, accepted candidate primary optima, not all
// servo positions, rank-deficient maps, or the entire primary-tolerance motor
// sublevel set. Unavailable state/authority never means fabricated zero state.
tricopterDynamicAllocatorOutput_t tricopterDynamicAllocatorAllocate(
    const tricopterDynamicAllocatorModel_t *model, const tricopterDynamicAllocatorParams_t *params,
    const tricopterServoModelState_t *currentServo, const tricopterDynamicAllocatorInput_t *input);
