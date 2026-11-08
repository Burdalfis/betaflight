/* Offline actuator/response coordinator. No PID, body dynamics or firmware I/O. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tricopter_allocator.h"
#include "tricopter_reaction_torque.h"
#include "tricopter_rpm_observer.h"
#include "tricopter_servo_model.h"
#include "tricopter_dynamic_allocator.h"
#include "tricopter_angle_calibration.h"

typedef enum tricopterSimulationMode_e {
    TRICOPTER_SIMULATION_BASELINE = 0,
    TRICOPTER_SIMULATION_REACTION_COMPENSATION = 1U << 0,
    TRICOPTER_SIMULATION_TAIL_VERTICAL_COORDINATION = 1U << 1,
    // May combine with reaction compensation, not legacy vertical coordination.
    TRICOPTER_SIMULATION_DYNAMIC_ALLOCATION = 1U << 2,
} tricopterSimulationMode_e;

typedef struct tricopterSimulationParams_s {
    tricopterAllocatorVirtualParams_t allocator;
    tricopterServoModelParams_t servo;
    tricopterRpmObserverParams_t rpm[3];
    tricopterReactionTorqueParams_t reaction;
    // Explicit synthetic effort/(rad/s)^2; not physical thrust calibration.
    float effortPerOmegaSquared[3];
    // Legacy synthetic linear corrected-count <-> mathematical virtual direction.
    // Endpoints correspond to allocator direction min/max; either count sign.
    // Ignored by the opt-in measured map after configuration validation.
    float correctedCountsAtDirectionMin;
    float correctedCountsAtDirectionMax;
    // Synthetic additive yaw disturbance, independent of virtual direction.
    // Controller AND plant effectiveness maps must exclude this contribution.
    float yawResponsePerNm;
    float maxAbsReactionCompensationResponse;
    float minTailSupportCosine; // Explicit software guard, [0.001,1].
    bool effectivenessExcludesReaction; // Must be explicitly true.
    bool dynamicConfigured; // False preserves all existing offline modes.
    tricopterDynamicAllocatorParams_t dynamic;
    bool measuredAngleConfigured; // Opt-in; false preserves synthetic mapping.
    tricopterAngleCalibration_t measuredAngle;
} tricopterSimulationParams_t;

typedef struct tricopterSimulationInitialState_s {
    uint64_t timeUs;
    float correctedCounts; // Sole position input: valid canonical PB0-PC0.
    float pwmUs;
    int8_t approachDirection; // Servo model initialization convention.
    bool correctedPositionValid;
} tricopterSimulationInitialState_t;

typedef struct tricopterSimulationInput_s {
    float requestedResponse[4];
    tricopterRpmObservation_t rpm[3]; // Actual decoder timestamps/identities.
    uint8_t modes; // Only the above explicitly named bits are accepted.
} tricopterSimulationInput_t;

typedef enum tricopterSimulationStatus_e {
    TRICOPTER_SIMULATION_OK = 0,
    TRICOPTER_SIMULATION_INVALID_CONFIG = 1U << 0,
    TRICOPTER_SIMULATION_INVALID_INITIAL_STATE = 1U << 1,
    TRICOPTER_SIMULATION_INVALID_INPUT = 1U << 2,
    TRICOPTER_SIMULATION_INVALID_TIME = 1U << 3,
    TRICOPTER_SIMULATION_RPM_UNAVAILABLE = 1U << 4,
    TRICOPTER_SIMULATION_ACCELERATION_UNAVAILABLE = 1U << 5,
    TRICOPTER_SIMULATION_REACTION_UNAVAILABLE = 1U << 6,
    TRICOPTER_SIMULATION_REACTION_COMPENSATION_CLIPPED = 1U << 7,
    TRICOPTER_SIMULATION_NET_REQUEST_CLIPPED = 1U << 8,
    TRICOPTER_SIMULATION_SERVO_PREDICTION_UNAVAILABLE = 1U << 9,
    TRICOPTER_SIMULATION_TAIL_COSINE_GUARD = 1U << 10,
    TRICOPTER_SIMULATION_TAIL_EFFORT_CLIPPED = 1U << 11,
    TRICOPTER_SIMULATION_ALLOCATOR_NOT_CONVERGED = 1U << 12,
    TRICOPTER_SIMULATION_PREDICTED_RESPONSE_UNAVAILABLE = 1U << 13,
    TRICOPTER_SIMULATION_COORDINATED_RESPONSE_UNAVAILABLE = 1U << 14,
    TRICOPTER_SIMULATION_NOT_INITIALIZED = 1U << 15,
    TRICOPTER_SIMULATION_DYNAMIC_UNAVAILABLE = 1U << 16,
} tricopterSimulationStatus_e;

typedef struct tricopterSimulationOutput_s {
    float requestedResponse[4];
    float netRequestedResponse[4];
    tricopterAllocatorVirtualOutput_t nominalAllocation;
    tricopterRpmObserverOutput_t rpm[3];
    tricopterReactionTorqueOutput_t reaction;
    tricopterServoModelOutput_t servo; // Prediction immediately before new PWM.
    float appliedReactionCompensationResponse;
    float targetDirectionRad; // Nominal static allocator goal, also in dynamic mode.
    float pwmCommandUs;
    float commandedEffort[3];
    float commandedOmegaRadS[3];
    float commandedMechanicalRpm[3];
    float predictedDirectionRad;
    // Commanded motor efforts evaluated at predicted CURRENT direction. These
    // are target diagnostics, not instantaneous achieved motor state/optimality.
    float coordinatedTargetControls[4];
    float coordinatedTargetActuatorResponse[4];
    float coordinatedTargetTotalResponse[4];
    float postCoordinationResidual[4]; // Total minus original requested response.
    // CURRENT motor estimates from observed/predicted RPM, never command effort.
    float predictedControls[4];
    float predictedActuatorResponse[4];
    float predictedTotalResponse[4];
    bool accelerationValid[3];
    bool servoPredictionAvailable;
    bool reactionAvailable;
    bool coordinatedActuatorResponseAvailable;
    bool coordinatedTotalResponseAvailable;
    bool predictedActuatorResponseAvailable;
    bool predictedTotalResponseAvailable;
    uint32_t status;
    bool valid; // Bounded feasible commands; not submodel/allocator convergence.
    bool calibrated; // Always false.
    bool forFlight; // Always false.
    tricopterDynamicAllocatorOutput_t dynamicAllocation;
    bool dynamicAllocationUsed; // False on explicit static nominal fallback.
    bool measuredAngleAvailable; // Current prediction converted by measured map.
    float angleHalfBranchSeparationRad; // Descriptive major-loop envelope only.
} tricopterSimulationOutput_t;

typedef struct tricopterSimulationState_s {
    tricopterSimulationParams_t params;
    tricopterServoModelState_t servo;
    tricopterRpmObserverState_t rpm[3];
    uint64_t timeUs;
    float previousPwmUs;
    uint32_t qualifiedObservationTimeUs[3];
    uint8_t freshDistinctObservations[3];
    uint32_t latchedStatus;
    bool initialized;
} tricopterSimulationState_t;

bool tricopterSimulationValidateParams(const tricopterSimulationParams_t *params);
bool tricopterSimulationReset(tricopterSimulationState_t *state,
    const tricopterSimulationParams_t *params, const tricopterSimulationInitialState_t *initial);

// Causal tick: predictor advances under previous command; RPM observations are
// processed; optional bounded reaction scalar is subtracted once; allocation
// sets direction/PWM. Optional virtual-tail-vertical coordination sets only
// tail effort=clip(allocatedVertical/cos(predictedCurrentDirection),0,max).
// Arbitrary response matrices need not preserve collective under that rule.
// Newly issued commands apply from this boundary onward, never retroactively.
// No truth samples, measured positions, or future observations after reset.
// Full total-response predictions require known reaction; no unknown term is
// silently treated as measured zero. Acceleration requires two distinct fresh
// timestamps after initialization/reacquisition/coalescing/staleness.
// Invalid tick/time/config latches invalid until reset, zero numeric output.
// Missing/submodel-invalid data instead degrades explicitly to feasible nominal
// commands with quality flags. Public fields remain finite and bounded.
tricopterSimulationOutput_t tricopterSimulationTick(tricopterSimulationState_t *state,
    uint64_t nowUs, const tricopterSimulationInput_t *input);
