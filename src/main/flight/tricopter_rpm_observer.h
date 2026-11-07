/* Offline mechanical-RPM observer. No actuator, telemetry or scheduler I/O. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// These are software/filter guards, not identified motor constants or physical
// acceleration limits. No command/voltage model or torque coefficients exist.
typedef struct tricopterRpmObserverParams_s {
    float trackingTauS;
    float staleAccelerationDecayTauS;
    float maxRpm;
    float maxAccelerationRpmS;
    uint32_t staleAfterUs;
    uint32_t maxPredictionUs;
    uint32_t maxCallGapUs;
} tricopterRpmObserverParams_t;

typedef struct tricopterRpmObservation_s {
    bool seen; // False/NULL means no observation (including EDT/missing frames).
    uint32_t observedAtUs; // Actual decoder observation, not ESC measurement time.
    uint32_t sequence; // Identity, independent of RPM value; wraps modulo 2^32.
    float mechanicalRpm; // Nonnegative mechanical RPM; a genuine zero is valid.
} tricopterRpmObservation_t;

typedef enum tricopterRpmObserverStatus_e {
    TRICOPTER_RPM_OBSERVER_OK = 0,
    TRICOPTER_RPM_OBSERVER_NOT_INITIALIZED = 1U << 0,
    TRICOPTER_RPM_OBSERVER_INVALID_CONFIG = 1U << 1,
    TRICOPTER_RPM_OBSERVER_INVALID_TIME = 1U << 2,
    TRICOPTER_RPM_OBSERVER_CALL_GAP = 1U << 3,
    TRICOPTER_RPM_OBSERVER_INVALID_OBSERVATION = 1U << 4,
    TRICOPTER_RPM_OBSERVER_OUT_OF_ORDER = 1U << 5,
    TRICOPTER_RPM_OBSERVER_METADATA_MISMATCH = 1U << 6,
    TRICOPTER_RPM_OBSERVER_UNSEEN = 1U << 7,
    TRICOPTER_RPM_OBSERVER_STALE = 1U << 8,
    TRICOPTER_RPM_OBSERVER_HORIZON_EXCEEDED = 1U << 9,
    TRICOPTER_RPM_OBSERVER_REACQUIRED = 1U << 10,
    TRICOPTER_RPM_OBSERVER_COALESCED_TIME = 1U << 11,
} tricopterRpmObserverStatus_e;

typedef struct tricopterRpmObserverOutput_s {
    float mechanicalRpm; // Prediction at nowUs, finite even when invalid.
    float accelerationRpmS; // Filter state/prediction, not measured acceleration.
    uint32_t ageUs; // Saturates; UINT32_MAX when unseen.
    uint32_t sequence;
    uint32_t status;
    bool seen;
    bool acceptedNewObservation;
    bool fresh;
    bool valid; // Prediction horizon and software guards, not physical accuracy.
} tricopterRpmObserverOutput_t;

typedef struct tricopterRpmObserverState_s {
    tricopterRpmObserverParams_t params;
    uint32_t queryTimeUs;
    uint32_t observationTimeUs;
    uint32_t observationAgeUs;
    uint32_t sequence;
    uint32_t latchedStatus;
    float observationRpm;
    float estimatedRpm;
    float estimatedAccelerationRpmS;
    bool initialized;
    bool seen;
} tricopterRpmObserverState_t;

bool tricopterRpmObserverValidateParams(const tricopterRpmObserverParams_t *params);
// Copies parameters (including when params aliases state.params), clears all
// observation history, and starts a new clock/sequence epoch. No measurement.
bool tricopterRpmObserverReset(tricopterRpmObserverState_t *state,
    const tricopterRpmObserverParams_t *params, uint32_t nowUs);

// Clock/sequence arithmetic is modulo 2^32; successive forward distances must
// be <2^31. Backward/ambiguous clock or identity, excessive call gap, or changed
// metadata for an existing sequence latches invalid until explicit reset.
// Numerical/age validation precedes identity checks: invalid RPM/future/too-old
// observations are rejected transiently without refreshing age, even when
// they reuse a sequence. Plausible changed same-sequence metadata latches.
// Equal-valued new sequences (including zero) are accepted. A new sequence at
// the same observation timestamp replaces speed and clears acceleration since
// no time derivative is identifiable. A telemetry gap beyond the prediction
// horizon reacquires with speed=measurement, acceleration=0. Repeated queries
// project a copy of observation-anchored state; they never double-integrate.
// Acceleration decays beyond staleAfterUs. At the prediction horizon speed is
// held and acceleration becomes zero; valid/fresh remain separate from seen.
tricopterRpmObserverOutput_t tricopterRpmObserverStep(tricopterRpmObserverState_t *state,
    uint32_t nowUs, const tricopterRpmObservation_t *observation);
