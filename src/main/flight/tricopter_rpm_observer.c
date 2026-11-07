/* Offline causal mechanical-RPM observer; no firmware I/O or control output. */
#include "tricopter_rpm_observer.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "The tricopter RPM observer requires IEEE-754 binary32 float"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "The tricopter RPM observer requires 32-bit float");

#define HALF_CLOCK_RANGE UINT32_C(0x80000000)

// Representation checks survive the firmware's finite-math-only assumption.
static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static float clampFloat(float value, float lower, float upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

static bool validTau(float value)
{
    return finiteFloat(value) && value >= 0.000001f && value <= 60.0f;
}

bool tricopterRpmObserverValidateParams(const tricopterRpmObserverParams_t *params)
{
    return params && validTau(params->trackingTauS) && validTau(params->staleAccelerationDecayTauS)
        && finiteFloat(params->maxRpm) && params->maxRpm > 0.0f && params->maxRpm <= 1e7f
        && finiteFloat(params->maxAccelerationRpmS) && params->maxAccelerationRpmS > 0.0f && params->maxAccelerationRpmS <= 1e10f
        && params->staleAfterUs > 0 && params->staleAfterUs <= params->maxPredictionUs
        && params->maxPredictionUs < HALF_CLOCK_RANGE && params->maxCallGapUs > 0 && params->maxCallGapUs < HALF_CLOCK_RANGE;
}

bool tricopterRpmObserverReset(tricopterRpmObserverState_t *state,
    const tricopterRpmObserverParams_t *params, uint32_t nowUs)
{
    if (!state) {
        return false;
    }
    const bool valid = tricopterRpmObserverValidateParams(params);
    tricopterRpmObserverParams_t copy = {0};
    if (valid) {
        copy = *params;
    }
    memset(state, 0, sizeof(*state));
    if (!valid) {
        state->latchedStatus = TRICOPTER_RPM_OBSERVER_INVALID_CONFIG;
        return false;
    }
    state->params = copy;
    state->queryTimeUs = nowUs;
    state->observationAgeUs = UINT32_MAX;
    state->initialized = true;
    return true;
}

// Predict from the observation epoch, never from a previous query prediction.
static void project(const tricopterRpmObserverState_t *state, uint32_t ageUs, float *rpm, float *acceleration)
{
    const tricopterRpmObserverParams_t *params = &state->params;
    const uint32_t boundedAge = ageUs < params->maxPredictionUs ? ageUs : params->maxPredictionUs;
    const uint32_t freshTime = boundedAge < params->staleAfterUs ? boundedAge : params->staleAfterUs;
    float speed = state->estimatedRpm + state->estimatedAccelerationRpmS * ((float)freshTime * 0.000001f);
    float accel = state->estimatedAccelerationRpmS;
    if (boundedAge > freshTime) {
        const float dt = (float)(boundedAge - freshTime) * 0.000001f;
        const float oneMinusDecay = -expm1f(-dt / params->staleAccelerationDecayTauS);
        speed += accel * params->staleAccelerationDecayTauS * oneMinusDecay;
        accel *= 1.0f - oneMinusDecay;
    }
    *rpm = clampFloat(speed, 0.0f, params->maxRpm);
    *acceleration = ageUs >= params->maxPredictionUs
        || (*rpm == 0.0f && accel < 0.0f) || (*rpm == params->maxRpm && accel > 0.0f) ? 0.0f : accel;
}

static tricopterRpmObserverOutput_t output(const tricopterRpmObserverState_t *state, uint32_t status, bool accepted)
{
    tricopterRpmObserverOutput_t out = { .ageUs = UINT32_MAX, .status = status };
    if (!state) {
        out.status |= TRICOPTER_RPM_OBSERVER_NOT_INITIALIZED;
        return out;
    }
    out.status |= state->latchedStatus;
    if (!state->initialized) {
        out.status |= TRICOPTER_RPM_OBSERVER_NOT_INITIALIZED;
        return out;
    }
    out.seen = state->seen;
    out.acceptedNewObservation = accepted;
    if (!state->seen) {
        out.status |= TRICOPTER_RPM_OBSERVER_UNSEEN;
        return out;
    }
    out.ageUs = state->observationAgeUs;
    out.sequence = state->sequence;
    project(state, out.ageUs, &out.mechanicalRpm, &out.accelerationRpmS);
    if (out.ageUs > state->params.staleAfterUs) {
        out.status |= TRICOPTER_RPM_OBSERVER_STALE;
    }
    if (out.ageUs > state->params.maxPredictionUs) {
        out.status |= TRICOPTER_RPM_OBSERVER_HORIZON_EXCEEDED;
    }
    out.valid = state->latchedStatus == 0 && out.ageUs <= state->params.maxPredictionUs;
    out.fresh = out.valid && out.ageUs <= state->params.staleAfterUs;
    if (!out.valid) {
        out.accelerationRpmS = 0.0f;
    }
    return out;
}

static tricopterRpmObserverOutput_t fail(tricopterRpmObserverState_t *state, uint32_t status)
{
    state->latchedStatus |= status;
    return output(state, 0, false);
}

tricopterRpmObserverOutput_t tricopterRpmObserverStep(tricopterRpmObserverState_t *state,
    uint32_t nowUs, const tricopterRpmObservation_t *observation)
{
    if (!state || !state->initialized || state->latchedStatus) {
        return output(state, 0, false);
    }
    const uint32_t queryDt = nowUs - state->queryTimeUs;
    if (queryDt >= HALF_CLOCK_RANGE) {
        return fail(state, TRICOPTER_RPM_OBSERVER_INVALID_TIME);
    }
    if (queryDt > state->params.maxCallGapUs) {
        return fail(state, TRICOPTER_RPM_OBSERVER_CALL_GAP);
    }
    state->queryTimeUs = nowUs;
    if (state->seen) {
        state->observationAgeUs = queryDt > UINT32_MAX - state->observationAgeUs
            ? UINT32_MAX : state->observationAgeUs + queryDt;
    }
    if (!observation || !observation->seen) {
        return output(state, 0, false);
    }
    const uint32_t age = nowUs - observation->observedAtUs;
    if (!finiteFloat(observation->mechanicalRpm) || observation->mechanicalRpm < 0.0f
        || observation->mechanicalRpm > state->params.maxRpm || age > state->params.maxPredictionUs) {
        return output(state, TRICOPTER_RPM_OBSERVER_INVALID_OBSERVATION, false);
    }
    uint32_t status = 0;
    if (state->seen) {
        const uint32_t sequenceDt = observation->sequence - state->sequence;
        if (!sequenceDt) {
            if (observation->observedAtUs != state->observationTimeUs || observation->mechanicalRpm != state->observationRpm) {
                return fail(state, TRICOPTER_RPM_OBSERVER_METADATA_MISMATCH);
            }
            return output(state, 0, false);
        }
        if (sequenceDt >= HALF_CLOCK_RANGE) {
            return fail(state, TRICOPTER_RPM_OBSERVER_OUT_OF_ORDER);
        }
        const uint32_t dtUs = observation->observedAtUs - state->observationTimeUs;
        const bool reacquire = state->observationAgeUs > state->params.maxPredictionUs;
        if (!reacquire && dtUs >= HALF_CLOCK_RANGE) {
            return fail(state, TRICOPTER_RPM_OBSERVER_OUT_OF_ORDER);
        }
        if (reacquire || !dtUs) {
            state->estimatedRpm = observation->mechanicalRpm;
            state->estimatedAccelerationRpmS = 0.0f;
            status |= reacquire ? TRICOPTER_RPM_OBSERVER_REACQUIRED : TRICOPTER_RPM_OBSERVER_COALESCED_TIME;
        } else {
            float predictedRpm, predictedAcceleration;
            project(state, dtUs, &predictedRpm, &predictedAcceleration);
            const float dt = (float)dtUs * 0.000001f;
            // Repeated observer error poles exp(-dt/tau), at the actual
            // observation interval. No raw-RPM difference is differentiated.
            const float oneMinusPole = -expm1f(-dt / state->params.trackingTauS);
            const float alpha = oneMinusPole * (2.0f - oneMinusPole);
            const float beta = oneMinusPole * oneMinusPole;
            const float innovation = observation->mechanicalRpm - predictedRpm;
            state->estimatedRpm = clampFloat(predictedRpm + alpha * innovation, 0.0f, state->params.maxRpm);
            state->estimatedAccelerationRpmS = clampFloat(predictedAcceleration + (beta / dt) * innovation,
                -state->params.maxAccelerationRpmS, state->params.maxAccelerationRpmS);
        }
    } else {
        state->estimatedRpm = observation->mechanicalRpm;
        state->estimatedAccelerationRpmS = 0.0f;
    }
    state->seen = true;
    state->observationTimeUs = observation->observedAtUs;
    state->observationAgeUs = age;
    state->sequence = observation->sequence;
    state->observationRpm = observation->mechanicalRpm;
    return output(state, status, true);
}
