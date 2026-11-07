/* Offline causal composition of the four pure actuator/response modules. */
#include "tricopter_simulation.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128 || DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024
#error "Tricopter simulation requires binary32 float and binary64 double"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "Simulation requires binary32 float");
_Static_assert(sizeof(double) == sizeof(uint64_t), "Simulation requires binary64 double");

static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static double clamp(double value, double minimum, double maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool tricopterSimulationValidateParams(const tricopterSimulationParams_t *p)
{
    if (!p || !p->effectivenessExcludesReaction
        || !tricopterAllocatorVirtualValidateParams(&p->allocator)
        || !tricopterServoModelValidateParams(&p->servo)
        || !tricopterReactionTorqueValidateParams(&p->reaction)
        || p->reaction.maxAbsTorqueNm > 1e6f
        || p->allocator.tailDirectionMinRad >= p->allocator.tailDirectionMaxRad
        || !finiteFloat(p->correctedCountsAtDirectionMin) || !finiteFloat(p->correctedCountsAtDirectionMax)
        || fabsf(p->correctedCountsAtDirectionMin) > 4095 || fabsf(p->correctedCountsAtDirectionMax) > 4095
        || fabsf(p->correctedCountsAtDirectionMax - p->correctedCountsAtDirectionMin) < 1
        || !finiteFloat(p->yawResponsePerNm) || fabsf(p->yawResponsePerNm) > 1e6f
        || !finiteFloat(p->maxAbsReactionCompensationResponse) || p->maxAbsReactionCompensationResponse <= 0 || p->maxAbsReactionCompensationResponse > 1e8f
        || !finiteFloat(p->minTailSupportCosine) || p->minTailSupportCosine < .001f || p->minTailSupportCosine > 1) {
        return false;
    }
    const double first = p->servo.positionCounts[0];
    const double last = p->servo.positionCounts[p->servo.knotCount - 1];
    const double lower = fmin(first, last), upper = fmax(first, last);
    if (p->correctedCountsAtDirectionMin < lower || p->correctedCountsAtDirectionMin > upper
        || p->correctedCountsAtDirectionMax < lower || p->correctedCountsAtDirectionMax > upper) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!tricopterRpmObserverValidateParams(&p->rpm[i])
            || !finiteFloat(p->effortPerOmegaSquared[i]) || p->effortPerOmegaSquared[i] < 1e-12f || p->effortPerOmegaSquared[i] > 1e3f) return false;
        const double maxRpm = sqrt((double)p->allocator.maxEffort[i] / p->effortPerOmegaSquared[i]) * (30.0 / 3.14159265358979323846);
        if (maxRpm > p->rpm[i].maxRpm || maxRpm > p->reaction.maxMechanicalRpm) return false;
    }
    return true;
}

static bool directionFromCounts(const tricopterSimulationParams_t *p, float counts, float *direction)
{
    if (!finiteFloat(counts)) return false;
    const double fraction = ((double)counts - p->correctedCountsAtDirectionMin)
        / ((double)p->correctedCountsAtDirectionMax - p->correctedCountsAtDirectionMin);
    if (fraction < 0 || fraction > 1) return false;
    *direction = (float)(p->allocator.tailDirectionMinRad
        + fraction * ((double)p->allocator.tailDirectionMaxRad - p->allocator.tailDirectionMinRad));
    return true;
}

static float pwmFromDirection(const tricopterSimulationParams_t *p, float direction)
{
    const double fraction = ((double)direction - p->allocator.tailDirectionMinRad)
        / ((double)p->allocator.tailDirectionMaxRad - p->allocator.tailDirectionMinRad);
    const double counts = p->correctedCountsAtDirectionMin
        + fraction * ((double)p->correctedCountsAtDirectionMax - p->correctedCountsAtDirectionMin);
    for (unsigned i = 1; i < p->servo.knotCount; ++i) {
        const double before = p->servo.positionCounts[i - 1], after = p->servo.positionCounts[i];
        if (counts >= fmin(before, after) && counts <= fmax(before, after)) {
            const double ratio = after == before ? 0 : (counts - before) / (after - before);
            return (float)(p->servo.pwmUs[i - 1] + ratio * ((double)p->servo.pwmUs[i] - p->servo.pwmUs[i - 1]));
        }
    }
    // Endpoints and monotonic map coverage were validated. Rounding of a
    // bounded direction can only select the final endpoint here.
    return p->servo.pwmUs[p->servo.knotCount - 1];
}

bool tricopterSimulationReset(tricopterSimulationState_t *state,
    const tricopterSimulationParams_t *params, const tricopterSimulationInitialState_t *initial)
{
    if (!state) return false;
    const bool validParams = tricopterSimulationValidateParams(params);
    tricopterSimulationParams_t copy = {0};
    tricopterSimulationInitialState_t start = {0};
    if (validParams) copy = *params;
    if (initial) start = *initial;
    memset(state, 0, sizeof(*state));
    if (!validParams) { state->latchedStatus = TRICOPTER_SIMULATION_INVALID_CONFIG; return false; }
    float direction;
    if (!initial || !start.correctedPositionValid || !directionFromCounts(&copy, start.correctedCounts, &direction)
        || start.timeUs > UINT64_MAX - copy.servo.delayUs) {
        state->latchedStatus = TRICOPTER_SIMULATION_INVALID_INITIAL_STATE; return false;
    }
    if (!tricopterServoModelReset(&state->servo, &copy.servo, start.timeUs, start.pwmUs, start.correctedCounts, start.approachDirection)) {
        state->latchedStatus = TRICOPTER_SIMULATION_INVALID_INITIAL_STATE; return false;
    }
    for (unsigned i = 0; i < 3; ++i) tricopterRpmObserverReset(&state->rpm[i], &copy.rpm[i], (uint32_t)start.timeUs);
    state->params = copy;
    state->timeUs = start.timeUs;
    state->previousPwmUs = start.pwmUs;
    state->initialized = true;
    return true;
}

static tricopterSimulationOutput_t fail(tricopterSimulationState_t *state, uint32_t status)
{
    if (state) state->latchedStatus |= status;
    return (tricopterSimulationOutput_t) { .status = status | (state ? state->latchedStatus : 0) };
}

static void updateRpm(tricopterSimulationState_t *state, uint32_t now, const tricopterSimulationInput_t *input,
    tricopterSimulationOutput_t *out)
{
    tricopterReactionTorqueMotorState_t reactionInput[3];
    for (unsigned i = 0; i < 3; ++i) {
        const tricopterRpmObserverState_t *prior = &state->rpm[i];
        const uint64_t oldAgeAtNow = (uint64_t)prior->observationAgeUs + (uint32_t)(now - prior->queryTimeUs);
        if (!prior->seen || oldAgeAtNow > state->params.rpm[i].staleAfterUs) state->freshDistinctObservations[i] = 0;
        out->rpm[i] = tricopterRpmObserverStep(&state->rpm[i], now, &input->rpm[i]);
        const tricopterRpmObserverOutput_t *sample = &out->rpm[i];
        if (!sample->valid || !sample->fresh) {
            state->freshDistinctObservations[i] = 0;
            out->status |= TRICOPTER_SIMULATION_RPM_UNAVAILABLE;
        } else if (sample->acceptedNewObservation) {
            if (sample->status & (TRICOPTER_RPM_OBSERVER_REACQUIRED | TRICOPTER_RPM_OBSERVER_COALESCED_TIME)) {
                state->freshDistinctObservations[i] = 1;
            } else if (!state->freshDistinctObservations[i]) {
                state->freshDistinctObservations[i] = 1;
            } else if (input->rpm[i].observedAtUs != state->qualifiedObservationTimeUs[i]) {
                state->freshDistinctObservations[i] = 2;
            }
            state->qualifiedObservationTimeUs[i] = input->rpm[i].observedAtUs;
        }
        out->accelerationValid[i] = sample->valid && sample->fresh && state->freshDistinctObservations[i] >= 2;
        if (!out->accelerationValid[i]) out->status |= TRICOPTER_SIMULATION_ACCELERATION_UNAVAILABLE;
        reactionInput[i] = (tricopterReactionTorqueMotorState_t) {
            .mechanicalRpm = sample->mechanicalRpm, .accelerationRpmS = sample->accelerationRpmS,
            .ageUs = sample->ageUs, .seen = sample->seen, .fresh = sample->fresh, .valid = sample->valid,
            .accelerationValid = out->accelerationValid[i],
        };
    }
    // This existing scalar equation is deliberately used as an additive
    // synthetic yaw disturbance independent of virtual tail direction.
    out->reaction = tricopterReactionTorqueEvaluate(&state->params.reaction, reactionInput);
    out->reactionAvailable = out->reaction.valid;
    if (!out->reactionAvailable) out->status |= TRICOPTER_SIMULATION_REACTION_UNAVAILABLE;
}

static void responseDiagnostics(const tricopterSimulationParams_t *p, tricopterSimulationOutput_t *out)
{
    if (!out->servoPredictionAvailable) {
        out->status |= TRICOPTER_SIMULATION_COORDINATED_RESPONSE_UNAVAILABLE | TRICOPTER_SIMULATION_PREDICTED_RESPONSE_UNAVAILABLE;
        return;
    }
    const tricopterAllocatorVirtualOutput_t coordinated = tricopterAllocatorVirtualForward(&p->allocator,
        out->commandedEffort, out->predictedDirectionRad);
    if (coordinated.valid) {
        out->coordinatedActuatorResponseAvailable = true;
        memcpy(out->coordinatedTargetControls, coordinated.controls, sizeof(out->coordinatedTargetControls));
        memcpy(out->coordinatedTargetActuatorResponse, coordinated.achievedResponse, sizeof(out->coordinatedTargetActuatorResponse));
        if (out->reactionAvailable) {
            out->coordinatedTotalResponseAvailable = true;
            for (unsigned axis = 0; axis < 4; ++axis) {
                const double total = coordinated.achievedResponse[axis]
                    + (axis == 3 ? (double)p->yawResponsePerNm * out->reaction.summedTotalNm : 0);
                out->coordinatedTargetTotalResponse[axis] = (float)total;
                out->postCoordinationResidual[axis] = (float)(total - out->requestedResponse[axis]);
            }
        }
    }
    if (!out->coordinatedTotalResponseAvailable) out->status |= TRICOPTER_SIMULATION_COORDINATED_RESPONSE_UNAVAILABLE;
    float estimatedEffort[3];
    for (unsigned i = 0; i < 3; ++i) {
        if (!out->rpm[i].valid || !out->rpm[i].fresh) { out->status |= TRICOPTER_SIMULATION_PREDICTED_RESPONSE_UNAVAILABLE; return; }
        const double omega = out->rpm[i].mechanicalRpm * (3.14159265358979323846 / 30.0);
        const double effort = p->effortPerOmegaSquared[i] * omega * omega;
        // Out-of-bounds estimates stay unavailable; no clipped state is
        // presented as a measured/predicted achievable response.
        if (effort < 0 || effort > p->allocator.maxEffort[i]) { out->status |= TRICOPTER_SIMULATION_PREDICTED_RESPONSE_UNAVAILABLE; return; }
        estimatedEffort[i] = (float)effort;
    }
    const tricopterAllocatorVirtualOutput_t predicted = tricopterAllocatorVirtualForward(&p->allocator, estimatedEffort, out->predictedDirectionRad);
    if (predicted.valid) {
        out->predictedActuatorResponseAvailable = true;
        memcpy(out->predictedControls, predicted.controls, sizeof(out->predictedControls));
        memcpy(out->predictedActuatorResponse, predicted.achievedResponse, sizeof(out->predictedActuatorResponse));
        if (out->reactionAvailable) {
            out->predictedTotalResponseAvailable = true;
            for (unsigned axis = 0; axis < 4; ++axis) out->predictedTotalResponse[axis] = (float)(predicted.achievedResponse[axis]
                + (axis == 3 ? (double)p->yawResponsePerNm * out->reaction.summedTotalNm : 0));
        }
    }
    if (!out->predictedTotalResponseAvailable) out->status |= TRICOPTER_SIMULATION_PREDICTED_RESPONSE_UNAVAILABLE;
}

tricopterSimulationOutput_t tricopterSimulationTick(tricopterSimulationState_t *state,
    uint64_t nowUs, const tricopterSimulationInput_t *input)
{
    if (!state || !state->initialized) return fail(state, TRICOPTER_SIMULATION_NOT_INITIALIZED);
    if (state->latchedStatus) return fail(state, state->latchedStatus);
    const tricopterSimulationParams_t *p = &state->params;
    if (!input || (input->modes & ~(TRICOPTER_SIMULATION_REACTION_COMPENSATION | TRICOPTER_SIMULATION_TAIL_VERTICAL_COORDINATION))) return fail(state, TRICOPTER_SIMULATION_INVALID_INPUT);
    for (unsigned i = 0; i < 4; ++i) {
        if (!finiteFloat(input->requestedResponse[i]) || fabsf(input->requestedResponse[i]) > 1e8f) return fail(state, TRICOPTER_SIMULATION_INVALID_INPUT);
    }
    if (nowUs < state->timeUs || nowUs > UINT64_MAX - p->servo.delayUs || nowUs - state->timeUs > p->servo.maxGapUs) return fail(state, TRICOPTER_SIMULATION_INVALID_TIME);
    for (unsigned i = 0; i < 3; ++i) if (nowUs - state->timeUs > p->rpm[i].maxCallGapUs) return fail(state, TRICOPTER_SIMULATION_INVALID_TIME);
    state->timeUs = nowUs;
    tricopterSimulationOutput_t out = {0};
    memcpy(out.requestedResponse, input->requestedResponse, sizeof(out.requestedResponse));
    memcpy(out.netRequestedResponse, input->requestedResponse, sizeof(out.netRequestedResponse));
    // Capture CURRENT prediction before queuing any newly selected command.
    out.servo = tricopterServoModelStep(&state->servo, nowUs, state->previousPwmUs);
    out.servoPredictionAvailable = out.servo.valid && directionFromCounts(p, out.servo.positionCounts, &out.predictedDirectionRad);
    if (!out.servoPredictionAvailable) out.status |= TRICOPTER_SIMULATION_SERVO_PREDICTION_UNAVAILABLE;
    updateRpm(state, (uint32_t)nowUs, input, &out);
    if ((input->modes & TRICOPTER_SIMULATION_REACTION_COMPENSATION) && out.reactionAvailable) {
        const double estimated = (double)p->yawResponsePerNm * out.reaction.summedTotalNm;
        const double limited = clamp(estimated, -p->maxAbsReactionCompensationResponse, p->maxAbsReactionCompensationResponse);
        if (limited != estimated) out.status |= TRICOPTER_SIMULATION_REACTION_COMPENSATION_CLIPPED;
        const double net = input->requestedResponse[3] - limited;
        const double bounded = clamp(net, -1e8, 1e8);
        if (bounded != net) out.status |= TRICOPTER_SIMULATION_NET_REQUEST_CLIPPED;
        out.netRequestedResponse[3] = (float)bounded;
        // Float32 publication can enlarge a small bounded correction (e.g.
        // 1e8 - 5 rounds by 8). Move one adjacent value toward the original
        // request if needed so the ACTUAL published subtraction stays bounded.
        if (fabs((double)input->requestedResponse[3] - out.netRequestedResponse[3]) > p->maxAbsReactionCompensationResponse) {
            out.netRequestedResponse[3] = nextafterf(out.netRequestedResponse[3], input->requestedResponse[3]);
            out.status |= TRICOPTER_SIMULATION_REACTION_COMPENSATION_CLIPPED;
        }
        out.appliedReactionCompensationResponse = (float)((double)input->requestedResponse[3] - out.netRequestedResponse[3]);
    }
    tricopterAllocatorVirtualRequest_t request;
    memcpy(request.response, out.netRequestedResponse, sizeof(request.response));
    out.nominalAllocation = tricopterAllocatorVirtualAllocate(&p->allocator, &request);
    if (!out.nominalAllocation.valid) return fail(state, TRICOPTER_SIMULATION_INVALID_INPUT);
    if (!out.nominalAllocation.converged) out.status |= TRICOPTER_SIMULATION_ALLOCATOR_NOT_CONVERGED;
    out.targetDirectionRad = out.nominalAllocation.tailDirectionRad;
    out.pwmCommandUs = pwmFromDirection(p, out.targetDirectionRad);
    memcpy(out.commandedEffort, out.nominalAllocation.effort, sizeof(out.commandedEffort));
    if ((input->modes & TRICOPTER_SIMULATION_TAIL_VERTICAL_COORDINATION) && out.servoPredictionAvailable) {
        const double cosine = cos(out.predictedDirectionRad);
        if (cosine < p->minTailSupportCosine) {
            out.status |= TRICOPTER_SIMULATION_TAIL_COSINE_GUARD;
        } else {
            const double effort = out.nominalAllocation.controls[2] / cosine;
            const double limited = clamp(effort, 0, p->allocator.maxEffort[0]);
            if (limited != effort) out.status |= TRICOPTER_SIMULATION_TAIL_EFFORT_CLIPPED;
            out.commandedEffort[0] = (float)limited;
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        const double omega = sqrt((double)out.commandedEffort[i] / p->effortPerOmegaSquared[i]);
        out.commandedOmegaRadS[i] = (float)omega;
        out.commandedMechanicalRpm[i] = (float)(omega * (30.0 / 3.14159265358979323846));
    }
    // Queue the new command at now, after the captured prediction. It affects
    // the following interval, never observations/prediction already captured.
    const tricopterServoModelOutput_t queued = tricopterServoModelStep(&state->servo, nowUs, out.pwmCommandUs);
    if (!queued.valid) { out.status |= TRICOPTER_SIMULATION_SERVO_PREDICTION_UNAVAILABLE; out.servoPredictionAvailable = false; }
    state->previousPwmUs = out.pwmCommandUs;
    responseDiagnostics(p, &out);
    out.valid = true;
    return out;
}
