/* Synthetic plant + batch transport. Only the portable coordinator receives
 * sensor metadata. Plant truth is used exclusively for packet generation and
 * reporting, with independent plant/predictor states and parameters.
 */
#include "simulation_bridge.h"
#include "flight/tricopter_simulation.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

typedef struct packet_s { uint64_t dueUs; uint32_t sampleUs; uint32_t sequence; float rpm; } packet_t;
typedef struct packetQueue_s { packet_t data[SIMULATION_PACKET_CAPACITY]; unsigned head, count; uint32_t generated; } packetQueue_t;

static bool finiteDouble(double value)
{
    uint64_t bits; memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
}
static bool unsignedValue(double value, double maximum) { return value >= 0 && value <= maximum && value == floor(value); }
static bool finiteVector(const double *values, unsigned count)
{
    for (unsigned i = 0; i < count; ++i) if (!finiteDouble(values[i]) || fabs(values[i]) > FLT_MAX) return false;
    return true;
}
static bool readServo(const double *p, tricopterServoModelParams_t *out)
{
    if (!unsignedValue(p[7], UINT32_MAX) || !unsignedValue(p[12], UINT32_MAX)) return false;
    *out = (tricopterServoModelParams_t) { .knotCount = 3, .backlashUs = (float)p[6], .delayUs = (uint32_t)p[7],
        .tauIncreasingPwmS = (float)p[8], .tauDecreasingPwmS = (float)p[9],
        .rateIncreasingPwmCountsS = (float)p[10], .rateDecreasingPwmCountsS = (float)p[11], .maxGapUs = (uint32_t)p[12] };
    for (unsigned i = 0; i < 3; ++i) { out->pwmUs[i] = (float)p[i]; out->positionCounts[i] = (float)p[3+i]; }
    return tricopterServoModelValidateParams(out);
}
static bool readReaction(const double *p, tricopterReactionTorqueParams_t *out)
{
    if (!unsignedValue(p[12], UINT32_MAX)) return false;
    *out = (tricopterReactionTorqueParams_t) { .coefficientSource = TRICOPTER_REACTION_TORQUE_COEFFICIENTS_SYNTHETIC_PLACEHOLDER,
        .maxMechanicalRpm = (float)p[9], .maxAbsAccelerationRpmS = (float)p[10], .maxAbsTorqueNm = (float)p[11], .maxRpmAgeUs = (uint32_t)p[12] };
    for (unsigned i = 0; i < 3; ++i) {
        if (p[6+i] != -1 && p[6+i] != 1) return false;
        out->kQ[i] = (float)p[i]; out->inertiaKgM2[i] = (float)p[3+i]; out->reactionSign[i] = (int8_t)p[6+i];
    }
    return tricopterReactionTorqueValidateParams(out);
}
static bool readController(const double *p, tricopterSimulationParams_t *out, tricopterSimulationInitialState_t *initial, uint64_t now)
{
    if (!finiteVector(p, SIMULATION_CONTROLLER_PARAMETERS) || !unsignedValue(p[28], UINT16_MAX)
        || !unsignedValue(p[46], UINT32_MAX) || !unsignedValue(p[47], UINT32_MAX) || !unsignedValue(p[48], UINT32_MAX)
        || p[70] != 1 || p[74] != 1 || p[73] < -1 || p[73] > 1 || p[73] != floor(p[73])) return false;
    memset(out, 0, sizeof(*out));
    tricopterAllocatorVirtualParams_t *a = &out->allocator;
    for (unsigned i = 0; i < 16; ++i) a->effectivenessMatrix[i/4][i%4] = (float)p[i];
    for (unsigned i = 0; i < 3; ++i) a->maxEffort[i] = (float)p[16+i];
    a->tailDirectionMinRad = (float)p[19]; a->tailDirectionMaxRad = (float)p[20];
    for (unsigned i = 0; i < 4; ++i) a->responseScale[i] = (float)p[21+i];
    a->lowTailEffort = (float)p[25]; a->projectedGradientTolerance = (float)p[26];
    a->maxConditionNumber = (float)p[27]; a->maxIterations = (uint16_t)p[28]; a->source = TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER;
    if (!readServo(p+29, &out->servo) || !readReaction(p+49, &out->reaction)) return false;
    for (unsigned i = 0; i < 3; ++i) {
        out->rpm[i] = (tricopterRpmObserverParams_t) { .trackingTauS = (float)p[42], .staleAccelerationDecayTauS = (float)p[43],
            .maxRpm = (float)p[44], .maxAccelerationRpmS = (float)p[45], .staleAfterUs = (uint32_t)p[46],
            .maxPredictionUs = (uint32_t)p[47], .maxCallGapUs = (uint32_t)p[48] };
        out->effortPerOmegaSquared[i] = (float)p[62+i];
    }
    out->correctedCountsAtDirectionMin = (float)p[65]; out->correctedCountsAtDirectionMax = (float)p[66];
    out->yawResponsePerNm = (float)p[67]; out->maxAbsReactionCompensationResponse = (float)p[68];
    out->minTailSupportCosine = (float)p[69]; out->effectivenessExcludesReaction = true;
    *initial = (tricopterSimulationInitialState_t) { .timeUs = now, .correctedCounts = (float)p[71], .pwmUs = (float)p[72],
        .approachDirection = (int8_t)p[73], .correctedPositionValid = true };
    return tricopterSimulationValidateParams(out);
}
static void motorAdvance(double *rpm, double target, double dt, double tauInc, double tauDec, double rateInc, double rateDec, double *acceleration)
{
    const double error = target-*rpm, sign = error < 0 ? -1 : 1, distance = fabs(error);
    const double tau = error >= 0 ? tauInc : tauDec, rate = error >= 0 ? rateInc : rateDec;
    double remaining;
    if (rate == 0 || distance <= rate*tau) remaining = distance*exp(-dt/tau);
    else {
        const double linearTime = (distance-rate*tau)/rate;
        remaining = dt <= linearTime ? distance-rate*dt : rate*tau*exp(-(dt-linearTime)/tau);
    }
    *rpm = target-sign*remaining;
    const double speed = rate == 0 ? remaining/tau : fmin(rate, remaining/tau);
    *acceleration = sign*speed;
}

int tricopterSimulationReplayWithPolicy(const double *controller, const double *plant, size_t count,
    const uint64_t *timeUs, const float *requestedResponse, const uint8_t *modes,
    const uint8_t *dropGeneration, const uint8_t *outlierGeneration,
    double *numeric, uint32_t *integer, uint8_t *flags, const simulationReplayPolicy_t *policy)
{
    if (!controller || !plant || !count || count > SIZE_MAX/SIMULATION_NUMERIC_COLUMNS || !timeUs || !requestedResponse
        || !modes || !dropGeneration || !outlierGeneration || !numeric || !integer || !flags
        || !finiteVector(plant, SIMULATION_PLANT_PARAMETERS) || plant[67] != 1 || plant[57] == plant[58]
        || plant[60] <= 0 || !unsignedValue(plant[61], UINT32_MAX) || plant[61] < 1
        || !unsignedValue(plant[62], UINT32_MAX) || plant[62] > (SIMULATION_PACKET_CAPACITY-2)*plant[61]
        || plant[70] < -1 || plant[70] > 1 || plant[70] != floor(plant[70])
        || (policy && (!policy->context || !policy->reset || !policy->tick))) return 0;
    tricopterSimulationParams_t params; tricopterSimulationInitialState_t initial;
    if (!readController(controller, &params, &initial, timeUs[0])) return 0;
    tricopterSimulationState_t coordinator;
    if (policy) {
        if (!policy->reset(policy->context, &params, &initial)) return 0;
    } else if (!tricopterSimulationReset(&coordinator, &params, &initial)) return 0;
    tricopterServoModelParams_t truthServoParams; tricopterServoModelState_t truthServo;
    tricopterReactionTorqueParams_t truthReactionParams;
    if (!readServo(plant+16, &truthServoParams) || !readReaction(plant+44, &truthReactionParams)
        || !tricopterServoModelReset(&truthServo, &truthServoParams, timeUs[0], (float)plant[69], (float)plant[68], (int)plant[70])) return 0;
    double rpm[3], acceleration[3] = {0}, priorTargetRpm[3];
    float priorPwm = (float)plant[69];
    for (unsigned i = 0; i < 3; ++i) {
        if (plant[29+i] < 1e-6 || plant[29+i] > 60 || plant[32+i] < 1e-6 || plant[32+i] > 60
            || plant[35+i] < 0 || plant[35+i] > 1e9 || plant[38+i] < 0 || plant[38+i] > 1e9
            || plant[41+i] <= 0 || !unsignedValue(plant[63+i], UINT32_MAX)
            || plant[63+i] >= plant[61] || plant[71+i] < 0 || plant[71+i] > plant[60]) return 0;
        rpm[i] = priorTargetRpm[i] = plant[71+i];
    }
    const uint64_t grid = count > 1 && timeUs[1] > timeUs[0] ? timeUs[1]-timeUs[0] : 0;
    if (count > 1 && (!grid || (uint64_t)plant[61]%grid || (uint64_t)plant[62]%grid)) return 0;
    for (unsigned motor = 0; motor < 3; ++motor) if (grid && (uint64_t)plant[63+motor]%grid) return 0;
    packetQueue_t queues[3] = {0}; tricopterRpmObservation_t cached[3] = {0}; uint32_t deliveryTime[3] = {0};
    uint64_t previousTime = timeUs[0];
    for (size_t row = 0; row < count; ++row) {
        const uint64_t now = timeUs[row];
        if (now < previousTime || (row && now-previousTime != grid) || now-previousTime > params.servo.maxGapUs) return 0;
        const double dt = (double)(now-previousTime)*1e-6;
        if (row) for (unsigned motor = 0; motor < 3; ++motor)
            motorAdvance(&rpm[motor], priorTargetRpm[motor], dt, plant[29+motor], plant[32+motor], plant[35+motor], plant[38+motor], &acceleration[motor]);
        const tricopterServoModelOutput_t servo = tricopterServoModelStep(&truthServo, now, priorPwm);
        if (!servo.valid) return 0;
        double direction = params.allocator.tailDirectionMinRad
            + (servo.positionCounts-plant[57])/(plant[58]-plant[57])*(params.allocator.tailDirectionMaxRad-params.allocator.tailDirectionMinRad);
        if (policy && policy->plantAngleMap) {
            const tricopterAngleCalibrationOutput_t angle = tricopterAngleCalibrationEvaluate(policy->plantAngleMap, servo.positionCounts);
            if (!angle.valid) return 0;
            direction = angle.angleRad;
        }
        double effort[3]; tricopterReactionTorqueMotorState_t truthMotors[3];
        for (unsigned motor = 0; motor < 3; ++motor) {
            const double omega = rpm[motor]*(3.14159265358979323846/30.0);
            effort[motor] = plant[41+motor]*omega*omega;
            truthMotors[motor] = (tricopterReactionTorqueMotorState_t) { .mechanicalRpm = (float)rpm[motor], .accelerationRpmS = (float)acceleration[motor],
                .ageUs = 0, .seen = true, .fresh = true, .valid = true, .accelerationValid = true };
        }
        const tricopterReactionTorqueOutput_t truthReaction = tricopterReactionTorqueEvaluate(&truthReactionParams, truthMotors);
        if (!truthReaction.valid) return 0;
        const double controls[4] = { effort[1], effort[2], effort[0]*cos(direction), effort[0]*sin(direction) };
        double actual[4] = {0};
        for (unsigned axis = 0; axis < 4; ++axis) for (unsigned col = 0; col < 4; ++col) actual[axis] += plant[4*axis+col]*controls[col];
        const double disturbance = plant[59]*truthReaction.summedTotalNm;
        double *f = numeric+row*SIMULATION_NUMERIC_COLUMNS;
        uint32_t *u = integer+row*SIMULATION_INTEGER_COLUMNS;
        uint8_t *b = flags+row*SIMULATION_FLAG_COLUMNS;
        memset(f, 0, SIMULATION_NUMERIC_COLUMNS*sizeof(*f)); memset(u, 0, SIMULATION_INTEGER_COLUMNS*sizeof(*u)); memset(b, 0, SIMULATION_FLAG_COLUMNS*sizeof(*b));
        const uint64_t elapsed = now-timeUs[0];
        for (unsigned motor = 0; motor < 3; ++motor) {
            packetQueue_t *queue = &queues[motor];
            if (elapsed >= (uint64_t)plant[63+motor] && (elapsed-(uint64_t)plant[63+motor])%(uint64_t)plant[61] == 0) {
                if (dropGeneration[3*row+motor]) b[SIM_PACKET_DROPPED+motor] = 1;
                else {
                    if (queue->count == SIMULATION_PACKET_CAPACITY || now > UINT64_MAX-(uint64_t)plant[62]) return 0;
                    const bool outlier = outlierGeneration[3*row+motor] != 0;
                    const unsigned tail = (queue->head+queue->count)%SIMULATION_PACKET_CAPACITY;
                    queue->data[tail] = (packet_t) { .dueUs = now+(uint64_t)plant[62], .sampleUs = (uint32_t)now,
                        .sequence = ++queue->generated, .rpm = outlier ? (float)plant[66] : (float)rpm[motor] };
                    ++queue->count; b[SIM_PACKET_GENERATED+motor] = 1; b[SIM_PACKET_OUTLIER+motor] = outlier;
                }
            }
            while (queue->count && queue->data[queue->head].dueUs <= now) {
                const packet_t packet = queue->data[queue->head];
                cached[motor] = (tricopterRpmObservation_t) { .seen = true, .observedAtUs = packet.sampleUs,
                    .sequence = packet.sequence, .mechanicalRpm = packet.rpm };
                deliveryTime[motor] = (uint32_t)packet.dueUs;
                queue->head = (queue->head+1)%SIMULATION_PACKET_CAPACITY; --queue->count;
                b[SIM_PACKET_DELIVERED+motor] = 1;
            }
            u[SIM_PACKET_OBSERVED_AT+motor] = cached[motor].observedAtUs;
            u[SIM_PACKET_DELIVERED_AT+motor] = deliveryTime[motor]; u[SIM_PACKET_SEQUENCE+motor] = cached[motor].sequence;
            u[SIM_PACKET_QUEUE_DEPTH+motor] = queue->count;
            b[SIM_PACKET_SEEN+motor] = cached[motor].seen; f[SIM_PACKET_RPM+motor] = cached[motor].mechanicalRpm;
        }
        tricopterSimulationInput_t input = { .modes = modes[row] };
        for (unsigned axis = 0; axis < 4; ++axis) input.requestedResponse[axis] = requestedResponse[4*row+axis];
        for (unsigned motor = 0; motor < 3; ++motor) input.rpm[motor] = cached[motor];
        const tricopterSimulationOutput_t out = policy ? policy->tick(policy->context, now, &input)
            : tricopterSimulationTick(&coordinator, now, &input);
        if (!out.valid) return 0;
        if (out.calibrated || out.forFlight || out.nominalAllocation.calibrated || out.reaction.coefficientsCalibrated || out.servo.angleCalibrated) return -1;
        if (policy && policy->record) policy->record(policy->context, row, &out);
        for (unsigned axis = 0; axis < 4; ++axis) {
            f[SIM_REQUEST+axis] = out.requestedResponse[axis]; f[SIM_ACTUAL_ACTUATOR_RESPONSE+axis] = actual[axis];
            f[SIM_ACTUAL_RESPONSE+axis] = actual[axis]+(axis == 3 ? disturbance : 0);
            f[SIM_PREDICTED_ACTUATOR_RESPONSE+axis] = out.predictedActuatorResponse[axis];
            f[SIM_PREDICTED_TOTAL_RESPONSE+axis] = out.predictedTotalResponse[axis];
            f[SIM_COORDINATED_ACTUATOR_RESPONSE+axis] = out.coordinatedTargetActuatorResponse[axis];
            f[SIM_COORDINATED_TOTAL_RESPONSE+axis] = out.coordinatedTargetTotalResponse[axis];
            f[SIM_POST_COORDINATION_RESIDUAL+axis] = out.postCoordinationResidual[axis];
            f[SIM_ALLOCATION_RESPONSE+axis] = out.nominalAllocation.achievedResponse[axis];
            f[SIM_ALLOCATION_RESIDUAL+axis] = out.nominalAllocation.residualResponse[axis];
            f[SIM_PREDICTED_CONTROLS+axis] = out.predictedControls[axis];
        }
        f[SIM_TRUE_REACTION_NM] = truthReaction.summedTotalNm; f[SIM_TRUE_REACTION_RESPONSE] = disturbance;
        f[SIM_TRUE_SERVO_COUNTS] = servo.positionCounts; f[SIM_TRUE_DIRECTION] = direction; f[SIM_TRUE_SERVO_DERIVATIVE] = servo.velocityCountsS;
        f[SIM_COMMAND_PWM] = out.pwmCommandUs; f[SIM_TARGET_DIRECTION] = out.targetDirectionRad; f[SIM_PREDICTED_DIRECTION] = out.predictedDirectionRad;
        f[SIM_PREDICTED_SERVO_COUNTS] = out.servo.positionCounts; f[SIM_PREDICTED_SERVO_DERIVATIVE] = out.servo.velocityCountsS;
        f[SIM_ESTIMATED_REACTION_NM] = out.reaction.summedTotalNm; f[SIM_APPLIED_REACTION_RESPONSE] = out.appliedReactionCompensationResponse;
        f[SIM_CONTROLLER_PREDICTED_REACTION_RESPONSE] = params.yawResponsePerNm*out.reaction.summedTotalNm;
        f[SIM_ALLOCATION_COST] = out.nominalAllocation.cost; f[SIM_ALLOCATION_GRADIENT_NORM] = out.nominalAllocation.projectedGradientNorm;
        f[SIM_TRUE_TAIL_VERTICAL] = controls[2]; f[SIM_TRUE_TAIL_RIGHTWARD] = controls[3];
        for (unsigned motor = 0; motor < 3; ++motor) {
            f[SIM_TRUE_RPM+motor] = rpm[motor]; f[SIM_TRUE_ACCELERATION+motor] = acceleration[motor]; f[SIM_TRUE_EFFORT+motor] = effort[motor];
            f[SIM_COMMAND_RPM+motor] = out.commandedMechanicalRpm[motor]; f[SIM_COMMAND_EFFORT+motor] = out.commandedEffort[motor];
            f[SIM_OBSERVER_RPM+motor] = out.rpm[motor].mechanicalRpm; f[SIM_OBSERVER_ACCELERATION+motor] = out.rpm[motor].accelerationRpmS;
            u[SIM_OBSERVER_STATUS+motor] = out.rpm[motor].status; u[SIM_OBSERVER_AGE+motor] = out.rpm[motor].ageUs; u[SIM_OBSERVER_SEQUENCE+motor] = out.rpm[motor].sequence;
            b[SIM_ACCELERATION_VALID+motor] = out.accelerationValid[motor]; b[SIM_RPM_SEEN+motor] = out.rpm[motor].seen;
            b[SIM_RPM_ACCEPTED+motor] = out.rpm[motor].acceptedNewObservation; b[SIM_RPM_FRESH+motor] = out.rpm[motor].fresh; b[SIM_RPM_VALID+motor] = out.rpm[motor].valid;
            // The only plant command update occurs AFTER this row's truth and
            // coordinator predictions have been captured.
            b[SIM_PLANT_RPM_CLIPPED+motor] = out.commandedMechanicalRpm[motor] > plant[60];
            priorTargetRpm[motor] = fmin(out.commandedMechanicalRpm[motor], plant[60]);
        }
        u[SIM_CONTROLLER_STATUS] = out.status; u[SIM_ALLOCATOR_STATUS] = out.nominalAllocation.status;
        u[SIM_ALLOCATOR_CONSTRAINTS] = out.nominalAllocation.constraints; u[SIM_ALLOCATOR_ITERATIONS] = out.nominalAllocation.iterations;
        u[SIM_TRUE_SERVO_STATUS] = servo.status; u[SIM_REACTION_STATUS] = out.reaction.status;
        b[SIM_CONTROLLER_VALID] = out.valid; b[SIM_ALLOCATOR_VALID] = out.nominalAllocation.valid;
        b[SIM_ALLOCATOR_CONVERGED] = out.nominalAllocation.converged; b[SIM_ALLOCATOR_EXACT] = out.nominalAllocation.exact;
        b[SIM_SERVO_PREDICTION_AVAILABLE] = out.servoPredictionAvailable; b[SIM_REACTION_AVAILABLE] = out.reactionAvailable;
        b[SIM_COORDINATED_ACTUATOR_AVAILABLE] = out.coordinatedActuatorResponseAvailable; b[SIM_COORDINATED_TOTAL_AVAILABLE] = out.coordinatedTotalResponseAvailable;
        b[SIM_PREDICTED_ACTUATOR_AVAILABLE] = out.predictedActuatorResponseAvailable; b[SIM_PREDICTED_TOTAL_AVAILABLE] = out.predictedTotalResponseAvailable;
        b[SIM_TRUE_SERVO_VALID] = servo.valid; b[SIM_CALIBRATED] = out.calibrated; b[SIM_FOR_FLIGHT] = out.forFlight; b[SIM_TRUE_REACTION_VALID] = truthReaction.valid;
        priorPwm = out.pwmCommandUs;
        if (!tricopterServoModelStep(&truthServo, now, priorPwm).valid) return 0;
        for (unsigned col = 0; col < SIMULATION_NUMERIC_COLUMNS; ++col) if (!finiteDouble(f[col])) return 0;
        previousTime = now;
    }
    return 1;
}

int tricopterSimulationReplay(const double *controller, const double *plant, size_t count,
    const uint64_t *timeUs, const float *requestedResponse, const uint8_t *modes,
    const uint8_t *dropGeneration, const uint8_t *outlierGeneration,
    double *numeric, uint32_t *integer, uint8_t *flags)
{
    return tricopterSimulationReplayWithPolicy(controller, plant, count, timeUs, requestedResponse,
        modes, dropGeneration, outlierGeneration, numeric, integer, flags, NULL);
}
