/* Offline finite-horizon allocation, bounded sampled servo candidates. */
#include "tricopter_dynamic_allocator.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128 || DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024
#error "Dynamic allocation requires binary32 float and binary64 double"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "Dynamic allocation requires binary32 float");
_Static_assert(sizeof(double) == sizeof(uint64_t), "Dynamic allocation requires binary64 double");

static bool finiteFloat(float value)
{
    uint32_t bits; memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}
static double clamp(double value, double lower, double upper) { return value < lower ? lower : (value > upper ? upper : value); }
static bool tauValid(float value) { return finiteFloat(value) && value >= 1e-6f && value <= 60; }
static bool rateValid(float value) { return finiteFloat(value) && value >= 0 && value <= 1e9f; }
static bool toleranceValid(float value) { return finiteFloat(value) && value >= 0 && value <= 1e-6f; }

static double inverseCounts(const tricopterServoModelParams_t *s, double counts)
{
    for (unsigned i = 1; i < s->knotCount; ++i) {
        const double a = s->positionCounts[i - 1], b = s->positionCounts[i];
        if (counts >= fmin(a,b) && counts <= fmax(a,b)) {
            const double ratio = a == b ? 0 : (counts - a) / (b - a);
            return s->pwmUs[i - 1] + ratio * ((double)s->pwmUs[i] - s->pwmUs[i - 1]);
        }
    }
    return s->pwmUs[s->knotCount - 1];
}
static bool pwmBounds(const tricopterDynamicAllocatorModel_t *m, const tricopterServoModelParams_t *s, float *lower, float *upper)
{
    const double a = inverseCounts(s, m->correctedCountsAtDirectionMin), b = inverseCounts(s, m->correctedCountsAtDirectionMax);
    const double lo = fmin(a,b), hi = fmax(a,b);
    *lower = (float)lo; *upper = (float)hi;
    if (*lower < lo) *lower = nextafterf(*lower, INFINITY);
    if (*upper > hi) *upper = nextafterf(*upper, 0);
    return *upper > *lower;
}

bool tricopterDynamicAllocatorValidateParams(const tricopterDynamicAllocatorModel_t *m,
    const tricopterDynamicAllocatorParams_t *p, const tricopterServoModelParams_t *s)
{
    if (!m || !p || !s || !tricopterAllocatorVirtualValidateParams(&m->allocator) || !tricopterServoModelValidateParams(s)
        || !finiteFloat(m->correctedCountsAtDirectionMin) || !finiteFloat(m->correctedCountsAtDirectionMax)
        || fabsf(m->correctedCountsAtDirectionMin) > 4095 || fabsf(m->correctedCountsAtDirectionMax) > 4095
        || fabsf(m->correctedCountsAtDirectionMax - m->correctedCountsAtDirectionMin) < 1
        || m->allocator.tailDirectionMinRad >= m->allocator.tailDirectionMaxRad
        || !p->horizonUs || p->horizonUs > 1000000 || p->horizonUs <= s->delayUs
        || ((uint64_t)p->horizonUs + s->maxGapUs - 1) / s->maxGapUs > TRICOPTER_DYNAMIC_ALLOCATOR_MAX_SERVO_STEPS
        || !p->maxRpmAgeUs || p->maxRpmAgeUs >= UINT32_C(0x80000000)
        || p->gridCount > TRICOPTER_DYNAMIC_ALLOCATOR_MAX_GRID
        || !finiteFloat(p->maxHighMatrixCondition) || p->maxHighMatrixCondition < 1 || p->maxHighMatrixCondition > 1e8f
        || !toleranceValid(p->highCostTolerance) || !toleranceValid(p->yawCostTolerance) || !toleranceValid(p->movementCostTolerance)) return false;
    const double first = s->positionCounts[0], last = s->positionCounts[s->knotCount - 1];
    const double lo = fmin(first,last), hi = fmax(first,last);
    if (m->correctedCountsAtDirectionMin < lo || m->correctedCountsAtDirectionMin > hi
        || m->correctedCountsAtDirectionMax < lo || m->correctedCountsAtDirectionMax > hi) return false;
    float pwmLo, pwmHi;
    if (!pwmBounds(m,s,&pwmLo,&pwmHi)) return false;
    const double rounding = 4 * FLT_EPSILON * fmax(1, pwmHi);
    for (unsigned i = 0; i < p->gridCount; ++i) {
        if (!finiteFloat(p->pwmGridUs[i]) || p->pwmGridUs[i] < pwmLo - rounding || p->pwmGridUs[i] > pwmHi + rounding) return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!tauValid(p->motorTauIncreasingS[i]) || !tauValid(p->motorTauDecreasingS[i])
            || !rateValid(p->motorSlewIncreasingRpmS[i]) || !rateValid(p->motorSlewDecreasingRpmS[i])
            || !finiteFloat(m->effortPerOmegaSquared[i]) || m->effortPerOmegaSquared[i] < 1e-12f || m->effortPerOmegaSquared[i] > 1e3f
            || sqrt((double)m->allocator.maxEffort[i] / m->effortPerOmegaSquared[i]) * (30.0 / 3.14159265358979323846) > 1e7) return false;
    }
    return true;
}

static double motorAt(const tricopterDynamicAllocatorParams_t *p, unsigned i, double current, double command)
{
    const double error = command - current, distance = fabs(error), sign = error < 0 ? -1 : 1;
    const double tau = error < 0 ? p->motorTauDecreasingS[i] : p->motorTauIncreasingS[i];
    const double rate = error < 0 ? p->motorSlewDecreasingRpmS[i] : p->motorSlewIncreasingRpmS[i];
    const double dt = p->horizonUs * 1e-6;
    double remaining;
    if (!rate || distance <= rate * tau) remaining = distance * exp(-dt / tau);
    else {
        const double linearTime = (distance - rate * tau) / rate;
        remaining = dt <= linearTime ? distance - rate * dt : rate * tau * exp(-(dt - linearTime) / tau);
    }
    return current + sign * (distance - remaining);
}
static double effortAt(const tricopterDynamicAllocatorModel_t *m, unsigned i, double rpm)
{
    const double omega = rpm * (3.14159265358979323846 / 30.0);
    return m->effortPerOmegaSquared[i] * omega * omega;
}
static float commandFor(const tricopterDynamicAllocatorModel_t *m, const tricopterDynamicAllocatorParams_t *p,
    unsigned i, double current, double effort, float maximum)
{
    const double target = sqrt(fmax(0, effort) / m->effortPerOmegaSquared[i]) * (30.0 / 3.14159265358979323846);
    double lo = 0, hi = maximum;
    if (target <= motorAt(p,i,current,lo)) return 0;
    if (target >= motorAt(p,i,current,hi)) return maximum;
    for (unsigned iteration = 0; iteration < 64; ++iteration) {
        const double middle = .5 * (lo + hi);
        if (middle == lo || middle == hi) break;
        if (motorAt(p,i,current,middle) < target) lo = middle; else hi = middle;
    }
    float published = (float)clamp(lo,0,maximum);
    if (published > lo) published = nextafterf(published,0);
    return published;
}

static bool highCondition(const double a[3][3], double *condition)
{
    double aug[3][6], norm = 0;
    for (unsigned i = 0; i < 3; ++i) {
        double row = 0;
        for (unsigned j = 0; j < 3; ++j) { aug[i][j] = a[i][j]; aug[i][j + 3] = i == j; row += fabs(a[i][j]); }
        norm = fmax(norm,row);
    }
    for (unsigned col = 0; col < 3; ++col) {
        unsigned pivot = col;
        for (unsigned i = col + 1; i < 3; ++i) if (fabs(aug[i][col]) > fabs(aug[pivot][col])) pivot = i;
        if (fabs(aug[pivot][col]) <= norm * 1e-14) return false;
        if (pivot != col) for (unsigned j = 0; j < 6; ++j) { const double tmp = aug[col][j]; aug[col][j] = aug[pivot][j]; aug[pivot][j] = tmp; }
        const double divisor = aug[col][col];
        for (unsigned j = 0; j < 6; ++j) aug[col][j] /= divisor;
        for (unsigned i = 0; i < 3; ++i) if (i != col) {
            const double factor = aug[i][col];
            for (unsigned j = 0; j < 6; ++j) aug[i][j] -= factor * aug[col][j];
        }
    }
    double inverseNorm = 0;
    for (unsigned i = 0; i < 3; ++i) { double row = 0; for (unsigned j = 3; j < 6; ++j) row += fabs(aug[i][j]); inverseNorm = fmax(inverseNorm,row); }
    *condition = norm * inverseNorm;
    return true;
}

// Small least-squares solve with reorthogonalized QR, avoiding A' A condition
// squaring. Full-rank high3 guard makes every free-column subset independent.
static bool solveFace(const double a[3][3], const double target[3], const double lower[3], const double upper[3],
    unsigned face, double x[3])
{
    unsigned freeIndex[3], freeCount = 0;
    double residual[3] = {target[0], target[1], target[2]};
    for (unsigned j = 0; j < 3; ++j) {
        const unsigned kind = face % 3; face /= 3;
        if (kind == 1) { freeIndex[freeCount++] = j; x[j] = 0; }
        else { x[j] = kind == 0 ? lower[j] : upper[j]; for (unsigned i = 0; i < 3; ++i) residual[i] -= a[i][j] * x[j]; }
    }
    double q[3][3] = {{0}}, r[3][3] = {{0}}, y[3] = {0};
    for (unsigned j = 0; j < freeCount; ++j) {
        double v[3]; for (unsigned i = 0; i < 3; ++i) v[i] = a[i][freeIndex[j]];
        for (unsigned pass = 0; pass < 2; ++pass) for (unsigned k = 0; k < j; ++k) {
            double product = 0; for (unsigned i = 0; i < 3; ++i) product += q[i][k] * v[i];
            r[k][j] += product; for (unsigned i = 0; i < 3; ++i) v[i] -= product * q[i][k];
        }
        r[j][j] = hypot(hypot(v[0],v[1]),v[2]);
        if (r[j][j] <= 1e-30) return false;
        for (unsigned i = 0; i < 3; ++i) { q[i][j] = v[i] / r[j][j]; y[j] += q[i][j] * residual[i]; }
    }
    for (int j = (int)freeCount - 1; j >= 0; --j) {
        double value = y[j]; for (unsigned k = (unsigned)j + 1; k < freeCount; ++k) value -= r[j][k] * x[freeIndex[k]];
        x[freeIndex[j]] = value / r[j][j];
    }
    for (unsigned j = 0; j < 3; ++j) {
        const double tolerance = 1e-10 * fmax(1,upper[j]);
        if (x[j] < lower[j] - tolerance || x[j] > upper[j] + tolerance) return false;
        x[j] = clamp(x[j],lower[j],upper[j]);
    }
    return true;
}

static bool servoStateValid(const tricopterServoModelState_t *s, uint64_t now)
{
    if (!s || !s->initialized || s->status || s->timeUs != now || s->commandCount > TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY
        || s->commandHead >= TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY || !finiteFloat(s->positionCounts)
        || fabsf(s->positionCounts) > 4095 || !finiteFloat(s->lastCommandPwmUs)
        || s->lastCommandPwmUs < s->params.pwmUs[0] || s->lastCommandPwmUs > s->params.pwmUs[s->params.knotCount - 1]
        || !finiteFloat(s->playedPwmUs) || s->playedPwmUs < s->params.pwmUs[0] || s->playedPwmUs > s->params.pwmUs[s->params.knotCount - 1]
        || !finiteFloat(s->targetCounts) || fabsf(s->targetCounts) > 4095 || !finiteFloat(s->velocityCountsS)) return false;
    uint64_t previous = now;
    for (unsigned i = 0; i < s->commandCount; ++i) {
        const tricopterServoModelCommand_t *c = &s->commands[(s->commandHead + i) % TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY];
        if (c->dueUs < previous || c->dueUs - now > s->params.delayUs || !finiteFloat(c->pwmUs)
            || c->pwmUs < s->params.pwmUs[0] || c->pwmUs > s->params.pwmUs[s->params.knotCount - 1]) return false;
        previous = c->dueUs;
    }
    return true;
}

static bool predictDirection(const tricopterDynamicAllocatorModel_t *m, const tricopterDynamicAllocatorParams_t *p,
    const tricopterServoModelState_t *current, uint64_t now, float pwm, float *counts, float *direction)
{
    tricopterServoModelState_t copy = *current;
    tricopterServoModelOutput_t out = tricopterServoModelStep(&copy, now, pwm);
    const uint64_t end = now + p->horizonUs;
    while (out.valid && copy.timeUs < end) {
        const uint64_t step = end - copy.timeUs < copy.params.maxGapUs ? end - copy.timeUs : copy.params.maxGapUs;
        out = tricopterServoModelStep(&copy, copy.timeUs + step, pwm);
    }
    if (!out.valid) return false;
    const double fraction = ((double)out.positionCounts - m->correctedCountsAtDirectionMin)
        / ((double)m->correctedCountsAtDirectionMax - m->correctedCountsAtDirectionMin);
    if (fraction < 0 || fraction > 1) return false;
    *counts = out.positionCounts;
    *direction = (float)(m->allocator.tailDirectionMinRad + fraction * ((double)m->allocator.tailDirectionMaxRad - m->allocator.tailDirectionMinRad));
    return true;
}

static void evaluateCandidate(const tricopterDynamicAllocatorModel_t *m, const tricopterDynamicAllocatorParams_t *p,
    const tricopterDynamicAllocatorInput_t *input, const double lower[3], const double upper[3],
    const float commandMax[3], double pwmSpan, tricopterDynamicAllocatorCandidate_t *candidate)
{
    const double cosine = cos(candidate->horizonDirectionRad), sine = sin(candidate->horizonDirectionRad);
    double responseMap[4][3], weightedHigh[3][3], target[3];
    // Effort order here is logical tail/front-right/front-left.
    for (unsigned i = 0; i < 4; ++i) {
        responseMap[i][0] = m->allocator.effectivenessMatrix[i][2] * cosine + m->allocator.effectivenessMatrix[i][3] * sine;
        responseMap[i][1] = m->allocator.effectivenessMatrix[i][0]; responseMap[i][2] = m->allocator.effectivenessMatrix[i][1];
        if (i < 3) {
            for (unsigned j = 0; j < 3; ++j) weightedHigh[i][j] = responseMap[i][j] / m->allocator.responseScale[i];
            target[i] = input->requestedResponse[i] / (double)m->allocator.responseScale[i];
        }
    }
    double condition;
    if (!highCondition(weightedHigh,&condition) || condition > p->maxHighMatrixCondition) { candidate->status = TRICOPTER_DYNAMIC_ALLOCATOR_HIGH_MATRIX_UNAVAILABLE; return; }
    candidate->highMatrixCondition = (float)condition;
    double best = DBL_MAX, optimal[3] = {0};
    for (unsigned face = 0; face < 27; ++face) {
        double x[3]; ++candidate->evaluatedFaces;
        if (!solveFace(weightedHigh,target,lower,upper,face,x)) continue;
        double cost = 0;
        for (unsigned i = 0; i < 3; ++i) { double residual = -target[i]; for (unsigned j = 0; j < 3; ++j) residual += weightedHigh[i][j] * x[j]; cost += .5 * residual * residual; }
        if (cost < best) { best = cost; memcpy(optimal,x,sizeof(optimal)); }
    }
    if (best == DBL_MAX) { candidate->status = TRICOPTER_DYNAMIC_ALLOCATOR_HIGH_MATRIX_UNAVAILABLE; return; }
    double predicted[3];
    for (unsigned i = 0; i < 3; ++i) {
        candidate->continuousOptimalEffort[i] = (float)optimal[i];
        candidate->commandMechanicalRpm[i] = commandFor(m,p,i,input->motors[i].mechanicalRpm,optimal[i],commandMax[i]);
        const double rpm = motorAt(p,i,input->motors[i].mechanicalRpm,candidate->commandMechanicalRpm[i]);
        predicted[i] = effortAt(m,i,rpm);
        candidate->predictedMechanicalRpm[i] = (float)rpm; candidate->predictedEffort[i] = (float)predicted[i];
        const double move = (predicted[i] - effortAt(m,i,input->motors[i].mechanicalRpm)) / m->allocator.maxEffort[i];
        candidate->movementCost += move * move;
    }
    const double pwmMove = ((double)candidate->pwmUs - input->currentPwmUs) / pwmSpan;
    candidate->movementCost += pwmMove * pwmMove;
    const double controls[4] = {predicted[1],predicted[2],predicted[0] * cosine,predicted[0] * sine};
    for (unsigned j = 0; j < 4; ++j) candidate->predictedControls[j] = (float)controls[j];
    for (unsigned i = 0; i < 4; ++i) {
        double achieved = 0; for (unsigned j = 0; j < 3; ++j) achieved += responseMap[i][j] * predicted[j];
        const double residual = achieved - input->requestedResponse[i];
        candidate->predictedResponse[i] = (float)achieved; candidate->residualResponse[i] = (float)residual;
        const double normalized = residual / m->allocator.responseScale[i];
        if (i < 3) candidate->highCost += .5 * normalized * normalized; else candidate->yawCost = .5 * normalized * normalized;
    }
    candidate->valid = true;
}

tricopterDynamicAllocatorOutput_t tricopterDynamicAllocatorAllocate(
    const tricopterDynamicAllocatorModel_t *m, const tricopterDynamicAllocatorParams_t *p,
    const tricopterServoModelState_t *servo, const tricopterDynamicAllocatorInput_t *input)
{
    tricopterDynamicAllocatorOutput_t out = {0};
    if (!servo || !tricopterDynamicAllocatorValidateParams(m,p,&servo->params)) { out.status = TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_CONFIG; return out; }
    if (!input || !finiteFloat(input->currentPwmUs) || !finiteFloat(input->nominalPwmUs)) { out.status = TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_INPUT; return out; }
    if (input->nominalPwmUs < servo->params.pwmUs[0] || input->nominalPwmUs > servo->params.pwmUs[servo->params.knotCount - 1]) { out.status = TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_INPUT; return out; }
    for (unsigned i = 0; i < 4; ++i) if (!finiteFloat(input->requestedResponse[i]) || fabsf(input->requestedResponse[i]) > 1e8f) { out.status = TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_INPUT; return out; }
    if (!servoStateValid(servo,input->nowUs) || servo->lastCommandPwmUs != input->currentPwmUs) { out.status = TRICOPTER_DYNAMIC_ALLOCATOR_STATE_UNAVAILABLE; return out; }
    if (input->nowUs > UINT64_MAX - p->horizonUs) { out.status = TRICOPTER_DYNAMIC_ALLOCATOR_INVALID_HORIZON; return out; }
    double lower[3], upper[3]; float commandMax[3];
    for (unsigned i = 0; i < 3; ++i) {
        const tricopterDynamicAllocatorMotorState_t *motor = &input->motors[i];
        if (!motor->seen || !motor->fresh || !motor->valid || motor->ageUs > p->maxRpmAgeUs || !finiteFloat(motor->mechanicalRpm) || motor->mechanicalRpm < 0 || motor->mechanicalRpm > 1e7f) {
            return (tricopterDynamicAllocatorOutput_t) { .status = TRICOPTER_DYNAMIC_ALLOCATOR_STATE_UNAVAILABLE };
        }
        const double exactMaximum = sqrt((double)m->allocator.maxEffort[i] / m->effortPerOmegaSquared[i]) * (30.0 / 3.14159265358979323846);
        commandMax[i] = (float)exactMaximum; if (commandMax[i] > exactMaximum) commandMax[i] = nextafterf(commandMax[i],0);
        lower[i] = effortAt(m,i,motorAt(p,i,motor->mechanicalRpm,0));
        upper[i] = fmin(m->allocator.maxEffort[i], effortAt(m,i,motorAt(p,i,motor->mechanicalRpm,commandMax[i])));
        if (lower[i] > upper[i]) return (tricopterDynamicAllocatorOutput_t) { .status = TRICOPTER_DYNAMIC_ALLOCATOR_EMPTY_REACHABLE_BOX };
        out.reachableMinEffort[i] = (float)lower[i]; out.reachableMaxEffort[i] = (float)upper[i]; out.maximumCommandRpm[i] = commandMax[i];
    }
    pwmBounds(m,&servo->params,&out.minimumPwmUs,&out.maximumPwmUs);
    for (unsigned i = 0; i < (unsigned)p->gridCount + 2; ++i) {
        const float requested = i < p->gridCount ? p->pwmGridUs[i] : i == p->gridCount ? input->currentPwmUs : input->nominalPwmUs;
        const float pwm = (float)clamp(requested,out.minimumPwmUs,out.maximumPwmUs);
        if (pwm != requested) out.status |= TRICOPTER_DYNAMIC_ALLOCATOR_PWM_TARGET_CLIPPED;
        bool duplicate = false; for (unsigned k = 0; k < out.candidateCount; ++k) if (out.candidates[k].pwmUs == pwm) duplicate = true;
        if (duplicate) continue;
        tricopterDynamicAllocatorCandidate_t *candidate = &out.candidates[out.candidateCount++]; candidate->pwmUs = pwm;
        if (!predictDirection(m,p,servo,input->nowUs,pwm,&candidate->horizonCorrectedCounts,&candidate->horizonDirectionRad)) {
            candidate->status = TRICOPTER_DYNAMIC_ALLOCATOR_SERVO_PREDICTION_UNAVAILABLE; out.status |= candidate->status; continue;
        }
        evaluateCandidate(m,p,input,lower,upper,commandMax,(double)out.maximumPwmUs - out.minimumPwmUs,candidate);
        out.status |= candidate->status;
        if (candidate->valid) ++out.validCandidateCount;
    }
    if (!out.validCandidateCount) { out.status |= TRICOPTER_DYNAMIC_ALLOCATOR_NO_VALID_CANDIDATE; return out; }
    double minDirection = DBL_MAX, maxDirection = -DBL_MAX;
    out.minimumHighCost = DBL_MAX;
    for (unsigned i = 0; i < out.candidateCount; ++i) if (out.candidates[i].valid) {
        const tricopterDynamicAllocatorCandidate_t *candidate = &out.candidates[i];
        out.minimumHighCost = fmin(out.minimumHighCost,candidate->highCost);
        minDirection = fmin(minDirection,candidate->horizonDirectionRad); maxDirection = fmax(maxDirection,candidate->horizonDirectionRad);
    }
    if (maxDirection - minDirection <= 1e-7 && fabs((double)input->nominalPwmUs - input->currentPwmUs) > 1e-4) {
        out.status |= TRICOPTER_DYNAMIC_ALLOCATOR_NO_DIRECTION_AUTHORITY; return out;
    }
    out.minimumYawCostWithinHighTolerance = DBL_MAX;
    for (unsigned i = 0; i < out.candidateCount; ++i) if (out.candidates[i].valid && out.candidates[i].highCost <= out.minimumHighCost + p->highCostTolerance)
        out.minimumYawCostWithinHighTolerance = fmin(out.minimumYawCostWithinHighTolerance,out.candidates[i].yawCost);
    double minMovement = DBL_MAX;
    for (unsigned i = 0; i < out.candidateCount; ++i) {
        const tricopterDynamicAllocatorCandidate_t *candidate = &out.candidates[i];
        if (candidate->valid && candidate->highCost <= out.minimumHighCost + p->highCostTolerance
            && candidate->yawCost <= out.minimumYawCostWithinHighTolerance + p->yawCostTolerance) minMovement = fmin(minMovement,candidate->movementCost);
    }
    for (unsigned i = 0; i < out.candidateCount; ++i) {
        const tricopterDynamicAllocatorCandidate_t *candidate = &out.candidates[i];
        if (candidate->valid && candidate->highCost <= out.minimumHighCost + p->highCostTolerance
            && candidate->yawCost <= out.minimumYawCostWithinHighTolerance + p->yawCostTolerance
            && candidate->movementCost <= minMovement + p->movementCostTolerance) {
            out.selected = *candidate; out.selectedCandidate = i; out.valid = out.priorityAvailable = true; break;
        }
    }
    return out;
}
