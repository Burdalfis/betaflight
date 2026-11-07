/* Offline convex geometric allocation. No firmware or actuator dependencies. */
#include "tricopter_allocator.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128 || DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024
#error "Tricopter allocation requires binary32 float and binary64 double"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "Allocator requires binary32 float");
_Static_assert(sizeof(double) == sizeof(uint64_t), "Allocator requires binary64 double");

typedef struct allocationMatrix_s {
    double a[4][4];
    double b[4][4];
    double inverse[4][4];
    double step;
    double condition;
    double minRay[2];
    double maxRay[2];
} allocationMatrix_t;

static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static double clamp(double value, double lower, double upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

static bool invert(const double matrix[4][4], double inverse[4][4], double *condition)
{
    double augmented[4][8];
    double norm = 0;
    for (unsigned i = 0; i < 4; ++i) {
        double rowNorm = 0;
        for (unsigned j = 0; j < 4; ++j) {
            augmented[i][j] = matrix[i][j];
            augmented[i][4 + j] = i == j ? 1.0 : 0.0;
            rowNorm += fabs(matrix[i][j]);
        }
        norm = fmax(norm, rowNorm);
    }
    for (unsigned column = 0; column < 4; ++column) {
        unsigned pivot = column;
        for (unsigned i = column + 1; i < 4; ++i) {
            if (fabs(augmented[i][column]) > fabs(augmented[pivot][column])) {
                pivot = i;
            }
        }
        if (fabs(augmented[pivot][column]) <= norm * 1e-14) {
            return false;
        }
        if (pivot != column) {
            for (unsigned j = 0; j < 8; ++j) {
                const double swap = augmented[column][j];
                augmented[column][j] = augmented[pivot][j];
                augmented[pivot][j] = swap;
            }
        }
        const double divisor = augmented[column][column];
        for (unsigned j = 0; j < 8; ++j) {
            augmented[column][j] /= divisor;
        }
        for (unsigned i = 0; i < 4; ++i) {
            if (i != column) {
                const double factor = augmented[i][column];
                for (unsigned j = 0; j < 8; ++j) {
                    augmented[i][j] -= factor * augmented[column][j];
                }
            }
        }
    }
    double inverseNorm = 0;
    for (unsigned i = 0; i < 4; ++i) {
        double rowNorm = 0;
        for (unsigned j = 0; j < 4; ++j) {
            inverse[i][j] = augmented[i][4 + j];
            rowNorm += fabs(inverse[i][j]);
        }
        inverseNorm = fmax(inverseNorm, rowNorm);
    }
    *condition = norm * inverseNorm;
    return true;
}

static uint32_t prepare(const tricopterAllocatorVirtualParams_t *p, allocationMatrix_t *matrix)
{
    if (!p || p->source != TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER
        || !finiteFloat(p->tailDirectionMinRad) || !finiteFloat(p->tailDirectionMaxRad)
        || p->tailDirectionMinRad <= -1.57079632679489661923 || p->tailDirectionMaxRad >= 1.57079632679489661923
        || p->tailDirectionMinRad > 0 || p->tailDirectionMaxRad < 0
        || !finiteFloat(p->lowTailEffort) || p->lowTailEffort < 0 || p->lowTailEffort > p->maxEffort[0]
        || !finiteFloat(p->projectedGradientTolerance) || p->projectedGradientTolerance < 1e-12f || p->projectedGradientTolerance > 1e6f
        || !finiteFloat(p->maxConditionNumber) || p->maxConditionNumber < 1 || p->maxConditionNumber > 1e8f
        || !p->maxIterations || p->maxIterations > TRICOPTER_ALLOCATOR_MAX_ITERATIONS) {
        return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!finiteFloat(p->maxEffort[i]) || p->maxEffort[i] <= 0 || p->maxEffort[i] > 1e6f) return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
    }
    for (unsigned i = 0; i < 4; ++i) {
        if (!finiteFloat(p->responseScale[i]) || p->responseScale[i] < 1e-6f || p->responseScale[i] > 1e6f) {
            return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
        }
    }
    memset(matrix, 0, sizeof(*matrix));
    double trace = 0;
    for (unsigned i = 0; i < 4; ++i) {
        for (unsigned j = 0; j < 4; ++j) {
            if (!finiteFloat(p->effectivenessMatrix[i][j]) || fabsf(p->effectivenessMatrix[i][j]) > 1e6f) return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
            matrix->a[i][j] = p->effectivenessMatrix[i][j];
            matrix->b[i][j] = matrix->a[i][j] / p->responseScale[i];
            trace += matrix->b[i][j] * matrix->b[i][j];
        }
    }
    if (!invert(matrix->b, matrix->inverse, &matrix->condition)) {
        return TRICOPTER_ALLOCATOR_DEGENERATE_GEOMETRY;
    }
    if (matrix->condition > p->maxConditionNumber || trace < 1e-20) {
        return TRICOPTER_ALLOCATOR_ILL_CONDITIONED;
    }
    matrix->step = 1.0 / trace;
    matrix->minRay[0] = cos(p->tailDirectionMinRad);
    matrix->minRay[1] = sin(p->tailDirectionMinRad);
    matrix->maxRay[0] = cos(p->tailDirectionMaxRad);
    matrix->maxRay[1] = sin(p->tailDirectionMaxRad);
    return 0;
}

bool tricopterAllocatorVirtualValidateParams(const tricopterAllocatorVirtualParams_t *params)
{
    allocationMatrix_t matrix;
    return prepare(params, &matrix) == 0;
}

static void project(const tricopterAllocatorVirtualParams_t *p, const allocationMatrix_t *m, const double x[4], double result[4])
{
    result[0] = clamp(x[0], 0, p->maxEffort[1]);
    result[1] = clamp(x[1], 0, p->maxEffort[2]);
    double v = x[2], h = x[3];
    if (v < 0 || m->minRay[0] * h - m->minRay[1] * v < 0 || v * m->maxRay[1] - h * m->maxRay[0] < 0) {
        const double lower = fmax(0, v * m->minRay[0] + h * m->minRay[1]);
        const double upper = fmax(0, v * m->maxRay[0] + h * m->maxRay[1]);
        // Projection onto either cone boundary; the origin is included by
        // clamping ray coordinates to nonnegative values.
        const double *ray = lower >= upper ? m->minRay : m->maxRay;
        const double length = lower >= upper ? lower : upper;
        v = length * ray[0];
        h = length * ray[1];
    }
    const double length = hypot(v, h);
    if (length > p->maxEffort[0]) {
        const double factor = p->maxEffort[0] / length;
        v *= factor; h *= factor;
    }
    result[2] = v; result[3] = h;
}

static double costGradient(const tricopterAllocatorVirtualParams_t *p, const allocationMatrix_t *m,
    const double x[4], const double demand[4], double gradient[4])
{
    double cost = 0;
    for (unsigned j = 0; j < 4; ++j) gradient[j] = 0;
    for (unsigned i = 0; i < 4; ++i) {
        double error = -demand[i];
        for (unsigned j = 0; j < 4; ++j) error += m->a[i][j] * x[j];
        error /= p->responseScale[i];
        cost += 0.5 * error * error;
        for (unsigned j = 0; j < 4; ++j) gradient[j] += m->b[i][j] * error;
    }
    return cost;
}

static double mapping(const tricopterAllocatorVirtualParams_t *p, const allocationMatrix_t *m,
    const double x[4], const double gradient[4], double projected[4])
{
    double next[4];
    for (unsigned j = 0; j < 4; ++j) next[j] = x[j] - m->step * gradient[j];
    project(p, m, next, projected);
    double norm = 0;
    for (unsigned j = 0; j < 4; ++j) norm = hypot(norm, x[j] - projected[j]);
    return norm / m->step;
}

static uint32_t reactionStatus(float yaw, bool enabled, bool valid)
{
    if (!finiteFloat(yaw) || fabsf(yaw) > 1e8f) return TRICOPTER_ALLOCATOR_INVALID_INPUT;
    return enabled && !valid ? TRICOPTER_ALLOCATOR_REACTION_UNAVAILABLE : 0;
}

static void fill(const tricopterAllocatorVirtualParams_t *p, const allocationMatrix_t *m, const double input[4],
    const double demand[4], const float requested[4], tricopterAllocatorVirtualOutput_t *out)
{
    const double length = hypot(input[2], input[3]);
    out->effort[0] = (float)clamp(length, 0, p->maxEffort[0]);
    out->effort[1] = (float)clamp(input[0], 0, p->maxEffort[1]);
    out->effort[2] = (float)clamp(input[1], 0, p->maxEffort[2]);
    out->tailDirectionRad = out->effort[0] > 0 ? (float)clamp(atan2(input[3], input[2]), p->tailDirectionMinRad, p->tailDirectionMaxRad) : 0;
    // Published effort/direction are canonical; re-evaluate from their float32
    // values so forward reconstruction agrees within output roundoff.
    const double x[4] = {out->effort[1], out->effort[2],
        out->effort[0] * cos(out->tailDirectionRad), out->effort[0] * sin(out->tailDirectionRad)};
    for (unsigned j = 0; j < 4; ++j) out->controls[j] = (float)x[j];
    for (unsigned i = 0; i < 4; ++i) {
        double achieved = 0;
        for (unsigned j = 0; j < 4; ++j) achieved += m->a[i][j] * x[j];
        out->requestedResponse[i] = requested[i];
        out->achievedResponse[i] = (float)achieved;
        out->residualResponse[i] = (float)(achieved - requested[i]);
    }
    double gradient[4], projected[4];
    out->cost = (float)costGradient(p, m, x, demand, gradient);
    out->projectedGradientNorm = (float)mapping(p, m, x, gradient, projected);
    out->conditionNumber = (float)m->condition;
    out->tailDirectionDefined = out->effort[0] > 0;
    out->tailEffortLow = out->effort[0] <= p->lowTailEffort;
    if (out->effort[0] == 0) out->constraints |= TRICOPTER_ALLOCATOR_TAIL_ZERO;
    if (out->effort[0] == p->maxEffort[0]) out->constraints |= TRICOPTER_ALLOCATOR_TAIL_MAX;
    if (out->effort[1] == 0) out->constraints |= TRICOPTER_ALLOCATOR_FRONT_RIGHT_ZERO;
    if (out->effort[1] == p->maxEffort[1]) out->constraints |= TRICOPTER_ALLOCATOR_FRONT_RIGHT_MAX;
    if (out->effort[2] == 0) out->constraints |= TRICOPTER_ALLOCATOR_FRONT_LEFT_ZERO;
    if (out->effort[2] == p->maxEffort[2]) out->constraints |= TRICOPTER_ALLOCATOR_FRONT_LEFT_MAX;
    if (out->tailDirectionDefined && out->tailDirectionRad <= p->tailDirectionMinRad) out->constraints |= TRICOPTER_ALLOCATOR_ANGLE_MIN;
    if (out->tailDirectionDefined && out->tailDirectionRad >= p->tailDirectionMaxRad) out->constraints |= TRICOPTER_ALLOCATOR_ANGLE_MAX;
    out->valid = true;
}

static tricopterAllocatorVirtualOutput_t allocateVirtual(const tricopterAllocatorVirtualParams_t *p,
    const double demand[4])
{
    tricopterAllocatorVirtualOutput_t out = {0};
    allocationMatrix_t m;
    out.status = prepare(p, &m);
    if (out.status) return out;
    if (!demand) { out.status = TRICOPTER_ALLOCATOR_INVALID_INPUT; return out; }
    double inverseControl[4] = {0}, x[4];
    float requested[4];
    for (unsigned i = 0; i < 4; ++i) {
        requested[i] = (float)demand[i];
    }
    for (unsigned i = 0; i < 4; ++i) {
        for (unsigned j = 0; j < 4; ++j) inverseControl[i] += m.inverse[i][j] * (demand[j] / p->responseScale[j]);
    }
    project(p, &m, inverseControl, x);
    double distance = 0, scale = 1;
    for (unsigned i = 0; i < 4; ++i) { distance = hypot(distance, x[i] - inverseControl[i]); scale = fmax(scale, fabs(x[i])); }
    out.exact = distance <= 1e-10 * scale;
    if (!out.exact) {
        for (unsigned iteration = 0; iteration < p->maxIterations; ++iteration) {
            double gradient[4], next[4];
            costGradient(p, &m, x, demand, gradient);
            if (mapping(p, &m, x, gradient, next) <= p->projectedGradientTolerance) {
                tricopterAllocatorVirtualOutput_t published = {0};
                fill(p, &m, x, demand, requested, &published);
                if (published.projectedGradientNorm <= p->projectedGradientTolerance) break;
            }
            memcpy(x, next, sizeof(x));
            ++out.iterations;
        }
    }
    fill(p, &m, x, demand, requested, &out);
    out.converged = out.projectedGradientNorm <= p->projectedGradientTolerance;
    if (!out.converged) out.status |= TRICOPTER_ALLOCATOR_NOT_CONVERGED;
    return out;
}

tricopterAllocatorVirtualOutput_t tricopterAllocatorVirtualAllocate(const tricopterAllocatorVirtualParams_t *p,
    const tricopterAllocatorVirtualRequest_t *request)
{
    if (!request) return allocateVirtual(p, NULL);
    double demand[4];
    for (unsigned i = 0; i < 4; ++i) {
        if (!finiteFloat(request->response[i]) || fabsf(request->response[i]) > 1e8f) {
            return (tricopterAllocatorVirtualOutput_t) { .status = TRICOPTER_ALLOCATOR_INVALID_INPUT };
        }
        demand[i] = request->response[i];
    }
    return allocateVirtual(p, demand);
}

tricopterAllocatorVirtualOutput_t tricopterAllocatorVirtualForward(const tricopterAllocatorVirtualParams_t *p,
    const float effort[3], float tailDirectionRad)
{
    tricopterAllocatorVirtualOutput_t out = {0};
    allocationMatrix_t m;
    out.status = prepare(p, &m);
    if (out.status) return out;
    if (!effort || !finiteFloat(tailDirectionRad)) { out.status = TRICOPTER_ALLOCATOR_INVALID_INPUT; return out; }
    if (tailDirectionRad < p->tailDirectionMinRad || tailDirectionRad > p->tailDirectionMaxRad) { out.status = TRICOPTER_ALLOCATOR_FORWARD_BOUNDS; return out; }
    for (unsigned i = 0; i < 3; ++i) {
        if (!finiteFloat(effort[i])) { out.status = TRICOPTER_ALLOCATOR_INVALID_INPUT; return out; }
        if (effort[i] < 0 || effort[i] > p->maxEffort[i]) { out.status = TRICOPTER_ALLOCATOR_FORWARD_BOUNDS; return out; }
    }
    const double x[4] = {effort[1], effort[2], effort[0] * cos(tailDirectionRad), effort[0] * sin(tailDirectionRad)};
    const float empty[4] = {0};
    const double demand[4] = {0};
    fill(p, &m, x, demand, empty, &out);
    // Forward evaluation has no optimization request or residual.
    for (unsigned i = 0; i < 4; ++i) { out.requestedResponse[i] = out.achievedResponse[i]; out.residualResponse[i] = 0; }
    out.cost = out.projectedGradientNorm = 0;
    out.exact = out.converged = true;
    return out;
}

static uint32_t physicalToVirtual(const tricopterAllocatorParams_t *p, tricopterAllocatorVirtualParams_t *v)
{
    if (!p) return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
    memset(v, 0, sizeof(*v));
    for (unsigned i = 0; i < 3; ++i) {
        if (!finiteFloat(p->kT[i]) || p->kT[i] < 1e-12f || p->kT[i] > 1e3f) return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
        for (unsigned j = 0; j < 3; ++j) {
            if (!finiteFloat(p->motorPositionM[i][j]) || fabsf(p->motorPositionM[i][j]) > 1000) return TRICOPTER_ALLOCATOR_INVALID_CONFIG;
        }
        v->maxEffort[i] = p->maxThrustN[i];
    }
    v->effectivenessMatrix[0][0] = v->effectivenessMatrix[0][1] = v->effectivenessMatrix[0][2] = 1;
    v->effectivenessMatrix[1][0] = p->motorPositionM[1][1];
    v->effectivenessMatrix[1][1] = p->motorPositionM[2][1];
    v->effectivenessMatrix[1][2] = p->motorPositionM[0][1];
    v->effectivenessMatrix[1][3] = p->motorPositionM[0][2];
    v->effectivenessMatrix[2][0] = -p->motorPositionM[1][0];
    v->effectivenessMatrix[2][1] = -p->motorPositionM[2][0];
    v->effectivenessMatrix[2][2] = -p->motorPositionM[0][0];
    v->effectivenessMatrix[3][3] = -p->motorPositionM[0][0];
    v->tailDirectionMinRad = p->tailAngleMinRad;
    v->tailDirectionMaxRad = p->tailAngleMaxRad;
    memcpy(v->responseScale, p->wrenchScale, sizeof(v->responseScale));
    v->lowTailEffort = p->lowTailThrustN;
    v->projectedGradientTolerance = p->projectedGradientTolerance;
    v->maxConditionNumber = p->maxConditionNumber;
    v->maxIterations = p->maxIterations;
    v->source = p->source;
    return 0;
}

bool tricopterAllocatorValidateParams(const tricopterAllocatorParams_t *p)
{
    tricopterAllocatorVirtualParams_t v;
    return physicalToVirtual(p, &v) == 0 && tricopterAllocatorVirtualValidateParams(&v);
}

static tricopterAllocatorOutput_t physicalOutput(const tricopterAllocatorParams_t *p,
    const tricopterAllocatorVirtualParams_t *v, const tricopterAllocatorVirtualOutput_t *input,
    const float requested[4], double reaction)
{
    tricopterAllocatorOutput_t out = { .status = input->status };
    if (!input->valid) return out;
    memcpy(out.thrustN, input->effort, sizeof(out.thrustN));
    memcpy(out.controlsN, input->controls, sizeof(out.controlsN));
    out.tailAngleRad = input->tailDirectionRad;
    out.conditionNumber = input->conditionNumber;
    out.constraints = input->constraints;
    out.iterations = input->iterations;
    out.exact = input->exact;
    out.tailAngleDefined = input->tailDirectionDefined;
    out.tailThrustLow = input->tailEffortLow;
    out.valid = true;
    const double x[4] = {out.thrustN[1], out.thrustN[2], out.thrustN[0] * cos(out.tailAngleRad), out.thrustN[0] * sin(out.tailAngleRad)};
    allocationMatrix_t matrix;
    prepare(v, &matrix); // Already validated by the shared kernel.
    double demand[4];
    for (unsigned i = 0; i < 4; ++i) {
        double achieved = i == 3 ? reaction : 0;
        for (unsigned j = 0; j < 4; ++j) achieved += matrix.a[i][j] * x[j];
        out.achievedWrench[i] = (float)achieved;
        out.requestedWrench[i] = requested ? requested[i] : out.achievedWrench[i];
        out.residualWrench[i] = requested ? (float)(achieved - requested[i]) : 0;
        demand[i] = out.requestedWrench[i] - (i == 3 ? reaction : 0);
    }
    out.forceN[1] = -out.controlsN[3];
    out.forceN[2] = out.achievedWrench[0];
    for (unsigned i = 0; i < 3; ++i) {
        const double omega = sqrt((double)out.thrustN[i] / p->kT[i]);
        out.omegaRadS[i] = (float)omega;
        out.mechanicalRpm[i] = (float)(omega * (30.0 / 3.14159265358979323846));
    }
    if (requested) {
        double gradient[4], projected[4];
        out.cost = (float)costGradient(v, &matrix, x, demand, gradient);
        out.projectedGradientNorm = (float)mapping(v, &matrix, x, gradient, projected);
        out.converged = out.projectedGradientNorm <= p->projectedGradientTolerance;
        out.status &= ~TRICOPTER_ALLOCATOR_NOT_CONVERGED;
        if (!out.converged) out.status |= TRICOPTER_ALLOCATOR_NOT_CONVERGED;
    } else {
        out.converged = true;
    }
    return out;
}

tricopterAllocatorOutput_t tricopterAllocatorAllocate(const tricopterAllocatorParams_t *p,
    const tricopterAllocatorRequest_t *request)
{
    tricopterAllocatorOutput_t failed = {0};
    tricopterAllocatorVirtualParams_t v;
    failed.status = physicalToVirtual(p, &v);
    if (failed.status) return failed;
    if (!request) { failed.status = TRICOPTER_ALLOCATOR_INVALID_INPUT; return failed; }
    failed.status = reactionStatus(request->reactionYawNm, request->useReactionYaw, request->reactionYawValid);
    if (failed.status) return failed;
    double net[4];
    for (unsigned i = 0; i < 4; ++i) {
        if (!finiteFloat(request->wrench[i]) || fabsf(request->wrench[i]) > 1e8f) { failed.status = TRICOPTER_ALLOCATOR_INVALID_INPUT; return failed; }
        net[i] = request->wrench[i] - (i == 3 && request->useReactionYaw ? (double)request->reactionYawNm : 0);
    }
    const tricopterAllocatorVirtualOutput_t result = allocateVirtual(&v, net);
    return physicalOutput(p, &v, &result, request->wrench, request->useReactionYaw ? request->reactionYawNm : 0);
}

tricopterAllocatorOutput_t tricopterAllocatorForward(const tricopterAllocatorParams_t *p,
    const float thrustN[3], float tailAngleRad, float reactionYawNm, bool useReactionYaw, bool reactionYawValid)
{
    tricopterAllocatorOutput_t failed = {0};
    tricopterAllocatorVirtualParams_t v;
    failed.status = physicalToVirtual(p, &v);
    if (failed.status) return failed;
    failed.status = reactionStatus(reactionYawNm, useReactionYaw, reactionYawValid);
    if (failed.status) return failed;
    const tricopterAllocatorVirtualOutput_t result = tricopterAllocatorVirtualForward(&v, thrustN, tailAngleRad);
    return physicalOutput(p, &v, &result, NULL, useReactionYaw ? reactionYawNm : 0);
}
