#include "tricopter_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CONTROL_PI 3.14159265358979323846f
#define CONTROL_PWM_ABSOLUTE_MIN 500.0f
#define CONTROL_PWM_ABSOLUTE_MAX 2500.0f

static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static float bounded(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

static bool monotone(const float values[3])
{
    return (values[1] > values[0] && values[2] > values[1])
        || (values[1] < values[0] && values[2] < values[1]);
}

// All arrays are validated before interpolation. Results are not extrapolated.
static bool interpolate(const float x[3], const float y[3], float query, float *result)
{
    if (!result || !finiteFloat(query)) return false;
    for (unsigned i = 1; i < 3; ++i) {
        const float a = x[i-1], b = x[i];
        const float low = a < b ? a : b, high = a > b ? a : b;
        if (query < low || query > high) continue;
        const float fraction = ((float)query-a)/(b-a);
        *result = (float)(y[i-1] + fraction*((float)y[i]-y[i-1]));
        return true;
    }
    return false;
}

tricopterControlParams_t tricopterControlDefaultParams(void)
{
    return (tricopterControlParams_t) {
        .pwmUs = {1100, 1509, 1700},
        .angleDeg = {-47.8f, 0, 21.6f},
        .adcCounts = {1936, 1490, 1281},
        .positionGainUsPerDeg = 2,
        .yawGain = .7f,
        .yawDirection = 1,
        .neutralTrimDeg = 0,
        .feedbackTimeoutUs = 20000,
        .adcMarginCounts = 32,
        .minVerticalEffort = .05f,
        .pwmMinUs = 1100,
        .pwmMaxUs = 1700,
    };
}

bool tricopterControlValidateParams(const tricopterControlParams_t *p)
{
    if (!p) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!finiteFloat(p->pwmUs[i]) || p->pwmUs[i] < CONTROL_PWM_ABSOLUTE_MIN || p->pwmUs[i] > CONTROL_PWM_ABSOLUTE_MAX
            || !finiteFloat(p->angleDeg[i]) || fabsf(p->angleDeg[i]) > TRICOPTER_CONTROL_MAX_ANGLE_DEG
            || !finiteFloat(p->adcCounts[i]) || fabsf(p->adcCounts[i]) > 4095) return false;
    }
    if (p->angleDeg[0] >= 0 || p->angleDeg[1] != 0 || p->angleDeg[2] <= 0
        || !monotone(p->pwmUs) || !monotone(p->adcCounts)
        || !finiteFloat(p->positionGainUsPerDeg) || p->positionGainUsPerDeg < 0 || p->positionGainUsPerDeg > 1000
        || !finiteFloat(p->yawGain) || p->yawGain < 0 || p->yawGain > 100
        || (p->yawDirection != 1 && p->yawDirection != -1)
        || !finiteFloat(p->neutralTrimDeg) || fabsf(p->neutralTrimDeg) > TRICOPTER_CONTROL_MAX_ANGLE_DEG
        || !p->feedbackTimeoutUs || p->feedbackTimeoutUs > 1000000
        || !finiteFloat(p->adcMarginCounts) || p->adcMarginCounts < 0 || p->adcMarginCounts > 4095
        || !finiteFloat(p->minVerticalEffort) || p->minVerticalEffort <= 0 || p->minVerticalEffort > 1
        || !finiteFloat(p->pwmMinUs) || !finiteFloat(p->pwmMaxUs)
        || p->pwmMinUs < CONTROL_PWM_ABSOLUTE_MIN || p->pwmMaxUs > CONTROL_PWM_ABSOLUTE_MAX
        || p->pwmMinUs >= p->pwmMaxUs || p->pwmUs[1] < p->pwmMinUs || p->pwmUs[1] > p->pwmMaxUs) return false;
    return true;
}

bool tricopterControlAngleFromAdc(const tricopterControlParams_t *p, float counts, float *angle)
{
    if (!angle || !tricopterControlValidateParams(p) || !finiteFloat(counts)) return false;
    const float low = p->adcCounts[0] < p->adcCounts[2] ? p->adcCounts[0] : p->adcCounts[2];
    const float high = p->adcCounts[0] > p->adcCounts[2] ? p->adcCounts[0] : p->adcCounts[2];
    if (counts < low-p->adcMarginCounts || counts > high+p->adcMarginCounts) return false;
    return interpolate(p->adcCounts, p->angleDeg, (float)bounded(counts, low, high), angle);
}

bool tricopterControlPwmFromAngle(const tricopterControlParams_t *p, float angle, float *pwm)
{
    return pwm && tricopterControlValidateParams(p) && interpolate(p->angleDeg, p->pwmUs, angle, pwm);
}

static float safeNeutral(const tricopterControlParams_t *p)
{
    float neutral = p && finiteFloat(p->pwmUs[1]) ? p->pwmUs[1] : 1500;
    neutral = bounded(neutral, CONTROL_PWM_ABSOLUTE_MIN, CONTROL_PWM_ABSOLUTE_MAX);
    if (p && finiteFloat(p->pwmMinUs) && finiteFloat(p->pwmMaxUs)
        && p->pwmMinUs < p->pwmMaxUs) {
        const float low = bounded(p->pwmMinUs, CONTROL_PWM_ABSOLUTE_MIN, CONTROL_PWM_ABSOLUTE_MAX);
        const float high = bounded(p->pwmMaxUs, CONTROL_PWM_ABSOLUTE_MIN, CONTROL_PWM_ABSOLUTE_MAX);
        if (low <= high) neutral = bounded(neutral, low, high);
    }
    return (float)neutral;
}

tricopterControlOutput_t tricopterControlEvaluate(const tricopterControlParams_t *p,
    const tricopterControlInput_t *in)
{
    tricopterControlOutput_t out = {.pwmUs = safeNeutral(p)};
    if (in && finiteFloat(in->verticalEffort)) {
        out.tailEffort = (float)bounded(in->verticalEffort, 0.0f, 1.0f);
        out.verticalSaturated = out.tailEffort != in->verticalEffort;
    }
    out.validConfig = tricopterControlValidateParams(p);
    out.validInput = in && finiteFloat(in->verticalEffort) && finiteFloat(in->normalizedYaw);
    if (!out.validConfig || !out.validInput) return out;

    const float vertical = out.tailEffort;
    const float yaw = bounded(in->normalizedYaw, -1e6f, 1e6f);
    const float lateral = yaw*p->yawGain*p->yawDirection;
    out.yawSaturated = yaw != in->normalizedYaw;
    const float denominator = vertical > p->minVerticalEffort ? vertical : p->minVerticalEffort;
    const float requestedAngle = atan2f(lateral, denominator)*(180.0f/CONTROL_PI)+p->neutralTrimDeg;

    const float mapPwmLow = p->pwmUs[0] < p->pwmUs[2] ? p->pwmUs[0] : p->pwmUs[2];
    const float mapPwmHigh = p->pwmUs[0] > p->pwmUs[2] ? p->pwmUs[0] : p->pwmUs[2];
    const float pwmLow = (float)(p->pwmMinUs > mapPwmLow ? p->pwmMinUs : mapPwmLow);
    const float pwmHigh = (float)(p->pwmMaxUs < mapPwmHigh ? p->pwmMaxUs : mapPwmHigh);
    float angleA, angleB;
    if (!interpolate(p->pwmUs, p->angleDeg, pwmLow, &angleA)
        || !interpolate(p->pwmUs, p->angleDeg, pwmHigh, &angleB)) {
        out.validConfig = false;
        return out;
    }
    const float angleLow = angleA < angleB ? angleA : angleB;
    const float angleHigh = angleA > angleB ? angleA : angleB;
    const float feasible = acosf(vertical)*(180.0f/CONTROL_PI);
    const float lower = angleLow > -feasible ? angleLow : -feasible;
    const float upper = angleHigh < feasible ? angleHigh : feasible;
    out.targetAngleDeg = (float)bounded(requestedAngle, lower, upper);
    if (requestedAngle < lower || requestedAngle > upper
        || (vertical < p->minVerticalEffort && lateral != 0)) out.yawSaturated = true;

    float pwm;
    if (!interpolate(p->angleDeg, p->pwmUs, out.targetAngleDeg, &pwm)) {
        out.validConfig = false;
        return out;
    }
    out.feedbackValid = in->feedbackValid && in->feedbackAgeUs <= p->feedbackTimeoutUs
        && tricopterControlAngleFromAdc(p, in->correctedCounts, &out.actualAngleDeg);
    if (out.feedbackValid) {
        const float actualRadians = out.actualAngleDeg*(CONTROL_PI/180.0f);
        const float cosine = cosf(actualRadians);
        if (cosine < TRICOPTER_CONTROL_MIN_COSINE) {
            out.feedbackValid = false;
            out.actualAngleDeg = 0;
        } else {
            const float compensated = vertical/cosine;
            out.tailEffort = (float)bounded(compensated, 0.0f, 1.0f);
            if (compensated > 1) out.verticalSaturated = true;
            out.currentVertical = (float)(out.tailEffort*cosine);
            out.currentLateral = (float)(out.tailEffort*sinf(actualRadians));
            out.currentVectorAvailable = true;
            const float pwmDirection = p->pwmUs[2] > p->pwmUs[0] ? 1 : -1;
            pwm = (float)(pwm+p->positionGainUsPerDeg*((float)out.targetAngleDeg-out.actualAngleDeg)*pwmDirection);
        }
    }
    out.pwmUs = (float)bounded(pwm, pwmLow, pwmHigh);
    if (pwm < pwmLow || pwm > pwmHigh) out.yawSaturated = true;
    return out;
}
