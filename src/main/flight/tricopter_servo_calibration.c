#include "tricopter_servo_calibration.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

bool tricopterServoCalibrationValidateParams(const tricopterServoCalibrationParams_t *p)
{
    if (!p || p->pwmUs[0] < 500 || p->pwmUs[2] > 2500
        || p->pwmUs[0] >= p->pwmUs[1] || p->pwmUs[1] >= p->pwmUs[2]) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!finiteFloat(p->angleDeg[i]) || fabsf(p->angleDeg[i]) > 180) return false;
    }
    const float a = p->angleDeg[1] - p->angleDeg[0];
    const float b = p->angleDeg[2] - p->angleDeg[1];
    return ((a > 0 && b > 0) || (a < 0 && b < 0))
        && fabsf(p->angleDeg[2] - p->angleDeg[0]) >= 1;
}

static uint16_t centerPwm(const tricopterServoCalibrationState_t *s)
{
    const uint16_t value = s->params.pwmUs[1];
    return value >= 500 && value <= 2500 ? value : 1500;
}

static tricopterServoCalibrationOutput_t invalidOutput(void)
{
    return (tricopterServoCalibrationOutput_t) {
        .stage = TRICOPTER_SERVO_CALIBRATION_FAILED,
        .status = TRICOPTER_SERVO_CALIBRATION_INVALID_PARAMS,
        .commandedPwmUs = 1500, .failed = true,
    };
}

static void stop(tricopterServoCalibrationState_t *s, tricopterServoCalibrationStatus_t reason)
{
    s->output.stage = reason == TRICOPTER_SERVO_CALIBRATION_ARMED || reason == TRICOPTER_SERVO_CALIBRATION_USER_ABORT
        ? TRICOPTER_SERVO_CALIBRATION_ABORTED : TRICOPTER_SERVO_CALIBRATION_FAILED;
    s->output.status = reason;
    s->output.active = false;
    s->output.complete = s->output.resultValid = s->output.speedEstimateValid = false;
    s->output.failed = true;
    s->output.observedSpeedDegPerSecond = 0;
    s->output.observed10To90Us = 0;
    s->output.commandedPwmUs = centerPwm(s);
    s->output.commandValid = reason != TRICOPTER_SERVO_CALIBRATION_ARMED
        && reason != TRICOPTER_SERVO_CALIBRATION_INVALID_PARAMS;
}

static void clearAverage(tricopterServoCalibrationState_t *s)
{
    s->averageActive = false;
    s->averageSum = s->firstHalfSum = s->secondHalfSum = 0;
    s->averageSamples = s->firstHalfSamples = s->secondHalfSamples = 0;
    s->averageMin = INT16_MAX;
    s->averageMax = INT16_MIN;
}

static void enterStage(tricopterServoCalibrationState_t *s, tricopterServoCalibrationStage_t stage, uint32_t now)
{
    s->output.stage = stage;
    s->stageStartUs = s->lastSampleUs = s->lastRampUs = now;
    clearAverage(s);
    if (stage == TRICOPTER_SERVO_CALIBRATION_HIGH) {
        s->traceStartUs = s->lastTraceUs = now;
        s->traceIntervalUs = TRICOPTER_SERVO_CALIBRATION_SAMPLE_US;
        s->traceCount = 1;
        s->traceTimeUs[0] = 0;
        s->traceCounts[0] = s->output.learnedAdc[0];
    }
}

bool tricopterServoCalibrationStart(tricopterServoCalibrationState_t *s,
    const tricopterServoCalibrationParams_t *p, uint32_t now, bool disarmed)
{
    if (!s || s->output.active) return false;
    const bool valid = tricopterServoCalibrationValidateParams(p);
    tricopterServoCalibrationParams_t copy = {0};
    if (valid) copy = *p;
    memset(s, 0, sizeof(*s));
    s->output = invalidOutput();
    s->params = copy;
    if (!valid) return false;
    s->output.commandedPwmUs = centerPwm(s);
    if (!disarmed) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_ARMED);
        return false;
    }
    s->output.status = TRICOPTER_SERVO_CALIBRATION_OK;
    s->output.failed = false;
    s->output.commandValid = s->output.active = true;
    s->lastStepUs = s->lastFeedbackUs = now;
    enterStage(s, TRICOPTER_SERVO_CALIBRATION_CENTER, now);
    return true;
}

static void sampleTrace(tricopterServoCalibrationState_t *s, uint32_t now, int16_t counts)
{
    if (now - s->lastTraceUs < s->traceIntervalUs) return;
    if (s->traceCount == TRICOPTER_SERVO_CALIBRATION_TRACE_CAPACITY) {
        // Retain the complete move under a fixed memory bound. Coarsening
        // preserves the actual timestamps; it never invents a sample time.
        for (unsigned i = 0; i < TRICOPTER_SERVO_CALIBRATION_TRACE_CAPACITY / 2; ++i) {
            s->traceTimeUs[i] = s->traceTimeUs[2 * i];
            s->traceCounts[i] = s->traceCounts[2 * i];
        }
        s->traceCount /= 2;
        s->traceIntervalUs *= 2;
    }
    s->traceTimeUs[s->traceCount] = now - s->traceStartUs;
    s->traceCounts[s->traceCount++] = counts;
    s->lastTraceUs = now;
}

static bool crossing(const tricopterServoCalibrationState_t *s, float fraction, float *time, unsigned *interval)
{
    const float low = s->output.learnedAdc[0];
    const float span = s->output.learnedAdc[2] - low;
    for (unsigned i = 1; i + 2 < s->traceCount; ++i) {
        const float before = (s->traceCounts[i - 1] - low) / span;
        const float after = (s->traceCounts[i] - low) / span;
        if (before < fraction && after >= fraction
            && (s->traceCounts[i + 1] - low) / span >= fraction
            && (s->traceCounts[i + 2] - low) / span >= fraction) {
            const float part = (fraction - before) / (after - before);
            *time = s->traceTimeUs[i - 1] + part * (s->traceTimeUs[i] - s->traceTimeUs[i - 1]);
            *interval = i;
            return true;
        }
    }
    return false;
}

static float angleAt(const tricopterServoCalibrationState_t *s, float counts)
{
    for (unsigned i = 1; i < 3; ++i) {
        const float a = s->output.learnedAdc[i - 1], b = s->output.learnedAdc[i];
        if (counts >= fminf(a, b) && counts <= fmaxf(a, b)) {
            const float part = (counts - a) / (b - a);
            return s->params.angleDeg[i - 1] + part * (s->params.angleDeg[i] - s->params.angleDeg[i - 1]);
        }
    }
    return s->params.angleDeg[2];
}

static bool completeResult(tricopterServoCalibrationState_t *s)
{
    const int a = s->output.learnedAdc[1] - s->output.learnedAdc[0];
    const int b = s->output.learnedAdc[2] - s->output.learnedAdc[1];
    if (abs(s->output.learnedAdc[2] - s->output.learnedAdc[0]) < 64 || abs(a) < 16 || abs(b) < 16) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_INSUFFICIENT_SPAN);
        return false;
    }
    if (!((a > 0 && b > 0) || (a < 0 && b < 0))) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_NONMONOTONIC);
        return false;
    }
    float t10, t90;
    unsigned interval10, interval90;
    // Both crossings inside one sample gap are an unresolved jump. Linear
    // interpolation across that gap must not manufacture an observed speed.
    if (!crossing(s, .1f, &t10, &interval10) || !crossing(s, .9f, &t90, &interval90)
        || interval90 <= interval10 || t90 - t10 < 1000) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_SPEED_UNAVAILABLE);
        return false;
    }
    const float span = s->output.learnedAdc[2] - s->output.learnedAdc[0];
    const float angle10 = angleAt(s, s->output.learnedAdc[0] + .1f * span);
    const float angle90 = angleAt(s, s->output.learnedAdc[0] + .9f * span);
    const float speed = fabsf(angle90 - angle10) * 1000000 / (t90 - t10);
    if (!finiteFloat(speed) || speed < 1 || speed > 10000) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_SPEED_UNAVAILABLE);
        return false;
    }
    s->output.observedSpeedDegPerSecond = speed;
    s->output.observed10To90Us = (uint32_t)lroundf(t90 - t10);
    s->output.traceSamples = s->traceCount;
    s->output.speedEstimateValid = s->output.complete = s->output.resultValid = true;
    s->output.active = s->output.failed = false;
    s->output.status = TRICOPTER_SERVO_CALIBRATION_OK;
    s->output.stage = TRICOPTER_SERVO_CALIBRATION_COMPLETE;
    s->output.commandedPwmUs = centerPwm(s);
    return true;
}

tricopterServoCalibrationOutput_t tricopterServoCalibrationStep(tricopterServoCalibrationState_t *s,
    uint32_t now, bool armed, bool rawPairValid, int16_t counts)
{
    if (!s) return invalidOutput();
    if (armed) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_ARMED);
        return s->output;
    }
    if (!s->output.active) return s->output;
    if (now - s->lastStepUs >= UINT32_C(0x80000000)) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_INVALID_TIME);
        return s->output;
    }
    s->lastStepUs = now;
    if (now - s->lastFeedbackUs > TRICOPTER_SERVO_CALIBRATION_FEEDBACK_TIMEOUT_US) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_FEEDBACK_TIMEOUT);
        return s->output;
    }
    if (now - s->stageStartUs >= TRICOPTER_SERVO_CALIBRATION_STAGE_TIMEOUT_US) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_STAGE_TIMEOUT);
        return s->output;
    }
    if (!rawPairValid || counts <= -4095 || counts >= 4095) {
        clearAverage(s);
        s->lastRampUs = now; // Do not keep moving, or catch up a ramp, blindly.
        return s->output;
    }
    s->lastFeedbackUs = now;
    const uint16_t target = s->output.stage == TRICOPTER_SERVO_CALIBRATION_LOW ? s->params.pwmUs[0]
        : s->output.stage == TRICOPTER_SERVO_CALIBRATION_HIGH ? s->params.pwmUs[2] : s->params.pwmUs[1];
    const uint32_t ticks = (now - s->lastRampUs) / TRICOPTER_SERVO_CALIBRATION_SAMPLE_US;
    if (ticks) {
        const unsigned distance = target > s->output.commandedPwmUs ? target - s->output.commandedPwmUs : s->output.commandedPwmUs - target;
        const unsigned change = 20U * ticks < distance ? 20U * ticks : distance;
        s->output.commandedPwmUs = (uint16_t)(target > s->output.commandedPwmUs ? s->output.commandedPwmUs + change : s->output.commandedPwmUs - change);
        s->lastRampUs += ticks * TRICOPTER_SERVO_CALIBRATION_SAMPLE_US;
    }
    if (now - s->lastSampleUs < TRICOPTER_SERVO_CALIBRATION_SAMPLE_US) return s->output;
    s->lastSampleUs = now;
    if (s->output.stage == TRICOPTER_SERVO_CALIBRATION_HIGH) sampleTrace(s, now, counts);
    if (now - s->stageStartUs < TRICOPTER_SERVO_CALIBRATION_SETTLE_US || s->output.commandedPwmUs != target) return s->output;
    if (!s->averageActive) {
        clearAverage(s);
        s->averageActive = true;
        s->averageStartUs = now;
    }
    s->averageSum += counts;
    ++s->averageSamples;
    if (counts < s->averageMin) s->averageMin = counts;
    if (counts > s->averageMax) s->averageMax = counts;
    if (now - s->averageStartUs < TRICOPTER_SERVO_CALIBRATION_AVERAGE_US / 2) {
        s->firstHalfSum += counts;
        ++s->firstHalfSamples;
    } else {
        s->secondHalfSum += counts;
        ++s->secondHalfSamples;
    }
    if (now - s->averageStartUs < TRICOPTER_SERVO_CALIBRATION_AVERAGE_US) return s->output;
    const bool stable = s->averageSamples >= 20 && s->firstHalfSamples >= 5 && s->secondHalfSamples >= 5
        && s->averageMax - s->averageMin <= 64
        && fabsf((float)s->firstHalfSum / s->firstHalfSamples - (float)s->secondHalfSum / s->secondHalfSamples) <= 8;
    if (!stable) {
        clearAverage(s);
        return s->output;
    }
    const int16_t mean = (int16_t)lroundf((float)s->averageSum / s->averageSamples);
    switch (s->output.stage) {
    case TRICOPTER_SERVO_CALIBRATION_CENTER:
        s->output.initialCenterAdc = s->output.learnedAdc[1] = mean;
        enterStage(s, TRICOPTER_SERVO_CALIBRATION_LOW, now);
        break;
    case TRICOPTER_SERVO_CALIBRATION_LOW:
        s->output.learnedAdc[0] = mean;
        enterStage(s, TRICOPTER_SERVO_CALIBRATION_HIGH, now);
        break;
    case TRICOPTER_SERVO_CALIBRATION_HIGH:
        s->output.learnedAdc[2] = mean;
        enterStage(s, TRICOPTER_SERVO_CALIBRATION_RETURN_CENTER, now);
        break;
    case TRICOPTER_SERVO_CALIBRATION_RETURN_CENTER:
        s->output.learnedAdc[1] = mean;
        completeResult(s);
        break;
    default:
        stop(s, TRICOPTER_SERVO_CALIBRATION_INVALID_PARAMS);
        break;
    }
    return s->output;
}

tricopterServoCalibrationOutput_t tricopterServoCalibrationAbort(tricopterServoCalibrationState_t *s)
{
    if (!s) return invalidOutput();
    // A completed result belongs to the same cancellable session as an active
    // move. Cancellation must prevent a later apply of that stale result.
    if (s->output.status != TRICOPTER_SERVO_CALIBRATION_ARMED
        && tricopterServoCalibrationValidateParams(&s->params)) {
        stop(s, TRICOPTER_SERVO_CALIBRATION_USER_ABORT);
    }
    return s->output;
}

const tricopterServoCalibrationOutput_t *tricopterServoCalibrationGetOutput(const tricopterServoCalibrationState_t *s)
{
    return s ? &s->output : NULL;
}
