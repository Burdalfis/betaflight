/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "tricopter_servo_model.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "The tricopter servo model requires IEEE-754 binary32 float"
#endif

_Static_assert(sizeof(float) == sizeof(uint32_t), "The tricopter servo model requires 32-bit float");

// Betaflight may compile with -ffast-math, whose finite-math-only assumption
// can remove isfinite(). Inspect the representation so fault validation also
// works under the eventual firmware's normal optimization flags.
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
    return finiteFloat(value) && value >= 0.0f && value <= 60.0f
        && (value == 0.0f || value >= 0.000001f);
}

static bool validRate(float value)
{
    return finiteFloat(value) && value >= 0.0f && value <= 1e9f;
}

bool tricopterServoModelValidateParams(const tricopterServoModelParams_t *params)
{
    if (!params || params->knotCount < 2 || params->knotCount > TRICOPTER_SERVO_MODEL_MAX_KNOTS
        || !params->maxGapUs || !finiteFloat(params->backlashUs) || params->backlashUs < 0.0f
        || !validTau(params->tauIncreasingPwmS) || !validTau(params->tauDecreasingPwmS)
        || !validRate(params->rateIncreasingPwmCountsS) || !validRate(params->rateDecreasingPwmCountsS)) {
        return false;
    }
    int countsDirection = 0;
    for (unsigned i = 0; i < params->knotCount; ++i) {
        if (!finiteFloat(params->pwmUs[i]) || params->pwmUs[i] <= 0.0f || params->pwmUs[i] > 65535.0f
            || !finiteFloat(params->positionCounts[i]) || fabsf(params->positionCounts[i]) > 4095.0f) {
            return false;
        }
        if (i) {
            if (params->pwmUs[i] <= params->pwmUs[i - 1]) {
                return false;
            }
            const float countsChange = params->positionCounts[i] - params->positionCounts[i - 1];
            const int direction = countsChange > 0.0f ? 1 : (countsChange < 0.0f ? -1 : 0);
            if (direction && countsDirection && direction != countsDirection) {
                return false;
            }
            if (direction) {
                countsDirection = direction;
            }
        }
    }
    return countsDirection != 0 && params->backlashUs <= params->pwmUs[params->knotCount - 1] - params->pwmUs[0];
}

static bool validPwm(const tricopterServoModelParams_t *params, float pwmUs)
{
    return finiteFloat(pwmUs) && pwmUs >= params->pwmUs[0] && pwmUs <= params->pwmUs[params->knotCount - 1];
}

static float equilibriumCounts(const tricopterServoModelParams_t *params, float pwmUs)
{
    for (unsigned i = 1; i < params->knotCount; ++i) {
        if (pwmUs <= params->pwmUs[i]) {
            const float fraction = (pwmUs - params->pwmUs[i - 1]) / (params->pwmUs[i] - params->pwmUs[i - 1]);
            return params->positionCounts[i - 1] + fraction * (params->positionCounts[i] - params->positionCounts[i - 1]);
        }
    }
    return params->positionCounts[params->knotCount - 1];
}

static void dynamics(const tricopterServoModelState_t *state, float *tau, float *rate)
{
    const tricopterServoModelParams_t *params = &state->params;
    const bool increasingCounts = params->positionCounts[params->knotCount - 1] > params->positionCounts[0];
    const bool increasingPwm = (state->targetCounts > state->positionCounts) == increasingCounts;
    *tau = increasingPwm ? params->tauIncreasingPwmS : params->tauDecreasingPwmS;
    *rate = increasingPwm ? params->rateIncreasingPwmCountsS : params->rateDecreasingPwmCountsS;
}

static void updateVelocity(tricopterServoModelState_t *state)
{
    const float error = state->targetCounts - state->positionCounts;
    if (error == 0.0f) {
        state->velocityCountsS = 0.0f;
        return;
    }
    float tau, rate;
    dynamics(state, &tau, &rate);
    float speed = tau > 0.0f ? fabsf(error) / tau : rate;
    if (rate > 0.0f && speed > rate) {
        speed = rate;
    }
    state->velocityCountsS = error < 0.0f ? -speed : speed;
}

static void advanceTo(tricopterServoModelState_t *state, uint64_t nowUs)
{
    const float dt = (float)(nowUs - state->timeUs) * 0.000001f;
    const float error = state->targetCounts - state->positionCounts;
    float tau, rate;
    dynamics(state, &tau, &rate);
    const float distance = fabsf(error);
    float remaining = distance;
    if (tau == 0.0f) {
        remaining = rate == 0.0f ? 0.0f : fmaxf(0.0f, distance - rate * dt);
    } else if (rate == 0.0f || distance <= rate * tau) {
        remaining = distance * expf(-dt / tau);
    } else {
        // Exact solution of dx/dt=clamp((target-x)/tau, -rate, rate):
        // a constant-rate prefix followed by an exponential tail.
        const float linearTime = (distance - rate * tau) / rate;
        remaining = dt <= linearTime ? distance - rate * dt : rate * tau * expf(-(dt - linearTime) / tau);
    }
    state->positionCounts = state->targetCounts - copysignf(remaining, error);
    state->timeUs = nowUs;
    updateVelocity(state);
}

static void applyCommand(tricopterServoModelState_t *state, float pwmUs)
{
    const float halfPlay = state->params.backlashUs * 0.5f;
    state->playedPwmUs = clampFloat(state->playedPwmUs, pwmUs - halfPlay, pwmUs + halfPlay);
    state->playedPwmUs = clampFloat(state->playedPwmUs, state->params.pwmUs[0], state->params.pwmUs[state->params.knotCount - 1]);
    state->targetCounts = equilibriumCounts(&state->params, state->playedPwmUs);
    // Static-map baselines act at the event boundary, including zero dt.
    advanceTo(state, state->timeUs);
}

static tricopterServoModelOutput_t output(const tricopterServoModelState_t *state)
{
    if (!state) {
        return (tricopterServoModelOutput_t) { .status = TRICOPTER_SERVO_MODEL_NOT_INITIALIZED };
    }
    return (tricopterServoModelOutput_t) {
        .positionCounts = state->positionCounts,
        .velocityCountsS = state->velocityCountsS,
        .status = state->initialized ? state->status : (state->status | TRICOPTER_SERVO_MODEL_NOT_INITIALIZED),
        .valid = state->initialized && state->status == TRICOPTER_SERVO_MODEL_OK,
        .angleCalibrated = false,
    };
}

static tricopterServoModelOutput_t fail(tricopterServoModelState_t *state, uint32_t status)
{
    state->status |= status;
    state->velocityCountsS = 0.0f;
    return output(state);
}

bool tricopterServoModelReset(tricopterServoModelState_t *state, const tricopterServoModelParams_t *params,
    uint64_t nowUs, float initialPwmUs, float initialCorrectedCounts, int initialDirection)
{
    if (!state) {
        return false;
    }
    // A caller may deliberately reset using the state's retained parameters.
    // Preserve that value before clearing history and any previous fault.
    tricopterServoModelParams_t retainedParams;
    if (params) {
        retainedParams = *params;
        params = &retainedParams;
    }
    memset(state, 0, sizeof(*state));
    if (!tricopterServoModelValidateParams(params)) {
        state->status = TRICOPTER_SERVO_MODEL_INVALID_CONFIG;
        return false;
    }
    if (!validPwm(params, initialPwmUs)) {
        state->status = TRICOPTER_SERVO_MODEL_INVALID_PWM;
        return false;
    }
    if (!finiteFloat(initialCorrectedCounts) || fabsf(initialCorrectedCounts) > 4095.0f) {
        state->status = TRICOPTER_SERVO_MODEL_INVALID_POSITION;
        return false;
    }
    if (initialDirection < -1 || initialDirection > 1) {
        state->status = TRICOPTER_SERVO_MODEL_INVALID_DIRECTION;
        return false;
    }
    state->params = *params;
    state->timeUs = nowUs;
    state->lastCommandPwmUs = initialPwmUs;
    state->playedPwmUs = clampFloat(initialPwmUs - (float)initialDirection * params->backlashUs * 0.5f,
        params->pwmUs[0], params->pwmUs[params->knotCount - 1]);
    state->targetCounts = equilibriumCounts(params, state->playedPwmUs);
    state->positionCounts = initialCorrectedCounts;
    state->initialized = true;
    return true;
}

tricopterServoModelOutput_t tricopterServoModelStep(tricopterServoModelState_t *state, uint64_t nowUs, float commandPwmUs)
{
    if (!state || !state->initialized || state->status != TRICOPTER_SERVO_MODEL_OK) {
        return output(state);
    }
    if (!validPwm(&state->params, commandPwmUs)) {
        return fail(state, TRICOPTER_SERVO_MODEL_INVALID_PWM);
    }
    if (nowUs < state->timeUs || (commandPwmUs != state->lastCommandPwmUs && nowUs > UINT64_MAX - state->params.delayUs)) {
        return fail(state, TRICOPTER_SERVO_MODEL_INVALID_TIME);
    }
    if (nowUs - state->timeUs > state->params.maxGapUs) {
        return fail(state, TRICOPTER_SERVO_MODEL_COMMAND_GAP);
    }
    while (state->commandCount && state->commands[state->commandHead].dueUs <= nowUs) {
        const tricopterServoModelCommand_t command = state->commands[state->commandHead];
        advanceTo(state, command.dueUs);
        applyCommand(state, command.pwmUs);
        state->commandHead = (state->commandHead + 1U) % TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY;
        --state->commandCount;
    }
    advanceTo(state, nowUs);
    if (commandPwmUs != state->lastCommandPwmUs) {
        if (state->commandCount == TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY) {
            return fail(state, TRICOPTER_SERVO_MODEL_QUEUE_FULL);
        }
        const unsigned tail = (state->commandHead + state->commandCount) % TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY;
        state->commands[tail] = (tricopterServoModelCommand_t) { .dueUs = nowUs + state->params.delayUs, .pwmUs = commandPwmUs };
        ++state->commandCount;
        state->lastCommandPwmUs = commandPwmUs;
        if (state->params.delayUs == 0) {
            applyCommand(state, commandPwmUs);
            state->commandHead = (state->commandHead + 1U) % TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY;
            --state->commandCount;
        }
    }
    return output(state);
}
