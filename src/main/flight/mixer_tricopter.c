/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERVOS
#include "common/maths.h"
#include "common/filter.h"
#include "drivers/time.h"
#include "common/utils.h"
#include "config/feature.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/servos.h"
#include "flight/tricopter_servo_position.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(tricopterMixerConfig_t, tricopterMixerConfig, PG_TRICOPTER_CONFIG, 1);
PG_RESET_TEMPLATE(tricopterMixerConfig_t, tricopterMixerConfig,
    .angleDeciDeg = { -478, 0, 216 },
    .adcCounts = { 1936, 1490, 1281 },
    .yawGainPermille = 700,
    .positionGainDeciUsPerDeg = 20,
    .feedbackTimeoutMs = 20,
    .adcMarginCounts = 32,
    .neutralTrimDeciDeg = 0,
    .yawDirection = 1,
    .observedServoSpeedDps = 0
);

static tricopterControlOutput_t control;
static tricopterServoCalibrationState_t calibration;
static tricopterServoCalibrationOutput_t calibrationOutput;
static float motorCorrection;
static uint16_t servoPwm;
static bool calibrationSession;
static bool returningToCenter;
static timeUs_t centerReturnStarted;

static void requestCenterReturn(timeUs_t now)
{
    returningToCenter = !ARMING_FLAG(ARMED);
    centerReturnStarted = now;
}

static bool calibrationMatchesParams(const tricopterControlParams_t *p)
{
    for (unsigned i = 0; i < 3; ++i) {
        if (calibration.params.pwmUs[i] != p->pwmUs[i] || calibration.params.angleDeg[i] != p->angleDeg[i]) return false;
    }
    return true;
}

static tricopterControlParams_t controlParams(void)
{
    const tricopterMixerConfig_t *cfg = tricopterMixerConfig();
    const servoParam_t *s = servoParams(SERVO_RUDDER);
    tricopterControlParams_t p = tricopterControlDefaultParams();
    p.pwmUs[0] = s->min; p.pwmUs[1] = s->middle; p.pwmUs[2] = s->max;
    for (unsigned i = 0; i < 3; ++i) {
        p.angleDeg[i] = cfg->angleDeciDeg[i] * 0.1f;
        p.adcCounts[i] = cfg->adcCounts[i];
    }
    p.pwmMinUs = s->min; p.pwmMaxUs = s->max;
    p.positionGainUsPerDeg = cfg->positionGainDeciUsPerDeg * 0.1f;
    p.yawGain = cfg->yawGainPermille * 0.001f;
    p.yawDirection = cfg->yawDirection;
    // Preserve the existing rudder output reversal conventions, without
    // coupling physical position calibration to propeller rotation.
    if (s->rate < 0) p.yawDirection = -p.yawDirection;
    if (s->reversedSources & (1U << INPUT_STABILIZED_YAW)) p.yawDirection = -p.yawDirection;
    p.neutralTrimDeg = cfg->neutralTrimDeciDeg * 0.1f;
    p.feedbackTimeoutUs = (uint32_t)cfg->feedbackTimeoutMs * 1000U;
    p.adcMarginCounts = cfg->adcMarginCounts;
    return p;
}

static bool rawPositionValid(const tricopterServoPositionSample_t *s)
{
    return s->supported && s->ready && s->sampleSeen
        && s->rawWiperCounts > 0 && s->rawWiperCounts < 4095
        && s->rawGroundCounts < 4095
        && s->correctedCounts != TRICOPTER_SERVO_POSITION_UNAVAILABLE;
}

static bool motorsStopped(void)
{
    if (mixerRuntime.motorCount < 3) return false;
    for (unsigned i = 0; i < mixerRuntime.motorCount; ++i) {
        if (motor[i] != mixerRuntime.disarmMotorOutput || motor_disarmed[i] != mixerRuntime.disarmMotorOutput) return false;
    }
    return true;
}

void mixerTricopterInit(void)
{
    memset(&control, 0, sizeof(control));
    memset(&calibration, 0, sizeof(calibration));
    memset(&calibrationOutput, 0, sizeof(calibrationOutput));
    motorCorrection = 0;
    servoPwm = servoParams(SERVO_RUDDER)->middle;
    calibrationSession = returningToCenter = false;
}

void mixerTricopterUpdate(timeUs_t now, float tailVertical, float normalizedYaw)
{
    const bool armed = ARMING_FLAG(ARMED);
    const tricopterServoPositionSample_t position = servosTricopterGetPosition(now);
    const tricopterControlParams_t p = controlParams();
    const servoParam_t *s = servoParams(SERVO_RUDDER);
    const tricopterControlInput_t input = {
        .verticalEffort = constrainf(tailVertical, 0.0f, 1.0f),
        .normalizedYaw = normalizedYaw * (abs(s->rate) * 0.01f),
        .correctedCounts = position.correctedCounts,
        .feedbackAgeUs = position.ageUs,
        .feedbackValid = rawPositionValid(&position),
    };
    control = tricopterControlEvaluate(&p, &input);
    servoPwm = control.validConfig && control.validInput ? lrintf(control.pwmUs) : s->middle;
    servoPwm = constrain(servoPwm, s->min, s->max);
    motorCorrection = control.validConfig && control.validInput ? control.tailEffort - tailVertical : 0;

    if (returningToCenter && (armed || (uint32_t)(now - centerReturnStarted) >= 300000U)) returningToCenter = false;
    if (calibrationSession && !calibrationMatchesParams(&p)) {
        calibrationOutput = tricopterServoCalibrationAbort(&calibration);
        calibrationSession = false;
        requestCenterReturn(now);
    }
    if (calibrationSession) {
        const bool stopped = !armed && motorsStopped();
        if (calibrationOutput.active || !stopped) {
            calibrationOutput = tricopterServoCalibrationStep(&calibration, now, !stopped,
                rawPositionValid(&position) && position.ageUs <= p.feedbackTimeoutUs, position.correctedCounts);
        }
        if (calibrationOutput.failed || !stopped) {
            calibrationSession = false;
            requestCenterReturn(now);
        }
        if (!armed) {
            servoPwm = constrain(calibrationOutput.commandedPwmUs, s->min, s->max);
            motorCorrection = 0;
        }
    }
    if (!armed && (returningToCenter || (!calibrationSession && s->rate == 0))) {
        // Retain the existing disarmed fixed-output bench-test convention.
        servoPwm = constrain(s->middle, s->min, s->max);
        motorCorrection = 0;
    }
}

float mixerTricopterMotorCorrection(int motor)
{
    return motor == 0 ? motorCorrection : 0.0f;
}

uint16_t mixerTricopterServoPwm(void) { return servoPwm; }
const tricopterControlOutput_t *mixerTricopterGetControl(void) { return &control; }
bool mixerTricopterIsServoSaturated(float errorRate)
{
    UNUSED(errorRate);
    return control.yawSaturated || control.verticalSaturated || !control.feedbackValid;
}

bool mixerTricopterCanArm(void)
{
    return control.validConfig && control.validInput && control.feedbackValid
        && !mixerTricopterIsCalibrating() && servoParams(SERVO_RUDDER)->rate != 0
        && (uint8_t)servoParams(SERVO_RUDDER)->forwardFromChannel == CHANNEL_FORWARDING_DISABLED;
}

bool mixerTricopterCalibrationStart(timeUs_t now)
{
    if (!mixerIsTricopter() || ARMING_FLAG(ARMED) || !motorsStopped() || mixerTricopterIsCalibrating()) return false;
    const tricopterControlParams_t p = controlParams();
    tricopterServoCalibrationParams_t params;
    for (unsigned i = 0; i < 3; ++i) { params.pwmUs[i] = lrintf(p.pwmUs[i]); params.angleDeg[i] = p.angleDeg[i]; }
    if (!tricopterServoCalibrationStart(&calibration, &params, now, true)) return false;
    calibrationSession = true;
    calibrationOutput = tricopterServoCalibrationStep(&calibration, now, false, false, 0);
    return calibrationOutput.active;
}

void mixerTricopterCalibrationAbort(void)
{
    calibrationOutput = tricopterServoCalibrationAbort(&calibration);
    calibrationSession = false;
    requestCenterReturn(micros());
    if (!ARMING_FLAG(ARMED)) servoPwm = servoParams(SERVO_RUDDER)->middle;
}

bool mixerTricopterCalibrationApply(void)
{
    const tricopterControlParams_t p = controlParams();
    if (ARMING_FLAG(ARMED) || !calibrationSession || !calibrationOutput.complete
        || !calibrationOutput.resultValid || calibrationOutput.active || !calibrationMatchesParams(&p)) return false;
    for (unsigned i = 0; i < 3; ++i) tricopterMixerConfigMutable()->adcCounts[i] = calibrationOutput.learnedAdc[i];
    if (calibrationOutput.speedEstimateValid) {
        tricopterMixerConfigMutable()->observedServoSpeedDps = constrain(lrintf(calibrationOutput.observedSpeedDegPerSecond), 1, 2000);
    }
    calibrationSession = false;
    returningToCenter = false;
    return true;
}

bool mixerTricopterIsCalibrating(void) { return calibrationSession || returningToCenter; }
const tricopterServoCalibrationOutput_t *mixerTricopterGetCalibration(void) { return &calibrationOutput; }
#endif
