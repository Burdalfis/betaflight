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

#pragma once

#include "platform.h"

#include "pg/pg.h"
#include "common/time.h"
#include "flight/tricopter_control.h"
#include "flight/tricopter_servo_calibration.h"

typedef struct tricopterMixerConfig_s {
    int16_t angleDeciDeg[3]; // Angle at servo5 min/middle/max PWM.
    int16_t adcCounts[3];    // Signed PB0-PC0 at those same three angles.
    uint16_t yawGainPermille;
    uint16_t positionGainDeciUsPerDeg;
    uint16_t feedbackTimeoutMs;
    uint16_t adcMarginCounts;
    int16_t neutralTrimDeciDeg;
    int8_t yawDirection;
    uint16_t observedServoSpeedDps; // Optional calibration estimate, not prediction.
} tricopterMixerConfig_t;

PG_DECLARE(tricopterMixerConfig_t, tricopterMixerConfig);

bool mixerTricopterIsServoSaturated(float errorRate);
void mixerTricopterInit(void);
// Correction is added AFTER the common vertical-effort scale.
float mixerTricopterMotorCorrection(int motor);
float mixerTricopterCollectiveScale(void);
float mixerTricopterApplyMotorEffort(int motor, float verticalEffort);
// Motor inputs are normalized pre-thrust-linearization effort. Yaw is the
// existing un-reversed yaw PID sum / PID_MIXER_SCALING.
void mixerTricopterUpdate(timeUs_t now, float tailVertical, float normalizedYaw);
uint16_t mixerTricopterServoPwm(void);
bool mixerTricopterCanArm(void);
const tricopterControlOutput_t *mixerTricopterGetControl(void);
bool mixerTricopterCalibrationStart(timeUs_t now);
void mixerTricopterCalibrationAbort(void);
bool mixerTricopterCalibrationApply(void);
bool mixerTricopterIsCalibrating(void);
const tricopterServoCalibrationOutput_t *mixerTricopterGetCalibration(void);
