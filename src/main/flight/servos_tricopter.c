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

#include "platform.h"

#ifdef USE_SERVOS

#include "build/debug.h"
#include "drivers/time.h"
#ifdef USE_ADC
#include "drivers/adc.h"
#endif
#ifdef USE_DSHOT_TELEMETRY
#include "drivers/dshot.h"
#endif

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/servos.h"

// -1 means unavailable; nonnegative ages use 100 us units and saturate at
// 3.2767 seconds. These diagnostics never feed back into the legacy mixer.
#if defined(USE_ADC) || defined(USE_DSHOT_TELEMETRY)
static int16_t diagnosticAge(uint32_t ageUs)
{
    if (ageUs == UINT32_MAX) {
        return -1;
    }
    const uint32_t age = ageUs / 100;
    return age > INT16_MAX ? INT16_MAX : age;
}
#endif

static bool isTricopterDiagnosticMode(uint8_t mode)
{
    return mode == DEBUG_TRI_SERVO || mode == DEBUG_TRI_GROUND;
}

tricopterServoPositionSample_t servosTricopterGetPosition(timeUs_t currentTimeUs)
{
    tricopterServoPositionSample_t position = {
        .correctedCounts = TRICOPTER_SERVO_POSITION_UNAVAILABLE,
        .ageUs = UINT32_MAX,
    };
#ifdef USE_ADC
    // Fixed measurement path: both configured channels form one coherent scan
    // pair. Never replace a missing ground reference with raw PB0 or zero.
    const adcDiagnosticPairSnapshot_t pair = adcGetDiagnosticPairSnapshot(ADC_EXTERNAL1, ADC_RSSI, currentTimeUs);
    position.supported = pair.supported;
    position.ready = pair.ready;
    position.sampleSeen = pair.sampleSeen;
    position.ageUs = pair.sampleSeen ? pair.ageUs : UINT32_MAX;
    if (pair.sampleSeen) {
        position.rawWiperCounts = pair.firstValue;
        position.rawGroundCounts = pair.secondValue;
        if (pair.supported && pair.firstValue <= 4095 && pair.secondValue <= 4095) {
            position.correctedCounts = (int32_t)pair.firstValue - (int32_t)pair.secondValue;
            position.fresh = pair.ready && pair.ageUs <= TRICOPTER_SERVO_POSITION_MAX_AGE_US;
        }
    }
#else
    (void)currentTimeUs;
#endif
    return position;
}

static void logTricopterGround(int16_t finalPwm, timeUs_t now)
{
    (void)now;
    uint16_t flags = 0;
    debug[0] = -1;             // Raw wiper (ADC_EXTERNAL1 / PB0).
    debug[1] = finalPwm;
    debug[2] = -1;             // Raw local servo ground (ADC_RSSI / PC0).
    debug[3] = INT16_MIN;      // Signed wiper minus ground; -1 is valid data.
    debug[4] = -1;             // Coherent pair observation age, 100 us/count.
    debug[6] = -1;             // Worst RPM age, only if all three have samples.
    debug[7] = -1;             // Existing worst invalid-packet percentage.

#ifdef USE_ADC
    const tricopterServoPositionSample_t position = servosTricopterGetPosition(now);
    if (position.supported) {
        flags |= 1 << 0;
    }
    if (position.ready) {
        flags |= 1 << 1;
    }
    if (position.sampleSeen) {
        flags |= 1 << 2;
        debug[0] = position.rawWiperCounts;
        debug[2] = position.rawGroundCounts;
        debug[3] = position.correctedCounts;
        debug[4] = diagnosticAge(position.ageUs);
        if (position.fresh) {
            flags |= 1 << 3;
        }
        if (position.rawWiperCounts == 0 || position.rawWiperCounts == 4095) {
            flags |= 1 << 8;
        }
        // A zero ground reading is expected, but cannot show negative voltage.
        if (position.rawGroundCounts == 0) {
            flags |= 1 << 9;
        }
        if (position.rawGroundCounts == 4095) {
            flags |= 1 << 10;
        }
    }
#endif

#ifdef USE_DSHOT_TELEMETRY
    bool allRpmSeen = useDshotTelemetry && dshotMotorCount >= 3;
    uint32_t worstRpmAgeUs = 0;
    if (useDshotTelemetry) {
        flags |= 1 << 4;
    }
    for (unsigned motor = 0; motor < 3; motor++) {
        const dshotTelemetryRpmSample_t sample = getDshotTelemetryRpmSample(motor, now);
        if (sample.seen) {
            flags |= 1 << (5 + motor);
            if (sample.ageUs > worstRpmAgeUs) {
                worstRpmAgeUs = sample.ageUs;
            }
        } else {
            allRpmSeen = false;
        }
#ifdef USE_DSHOT_TELEMETRY_STATS
        if (useDshotTelemetry && motor < dshotMotorCount) {
            const int16_t invalidPercent = getDshotTelemetryMotorInvalidPercent(motor);
            if (invalidPercent > debug[7]) {
                debug[7] = invalidPercent;
            }
        }
#endif
    }
    if (allRpmSeen) {
        debug[6] = diagnosticAge(worstRpmAgeUs);
    }
#endif
    debug[5] = flags;
}

void servosTricopterLog(int16_t finalPwm)
{
#ifdef USE_ADC
    static uint8_t previousMode = DEBUG_NONE;
    if (previousMode != debugMode) {
        if (isTricopterDiagnosticMode(previousMode)) {
            adcResetDiagnosticSnapshot();
        }
        previousMode = debugMode;
    }
#endif
    if (!isTricopterDiagnosticMode(debugMode)) {
        return;
    }

#if defined(USE_ADC) || defined(USE_DSHOT_TELEMETRY)
    const timeUs_t now = micros();
#else
    const timeUs_t now = 0;
#endif
    if (debugMode == DEBUG_TRI_GROUND) {
        logTricopterGround(finalPwm, now);
        return;
    }

    uint16_t flags = 0;
    debug[0] = -1;
    debug[1] = finalPwm;
    debug[2] = -1;
    debug[7] = -1;
#ifdef USE_ADC
    const adcDiagnosticSnapshot_t adc = adcGetDiagnosticSnapshot(ADC_EXTERNAL1, now);
    if (adc.supported) {
        flags |= 1 << 0;
    }
    if (adc.ready) {
        flags |= 1 << 1;
        debug[0] = adc.value;
    }
    if (adc.sampleSeen) {
        flags |= 1 << 2;
        debug[2] = diagnosticAge(adc.ageUs);
        // Non-rail is a plausibility indication, not calibrated validity.
        if (adc.value > 0 && adc.value < 4095) {
            flags |= 1 << 3;
        }
    }
#endif

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        flags |= 1 << 4;
    }
#endif
    for (unsigned motor = 0; motor < 3; motor++) {
        debug[4 + motor] = -1;
#ifdef USE_DSHOT_TELEMETRY
        const dshotTelemetryRpmSample_t sample = getDshotTelemetryRpmSample(motor, now);
        if (sample.seen) {
            flags |= 1 << (5 + motor);
            debug[4 + motor] = diagnosticAge(sample.ageUs);
        }
#ifdef USE_DSHOT_TELEMETRY_STATS
        if (useDshotTelemetry && motor < dshotMotorCount) {
            const int16_t invalidPercent = getDshotTelemetryMotorInvalidPercent(motor);
            if (invalidPercent > debug[7]) {
                debug[7] = invalidPercent;
            }
        }
#endif
#endif
    }
    debug[3] = flags;
}

bool servosTricopterIsEnabledServoUnarmed(void)
{
    return servoConfig()->tri_unarmed_servo;
}

void servosTricopterMixer(void)
{
    servoMixer();
}

void servosTricopterInit(void)
{

}

#endif // USE_SERVOS
