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

void servosTricopterLog(int16_t finalPwm)
{
#ifdef USE_ADC
    static bool logging;
#endif
    if (debugMode != DEBUG_TRI_SERVO) {
#ifdef USE_ADC
        if (logging) {
            adcResetDiagnosticSnapshot();
        }
        logging = false;
#endif
        return;
    }
#ifdef USE_ADC
    logging = true;
#endif

#if defined(USE_ADC) || defined(USE_DSHOT_TELEMETRY)
    const timeUs_t now = micros();
#endif
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
