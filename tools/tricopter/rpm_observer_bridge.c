/* Offline batch adapter. Every estimate comes from the real portable C core. */
#include <stddef.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#include "flight/tricopter_rpm_observer.h"

int tricopterRpmReplay(const double *values, size_t count, const uint32_t *nowUs,
    const uint32_t *observedUs, const uint32_t *sequence, const uint8_t *seen,
    const float *mechanicalRpm, float *estimatedRpm, float *accelerationRpmS,
    uint32_t *ageUs, uint32_t *outputSequence, uint32_t *status,
    uint8_t *outputSeen, uint8_t *accepted, uint8_t *fresh, uint8_t *valid)
{
    if (!values || !count || count > SIZE_MAX / 3 || !nowUs || !observedUs || !sequence || !seen
        || !mechanicalRpm || !estimatedRpm || !accelerationRpmS || !ageUs || !outputSequence
        || !status || !outputSeen || !accepted || !fresh || !valid) {
        return 0;
    }
    for (unsigned i = 0; i < 7; ++i) {
        if (!isfinite(values[i]) || (i < 4 && fabs(values[i]) > FLT_MAX)
            || (i >= 4 && (values[i] < 0 || values[i] > UINT32_MAX || values[i] != floor(values[i])))) {
            return 0;
        }
    }
    const tricopterRpmObserverParams_t params = {
        .trackingTauS = (float)values[0],
        .staleAccelerationDecayTauS = (float)values[1],
        .maxRpm = (float)values[2],
        .maxAccelerationRpmS = (float)values[3],
        .staleAfterUs = (uint32_t)values[4],
        .maxPredictionUs = (uint32_t)values[5],
        .maxCallGapUs = (uint32_t)values[6],
    };
    tricopterRpmObserverState_t states[3];
    for (unsigned motor = 0; motor < 3; ++motor) {
        if (!tricopterRpmObserverReset(&states[motor], &params, nowUs[0])) {
            return 0;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        for (unsigned motor = 0; motor < 3; ++motor) {
            const size_t index = 3 * i + motor;
            const tricopterRpmObservation_t observation = {
                .seen = seen[index] != 0,
                .observedAtUs = observedUs[index],
                .sequence = sequence[index],
                .mechanicalRpm = mechanicalRpm[index],
            };
            const tricopterRpmObserverOutput_t output = tricopterRpmObserverStep(&states[motor], nowUs[i], &observation);
            estimatedRpm[index] = output.mechanicalRpm;
            accelerationRpmS[index] = output.accelerationRpmS;
            ageUs[index] = output.ageUs;
            outputSequence[index] = output.sequence;
            status[index] = output.status;
            outputSeen[index] = output.seen;
            accepted[index] = output.acceptedNewObservation;
            fresh[index] = output.fresh;
            valid[index] = output.valid;
        }
    }
    return 1;
}
