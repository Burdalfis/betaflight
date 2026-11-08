/* Virtual coherent ADC pair. This file is linked only into the SITL platform;
 * the real flight controller continues to use its unchanged hardware backend. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "platform.h"

#ifdef USE_ADC
#include "drivers/adc.h"
#include "drivers/time.h"
#include "pg/adc.h"
#include "sitl_adc.h"
#include "udplink.h"

static udpLink_t adcLink = { .fd = -1 };
static bool adcReady;
static bool channelEnabled[ADC_SOURCE_COUNT];
static sitlAdcState_t adcState;

static bool positionSource(adcSource_e source)
{
    return source == ADC_EXTERNAL1 || source == ADC_RSSI;
}

static void receiveAdc(timeUs_t observedAt)
{
    if (!adcReady) return;
    /* One extra byte detects oversized UDP datagrams rather than accepting a
     * truncated valid prefix. Bound draining so an input flood cannot monopolize
     * the flight loop. No worker thread writes the acquisition state. */
    uint8_t bytes[SITL_ADC_PACKET_SIZE + 1];
    for (unsigned i = 0; i < 32; ++i) {
        const int size = udpRecv(&adcLink, bytes, sizeof(bytes), 0);
        if (size < 0) break;
        sitlAdcAccept(&adcState, bytes, (size_t)size, observedAt);
    }
}

void adcInit(const adcConfig_t *config)
{
    if (adcLink.fd >= 0) close(adcLink.fd);
    memset(&adcState, 0, sizeof(adcState));
    memset(channelEnabled, 0, sizeof(channelEnabled));
    channelEnabled[ADC_BATTERY] = config->vbat.enabled;
    channelEnabled[ADC_CURRENT] = config->current.enabled;
    channelEnabled[ADC_EXTERNAL1] = config->external1.enabled;
    channelEnabled[ADC_RSSI] = config->rssi.enabled;
    adcReady = udpInit(&adcLink, "127.0.0.1", SITL_ADC_PORT, true) == 0;
    if (!adcReady && adcLink.fd >= 0) {
        close(adcLink.fd);
        adcLink.fd = -1;
    }
    printf("[SITL] virtual PB0/PC0 ADC on 127.0.0.1:%d: %s\n",
        SITL_ADC_PORT, adcReady ? "ready" : "unavailable");
}

uint16_t adcGetValue(adcSource_e source)
{
    receiveAdc(micros());
    if (!positionSource(source) || !channelEnabled[source] || !adcState.seen) return 0;
    return source == ADC_EXTERNAL1 ? adcState.sample.pb0 : adcState.sample.pc0;
}

adcDiagnosticSnapshot_t adcGetDiagnosticSnapshot(adcSource_e source, timeUs_t now)
{
    receiveAdc(now);
    adcDiagnosticSnapshot_t result = { .ageUs = UINT32_MAX };
    if (!positionSource(source)) return result;
    result.supported = true;
    result.ready = adcReady && channelEnabled[source];
    result.sampleSeen = adcState.seen;
    result.ageUs = sitlAdcAgeUs(&adcState, now);
    if (adcState.seen) result.value = source == ADC_EXTERNAL1 ? adcState.sample.pb0 : adcState.sample.pc0;
    return result;
}

adcDiagnosticPairSnapshot_t adcGetDiagnosticPairSnapshot(adcSource_e first, adcSource_e second, timeUs_t now)
{
    receiveAdc(now);
    adcDiagnosticPairSnapshot_t result = { .ageUs = UINT32_MAX };
    if (!positionSource(first) || !positionSource(second) || first == second) return result;
    result.supported = true;
    result.ready = adcReady && channelEnabled[first] && channelEnabled[second];
    result.sampleSeen = adcState.seen;
    result.ageUs = sitlAdcAgeUs(&adcState, now);
    if (adcState.seen) {
        result.firstValue = first == ADC_EXTERNAL1 ? adcState.sample.pb0 : adcState.sample.pc0;
        result.secondValue = second == ADC_EXTERNAL1 ? adcState.sample.pb0 : adcState.sample.pc0;
    }
    return result;
}

void adcResetDiagnosticSnapshot(void)
{
    memset(&adcState, 0, sizeof(adcState));
}
#endif
