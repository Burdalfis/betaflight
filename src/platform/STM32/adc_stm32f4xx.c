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
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "drivers/dma.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"
#include "drivers/adc_pair_snapshot.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"

// These are missing from STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h
#ifdef STM32F446xx
#define ADC_Channel_TempSensor ADC_Channel_18
#endif

const adcDevice_t adcHardware[] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_Channel_0
#endif
    },
#if !defined(STM32F411xE)
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_Channel_1
#endif
    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC3_DMA_STREAM,
        .channel = DMA_Channel_2
#endif
    }
#endif
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
/*
    { DEFIO_TAG_E__PF3, ADC_DEVICES_3,   ADC_Channel_9  },
    { DEFIO_TAG_E__PF4, ADC_DEVICES_3,   ADC_Channel_14 },
    { DEFIO_TAG_E__PF5, ADC_DEVICES_3,   ADC_Channel_15 },
    { DEFIO_TAG_E__PF6, ADC_DEVICES_3,   ADC_Channel_4  },
    { DEFIO_TAG_E__PF7, ADC_DEVICES_3,   ADC_Channel_5  },
    { DEFIO_TAG_E__PF8, ADC_DEVICES_3,   ADC_Channel_6  },
    { DEFIO_TAG_E__PF9, ADC_DEVICES_3,   ADC_Channel_7  },
    { DEFIO_TAG_E__PF10,ADC_DEVICES_3,   ADC_Channel_8  },
*/
#if defined(STM32F411xE)
    { DEFIO_TAG_E__PC0, ADC_DEVICES_1,   ADC_Channel_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_1,   ADC_Channel_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_1,   ADC_Channel_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_1,   ADC_Channel_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_1,   ADC_Channel_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_1,   ADC_Channel_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_1,   ADC_Channel_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_1,   ADC_Channel_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_1,   ADC_Channel_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_1,   ADC_Channel_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_1,   ADC_Channel_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_1,   ADC_Channel_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_1,   ADC_Channel_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_1,   ADC_Channel_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_1,   ADC_Channel_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_1,   ADC_Channel_7  },
#else
    { DEFIO_TAG_E__PC0, ADC_DEVICES_123, ADC_Channel_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_123, ADC_Channel_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_123, ADC_Channel_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_123, ADC_Channel_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_12,  ADC_Channel_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_12,  ADC_Channel_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_12,  ADC_Channel_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_12,  ADC_Channel_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_123, ADC_Channel_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_123, ADC_Channel_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_123, ADC_Channel_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_123, ADC_Channel_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_12,  ADC_Channel_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_12,  ADC_Channel_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_12,  ADC_Channel_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_12,  ADC_Channel_7  },
#endif
};

#define VREFINT_CAL_ADDR  0x1FFF7A2A
#define TS_CAL1_ADDR      0x1FFF7A2C
#define TS_CAL2_ADDR      0x1FFF7A2E

static dmaChannelDescriptor_t *adcDiagnosticDma;
static bool adcDiagnosticObserving;
static bool adcDiagnosticSampleSeen;
static timeUs_t adcDiagnosticLastScanUs;
static uint8_t adcDiagnosticChannelCount;
static bool adcDiagnosticPairObserving;
static bool adcDiagnosticPairCompletionPending;
static adcSource_e adcDiagnosticPairFirst = ADC_NONE;
static adcSource_e adcDiagnosticPairSecond = ADC_NONE;
static adcDiagnosticPairCache_t adcDiagnosticPairCache;

void adcResetDiagnosticSnapshot(void)
{
    adcDiagnosticObserving = false;
    adcDiagnosticSampleSeen = false;
    adcDiagnosticPairObserving = false;
    adcDiagnosticPairCompletionPending = false;
    adcDiagnosticPairFirst = ADC_NONE;
    adcDiagnosticPairSecond = ADC_NONE;
    memset(&adcDiagnosticPairCache, 0, sizeof(adcDiagnosticPairCache));
}

adcDiagnosticSnapshot_t adcGetDiagnosticSnapshot(adcSource_e source, timeUs_t currentTimeUs)
{
    adcDiagnosticSnapshot_t snapshot = { .supported = true, .ageUs = UINT32_MAX };
    if ((unsigned)source >= ADC_EXTERNAL_COUNT || !adcOperatingConfig[source].enabled || !adcDiagnosticDma) {
        return snapshot;
    }

    snapshot.value = adcGetValue(source);
    const dmaChannelDescriptor_t *descriptor = adcDiagnosticDma;
    const uint32_t errorFlags = DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF;
    snapshot.ready = (((DMA_Stream_TypeDef *)descriptor->ref)->CR & DMA_SxCR_EN)
        && !DMA_GET_FLAG_STATUS(descriptor, errorFlags);

    if (!adcDiagnosticObserving) {
        // TC may have been sticky since boot. Await a subsequent completed scan
        // before claiming any progress during this logging session.
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        adcDiagnosticObserving = true;
    } else if (snapshot.ready && DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        adcDiagnosticLastScanUs = currentTimeUs;
        adcDiagnosticSampleSeen = true;
    }

    snapshot.sampleSeen = adcDiagnosticSampleSeen;
    if (snapshot.sampleSeen) {
        snapshot.ageUs = (uint32_t)(currentTimeUs - adcDiagnosticLastScanUs);
    }
    return snapshot;
}

adcDiagnosticPairSnapshot_t adcGetDiagnosticPairSnapshot(adcSource_e first, adcSource_e second, timeUs_t currentTimeUs)
{
    adcDiagnosticPairSnapshot_t snapshot = { .supported = true, .ageUs = UINT32_MAX };
    if ((unsigned)first >= ADC_EXTERNAL_COUNT || (unsigned)second >= ADC_EXTERNAL_COUNT
        || first == second || !adcDiagnosticDma || !adcDiagnosticChannelCount
        || !adcOperatingConfig[first].enabled || !adcOperatingConfig[second].enabled
        || adcOperatingConfig[first].tag == adcOperatingConfig[second].tag) {
        return snapshot;
    }

    const unsigned firstIndex = adcOperatingConfig[first].dmaIndex;
    const unsigned secondIndex = adcOperatingConfig[second].dmaIndex;
    // For configurations with no earlier unrelated conversion there is no
    // guarded prefix in this single DMA buffer. Report unavailable honestly.
    if (!adcDiagnosticPairPhaseIsSafe(adcDiagnosticChannelCount, firstIndex, secondIndex, adcDiagnosticChannelCount)) {
        return snapshot;
    }

    const dmaChannelDescriptor_t *descriptor = adcDiagnosticDma;
    const uint32_t errorFlags = DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF;
    snapshot.ready = (((DMA_Stream_TypeDef *)descriptor->ref)->CR & DMA_SxCR_EN)
        && !DMA_GET_FLAG_STATUS(descriptor, errorFlags);

    if (first != adcDiagnosticPairFirst || second != adcDiagnosticPairSecond) {
        adcDiagnosticPairObserving = false;
        adcDiagnosticPairCompletionPending = false;
        adcDiagnosticPairFirst = first;
        adcDiagnosticPairSecond = second;
        memset(&adcDiagnosticPairCache, 0, sizeof(adcDiagnosticPairCache));
    }

    if (!adcDiagnosticPairObserving) {
        // Discard an old sticky completion from before this logging session.
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        adcDiagnosticPairObserving = true;
    } else if (snapshot.ready) {
        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
            DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
            adcDiagnosticPairCompletionPending = true;
        }

        if (adcDiagnosticPairCompletionPending) {
            const uint32_t startedAtUs = (uint32_t)micros();
            const unsigned remainingBefore = dmaGetDataLength(descriptor->ref);
            if (adcDiagnosticPairPhaseIsSafe(adcDiagnosticChannelCount, firstIndex, secondIndex, remainingBefore)) {
                const uint16_t firstValue = adcValues[firstIndex];
                const uint16_t secondValue = adcValues[secondIndex];
                const unsigned remainingAfter = dmaGetDataLength(descriptor->ref);
                const uint32_t endedAtUs = (uint32_t)micros();
                if (adcDiagnosticPairCopyIsCoherent(adcDiagnosticChannelCount, firstIndex, secondIndex,
                    remainingBefore, remainingAfter, startedAtUs, endedAtUs)) {
                    adcDiagnosticPairCachePublish(&adcDiagnosticPairCache, firstValue, secondValue, (uint32_t)currentTimeUs);
                    adcDiagnosticPairCompletionPending = false;
                    // Consume any completion that occurred before this bounded
                    // copy, including during preemption before startedAtUs.
                    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
                }
            }
        }
    }

    snapshot.sampleSeen = adcDiagnosticPairCache.seen;
    snapshot.firstValue = adcDiagnosticPairCache.firstValue;
    snapshot.secondValue = adcDiagnosticPairCache.secondValue;
    snapshot.ageUs = adcDiagnosticPairCacheAge(&adcDiagnosticPairCache, (uint32_t)currentTimeUs);
    return snapshot;
}

static void adcInitDevice(ADC_TypeDef *adcdev, int channelCount)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion          = channelCount;

    // Multiple injected channel seems to require scan conversion mode to be
    // enabled even if main (non-injected) channel count is 1.
#ifdef USE_ADC_INTERNAL
    ADC_InitStructure.ADC_ScanConvMode             = ENABLE;
#else
    ADC_InitStructure.ADC_ScanConvMode             = channelCount > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group
#endif
    ADC_Init(adcdev, &ADC_InitStructure);
}

#ifdef USE_ADC_INTERNAL
static void adcInitInternalInjected(const adcConfig_t *config)
{
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_InjectedDiscModeCmd(ADC1, DISABLE);
    ADC_InjectedSequencerLengthConfig(ADC1, 2);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_480Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_480Cycles);

    adcVREFINTCAL = config->vrefIntCalibration ? config->vrefIntCalibration : *(uint16_t *)VREFINT_CAL_ADDR;
    adcTSCAL1 = config->tempSensorCalibration1 ? config->tempSensorCalibration1 : *(uint16_t *)TS_CAL1_ADDR;
    adcTSCAL2 = config->tempSensorCalibration2 ? config->tempSensorCalibration2 : *(uint16_t *)TS_CAL2_ADDR;

    adcTSSlopeK = (110 - 30) * 1000 / (adcTSCAL2 - adcTSCAL1);
}

// Note on sampling time for temperature sensor and vrefint:
// Both sources have minimum sample time of 10us.
// With prescaler = 8:
// 168MHz : fAPB2 = 84MHz, fADC = 10.5MHz, tcycle = 0.090us, 10us = 105cycle < 144cycle
// 240MHz : fAPB2 = 120MHz, fADC = 15.0MHz, tcycle = 0.067usk 10us = 150cycle < 480cycle
//
// 480cycles@15.0MHz = 32us

static bool adcInternalConversionInProgress = false;

bool adcInternalIsBusy(void)
{
    if (adcInternalConversionInProgress) {
        if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET) {
            adcInternalConversionInProgress = false;
        }
    }

    return adcInternalConversionInProgress;
}

void adcInternalStartConversion(void)
{
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConv(ADC1);

    adcInternalConversionInProgress = true;
}

uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
        return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    case ADC_TEMPSENSOR:
        return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    default:
        return 0;
    }
}
#endif

void adcInit(const adcConfig_t *config)
{
    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    adcDiagnosticDma = NULL;
    adcDiagnosticChannelCount = 0;
    adcResetDiagnosticSnapshot();

    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  //CURRENT_METER_ADC_CHANNEL;
    }

    adcDevice_e device = ADC_CFG_TO_DEV(config->device);

    if (device == ADCINVALID) {
        return;
    }

    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }

        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = ADC_SampleTime_480Cycles;
        adcOperatingConfig[i].enabled = true;
    }

#ifndef USE_ADC_INTERNAL
    if (!adcActive) {
        return;
    }
#endif

    RCC_ClockCmd(adc.rccADC, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

#ifdef USE_ADC_INTERNAL
    // If device is not ADC1 or there's no active channel, then initialize ADC1 separately
    if (device != ADCDEV_1 || !adcActive) {
        RCC_ClockCmd(adcHardware[ADCDEV_1].rccADC, ENABLE);
        adcInitDevice(ADC1, 2);
        ADC_Cmd(ADC1, ENABLE);
    }

    // Initialize for injected conversion
    adcInitInternalInjected(config);

    adcOperatingConfig[ADC_VREFINT].enabled = true;
    adcOperatingConfig[ADC_TEMPSENSOR].enabled = true;

    if (!adcActive) {
        return;
    }
#endif

    adcInitDevice(adc.ADCx, configuredAdcChannels);

    uint8_t rank = 1;
    for (i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(adc.ADCx, adcOperatingConfig[i].adcChannel, rank++, adcOperatingConfig[i].sampleTime);
    }
    ADC_DMARequestAfterLastTransferCmd(adc.ADCx, ENABLE);

    ADC_DMACmd(adc.ADCx, ENABLE);
    ADC_Cmd(adc.ADCx, ENABLE);

#ifdef USE_DMA_SPEC
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, device, config->dmaopt[device]);

    if (!dmaSpec || !dmaAllocate(dmaGetIdentifier(dmaSpec->ref), OWNER_ADC, RESOURCE_INDEX(device))) {
        return;
    }

    dmaEnable(dmaGetIdentifier(dmaSpec->ref));

    xDMA_DeInit(dmaSpec->ref);
#else
    if (!dmaAllocate(dmaGetIdentifier(adc.dmaResource), OWNER_ADC, 0)) {
        return;
    }

    dmaEnable(dmaGetIdentifier(adc.dmaResource));

    xDMA_DeInit(adc.dmaResource);
#endif

    DMA_InitTypeDef DMA_InitStructure;

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&adc.ADCx->DR;

#ifdef USE_DMA_SPEC
    DMA_InitStructure.DMA_Channel = dmaSpec->channel;
#else
    DMA_InitStructure.DMA_Channel = adc.channel;
#endif

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

#ifdef USE_DMA_SPEC
    xDMA_Init(dmaSpec->ref, &DMA_InitStructure);
    xDMA_Cmd(dmaSpec->ref, ENABLE);
#else
    xDMA_Init(adc.dmaResource, &DMA_InitStructure);
    xDMA_Cmd(adc.dmaResource, ENABLE);
#endif

    ADC_SoftwareStartConv(adc.ADCx);

    adcDiagnosticChannelCount = configuredAdcChannels;

#ifdef USE_DMA_SPEC
    adcDiagnosticDma = dmaGetDescriptorByIdentifier(dmaGetIdentifier(dmaSpec->ref));
#else
    adcDiagnosticDma = dmaGetDescriptorByIdentifier(dmaGetIdentifier(adc.dmaResource));
#endif
}

void adcGetChannelValues(void)
{
    // Nothing to do
}
#endif
