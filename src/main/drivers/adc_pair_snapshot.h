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

#pragma once

#include <stdbool.h>
#include <stdint.h>

// F4 regular channels use 480-cycle sampling. The guarded copy must finish
// well before another conversion can reach either member of the pair.
#define ADC_DIAGNOSTIC_PAIR_COPY_MAX_US 5U

static inline bool adcDiagnosticPairPhaseIsSafe(unsigned channelCount, unsigned firstIndex, unsigned secondIndex, unsigned remaining)
{
    if (!channelCount || firstIndex >= channelCount || secondIndex >= channelCount
        || firstIndex == secondIndex || !remaining || remaining > channelCount) {
        return false;
    }

    const unsigned earliestIndex = firstIndex < secondIndex ? firstIndex : secondIndex;
    const unsigned nextIndex = channelCount - remaining;
    // A whole unrelated conversion must still precede the first pair member.
    // Equality is deliberately excluded: that member might finish immediately.
    return nextIndex < earliestIndex;
}

static inline bool adcDiagnosticPairCopyIsCoherent(unsigned channelCount, unsigned firstIndex, unsigned secondIndex,
    unsigned remainingBefore, unsigned remainingAfter, uint32_t startedAtUs, uint32_t endedAtUs)
{
    return remainingBefore == remainingAfter
        && (uint32_t)(endedAtUs - startedAtUs) <= ADC_DIAGNOSTIC_PAIR_COPY_MAX_US
        && adcDiagnosticPairPhaseIsSafe(channelCount, firstIndex, secondIndex, remainingBefore);
}

typedef struct adcDiagnosticPairCache_s {
    bool seen;
    uint16_t firstValue;
    uint16_t secondValue;
    uint32_t capturedAtUs;
} adcDiagnosticPairCache_t;

static inline void adcDiagnosticPairCachePublish(adcDiagnosticPairCache_t *cache, uint16_t firstValue, uint16_t secondValue, uint32_t currentTimeUs)
{
    cache->firstValue = firstValue;
    cache->secondValue = secondValue;
    cache->capturedAtUs = currentTimeUs;
    cache->seen = true;
}

static inline uint32_t adcDiagnosticPairCacheAge(const adcDiagnosticPairCache_t *cache, uint32_t currentTimeUs)
{
    return cache->seen ? (uint32_t)(currentTimeUs - cache->capturedAtUs) : UINT32_MAX;
}
