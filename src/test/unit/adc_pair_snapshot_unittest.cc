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

extern "C" {
#include "drivers/adc_pair_snapshot.h"
}

#include "gtest/gtest.h"

TEST(AdcPairSnapshot, LastTwoRanksUseOnlyEarlierUnrelatedConversions)
{
    // VBAT, current, PB0 wiper, PC0 reference: indices 0, 1, 2, 3.
    EXPECT_TRUE(adcDiagnosticPairPhaseIsSafe(4, 2, 3, 4));
    EXPECT_TRUE(adcDiagnosticPairPhaseIsSafe(4, 2, 3, 3));
    // With two transfers remaining the wiper is pending and can change now.
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 2, 3, 2));
    // With one remaining the wiper may already belong to the next scan.
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 2, 3, 1));
    EXPECT_TRUE(adcDiagnosticPairPhaseIsSafe(4, 3, 2, 4));
    EXPECT_TRUE(adcDiagnosticPairPhaseIsSafe(3, 1, 2, 3));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(3, 1, 2, 2));
}

TEST(AdcPairSnapshot, ConfigurationsWithoutGuardedPrefixAreUnavailable)
{
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(2, 0, 1, 2));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 0, 3, 4));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(0, 0, 1, 0));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 2, 2, 4));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 2, 4, 4));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 2, 3, 0));
    EXPECT_FALSE(adcDiagnosticPairPhaseIsSafe(4, 2, 3, 5));
}

TEST(AdcPairSnapshot, DmaProgressDuringCopyRejectsPublication)
{
    EXPECT_TRUE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 4, 4, 100, 101));
    EXPECT_TRUE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 3, 3, 100, 105));
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 4, 3, 100, 101));
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 3, 2, 100, 101));
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 1, 4, 100, 101));
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 2, 2, 100, 101));
}

TEST(AdcPairSnapshot, InterruptOrFullDmaWrapCannotMasqueradeAsStableCount)
{
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 4, 4, 100, 106));
    // NDTR can look identical after one or several complete scan periods.
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 4, 4, 100, 500));
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 3, 3, 100, 1500));
}

TEST(AdcPairSnapshot, CopyClockWrapUsesUnsignedElapsedTime)
{
    EXPECT_TRUE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 4, 4, UINT32_MAX - 2U, 2));
    EXPECT_FALSE(adcDiagnosticPairCopyIsCoherent(4, 2, 3, 4, 4, UINT32_MAX - 2U, 3));
}

TEST(AdcPairSnapshot, UnseenCacheAndRealZeroReferenceRemainDistinct)
{
    adcDiagnosticPairCache_t cache = {};
    EXPECT_FALSE(cache.seen);
    EXPECT_EQ(UINT32_MAX, adcDiagnosticPairCacheAge(&cache, 100));
    adcDiagnosticPairCachePublish(&cache, 1590, 0, 100);
    EXPECT_TRUE(cache.seen);
    EXPECT_EQ(1590, cache.firstValue);
    EXPECT_EQ(0, cache.secondValue);
    EXPECT_EQ(0U, adcDiagnosticPairCacheAge(&cache, 100));
    EXPECT_EQ(275U, adcDiagnosticPairCacheAge(&cache, 375));
}

TEST(AdcPairSnapshot, RejectedCopyPreservesPreviousPairAndItsAge)
{
    adcDiagnosticPairCache_t cache = {};
    adcDiagnosticPairCachePublish(&cache, 1590, 48, 100);
    if (adcDiagnosticPairCopyIsCoherent(4, 2, 3, 2, 2, 300, 301)) {
        adcDiagnosticPairCachePublish(&cache, 1700, 80, 300);
    }
    EXPECT_EQ(1590, cache.firstValue);
    EXPECT_EQ(48, cache.secondValue);
    EXPECT_EQ(200U, adcDiagnosticPairCacheAge(&cache, 300));
    // A genuinely accepted equal-valued pair still advances observation time.
    adcDiagnosticPairCachePublish(&cache, 1590, 48, 400);
    EXPECT_EQ(0U, adcDiagnosticPairCacheAge(&cache, 400));
}

TEST(AdcPairSnapshot, CachedAgeWrapIsIndependentOfPairValues)
{
    adcDiagnosticPairCache_t cache = {};
    adcDiagnosticPairCachePublish(&cache, 0, 4095, UINT32_MAX - 49U);
    EXPECT_EQ(100U, adcDiagnosticPairCacheAge(&cache, 50));
    EXPECT_EQ(0, cache.firstValue);
    EXPECT_EQ(4095, cache.secondValue);
}
