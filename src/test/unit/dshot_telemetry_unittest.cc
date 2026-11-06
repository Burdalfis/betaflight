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

#include <cstring>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "common/filter.h"
#include "config/feature.h"
#include "drivers/dshot.h"
#include "drivers/time.h"

motorConfig_t motorConfig_System;
bool useDshotTelemetry;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

static timeUs_t testTimeUs;
static unsigned timerReadCount;

timeUs_t micros(void)
{
    timerReadCount++;
    return testTimeUs;
}

bool featureIsEnabled(uint32_t mask)
{
    (void)mask;
    return false;
}

// The test exercises the real raw decoder, pole conversion, cached RPM path,
// and freshness metadata. RPM-filter behavior is independent of this change.
float pt1FilterApply(pt1Filter_t *filter, float input)
{
    (void)filter;
    return input;
}
}

#include "gtest/gtest.h"

namespace {
constexpr uint16_t RPM_FRAME = 0x0300; // decoded period = 512 us
constexpr uint16_t STOPPED_FRAME = 0x0fff;
constexpr uint16_t TEMPERATURE_FRAME = 0x022a; // EDT: 42 C

class DshotTelemetryTest : public testing::TestWithParam<debugType_e> {
protected:
    void SetUp() override
    {
        std::memset(&motorConfig_System, 0, sizeof(motorConfig_System));
        std::memset(debug, 0, sizeof(debug));
        motorConfigMutable()->motorPoleCount = 14;
        motorConfigMutable()->dev.useDshotTelemetry = DSHOT_TELEMETRY_ON;
        motorConfigMutable()->dev.useDshotEdt = DSHOT_EDT_FORCE;
        useDshotTelemetry = true;
        debugMode = GetParam();
        dshotMotorCount = 3;
        testTimeUs = 0;
        timerReadCount = 0;
        dshotCleanTelemetryData();
        initDshotTelemetry(250);
    }

    void process(timeUs_t atUs)
    {
        testTimeUs = atUs;
        dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
        updateDshotTelemetry();
    }

    void receive(uint8_t motor, uint16_t frame, timeUs_t atUs)
    {
        dshotTelemetryStoreRawValue(motor, frame);
        process(atUs);
    }

    dshotTelemetryRpmSample_t sample(uint8_t motor, timeUs_t atUs)
    {
        return getDshotTelemetryRpmSample(motor, atUs);
    }

    void expectUnknown(uint8_t motor, timeUs_t atUs)
    {
        const auto state = sample(motor, atUs);
        EXPECT_FALSE(state.seen);
        EXPECT_EQ(UINT32_MAX, state.ageUs);
        EXPECT_EQ(0U, state.sequence);
    }
};
}

TEST_P(DshotTelemetryTest, UnseenAndOutOfRangeAreExplicitlyUnknown)
{
    expectUnknown(0, 1000);
    expectUnknown(3, 1000);
    expectUnknown(MAX_SUPPORTED_MOTORS, 1000);
    dshotTelemetryStoreRawValue(MAX_SUPPORTED_MOTORS, RPM_FRAME);
    expectUnknown(0, 1000);
}

TEST_P(DshotTelemetryTest, NewlyReceivedEqualFramesRefreshButCachedDecodingDoesNot)
{
    receive(0, RPM_FRAME, 100);
    const uint16_t expectedErpm = (600000U + 256U) / 512U;
    EXPECT_EQ(expectedErpm, getDshotErpm(0));
    EXPECT_NEAR(expectedErpm * 100.0f / 7.0f, getDshotRpm(0), 0.01f);
    EXPECT_TRUE(sample(0, 100).seen);
    EXPECT_EQ(0U, sample(0, 100).ageUs);
    EXPECT_EQ(1U, sample(0, 100).sequence);

    // Transports leave rawValue cached after missing/invalid/aborted packets.
    process(200);
    EXPECT_EQ(expectedErpm, getDshotErpm(0));
    EXPECT_EQ(100U, sample(0, 200).ageUs);
    EXPECT_EQ(1U, sample(0, 200).sequence);

    receive(0, RPM_FRAME, 300);
    EXPECT_EQ(2U, sample(0, 300).sequence);
    EXPECT_EQ(0U, sample(0, 300).ageUs);
}

TEST_P(DshotTelemetryTest, MissingInvalidAbortedAndHandshakeCyclesDoNotRefresh)
{
    receive(0, RPM_FRAME, 100);
    // These transport outcomes never call dshotTelemetryStoreRawValue. The
    // existing decoder is still invoked for the cached value each cycle.
    const char *outcomes[] = {"no edges", "invalid checksum", "aborted reception", "EDT enable handshake"};
    for (unsigned i = 0; i < sizeof(outcomes) / sizeof(outcomes[0]); i++) {
        SCOPED_TRACE(outcomes[i]);
        if (i == 3) {
            dshotTelemetryState.motorState[0].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
        }
        process(200 + i * 100);
        EXPECT_EQ(1U, sample(0, testTimeUs).sequence);
        EXPECT_EQ(static_cast<uint32_t>(testTimeUs - 100), sample(0, testTimeUs).ageUs);
    }
    EXPECT_EQ(1000000U, sample(0, 1000100).ageUs);
}

TEST_P(DshotTelemetryTest, ExtendedTelemetryDoesNotBecomeAnRpmObservation)
{
    receive(1, TEMPERATURE_FRAME, 50);
    expectUnknown(1, 50);
    EXPECT_EQ(42, dshotTelemetryState.motorState[1].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE]);
    receive(0, RPM_FRAME, 100);
    receive(0, TEMPERATURE_FRAME, 500);
    EXPECT_EQ(1U, sample(0, 500).sequence);
    EXPECT_EQ(400U, sample(0, 500).ageUs);
    process(600); // repeated cached EDT is still not RPM
    EXPECT_EQ(500U, sample(0, 600).ageUs);
}

TEST_P(DshotTelemetryTest, ZeroSpeedIsAValidFreshRpmSample)
{
    receive(2, STOPPED_FRAME, 0);
    EXPECT_TRUE(sample(2, 0).seen);
    EXPECT_EQ(0U, sample(2, 0).ageUs);
    EXPECT_EQ(1U, sample(2, 0).sequence);
    EXPECT_EQ(0, getDshotErpm(2));
    EXPECT_FLOAT_EQ(0.0f, getDshotRpm(2));
}

TEST_P(DshotTelemetryTest, InvalidDecodedPeriodAndSentinelsDoNotRefresh)
{
    receive(0, 0, 100); // zero encoded period is invalid
    expectUnknown(0, 100);
    receive(0, RPM_FRAME, 200);
    receive(0, 0, 300);
    receive(0, DSHOT_TELEMETRY_INVALID, 400);
    receive(0, DSHOT_TELEMETRY_NOEDGE, 500);
    EXPECT_EQ(1U, sample(0, 500).sequence);
    EXPECT_EQ(300U, sample(0, 500).ageUs);
}

TEST_P(DshotTelemetryTest, PerMotorAgesRemainIndependentAndWrapSafely)
{
    receive(0, RPM_FRAME, UINT32_MAX - 49U);
    receive(1, RPM_FRAME, UINT32_MAX - 9U);
    EXPECT_EQ(100U, sample(0, 50).ageUs);
    EXPECT_EQ(60U, sample(1, 50).ageUs);
    expectUnknown(2, 50);
}

TEST_P(DshotTelemetryTest, DisabledDiagnosticsPreserveRpmAndAvoidTimerReads)
{
    debugMode = DEBUG_NONE;
    receive(0, RPM_FRAME, 100);
    EXPECT_GT(getDshotRpm(0), 0.0f);
    EXPECT_EQ(0U, timerReadCount);
    expectUnknown(0, 100);
    debugMode = GetParam();
    process(200); // selecting diagnostics does not freshen a cached value
    expectUnknown(0, 200);
    receive(0, RPM_FRAME, 300);
    EXPECT_EQ(1U, sample(0, 300).sequence);

    useDshotTelemetry = false;
    expectUnknown(0, 300);
}

TEST_P(DshotTelemetryTest, ProcessedBatchAndCleanCannotCreatePhantomSamples)
{
    receive(0, RPM_FRAME, 100);
    testTimeUs = 200;
    const unsigned readsBefore = timerReadCount;
    updateDshotTelemetry(); // already processed batch
    EXPECT_EQ(readsBefore, timerReadCount);
    EXPECT_EQ(1U, sample(0, 200).sequence);
    dshotCleanTelemetryData();
    expectUnknown(0, 300);
    process(300);
    expectUnknown(0, 300);
}

TEST_P(DshotTelemetryTest, SwitchingDiagnosticModesDoesNotFreshenCachedRpm)
{
    receive(0, RPM_FRAME, 100);
    debugMode = GetParam() == DEBUG_TRI_SERVO ? DEBUG_TRI_GROUND : DEBUG_TRI_SERVO;
    process(200);
    EXPECT_EQ(1U, sample(0, 200).sequence);
    EXPECT_EQ(100U, sample(0, 200).ageUs);

    receive(0, TEMPERATURE_FRAME, 300);
    EXPECT_EQ(1U, sample(0, 300).sequence);
    EXPECT_EQ(200U, sample(0, 300).ageUs);
    receive(0, RPM_FRAME, 400);
    EXPECT_EQ(2U, sample(0, 400).sequence);
    EXPECT_EQ(0U, sample(0, 400).ageUs);
}

INSTANTIATE_TEST_SUITE_P(TricopterDebugModes, DshotTelemetryTest,
    testing::Values(DEBUG_TRI_SERVO, DEBUG_TRI_GROUND));
