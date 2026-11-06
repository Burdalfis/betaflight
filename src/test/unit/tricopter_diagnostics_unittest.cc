/* Tricopter LOG_ONLY diagnostics: no actuator feedback or control changes. */
#include <algorithm>
#include <cstring>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "drivers/adc.h"
#include "drivers/dshot.h"
#include "flight/servos.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;
servoConfig_t servoConfig_System;
int16_t servo[MAX_SUPPORTED_SERVOS];
bool useDshotTelemetry;
uint8_t dshotMotorCount;
}

#include "gtest/gtest.h"

static adcDiagnosticSnapshot_t adcSample;
static adcDiagnosticPairSnapshot_t adcPair;
static dshotTelemetryRpmSample_t rpmSamples[3];
static int16_t invalidPercent[3];
static unsigned adcReads, adcPairReads, adcResets, rpmReads, clockReads, mixerCalls;
static timeUs_t now;

extern "C" {
timeUs_t micros(void) { clockReads++; return now; }
adcDiagnosticSnapshot_t adcGetDiagnosticSnapshot(adcSource_e source, timeUs_t time)
{
    EXPECT_EQ(ADC_EXTERNAL1, source);
    EXPECT_EQ(now, time);
    adcReads++;
    return adcSample;
}
void adcResetDiagnosticSnapshot(void) { adcResets++; }
adcDiagnosticPairSnapshot_t adcGetDiagnosticPairSnapshot(adcSource_e first, adcSource_e second, timeUs_t time)
{
    EXPECT_EQ(ADC_EXTERNAL1, first);
    EXPECT_EQ(ADC_RSSI, second);
    EXPECT_EQ(now, time);
    adcPairReads++;
    return adcPair;
}
dshotTelemetryRpmSample_t getDshotTelemetryRpmSample(uint8_t motor, timeUs_t time)
{
    EXPECT_LT(motor, 3);
    EXPECT_EQ(now, time);
    rpmReads++;
    return rpmSamples[motor];
}
int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motor) { return invalidPercent[motor]; }
void servoMixer(void) { mixerCalls++; }
}

class TricopterDiagnostics : public ::testing::Test {
protected:
    void SetUp() override
    {
        debugMode = DEBUG_NONE;
        servosTricopterLog(0); // End any previous logging session.
        std::fill(debug, debug + DEBUG16_VALUE_COUNT, 1234);
        std::fill(servo, servo + MAX_SUPPORTED_SERVOS, 1420);
        std::memset(&adcSample, 0, sizeof(adcSample));
        std::memset(&adcPair, 0, sizeof(adcPair));
        std::memset(rpmSamples, 0, sizeof(rpmSamples));
        std::fill(invalidPercent, invalidPercent + 3, 0);
        adcReads = adcPairReads = adcResets = rpmReads = clockReads = mixerCalls = 0;
        useDshotTelemetry = true;
        dshotMotorCount = 3;
        now = 123456;
    }
};

TEST_F(TricopterDiagnostics, LegacyModeDoesNotSampleOrOverwriteAnotherDebugMode)
{
    debugMode = DEBUG_GYRO_FILTERED;
    servosTricopterLog(1700);
    EXPECT_EQ(0u, adcReads + adcPairReads + rpmReads + clockReads + adcResets);
    for (int16_t value : debug) { EXPECT_EQ(1234, value); }
}

TEST_F(TricopterDiagnostics, LogsWrittenPulseIncludingDisarmedZeroWithoutChangingActuators)
{
    debugMode = DEBUG_TRI_SERVO;
    for (int16_t pulse : {1100, 1420, 1700, 0}) {
        servosTricopterLog(pulse);
        EXPECT_EQ(pulse, debug[1]);
        for (int16_t value : servo) { EXPECT_EQ(1420, value); }
    }
    EXPECT_EQ(0u, mixerCalls);
    servosTricopterMixer();
    EXPECT_EQ(1u, mixerCalls);
}

TEST_F(TricopterDiagnostics, EncodesLogicalMotorAgesAndWorstPacketError)
{
    debugMode = DEBUG_TRI_SERVO;
    adcSample = {true, true, true, 1590, 1234};
    rpmSamples[0] = {true, 200, 1};
    rpmSamples[1] = {true, 6789, 2};
    rpmSamples[2] = {true, 99999, 3};
    invalidPercent[0] = 100;
    invalidPercent[1] = 800;
    invalidPercent[2] = 350;
    servosTricopterLog(1420);
    EXPECT_EQ(1590, debug[0]);
    EXPECT_EQ(12, debug[2]);
    EXPECT_EQ(255, debug[3]);
    EXPECT_EQ(2, debug[4]);
    EXPECT_EQ(67, debug[5]);
    EXPECT_EQ(999, debug[6]);
    EXPECT_EQ(800, debug[7]);
}

TEST_F(TricopterDiagnostics, MissingSamplesRemainUnavailableAndOldSamplesSaturate)
{
    debugMode = DEBUG_TRI_SERVO;
    adcSample = {true, true, false, 1590, UINT32_MAX};
    rpmSamples[1] = {true, 4000000, 1};
    servosTricopterLog(1420);
    EXPECT_EQ(-1, debug[2]);
    EXPECT_EQ(-1, debug[4]);
    EXPECT_EQ(INT16_MAX, debug[5]);
    EXPECT_EQ(-1, debug[6]);
    EXPECT_EQ(1 | 2 | 16 | 64, debug[3]);
}

TEST_F(TricopterDiagnostics, RailsAndStoppedAcquisitionDoNotLookReadyAndPlausible)
{
    debugMode = DEBUG_TRI_SERVO;
    adcSample = {true, true, true, 0, 10};
    servosTricopterLog(1420);
    EXPECT_EQ(0, debug[3] & 8);
    adcSample.value = 4095;
    servosTricopterLog(1420);
    EXPECT_EQ(0, debug[3] & 8);
    adcSample.ready = false;
    servosTricopterLog(1420);
    EXPECT_EQ(-1, debug[0]);
    EXPECT_EQ(0, debug[3] & 2);
}

TEST_F(TricopterDiagnostics, UnsupportedOrDisabledInputsAreExplicit)
{
    debugMode = DEBUG_TRI_SERVO;
    useDshotTelemetry = false;
    servosTricopterLog(1420);
    EXPECT_EQ(0, debug[3]);
    EXPECT_EQ(-1, debug[0]);
    EXPECT_EQ(-1, debug[2]);
    EXPECT_EQ(-1, debug[7]);
}

TEST_F(TricopterDiagnostics, LeavingLogModeResetsAdcObservationSessionOnce)
{
    debugMode = DEBUG_TRI_SERVO;
    servosTricopterLog(1420);
    debugMode = DEBUG_NONE;
    servosTricopterLog(1420);
    servosTricopterLog(1420);
    EXPECT_EQ(1u, adcResets);
}

TEST_F(TricopterDiagnostics, GroundModeLogsBothRawValuesAndSignedDifference)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, true, 1590, 40, 250};
    rpmSamples[0] = {true, 200, 1};
    rpmSamples[1] = {true, 6789, 2};
    rpmSamples[2] = {true, 99999, 3};
    invalidPercent[1] = 125;
    servosTricopterLog(1420);
    EXPECT_EQ(1u, adcPairReads);
    EXPECT_EQ(0u, adcReads);
    EXPECT_EQ(1590, debug[0]);
    EXPECT_EQ(1420, debug[1]);
    EXPECT_EQ(40, debug[2]);
    EXPECT_EQ(1550, debug[3]);
    EXPECT_EQ(2, debug[4]);
    EXPECT_EQ(255, debug[5]);
    EXPECT_EQ(999, debug[6]);
    EXPECT_EQ(125, debug[7]);
}

TEST_F(TricopterDiagnostics, GroundModeCancelsCommonOffsetWithoutChangingRawData)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, true, 1590, 10, 0};
    servosTricopterLog(1420);
    EXPECT_EQ(1580, debug[3]);
    adcPair.firstValue += 45;
    adcPair.secondValue += 45;
    servosTricopterLog(1420);
    EXPECT_EQ(1635, debug[0]);
    EXPECT_EQ(55, debug[2]);
    EXPECT_EQ(1580, debug[3]);
}

TEST_F(TricopterDiagnostics, GroundModeAllowsZeroGroundAndSignedNegativeOne)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, true, 1590, 0, 0};
    servosTricopterLog(1420);
    EXPECT_EQ(0, debug[2]);
    EXPECT_EQ(1590, debug[3]);
    EXPECT_NE(0, debug[5] & (1 << 9));
    EXPECT_NE(0, debug[5] & (1 << 3));
    adcPair.firstValue = 0;
    adcPair.secondValue = 1;
    servosTricopterLog(1420);
    EXPECT_EQ(-1, debug[3]); // Real signed data, not the unavailable sentinel.
    adcPair.secondValue = 4095;
    servosTricopterLog(1420);
    EXPECT_EQ(-4095, debug[3]);
    EXPECT_NE(0, debug[5] & (1 << 10));
    adcPair.firstValue = 4095;
    adcPair.secondValue = 0;
    servosTricopterLog(1420);
    EXPECT_EQ(4095, debug[3]);
}

TEST_F(TricopterDiagnostics, GroundModeDoesNotInventValuesBeforeFirstCoherentPair)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, false, 1590, 40, UINT32_MAX};
    servosTricopterLog(1420);
    EXPECT_EQ(-1, debug[0]);
    EXPECT_EQ(-1, debug[2]);
    EXPECT_EQ(INT16_MIN, debug[3]);
    EXPECT_EQ(-1, debug[4]);
    EXPECT_EQ(0, debug[5] & (1 << 2));
    EXPECT_EQ(0, debug[5] & (1 << 3));
}

TEST_F(TricopterDiagnostics, GroundModeAgesCachedPairsAndClearsFreshness)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, true, 1590, 40, 1000};
    servosTricopterLog(1420);
    EXPECT_NE(0, debug[5] & (1 << 3));
    adcPair.ageUs = 1001;
    servosTricopterLog(1420);
    EXPECT_EQ(0, debug[5] & (1 << 3));
    EXPECT_EQ(1550, debug[3]);
    adcPair.ready = false;
    adcPair.ageUs = 4000000;
    servosTricopterLog(1420);
    EXPECT_EQ(0, debug[5] & (1 << 1));
    EXPECT_EQ(INT16_MAX, debug[4]);
    EXPECT_EQ(1550, debug[3]);
}

TEST_F(TricopterDiagnostics, GroundModeRequiresAllMotorSamplesForWorstRpmAge)
{
    debugMode = DEBUG_TRI_GROUND;
    rpmSamples[0] = {true, 200, 1};
    rpmSamples[1] = {true, 500, 1};
    servosTricopterLog(1420);
    EXPECT_EQ(-1, debug[6]);
    rpmSamples[2] = {true, 4000000, 1};
    servosTricopterLog(1420);
    EXPECT_EQ(INT16_MAX, debug[6]);
}

TEST_F(TricopterDiagnostics, SwitchingDiagnosticSchemasResetsAdcSession)
{
    debugMode = DEBUG_TRI_SERVO;
    servosTricopterLog(1420);
    debugMode = DEBUG_TRI_GROUND;
    servosTricopterLog(1420);
    debugMode = DEBUG_TRI_SERVO;
    servosTricopterLog(1420);
    EXPECT_EQ(2u, adcResets);
}

TEST_F(TricopterDiagnostics, GroundModeLogsSuppressedPulseWithoutWritingActuators)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, true, 1590, 40, 0};
    servosTricopterLog(0);
    EXPECT_EQ(0, debug[1]);
    for (int16_t value : servo) { EXPECT_EQ(1420, value); }
    EXPECT_EQ(0u, mixerCalls);
}

TEST_F(TricopterDiagnostics, CanonicalPositionUsesOnlyCorrectedCoherentPair)
{
    // The position API is independent of the selected debug presentation.
    debugMode = DEBUG_NONE;
    adcPair = {true, true, true, 1590, 40, 250};
    const tricopterServoPositionSample_t before = servosTricopterGetPosition(now);
    EXPECT_TRUE(before.supported);
    EXPECT_TRUE(before.ready);
    EXPECT_TRUE(before.sampleSeen);
    EXPECT_TRUE(before.fresh);
    EXPECT_EQ(1550, before.correctedCounts);
    EXPECT_EQ(250U, before.ageUs);
    EXPECT_EQ(1590, before.rawWiperCounts);
    EXPECT_EQ(40, before.rawGroundCounts);

    adcPair.firstValue += 45;
    adcPair.secondValue += 45;
    const tricopterServoPositionSample_t after = servosTricopterGetPosition(now);
    EXPECT_EQ(before.correctedCounts, after.correctedCounts);
    EXPECT_EQ(1635, after.rawWiperCounts);
    EXPECT_EQ(85, after.rawGroundCounts);
    EXPECT_EQ(2u, adcPairReads);
    EXPECT_EQ(0u, adcReads);
    EXPECT_EQ(0u, clockReads + mixerCalls);
    for (int16_t value : servo) { EXPECT_EQ(1420, value); }
}

TEST_F(TricopterDiagnostics, CanonicalPositionNeverFallsBackToRawWiper)
{
    adcSample = {true, true, true, 1590, 0}; // An independently valid PB0 is insufficient.
    adcPair = {true, true, false, 1590, 0, 0};
    const tricopterServoPositionSample_t missingReference = servosTricopterGetPosition(now);
    EXPECT_FALSE(missingReference.sampleSeen);
    EXPECT_FALSE(missingReference.fresh);
    EXPECT_EQ(TRICOPTER_SERVO_POSITION_UNAVAILABLE, missingReference.correctedCounts);
    EXPECT_EQ(UINT32_MAX, missingReference.ageUs);
    EXPECT_EQ(0u, adcReads);

    adcPair = {false, false, false, 1590, 0, 0};
    const tricopterServoPositionSample_t unsupported = servosTricopterGetPosition(now);
    EXPECT_FALSE(unsupported.supported);
    EXPECT_FALSE(unsupported.fresh);
    EXPECT_EQ(TRICOPTER_SERVO_POSITION_UNAVAILABLE, unsupported.correctedCounts);
    EXPECT_EQ(0u, adcReads);
}

TEST_F(TricopterDiagnostics, CanonicalPositionKeepsSignedCorrectionIncludingZeroGround)
{
    adcPair = {true, true, true, 1590, 0, 0};
    EXPECT_EQ(1590, servosTricopterGetPosition(now).correctedCounts);
    adcPair.firstValue = 0;
    adcPair.secondValue = 1;
    auto sample = servosTricopterGetPosition(now);
    EXPECT_TRUE(sample.fresh);
    EXPECT_EQ(-1, sample.correctedCounts);
    adcPair.secondValue = 4095;
    EXPECT_EQ(-4095, servosTricopterGetPosition(now).correctedCounts);
    adcPair.firstValue = 4095;
    adcPair.secondValue = 0;
    EXPECT_EQ(4095, servosTricopterGetPosition(now).correctedCounts);
}

TEST_F(TricopterDiagnostics, CanonicalPositionSeparatesCachedCorrectionFromFreshMeasurement)
{
    adcPair = {true, true, true, 1590, 40, TRICOPTER_SERVO_POSITION_MAX_AGE_US};
    EXPECT_TRUE(servosTricopterGetPosition(now).fresh);
    adcPair.ageUs++;
    auto stale = servosTricopterGetPosition(now);
    EXPECT_FALSE(stale.fresh);
    EXPECT_EQ(1550, stale.correctedCounts);
    EXPECT_EQ(1001U, stale.ageUs);
    adcPair.ready = false;
    adcPair.ageUs = 100;
    auto stopped = servosTricopterGetPosition(now);
    EXPECT_FALSE(stopped.ready);
    EXPECT_FALSE(stopped.fresh);
    EXPECT_EQ(1550, stopped.correctedCounts);
    EXPECT_EQ(0u, adcReads);
}

TEST_F(TricopterDiagnostics, CanonicalPositionRejectsNonTwelveBitRangeWithoutRawFallback)
{
    adcPair = {true, true, true, 4096, 40, 0};
    auto sample = servosTricopterGetPosition(now);
    EXPECT_TRUE(sample.sampleSeen);
    EXPECT_FALSE(sample.fresh);
    EXPECT_EQ(4096, sample.rawWiperCounts); // Retain the faulty diagnostic value.
    EXPECT_EQ(TRICOPTER_SERVO_POSITION_UNAVAILABLE, sample.correctedCounts);
    adcPair.firstValue = 1590;
    adcPair.secondValue = 5000;
    sample = servosTricopterGetPosition(now);
    EXPECT_FALSE(sample.fresh);
    EXPECT_EQ(TRICOPTER_SERVO_POSITION_UNAVAILABLE, sample.correctedCounts);
    EXPECT_EQ(0u, adcReads);
}

TEST_F(TricopterDiagnostics, GroundLoggerReportsCanonicalPositionAndFreshness)
{
    debugMode = DEBUG_TRI_GROUND;
    adcPair = {true, true, true, 1620, 65, 500};
    const auto sample = servosTricopterGetPosition(now);
    servosTricopterLog(1420);
    EXPECT_EQ(sample.correctedCounts, debug[3]);
    EXPECT_EQ(sample.rawWiperCounts, debug[0]);
    EXPECT_EQ(sample.rawGroundCounts, debug[2]);
    EXPECT_EQ(sample.fresh, (debug[5] & (1 << 3)) != 0);
    EXPECT_EQ(2u, adcPairReads); // Once per API request, never independent raw reads.
    EXPECT_EQ(0u, adcReads);
}
