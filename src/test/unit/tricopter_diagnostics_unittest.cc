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
static dshotTelemetryRpmSample_t rpmSamples[3];
static int16_t invalidPercent[3];
static unsigned adcReads, adcResets, rpmReads, clockReads, mixerCalls;
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
        std::memset(rpmSamples, 0, sizeof(rpmSamples));
        std::fill(invalidPercent, invalidPercent + 3, 0);
        adcReads = adcResets = rpmReads = clockReads = mixerCalls = 0;
        useDshotTelemetry = true;
        dshotMotorCount = 3;
        now = 123456;
    }
};

TEST_F(TricopterDiagnostics, LegacyModeDoesNotSampleOrOverwriteAnotherDebugMode)
{
    debugMode = DEBUG_GYRO_FILTERED;
    servosTricopterLog(1700);
    EXPECT_EQ(0u, adcReads + rpmReads + clockReads + adcResets);
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
