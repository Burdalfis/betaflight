/* Pure calibration state-machine tests; no actuator or persistent storage. */
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" {
#include "flight/tricopter_servo_calibration.h"
}
#include "gtest/gtest.h"

static tricopterServoCalibrationParams_t params()
{
    return {{1100, 1509, 1700}, {-40, 0, 20}};
}

// The measured trajectory is independent of the requested PWM ramp. This lets
// the speed tests distinguish observed feedback from command-derived speed.
static int16_t measured(const tricopterServoCalibrationState_t &s, uint32_t now,
    const int16_t adc[3], uint32_t movementUs)
{
    const float fraction = fminf(1, (float)(now - s.stageStartUs) / movementUs);
    float from = adc[1], to = adc[1];
    if (s.output.stage == TRICOPTER_SERVO_CALIBRATION_LOW) to = adc[0];
    if (s.output.stage == TRICOPTER_SERVO_CALIBRATION_HIGH) { from = adc[0]; to = adc[2]; }
    if (s.output.stage == TRICOPTER_SERVO_CALIBRATION_RETURN_CENTER) from = adc[2];
    return (int16_t)lroundf(from + fraction * (to - from));
}

static tricopterServoCalibrationOutput_t run(tricopterServoCalibrationState_t &s,
    const int16_t adc[3], uint32_t movementUs = 200000, bool noisy = false)
{
    uint32_t now = s.lastStepUs;
    for (unsigned i = 0; i < 2000 && s.output.active; ++i) {
        now += 5000;
        const auto old = s.output;
        int16_t count = measured(s, now, adc, movementUs);
        if (noisy) count += i % 2 ? 5 : -5;
        const auto out = tricopterServoCalibrationStep(&s, now, false, true, count);
        EXPECT_GE(out.commandedPwmUs, s.params.pwmUs[0]);
        EXPECT_LE(out.commandedPwmUs, s.params.pwmUs[2]);
        if (out.active) {
            EXPECT_LE(std::abs((int)out.commandedPwmUs - old.commandedPwmUs), 20);
        }
    }
    EXPECT_FALSE(s.output.active);
    return s.output;
}

TEST(TricopterServoCalibration, LearnsFreshSignedEndpointsAndObservedSpeed)
{
    tricopterServoCalibrationState_t s = {};
    auto p = params();
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
    p.pwmUs[0] = 600; // Start owns its parameter copy.
    const int16_t adc[] = {1900, 1500, 1200};
    const auto out = run(s, adc);
    ASSERT_TRUE(out.complete);
    EXPECT_TRUE(out.resultValid);
    EXPECT_FALSE(out.failed);
    EXPECT_TRUE(out.commandValid);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_COMPLETE, out.stage);
    EXPECT_EQ(1509, out.commandedPwmUs);
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(adc[i], out.learnedAdc[i]);
    EXPECT_EQ(1500, out.initialCenterAdc);
    EXPECT_TRUE(out.speedEstimateValid);
    EXPECT_NEAR(160000, out.observed10To90Us, 500);
    // Count10=1830 -> -33deg; count90=1270 -> 15 1/3deg.
    EXPECT_NEAR((33.0 + 46.0 / 3) / .16, out.observedSpeedDegPerSecond, 1.1);
    EXPECT_LE(out.traceSamples, TRICOPTER_SERVO_CALIBRATION_TRACE_CAPACITY);
    EXPECT_GT(out.traceSamples, 20);
    EXPECT_EQ(&s.output, tricopterServoCalibrationGetOutput(&s));
    EXPECT_LE(sizeof(s), 1200U);
}

TEST(TricopterServoCalibration, SpeedChangesWithObservedMovementNotCommandRamp)
{
    auto p = params();
    tricopterServoCalibrationState_t fast = {}, slow = {};
    ASSERT_TRUE(tricopterServoCalibrationStart(&fast, &p, 0, true));
    ASSERT_TRUE(tricopterServoCalibrationStart(&slow, &p, 0, true));
    const int16_t adc[] = {1900, 1500, 1200};
    const auto a = run(fast, adc, 200000);
    const auto b = run(slow, adc, 400000);
    ASSERT_TRUE(a.complete && b.complete);
    EXPECT_NEAR(2 * b.observedSpeedDegPerSecond, a.observedSpeedDegPerSecond, 1.5);
    EXPECT_NEAR(2 * a.observed10To90Us, b.observed10To90Us, 1000);
}

TEST(TricopterServoCalibration, AcceptsEitherAdcAndAngleDirectionAcrossTimerWrap)
{
    auto p = params();
    p.angleDeg[0] = 40; p.angleDeg[2] = -20;
    tricopterServoCalibrationState_t s = {};
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, UINT32_MAX - 2000000, true));
    const int16_t adc[] = {-1900, -1500, -1200};
    const auto out = run(s, adc, 200000, true);
    ASSERT_TRUE(out.complete);
    EXPECT_EQ(-1900, out.learnedAdc[0]);
    EXPECT_EQ(-1500, out.learnedAdc[1]);
    EXPECT_EQ(-1200, out.learnedAdc[2]);
    EXPECT_GT(out.observedSpeedDegPerSecond, 250);
}

TEST(TricopterServoCalibration, RejectsInvalidPwmAngleAndNonfiniteMaps)
{
    EXPECT_FALSE(tricopterServoCalibrationValidateParams(nullptr));
    for (unsigned which = 0; which < 7; ++which) {
        auto p = params();
        switch (which) {
        case 0: p.pwmUs[0] = 499; break;
        case 1: p.pwmUs[2] = 2501; break;
        case 2: p.pwmUs[1] = p.pwmUs[0]; break;
        case 3: p.angleDeg[1] = p.angleDeg[0]; break;
        case 4: p.angleDeg[1] = 50; break;
        case 5: p.angleDeg[1] = std::numeric_limits<float>::quiet_NaN(); break;
        case 6: p.angleDeg[1] = std::numeric_limits<float>::infinity(); break;
        }
        tricopterServoCalibrationState_t s = {};
        EXPECT_FALSE(tricopterServoCalibrationStart(&s, &p, 0, true)) << which;
        EXPECT_TRUE(s.output.failed);
        EXPECT_FALSE(s.output.commandValid);
        EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_INVALID_PARAMS, s.output.status);
    }
    EXPECT_FALSE(tricopterServoCalibrationStart(nullptr, nullptr, 0, true));
    EXPECT_EQ(nullptr, tricopterServoCalibrationGetOutput(nullptr));
    EXPECT_FALSE(tricopterServoCalibrationAbort(nullptr).commandValid);
}

TEST(TricopterServoCalibration, ArmingAndExplicitAbortStopImmediatelyAtBoundedCenter)
{
    auto p = params();
    tricopterServoCalibrationState_t s = {};
    EXPECT_FALSE(tricopterServoCalibrationStart(&s, &p, 0, false));
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_ARMED, s.output.status);
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
    auto out = tricopterServoCalibrationStep(&s, 5000, true, true, 1500);
    EXPECT_FALSE(out.active);
    EXPECT_FALSE(out.commandValid);
    EXPECT_EQ(1509, out.commandedPwmUs);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_ABORTED, out.stage);
    EXPECT_FALSE(out.resultValid);
    EXPECT_FALSE(tricopterServoCalibrationAbort(&s).commandValid);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_ARMED, s.output.status);
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 10000, true));
    EXPECT_FALSE(tricopterServoCalibrationStart(&s, &p, 20000, true));
    EXPECT_TRUE(s.output.active); // Busy Start does not clobber a running state.
    out = tricopterServoCalibrationAbort(&s);
    EXPECT_FALSE(out.active);
    EXPECT_TRUE(out.commandValid);
    EXPECT_EQ(1509, out.commandedPwmUs);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_USER_ABORT, out.status);
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &s.params, 20000, true));
    EXPECT_EQ(1509, s.params.pwmUs[1]); // In-state parameter alias is safe.
}

TEST(TricopterServoCalibration, MissingRailedAndSkippedFeedbackFailWithoutBlindMotion)
{
    for (unsigned mode = 0; mode < 3; ++mode) {
        auto p = params();
        tricopterServoCalibrationState_t s = {};
        ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
        if (mode == 2) {
            tricopterServoCalibrationStep(&s, 100001, false, true, 1500);
        } else {
            for (uint32_t t = 5000; t <= 105000; t += 5000) {
                tricopterServoCalibrationStep(&s, t, false, mode == 1, mode == 1 ? 4095 : 1500);
            }
        }
        EXPECT_FALSE(s.output.active);
        EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_FEEDBACK_TIMEOUT, s.output.status);
        EXPECT_EQ(1509, s.output.commandedPwmUs);
        EXPECT_FALSE(s.output.resultValid);
    }
}

TEST(TricopterServoCalibration, RejectsBackwardTimeAndUnstableEndpoint)
{
    auto p = params();
    tricopterServoCalibrationState_t s = {};
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 10000, true));
    auto out = tricopterServoCalibrationStep(&s, 9999, false, true, 1500);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_INVALID_TIME, out.status);
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
    for (uint32_t t = 5000; s.output.active && t <= 2100000; t += 5000) {
        tricopterServoCalibrationStep(&s, t, false, true, (t / 5000) % 2 ? 1400 : 1600);
    }
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_STAGE_TIMEOUT, s.output.status);
    EXPECT_FALSE(s.output.resultValid);
    EXPECT_EQ(1509, s.output.commandedPwmUs);
}

TEST(TricopterServoCalibration, RejectsNoMotionAndNonmonotonicFeedback)
{
    auto p = params();
    for (unsigned mode = 0; mode < 2; ++mode) {
        tricopterServoCalibrationState_t s = {};
        ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
        const int16_t adc[] = {1900, mode ? (int16_t)2200 : (int16_t)1500, 1200};
        const int16_t still[] = {1500, 1500, 1500};
        const auto out = run(s, mode ? adc : still);
        EXPECT_FALSE(out.complete);
        EXPECT_FALSE(out.resultValid);
        EXPECT_EQ(mode ? TRICOPTER_SERVO_CALIBRATION_NONMONOTONIC
            : TRICOPTER_SERVO_CALIBRATION_INSUFFICIENT_SPAN, out.status);
    }
}

TEST(TricopterServoCalibration, DoesNotPublishUnresolvedSpeedAsValidCalibration)
{
    auto p = params();
    tricopterServoCalibrationState_t s = {};
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
    const int16_t adc[] = {1900, 1500, 1200};
    const auto out = run(s, adc, 1); // One unresolved sample jump, not a measured move.
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_SPEED_UNAVAILABLE, out.status);
    EXPECT_FALSE(out.resultValid);
    EXPECT_FALSE(out.speedEstimateValid);
    EXPECT_EQ(1509, out.commandedPwmUs);
}

TEST(TricopterServoCalibration, AbortCancelsAnAlreadyCompletedResult)
{
    auto p = params();
    tricopterServoCalibrationState_t s = {};
    ASSERT_TRUE(tricopterServoCalibrationStart(&s, &p, 0, true));
    const int16_t adc[] = {1900, 1500, 1200};
    ASSERT_TRUE(run(s, adc).resultValid);
    const auto out = tricopterServoCalibrationAbort(&s);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_ABORTED, out.stage);
    EXPECT_EQ(TRICOPTER_SERVO_CALIBRATION_USER_ABORT, out.status);
    EXPECT_FALSE(out.active);
    EXPECT_FALSE(out.complete);
    EXPECT_FALSE(out.resultValid);
    EXPECT_FALSE(out.speedEstimateValid);
    EXPECT_EQ(0, out.observedSpeedDegPerSecond);
    EXPECT_TRUE(out.commandValid);
    EXPECT_EQ(1509, out.commandedPwmUs);
    // A later ordinary Step cannot resurrect the cancelled completion.
    EXPECT_FALSE(tricopterServoCalibrationStep(&s, s.lastStepUs + 5000, false, true, 1500).resultValid);
}
