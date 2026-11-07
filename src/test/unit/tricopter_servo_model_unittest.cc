/* Offline corrected-count servo predictor; no actuator or sensor dependencies. */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "flight/tricopter_servo_model.h"
}

#include "gtest/gtest.h"

static tricopterServoModelParams_t linearParams()
{
    tricopterServoModelParams_t params = {};
    params.knotCount = 2;
    params.pwmUs[0] = 1000.0f;
    params.pwmUs[1] = 2000.0f;
    params.positionCounts[0] = 0.0f;
    params.positionCounts[1] = 1000.0f;
    params.maxGapUs = 1000000;
    return params;
}

static bool finiteBits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static void expectFinite(const tricopterServoModelOutput_t &sample)
{
    EXPECT_TRUE(finiteBits(sample.positionCounts));
    EXPECT_TRUE(finiteBits(sample.velocityCountsS));
    EXPECT_FALSE(sample.angleCalibrated);
}

TEST(TricopterServoModel, ValidatesMapShapeAndParameterBounds)
{
    auto params = linearParams();
    EXPECT_TRUE(tricopterServoModelValidateParams(&params));
    EXPECT_FALSE(tricopterServoModelValidateParams(nullptr));
    params.knotCount = 1;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.knotCount = TRICOPTER_SERVO_MODEL_MAX_KNOTS + 1;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.pwmUs[1] = params.pwmUs[0];
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.positionCounts[1] = 4096;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.positionCounts[1] = 0;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.knotCount = 3;
    params.pwmUs[2] = 2500;
    params.positionCounts[2] = 999;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params.positionCounts[2] = 1000; // A plateau preserves monotonicity.
    EXPECT_TRUE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.backlashUs = 1001;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.tauIncreasingPwmS = 0.0000001f;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.tauDecreasingPwmS = 61;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.rateIncreasingPwmCountsS = -1;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
    params = linearParams();
    params.maxGapUs = 0;
    EXPECT_FALSE(tricopterServoModelValidateParams(&params));
}

TEST(TricopterServoModel, RejectsNonfiniteConfigurationEvenWithFastMathModelBuild)
{
    for (float bad : {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        for (unsigned field = 0; field < 7; ++field) {
            auto params = linearParams();
            switch (field) {
            case 0: params.pwmUs[1] = bad; break;
            case 1: params.positionCounts[0] = bad; break;
            case 2: params.backlashUs = bad; break;
            case 3: params.tauIncreasingPwmS = bad; break;
            case 4: params.tauDecreasingPwmS = bad; break;
            case 5: params.rateIncreasingPwmCountsS = bad; break;
            case 6: params.rateDecreasingPwmCountsS = bad; break;
            }
            EXPECT_FALSE(tricopterServoModelValidateParams(&params)) << field;
            tricopterServoModelState_t state;
            EXPECT_FALSE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
            auto sample = tricopterServoModelStep(&state, 1, 1000);
            EXPECT_FALSE(sample.valid);
            EXPECT_NE(0U, sample.status & TRICOPTER_SERVO_MODEL_INVALID_CONFIG);
            expectFinite(sample);
        }
    }
}

TEST(TricopterServoModel, StaticPiecewiseMapKeepsSignedCorrectedCountsAndCopiesConfig)
{
    auto params = linearParams();
    params.knotCount = 3;
    params.pwmUs[0] = 1100;
    params.pwmUs[1] = 1420;
    params.pwmUs[2] = 1700;
    params.positionCounts[0] = 1000;
    params.positionCounts[1] = 0;
    params.positionCounts[2] = -700;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 100, 1420, 25, 0));
    params.positionCounts[2] = 123; // Reset owns its configuration copy.
    auto sample = tricopterServoModelStep(&state, 100, 1560);
    EXPECT_TRUE(sample.valid);
    EXPECT_FLOAT_EQ(-350, sample.positionCounts);
    EXPECT_FLOAT_EQ(0, sample.velocityCountsS);
    sample = tricopterServoModelStep(&state, 101, 1260);
    EXPECT_FLOAT_EQ(500, sample.positionCounts);
    expectFinite(sample);
}

TEST(TricopterServoModel, DelayedCommandActsAtExactBoundaryWithIrregularCalls)
{
    auto params = linearParams();
    params.delayUs = 10000;
    params.rateIncreasingPwmCountsS = 1000;
    tricopterServoModelState_t sparse, dense;
    ASSERT_TRUE(tricopterServoModelReset(&sparse, &params, 0, 1000, 0, 0));
    ASSERT_TRUE(tricopterServoModelReset(&dense, &params, 0, 1000, 0, 0));
    EXPECT_FLOAT_EQ(0, tricopterServoModelStep(&sparse, 1000, 2000).positionCounts);
    EXPECT_FLOAT_EQ(0, tricopterServoModelStep(&dense, 1000, 2000).positionCounts);
    EXPECT_FLOAT_EQ(0, tricopterServoModelStep(&dense, 10000, 2000).positionCounts);
    auto boundary = tricopterServoModelStep(&dense, 11000, 2000);
    EXPECT_FLOAT_EQ(0, boundary.positionCounts);
    EXPECT_FLOAT_EQ(1000, boundary.velocityCountsS);
    const auto sparseEnd = tricopterServoModelStep(&sparse, 16000, 2000);
    const auto denseEnd = tricopterServoModelStep(&dense, 16000, 2000);
    EXPECT_NEAR(5, sparseEnd.positionCounts, 0.0001f);
    EXPECT_NEAR(sparseEnd.positionCounts, denseEnd.positionCounts, 0.0001f);
}

TEST(TricopterServoModel, SolvesSlewLimitedLagAndExponentialTail)
{
    auto params = linearParams();
    params.tauIncreasingPwmS = 0.1f;
    params.rateIncreasingPwmCountsS = 2000;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&state, 0, 2000);
    auto sample = tricopterServoModelStep(&state, 200000, 2000);
    EXPECT_NEAR(400, sample.positionCounts, 0.001f);
    EXPECT_NEAR(2000, sample.velocityCountsS, 0.001f);
    sample = tricopterServoModelStep(&state, 450000, 2000);
    EXPECT_NEAR(1000 - 200 * std::exp(-0.5), sample.positionCounts, 0.001f);
    EXPECT_NEAR(2000 * std::exp(-0.5), sample.velocityCountsS, 0.01f);
    EXPECT_LE(sample.velocityCountsS, 2000);
    expectFinite(sample);
}

TEST(TricopterServoModel, UnlimitedSlewLagAndSlewOnlyHaveDistinctLimits)
{
    auto params = linearParams();
    params.tauIncreasingPwmS = 0.1f;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&state, 0, 2000);
    auto sample = tricopterServoModelStep(&state, 100000, 2000);
    EXPECT_NEAR(1000 * (1 - std::exp(-1.0)), sample.positionCounts, 0.001f);
    params.tauIncreasingPwmS = 0;
    params.rateIncreasingPwmCountsS = 2000;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&state, 0, 2000);
    sample = tricopterServoModelStep(&state, 600000, 2000);
    EXPECT_FLOAT_EQ(1000, sample.positionCounts);
    EXPECT_FLOAT_EQ(0, sample.velocityCountsS);
}

TEST(TricopterServoModel, DirectionParametersReferToPwmEvenWhenCountsDecrease)
{
    auto params = linearParams();
    params.positionCounts[0] = 1000;
    params.positionCounts[1] = 0;
    params.rateIncreasingPwmCountsS = 1000;
    params.rateDecreasingPwmCountsS = 2000;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 1000, 0));
    tricopterServoModelStep(&state, 0, 2000);
    auto sample = tricopterServoModelStep(&state, 100000, 2000);
    EXPECT_FLOAT_EQ(900, sample.positionCounts);
    EXPECT_FLOAT_EQ(-1000, sample.velocityCountsS);
    tricopterServoModelStep(&state, 100000, 1000);
    sample = tricopterServoModelStep(&state, 125000, 1000);
    EXPECT_FLOAT_EQ(950, sample.positionCounts);
    EXPECT_FLOAT_EQ(2000, sample.velocityCountsS);
}

TEST(TricopterServoModel, PlayMemoryCrossesReversalsWithoutInstantDirectionOffset)
{
    auto params = linearParams();
    params.backlashUs = 100;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1400, 350, 1));
    EXPECT_FLOAT_EQ(1350, state.playedPwmUs);
    EXPECT_FLOAT_EQ(550, tricopterServoModelStep(&state, 1, 1600).positionCounts);
    EXPECT_FLOAT_EQ(550, tricopterServoModelStep(&state, 2, 1570).positionCounts);
    EXPECT_FLOAT_EQ(550, tricopterServoModelStep(&state, 3, 1500).positionCounts);
    EXPECT_FLOAT_EQ(540, tricopterServoModelStep(&state, 4, 1490).positionCounts);
    EXPECT_FLOAT_EQ(540, tricopterServoModelStep(&state, 5, 1500).positionCounts);
    EXPECT_FLOAT_EQ(550, tricopterServoModelStep(&state, 6, 1600).positionCounts);
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1400, 450, -1));
    EXPECT_FLOAT_EQ(1450, state.playedPwmUs);
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1400, 400, 0));
    EXPECT_FLOAT_EQ(1400, state.playedPwmUs);
}

TEST(TricopterServoModel, DelayPreservesOrderedZeroDtCommandsAndCoalescesRepeats)
{
    auto params = linearParams();
    params.delayUs = 100;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&state, 0, 1200);
    tricopterServoModelStep(&state, 0, 1200);
    EXPECT_EQ(1U, state.commandCount);
    tricopterServoModelStep(&state, 0, 1800);
    EXPECT_EQ(2U, state.commandCount);
    EXPECT_FLOAT_EQ(0, tricopterServoModelStep(&state, 99, 1800).positionCounts);
    EXPECT_FLOAT_EQ(800, tricopterServoModelStep(&state, 100, 1800).positionCounts);
    EXPECT_EQ(0U, state.commandCount);
}

TEST(TricopterServoModel, HistoryIsInvariantToExtraUnchangedPwmSamples)
{
    auto params = linearParams();
    params.delayUs = 7500;
    params.backlashUs = 80;
    params.tauIncreasingPwmS = 0.03f;
    params.tauDecreasingPwmS = 0.05f;
    params.rateIncreasingPwmCountsS = 7000;
    params.rateDecreasingPwmCountsS = 5000;
    tricopterServoModelState_t sparse, dense;
    ASSERT_TRUE(tricopterServoModelReset(&sparse, &params, 0, 1000, 0, 0));
    ASSERT_TRUE(tricopterServoModelReset(&dense, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&sparse, 1000, 2000);
    tricopterServoModelStep(&sparse, 44000, 1300);
    const auto sparseEnd = tricopterServoModelStep(&sparse, 150000, 1300);
    for (uint64_t now = 1000; now <= 150000; now += 1000) {
        tricopterServoModelStep(&dense, now, now < 44000 ? 2000 : 1300);
    }
    const auto denseEnd = tricopterServoModelStep(&dense, 150000, 1300);
    EXPECT_NEAR(sparseEnd.positionCounts, denseEnd.positionCounts, 0.002f);
    EXPECT_NEAR(sparseEnd.velocityCountsS, denseEnd.velocityCountsS, 0.1f);
}

TEST(TricopterServoModel, InvalidCommandLatchesFiniteOutputUntilExplicitReset)
{
    auto params = linearParams();
    for (float bad : {0.0f, 999.0f, 2001.0f, std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        tricopterServoModelState_t state;
        ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 123, 0));
        auto sample = tricopterServoModelStep(&state, 1, bad);
        EXPECT_FALSE(sample.valid);
        EXPECT_EQ(TRICOPTER_SERVO_MODEL_INVALID_PWM, sample.status);
        EXPECT_FLOAT_EQ(123, sample.positionCounts);
        expectFinite(sample);
        sample = tricopterServoModelStep(&state, 2, 2000);
        EXPECT_FALSE(sample.valid);
        EXPECT_FLOAT_EQ(123, sample.positionCounts);
        ASSERT_TRUE(tricopterServoModelReset(&state, &params, 2, 1000, -123, 0));
        EXPECT_TRUE(tricopterServoModelStep(&state, 2, 1000).valid);
    }
}

TEST(TricopterServoModel, ResetRequiresValidInitialSamplePwmAndApproach)
{
    auto params = linearParams();
    tricopterServoModelState_t state;
    EXPECT_FALSE(tricopterServoModelReset(nullptr, &params, 0, 1000, 0, 0));
    auto sample = tricopterServoModelStep(nullptr, 0, 1000);
    EXPECT_FALSE(sample.valid);
    expectFinite(sample);
    for (float bad : {4096.0f, -4096.0f, std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity()}) {
        EXPECT_FALSE(tricopterServoModelReset(&state, &params, 0, 1000, bad, 0));
        sample = tricopterServoModelStep(&state, 0, 1000);
        EXPECT_NE(0U, sample.status & TRICOPTER_SERVO_MODEL_INVALID_POSITION);
        expectFinite(sample);
    }
    EXPECT_FALSE(tricopterServoModelReset(&state, &params, 0, 0, 0, 0));
    EXPECT_FALSE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 2));
    EXPECT_FALSE(tricopterServoModelReset(&state, &params, 0, 1000, 0, -2));
}

TEST(TricopterServoModel, RejectsBackwardOverflowAndUnobservedCommandGaps)
{
    auto params = linearParams();
    params.delayUs = 500;
    params.maxGapUs = 1000;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 100, 1000, 0, 0));
    auto sample = tricopterServoModelStep(&state, 99, 2000);
    EXPECT_EQ(TRICOPTER_SERVO_MODEL_INVALID_TIME, sample.status);
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, UINT64_MAX - 10, 1000, 0, 0));
    sample = tricopterServoModelStep(&state, UINT64_MAX - 10, 2000);
    EXPECT_EQ(TRICOPTER_SERVO_MODEL_INVALID_TIME, sample.status);
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&state, 0, 2000); // Event at 500 cannot hide a 2000 us gap.
    sample = tricopterServoModelStep(&state, 2000, 2000);
    EXPECT_EQ(TRICOPTER_SERVO_MODEL_COMMAND_GAP, sample.status);
    EXPECT_FLOAT_EQ(0, sample.positionCounts);
    expectFinite(sample);
    // Large absolute time is preserved; only interval/delay arithmetic matters.
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, UINT64_MAX - 2000, 1000, 0, 0));
    sample = tricopterServoModelStep(&state, UINT64_MAX - 1000, 1000);
    EXPECT_TRUE(sample.valid);
}

TEST(TricopterServoModel, BoundedQueueCannotSilentlyDropCommandHistory)
{
    auto params = linearParams();
    params.delayUs = 100000;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    for (unsigned i = 0; i < TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY; ++i) {
        EXPECT_TRUE(tricopterServoModelStep(&state, i, i % 2 ? 1000 : 2000).valid);
    }
    auto sample = tricopterServoModelStep(&state, TRICOPTER_SERVO_MODEL_COMMAND_CAPACITY, 2000);
    EXPECT_FALSE(sample.valid);
    EXPECT_EQ(TRICOPTER_SERVO_MODEL_QUEUE_FULL, sample.status);
    EXPECT_FLOAT_EQ(0, sample.velocityCountsS);
    expectFinite(sample);
}

TEST(TricopterServoModel, ExplicitResetCanReuseRetainedParametersAfterFault)
{
    auto params = linearParams();
    params.delayUs = 100;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, 0, 0));
    tricopterServoModelStep(&state, 0, 2000);
    EXPECT_FALSE(tricopterServoModelStep(&state, 1, 0).valid);
    ASSERT_TRUE(tricopterServoModelReset(&state, &state.params, 2, 1500, 500, -1));
    EXPECT_EQ(0U, state.commandCount);
    EXPECT_EQ(100U, state.params.delayUs);
    const auto sample = tricopterServoModelStep(&state, 2, 1500);
    EXPECT_TRUE(sample.valid);
    EXPECT_FLOAT_EQ(500, sample.positionCounts);
}

TEST(TricopterServoModel, AcceptedNumericExtremesProduceFiniteBoundedPredictions)
{
    auto params = linearParams();
    params.positionCounts[0] = -4095;
    params.positionCounts[1] = 4095;
    params.tauIncreasingPwmS = 0.000001f;
    params.tauDecreasingPwmS = 60;
    params.rateIncreasingPwmCountsS = 1e9f;
    params.rateDecreasingPwmCountsS = 0;
    params.maxGapUs = UINT32_MAX;
    tricopterServoModelState_t state;
    ASSERT_TRUE(tricopterServoModelReset(&state, &params, 0, 1000, -4095, 0));
    auto sample = tricopterServoModelStep(&state, 0, 2000);
    expectFinite(sample);
    EXPECT_LE(std::fabs(sample.velocityCountsS), 1e9f);
    sample = tricopterServoModelStep(&state, UINT32_MAX, 2000);
    expectFinite(sample);
    EXPECT_FLOAT_EQ(4095, sample.positionCounts);
    sample = tricopterServoModelStep(&state, UINT32_MAX, 1000);
    expectFinite(sample);
    EXPECT_LT(sample.velocityCountsS, 0);
    sample = tricopterServoModelStep(&state, UINT64_C(2) * UINT32_MAX, 1000);
    expectFinite(sample);
    EXPECT_GE(sample.positionCounts, -4095);
    EXPECT_LE(sample.positionCounts, 4095);
}
