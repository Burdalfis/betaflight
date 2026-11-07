/* Synthetic equation tests; coefficients below are not aircraft calibration. */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "flight/tricopter_reaction_torque.h"
}

#include "gtest/gtest.h"

static constexpr double PI = 3.14159265358979323846;

static tricopterReactionTorqueParams_t syntheticParams()
{
    tricopterReactionTorqueParams_t p = {};
    for (unsigned i = 0; i < 3; ++i) {
        p.kQ[i] = 1e-7f;
        p.inertiaKgM2[i] = 2e-5f;
        p.reactionSign[i] = i == 1 ? 1 : -1;
    }
    p.coefficientSource = TRICOPTER_REACTION_TORQUE_COEFFICIENTS_SYNTHETIC_PLACEHOLDER;
    p.maxMechanicalRpm = 20000;
    p.maxAbsAccelerationRpmS = 100000;
    p.maxAbsTorqueNm = 2;
    p.maxRpmAgeUs = 10000;
    return p;
}

static std::array<tricopterReactionTorqueMotorState_t, 3> knownStates(float rpm = 0, float rpmS = 0)
{
    std::array<tricopterReactionTorqueMotorState_t, 3> states = {};
    for (auto &state : states) {
        state = {rpm, rpmS, 0, true, true, true, true};
    }
    return states;
}

static bool finiteBits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static void expectUnavailable(const tricopterReactionTorqueOutput_t &out)
{
    EXPECT_FALSE(out.valid); EXPECT_FALSE(out.coefficientsCalibrated); EXPECT_FALSE(out.forFlight);
    EXPECT_NE(out.status, 0U);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_EQ(out.omegaRadS[i], 0); EXPECT_EQ(out.alphaRadS2[i], 0);
        EXPECT_EQ(out.aerodynamicNm[i], 0); EXPECT_EQ(out.inertialNm[i], 0); EXPECT_EQ(out.totalNm[i], 0);
        EXPECT_TRUE(finiteBits(out.totalNm[i]));
    }
    EXPECT_EQ(out.summedAerodynamicNm, 0); EXPECT_EQ(out.summedInertialNm, 0); EXPECT_EQ(out.summedTotalNm, 0);
}

TEST(TricopterReactionTorque, RequiresExplicitSyntheticProvenanceAndValidSoftwareGuards)
{
    auto p = syntheticParams();
    EXPECT_TRUE(tricopterReactionTorqueValidateParams(&p));
    EXPECT_FALSE(tricopterReactionTorqueValidateParams(nullptr));
    const auto states = knownStates();
    expectUnavailable(tricopterReactionTorqueEvaluate(nullptr, states.data()));
    expectUnavailable(tricopterReactionTorqueEvaluate(&p, nullptr));
    for (unsigned field = 0; field < 9; ++field) {
        p = syntheticParams();
        switch (field) {
        case 0: p.coefficientSource = TRICOPTER_REACTION_TORQUE_COEFFICIENTS_UNSPECIFIED; break;
        case 1: p.reactionSign[0] = 0; break;
        case 2: p.kQ[0] = -1; break;
        case 3: p.inertiaKgM2[0] = -1; break;
        case 4: p.maxMechanicalRpm = 0; break;
        case 5: p.maxAbsAccelerationRpmS = 0; break;
        case 6: p.maxAbsTorqueNm = 0; break;
        case 7: p.maxRpmAgeUs = 0; break;
        case 8: p.maxRpmAgeUs = UINT32_C(0x80000000); break;
        }
        EXPECT_FALSE(tricopterReactionTorqueValidateParams(&p)) << field;
        expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
    }
}

TEST(TricopterReactionTorque, RejectsNonfiniteCoefficientsAndBoundsWithFastMath)
{
    const auto states = knownStates();
    for (float bad : {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        for (unsigned field = 0; field < 9; ++field) {
            auto p = syntheticParams();
            if (field < 3) p.kQ[field] = bad;
            else if (field < 6) p.inertiaKgM2[field - 3] = bad;
            else if (field == 6) p.maxMechanicalRpm = bad;
            else if (field == 7) p.maxAbsAccelerationRpmS = bad;
            else p.maxAbsTorqueNm = bad;
            EXPECT_FALSE(tricopterReactionTorqueValidateParams(&p));
            expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
        }
    }
}

TEST(TricopterReactionTorque, KnownStoppedRotorsGiveValidZeroWithoutFlightClaim)
{
    const auto p = syntheticParams();
    const auto states = knownStates();
    const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
    EXPECT_TRUE(out.valid); EXPECT_EQ(out.status, 0U); EXPECT_EQ(out.invalidMotorMask, 0U);
    EXPECT_EQ(out.summedTotalNm, 0); EXPECT_EQ(out.summedAerodynamicNm, 0); EXPECT_EQ(out.summedInertialNm, 0);
    EXPECT_FALSE(out.coefficientsCalibrated); EXPECT_FALSE(out.forFlight);
}

TEST(TricopterReactionTorque, ConvertsMechanicalRpmWithoutPoleCountMultiplication)
{
    const auto p = syntheticParams();
    const auto states = knownStates(60, -60);
    const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(out.valid);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.omegaRadS[i], 2 * PI, 1e-6);
        EXPECT_NEAR(out.alphaRadS2[i], -2 * PI, 1e-6);
        EXPECT_NEAR(out.aerodynamicNm[i], p.reactionSign[i] * p.kQ[i] * 4 * PI * PI, 1e-11);
        EXPECT_NEAR(out.inertialNm[i], -p.reactionSign[i] * p.inertiaKgM2[i] * 2 * PI, 1e-10);
    }
}

TEST(TricopterReactionTorque, EqualSymmetricThreeRotorsDoNotCancel)
{
    const auto p = syntheticParams();
    const auto states = knownStates(3000, 1000);
    const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(out.valid);
    const double omega = 3000 * PI / 30, alpha = 1000 * PI / 30;
    EXPECT_NEAR(out.summedAerodynamicNm, -p.kQ[0] * omega * omega, 1e-9);
    EXPECT_NEAR(out.summedInertialNm, -p.inertiaKgM2[0] * alpha, 1e-9);
    EXPECT_LT(out.summedTotalNm, 0);
    EXPECT_NEAR(out.summedTotalNm, out.totalNm[0], 1e-9);
}

TEST(TricopterReactionTorque, EveryLogicalMotorUsesItsExplicitReactionSign)
{
    auto p = syntheticParams();
    for (unsigned motor = 0; motor < 3; ++motor) {
        for (int sign : {-1, 1}) {
            p.reactionSign[motor] = sign;
            auto states = knownStates();
            states[motor].mechanicalRpm = 2000;
            states[motor].accelerationRpmS = 500;
            const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
            ASSERT_TRUE(out.valid);
            EXPECT_GT(sign * out.summedAerodynamicNm, 0);
            EXPECT_GT(sign * out.summedInertialNm, 0);
            EXPECT_NEAR(out.summedTotalNm, out.totalNm[motor], 1e-9);
        }
    }
}

TEST(TricopterReactionTorque, MotorPermutationPreservesSummedTorque)
{
    auto p = syntheticParams();
    auto states = knownStates();
    for (unsigned i = 0; i < 3; ++i) {
        p.kQ[i] *= i + 1; p.inertiaKgM2[i] *= 3 - i;
        states[i].mechanicalRpm = 1000 + 700 * i;
        states[i].accelerationRpmS = i == 1 ? -500 : 1000;
    }
    const auto expected = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(expected.valid);
    std::array<unsigned, 3> order = {0, 1, 2};
    do {
        auto permutedParams = p;
        auto permutedStates = states;
        for (unsigned i = 0; i < 3; ++i) {
            permutedParams.kQ[i] = p.kQ[order[i]];
            permutedParams.inertiaKgM2[i] = p.inertiaKgM2[order[i]];
            permutedParams.reactionSign[i] = p.reactionSign[order[i]];
            permutedStates[i] = states[order[i]];
        }
        const auto out = tricopterReactionTorqueEvaluate(&permutedParams, permutedStates.data());
        ASSERT_TRUE(out.valid);
        EXPECT_NEAR(out.summedTotalNm, expected.summedTotalNm, 1e-9);
        for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(out.totalNm[i], expected.totalNm[order[i]]);
    } while (std::next_permutation(order.begin(), order.end()));
}

TEST(TricopterReactionTorque, IsolatedInertiaChangesSignOnDeceleration)
{
    auto p = syntheticParams();
    for (float &kQ : p.kQ) kQ = 0;
    auto states = knownStates(2000, 1000);
    const auto acceleration = tricopterReactionTorqueEvaluate(&p, states.data());
    for (auto &state : states) state.accelerationRpmS *= -1;
    const auto deceleration = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(acceleration.valid); ASSERT_TRUE(deceleration.valid);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_EQ(acceleration.aerodynamicNm[i], 0);
        EXPECT_EQ(acceleration.inertialNm[i], -deceleration.inertialNm[i]);
    }
    EXPECT_EQ(acceleration.summedTotalNm, -deceleration.summedTotalNm);
}

TEST(TricopterReactionTorque, IntegratedInertialImpulseMatchesAngularMomentumChange)
{
    auto p = syntheticParams();
    for (float &kQ : p.kQ) kQ = 0;
    auto states = knownStates();
    double impulse = 0;
    const double dt = 0.001;
    for (unsigned i = 1; i <= 200; ++i) {
        states[0].mechanicalRpm = 1000 + i * 15.0f;
        states[0].accelerationRpmS = 15000;
        const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
        ASSERT_TRUE(out.valid);
        impulse += out.summedInertialNm * dt;
    }
    const double expected = -p.inertiaKgM2[0] * (4000 - 1000) * PI / 30;
    EXPECT_NEAR(impulse, expected, 1e-9);
    for (unsigned i = 1; i <= 200; ++i) {
        states[0].mechanicalRpm = 4000 - i * 15.0f;
        states[0].accelerationRpmS = -15000;
        const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
        ASSERT_TRUE(out.valid);
        impulse += out.summedInertialNm * dt;
    }
    EXPECT_NEAR(impulse, 0, 1e-12);
}

TEST(TricopterReactionTorque, CoastingCancelsMountTorqueAndStrongerBrakingReversesIt)
{
    const auto p = syntheticParams();
    auto states = knownStates(static_cast<float>(100 * 30 / PI));
    for (unsigned i = 0; i < 3; ++i) {
        const double omega = states[i].mechanicalRpm * PI / 30;
        states[i].accelerationRpmS = static_cast<float>(-p.kQ[i] * omega * omega / p.inertiaKgM2[i] * 30 / PI);
    }
    const auto coasting = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(coasting.valid);
    for (float torque : coasting.totalNm) EXPECT_NEAR(torque, 0, 1e-9);
    EXPECT_NEAR(coasting.summedTotalNm, 0, 1e-9);
    for (auto &state : states) state.accelerationRpmS *= 2;
    const auto braking = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(braking.valid);
    for (unsigned i = 0; i < 3; ++i) EXPECT_LT(p.reactionSign[i] * braking.totalNm[i], 0);
}

TEST(TricopterReactionTorque, AerodynamicQuadraticAndCoefficientLinearScaling)
{
    auto p = syntheticParams();
    auto states = knownStates(2000, 1000);
    const auto base = tricopterReactionTorqueEvaluate(&p, states.data());
    for (auto &state : states) state.mechanicalRpm *= 2;
    const auto speed2 = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(base.valid); ASSERT_TRUE(speed2.valid);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_EQ(speed2.aerodynamicNm[i], 4 * base.aerodynamicNm[i]);
        EXPECT_EQ(speed2.inertialNm[i], base.inertialNm[i]);
        p.kQ[i] *= 3; p.inertiaKgM2[i] *= 2;
    }
    const auto coefficients = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(coefficients.valid);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_NEAR(coefficients.aerodynamicNm[i], 3 * speed2.aerodynamicNm[i], 1e-8);
        EXPECT_EQ(coefficients.inertialNm[i], 2 * speed2.inertialNm[i]);
    }
}

TEST(TricopterReactionTorque, PositiveRotorSqrtTwoSpeedCancelsTwoEqualOpposingRotors)
{
    const auto p = syntheticParams();
    auto states = knownStates(2000);
    states[1].mechanicalRpm *= std::sqrt(2.0f);
    const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.summedTotalNm, 0, 2e-9);
}

TEST(TricopterReactionTorque, AnyUnavailableMotorSuppressesAllTermsWithoutPartialSum)
{
    const auto p = syntheticParams();
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned fault = 0; fault < 5; ++fault) {
            auto states = knownStates(3000, 2000);
            switch (fault) {
            case 0: states[i].seen = false; break;
            case 1: states[i].fresh = false; break;
            case 2: states[i].valid = false; break;
            case 3: states[i].accelerationValid = false; break;
            case 4: states[i].ageUs = p.maxRpmAgeUs + 1; break;
            }
            const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
            expectUnavailable(out); EXPECT_EQ(out.invalidMotorMask, 1U << i);
        }
    }
}

TEST(TricopterReactionTorque, UnknownAccelerationInvalidEvenWithZeroInertia)
{
    auto p = syntheticParams();
    auto states = knownStates(1000);
    for (float &j : p.inertiaKgM2) j = 0;
    states[2].accelerationValid = false;
    const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
    expectUnavailable(out);
    EXPECT_NE(out.status & TRICOPTER_REACTION_TORQUE_ACCELERATION_UNAVAILABLE, 0U);
}

TEST(TricopterReactionTorque, NumericalInputAndSoftwareRangeGuardsAreFinite)
{
    const auto p = syntheticParams();
    for (unsigned i = 0; i < 3; ++i) {
        for (float bad : {std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
            auto states = knownStates(1000, 10);
            states[i].mechanicalRpm = bad;
            expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
            states = knownStates(1000, 10); states[i].accelerationRpmS = bad;
            expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
        }
    }
    for (unsigned fault = 0; fault < 4; ++fault) {
        auto states = knownStates(1000, 10);
        if (fault == 0) states[0].mechanicalRpm = -1;
        if (fault == 1) states[0].mechanicalRpm = p.maxMechanicalRpm + 1;
        if (fault == 2) states[0].accelerationRpmS = p.maxAbsAccelerationRpmS + 1;
        if (fault == 3) states[0].accelerationRpmS = -p.maxAbsAccelerationRpmS - 1;
        const auto out = tricopterReactionTorqueEvaluate(&p, states.data());
        expectUnavailable(out); EXPECT_NE(out.status & TRICOPTER_REACTION_TORQUE_INPUT_BOUNDS, 0U);
    }
}

TEST(TricopterReactionTorque, FreshnessBoundaryIsInclusiveAndSentinelAgeRejected)
{
    const auto p = syntheticParams();
    auto states = knownStates();
    for (auto &state : states) state.ageUs = p.maxRpmAgeUs;
    EXPECT_TRUE(tricopterReactionTorqueEvaluate(&p, states.data()).valid);
    states[1].ageUs = UINT32_MAX;
    expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
}

TEST(TricopterReactionTorque, HugeFiniteCoefficientsCannotOverflowOrHideBehindCancellation)
{
    auto p = syntheticParams();
    auto states = knownStates(p.maxMechanicalRpm, p.maxAbsAccelerationRpmS);
    for (unsigned i = 0; i < 3; ++i) {
        p.kQ[i] = std::numeric_limits<float>::max();
        p.inertiaKgM2[i] = std::numeric_limits<float>::max();
    }
    EXPECT_TRUE(tricopterReactionTorqueValidateParams(&p));
    expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
    states = knownStates();
    EXPECT_TRUE(tricopterReactionTorqueEvaluate(&p, states.data()).valid);
    states = knownStates(1000, -1000);
    expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data()));
    // Exact opposing large aero terms would give zero net torque; their
    // individual guard failures must survive that apparent cancellation.
    p = syntheticParams();
    p.kQ[0] = p.kQ[1] = std::numeric_limits<float>::max();
    p.kQ[2] = 0;
    for (float &j : p.inertiaKgM2) j = 0;
    states = knownStates(1000);
    const auto canceled = tricopterReactionTorqueEvaluate(&p, states.data());
    expectUnavailable(canceled); EXPECT_EQ(canceled.invalidMotorMask, 3U);
}

TEST(TricopterReactionTorque, ComponentPerMotorAndGrandSumsEachRespectTorqueGuard)
{
    auto p = syntheticParams();
    p.maxAbsTorqueNm = 0.25f;
    auto states = knownStates(static_cast<float>(100 * 30 / PI));
    for (unsigned i = 0; i < 3; ++i) { p.kQ[i] = 1e-5f; p.reactionSign[i] = 1; }
    auto out = tricopterReactionTorqueEvaluate(&p, states.data()); // Each 0.1 Nm, grand sum 0.3 Nm.
    expectUnavailable(out); EXPECT_EQ(out.invalidMotorMask, 7U);
    p = syntheticParams(); p.maxAbsTorqueNm = 0.25f;
    states = knownStates(); states[0].mechanicalRpm = static_cast<float>(100 * 30 / PI);
    p.kQ[0] = 1.5e-5f; p.inertiaKgM2[0] = 0.0015f;
    states[0].accelerationRpmS = static_cast<float>(100 * 30 / PI);
    out = tricopterReactionTorqueEvaluate(&p, states.data()); // 0.15+0.15 Nm exceeds per-motor bound.
    expectUnavailable(out); EXPECT_NE(out.status & TRICOPTER_REACTION_TORQUE_TORQUE_BOUNDS, 0U);
    p.kQ[0] = 1e-4f; states[0].accelerationRpmS = 0;
    expectUnavailable(tricopterReactionTorqueEvaluate(&p, states.data())); // Single component too large.
}
