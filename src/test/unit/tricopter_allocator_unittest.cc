/* Offline synthetic allocation tests; no characterized aircraft constants. */
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "flight/tricopter_allocator.h"
}

#include "gtest/gtest.h"

static constexpr double PI = 3.14159265358979323846;

static tricopterAllocatorParams_t geometry()
{
    tricopterAllocatorParams_t p = {};
    const float positions[3][3] = {{-.325f, .015f, -.04f}, {.125f, -.225f, -.04f}, {.145f, .275f, -.04f}};
    std::memcpy(p.motorPositionM, positions, sizeof(positions));
    const float maxima[3] = {8, 7.5f, 8.5f};
    const float coefficients[3] = {1.1e-5f, 1.2e-5f, 1.15e-5f};
    std::memcpy(p.maxThrustN, maxima, sizeof(maxima));
    std::memcpy(p.kT, coefficients, sizeof(coefficients));
    p.tailAngleMinRad = static_cast<float>(-25 * PI / 180);
    p.tailAngleMaxRad = static_cast<float>(35 * PI / 180);
    p.wrenchScale[0] = 1; p.wrenchScale[1] = p.wrenchScale[2] = .2f; p.wrenchScale[3] = .1f;
    p.lowTailThrustN = 1e-5f; p.projectedGradientTolerance = 1e-5f;
    p.maxConditionNumber = 1e6f; p.maxIterations = 4096;
    p.source = TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER;
    return p;
}

static tricopterAllocatorVirtualParams_t identityMap()
{
    tricopterAllocatorVirtualParams_t p = {};
    for (unsigned i = 0; i < 4; ++i) { p.effectivenessMatrix[i][i] = 1; p.responseScale[i] = 1; }
    for (float &value : p.maxEffort) value = 10;
    p.tailDirectionMinRad = static_cast<float>(-PI / 4);
    p.tailDirectionMaxRad = static_cast<float>(PI / 4);
    p.lowTailEffort = 1e-5f; p.projectedGradientTolerance = 1e-5f;
    p.maxConditionNumber = 1e6f; p.maxIterations = 4096;
    p.source = TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER;
    return p;
}

// Independent r cross F oracle; does not call allocator forward/matrix code.
static std::array<double, 4> wrench(const tricopterAllocatorParams_t &p, const float thrust[3], double angle, double reaction = 0)
{
    std::array<double, 4> result = {0, 0, 0, reaction};
    for (unsigned motor = 0; motor < 3; ++motor) {
        const double fy = motor == 0 ? -thrust[motor] * std::sin(angle) : 0;
        const double fz = motor == 0 ? thrust[motor] * std::cos(angle) : thrust[motor];
        result[0] += fz;
        result[1] += p.motorPositionM[motor][1] * fz - p.motorPositionM[motor][2] * fy;
        result[2] -= p.motorPositionM[motor][0] * fz;
        result[3] += p.motorPositionM[motor][0] * fy;
    }
    return result;
}

static tricopterAllocatorRequest_t requestFrom(const std::array<double, 4> &value)
{
    tricopterAllocatorRequest_t r = {};
    for (unsigned i = 0; i < 4; ++i) r.wrench[i] = static_cast<float>(value[i]);
    return r;
}

static bool finiteBits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static void expectUnavailable(const tricopterAllocatorOutput_t &out)
{
    EXPECT_FALSE(out.valid); EXPECT_FALSE(out.calibrated); EXPECT_FALSE(out.forFlight);
    EXPECT_NE(out.status, 0U); EXPECT_EQ(out.cost, 0); EXPECT_EQ(out.conditionNumber, 0);
    for (unsigned i = 0; i < 4; ++i) {
        EXPECT_EQ(out.controlsN[i], 0); EXPECT_EQ(out.achievedWrench[i], 0);
        EXPECT_EQ(out.requestedWrench[i], 0); EXPECT_EQ(out.residualWrench[i], 0);
    }
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_EQ(out.thrustN[i], 0); EXPECT_EQ(out.omegaRadS[i], 0); EXPECT_EQ(out.mechanicalRpm[i], 0);
        EXPECT_EQ(out.forceN[i], 0); EXPECT_TRUE(finiteBits(out.forceN[i]));
    }
}

static void expectFeasible(const tricopterAllocatorParams_t &p, const tricopterAllocatorOutput_t &out)
{
    ASSERT_TRUE(out.valid);
    EXPECT_GE(out.tailAngleRad, p.tailAngleMinRad); EXPECT_LE(out.tailAngleRad, p.tailAngleMaxRad);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_GE(out.thrustN[i], 0); EXPECT_LE(out.thrustN[i], p.maxThrustN[i]);
        EXPECT_TRUE(finiteBits(out.thrustN[i]));
    }
    const auto independent = wrench(p, out.thrustN, out.tailAngleRad);
    for (unsigned i = 0; i < 3; ++i) EXPECT_NEAR(out.achievedWrench[i], independent[i], 2e-6);
    EXPECT_FALSE(out.calibrated); EXPECT_FALSE(out.forFlight);
}

TEST(TricopterAllocator, ExactAsymmetricGeometryReconstructsFeasibleWrenches)
{
    const auto p = geometry();
    ASSERT_TRUE(tricopterAllocatorValidateParams(&p));
    for (unsigned i = 0; i < 20; ++i) {
        const float thrust[3] = {1.1f + i * .12f, 2.3f, 3.7f};
        const float angle = -.2f + i * .02f;
        const auto target = wrench(p, thrust, angle);
        const auto r = requestFrom(target);
        const auto out = tricopterAllocatorAllocate(&p, &r);
        expectFeasible(p, out); EXPECT_TRUE(out.exact); EXPECT_TRUE(out.converged); EXPECT_EQ(out.iterations, 0U);
        for (unsigned j = 0; j < 4; ++j) EXPECT_NEAR(out.achievedWrench[j], target[j], 2e-6);
        for (unsigned j = 0; j < 3; ++j) EXPECT_NEAR(out.thrustN[j], thrust[j], 2e-5);
        EXPECT_NEAR(out.tailAngleRad, angle, 1e-6);
    }
}

TEST(TricopterAllocator, ForwardUsesActualAngleIncludingCgHeightAndLateralForce)
{
    const auto p = geometry();
    const float thrust[3] = {3, 4, 2};
    const float angle = .3f;
    const auto expected = wrench(p, thrust, angle);
    const auto out = tricopterAllocatorForward(&p, thrust, angle, 0, false, false);
    expectFeasible(p, out);
    for (unsigned i = 0; i < 4; ++i) EXPECT_NEAR(out.achievedWrench[i], expected[i], 1e-6);
    EXPECT_LT(out.forceN[1], 0); EXPECT_GT(out.achievedWrench[3], 0);
    auto noHeight = p; noHeight.motorPositionM[0][2] = 0;
    const auto flat = tricopterAllocatorForward(&noHeight, thrust, angle, 0, false, false);
    EXPECT_NEAR(out.achievedWrench[1] - flat.achievedWrench[1], p.motorPositionM[0][2] * thrust[0] * std::sin(angle), 1e-7);
    const auto fixedDifferentAngle = tricopterAllocatorForward(&p, thrust, -.1f, 0, false, false);
    EXPECT_LT(fixedDifferentAngle.achievedWrench[3], 0);
    EXPECT_NE(out.achievedWrench[0], fixedDifferentAngle.achievedWrench[0]);
}

TEST(TricopterAllocator, KnownReactionOffsetIsRemovedBeforeSolveAndAddedToAchieved)
{
    const auto p = geometry();
    const float thrust[3] = {3, 2, 4}; const float angle = .2f;
    const auto target = wrench(p, thrust, angle, .1);
    auto r = requestFrom(target); r.reactionYawNm = .1f; r.useReactionYaw = r.reactionYawValid = true;
    const auto out = tricopterAllocatorAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.exact);
    EXPECT_NEAR(out.tailAngleRad, angle, 1e-6);
    EXPECT_NEAR(out.achievedWrench[3], target[3], 1e-6);
    const auto forward = tricopterAllocatorForward(&p, out.thrustN, out.tailAngleRad, .1f, true, true);
    for (unsigned i = 0; i < 4; ++i) EXPECT_NEAR(forward.achievedWrench[i], out.achievedWrench[i], 1e-7);
    r.reactionYawValid = false;
    expectUnavailable(tricopterAllocatorAllocate(&p, &r));
}

TEST(TricopterAllocator, ZeroAndLowTailPolicyPreservesFiniteMeaningfulDiagnostics)
{
    const auto p = geometry(); const tricopterAllocatorRequest_t r = {};
    const auto stopped = tricopterAllocatorAllocate(&p, &r);
    expectFeasible(p, stopped); EXPECT_EQ(stopped.tailAngleRad, 0); EXPECT_FALSE(stopped.tailAngleDefined);
    EXPECT_TRUE(stopped.tailThrustLow); EXPECT_EQ(stopped.cost, 0);
    EXPECT_NE(stopped.constraints & TRICOPTER_ALLOCATOR_TAIL_ZERO, 0U);
    const float thrust[3] = {1e-8f, 0, 0};
    const auto low = tricopterAllocatorForward(&p, thrust, .2f, 0, false, false);
    ASSERT_TRUE(low.valid); EXPECT_TRUE(low.tailAngleDefined); EXPECT_TRUE(low.tailThrustLow);
    EXPECT_NEAR(low.tailAngleRad, .2f, 1e-7);
    EXPECT_GT(low.controlsN[3], 0);
}

TEST(TricopterAllocator, ExplicitKtOnlyConvertsSiThrustToMechanicalSpeed)
{
    auto p = geometry(); const float thrust[3] = {3, 2, 4};
    const auto a = tricopterAllocatorForward(&p, thrust, .1f, 0, false, false);
    for (float &coefficient : p.kT) coefficient *= 4;
    const auto b = tricopterAllocatorForward(&p, thrust, .1f, 0, false, false);
    ASSERT_TRUE(a.valid); ASSERT_TRUE(b.valid);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_NEAR(a.omegaRadS[i], 2 * b.omegaRadS[i], 1e-5);
        EXPECT_NEAR(a.mechanicalRpm[i], a.omegaRadS[i] * 30 / PI, .001);
    }
    for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(a.achievedWrench[i], b.achievedWrench[i]);
}

TEST(TricopterAllocator, SaturationRemainsFeasibleAndBeatsObviousFeasibleControls)
{
    const auto p = geometry(); tricopterAllocatorRequest_t r = {{50, 2, -1, 8}, 0, false, false};
    const auto out = tricopterAllocatorAllocate(&p, &r);
    expectFeasible(p, out); EXPECT_FALSE(out.exact);
    auto independentCost = [&](const float t[3], double angle) {
        const auto achieved = wrench(p, t, angle); double cost = 0;
        for (unsigned i = 0; i < 4; ++i) cost += .5 * std::pow((achieved[i] - r.wrench[i]) / p.wrenchScale[i], 2);
        return cost;
    };
    const float zero[3] = {0, 0, 0};
    EXPECT_LE(out.cost, independentCost(zero, 0) + .001);
    EXPECT_LE(out.cost, independentCost(p.maxThrustN, p.tailAngleMaxRad) + .001);
    EXPECT_NEAR(out.cost, independentCost(out.thrustN, out.tailAngleRad), .001);
    for (unsigned i = 0; i < 4; ++i) EXPECT_NEAR(out.residualWrench[i], out.achievedWrench[i] - r.wrench[i], 4e-6);
}

TEST(TricopterAllocator, ConeProjectionPrecedesRadialCap)
{
    auto p = identityMap(); const tricopterAllocatorVirtualRequest_t r = {{0, 0, 0, 2}};
    auto out = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.converged);
    EXPECT_NEAR(out.controls[2], 1, 1e-6); EXPECT_NEAR(out.controls[3], 1, 1e-6);
    p.maxEffort[0] = 1;
    out = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.converged);
    EXPECT_EQ(out.effort[0], 1); EXPECT_NEAR(out.controls[2], std::sqrt(.5), 1e-6);
    EXPECT_NEAR(out.controls[3], std::sqrt(.5), 1e-6);
    EXPECT_NE(out.constraints & TRICOPTER_ALLOCATOR_ANGLE_MAX, 0U);
    EXPECT_NE(out.constraints & TRICOPTER_ALLOCATOR_TAIL_MAX, 0U);
}

TEST(TricopterAllocator, VirtualMapRequiresNeitherGeometryNorPhysicalCoefficients)
{
    const auto p = identityMap(); const tricopterAllocatorVirtualRequest_t r = {{2, 3, 4, .5f}};
    ASSERT_TRUE(tricopterAllocatorVirtualValidateParams(&p));
    const auto out = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.exact); EXPECT_TRUE(out.converged);
    for (unsigned i = 0; i < 4; ++i) EXPECT_NEAR(out.achievedResponse[i], r.response[i], 1e-6);
    EXPECT_EQ(out.effort[1], 2); EXPECT_EQ(out.effort[2], 3);
    EXPECT_FALSE(out.calibrated); EXPECT_FALSE(out.forFlight);
}

static tricopterAllocatorVirtualParams_t denseMap()
{
    auto p = identityMap();
    const float matrix[4][4] = {{1, .8f, 1.2f, .05f}, {.1f, -.3f, .2f, .5f}, {-.3f, .25f, .8f, .1f}, {.05f, .1f, -.15f, .7f}};
    std::memcpy(p.effectivenessMatrix, matrix, sizeof(matrix));
    p.maxEffort[0] = 3; p.maxEffort[1] = 2; p.maxEffort[2] = 4;
    return p;
}

TEST(TricopterAllocator, FullyCoupledMapAndResponseUnitScalingPreserveAllocation)
{
    auto p = denseMap(); tricopterAllocatorVirtualRequest_t r = {{8, -1, 2, 2}};
    const auto base = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(base.valid); EXPECT_TRUE(base.converged);
    const float scales[4] = {10, .1f, 100, .5f};
    for (unsigned i = 0; i < 4; ++i) {
        r.response[i] *= scales[i]; p.responseScale[i] *= scales[i];
        for (unsigned j = 0; j < 4; ++j) p.effectivenessMatrix[i][j] *= scales[i];
    }
    const auto changed = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(changed.valid); EXPECT_TRUE(changed.converged);
    for (unsigned i = 0; i < 4; ++i) EXPECT_NEAR(changed.controls[i], base.controls[i], 2e-5);
    EXPECT_NEAR(changed.cost, base.cost, 2e-5);
}

TEST(TricopterAllocator, EffortUnitChangePreservesResponseAndDirection)
{
    auto p = denseMap(); const tricopterAllocatorVirtualRequest_t r = {{8, -1, 2, 2}};
    const auto base = tricopterAllocatorVirtualAllocate(&p, &r);
    for (float &maximum : p.maxEffort) maximum *= 10;
    p.lowTailEffort *= 10; p.projectedGradientTolerance /= 10;
    for (auto &row : p.effectivenessMatrix) for (float &entry : row) entry /= 10;
    const auto changed = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(base.valid); ASSERT_TRUE(changed.valid);
    for (unsigned i = 0; i < 4; ++i) {
        EXPECT_NEAR(changed.controls[i] / 10, base.controls[i], 2e-5);
        EXPECT_NEAR(changed.achievedResponse[i], base.achievedResponse[i], 2e-5);
    }
    EXPECT_NEAR(changed.tailDirectionRad, base.tailDirectionRad, 1e-5);
}

TEST(TricopterAllocator, BoundedIterationsExposeUnconvergedFeasibleResult)
{
    auto p = denseMap(); p.maxIterations = 1;
    const tricopterAllocatorVirtualRequest_t r = {{1.5f, 1, -.3f, .6f}};
    const auto out = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_FALSE(out.converged); EXPECT_EQ(out.iterations, 1U);
    EXPECT_NE(out.status & TRICOPTER_ALLOCATOR_NOT_CONVERGED, 0U);
    EXPECT_GT(out.projectedGradientNorm, p.projectedGradientTolerance);
    for (unsigned i = 0; i < 3; ++i) { EXPECT_GE(out.effort[i], 0); EXPECT_LE(out.effort[i], p.maxEffort[i]); }
}

TEST(TricopterAllocator, ZeroWidthDirectionSectorAndFrontBoundsRemainConvex)
{
    auto p = identityMap(); p.tailDirectionMinRad = p.tailDirectionMaxRad = 0;
    p.maxEffort[1] = 2;
    const tricopterAllocatorVirtualRequest_t r = {{4, -1, 3, 2}};
    const auto out = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.converged);
    EXPECT_EQ(out.effort[1], 2); EXPECT_EQ(out.effort[2], 0);
    EXPECT_EQ(out.tailDirectionRad, 0); EXPECT_EQ(out.controls[3], 0);
    EXPECT_NEAR(out.controls[2], 3, 1e-6);
}

TEST(TricopterAllocator, FloatUnderflowedTailEffortHasZeroUndefinedDirection)
{
    auto p = identityMap();
    for (unsigned i = 0; i < 4; ++i) p.effectivenessMatrix[i][i] = 1e6f;
    const tricopterAllocatorVirtualRequest_t r = {{0, 0, 1e-40f, 1e-40f}};
    const auto out = tricopterAllocatorVirtualAllocate(&p, &r);
    ASSERT_TRUE(out.valid); EXPECT_EQ(out.effort[0], 0); EXPECT_EQ(out.tailDirectionRad, 0);
    EXPECT_FALSE(out.tailDirectionDefined); EXPECT_TRUE(out.tailEffortLow);
}

TEST(TricopterAllocator, RejectsDegenerateAndIllConditionedGeometryAndTinyWeightedMap)
{
    auto p = geometry(); p.motorPositionM[0][0] = 0;
    tricopterAllocatorRequest_t request = {};
    auto out = tricopterAllocatorAllocate(&p, &request);
    expectUnavailable(out); EXPECT_NE(out.status & TRICOPTER_ALLOCATOR_DEGENERATE_GEOMETRY, 0U);
    p = geometry();
    p.motorPositionM[2][0] = p.motorPositionM[1][0]; p.motorPositionM[2][1] = p.motorPositionM[1][1];
    p.motorPositionM[2][2] = 1;
    expectUnavailable(tricopterAllocatorAllocate(&p, &request));
    p = geometry(); p.motorPositionM[0][0] = -1e-8f;
    out = tricopterAllocatorAllocate(&p, &request);
    expectUnavailable(out); EXPECT_NE(out.status & TRICOPTER_ALLOCATOR_ILL_CONDITIONED, 0U);
    auto v = identityMap();
    for (unsigned i = 0; i < 4; ++i) v.effectivenessMatrix[i][i] = 1e-20f;
    EXPECT_FALSE(tricopterAllocatorVirtualValidateParams(&v));
}

TEST(TricopterAllocator, PhysicalAndGenericParameterGuardsSurviveFastMath)
{
    tricopterAllocatorRequest_t r = {};
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        for (unsigned field = 0; field < 9; ++field) {
            auto p = geometry();
            switch (field) {
            case 0: p.motorPositionM[0][0] = bad; break;
            case 1: p.maxThrustN[0] = bad; break;
            case 2: p.kT[0] = bad; break;
            case 3: p.tailAngleMinRad = bad; break;
            case 4: p.tailAngleMaxRad = bad; break;
            case 5: p.wrenchScale[0] = bad; break;
            case 6: p.lowTailThrustN = bad; break;
            case 7: p.projectedGradientTolerance = bad; break;
            case 8: p.maxConditionNumber = bad; break;
            }
            EXPECT_FALSE(tricopterAllocatorValidateParams(&p));
            expectUnavailable(tricopterAllocatorAllocate(&p, &r));
        }
        auto v = identityMap(); v.effectivenessMatrix[1][3] = bad;
        EXPECT_FALSE(tricopterAllocatorVirtualValidateParams(&v));
    }
    auto p = geometry(); p.source = TRICOPTER_ALLOCATOR_SOURCE_UNSPECIFIED;
    expectUnavailable(tricopterAllocatorAllocate(&p, &r));
    p = geometry(); p.kT[0] = 1e-30f; EXPECT_FALSE(tricopterAllocatorValidateParams(&p));
    p = geometry(); p.motorPositionM[0][0] = 1001; EXPECT_FALSE(tricopterAllocatorValidateParams(&p));
    p = geometry(); p.maxIterations = 4097; EXPECT_FALSE(tricopterAllocatorValidateParams(&p));
}

TEST(TricopterAllocator, InvalidInputsAndForwardBoundsClearEveryNumericOutput)
{
    const auto p = geometry(); const float t[3] = {1, 2, 3};
    expectUnavailable(tricopterAllocatorAllocate(&p, nullptr));
    expectUnavailable(tricopterAllocatorForward(&p, nullptr, 0, 0, false, false));
    expectUnavailable(tricopterAllocatorForward(&p, t, p.tailAngleMaxRad + .01f, 0, false, false));
    float negative[3] = {-1, 0, 0};
    expectUnavailable(tricopterAllocatorForward(&p, negative, 0, 0, false, false));
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), 1e30f}) {
        for (unsigned axis = 0; axis < 4; ++axis) {
            tricopterAllocatorRequest_t r = {}; r.wrench[axis] = bad;
            expectUnavailable(tricopterAllocatorAllocate(&p, &r));
        }
        expectUnavailable(tricopterAllocatorForward(&p, t, bad, 0, false, false));
        expectUnavailable(tricopterAllocatorForward(&p, t, 0, bad, false, false));
    }
    auto v = identityMap(); tricopterAllocatorVirtualRequest_t vr = {{0, 0, 0, std::numeric_limits<float>::quiet_NaN()}};
    const auto invalid = tricopterAllocatorVirtualAllocate(&v, &vr);
    EXPECT_FALSE(invalid.valid); EXPECT_EQ(invalid.cost, 0); EXPECT_EQ(invalid.achievedResponse[0], 0);
}
