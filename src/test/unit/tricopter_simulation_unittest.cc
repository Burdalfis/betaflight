/* Synthetic integration tests. No aircraft calibration or plant truth input. */
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "flight/tricopter_simulation.h"
}

#include "gtest/gtest.h"

static tricopterSimulationParams_t parameters()
{
    tricopterSimulationParams_t p = {};
    const float matrix[4][4] = {{1, 1, 1, 0}, {-.25f, .25f, 0, 0}, {-.15f, -.15f, .3f, 0}, {0, 0, 0, .3f}};
    std::memcpy(p.allocator.effectivenessMatrix, matrix, sizeof(matrix));
    p.allocator.tailDirectionMinRad = -.6f; p.allocator.tailDirectionMaxRad = .6f;
    p.allocator.responseScale[0] = 1; p.allocator.responseScale[1] = p.allocator.responseScale[2] = .2f; p.allocator.responseScale[3] = .1f;
    p.allocator.projectedGradientTolerance = 1e-5f; p.allocator.maxConditionNumber = 1e6f;
    p.allocator.maxIterations = 4096; p.allocator.lowTailEffort = 1e-5f;
    p.allocator.source = TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER;
    p.servo.knotCount = 3;
    p.servo.pwmUs[0] = 1100; p.servo.pwmUs[1] = 1400; p.servo.pwmUs[2] = 1700;
    p.servo.positionCounts[0] = 2000; p.servo.positionCounts[1] = 1500; p.servo.positionCounts[2] = 1000;
    p.servo.tauIncreasingPwmS = p.servo.tauDecreasingPwmS = .05f;
    p.servo.delayUs = 10000; p.servo.maxGapUs = 20000;
    p.reaction.coefficientSource = TRICOPTER_REACTION_TORQUE_COEFFICIENTS_SYNTHETIC_PLACEHOLDER;
    p.reaction.maxMechanicalRpm = 10000; p.reaction.maxAbsAccelerationRpmS = 100000;
    p.reaction.maxAbsTorqueNm = 10; p.reaction.maxRpmAgeUs = 10000;
    for (unsigned i = 0; i < 3; ++i) {
        p.allocator.maxEffort[i] = 5;
        p.rpm[i] = { .02f, .03f, 10000, 100000, 10000, 100000, 20000 };
        p.reaction.kQ[i] = 1e-7f; p.reaction.inertiaKgM2[i] = 2e-5f; p.reaction.reactionSign[i] = i == 1 ? 1 : -1;
        p.effortPerOmegaSquared[i] = 1e-5f;
    }
    p.correctedCountsAtDirectionMin = 2000; p.correctedCountsAtDirectionMax = 1000;
    p.yawResponsePerNm = 10; p.maxAbsReactionCompensationResponse = 1;
    p.minTailSupportCosine = .2f; p.effectivenessExcludesReaction = true;
    return p;
}

static tricopterSimulationInitialState_t initial(uint64_t time = 0)
{
    return {time, 1500, 1400, 0, true};
}

static tricopterSimulationInput_t input(uint32_t time, uint32_t sequence, float rpm = 1000,
    uint8_t modes = TRICOPTER_SIMULATION_BASELINE)
{
    tricopterSimulationInput_t in = {};
    in.requestedResponse[0] = 6;
    in.modes = modes;
    for (auto &sample : in.rpm) sample = {true, time, sequence, rpm};
    return in;
}

static void expectCommands(const tricopterSimulationParams_t &p, const tricopterSimulationOutput_t &out)
{
    ASSERT_TRUE(out.valid); EXPECT_FALSE(out.calibrated); EXPECT_FALSE(out.forFlight);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_GE(out.commandedEffort[i], 0); EXPECT_LE(out.commandedEffort[i], p.allocator.maxEffort[i]);
        EXPECT_GE(out.commandedMechanicalRpm[i], 0); EXPECT_LE(out.commandedMechanicalRpm[i], p.rpm[i].maxRpm);
    }
    EXPECT_GE(out.pwmCommandUs, p.servo.pwmUs[0]); EXPECT_LE(out.pwmCommandUs, p.servo.pwmUs[p.servo.knotCount - 1]);
}

static void expectFatal(const tricopterSimulationOutput_t &out)
{
    EXPECT_FALSE(out.valid); EXPECT_NE(out.status, 0U); EXPECT_EQ(out.pwmCommandUs, 0);
    EXPECT_EQ(out.appliedReactionCompensationResponse, 0);
    for (unsigned i = 0; i < 3; ++i) { EXPECT_EQ(out.commandedEffort[i], 0); EXPECT_EQ(out.commandedMechanicalRpm[i], 0); }
    for (unsigned i = 0; i < 4; ++i) { EXPECT_EQ(out.predictedControls[i], 0); EXPECT_EQ(out.requestedResponse[i], 0); }
}

TEST(TricopterSimulation, ExplicitSyntheticMapAndInitialCorrectedPositionAreRequired)
{
    auto p = parameters(); auto start = initial(); tricopterSimulationState_t state = {};
    EXPECT_TRUE(tricopterSimulationValidateParams(&p));
    EXPECT_FALSE(tricopterSimulationValidateParams(nullptr));
    EXPECT_FALSE(tricopterSimulationReset(nullptr, &p, &start));
    p.effectivenessExcludesReaction = false;
    EXPECT_FALSE(tricopterSimulationReset(&state, &p, &start));
    p = parameters(); start.correctedPositionValid = false;
    EXPECT_FALSE(tricopterSimulationReset(&state, &p, &start));
    start = initial(); start.correctedCounts = 3000;
    EXPECT_FALSE(tricopterSimulationReset(&state, &p, &start));
    start = initial(); ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    EXPECT_TRUE(tricopterSimulationReset(&state, &state.params, &start));
}

TEST(TricopterSimulation, CommandedTargetsAndCurrentRpmPredictionStayDistinct)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1, 0); in.requestedResponse[3] = .3f;
    auto out = tricopterSimulationTick(&state, 0, &in);
    expectCommands(p, out); EXPECT_GT(out.commandedEffort[0], 2);
    EXPECT_TRUE(out.predictedActuatorResponseAvailable); EXPECT_FALSE(out.predictedTotalResponseAvailable);
    EXPECT_GT(out.coordinatedTargetActuatorResponse[0], 5);
    for (float value : out.predictedControls) EXPECT_EQ(value, 0);
    for (float value : out.predictedActuatorResponse) EXPECT_EQ(value, 0);
    in = input(1000, 2, 0);
    out = tricopterSimulationTick(&state, 1000, &in);
    EXPECT_TRUE(out.reactionAvailable); EXPECT_TRUE(out.predictedTotalResponseAvailable);
    for (float value : out.predictedTotalResponse) EXPECT_EQ(value, 0);
}

TEST(TricopterSimulation, FirstMotionPredictionUsesPreviousCommandAndDeclaredDelay)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1); in.requestedResponse[3] = .3f;
    auto out = tricopterSimulationTick(&state, 0, &in);
    EXPECT_GT(out.pwmCommandUs, 1400); EXPECT_EQ(out.servo.positionCounts, 1500); EXPECT_EQ(out.predictedDirectionRad, 0);
    in.rpm[0].seen = in.rpm[1].seen = in.rpm[2].seen = false;
    out = tricopterSimulationTick(&state, 5000, &in); EXPECT_EQ(out.predictedDirectionRad, 0);
    out = tricopterSimulationTick(&state, 10000, &in); EXPECT_EQ(out.predictedDirectionRad, 0);
    out = tricopterSimulationTick(&state, 11000, &in);
    EXPECT_GT(out.predictedDirectionRad, 0); EXPECT_LT(out.predictedDirectionRad, out.targetDirectionRad);
}

TEST(TricopterSimulation, NewRequestCannotRetroactivelyChangeCapturedPrediction)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t a = {};
    ASSERT_TRUE(tricopterSimulationReset(&a, &p, &start));
    auto first = input(0, 1); tricopterSimulationTick(&a, 0, &first); auto b = a;
    auto left = input(1000, 2), right = left; left.requestedResponse[3] = -.3f; right.requestedResponse[3] = .3f;
    const auto aa = tricopterSimulationTick(&a, 1000, &left), bb = tricopterSimulationTick(&b, 1000, &right);
    EXPECT_LT(aa.pwmCommandUs, bb.pwmCommandUs);
    EXPECT_EQ(aa.servo.positionCounts, bb.servo.positionCounts); EXPECT_EQ(aa.predictedDirectionRad, bb.predictedDirectionRad);
    for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(aa.predictedControls[i], bb.predictedControls[i]);
}

TEST(TricopterSimulation, AccelerationRequiresFreshDistinctPairAfterEveryStaleGap)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1); auto out = tricopterSimulationTick(&state, 0, &in);
    EXPECT_FALSE(out.accelerationValid[0]); EXPECT_FALSE(out.reactionAvailable);
    in = input(1000, 2); out = tricopterSimulationTick(&state, 1000, &in); EXPECT_TRUE(out.reactionAvailable);
    out = tricopterSimulationTick(&state, 11001, &in); EXPECT_FALSE(out.reactionAvailable);
    in = input(12000, 3); out = tricopterSimulationTick(&state, 12000, &in); EXPECT_FALSE(out.accelerationValid[0]);
    out = tricopterSimulationTick(&state, 12500, &in); EXPECT_FALSE(out.accelerationValid[0]);
    in = input(13000, 4); out = tricopterSimulationTick(&state, 13000, &in); EXPECT_TRUE(out.reactionAvailable);
    // Even without an explicit stale decision tick, the old packet would
    // have expired before this first recovery packet arrived.
    in = input(24001, 5); out = tricopterSimulationTick(&state, 24001, &in); EXPECT_FALSE(out.accelerationValid[0]);
    in = input(25000, 6); out = tricopterSimulationTick(&state, 25000, &in); EXPECT_TRUE(out.reactionAvailable);
}

TEST(TricopterSimulation, CoalescedTimeAndLongReacquisitionCannotInventKnownZeroAcceleration)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1); tricopterSimulationTick(&state, 0, &in);
    in = input(1000, 2); EXPECT_TRUE(tricopterSimulationTick(&state, 1000, &in).reactionAvailable);
    in = input(1000, 3); auto out = tricopterSimulationTick(&state, 1000, &in);
    EXPECT_FALSE(out.reactionAvailable); EXPECT_FALSE(out.accelerationValid[0]);
    for (uint32_t now = 11000; now <= 111000; now += 10000) tricopterSimulationTick(&state, now, &in);
    in = input(112000, 4); out = tricopterSimulationTick(&state, 112000, &in);
    EXPECT_NE(out.rpm[0].status & TRICOPTER_RPM_OBSERVER_REACQUIRED, 0U); EXPECT_FALSE(out.reactionAvailable);
    in = input(113000, 5); EXPECT_TRUE(tricopterSimulationTick(&state, 113000, &in).reactionAvailable);
}

TEST(TricopterSimulation, ReactionModeSubtractsExactlyOneBoundedScalarAndBaselineLeavesDemand)
{
    auto p = parameters(); p.yawResponsePerNm = 1000; p.maxAbsReactionCompensationResponse = .01f;
    const auto start = initial(); tricopterSimulationState_t baseline = {};
    ASSERT_TRUE(tricopterSimulationReset(&baseline, &p, &start));
    auto in = input(0, 1, 3000); tricopterSimulationTick(&baseline, 0, &in); auto aware = baseline;
    in = input(1000, 2, 3000);
    const auto off = tricopterSimulationTick(&baseline, 1000, &in);
    in.modes = TRICOPTER_SIMULATION_REACTION_COMPENSATION;
    const auto on = tricopterSimulationTick(&aware, 1000, &in);
    expectCommands(p, on); EXPECT_TRUE(on.reactionAvailable); EXPECT_EQ(off.appliedReactionCompensationResponse, 0);
    EXPECT_EQ(off.netRequestedResponse[3], 0); EXPECT_NEAR(on.netRequestedResponse[3], .01, 1e-8);
    EXPECT_NEAR(on.appliedReactionCompensationResponse, -.01, 1e-8);
    EXPECT_NE(on.status & TRICOPTER_SIMULATION_REACTION_COMPENSATION_CLIPPED, 0U);
    EXPECT_NEAR(on.coordinatedTargetTotalResponse[3], on.coordinatedTargetActuatorResponse[3]
        + p.yawResponsePerNm * on.reaction.summedTotalNm, 1e-6);
    for (unsigned axis = 0; axis < 4; ++axis) EXPECT_NEAR(on.postCoordinationResidual[axis], on.coordinatedTargetTotalResponse[axis] - on.requestedResponse[axis], 1e-6);
}

TEST(TricopterSimulation, VirtualVerticalCoordinationPreservesThatCoordinateNotInstantYaw)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t a = {};
    ASSERT_TRUE(tricopterSimulationReset(&a, &p, &start)); auto b = a;
    auto in = input(0, 1, 0); in.requestedResponse[3] = .3f;
    const auto baseline = tricopterSimulationTick(&a, 0, &in);
    in.modes = TRICOPTER_SIMULATION_TAIL_VERTICAL_COORDINATION;
    const auto aware = tricopterSimulationTick(&b, 0, &in);
    expectCommands(p, aware);
    EXPECT_EQ(aware.pwmCommandUs, baseline.pwmCommandUs);
    EXPECT_NEAR(aware.coordinatedTargetControls[2], aware.nominalAllocation.controls[2], 1e-6);
    EXPECT_GT(baseline.coordinatedTargetControls[2] - baseline.nominalAllocation.controls[2], .2);
    EXPECT_EQ(aware.coordinatedTargetControls[3], 0); EXPECT_GT(aware.nominalAllocation.controls[3], .9);
    EXPECT_FALSE(aware.coordinatedTotalResponseAvailable); // Unknown derivative is not zero reaction.
}

TEST(TricopterSimulation, PublishedReactionCorrectionCannotExceedBoundAtLargeDemand)
{
    auto p = parameters(); p.yawResponsePerNm = -1000; p.maxAbsReactionCompensationResponse = 5;
    const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1, 3000); tricopterSimulationTick(&state, 0, &in);
    in = input(1000, 2, 3000, TRICOPTER_SIMULATION_REACTION_COMPENSATION); in.requestedResponse[3] = 1e8f;
    const auto out = tricopterSimulationTick(&state, 1000, &in);
    expectCommands(p, out); ASSERT_TRUE(out.reactionAvailable);
    EXPECT_LE(std::fabs(out.appliedReactionCompensationResponse), p.maxAbsReactionCompensationResponse);
    EXPECT_EQ(static_cast<double>(out.requestedResponse[3]) - out.netRequestedResponse[3], out.appliedReactionCompensationResponse);
    EXPECT_NE(out.status & TRICOPTER_SIMULATION_REACTION_COMPENSATION_CLIPPED, 0U);
}

TEST(TricopterSimulation, CosineAndEffortGuardsExposeCoordinationLimits)
{
    auto p = parameters(); p.minTailSupportCosine = 1;
    auto start = initial(); start.correctedCounts = 1250; start.pwmUs = 1550;
    tricopterSimulationState_t state = {}; ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1, 1000, TRICOPTER_SIMULATION_TAIL_VERTICAL_COORDINATION);
    auto out = tricopterSimulationTick(&state, 0, &in);
    expectCommands(p, out); EXPECT_NE(out.status & TRICOPTER_SIMULATION_TAIL_COSINE_GUARD, 0U);
    EXPECT_EQ(out.commandedEffort[0], out.nominalAllocation.effort[0]);
    p = parameters(); start.correctedCounts = static_cast<float>(2000 - (.55 + .6) / 1.2 * 1000); start.pwmUs = 1675;
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    in.requestedResponse[0] = 14.7f;
    out = tricopterSimulationTick(&state, 0, &in);
    expectCommands(p, out); EXPECT_EQ(out.commandedEffort[0], p.allocator.maxEffort[0]);
    EXPECT_NE(out.status & TRICOPTER_SIMULATION_TAIL_EFFORT_CLIPPED, 0U);
}

TEST(TricopterSimulation, OutboundCurrentEffortAndRejectedPacketsDoNotBecomeClippedPredictions)
{
    const auto p = parameters(); const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1, 10000); auto out = tricopterSimulationTick(&state, 0, &in);
    expectCommands(p, out); EXPECT_FALSE(out.predictedActuatorResponseAvailable);
    for (float value : out.predictedControls) EXPECT_EQ(value, 0);
    in = input(1000, 2, 1e6f); out = tricopterSimulationTick(&state, 1000, &in);
    EXPECT_FALSE(out.rpm[0].acceptedNewObservation); EXPECT_EQ(out.rpm[0].ageUs, 1000U);
    EXPECT_FALSE(out.predictedActuatorResponseAvailable); EXPECT_TRUE(out.valid);
}

TEST(TricopterSimulation, SyntheticCountMappingSupportsSignedCountsAndPiecewisePwmInverse)
{
    auto p = parameters(); auto start = initial();
    for (unsigned i = 0; i < p.servo.knotCount; ++i) p.servo.positionCounts[i] *= -1;
    p.correctedCountsAtDirectionMin *= -1; p.correctedCountsAtDirectionMax *= -1; start.correctedCounts *= -1;
    tricopterSimulationState_t state = {}; ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(0, 1); in.requestedResponse[3] = .3f;
    auto out = tricopterSimulationTick(&state, 0, &in); EXPECT_GT(out.pwmCommandUs, 1400); EXPECT_EQ(out.servo.positionCounts, -1500);
    p = parameters(); p.servo.pwmUs[1] = 1300; p.servo.positionCounts[1] = 1600;
    start = initial(); start.pwmUs = 1366.6666f;
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    const float effort[3] = {2, 2, 2};
    const auto target = tricopterAllocatorVirtualForward(&p.allocator, effort, .3f);
    std::memcpy(in.requestedResponse, target.achievedResponse, sizeof(in.requestedResponse));
    out = tricopterSimulationTick(&state, 0, &in);
    EXPECT_NEAR(out.targetDirectionRad, .3f, 1e-6); EXPECT_NEAR(out.pwmCommandUs, 1533.333333, .001);
}

TEST(TricopterSimulation, ClockAndSequenceWrapPreserveCausalFreshPair)
{
    const auto p = parameters(); const uint64_t time = UINT32_MAX - 1000ULL; const auto start = initial(time);
    tricopterSimulationState_t state = {}; ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(static_cast<uint32_t>(time), UINT32_MAX);
    tricopterSimulationTick(&state, time, &in);
    in = input(static_cast<uint32_t>(time + 1500), 0);
    const auto out = tricopterSimulationTick(&state, time + 1500, &in);
    expectCommands(p, out); EXPECT_TRUE(out.reactionAvailable); EXPECT_TRUE(out.accelerationValid[0]);
    EXPECT_EQ(out.rpm[0].sequence, 0U); EXPECT_EQ(out.rpm[0].ageUs, 0U);
}

TEST(TricopterSimulation, InvalidInputAndClockLatchUntilExplicitReset)
{
    const auto p = parameters(); auto start = initial(1000); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    auto in = input(1000, 1); tricopterSimulationTick(&state, 1000, &in);
    expectFatal(tricopterSimulationTick(&state, 999, &in));
    expectFatal(tricopterSimulationTick(&state, 1001, &in));
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    expectFatal(tricopterSimulationTick(&state, 21001, &in));
    ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start));
    in.modes = 8; expectFatal(tricopterSimulationTick(&state, 1000, &in));
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        ASSERT_TRUE(tricopterSimulationReset(&state, &p, &start)); in = input(1000, 1); in.requestedResponse[3] = bad;
        expectFatal(tricopterSimulationTick(&state, 1000, &in));
    }
}

TEST(TricopterSimulation, InvalidMappingAndNumericParametersRejectWithFastMath)
{
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        for (unsigned field = 0; field < 6; ++field) {
            auto p = parameters();
            switch (field) {
            case 0: p.effortPerOmegaSquared[1] = bad; break;
            case 1: p.correctedCountsAtDirectionMin = bad; break;
            case 2: p.correctedCountsAtDirectionMax = bad; break;
            case 3: p.yawResponsePerNm = bad; break;
            case 4: p.maxAbsReactionCompensationResponse = bad; break;
            case 5: p.minTailSupportCosine = bad; break;
            }
            EXPECT_FALSE(tricopterSimulationValidateParams(&p));
        }
    }
    auto p = parameters(); p.correctedCountsAtDirectionMax = p.correctedCountsAtDirectionMin;
    EXPECT_FALSE(tricopterSimulationValidateParams(&p));
    p = parameters(); p.effortPerOmegaSquared[0] = 1e-12f; // Command target would exceed RPM software bounds.
    EXPECT_FALSE(tricopterSimulationValidateParams(&p));
    p = parameters(); p.allocator.tailDirectionMinRad = p.allocator.tailDirectionMaxRad = 0;
    EXPECT_FALSE(tricopterSimulationValidateParams(&p));
}

static void configureDynamic(tricopterSimulationParams_t &p)
{
    p.dynamicConfigured = true; p.dynamic.horizonUs = 100000; p.dynamic.maxRpmAgeUs = 10000;
    for (unsigned i = 0; i < 3; ++i) p.dynamic.motorTauIncreasingS[i] = p.dynamic.motorTauDecreasingS[i] = .05f;
    const float grid[5] = {1100,1250,1400,1550,1700}; std::memcpy(p.dynamic.pwmGridUs,grid,sizeof(grid)); p.dynamic.gridCount = 5;
    p.dynamic.maxHighMatrixCondition = 1e6f;
    p.dynamic.highCostTolerance = p.dynamic.yawCostTolerance = p.dynamic.movementCostTolerance = 1e-10f;
}

TEST(TricopterSimulation, DynamicModePublishesChosenCommandsWithoutAdvancingLiveServoToFuture)
{
    auto p = parameters(); configureDynamic(p); const auto start = initial(); tricopterSimulationState_t state = {};
    ASSERT_TRUE(tricopterSimulationReset(&state,&p,&start));
    auto in = input(0,1,3000,TRICOPTER_SIMULATION_DYNAMIC_ALLOCATION); in.requestedResponse[3] = .3f;
    const auto out = tricopterSimulationTick(&state,0,&in);
    expectCommands(p,out); ASSERT_TRUE(out.dynamicAllocationUsed); EXPECT_TRUE(out.dynamicAllocation.priorityAvailable);
    EXPECT_EQ(out.pwmCommandUs,out.dynamicAllocation.selected.pwmUs);
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(out.commandedMechanicalRpm[i],out.dynamicAllocation.selected.commandMechanicalRpm[i]);
    EXPECT_EQ(state.servo.timeUs,0U); EXPECT_EQ(out.servo.positionCounts,1500);
    EXPECT_GT(out.dynamicAllocation.selected.horizonDirectionRad,out.predictedDirectionRad);
}

TEST(TricopterSimulation, DynamicUnavailableStateFallsBackExplicitlyWithoutPriorityClaim)
{
    auto p = parameters(); configureDynamic(p); const auto start = initial(); tricopterSimulationState_t a = {};
    ASSERT_TRUE(tricopterSimulationReset(&a,&p,&start)); auto b = a;
    auto in = input(0,1,1000); for (auto &sample : in.rpm) sample.seen = false;
    const auto off = tricopterSimulationTick(&a,0,&in);
    in.modes = TRICOPTER_SIMULATION_DYNAMIC_ALLOCATION;
    const auto degraded = tricopterSimulationTick(&b,0,&in);
    expectCommands(p,degraded); EXPECT_FALSE(degraded.dynamicAllocationUsed); EXPECT_FALSE(degraded.dynamicAllocation.priorityAvailable);
    EXPECT_NE(degraded.status & TRICOPTER_SIMULATION_DYNAMIC_UNAVAILABLE,0U);
    EXPECT_EQ(degraded.pwmCommandUs,off.pwmCommandUs);
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(degraded.commandedMechanicalRpm[i],off.commandedMechanicalRpm[i]);
    in.modes = TRICOPTER_SIMULATION_DYNAMIC_ALLOCATION | TRICOPTER_SIMULATION_TAIL_VERTICAL_COORDINATION;
    expectFatal(tricopterSimulationTick(&b,1000,&in));
}

TEST(TricopterSimulation, ConfiguredDynamicModuleDoesNotChangeAnyExistingMode)
{
    const auto oldParams = parameters(); auto extended = oldParams; configureDynamic(extended); const auto start = initial();
    for (uint8_t mode = 0; mode <= 3; ++mode) {
        tricopterSimulationState_t a = {}, b = {};
        ASSERT_TRUE(tricopterSimulationReset(&a,&oldParams,&start)); ASSERT_TRUE(tricopterSimulationReset(&b,&extended,&start));
        for (uint32_t tick = 0; tick <= 20000; tick += 1000) {
            auto in = input(tick,tick/1000,3000,mode); in.requestedResponse[3] = .2f;
            const auto old = tricopterSimulationTick(&a,tick,&in), current = tricopterSimulationTick(&b,tick,&in);
            EXPECT_EQ(old.status,current.status); EXPECT_EQ(old.pwmCommandUs,current.pwmCommandUs);
            EXPECT_EQ(old.servo.positionCounts,current.servo.positionCounts); EXPECT_EQ(old.appliedReactionCompensationResponse,current.appliedReactionCompensationResponse);
            for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(old.commandedMechanicalRpm[i],current.commandedMechanicalRpm[i]);
            for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(old.predictedTotalResponse[i],current.predictedTotalResponse[i]);
            EXPECT_FALSE(current.dynamicAllocationUsed);
        }
    }
}

static void measuredMap(tricopterSimulationParams_t *p)
{
    p->measuredAngleConfigured=true; auto &a=p->measuredAngle; a.knotCount=3; a.operatorDefinedReference=true;
    a.correctedCounts[0]=1000; a.correctedCounts[1]=1500; a.correctedCounts[2]=2000;
    a.angleRad[0]=.6f; a.angleRad[1]=.2f; a.angleRad[2]=-.6f;
    a.halfBranchSeparationRad[0]=.01f; a.halfBranchSeparationRad[1]=.03f; a.halfBranchSeparationRad[2]=.02f;
}
TEST(TricopterSimulation, MeasuredMapConvertsCurrentPredictionAndInverseGoalWithoutClaimingFlightCalibration)
{
    auto p=parameters(); measuredMap(&p); const auto start=initial(); tricopterSimulationState_t state={};
    ASSERT_TRUE(tricopterSimulationReset(&state,&p,&start));
    auto in=input(0,1); in.requestedResponse[3]=0;
    const auto out=tricopterSimulationTick(&state,0,&in); expectCommands(p,out);
    EXPECT_TRUE(out.measuredAngleAvailable); EXPECT_EQ(out.predictedDirectionRad,.2f);
    EXPECT_EQ(out.angleHalfBranchSeparationRad,.03f); EXPECT_FALSE(out.servo.angleCalibrated);
    float counts; ASSERT_TRUE(tricopterAngleCalibrationInverse(&p.measuredAngle,out.targetDirectionRad,&counts));
    EXPECT_NEAR(out.pwmCommandUs,1100+(2000-counts)*.6f,1e-3);
}
TEST(TricopterSimulation, MeasuredMapCoverageIsRequiredAndDefaultIgnoresUnusedMap)
{
    auto p=parameters(); measuredMap(&p); p.allocator.tailDirectionMaxRad=.7f;
    EXPECT_FALSE(tricopterSimulationValidateParams(&p));
    p=parameters(); p.measuredAngle.knotCount=65;
    EXPECT_TRUE(tricopterSimulationValidateParams(&p));
    measuredMap(&p); p.measuredAngle.correctedCounts[0]=1100;
    EXPECT_TRUE(tricopterSimulationValidateParams(&p));
    p.allocator.tailDirectionMaxRad=.61f;
    EXPECT_FALSE(tricopterSimulationValidateParams(&p));
}
