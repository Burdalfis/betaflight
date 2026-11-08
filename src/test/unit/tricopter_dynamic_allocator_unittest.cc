/* Independent analytic and bounded-policy tests; synthetic proxy units only. */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "flight/tricopter_dynamic_allocator.h"
}

#include "gtest/gtest.h"

static constexpr double PI = 3.14159265358979323846;
static tricopterDynamicAllocatorModel_t model()
{
    tricopterDynamicAllocatorModel_t m = {};
    const float matrix[4][4] = {{1,1,1,0},{-.25f,.25f,0,0},{-.15f,-.15f,.3f,0},{0,0,0,.3f}};
    std::memcpy(m.allocator.effectivenessMatrix,matrix,sizeof(matrix));
    for (unsigned i = 0; i < 3; ++i) { m.allocator.maxEffort[i] = 5; m.effortPerOmegaSquared[i] = 1e-5f; }
    m.allocator.tailDirectionMinRad = -.6f; m.allocator.tailDirectionMaxRad = .6f;
    m.allocator.responseScale[0] = 1; m.allocator.responseScale[1] = m.allocator.responseScale[2] = .2f; m.allocator.responseScale[3] = .1f;
    m.allocator.projectedGradientTolerance = 1e-5f; m.allocator.maxConditionNumber = 1e6f;
    m.allocator.maxIterations = 4096; m.allocator.source = TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER;
    m.correctedCountsAtDirectionMin = 2000; m.correctedCountsAtDirectionMax = 1000;
    return m;
}
static tricopterServoModelParams_t servoParams()
{
    tricopterServoModelParams_t p = {}; p.knotCount = 3;
    p.pwmUs[0] = 1100; p.pwmUs[1] = 1400; p.pwmUs[2] = 1700;
    p.positionCounts[0] = 2000; p.positionCounts[1] = 1500; p.positionCounts[2] = 1000;
    p.tauIncreasingPwmS = p.tauDecreasingPwmS = .05f; p.delayUs = 10000; p.maxGapUs = 5000;
    return p;
}
static tricopterServoModelState_t servoState()
{
    tricopterServoModelState_t state = {}; const auto p = servoParams();
    tricopterServoModelReset(&state,&p,0,1400,1500,0); return state;
}
static tricopterDynamicAllocatorParams_t parameters()
{
    tricopterDynamicAllocatorParams_t p = {}; p.horizonUs = 100000; p.maxRpmAgeUs = 10000;
    for (unsigned i = 0; i < 3; ++i) p.motorTauIncreasingS[i] = p.motorTauDecreasingS[i] = .05f;
    const float grid[5] = {1100,1250,1400,1550,1700}; std::memcpy(p.pwmGridUs,grid,sizeof(grid)); p.gridCount = 5;
    p.maxHighMatrixCondition = 1e6f; p.highCostTolerance = p.yawCostTolerance = p.movementCostTolerance = 1e-10f;
    return p;
}
static tricopterDynamicAllocatorInput_t input()
{
    tricopterDynamicAllocatorInput_t in = {}; in.currentPwmUs = 1400; in.nominalPwmUs = 1631.82385f;
    in.requestedResponse[0] = 6; in.requestedResponse[3] = .3f;
    const float rpm = static_cast<float>(std::sqrt(2.0/1e-5)*30/PI);
    for (auto &motor : in.motors) motor = {rpm,0,true,true,true};
    return in;
}
static bool finiteBits(float f)
{
    uint32_t bits; std::memcpy(&bits,&f,sizeof(bits)); return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}
static void expectValid(const tricopterDynamicAllocatorOutput_t &out)
{
    ASSERT_TRUE(out.valid); EXPECT_TRUE(out.priorityAvailable); EXPECT_FALSE(out.calibrated); EXPECT_FALSE(out.forFlight);
    EXPECT_GT(out.validCandidateCount,0U); EXPECT_LE(out.candidateCount,TRICOPTER_DYNAMIC_ALLOCATOR_MAX_CANDIDATES);
    EXPECT_GE(out.selected.pwmUs,out.minimumPwmUs); EXPECT_LE(out.selected.pwmUs,out.maximumPwmUs);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_GE(out.selected.commandMechanicalRpm[i],0); EXPECT_LE(out.selected.commandMechanicalRpm[i],out.maximumCommandRpm[i]);
        EXPECT_GE(out.selected.predictedEffort[i],out.reachableMinEffort[i]-1e-6f);
        EXPECT_LE(out.selected.predictedEffort[i],out.reachableMaxEffort[i]+1e-6f);
        EXPECT_TRUE(finiteBits(out.selected.predictedEffort[i]));
    }
}
static void expectUnavailable(const tricopterDynamicAllocatorOutput_t &out)
{
    EXPECT_FALSE(out.valid); EXPECT_FALSE(out.priorityAvailable); EXPECT_NE(out.status,0U);
    for (float rpm : out.selected.commandMechanicalRpm) EXPECT_EQ(rpm,0);
    EXPECT_FALSE(out.calibrated); EXPECT_FALSE(out.forFlight);
}

TEST(TricopterDynamicAllocator, PredictsServoAndMotorReachabilityAtDeclaredHorizon)
{
    const auto m = model(); const auto p = parameters(); const auto s = servoState(); const auto in = input();
    ASSERT_TRUE(tricopterDynamicAllocatorValidateParams(&m,&p,&s.params));
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    const double attenuation = std::exp(-.1/static_cast<double>(p.motorTauIncreasingS[0]));
    for (unsigned i = 0; i < 3; ++i) {
        const double minRpm = in.motors[i].mechanicalRpm*attenuation;
        const double maxRpm = out.maximumCommandRpm[i] + (in.motors[i].mechanicalRpm-out.maximumCommandRpm[i])*attenuation;
        EXPECT_NEAR(out.reachableMinEffort[i],m.effortPerOmegaSquared[i]*std::pow(minRpm*PI/30,2),1e-6);
        EXPECT_NEAR(out.reachableMaxEffort[i],m.effortPerOmegaSquared[i]*std::pow(maxRpm*PI/30,2),1e-6);
    }
    EXPECT_EQ(out.selected.pwmUs,1700);
    EXPECT_NEAR(out.selected.horizonDirectionRad,.6*(1-std::exp(-.09/.05)),2e-6);
    EXPECT_LT(out.selected.highCost,1e-10); EXPECT_EQ(out.selected.evaluatedFaces,27U);
}

TEST(TricopterDynamicAllocator, HugeYawDemandCannotBuyWorseCollectiveRollPitchGroup)
{
    const auto m = model(); const auto p = parameters(); const auto s = servoState(); auto in = input();
    in.requestedResponse[0] = 14; in.requestedResponse[3] = 100; in.nominalPwmUs = 1700;
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    EXPECT_EQ(out.selected.pwmUs,1400);
    EXPECT_LE(out.selected.highCost,out.minimumHighCost+p.highCostTolerance);
    bool temptingYaw = false;
    for (const auto &candidate : out.candidates) if (candidate.valid && candidate.yawCost < out.selected.yawCost && candidate.highCost > out.minimumHighCost+p.highCostTolerance) temptingYaw = true;
    EXPECT_TRUE(temptingYaw);
}

TEST(TricopterDynamicAllocator, GlobalToleranceBandsDoNotDriftWithCandidateOrdering)
{
    const auto m = model(); auto p = parameters(); const auto s = servoState(); const auto in = input();
    p.highCostTolerance = 1e-7f; p.yawCostTolerance = 1e-8f;
    const auto a = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in);
    std::reverse(p.pwmGridUs,p.pwmGridUs+p.gridCount);
    const auto b = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in);
    expectValid(a); expectValid(b); EXPECT_EQ(a.minimumHighCost,b.minimumHighCost);
    EXPECT_EQ(a.minimumYawCostWithinHighTolerance,b.minimumYawCostWithinHighTolerance);
    EXPECT_EQ(a.selected.pwmUs,b.selected.pwmUs);
    for (const auto &candidate : a.candidates) {
        if (candidate.valid) { EXPECT_LE(a.minimumHighCost,candidate.highCost); }
    }
}

TEST(TricopterDynamicAllocator, CostsAreForwardEvaluatedFromPublishedRpmCommands)
{
    const auto m = model(); const auto p = parameters(); const auto s = servoState(); const auto in = input();
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    for (const auto &candidate : out.candidates) if (candidate.valid) {
        double effort[3];
        for (unsigned i = 0; i < 3; ++i) {
            const double command = candidate.commandMechanicalRpm[i];
            const double tau = command >= in.motors[i].mechanicalRpm ? p.motorTauIncreasingS[i] : p.motorTauDecreasingS[i];
            const double rpm = command+(in.motors[i].mechanicalRpm-command)*std::exp(-(p.horizonUs*1e-6)/tau);
            effort[i] = m.effortPerOmegaSquared[i]*std::pow(rpm*PI/30,2);
            EXPECT_NEAR(candidate.predictedMechanicalRpm[i],rpm,.001);
        }
        const double direction = candidate.horizonDirectionRad;
        const double controls[4] = {effort[1],effort[2],effort[0]*std::cos(direction),effort[0]*std::sin(direction)};
        double high = 0,yaw = 0;
        for (unsigned i = 0; i < 4; ++i) {
            double achieved = 0; for (unsigned j = 0; j < 4; ++j) achieved += m.allocator.effectivenessMatrix[i][j]*controls[j];
            const double normalized = (achieved-in.requestedResponse[i])/m.allocator.responseScale[i];
            if (i < 3) high += .5*normalized*normalized; else yaw = .5*normalized*normalized;
            EXPECT_NEAR(candidate.predictedResponse[i],achieved,1e-6);
        }
        EXPECT_NEAR(candidate.highCost,high,1e-12); EXPECT_NEAR(candidate.yawCost,yaw,1e-10);
    }
}

TEST(TricopterDynamicAllocator, CopiesPendingServoHistoryWithoutMutatingLivePredictor)
{
    const auto m = model(); const auto p = parameters(); auto s = servoState(); auto in = input();
    tricopterServoModelStep(&s,0,1550); tricopterServoModelStep(&s,5000,1550);
    in.nowUs = 5000; in.currentPwmUs = 1550;
    const auto before = s;
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    EXPECT_EQ(std::memcmp(&before,&s,sizeof(s)),0);
    auto truthCopy = s; tricopterServoModelStep(&truthCopy,in.nowUs,out.selected.pwmUs);
    tricopterServoModelOutput_t predicted = {};
    for (uint64_t now = in.nowUs+5000; now <= in.nowUs+p.horizonUs; now += 5000) predicted = tricopterServoModelStep(&truthCopy,now,out.selected.pwmUs);
    EXPECT_EQ(predicted.positionCounts,out.selected.horizonCorrectedCounts);
}

TEST(TricopterDynamicAllocator, SlewBoundedReachableIntervalsUseActualCurrentRpm)
{
    const auto m = model(); auto p = parameters(); const auto s = servoState(); auto in = input();
    for (unsigned i = 0; i < 3; ++i) { p.motorSlewIncreasingRpmS[i] = 1000; p.motorSlewDecreasingRpmS[i] = 500; in.motors[i].mechanicalRpm = 3000; }
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    for (unsigned i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.reachableMinEffort[i],m.effortPerOmegaSquared[i]*std::pow(2950*PI/30,2),1e-6);
        EXPECT_NEAR(out.reachableMaxEffort[i],m.effortPerOmegaSquared[i]*std::pow(3100*PI/30,2),1e-6);
    }
}

TEST(TricopterDynamicAllocator, DegenerateReachableIntervalAndZeroStateStayFinite)
{
    const auto m = model(); auto p = parameters(); const auto s = servoState(); auto in = input();
    for (unsigned i = 0; i < 3; ++i) { p.motorSlewIncreasingRpmS[i] = p.motorSlewDecreasingRpmS[i] = 1e-30f; in.motors[i].mechanicalRpm = 1000; }
    auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(out.reachableMinEffort[i],out.reachableMaxEffort[i]);
    p = parameters(); in = input(); in.nominalPwmUs = in.currentPwmUs;
    for (auto &motor : in.motors) motor.mechanicalRpm = 0;
    for (float &response : in.requestedResponse) response = 0;
    out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    EXPECT_EQ(out.selected.pwmUs,in.currentPwmUs);
    for (float command : out.selected.commandMechanicalRpm) EXPECT_EQ(command,0);
}

TEST(TricopterDynamicAllocator, RankDeficientHighGroupIsSkippedDespiteFullRankVirtualMap)
{
    auto m = model(); auto p = parameters(); const auto s = servoState(); auto in = input();
    std::memset(m.allocator.effectivenessMatrix,0,sizeof(m.allocator.effectivenessMatrix));
    m.allocator.effectivenessMatrix[0][0] = m.allocator.effectivenessMatrix[1][1] = 1;
    m.allocator.effectivenessMatrix[2][3] = m.allocator.effectivenessMatrix[3][2] = 1;
    ASSERT_TRUE(tricopterAllocatorVirtualValidateParams(&m.allocator));
    p.gridCount = 0; in.nominalPwmUs = in.currentPwmUs;
    auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectUnavailable(out);
    EXPECT_NE(out.status & TRICOPTER_DYNAMIC_ALLOCATOR_HIGH_MATRIX_UNAVAILABLE,0U);
    p.gridCount = 1; p.pwmGridUs[0] = 1700;
    out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    EXPECT_LT(out.validCandidateCount,out.candidateCount);
    EXPECT_NE(out.status & TRICOPTER_DYNAMIC_ALLOCATOR_HIGH_MATRIX_UNAVAILABLE,0U);
}

TEST(TricopterDynamicAllocator, CurrentMotorQualityNeverComesFromCommandTargets)
{
    const auto m = model(); const auto p = parameters(); const auto s = servoState();
    for (unsigned fault = 0; fault < 5; ++fault) {
        auto in = input();
        if (fault == 0) in.motors[0].seen = false;
        if (fault == 1) in.motors[1].fresh = false;
        if (fault == 2) in.motors[2].valid = false;
        if (fault == 3) in.motors[0].ageUs = p.maxRpmAgeUs+1;
        if (fault == 4) in.motors[0].mechanicalRpm = std::numeric_limits<float>::quiet_NaN();
        expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
    }
}

TEST(TricopterDynamicAllocator, RestrictsTargetsToInverseCountDirectionPwmInterval)
{
    auto m = model(); auto p = parameters(); const auto s = servoState(); auto in = input();
    m.correctedCountsAtDirectionMin = 1800; m.correctedCountsAtDirectionMax = 1200;
    p.gridCount = 3; p.pwmGridUs[0] = 1220; p.pwmGridUs[1] = 1400; p.pwmGridUs[2] = 1580;
    in.nominalPwmUs = 1700;
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    EXPECT_EQ(out.minimumPwmUs,1220); EXPECT_EQ(out.maximumPwmUs,1580);
    EXPECT_NE(out.status & TRICOPTER_DYNAMIC_ALLOCATOR_PWM_TARGET_CLIPPED,0U);
    for (unsigned i = 0; i < out.candidateCount; ++i) { EXPECT_GE(out.candidates[i].pwmUs,1220); EXPECT_LE(out.candidates[i].pwmUs,1580); }
    p.pwmGridUs[0] = 1100; EXPECT_FALSE(tricopterDynamicAllocatorValidateParams(&m,&p,&s.params));
}

TEST(TricopterDynamicAllocator, SignedCorrectedCountMappingPreservesPredictions)
{
    auto m = model(); const auto p = parameters(); auto s = servoState(); const auto in = input();
    const auto a = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in);
    m.correctedCountsAtDirectionMin *= -1; m.correctedCountsAtDirectionMax *= -1;
    for (unsigned i = 0; i < s.params.knotCount; ++i) s.params.positionCounts[i] *= -1;
    s.positionCounts *= -1; s.targetCounts *= -1;
    const auto b = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(a); expectValid(b);
    EXPECT_EQ(a.selected.pwmUs,b.selected.pwmUs); EXPECT_EQ(a.selected.horizonDirectionRad,b.selected.horizonDirectionRad);
}

TEST(TricopterDynamicAllocator, HorizonAndNoObservableDirectionAuthorityAreExplicit)
{
    const auto m = model(); auto p = parameters(); auto s = servoState(); const auto in = input();
    p.horizonUs = s.params.delayUs; EXPECT_FALSE(tricopterDynamicAllocatorValidateParams(&m,&p,&s.params));
    p = parameters(); p.horizonUs = 10001; s.params.tauIncreasingPwmS = s.params.tauDecreasingPwmS = 60;
    const auto out = tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectUnavailable(out);
    EXPECT_NE(out.status & TRICOPTER_DYNAMIC_ALLOCATOR_NO_DIRECTION_AUTHORITY,0U);
}

TEST(TricopterDynamicAllocator, MalformedPredictorHistoryAndTimeCannotBeIndexed)
{
    const auto m = model(); const auto p = parameters(); auto s = servoState(); auto in = input();
    s.commandCount = 255; expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
    s = servoState(); s.commandHead = 255; expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
    s = servoState(); s.commandCount = 1; s.commands[0] = {UINT64_MAX,1400};
    expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
    s = servoState(); in.nowUs = 1; expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
    in.nowUs = UINT64_MAX-100; s.timeUs = in.nowUs;
    expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
}

TEST(TricopterDynamicAllocator, ParameterAndInputNonfiniteGuardsSurviveFastMath)
{
    const auto m = model(); const auto s = servoState();
    for (float bad : {std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}) {
        for (unsigned field = 0; field < 7; ++field) {
            auto p = parameters();
            switch (field) {
            case 0: p.motorTauIncreasingS[0] = bad; break;
            case 1: p.motorTauDecreasingS[0] = bad; break;
            case 2: p.motorSlewIncreasingRpmS[0] = bad; break;
            case 3: p.pwmGridUs[0] = bad; break;
            case 4: p.maxHighMatrixCondition = bad; break;
            case 5: p.highCostTolerance = bad; break;
            case 6: p.yawCostTolerance = bad; break;
            }
            EXPECT_FALSE(tricopterDynamicAllocatorValidateParams(&m,&p,&s.params));
            const auto in = input();
            expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
        }
        auto in = input(); in.requestedResponse[3] = bad; const auto p = parameters();
        expectUnavailable(tricopterDynamicAllocatorAllocate(&m,&p,&s,&in));
    }
}

static void measuredMap(tricopterDynamicAllocatorModel_t *m)
{
    m->measuredAngleConfigured=true; auto &a=m->measuredAngle; a.knotCount=3; a.operatorDefinedReference=true;
    a.correctedCounts[0]=1000; a.correctedCounts[1]=1500; a.correctedCounts[2]=2000;
    a.angleRad[0]=.6f; a.angleRad[1]=.2f; a.angleRad[2]=-.6f;
    a.halfBranchSeparationRad[0]=.01f; a.halfBranchSeparationRad[1]=.03f; a.halfBranchSeparationRad[2]=.02f;
}
TEST(TricopterDynamicAllocator, MeasuredMapForecastsUseNonlinearCountsConversionAndPreservePriority)
{
    auto m=model(); measuredMap(&m); const auto p=parameters(); const auto s=servoState(); auto in=input();
    in.requestedResponse[0]=14; in.requestedResponse[3]=100;
    const auto out=tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    bool differs=false;
    for(const auto &c : out.candidates) if(c.valid) {
        const auto a=tricopterAngleCalibrationEvaluate(&m.measuredAngle,c.horizonCorrectedCounts);
        EXPECT_TRUE(c.measuredAngleAvailable); EXPECT_EQ(c.horizonDirectionRad,a.angleRad);
        EXPECT_EQ(c.angleHalfBranchSeparationRad,a.halfBranchSeparationRad);
        const float old=.6f-(c.horizonCorrectedCounts-1000)*1.2f/1000;
        if(fabs(old-c.horizonDirectionRad)>.01)differs=true;
        EXPECT_LE(out.minimumHighCost,c.highCost);
    }
    EXPECT_TRUE(differs); EXPECT_LE(out.selected.highCost,out.minimumHighCost+p.highCostTolerance);
}
TEST(TricopterDynamicAllocator, MeasuredMapRejectsUnsupportedLimitsAndRestrictsPwmBounds)
{
    auto m=model(); measuredMap(&m); auto p=parameters(); const auto s=servoState();
    m.allocator.tailDirectionMaxRad=.7f;
    EXPECT_FALSE(tricopterDynamicAllocatorValidateParams(&m,&p,&s.params));
    m.allocator.tailDirectionMaxRad=.4f; m.allocator.tailDirectionMinRad=-.2f;
    p.gridCount=0;
    ASSERT_TRUE(tricopterDynamicAllocatorValidateParams(&m,&p,&s.params));
    const auto in=input(); const auto out=tricopterDynamicAllocatorAllocate(&m,&p,&s,&in); expectValid(out);
    EXPECT_NEAR(out.minimumPwmUs,1250,1e-3); EXPECT_NEAR(out.maximumPwmUs,1550,1e-3);
}
