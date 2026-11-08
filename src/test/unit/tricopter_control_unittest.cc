#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
extern "C" {
#include "flight/tricopter_control.h"
}
#include "gtest/gtest.h"

static constexpr double PI = 3.14159265358979323846;
static tricopterControlInput_t input(float vertical=.5f, float yaw=0, float counts=1490)
{
    return {vertical,yaw,counts,0,true};
}
static bool finiteBits(float f)
{
    uint32_t bits; std::memcpy(&bits,&f,sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}
static void finiteBounded(const tricopterControlOutput_t &o)
{
    EXPECT_TRUE(finiteBits(o.tailEffort)); EXPECT_GE(o.tailEffort,0); EXPECT_LE(o.tailEffort,1);
    EXPECT_TRUE(finiteBits(o.pwmUs)); EXPECT_GE(o.pwmUs,500); EXPECT_LE(o.pwmUs,2500);
    EXPECT_TRUE(finiteBits(o.targetAngleDeg)); EXPECT_TRUE(finiteBits(o.actualAngleDeg));
    EXPECT_TRUE(finiteBits(o.currentVertical)); EXPECT_TRUE(finiteBits(o.currentLateral));
}
TEST(TricopterControl, DefaultZeroUsesMeasuredPositionAndUnchangedVerticalEffort)
{
    const auto p=tricopterControlDefaultParams(); const auto in=input();
    ASSERT_TRUE(tricopterControlValidateParams(&p));
    const auto o=tricopterControlEvaluate(&p,&in); finiteBounded(o);
    EXPECT_TRUE(o.validConfig); EXPECT_TRUE(o.validInput); EXPECT_TRUE(o.feedbackValid); EXPECT_TRUE(o.currentVectorAvailable);
    EXPECT_EQ(o.actualAngleDeg,0); EXPECT_EQ(o.targetAngleDeg,0); EXPECT_EQ(o.pwmUs,1509);
    EXPECT_EQ(o.tailEffort,.5f); EXPECT_EQ(o.currentVertical,.5f); EXPECT_EQ(o.currentLateral,0);
    EXPECT_FALSE(o.yawSaturated); EXPECT_FALSE(o.verticalSaturated);
}
TEST(TricopterControl, CompensationUsesActualAngleEvenWhenDesiredAngleIsZero)
{
    const auto p=tricopterControlDefaultParams(); const auto in=input(.5f,0,1281);
    const auto o=tricopterControlEvaluate(&p,&in); finiteBounded(o);
    EXPECT_NEAR(o.actualAngleDeg,21.6,1e-5); EXPECT_EQ(o.targetAngleDeg,0);
    EXPECT_NEAR(o.tailEffort,.5/std::cos(21.6*PI/180),1e-6);
    EXPECT_NEAR(o.currentVertical,.5,1e-6); EXPECT_NEAR(o.currentLateral,.5*std::tan(21.6*PI/180),1e-6);
    EXPECT_NEAR(o.pwmUs,1509-2*21.6,1e-3);
}
TEST(TricopterControl, DesiredTiltNeverBecomesMeasuredTiltOrPrematureMotorBoost)
{
    auto p=tricopterControlDefaultParams(); p.positionGainUsPerDeg=0;
    const auto in=input(.5f,.2f); const auto o=tricopterControlEvaluate(&p,&in);
    EXPECT_NEAR(o.targetAngleDeg,std::atan2(.14,.5)*180/PI,1e-5);
    EXPECT_EQ(o.actualAngleDeg,0); EXPECT_EQ(o.tailEffort,.5f); EXPECT_EQ(o.currentLateral,0);
    EXPECT_FALSE(o.yawSaturated); EXPECT_FALSE(o.verticalSaturated);
}
TEST(TricopterControl, ThreePointAdcAndPwmInterpolationSupportEitherPhysicalDirection)
{
    auto p=tricopterControlDefaultParams();
    float angle=0,pwm=0;
    ASSERT_TRUE(tricopterControlAngleFromAdc(&p,(1936+1490)/2.f,&angle)); EXPECT_NEAR(angle,-23.9,1e-5);
    ASSERT_TRUE(tricopterControlPwmFromAngle(&p,-23.9f,&pwm)); EXPECT_NEAR(pwm,(1100+1509)/2.f,1e-4);
    p.adcCounts[0]=1000;p.adcCounts[1]=1500;p.adcCounts[2]=2000;
    p.pwmUs[0]=1700;p.pwmUs[1]=1509;p.pwmUs[2]=1100;
    ASSERT_TRUE(tricopterControlValidateParams(&p));
    ASSERT_TRUE(tricopterControlAngleFromAdc(&p,1750,&angle)); EXPECT_NEAR(angle,10.8,1e-5);
    ASSERT_TRUE(tricopterControlPwmFromAngle(&p,10.8f,&pwm)); EXPECT_NEAR(pwm,(1509+1100)/2.f,1e-4);
}
TEST(TricopterControl, PositionCorrectionRespectsPwmMapDirection)
{
    auto p=tricopterControlDefaultParams(); auto in=input(.5f,.1f);
    auto o=tricopterControlEvaluate(&p,&in);float ff=0;
    ASSERT_TRUE(tricopterControlPwmFromAngle(&p,o.targetAngleDeg,&ff));
    EXPECT_GT(o.pwmUs,ff);
    p.pwmUs[0]=1700;p.pwmUs[2]=1100;
    o=tricopterControlEvaluate(&p,&in);
    ASSERT_TRUE(tricopterControlPwmFromAngle(&p,o.targetAngleDeg,&ff));
    EXPECT_LT(o.pwmUs,ff); EXPECT_LT(o.pwmUs,1509);
}
TEST(TricopterControl, VerticalFeasibilityConstrainsTargetAndActualSaturationIsVisible)
{
    const auto p=tricopterControlDefaultParams(); const auto in=input(.95f,-.8f,1936);
    const auto o=tricopterControlEvaluate(&p,&in); finiteBounded(o);
    EXPECT_NEAR(o.targetAngleDeg,-std::acos(.95f)*180/PI,1e-5);
    EXPECT_EQ(o.tailEffort,1); EXPECT_TRUE(o.yawSaturated); EXPECT_TRUE(o.verticalSaturated);
    EXPECT_NEAR(o.currentVertical,std::cos(47.8*PI/180),1e-6);
    EXPECT_LT(o.currentVertical,.95);
}
TEST(TricopterControl, FullVerticalDemandLeavesNoRequestedTiltHeadroom)
{
    const auto p=tricopterControlDefaultParams();const auto in=input(1,1);
    const auto o=tricopterControlEvaluate(&p,&in);
    EXPECT_EQ(o.targetAngleDeg,0); EXPECT_EQ(o.tailEffort,1); EXPECT_EQ(o.pwmUs,1509);
    EXPECT_TRUE(o.yawSaturated); EXPECT_FALSE(o.verticalSaturated);
}
TEST(TricopterControl, ZeroAndLowThrottleAreFiniteAndReportLateralAuthorityLimit)
{
    const auto p=tricopterControlDefaultParams();auto in=input(0,.01f,1281);
    auto o=tricopterControlEvaluate(&p,&in); finiteBounded(o);
    EXPECT_GT(o.targetAngleDeg,0); EXPECT_EQ(o.tailEffort,0);
    EXPECT_EQ(o.currentVertical,0); EXPECT_EQ(o.currentLateral,0); EXPECT_TRUE(o.yawSaturated);
    in=input(.01f,.01f);o=tricopterControlEvaluate(&p,&in); finiteBounded(o);
    EXPECT_TRUE(o.yawSaturated); EXPECT_EQ(o.tailEffort,.01f);
}
TEST(TricopterControl, HardwarePwmIntersectionAlsoRestrictsAngleGoal)
{
    auto p=tricopterControlDefaultParams(); p.pwmMinUs=1450;p.pwmMaxUs=1580;
    const auto in=input(.5f,1);const auto o=tricopterControlEvaluate(&p,&in);
    EXPECT_LE(o.pwmUs,1580);EXPECT_GE(o.pwmUs,1450);EXPECT_TRUE(o.yawSaturated);
    EXPECT_NEAR(o.targetAngleDeg,(1580-1509)*21.6/(1700-1509),1e-5);
}
TEST(TricopterControl, DirectionAndTrimDoNotRewritePhysicalCalibration)
{
    auto p=tricopterControlDefaultParams();auto in=input(.5f,.1f);
    const auto positive=tricopterControlEvaluate(&p,&in);p.yawDirection=-1;
    const auto negative=tricopterControlEvaluate(&p,&in);
    EXPECT_GT(positive.targetAngleDeg,0);EXPECT_LT(negative.targetAngleDeg,0);
    EXPECT_EQ(positive.actualAngleDeg,negative.actualAngleDeg);
    p.neutralTrimDeg=5;in.normalizedYaw=0;
    const auto trim=tricopterControlEvaluate(&p,&in);
    EXPECT_EQ(trim.targetAngleDeg,5);EXPECT_EQ(trim.actualAngleDeg,0);
    p.yawDirection=1;EXPECT_EQ(tricopterControlEvaluate(&p,&in).targetAngleDeg,5);
}
TEST(TricopterControl, InvalidOrStaleFeedbackUsesOnlyFeedForwardAndUncompensatedBase)
{
    auto p=tricopterControlDefaultParams();auto in=input(.8f,.1f,1936);in.feedbackValid=false;
    auto o=tricopterControlEvaluate(&p,&in);float ff=0;
    ASSERT_TRUE(tricopterControlPwmFromAngle(&p,o.targetAngleDeg,&ff));
    EXPECT_FALSE(o.feedbackValid);EXPECT_FALSE(o.currentVectorAvailable);EXPECT_EQ(o.tailEffort,.8f);EXPECT_EQ(o.pwmUs,ff);
    in.feedbackValid=true;in.feedbackAgeUs=p.feedbackTimeoutUs+1;
    o=tricopterControlEvaluate(&p,&in);EXPECT_FALSE(o.feedbackValid);EXPECT_EQ(o.tailEffort,.8f);
    in.feedbackAgeUs=p.feedbackTimeoutUs;
    EXPECT_TRUE(tricopterControlEvaluate(&p,&in).feedbackValid);
}
TEST(TricopterControl, AdcMarginClampsNoiseButLargerExcursionsRemainUnavailable)
{
    const auto p=tricopterControlDefaultParams();float angle=42;
    ASSERT_TRUE(tricopterControlAngleFromAdc(&p,1281-32,&angle));EXPECT_NEAR(angle,21.6,1e-5);
    ASSERT_TRUE(tricopterControlAngleFromAdc(&p,1936+32,&angle));EXPECT_NEAR(angle,-47.8,1e-5);
    angle=42;EXPECT_FALSE(tricopterControlAngleFromAdc(&p,1281-33,&angle));EXPECT_EQ(angle,42);
    auto in=input(.5f,0,1281-33);const auto o=tricopterControlEvaluate(&p,&in);
    EXPECT_FALSE(o.feedbackValid);EXPECT_FALSE(o.currentVectorAvailable);EXPECT_EQ(o.tailEffort,.5f);
}
TEST(TricopterControl, TransientAngleErrorAloneDoesNotDeclareYawSaturation)
{
    const auto p=tricopterControlDefaultParams();const auto in=input(.25f,.01f,1281);
    const auto o=tricopterControlEvaluate(&p,&in);
    EXPECT_NE(o.targetAngleDeg,o.actualAngleDeg); EXPECT_FALSE(o.yawSaturated); EXPECT_FALSE(o.verticalSaturated);
}
TEST(TricopterControl, NonfiniteFeedbackCannotBecomeMeasuredAngleUnderFastMath)
{
    const auto p=tricopterControlDefaultParams();
    for(float c : {std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}) {
        const auto in=input(.4f,.1f,c);const auto o=tricopterControlEvaluate(&p,&in);finiteBounded(o);
        EXPECT_FALSE(o.feedbackValid);EXPECT_FALSE(o.currentVectorAvailable);EXPECT_EQ(o.tailEffort,.4f);
    }
}
TEST(TricopterControl, NonfiniteDemandAndInvalidConfigurationStayFiniteAndUnboosted)
{
    auto p=tricopterControlDefaultParams();auto in=input(.4f,std::numeric_limits<float>::quiet_NaN());
    auto o=tricopterControlEvaluate(&p,&in);finiteBounded(o);EXPECT_FALSE(o.validInput);EXPECT_EQ(o.tailEffort,.4f);
    in=input(std::numeric_limits<float>::quiet_NaN());o=tricopterControlEvaluate(&p,&in);
    finiteBounded(o);EXPECT_FALSE(o.validInput);EXPECT_EQ(o.tailEffort,0);
    in=input();o=tricopterControlEvaluate(nullptr,&in);finiteBounded(o);EXPECT_FALSE(o.validConfig);EXPECT_EQ(o.tailEffort,.5f);
    p.pwmUs[1]=std::numeric_limits<float>::quiet_NaN();p.pwmMinUs=1200;p.pwmMaxUs=1400;
    o=tricopterControlEvaluate(&p,&in);finiteBounded(o);EXPECT_FALSE(o.validConfig);EXPECT_EQ(o.pwmUs,1400);
}
TEST(TricopterControl, MalformedParametersAndUnboundedHelpersAreRejected)
{
    for(unsigned kind=0;kind<13;++kind) {
        auto p=tricopterControlDefaultParams();
        if(kind==0)p.positionGainUsPerDeg=std::numeric_limits<float>::quiet_NaN();
        if(kind==1)p.adcCounts[1]=p.adcCounts[0];
        if(kind==2)p.pwmUs[1]=p.pwmUs[0];
        if(kind==3)p.angleDeg[1]=1;
        if(kind==4)p.angleDeg[2]=81;
        if(kind==5)p.feedbackTimeoutUs=0;
        if(kind==6)p.pwmMinUs=1600;
        if(kind==7)p.yawDirection=0;
        if(kind==8)p.minVerticalEffort=0;
        if(kind==9)p.adcMarginCounts=-1;
        if(kind==10)p.neutralTrimDeg=std::numeric_limits<float>::infinity();
        if(kind==11)p.yawGain=-.1f;
        if(kind==12)p.adcCounts[2]=5000;
        EXPECT_FALSE(tricopterControlValidateParams(&p));
        const auto in=input();const auto o=tricopterControlEvaluate(&p,&in);finiteBounded(o);EXPECT_FALSE(o.validConfig);
    }
    const auto p=tricopterControlDefaultParams();float value=42;
    EXPECT_FALSE(tricopterControlPwmFromAngle(&p,22,&value));EXPECT_EQ(value,42);
    EXPECT_FALSE(tricopterControlAngleFromAdc(&p,1490,nullptr));
    EXPECT_FALSE(tricopterControlPwmFromAngle(&p,0,nullptr));
}
TEST(TricopterControl, SignedAdcMapsAndExtremeValidAnglesRemainBounded)
{
    auto p=tricopterControlDefaultParams();p.adcCounts[0]=-1000;p.adcCounts[1]=-1500;p.adcCounts[2]=-2000;
    const auto in=input(.5f,.1f,-1750);auto o=tricopterControlEvaluate(&p,&in);finiteBounded(o);
    EXPECT_TRUE(o.feedbackValid);EXPECT_NEAR(o.actualAngleDeg,10.8,1e-5);
    p.angleDeg[0]=-80;p.angleDeg[2]=80;
    const auto extreme=input(1,0,-1000);o=tricopterControlEvaluate(&p,&extreme);finiteBounded(o);
    EXPECT_TRUE(o.feedbackValid);EXPECT_TRUE(o.verticalSaturated);EXPECT_EQ(o.tailEffort,1);
}
