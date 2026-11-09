/* Live adapter contract: link only mixer_tricopter/control/calibration cores.
 * Hardware acquisition and motor arrays are independent test stubs. No 58-knot
 * map, prediction, RPM observer or torque allocator is linked into this target.
 */
#include <algorithm>
#include <cmath>
#include <cstring>

extern "C" {
#include "platform.h"
#include "common/filter.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/servos.h"
#include "flight/tricopter_servo_position.h"

servoParam_t servoParams_SystemArray[MAX_SUPPORTED_SERVOS];
uint8_t armingFlags;
mixerRuntime_t mixerRuntime;
float motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];
}

#include "gtest/gtest.h"

static tricopterServoPositionSample_t position;
static bool tricopterMode;
static unsigned positionReads;
static timeUs_t lastPositionQuery;
static timeUs_t fakeNow;

extern "C" timeUs_t micros(void) { return fakeNow; }

extern "C" tricopterServoPositionSample_t servosTricopterGetPosition(timeUs_t now)
{
    ++positionReads; lastPositionQuery=now; return position;
}
extern "C" bool mixerIsTricopter(void) { return tricopterMode; }

class TricopterFeedback : public ::testing::Test {
protected:
    void SetUp() override
    {
        std::memset(servoParams_SystemArray,0,sizeof(servoParams_SystemArray));
        servoParam_t *s=servoParamsMutable(SERVO_RUDDER);
        s->min=1100;s->middle=1509;s->max=1700;s->rate=100;s->forwardFromChannel=CHANNEL_FORWARDING_DISABLED;
        auto *cfg=tricopterMixerConfigMutable();
        std::memset(cfg,0,sizeof(*cfg));
        cfg->angleDeciDeg[0]=-478;cfg->angleDeciDeg[1]=0;cfg->angleDeciDeg[2]=216;
        cfg->adcCounts[0]=1936;cfg->adcCounts[1]=1490;cfg->adcCounts[2]=1281;
        cfg->yawGainPermille=700;cfg->positionGainDeciUsPerDeg=20;
        cfg->feedbackTimeoutMs=20;cfg->adcMarginCounts=32;cfg->yawDirection=1;
        cfg->observedServoSpeedDps=0;
        std::memset(&mixerRuntime,0,sizeof(mixerRuntime));
        mixerRuntime.motorCount=3;mixerRuntime.disarmMotorOutput=1000;
        std::fill(motor,motor+MAX_SUPPORTED_MOTORS,1000);
        std::fill(motor_disarmed,motor_disarmed+MAX_SUPPORTED_MOTORS,1000);
        position={true,true,true,true,1490,0,1490,0};
        armingFlags=0;tricopterMode=true;positionReads=0;lastPositionQuery=0;fakeNow=100000;
        mixerTricopterInit();
    }
    void update(float vertical=.5f,float yaw=0,timeUs_t now=100000)
    {
        fakeNow=now;
        mixerTricopterUpdate(now,vertical,yaw);
    }
    bool completeCalibration(timeUs_t *finished)
    {
        // The newly observed center is1700counts, deliberately inconsistent with
        // the old1490count map. No ADC result is applied until the explicit call.
        fakeNow=100000;
        if (!mixerTricopterCalibrationStart(fakeNow)) return false;
        for (unsigned i=1;i<=1400;++i) {
            const float pwm=mixerTricopterGetCalibration()->commandedPwmUs;
            const float counts=pwm<=1509 ? 2100+(pwm-1100)*(1700-2100)/(1509-1100)
                : 1700+(pwm-1509)*(1300-1700)/(1700-1509);
            position={true,true,true,true,static_cast<int16_t>(std::lround(counts)),0,
                static_cast<uint16_t>(std::lround(counts)),0};
            update(.5f,0,100000+i*5000);
            const auto *result=mixerTricopterGetCalibration();
            if (result->failed) return false;
            if (result->complete) { *finished=fakeNow; return result->resultValid; }
        }
        return false;
    }
};

TEST_F(TricopterFeedback, DefaultAdapterUsesCanonicalDifferenceForActualTailCorrectionOnly)
{
    // Wiper alone is at the zero count; subtracting local ground gives1281,
    // which is the measured positive endpoint. Raw PB0 must not replace it.
    position.rawWiperCounts=1490;position.rawGroundCounts=209;position.correctedCounts=1281;
    update(.5f,0,123456);
    const auto &o=*mixerTricopterGetControl();
    ASSERT_TRUE(o.feedbackValid);EXPECT_TRUE(o.currentVectorAvailable);
    EXPECT_NEAR(o.actualAngleDeg,21.6,1e-4);EXPECT_EQ(o.targetAngleDeg,0);
    EXPECT_NEAR(o.tailEffort,.5/std::cos(21.6*3.14159265358979323846/180),1e-6);
    EXPECT_NEAR(mixerTricopterMotorCorrection(0),o.tailEffort-.5,1e-7);
    EXPECT_EQ(mixerTricopterMotorCorrection(1),0);EXPECT_EQ(mixerTricopterMotorCorrection(2),0);
    EXPECT_EQ(mixerTricopterMotorCorrection(-1),0);EXPECT_EQ(mixerTricopterMotorCorrection(3),0);
    EXPECT_NEAR(o.currentVertical,.5,1e-6);EXPECT_GT(o.currentLateral,0);
    EXPECT_EQ(positionReads,1U);EXPECT_EQ(lastPositionQuery,123456U);
    EXPECT_LT(mixerTricopterServoPwm(),1509U);
}
TEST_F(TricopterFeedback, ZeroGroundCountIsValidAndNoFeedbackBoostComesFromDesiredAngle)
{
    update(.5f,.2f);
    const auto &o=*mixerTricopterGetControl();
    ASSERT_TRUE(o.feedbackValid);EXPECT_EQ(o.actualAngleDeg,0);EXPECT_GT(o.targetAngleDeg,0);
    EXPECT_EQ(mixerTricopterMotorCorrection(0),0);EXPECT_EQ(o.currentLateral,0);
    EXPECT_TRUE(mixerTricopterCanArm());
}
TEST_F(TricopterFeedback, StaleOrInvalidRawFeedbackCannotDriveCompensationOrArming)
{
    for(unsigned kind=0;kind<8;++kind) {
        position={true,true,true,true,1281,0,1281,0};
        if(kind==0)position.ageUs=20001;
        if(kind==1)position.supported=false;
        if(kind==2)position.ready=false;
        if(kind==3)position.sampleSeen=false;
        if(kind==4)position.rawWiperCounts=0;
        if(kind==5)position.rawWiperCounts=4095;
        if(kind==6)position.rawGroundCounts=4095;
        if(kind==7)position.correctedCounts=TRICOPTER_SERVO_POSITION_UNAVAILABLE;
        update(.4f,.1f,100000+kind*1000);
        const auto &o=*mixerTricopterGetControl();
        EXPECT_FALSE(o.feedbackValid);EXPECT_FALSE(o.currentVectorAvailable);
        EXPECT_EQ(o.tailEffort,.4f);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);EXPECT_EQ(mixerTricopterCollectiveScale(),1);
        EXPECT_FALSE(mixerTricopterCanArm());EXPECT_TRUE(mixerTricopterIsServoSaturated(0));
        EXPECT_GE(mixerTricopterServoPwm(),1100U);EXPECT_LE(mixerTricopterServoPwm(),1700U);
    }
}
TEST_F(TricopterFeedback, ConfiguredFeedbackTimeoutIsAuthoritativeAtItsBoundary)
{
    position.ageUs=20000;position.fresh=false; // Accessor's1ms diagnostic freshness differs from configured20ms hold limit.
    update();EXPECT_TRUE(mixerTricopterGetControl()->feedbackValid);
    position.ageUs=20001;update();EXPECT_FALSE(mixerTricopterGetControl()->feedbackValid);
}
TEST_F(TricopterFeedback, RawPidSignAndAbsoluteRatePreserveServoConvention)
{
    update(.5f,.2f);const float positive=mixerTricopterGetControl()->targetAngleDeg;
    EXPECT_GT(positive,0);EXPECT_GT(mixerTricopterServoPwm(),1509U);
    update(.5f,-.2f);EXPECT_NEAR(mixerTricopterGetControl()->targetAngleDeg,-positive,1e-5);
    EXPECT_LT(mixerTricopterServoPwm(),1509U);
    servoParamsMutable(SERVO_RUDDER)->rate=50;update(.5f,.2f);
    const float half=mixerTricopterGetControl()->targetAngleDeg;
    EXPECT_NEAR(half,std::atan2(.07,.5)*180/3.14159265358979323846,1e-4);
    EXPECT_LT(half,positive);
    servoParamsMutable(SERVO_RUDDER)->rate=-50;update(.5f,.2f);
    EXPECT_NEAR(mixerTricopterGetControl()->targetAngleDeg,-half,1e-5);
}
TEST_F(TricopterFeedback, SourceReversalAndYawDirectionDoNotAlterPhysicalAdcCalibration)
{
    servoParamsMutable(SERVO_RUDDER)->reversedSources=1U<<INPUT_STABILIZED_YAW;
    update(.5f,.1f);EXPECT_LT(mixerTricopterGetControl()->targetAngleDeg,0);
    EXPECT_EQ(mixerTricopterGetControl()->actualAngleDeg,0);
    tricopterMixerConfigMutable()->yawDirection=-1;update(.5f,.1f);
    EXPECT_GT(mixerTricopterGetControl()->targetAngleDeg,0);
    tricopterMixerConfigMutable()->neutralTrimDeciDeg=50;update(.5f,0);
    EXPECT_EQ(mixerTricopterGetControl()->targetAngleDeg,5);
    EXPECT_EQ(mixerTricopterGetControl()->actualAngleDeg,0);
}
TEST_F(TricopterFeedback, DisarmedRateZeroRetainsFixedBenchPulseAndBlocksArming)
{
    auto *s=servoParamsMutable(SERVO_RUDDER);s->middle=1550;s->rate=0;
    position.correctedCounts=1281;position.rawWiperCounts=1281;
    update(1,.2f);
    EXPECT_EQ(mixerTricopterServoPwm(),1550U);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    EXPECT_EQ(mixerTricopterCollectiveScale(),1);
    EXPECT_FALSE(mixerTricopterCanArm());
}
TEST_F(TricopterFeedback, ArmingNeedsValidFeedbackConfigurationAndNoChannelForwarding)
{
    EXPECT_FALSE(mixerTricopterCanArm());update();ASSERT_TRUE(mixerTricopterCanArm());
    servoParamsMutable(SERVO_RUDDER)->forwardFromChannel=0;
    EXPECT_FALSE(mixerTricopterCanArm());
    servoParamsMutable(SERVO_RUDDER)->forwardFromChannel=CHANNEL_FORWARDING_DISABLED;
    tricopterMixerConfigMutable()->adcCounts[1]=1936;update();
    EXPECT_FALSE(mixerTricopterCanArm());EXPECT_FALSE(mixerTricopterGetControl()->validConfig);
}
TEST_F(TricopterFeedback, CalibrationRejectsArmedMotorTestWrongMixerAndDuplicateStart)
{
    armingFlags=ARMED;EXPECT_FALSE(mixerTricopterCalibrationStart(100000));armingFlags=0;
    motor_disarmed[1]=1050;EXPECT_FALSE(mixerTricopterCalibrationStart(100000));motor_disarmed[1]=1000;
    motor[0]=1001;EXPECT_FALSE(mixerTricopterCalibrationStart(100000));motor[0]=1000;
    mixerRuntime.motorCount=2;EXPECT_FALSE(mixerTricopterCalibrationStart(100000));mixerRuntime.motorCount=3;
    tricopterMode=false;EXPECT_FALSE(mixerTricopterCalibrationStart(100000));tricopterMode=true;
    ASSERT_TRUE(mixerTricopterCalibrationStart(100000));
    EXPECT_FALSE(mixerTricopterCalibrationStart(100001));EXPECT_TRUE(mixerTricopterIsCalibrating());
    update(.5f,.1f,105000);EXPECT_FALSE(mixerTricopterCanArm());EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
}
TEST_F(TricopterFeedback, CalibrationMotorTestAbortReturnsMiddleAndKeepsCorrectionZero)
{
    ASSERT_TRUE(mixerTricopterCalibrationStart(100000));
    motor_disarmed[2]=1100;update(.5f,.1f,105000);
    EXPECT_TRUE(mixerTricopterIsCalibrating());EXPECT_TRUE(mixerTricopterGetCalibration()->failed);
    EXPECT_EQ(mixerTricopterServoPwm(),1509U);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    EXPECT_FALSE(mixerTricopterCalibrationApply());EXPECT_FALSE(mixerTricopterCanArm());
    motor_disarmed[2]=1000;
    update(.5f,0,405001);EXPECT_FALSE(mixerTricopterIsCalibrating());
}
TEST_F(TricopterFeedback, CalibrationUserAbortAndApplyCannotOverwriteLearnedConfiguration)
{
    ASSERT_TRUE(mixerTricopterCalibrationStart(100000));
    mixerTricopterCalibrationAbort();EXPECT_TRUE(mixerTricopterIsCalibrating());
    EXPECT_EQ(mixerTricopterServoPwm(),1509U);EXPECT_FALSE(mixerTricopterCalibrationApply());
    EXPECT_EQ(tricopterMixerConfig()->adcCounts[0],1936);EXPECT_EQ(tricopterMixerConfig()->adcCounts[1],1490);
    update(.5f,0,100001);EXPECT_FALSE(mixerTricopterCanArm());EXPECT_TRUE(mixerTricopterIsCalibrating());
    update(.5f,0,400001);EXPECT_FALSE(mixerTricopterIsCalibrating());EXPECT_TRUE(mixerTricopterCanArm());
}

TEST_F(TricopterFeedback, CompletedCalibrationHoldsCenterAgainstOldWrongMapUntilExplicitApply)
{
    timeUs_t done=0;ASSERT_TRUE(completeCalibration(&done));
    const auto *result=mixerTricopterGetCalibration();
    EXPECT_FALSE(result->active);EXPECT_TRUE(result->resultValid);EXPECT_TRUE(mixerTricopterIsCalibrating());
    EXPECT_EQ(result->learnedAdc[0],2100);EXPECT_EQ(result->learnedAdc[1],1700);EXPECT_EQ(result->learnedAdc[2],1300);
    EXPECT_EQ(tricopterMixerConfig()->adcCounts[1],1490);
    update(.5f,0,done+5000);
    EXPECT_GT(std::fabs(mixerTricopterGetControl()->pwmUs-1509),10);
    EXPECT_EQ(mixerTricopterServoPwm(),1509U);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    EXPECT_FALSE(mixerTricopterCanArm());EXPECT_TRUE(mixerTricopterIsCalibrating());
    ASSERT_TRUE(mixerTricopterCalibrationApply());EXPECT_FALSE(mixerTricopterIsCalibrating());
    EXPECT_EQ(tricopterMixerConfig()->adcCounts[0],2100);EXPECT_EQ(tricopterMixerConfig()->adcCounts[1],1700);
    EXPECT_EQ(tricopterMixerConfig()->adcCounts[2],1300);
    update(.5f,0,done+10000);EXPECT_NEAR(mixerTricopterGetControl()->actualAngleDeg,0,1e-5);
    EXPECT_EQ(mixerTricopterServoPwm(),1509U);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    EXPECT_TRUE(mixerTricopterCanArm());
}
TEST_F(TricopterFeedback, ChangingPwmOrAnglesDuringCalibrationAbortsAndRetainsCenterReturn)
{
    for (unsigned kind=0;kind<2;++kind) {
        SetUp();ASSERT_TRUE(mixerTricopterCalibrationStart(100000));
        if (kind==0)servoParamsMutable(SERVO_RUDDER)->max=1650;
        else tricopterMixerConfigMutable()->angleDeciDeg[2]=200;
        update(.5f,0,105000);
        EXPECT_TRUE(mixerTricopterIsCalibrating());EXPECT_FALSE(mixerTricopterGetCalibration()->resultValid);
        EXPECT_EQ(mixerTricopterServoPwm(),1509U);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
        EXPECT_FALSE(mixerTricopterCanArm());EXPECT_FALSE(mixerTricopterCalibrationApply());
        EXPECT_EQ(tricopterMixerConfig()->adcCounts[1],1490);
        update(.5f,0,405001);EXPECT_FALSE(mixerTricopterIsCalibrating());
    }
}
TEST_F(TricopterFeedback, CompletedResultCannotApplyAfterEndpointEdit)
{
    timeUs_t done=0;ASSERT_TRUE(completeCalibration(&done));
    servoParamsMutable(SERVO_RUDDER)->max=1650;
    EXPECT_FALSE(mixerTricopterCalibrationApply());EXPECT_EQ(tricopterMixerConfig()->adcCounts[1],1490);
    update(.5f,0,done+5000);
    EXPECT_FALSE(mixerTricopterGetCalibration()->resultValid);EXPECT_TRUE(mixerTricopterIsCalibrating());
    EXPECT_EQ(mixerTricopterServoPwm(),1509U);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    EXPECT_FALSE(mixerTricopterCalibrationApply());
    servoParamsMutable(SERVO_RUDDER)->max=1700;
    EXPECT_FALSE(mixerTricopterCalibrationApply()); // Restoring geometry cannot revive a cancelled result.
}

TEST_F(TricopterFeedback, ExplicitAbortClosesCompletedSessionAndWaitsForCenterReturn)
{
    timeUs_t done=0;ASSERT_TRUE(completeCalibration(&done));
    mixerTricopterCalibrationAbort();
    EXPECT_TRUE(mixerTricopterIsCalibrating());EXPECT_FALSE(mixerTricopterGetCalibration()->resultValid);
    EXPECT_FALSE(mixerTricopterGetCalibration()->complete);EXPECT_FALSE(mixerTricopterGetCalibration()->speedEstimateValid);
    EXPECT_FALSE(mixerTricopterCalibrationApply());EXPECT_FALSE(mixerTricopterCanArm());
    update(.5f,0,done+300001);
    EXPECT_FALSE(mixerTricopterIsCalibrating());EXPECT_FALSE(mixerTricopterCalibrationApply());
}

TEST_F(TricopterFeedback, FullCollectiveReservesHeadroomGloballyAndKeepsYawAvailable)
{
    const double cosine=std::cos(21.6*3.14159265358979323846/180);
    position={true,true,true,true,1281,0,1281,0};
    update(1,0);
    const auto &o=*mixerTricopterGetControl();
    ASSERT_TRUE(o.feedbackValid);EXPECT_TRUE(o.verticalSaturated);EXPECT_FALSE(o.yawSaturated);
    EXPECT_FALSE(mixerTricopterIsServoSaturated(0));
    EXPECT_NEAR(mixerTricopterCollectiveScale(),cosine,1e-6);
    for(int i=0;i<3;++i) {
        const float output=mixerTricopterApplyMotorEffort(i,1);
        EXPECT_GE(output,0);EXPECT_LE(output,1);
        EXPECT_NEAR(i==0 ? output*cosine : output,cosine,1e-6);
    }
    EXPECT_EQ(mixerTricopterApplyMotorEffort(0,1),1);
    EXPECT_LT(mixerTricopterApplyMotorEffort(1,1),1);
    EXPECT_LT(mixerTricopterApplyMotorEffort(2,1),1);
}
TEST_F(TricopterFeedback, ReservePreservesThreeMotorRollPitchVerticalRatios)
{
    position={true,true,true,true,1936,0,1936,0};
    update(.95f,-.1f);
    const float requested[3]={.95f,.82f,.9f};
    const float cosine=std::cos(47.8*3.14159265358979323846/180);
    for(int i=0;i<3;++i) {
        const float output=mixerTricopterApplyMotorEffort(i,requested[i]);
        const float vertical=i==0 ? output*cosine : output;
        EXPECT_GE(output,0);EXPECT_LE(output,1);
        EXPECT_NEAR(vertical/requested[i],mixerTricopterCollectiveScale(),1e-6);
    }
    EXPECT_EQ(mixerTricopterApplyMotorEffort(-1,.7f),.7f);
    EXPECT_EQ(mixerTricopterApplyMotorEffort(3,.7f),.7f);
}
TEST_F(TricopterFeedback, LostFeedbackClearsCommonReserveWithoutInventingPosition)
{
    position={true,true,true,true,1936,0,1936,0};update(1,0);
    ASSERT_LT(mixerTricopterCollectiveScale(),1);
    position.ageUs=20001;update(1,0);
    EXPECT_EQ(mixerTricopterCollectiveScale(),1);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    for(int i=0;i<3;++i)EXPECT_EQ(mixerTricopterApplyMotorEffort(i,.9f),.9f);
    EXPECT_TRUE(mixerTricopterIsServoSaturated(0));
}

TEST_F(TricopterFeedback, DisarmedCalibrationSuppressesCommonReserveAndTailBoost)
{
    position={true,true,true,true,1936,0,1936,0};update(1,0);
    ASSERT_LT(mixerTricopterCollectiveScale(),1);
    ASSERT_TRUE(mixerTricopterCalibrationStart(fakeNow));update(1,0,fakeNow+1000);
    EXPECT_EQ(mixerTricopterCollectiveScale(),1);EXPECT_EQ(mixerTricopterMotorCorrection(0),0);
    for(int i=0;i<3;++i)EXPECT_EQ(mixerTricopterApplyMotorEffort(i,.8f),.8f);
}
