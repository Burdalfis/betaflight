#include <cmath>
#include <limits>
extern "C" {
#include "flight/tricopter_angle_calibration.h"
}
#include "gtest/gtest.h"

static tricopterAngleCalibration_t map()
{
    tricopterAngleCalibration_t m = {}; m.knotCount=3; m.operatorDefinedReference=true;
    m.correctedCounts[0]=1000; m.correctedCounts[1]=1500; m.correctedCounts[2]=2000;
    m.angleRad[0]=.6f; m.angleRad[1]=.2f; m.angleRad[2]=-.6f;
    m.halfBranchSeparationRad[0]=.01f; m.halfBranchSeparationRad[1]=.03f; m.halfBranchSeparationRad[2]=.02f;
    return m;
}
TEST(TricopterAngleCalibration, IndependentPiecewiseValuesAndEnvelope)
{
    const auto m=map(); const auto a=tricopterAngleCalibrationEvaluate(&m,1250);
    ASSERT_TRUE(a.valid); EXPECT_TRUE(a.operatorDefinedReference);
    EXPECT_NEAR(a.angleRad,.4,1e-7); EXPECT_NEAR(a.halfBranchSeparationRad,.02,1e-7);
    const auto b=tricopterAngleCalibrationEvaluate(&m,1750);
    EXPECT_NEAR(b.angleRad,-.2,1e-7); EXPECT_NEAR(b.halfBranchSeparationRad,.025,1e-7);
}
TEST(TricopterAngleCalibration, EndpointsAndInverseBothSigns)
{
    auto m=map();
    for (unsigned sign=0;sign<2;++sign) {
        for(float c : {1000.f,1250.f,1500.f,1750.f,2000.f}) {
            const auto a=tricopterAngleCalibrationEvaluate(&m,c); ASSERT_TRUE(a.valid);
            float roundtrip=0; ASSERT_TRUE(tricopterAngleCalibrationInverse(&m,a.angleRad,&roundtrip));
            EXPECT_NEAR(roundtrip,c,.0002);
        }
        for(unsigned i=0;i<3;++i)m.angleRad[i]=-m.angleRad[i];
    }
}
TEST(TricopterAngleCalibration, UnsupportedCountsAndAnglesNeverExtrapolateOrMutateInverse)
{
    const auto m=map();
    for(float c : {999.f,2001.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()})
        EXPECT_FALSE(tricopterAngleCalibrationEvaluate(&m,c).valid);
    float out=42;
    EXPECT_FALSE(tricopterAngleCalibrationInverse(&m,.7f,&out)); EXPECT_EQ(out,42);
    EXPECT_FALSE(tricopterAngleCalibrationInverse(&m,std::numeric_limits<float>::quiet_NaN(),&out));
    EXPECT_FALSE(tricopterAngleCalibrationInverse(&m,0,nullptr));
}
TEST(TricopterAngleCalibration, MalformedAndUnreferencedMapsFail)
{
    EXPECT_FALSE(tricopterAngleCalibrationValidate(nullptr));
    for(unsigned kind=0;kind<9;++kind) {
        auto m=map();
        if(kind==0)m.knotCount=1;
        if(kind==1)m.knotCount=65;
        if(kind==2)m.correctedCounts[1]=m.correctedCounts[0];
        if(kind==3)m.angleRad[1]=m.angleRad[0];
        if(kind==4)m.angleRad[1]=-.7f;
        if(kind==5)m.halfBranchSeparationRad[1]=-.1f;
        if(kind==6)m.angleRad[1]=std::numeric_limits<float>::quiet_NaN();
        if(kind==7)m.operatorDefinedReference=false;
        if(kind==8)m.correctedCounts[1]=5000;
        EXPECT_FALSE(tricopterAngleCalibrationValidate(&m));
        EXPECT_FALSE(tricopterAngleCalibrationEvaluate(&m,1500).valid);
    }
}
TEST(TricopterAngleCalibration, DirectionChangesDoNotSwitchOrJumpMajorBranches)
{
    const auto m=map();
    const auto a=tricopterAngleCalibrationEvaluate(&m,1499.99f);
    const auto b=tricopterAngleCalibrationEvaluate(&m,1500);
    const auto c=tricopterAngleCalibrationEvaluate(&m,1500.01f);
    EXPECT_LT(fabs(a.angleRad-b.angleRad),.0001);
    EXPECT_LT(fabs(c.angleRad-b.angleRad),.0001);
    EXPECT_EQ(tricopterAngleCalibrationEvaluate(&m,1500).angleRad,b.angleRad);
}
