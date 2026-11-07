/* Causal mechanical-RPM software observer tests; no hardware calibration. */
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

extern "C" {
#include "flight/tricopter_rpm_observer.h"
}

#include "gtest/gtest.h"

static tricopterRpmObserverParams_t parameters()
{
    return {0.020f, 0.030f, 10000.0f, 100000.0f, 10000, 100000, 20000};
}

static tricopterRpmObservation_t observation(uint32_t time, uint32_t sequence, float rpm)
{
    return {true, time, sequence, rpm};
}

static tricopterRpmObserverOutput_t measure(tricopterRpmObserverState_t &state,
    uint32_t time, uint32_t sequence, float rpm)
{
    const auto sample = observation(time, sequence, rpm);
    return tricopterRpmObserverStep(&state, time, &sample);
}

static bool finiteBits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static void expectFinite(const tricopterRpmObserverOutput_t &out)
{
    EXPECT_TRUE(finiteBits(out.mechanicalRpm));
    EXPECT_TRUE(finiteBits(out.accelerationRpmS));
}

TEST(TricopterRpmObserver, ValidatesSoftwareConfiguration)
{
    auto p = parameters();
    EXPECT_TRUE(tricopterRpmObserverValidateParams(&p));
    EXPECT_FALSE(tricopterRpmObserverValidateParams(nullptr));
    for (unsigned field = 0; field < 7; ++field) {
        p = parameters();
        switch (field) {
        case 0: p.trackingTauS = 0; break;
        case 1: p.staleAccelerationDecayTauS = 0; break;
        case 2: p.maxRpm = 0; break;
        case 3: p.maxAccelerationRpmS = 0; break;
        case 4: p.staleAfterUs = 0; break;
        case 5: p.maxPredictionUs = p.staleAfterUs - 1; break;
        case 6: p.maxCallGapUs = UINT32_C(0x80000000); break;
        }
        EXPECT_FALSE(tricopterRpmObserverValidateParams(&p)) << field;
    }
    p = parameters(); p.maxPredictionUs = UINT32_C(0x80000000);
    EXPECT_FALSE(tricopterRpmObserverValidateParams(&p));
    p = parameters(); p.trackingTauS = 0.0000001f;
    EXPECT_FALSE(tricopterRpmObserverValidateParams(&p));
    p = parameters(); p.staleAccelerationDecayTauS = 61;
    EXPECT_FALSE(tricopterRpmObserverValidateParams(&p));
}

TEST(TricopterRpmObserver, RejectsNonfiniteConfigurationUnderFastMath)
{
    for (float bad : {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        for (unsigned field = 0; field < 4; ++field) {
            auto p = parameters();
            switch (field) {
            case 0: p.trackingTauS = bad; break;
            case 1: p.staleAccelerationDecayTauS = bad; break;
            case 2: p.maxRpm = bad; break;
            case 3: p.maxAccelerationRpmS = bad; break;
            }
            tricopterRpmObserverState_t state = {};
            EXPECT_FALSE(tricopterRpmObserverReset(&state, &p, 0));
            const auto out = tricopterRpmObserverStep(&state, 0, nullptr);
            EXPECT_FALSE(out.valid);
            EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_INVALID_CONFIG, 0U);
            expectFinite(out);
        }
    }
}

TEST(TricopterRpmObserver, ResetStartsUnseenAndCopiesAliasedConfiguration)
{
    auto p = parameters();
    tricopterRpmObserverState_t state = {};
    EXPECT_FALSE(tricopterRpmObserverReset(nullptr, &p, 0));
    EXPECT_TRUE(tricopterRpmObserverReset(&state, &p, 20));
    auto out = tricopterRpmObserverStep(&state, 20, nullptr);
    EXPECT_FALSE(out.seen); EXPECT_FALSE(out.valid); EXPECT_EQ(out.ageUs, UINT32_MAX);
    EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_UNSEEN, 0U);
    measure(state, 21, 900, 1234);
    EXPECT_TRUE(tricopterRpmObserverReset(&state, &state.params, 50));
    EXPECT_EQ(state.params.maxRpm, p.maxRpm);
    out = tricopterRpmObserverStep(&state, 50, nullptr);
    EXPECT_FALSE(out.seen);
    expectFinite(tricopterRpmObserverStep(nullptr, 0, nullptr));
}

TEST(TricopterRpmObserver, EqualAndZeroGenuineObservationsRefreshIdentity)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    for (uint32_t sequence = 0; sequence < 4; ++sequence) {
        const auto out = measure(state, sequence * 1000, sequence, 0);
        EXPECT_TRUE(out.acceptedNewObservation); EXPECT_TRUE(out.fresh); EXPECT_TRUE(out.valid);
        EXPECT_EQ(out.ageUs, 0U); EXPECT_EQ(out.sequence, sequence);
        EXPECT_EQ(out.mechanicalRpm, 0); EXPECT_EQ(out.accelerationRpmS, 0);
    }
}

TEST(TricopterRpmObserver, CachedSamplesAndMissingFramesNeverRefreshAge)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    auto cached = observation(0, 1, 1000);
    ASSERT_TRUE(tricopterRpmObserverStep(&state, 0, &cached).acceptedNewObservation);
    auto out = tricopterRpmObserverStep(&state, 5000, &cached);
    EXPECT_FALSE(out.acceptedNewObservation); EXPECT_EQ(out.ageUs, 5000U);
    auto missing = observation(6000, 90, std::numeric_limits<float>::quiet_NaN());
    missing.seen = false; // EDT/invalid/no-edge transport must map to no observation.
    out = tricopterRpmObserverStep(&state, 6000, &missing);
    EXPECT_FALSE(out.acceptedNewObservation); EXPECT_EQ(out.ageUs, 6000U);
    EXPECT_EQ(out.status, TRICOPTER_RPM_OBSERVER_OK);
    out = tricopterRpmObserverStep(&state, 11000, &cached);
    EXPECT_FALSE(out.fresh); EXPECT_TRUE(out.valid); EXPECT_EQ(out.ageUs, 11000U);
}

TEST(TricopterRpmObserver, UsesObservationTimeRatherThanDeliveryTime)
{
    const auto p = parameters();
    tricopterRpmObserverState_t direct = {}, delayed = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&direct, &p, 0));
    ASSERT_TRUE(tricopterRpmObserverReset(&delayed, &p, 0));
    auto sample = observation(1000, 1, 1000);
    tricopterRpmObserverStep(&direct, 1000, &sample);
    tricopterRpmObserverStep(&delayed, 1500, &sample);
    sample = observation(2000, 2, 1100);
    tricopterRpmObserverStep(&direct, 2000, &sample);
    const auto late = tricopterRpmObserverStep(&delayed, 2600, &sample);
    const auto queried = tricopterRpmObserverStep(&direct, 2600, nullptr);
    EXPECT_EQ(late.mechanicalRpm, queried.mechanicalRpm);
    EXPECT_EQ(late.accelerationRpmS, queried.accelerationRpmS);
    EXPECT_EQ(late.ageUs, 600U);
}

TEST(TricopterRpmObserver, RepeatedQueriesCannotIntegrateTwice)
{
    const auto p = parameters();
    tricopterRpmObserverState_t a = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&a, &p, 0));
    measure(a, 0, 1, 1000); measure(a, 1000, 2, 1200);
    auto b = a;
    for (uint32_t now = 1000; now <= 19000; now += 1000) {
        tricopterRpmObserverStep(&a, now, nullptr);
        tricopterRpmObserverStep(&a, now, nullptr);
    }
    const auto split = tricopterRpmObserverStep(&a, 20000, nullptr);
    const auto single = tricopterRpmObserverStep(&b, 20000, nullptr);
    EXPECT_EQ(split.mechanicalRpm, single.mechanicalRpm);
    EXPECT_EQ(split.accelerationRpmS, single.accelerationRpmS);
    EXPECT_EQ(a.estimatedRpm, b.estimatedRpm);
    const auto aa = measure(a, 21000, 3, 1500);
    const auto bb = measure(b, 21000, 3, 1500);
    EXPECT_EQ(aa.mechanicalRpm, bb.mechanicalRpm);
    EXPECT_EQ(aa.accelerationRpmS, bb.accelerationRpmS);
}

TEST(TricopterRpmObserver, TracksAnalyticRampOnIrregularObservationIntervals)
{
    auto p = parameters(); p.staleAfterUs = 5000;
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    uint32_t time = 0;
    double squaredSpeed = 0, squaredAcceleration = 0;
    unsigned n = 0;
    for (uint32_t i = 0; i < 1200; ++i) {
        time += 700 + (i % 7) * 100;
        const float rpm = 1000 + 2000 * (time * 1e-6f);
        const auto out = measure(state, time, i, rpm);
        ASSERT_TRUE(out.valid);
        if (time > 300000) {
            squaredSpeed += std::pow(out.mechanicalRpm - rpm, 2);
            squaredAcceleration += std::pow(out.accelerationRpmS - 2000, 2);
            ++n;
        }
    }
    ASSERT_GT(n, 500U);
    RecordProperty("speed_rmse_rpm", std::to_string(std::sqrt(squaredSpeed / n)));
    RecordProperty("acceleration_rmse_rpm_s", std::to_string(std::sqrt(squaredAcceleration / n)));
    EXPECT_LT(std::sqrt(squaredSpeed / n), 0.02);
    EXPECT_LT(std::sqrt(squaredAcceleration / n), 1.0);
}

TEST(TricopterRpmObserver, FiltersNoiseWithoutRawDifferentiation)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    double speedError = 0, accelerationError = 0, rawDerivativeError = 0;
    float previous = 0;
    unsigned n = 0;
    for (uint32_t i = 0; i <= 1200; ++i) {
        const float truth = 2000 + 3000 * (i * 0.001f);
        const float raw = truth + (i % 2 ? 25 : -25);
        const auto out = measure(state, i * 1000, i, raw);
        if (i > 300) {
            speedError += std::pow(out.mechanicalRpm - truth, 2);
            accelerationError += std::pow(out.accelerationRpmS - 3000, 2);
            rawDerivativeError += std::pow((raw - previous) / 0.001f - 3000, 2);
            ++n;
        }
        previous = raw;
    }
    RecordProperty("speed_rmse_rpm", std::to_string(std::sqrt(speedError / n)));
    RecordProperty("acceleration_rmse_rpm_s", std::to_string(std::sqrt(accelerationError / n)));
    RecordProperty("raw_derivative_rmse_rpm_s", std::to_string(std::sqrt(rawDerivativeError / n)));
    EXPECT_LT(std::sqrt(speedError / n), 3.0);
    EXPECT_LT(std::sqrt(accelerationError / n), 100.0);
    EXPECT_LT(accelerationError, rawDerivativeError * 0.0001);
}

TEST(TricopterRpmObserver, EstimatesAccelerationAndDecelerationSigns)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    for (uint32_t i = 0; i <= 800; ++i) {
        const float rpm = i <= 400 ? 1000 + i * 5.0f : 3000 - (i - 400) * 5.0f;
        const auto out = measure(state, i * 1000, i, rpm);
        if (i == 350) {
            EXPECT_NEAR(out.accelerationRpmS, 5000, 2);
        }
        if (i == 750) {
            EXPECT_NEAR(out.accelerationRpmS, -5000, 2);
        }
    }
}

TEST(TricopterRpmObserver, StalePredictionDecaysAndStopsAtHorizon)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    measure(state, 0, 1, 1000);
    const auto live = measure(state, 1000, 2, 1500);
    auto out = tricopterRpmObserverStep(&state, 11000, nullptr);
    EXPECT_TRUE(out.fresh); EXPECT_TRUE(out.valid);
    out = tricopterRpmObserverStep(&state, 21000, nullptr);
    EXPECT_FALSE(out.fresh); EXPECT_TRUE(out.valid);
    EXPECT_LT(std::fabs(out.accelerationRpmS), std::fabs(live.accelerationRpmS));
    for (uint32_t t = 41000; t <= 101000; t += 20000) out = tricopterRpmObserverStep(&state, t, nullptr);
    EXPECT_TRUE(out.valid); EXPECT_EQ(out.accelerationRpmS, 0);
    const float held = out.mechanicalRpm;
    out = tricopterRpmObserverStep(&state, 102000, nullptr);
    EXPECT_FALSE(out.valid); EXPECT_FALSE(out.fresh);
    EXPECT_EQ(out.mechanicalRpm, held); EXPECT_EQ(out.accelerationRpmS, 0);
    EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_HORIZON_EXCEEDED, 0U);
    out = measure(state, 103000, 3, 2500);
    EXPECT_TRUE(out.valid); EXPECT_TRUE(out.acceptedNewObservation);
    EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_REACQUIRED, 0U);
    EXPECT_EQ(out.mechanicalRpm, 2500); EXPECT_EQ(out.accelerationRpmS, 0);
}

TEST(TricopterRpmObserver, BoundsRemainFiniteAndRejectInvalidMeasurementsWithoutAgeRefresh)
{
    auto p = parameters(); p.maxAccelerationRpmS = 100;
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    measure(state, 0, 1, 0);
    for (uint32_t i = 1; i <= 30; ++i) {
        const auto out = measure(state, i * 1000, i + 1, i % 2 ? p.maxRpm : 0);
        EXPECT_GE(out.mechanicalRpm, 0); EXPECT_LE(out.mechanicalRpm, p.maxRpm);
        EXPECT_LE(std::fabs(out.accelerationRpmS), p.maxAccelerationRpmS);
        expectFinite(out);
    }
    for (float raw : {-1.0f, p.maxRpm + 1, std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        const auto bad = observation(31000, 90, raw);
        const auto out = tricopterRpmObserverStep(&state, 31000, &bad);
        EXPECT_FALSE(out.acceptedNewObservation); EXPECT_EQ(out.ageUs, 1000U);
        EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_INVALID_OBSERVATION, 0U);
        expectFinite(out);
    }
    EXPECT_TRUE(measure(state, 32000, 32, 50).acceptedNewObservation);
}

TEST(TricopterRpmObserver, ClockAndSequenceWrapForward)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    const uint32_t start = UINT32_MAX - 1000;
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, start));
    measure(state, start, UINT32_MAX, 1000);
    const auto out = measure(state, 500, 0, 1100);
    EXPECT_TRUE(out.acceptedNewObservation); EXPECT_TRUE(out.valid); EXPECT_EQ(out.ageUs, 0U);
    EXPECT_EQ(out.sequence, 0U); EXPECT_GT(out.accelerationRpmS, 0);
}

TEST(TricopterRpmObserver, LongOutageCannotBecomeFreshAfterFullClockWrap)
{
    auto p = parameters(); p.maxCallGapUs = UINT32_C(0x7fffffff);
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    auto cached = observation(0, 1, 1000);
    tricopterRpmObserverStep(&state, 0, &cached);
    tricopterRpmObserverStep(&state, UINT32_C(0x7fffffff), nullptr);
    tricopterRpmObserverStep(&state, UINT32_C(0xfffffffe), nullptr);
    auto out = tricopterRpmObserverStep(&state, 2, &cached);
    EXPECT_EQ(out.ageUs, UINT32_MAX); EXPECT_FALSE(out.fresh); EXPECT_FALSE(out.valid);
    out = measure(state, 3, 2, 1200);
    EXPECT_TRUE(out.fresh); EXPECT_TRUE(out.valid); EXPECT_EQ(out.accelerationRpmS, 0);
    EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_REACQUIRED, 0U);
}

TEST(TricopterRpmObserver, NewSameTimeObservationCoalescesWithoutDerivative)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    measure(state, 0, 1, 1000);
    const auto out = measure(state, 0, 2, 2000);
    EXPECT_TRUE(out.acceptedNewObservation); EXPECT_TRUE(out.valid);
    EXPECT_EQ(out.mechanicalRpm, 2000); EXPECT_EQ(out.accelerationRpmS, 0);
    EXPECT_NE(out.status & TRICOPTER_RPM_OBSERVER_COALESCED_TIME, 0U);
}

TEST(TricopterRpmObserver, BadClockAndCallGapLatchUntilExplicitReset)
{
    const auto p = parameters();
    for (uint32_t badTime : {999U, 1000U + p.maxCallGapUs + 1, 1000U + UINT32_C(0x80000000)}) {
        tricopterRpmObserverState_t state = {};
        ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 1000));
        measure(state, 1000, 1, 1000);
        const auto out = tricopterRpmObserverStep(&state, badTime, nullptr);
        EXPECT_FALSE(out.valid); expectFinite(out);
        EXPECT_FALSE(measure(state, 1001, 2, 1100).acceptedNewObservation);
        ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 1001));
        EXPECT_TRUE(measure(state, 1001, 2, 1100).valid);
    }
}

TEST(TricopterRpmObserver, SequenceRollbackBackwardObservationAndMetadataMismatchLatch)
{
    const auto p = parameters();
    for (unsigned error = 0; error < 4; ++error) {
        tricopterRpmObserverState_t state = {};
        ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 1000));
        measure(state, 1000, 10, 1000);
        auto bad = observation(2000, 11, 1100);
        if (error == 0) bad.sequence = 9;
        if (error == 1) bad.observedAtUs = 999;
        if (error == 2) { bad.sequence = 10; bad.observedAtUs = 1000; }
        if (error == 3) { bad.sequence = 10; bad.mechanicalRpm = 1000; }
        const auto out = tricopterRpmObserverStep(&state, 2000, &bad);
        EXPECT_FALSE(out.valid); EXPECT_FALSE(out.acceptedNewObservation);
        EXPECT_FALSE(measure(state, 3000, 12, 1200).valid);
        expectFinite(out);
    }
}

TEST(TricopterRpmObserver, FutureAndTooOldObservationsAreRejectedWithoutPoisoningState)
{
    const auto p = parameters();
    tricopterRpmObserverState_t state = {};
    ASSERT_TRUE(tricopterRpmObserverReset(&state, &p, 0));
    measure(state, 0, 1, 1000);
    auto bad = observation(1001, 2, 1100);
    auto out = tricopterRpmObserverStep(&state, 1000, &bad);
    EXPECT_FALSE(out.acceptedNewObservation); EXPECT_TRUE(out.valid); EXPECT_EQ(out.ageUs, 1000U);
    bad = observation(2000 - p.maxPredictionUs - 1, 2, 1100);
    out = tricopterRpmObserverStep(&state, 2000, &bad);
    EXPECT_FALSE(out.acceptedNewObservation); EXPECT_EQ(out.ageUs, 2000U);
    EXPECT_TRUE(measure(state, 3000, 2, 1100).acceptedNewObservation);
}
