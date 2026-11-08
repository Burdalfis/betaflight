#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

#include "gtest/gtest.h"

extern "C" {
#include "sitl_adc.h"
}

static std::array<uint8_t, SITL_ADC_PACKET_SIZE> packet(double time = 1.25,
    uint32_t epoch = 10, uint32_t sequence = 1, uint16_t pb0 = 1490, uint16_t pc0 = 0)
{
    std::array<uint8_t, SITL_ADC_PACKET_SIZE> bytes{};
    const auto u16 = [&](unsigned offset, uint16_t value) {
        bytes[offset] = value; bytes[offset + 1] = value >> 8;
    };
    const auto u32 = [&](unsigned offset, uint32_t value) {
        for (unsigned i = 0; i < 4; ++i) bytes[offset + i] = value >> (8 * i);
    };
    u32(0, SITL_ADC_PACKET_MAGIC); u16(4, 1); u16(6, SITL_ADC_PACKET_SIZE);
    uint64_t timeBits; std::memcpy(&timeBits, &time, sizeof(timeBits));
    for (unsigned i = 0; i < 8; ++i) bytes[8 + i] = timeBits >> (8 * i);
    u32(16, epoch); u32(20, sequence); u16(24, pb0); u16(26, pc0);
    return bytes;
}

TEST(SitlAdc, DecodesExplicitLittleEndianWireAndZeroGround)
{
    const auto bytes = packet(); sitlAdcPacket_t out{};
    EXPECT_EQ(bytes[0], 'T'); EXPECT_EQ(bytes[1], 'D'); EXPECT_EQ(bytes[2], 'A'); EXPECT_EQ(bytes[3], '1');
    EXPECT_EQ(bytes[14], 0xf4); EXPECT_EQ(bytes[15], 0x3f);
    ASSERT_TRUE(sitlAdcDecode(bytes.data(), bytes.size(), &out));
    EXPECT_EQ(out.epoch, 10U); EXPECT_EQ(out.sequence, 1U);
    EXPECT_DOUBLE_EQ(out.simulationSeconds, 1.25); EXPECT_EQ(out.pb0, 1490); EXPECT_EQ(out.pc0, 0);
}

TEST(SitlAdc, RejectsMalformedHeadersAndExactDatagramLengths)
{
    const auto valid = packet(); sitlAdcPacket_t out{};
    for (unsigned offset : {0U, 4U, 6U, 28U}) {
        auto bad = valid; bad[offset] ^= 0x01;
        EXPECT_FALSE(sitlAdcDecode(bad.data(), bad.size(), &out)) << offset;
    }
    EXPECT_FALSE(sitlAdcDecode(nullptr, valid.size(), &out));
    EXPECT_FALSE(sitlAdcDecode(valid.data(), valid.size(), nullptr));
    EXPECT_FALSE(sitlAdcDecode(valid.data(), valid.size() - 1, &out));
    EXPECT_FALSE(sitlAdcDecode(valid.data(), valid.size() + 1, &out));
}

TEST(SitlAdc, RejectsNonfiniteOrNegativeSimulationTimesUnderFastMath)
{
    for (const double time : {-1.0, -0.0, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        const auto bytes = packet(time); sitlAdcPacket_t out{};
        EXPECT_FALSE(sitlAdcDecode(bytes.data(), bytes.size(), &out));
    }
    const auto zero = packet(0); sitlAdcPacket_t out{};
    EXPECT_TRUE(sitlAdcDecode(zero.data(), zero.size(), &out));
}

TEST(SitlAdc, PreservesBothRawChannelsAndRejectsOutOfRangeCounts)
{
    sitlAdcPacket_t out{};
    auto bytes = packet(1.25, 10, 1, 2100, 110);
    ASSERT_TRUE(sitlAdcDecode(bytes.data(), bytes.size(), &out));
    EXPECT_EQ(out.pb0 - out.pc0, 1990);
    bytes = packet(1.25, 10, 1, 0, 4095);
    ASSERT_TRUE(sitlAdcDecode(bytes.data(), bytes.size(), &out));
    EXPECT_EQ(static_cast<int>(out.pb0) - out.pc0, -4095);
    for (const auto bad : {packet(1.25, 10, 1, 4096, 0), packet(1.25, 10, 1, 0, 4096)}) {
        EXPECT_FALSE(sitlAdcDecode(bad.data(), bad.size(), &out));
    }
}

TEST(SitlAdc, AgeIsUnavailableBeforeFirstPairAndAgesTheAcceptedPair)
{
    sitlAdcState_t state{};
    EXPECT_EQ(sitlAdcAgeUs(&state, 123), UINT32_MAX);
    EXPECT_EQ(sitlAdcAgeUs(nullptr, 123), UINT32_MAX);
    const auto bytes = packet();
    ASSERT_TRUE(sitlAdcAccept(&state, bytes.data(), bytes.size(), 1000));
    EXPECT_EQ(sitlAdcAgeUs(&state, 1000), 0U);
    EXPECT_EQ(sitlAdcAgeUs(&state, 21000), 20000U);
    EXPECT_EQ(sitlAdcAgeUs(&state, 21001), 20001U);
}

TEST(SitlAdc, DuplicateOutOfOrderPausedAndInvalidPacketsDoNotRefreshAge)
{
    sitlAdcState_t state{}; const auto first = packet(2, 10, 100);
    ASSERT_TRUE(sitlAdcAccept(&state, first.data(), first.size(), 1000));
    for (auto rejected : {first, packet(3, 10, 99), packet(3, 10, 100),
        packet(2, 10, 101), packet(1, 10, 102), packet(3, 9, 101), packet(3, 10, 101, 4096)}) {
        EXPECT_FALSE(sitlAdcAccept(&state, rejected.data(), rejected.size(), 25000));
        EXPECT_EQ(state.arrivalUs, 1000U); EXPECT_EQ(sitlAdcAgeUs(&state, 25000), 24000U);
        EXPECT_EQ(state.sample.sequence, 100U); EXPECT_EQ(state.sample.pb0, 1490);
    }
}

TEST(SitlAdc, NewerSamplesUpdateTheCoherentPairTogether)
{
    sitlAdcState_t state{}; const auto first = packet();
    ASSERT_TRUE(sitlAdcAccept(&state, first.data(), first.size(), 1000));
    const auto next = packet(1.251, 10, 2, 1936, 10);
    ASSERT_TRUE(sitlAdcAccept(&state, next.data(), next.size(), 2000));
    EXPECT_EQ(state.sample.pb0, 1936); EXPECT_EQ(state.sample.pc0, 10);
    EXPECT_EQ(state.arrivalUs, 2000U); EXPECT_EQ(state.sample.sequence, 2U);
}

TEST(SitlAdc, AdvancingEpochAcceptsWorldTimeResetButRejectsOldEpochReplay)
{
    sitlAdcState_t state{}; auto bytes = packet(500, 20, 1000);
    ASSERT_TRUE(sitlAdcAccept(&state, bytes.data(), bytes.size(), 1000));
    bytes = packet(0, 21, 0, 1281);
    ASSERT_TRUE(sitlAdcAccept(&state, bytes.data(), bytes.size(), 2000));
    bytes = packet(501, 20, 1001, 1936);
    EXPECT_FALSE(sitlAdcAccept(&state, bytes.data(), bytes.size(), 3000));
    EXPECT_EQ(state.sample.pb0, 1281); EXPECT_DOUBLE_EQ(state.sample.simulationSeconds, 0);
    EXPECT_EQ(state.arrivalUs, 2000U);
}

TEST(SitlAdc, SequenceEpochAndObservationClockWrapSafely)
{
    sitlAdcState_t state{}; auto bytes = packet(1, UINT32_MAX, UINT32_MAX);
    ASSERT_TRUE(sitlAdcAccept(&state, bytes.data(), bytes.size(), UINT32_MAX - 99));
    EXPECT_EQ(sitlAdcAgeUs(&state, 50), 150U);
    bytes = packet(2, UINT32_MAX, 0);
    ASSERT_TRUE(sitlAdcAccept(&state, bytes.data(), bytes.size(), 100));
    bytes = packet(0, 0, 0);
    ASSERT_TRUE(sitlAdcAccept(&state, bytes.data(), bytes.size(), 200));
    EXPECT_EQ(state.sample.epoch, 0U); EXPECT_EQ(state.arrivalUs, 200U);
}
