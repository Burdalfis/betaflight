// Shared wire encoding; independent of Arduino so host tests can check it.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace adxl_logger {

constexpr size_t kFrameSize = 64;
constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kSampleFrame = 1;
constexpr uint8_t kStatusFrame = 2;
constexpr uint8_t kDiagnosticFrame = 3;
constexpr uint32_t kSamplePeriodUs = 1250; // 800 Hz.
constexpr uint32_t kMaximumQualifiedLatencyUs = 625;
constexpr uint32_t kPwmFreshnessUs = 100000;

enum Flag : uint16_t {
    SensorConfigured = 1U << 0,
    QualifiedDrdyTime = 1U << 1,
    FreshPwm = 1U << 2,
    SensorOverrun = 1U << 3,
    ServiceGap = 1U << 4,
    PolledWithoutEdge = 1U << 5,
    UsbSessionStart = 1U << 6,
    FifoCoherent = 1U << 7,
};

enum Fault : uint16_t {
    Ok = 0,
    BadDeviceId = 1,
    ConfigReadbackMismatch = 2,
    NoSampleTimeout = 3,
    DrdyFallback = 4,
    FifoDiscard = 5,
};

struct Fields {
    uint8_t type;
    uint16_t flags;
    uint32_t sequence;
    uint64_t timeUs;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t auxiliary; // Sample: INT_SOURCE. Status: Fault.
    uint32_t readLatencyUs;
    uint32_t pwmWidthUs;
    uint32_t pwmSequence;
    uint64_t pwmRiseTimeUs;
    uint32_t sensorOverruns;
    uint32_t serviceGaps;
    uint32_t queueDroppedFrames;
};

inline void put16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
}

inline void put32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}

inline void put64(uint8_t *out, uint64_t value)
{
    for (unsigned i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}

inline uint32_t crc32(const uint8_t *bytes, size_t count)
{
    uint32_t crc = 0xffffffffU;
    for (size_t i = 0; i < count; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
        }
    }
    return crc ^ 0xffffffffU;
}

inline void encode(const Fields &fields, uint8_t out[kFrameSize])
{
    memcpy(out, "ADXL", 4);
    out[4] = kProtocolVersion;
    out[5] = fields.type;
    put16(out + 6, fields.flags);
    put32(out + 8, fields.sequence);
    put64(out + 12, fields.timeUs);
    put16(out + 20, static_cast<uint16_t>(fields.x));
    put16(out + 22, static_cast<uint16_t>(fields.y));
    put16(out + 24, static_cast<uint16_t>(fields.z));
    put16(out + 26, fields.auxiliary);
    put32(out + 28, fields.readLatencyUs);
    put32(out + 32, fields.pwmWidthUs);
    put32(out + 36, fields.pwmSequence);
    put64(out + 40, fields.pwmRiseTimeUs);
    put32(out + 48, fields.sensorOverruns);
    put32(out + 52, fields.serviceGaps);
    put32(out + 56, fields.queueDroppedFrames);
    put32(out + 60, crc32(out, 60));
}

// Call at least once per 32-bit micros() wrap. The firmware's foreground
// loop calls this continuously, even without a USB host or sensor.
struct MicrosClock {
    uint32_t previous = 0;
    uint64_t high = 0;

    uint64_t update(uint32_t low)
    {
        if (low < previous) high += UINT64_C(1) << 32;
        previous = low;
        return high + low;
    }
};

// Spread a periodic register audit over foreground iterations. Sampling can
// defer a check without skipping that register or moving the next deadline.
// The caller performs at most one hardware read after ready(), then calls
// accept() only for a successful readback. Startup validation is separate.
class PeriodicRegisterScan {
public:
    void restart(uint64_t nowUs, uint64_t intervalUs)
    {
        dueUs_ = nowUs + intervalUs;
        index_ = 0;
        active_ = false;
    }

    bool ready(uint64_t nowUs, bool samplePending)
    {
        if (samplePending) return false;
        if (!active_ && nowUs >= dueUs_) active_ = true;
        return active_;
    }

    uint8_t index() const { return index_; }

    void accept(uint64_t nowUs, uint8_t registerCount, uint64_t intervalUs)
    {
        if (!active_) return;
        if (++index_ >= registerCount) restart(nowUs, intervalUs);
    }

private:
    uint64_t dueUs_ = 0;
    uint8_t index_ = 0;
    bool active_ = false;
};

// Expand a recent ISR timestamp after its snapshot and a current clock read.
// A timestamp older than one micros() wrap cannot be reconstructed.
inline uint64_t expandRecent(uint64_t nowUs, uint32_t capturedUs)
{
    const uint32_t age = static_cast<uint32_t>(nowUs) - capturedUs;
    return age <= nowUs ? nowUs - age : 0;
}

inline bool validPwmWidth(uint32_t widthUs)
{
    return widthUs >= 500 && widthUs <= 2500;
}

inline bool freshPwmAt(bool valid, uint32_t sequence, uint64_t timeUs,
    uint64_t riseUs, uint32_t widthUs)
{
    return valid && sequence && riseUs <= timeUs
        && widthUs <= timeUs - riseUs
        && timeUs - riseUs <= kPwmFreshnessUs;
}

// Active-high ADXL DATA_READY stays asserted until XYZ is read. A fresh GPIO
// edge plus unread data alone cannot authenticate a foreign/noise edge.
inline bool qualifiedTime(bool hasEdge, uint32_t edgeAgeUs,
    bool overrun, bool edgeChangedDuringRead, uint64_t readDurationUs = 0,
    bool pinAsserted = true)
{
    return hasEdge && pinAsserted && edgeAgeUs <= kMaximumQualifiedLatencyUs
        && readDurationUs <= kMaximumQualifiedLatencyUs - edgeAgeUs
        && !overrun && !edgeChangedDuringRead;
}

enum class FifoDecision : uint8_t { Empty, Read, Discard };

inline FifoDecision fifoDecision(uint8_t depth, uint8_t source)
{
    // Reserve one slot: a full/overrun condition during XYZ read could be
    // cleared by that same pop and disappear before a post-read check.
    if (depth >= 31 || (source & 1U)) return FifoDecision::Discard;
    return depth ? FifoDecision::Read : FifoDecision::Empty;
}

inline bool fifoOverrunEvidence(uint8_t depth, uint8_t source)
{
    return depth == 32 || (source & 1U);
}

inline bool fifoPopAccepted(uint8_t depthBefore, uint8_t sourceBefore,
    uint8_t depthAfter, uint8_t sourceAfter, uint64_t observationToReadEndUs)
{
    return fifoDecision(depthBefore, sourceBefore) == FifoDecision::Read
        && fifoDecision(depthAfter, sourceAfter) != FifoDecision::Discard
        && observationToReadEndUs <= kSamplePeriodUs / 2;
}

} // namespace adxl_logger
