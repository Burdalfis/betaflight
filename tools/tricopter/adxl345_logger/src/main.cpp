#include <Arduino.h>
#include <SPI.h>
#include <util/atomic.h>

#include "logger_protocol.h"

#ifndef LOGGER_ADXL_INT_MAP
#define LOGGER_ADXL_INT_MAP 0
#endif
static_assert(LOGGER_ADXL_INT_MAP == 0 || LOGGER_ADXL_INT_MAP == 0x80,
    "LOGGER_ADXL_INT_MAP must select DATA_READY on INT1 (0) or INT2 (0x80)");

#ifndef LOGGER_DRDY_PULLUP
#define LOGGER_DRDY_PULLUP 0
#endif
static_assert(LOGGER_DRDY_PULLUP == 0 || LOGGER_DRDY_PULLUP == 1,
    "LOGGER_DRDY_PULLUP must select weak pulldown (0) or weak pullup (1)");

#ifndef LOGGER_STATIC_FIFO
#define LOGGER_STATIC_FIFO 0
#endif
static_assert(LOGGER_STATIC_FIFO == 0 || LOGGER_STATIC_FIFO == 1,
    "LOGGER_STATIC_FIFO must be 0 or 1");

namespace {

using namespace adxl_logger;

constexpr uint8_t kChipSelectPin = 10;
constexpr uint8_t kDataReadyPin = 2;
constexpr uint8_t kPwmMonitorPin = 3;
constexpr uint8_t kDeviceId = 0xe5;
constexpr uint8_t kBwRate = 0x0d; // Normal-power 800 Hz, 400 Hz bandwidth.
constexpr uint8_t kDataFormat = 0x08; // Full-resolution, +/-2 g, 4-wire SPI.
constexpr uint8_t kPowerControl = 0x08;
constexpr bool kStaticFifo = LOGGER_STATIC_FIFO;
constexpr uint8_t kInterruptEnable = kStaticFifo ? 0 : 0x80;
constexpr uint8_t kInterruptMap = LOGGER_ADXL_INT_MAP;
constexpr uint8_t kFifoControl = kStaticFifo ? 0x80 : 0;
constexpr uint32_t kFifoPollIntervalUs = 1000;
constexpr uint8_t kFifoDrainLimit = 4;
constexpr uint32_t kPollingIntervalUs = 2 * kSamplePeriodUs;
constexpr uint32_t kNoSampleTimeoutUs = 100000;
constexpr uint64_t kRetryIntervalUs = 1000000;
constexpr uint64_t kStatusIntervalUs = 1000000;
constexpr uint8_t kQueueCapacity = 16;

enum Register : uint8_t {
    DeviceId = 0x00,
    OffsetX = 0x1e,
    OffsetY = 0x1f,
    OffsetZ = 0x20,
    BwRate = 0x2c,
    PowerControl = 0x2d,
    InterruptEnable = 0x2e,
    InterruptMap = 0x2f,
    InterruptSource = 0x30,
    DataFormat = 0x31,
    DataX0 = 0x32,
    FifoControl = 0x38,
    FifoStatus = 0x39,
};

struct RegisterExpectation {
    uint8_t address;
    uint8_t expected;
};

constexpr RegisterExpectation kConfigurationRegisters[] = {
    {DeviceId, kDeviceId},
    {BwRate, kBwRate},
    {DataFormat, kDataFormat},
    {OffsetX, 0},
    {OffsetY, 0},
    {OffsetZ, 0},
    {PowerControl, kPowerControl},
    {InterruptEnable, kInterruptEnable},
    {InterruptMap, kInterruptMap},
    {FifoControl, kFifoControl},
};
constexpr uint8_t kConfigurationRegisterCount = sizeof(kConfigurationRegisters)
    / sizeof(kConfigurationRegisters[0]);

class InterruptGuard {
public:
    InterruptGuard() : previousMask_(__get_primask()) { __set_primask(1); }
    ~InterruptGuard() { __set_primask(previousMask_); }
private:
    uint32_t previousMask_;
};

volatile uint32_t drdySequence = 0;
volatile uint32_t drdyTimeUs = 0;
volatile bool drdyPending = false;
volatile uint32_t pwmPendingRiseUs = 0;
volatile bool pwmHasRise = false;
volatile uint32_t pwmWidthUs = 0;
volatile uint32_t pwmRiseUs = 0;
volatile uint32_t pwmSequence = 0;
volatile bool pwmValid = false;

MicrosClock clockUs;
PeriodicRegisterScan configurationScan;
uint8_t frameQueue[kQueueCapacity][kFrameSize];
uint8_t queueHead = 0;
uint8_t queueCount = 0;
uint32_t frameSequence = 0;
uint32_t sensorOverruns = 0;
uint32_t serviceGaps = 0;
uint32_t queueDroppedFrames = 0;
uint32_t mappedPwmSequence = 0;
uint64_t mappedPwmRiseUs = 0;
bool usbConnected = false;
bool sensorConfigured = false;
uint16_t sensorFault = BadDeviceId;
uint8_t observedDeviceId = 0;
uint8_t observedBwRate = 0;
uint8_t observedDataFormat = 0;
uint8_t observedInterruptEnable = 0;
uint8_t observedInterruptMap = 0;
bool lastPreXyzPinHigh = false;
bool lastPostXyzPinHigh = false;
bool lastPostSourceValid = false;
uint8_t lastPreXyzSource = 0;
uint8_t lastPostXyzSource = 0;
uint64_t lastSampleReadUs = 0;
uint64_t nextRetryUs = 0;
uint64_t nextStatusUs = 0;
uint64_t lastPollUs = 0;

uint64_t nowUs()
{
    return clockUs.update(micros());
}

void dataReadyInterrupt()
{
    drdyTimeUs = micros();
    ++drdySequence;
    drdyPending = true;
}

void pwmInterrupt()
{
    const uint32_t time = micros();
    if (digitalReadFast(kPwmMonitorPin)) {
        pwmPendingRiseUs = time;
        pwmHasRise = true;
    } else if (pwmHasRise) {
        const uint32_t width = time - pwmPendingRiseUs;
        pwmValid = validPwmWidth(width);
        if (pwmValid) {
            pwmWidthUs = width;
            pwmRiseUs = pwmPendingRiseUs;
            ++pwmSequence;
        }
        pwmHasRise = false;
    }
}

uint8_t readRegister(uint8_t address)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWriteFast(kChipSelectPin, LOW);
    SPI.transfer(address | 0x80);
    const uint8_t value = SPI.transfer(0);
    digitalWriteFast(kChipSelectPin, HIGH);
    SPI.endTransaction();
    return value;
}

void writeRegister(uint8_t address, uint8_t value)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWriteFast(kChipSelectPin, LOW);
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWriteFast(kChipSelectPin, HIGH);
    SPI.endTransaction();
}

void readAxes(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t bytes[6];
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWriteFast(kChipSelectPin, LOW);
    SPI.transfer(DataX0 | 0xc0);
    for (uint8_t &byte : bytes) byte = SPI.transfer(0);
    digitalWriteFast(kChipSelectPin, HIGH);
    SPI.endTransaction();
    *x = static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) | static_cast<uint16_t>(bytes[1]) << 8);
    *y = static_cast<int16_t>(static_cast<uint16_t>(bytes[2]) | static_cast<uint16_t>(bytes[3]) << 8);
    *z = static_cast<int16_t>(static_cast<uint16_t>(bytes[4]) | static_cast<uint16_t>(bytes[5]) << 8);
}

bool checkConfigurationRegister(const RegisterExpectation &check)
{
    const uint8_t value = readRegister(check.address);
    if (check.address == DeviceId) observedDeviceId = value;
    else if (check.address == BwRate) observedBwRate = value;
    else if (check.address == DataFormat) observedDataFormat = value;
    else if (check.address == InterruptEnable) observedInterruptEnable = value;
    else if (check.address == InterruptMap) observedInterruptMap = value;
    if (value != check.expected) {
        sensorFault = check.address == DeviceId ? BadDeviceId : ConfigReadbackMismatch;
        return false;
    }
    return true;
}

bool checkConfiguration()
{
    // Full startup readback; periodic audits below use the same expectations
    // but return to foreground sampling between each individual register.
    for (const RegisterExpectation &check : kConfigurationRegisters) {
        if (!checkConfigurationRegister(check)) return false;
    }
    return true;
}

void invalidateSensor(uint16_t fault, uint64_t time)
{
    sensorConfigured = false;
    sensorFault = fault;
    ++serviceGaps;
    nextRetryUs = time + kRetryIntervalUs;
    nextStatusUs = 0;
}

void serviceConfigurationCheck(uint64_t time)
{
    bool samplePending;
    {
        InterruptGuard guard;
        samplePending = drdyPending;
    }
    if (!configurationScan.ready(time, samplePending)) return;
    if (!checkConfigurationRegister(kConfigurationRegisters[configurationScan.index()])) {
        invalidateSensor(sensorFault, nowUs());
        return;
    }
    configurationScan.accept(nowUs(), kConfigurationRegisterCount, kStatusIntervalUs);
}

bool initializeSensor()
{
    detachInterrupt(digitalPinToInterrupt(kDataReadyPin));
    observedDeviceId = readRegister(DeviceId);
    if (observedDeviceId != kDeviceId) {
        sensorFault = BadDeviceId;
        observedBwRate = readRegister(BwRate);
        observedDataFormat = readRegister(DataFormat);
        return false;
    }
    writeRegister(InterruptEnable, 0);
    writeRegister(PowerControl, 0);
    writeRegister(FifoControl, 0); // Bypass: no invented FIFO timestamps.
    // Neutral hardware offsets; this is not a measured bias calibration.
    writeRegister(OffsetX, 0);
    writeRegister(OffsetY, 0);
    writeRegister(OffsetZ, 0);
    writeRegister(DataFormat, kDataFormat);
    writeRegister(BwRate, kBwRate);
    writeRegister(InterruptMap, kInterruptMap);
    if (!kStaticFifo) {
        attachInterrupt(digitalPinToInterrupt(kDataReadyPin), dataReadyInterrupt, RISING);
    }
    // Wake and settle with interrupt output disabled, discard startup data,
    // then expose only subsequent conversions to the configured INT output.
    // This bounded delay occurs only during initialization/retry.
    writeRegister(PowerControl, kPowerControl);
    delay(5);
    int16_t discardX, discardY, discardZ;
    readAxes(&discardX, &discardY, &discardZ);
    if (kStaticFifo) {
        delayMicroseconds(5);
        writeRegister(FifoControl, kFifoControl); // Start with an empty FIFO.
    }
    {
        InterruptGuard guard;
        drdyPending = false;
    }
    writeRegister(InterruptEnable, kInterruptEnable);
    if (!checkConfiguration()) return false;
    sensorFault = Ok;
    return true;
}

Fields commonFields(uint8_t type, uint64_t time)
{
    Fields fields = {};
    fields.type = type;
    fields.timeUs = time;
    fields.flags = sensorConfigured ? SensorConfigured : 0;
    uint32_t rise = 0;
    bool valid = false;
    {
        InterruptGuard guard;
        fields.pwmWidthUs = pwmWidthUs;
        fields.pwmSequence = pwmSequence;
        rise = pwmRiseUs;
        valid = pwmValid;
    }
    const uint64_t snapshotNow = nowUs();
    if (fields.pwmSequence != mappedPwmSequence) {
        mappedPwmSequence = fields.pwmSequence;
        mappedPwmRiseUs = expandRecent(snapshotNow, rise);
    }
    fields.pwmRiseTimeUs = mappedPwmRiseUs;
    if (freshPwmAt(valid, fields.pwmSequence, time, mappedPwmRiseUs, fields.pwmWidthUs)) {
        fields.flags |= FreshPwm;
    }
    fields.sensorOverruns = sensorOverruns;
    fields.serviceGaps = serviceGaps;
    fields.queueDroppedFrames = queueDroppedFrames;
    return fields;
}

void enqueue(Fields fields)
{
    if (!usbConnected) return;
    fields.sequence = frameSequence++;
    if (queueCount == kQueueCapacity) {
        ++queueDroppedFrames;
        return;
    }
    fields.queueDroppedFrames = queueDroppedFrames;
    encode(fields, frameQueue[(queueHead + queueCount) % kQueueCapacity]);
    ++queueCount;
}

void enqueueDiagnostic(uint64_t time)
{
    Fields fields = {};
    fields.type = kDiagnosticFrame;
    fields.flags = sensorConfigured ? SensorConfigured : 0;
    fields.timeUs = time;
    uint16_t state = 0;
    const uint32_t maskBeforeGuard = __get_primask();
    {
        InterruptGuard guard;
        if (digitalReadFast(kDataReadyPin)) state |= 1U << 0;
        if (lastPreXyzPinHigh) state |= 1U << 1;
        if (lastPostXyzPinHigh) state |= 1U << 2;
        if (drdyPending) state |= 1U << 3;
        if (maskBeforeGuard) state |= 1U << 6;
        if (lastPostSourceValid) state |= 1U << 8;
        fields.readLatencyUs = drdySequence;
        // Diagnostic raw tick, zero-extended, deliberately not expanded.
        fields.pwmRiseTimeUs = drdyTimeUs;
#if defined(KINETISL)
        constexpr uint32_t irqMask = UINT32_C(1) << IRQ_PORTCD;
        if (NVIC_ISER0 & irqMask) state |= 1U << 4;
        if (NVIC_IS_PENDING(IRQ_PORTCD)) state |= 1U << 5;
        state |= 1U << 7;
        fields.pwmWidthUs = CORE_PIN2_CONFIG;
        fields.pwmSequence = PORTD_ISFR;
#endif
    }
    fields.x = state;
    fields.y = observedInterruptEnable;
    fields.z = observedInterruptMap;
    fields.auxiliary = static_cast<uint16_t>(lastPreXyzSource)
        | static_cast<uint16_t>(lastPostXyzSource) << 8;
    fields.sensorOverruns = sensorOverruns;
    fields.serviceGaps = serviceGaps;
    fields.queueDroppedFrames = queueDroppedFrames;
    enqueue(fields);
}

void enqueueStatus(uint64_t time, bool sessionStart = false)
{
    Fields fields = commonFields(kStatusFrame, time);
    fields.x = observedDeviceId;
    fields.y = observedBwRate;
    fields.z = observedDataFormat;
    fields.auxiliary = sensorFault;
    if (sessionStart) fields.flags |= UsbSessionStart;
    enqueue(fields);
    enqueueDiagnostic(nowUs());
    nextStatusUs = time + kStatusIntervalUs;
}

void serviceUsb(uint64_t time)
{
    const bool connected = static_cast<bool>(Serial);
    if (connected != usbConnected) {
        queueDroppedFrames += queueCount;
        queueHead = queueCount = 0;
        usbConnected = connected;
        if (connected) enqueueStatus(time, true);
    }
    if (!usbConnected || !queueCount) return;
    // Capacity checks avoid routine waiting. Teensy core USB routines can
    // still stall after a reservation race or host disconnect. Keep IRQs
    // enabled; late-service, sensor-overrun and queue-loss flags expose the
    // consequence rather than claiming hard nonblocking/lossless sampling.
    if (Serial.availableForWrite() < static_cast<int>(kFrameSize)) return;
    if (Serial.write(frameQueue[queueHead], kFrameSize) == kFrameSize) {
        queueHead = (queueHead + 1) % kQueueCapacity;
        --queueCount;
    }
}

void discardFifo(uint8_t depth, uint8_t source)
{
    if (fifoOverrunEvidence(depth, source)) ++sensorOverruns;
    ++serviceGaps;
    sensorFault = FifoDiscard;
    writeRegister(FifoControl, 0);
    int16_t discardX, discardY, discardZ;
    readAxes(&discardX, &discardY, &discardZ);
    delayMicroseconds(5);
    writeRegister(FifoControl, kFifoControl);
    if (usbConnected) enqueueStatus(nowUs());
}

void serviceFifoSample(uint64_t time)
{
    if (time - lastPollUs < kFifoPollIntervalUs) return;
    lastPollUs = time;
    for (uint8_t drained = 0; drained < kFifoDrainLimit; ++drained) {
        const uint64_t observationStart = nowUs();
        const uint8_t depth = readRegister(FifoStatus) & 0x3f;
        const uint8_t source = readRegister(InterruptSource);
        const FifoDecision decision = fifoDecision(depth, source);
        if (decision == FifoDecision::Discard) {
            lastPreXyzSource = source;
            lastPostSourceValid = false;
            discardFifo(depth, source);
            return;
        }
        if (decision == FifoDecision::Empty) return;
        int16_t x, y, z;
        const uint64_t readStart = nowUs();
        lastPreXyzPinHigh = digitalReadFast(kDataReadyPin);
        readAxes(&x, &y, &z);
        lastPostXyzPinHigh = digitalReadFast(kDataReadyPin);
        const uint64_t readEnd = nowUs();
        // ADXL345 requires >=5 us after popping XYZ before another FIFO or
        // status access. This also separates consecutive XYZ pop bursts.
        delayMicroseconds(5);
        const uint8_t depthAfter = readRegister(FifoStatus) & 0x3f;
        const uint8_t sourceAfter = readRegister(InterruptSource);
        lastPreXyzSource = source;
        lastPostXyzSource = sourceAfter;
        lastPostSourceValid = true;
        if (!fifoPopAccepted(depth, source, depthAfter, sourceAfter,
            readEnd - observationStart)) {
            const bool overrun = fifoOverrunEvidence(depth, source)
                || fifoOverrunEvidence(depthAfter, sourceAfter);
            discardFifo(overrun ? 32 : depthAfter, sourceAfter);
            return;
        }
        lastSampleReadUs = readEnd;
        sensorFault = Ok;
        Fields fields = commonFields(kSampleFrame, readStart);
        fields.flags |= FifoCoherent;
        fields.x = x;
        fields.y = y;
        fields.z = z;
        fields.auxiliary = static_cast<uint16_t>(depth) << 8 | source;
        // timeUs is this SPI pop's read-start time. Sample age is unknown;
        // never substitute it for a DRDY/conversion timestamp.
        enqueue(fields);
    }
}

void serviceSample(uint64_t time)
{
    if (kStaticFifo) {
        serviceFifoSample(time);
        return;
    }
    bool hasEdge;
    uint32_t edgeTime, edgeSequence;
    {
        InterruptGuard guard;
        hasEdge = drdyPending;
        edgeTime = drdyTimeUs;
        edgeSequence = drdySequence;
        drdyPending = false;
    }
    if (!hasEdge && time - lastPollUs < kPollingIntervalUs) return;
    lastPollUs = time;
    const uint64_t readStart = nowUs();
    const uint32_t edgeAge = hasEdge ? static_cast<uint32_t>(readStart) - edgeTime : 0;
    const uint8_t source = readRegister(InterruptSource);
    if (!(source & 0x80)) {
        if (hasEdge) ++serviceGaps;
        return;
    }
    int16_t x, y, z;
    const bool pinBeforeXyz = digitalReadFast(kDataReadyPin);
    readAxes(&x, &y, &z);
    const bool pinAfterXyz = digitalReadFast(kDataReadyPin);
    uint32_t sequenceAfterRead;
    {
        InterruptGuard guard;
        sequenceAfterRead = drdySequence;
    }
    const uint64_t readEnd = nowUs();
    lastPreXyzPinHigh = pinBeforeXyz;
    lastPostXyzPinHigh = pinAfterXyz;
    lastPreXyzSource = source;
    lastPostSourceValid = !hasEdge;
    // Only inspect post-read source in already-unqualified fallback. Do not
    // add an SPI transaction to the healthy DRDY sampling path.
    lastPostXyzSource = lastPostSourceValid ? readRegister(InterruptSource) : 0;
    const bool overrun = source & 1;
    const bool edgeChanged = sequenceAfterRead != edgeSequence;
    const bool qualified = qualifiedTime(hasEdge, edgeAge, overrun,
        edgeChanged, readEnd - readStart, pinBeforeXyz);
    const bool gap = !qualified || (readStart - lastSampleReadUs > 2 * kSamplePeriodUs);
    if (overrun) ++sensorOverruns; // Each flag means at least one overwrite.
    if (gap) ++serviceGaps; // Service events, not exact missed-sample counts.
    lastSampleReadUs = readStart;
    sensorFault = hasEdge ? Ok : DrdyFallback;
    Fields fields = commonFields(kSampleFrame,
        qualified ? expandRecent(readStart, edgeTime) : readStart);
    fields.x = x;
    fields.y = y;
    fields.z = z;
    fields.auxiliary = source;
    fields.readLatencyUs = edgeAge;
    if (qualified) fields.flags |= QualifiedDrdyTime;
    if (overrun) fields.flags |= SensorOverrun;
    if (gap) fields.flags |= ServiceGap;
    if (!hasEdge) fields.flags |= PolledWithoutEdge;
    enqueue(fields);
}

} // namespace

void setup()
{
    // Teensy initializes USB. Serial.begin() on LC can wait for a host.
    pinMode(kChipSelectPin, OUTPUT);
    digitalWriteFast(kChipSelectPin, HIGH);
    pinMode(kDataReadyPin, LOGGER_DRDY_PULLUP ? INPUT_PULLUP : INPUT_PULLDOWN);
    pinMode(kPwmMonitorPin, INPUT_PULLDOWN);
    SPI.begin();
    attachInterrupt(digitalPinToInterrupt(kPwmMonitorPin), pwmInterrupt, CHANGE);
    // Pin 13 is both SCK and LED_BUILTIN; do not toggle an LED here.
    nextRetryUs = nowUs();
}

void loop()
{
    uint64_t time = nowUs();
    serviceUsb(time);
    time = nowUs();
    if (!sensorConfigured && time >= nextRetryUs) {
        sensorConfigured = initializeSensor();
        time = nowUs();
        nextRetryUs = time + kRetryIntervalUs;
        configurationScan.restart(time, kStatusIntervalUs);
        lastSampleReadUs = lastPollUs = time;
        if (!sensorConfigured) ++serviceGaps;
        nextStatusUs = 0;
    }
    if (sensorConfigured) {
        serviceSample(time);
        time = nowUs();
        if (time - lastSampleReadUs > kNoSampleTimeoutUs) {
            invalidateSensor(NoSampleTimeout, time);
        } else serviceConfigurationCheck(time);
    }
    time = nowUs();
    if (usbConnected && time >= nextStatusUs) enqueueStatus(time);
}
