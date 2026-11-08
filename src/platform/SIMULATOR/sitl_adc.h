/* Versioned virtual PB0/PC0 acquisition for the SITL platform only. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SITL_ADC_PORT 9005
#define SITL_ADC_PACKET_SIZE 32U
#define SITL_ADC_PACKET_MAGIC UINT32_C(0x31414454) /* TDA1, little endian */
#define SITL_ADC_PACKET_VERSION 1U

/* Wire layout: <IHHdIIHHI, exactly 32 bytes. Epoch advances when the physics
 * clock restarts; sequence and simulation time advance within an epoch. */
typedef struct sitlAdcPacket_s {
    double simulationSeconds;
    uint32_t epoch;
    uint32_t sequence;
    uint16_t pb0;
    uint16_t pc0;
} sitlAdcPacket_t;

typedef struct sitlAdcState_s {
    sitlAdcPacket_t sample;
    uint32_t arrivalUs;
    bool seen;
} sitlAdcState_t;

static inline uint16_t sitlAdcReadU16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8);
}

static inline uint32_t sitlAdcReadU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline bool sitlAdcDecode(const uint8_t *data, size_t size, sitlAdcPacket_t *packet)
{
    if (!data || !packet || size != SITL_ADC_PACKET_SIZE
        || sitlAdcReadU32(data) != SITL_ADC_PACKET_MAGIC
        || sitlAdcReadU16(data + 4) != SITL_ADC_PACKET_VERSION
        || sitlAdcReadU16(data + 6) != SITL_ADC_PACKET_SIZE
        || sitlAdcReadU32(data + 28) != 0) return false;

    const uint64_t bits = (uint64_t)sitlAdcReadU32(data + 8)
        | ((uint64_t)sitlAdcReadU32(data + 12) << 32);
    /* Bit checks remain valid with the SITL -Ofast compiler flags. */
    if ((bits & UINT64_C(0x8000000000000000))
        || (bits & UINT64_C(0x7ff0000000000000)) == UINT64_C(0x7ff0000000000000)) return false;
    if (sitlAdcReadU16(data + 24) > 4095 || sitlAdcReadU16(data + 26) > 4095) return false;

    memcpy(&packet->simulationSeconds, &bits, sizeof(bits));
    packet->epoch = sitlAdcReadU32(data + 16);
    packet->sequence = sitlAdcReadU32(data + 20);
    packet->pb0 = sitlAdcReadU16(data + 24);
    packet->pc0 = sitlAdcReadU16(data + 26);
    return true;
}

static inline bool sitlAdcSerialNewer(uint32_t next, uint32_t previous)
{
    const uint32_t delta = next - previous;
    return delta != 0 && delta < UINT32_C(0x80000000);
}

static inline bool sitlAdcAccept(sitlAdcState_t *state, const uint8_t *data, size_t size, uint32_t arrivalUs)
{
    sitlAdcPacket_t packet;
    if (!state || !sitlAdcDecode(data, size, &packet)) return false;
    if (state->seen) {
        if (packet.epoch == state->sample.epoch) {
            if (!sitlAdcSerialNewer(packet.sequence, state->sample.sequence)
                || packet.simulationSeconds <= state->sample.simulationSeconds) return false;
        } else if (!sitlAdcSerialNewer(packet.epoch, state->sample.epoch)) {
            return false;
        }
    }
    state->sample = packet;
    state->arrivalUs = arrivalUs;
    state->seen = true;
    return true;
}

static inline uint32_t sitlAdcAgeUs(const sitlAdcState_t *state, uint32_t now)
{
    return state && state->seen ? (uint32_t)(now - state->arrivalUs) : UINT32_MAX;
}
