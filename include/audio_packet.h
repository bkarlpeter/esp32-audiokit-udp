#pragma once
#include <stdint.h>
#include "config.h"   // for FRAME_SIZE

// ============================================================
// Wire format for every UDP audio packet
//
//   [ AudioPacketHeader (14 bytes) ][ PCM payload (payload_len bytes) ]
//
// All multi-byte fields are little-endian (native ESP32).
// ============================================================

// Magic value in every audio packet — sanity check on receive
#define PACKET_MAGIC_AUDIO  0xA1D10000U

// Maximum PCM payload per packet (must equal FRAME_SIZE)
#define MAX_AUDIO_PAYLOAD   FRAME_SIZE

// Total worst-case UDP datagram size
#define MAX_UDP_PACKET_SIZE (sizeof(AudioPacketHeader) + MAX_AUDIO_PAYLOAD)

#pragma pack(push, 1)
struct AudioPacketHeader {
    uint32_t magic;         // Must equal PACKET_MAGIC_AUDIO
    uint32_t seq;           // Monotonically increasing counter (wraps)
    uint32_t timestamp_ms;  // millis() on master at send time
    uint8_t  volume;        // Master volume 0–100 (synced to slave)
    uint8_t  pad;           // Reserved; always 0
    uint16_t payload_len;   // PCM payload in bytes (<= MAX_AUDIO_PAYLOAD)
};
#pragma pack(pop)
