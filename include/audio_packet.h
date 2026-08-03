#pragma once
#include <stdint.h>
#include "config.h"

// ── Packet type tag ──────────────────────────────────────────────────────────
enum PktType : uint8_t {
    PKT_AUDIO = 0x01,   // raw PCM audio data
    PKT_CTRL  = 0x02,   // control / configuration message
};

// ── Control command codes ────────────────────────────────────────────────────
enum CtrlCmd : uint8_t {
    CMD_VOLUME  = 0x01,  // value = 0-100 (volume percent)
    CMD_FLUSH   = 0x02,  // flush slave jitter buffer (value unused)
    CMD_START   = 0x03,  // slave: begin playback     (value unused)
    CMD_STOP    = 0x04,  // slave: halt playback      (value unused)
    CMD_BUFSIZE = 0x05,  // slave: set target buffer packets (value = 1-14)
};

// ── Common header (15 bytes, packed) ─────────────────────────────────────────
struct __attribute__((packed)) PktHeader {
    uint32_t magic;        // PKT_MAGIC
    uint8_t  type;         // PktType
    uint32_t seq;          // monotonically increasing sequence number
    uint32_t timestamp_us; // micros() at send time (wraps ~71 min, used for
                           // future RTT / drift detection)
    uint16_t payload_len;  // bytes following this header
};

static const size_t PKT_HEADER_SIZE = sizeof(PktHeader);  // 15

// ── Audio packet ─────────────────────────────────────────────────────────────
// Payload: stereo interleaved PCM  L,R,L,R,...  (int16, little-endian)
struct __attribute__((packed)) AudioPkt {
    PktHeader hdr;
    int16_t   samples[AUDIO_SAMPLES_TOTAL];  // SAMPLES_PER_PACKET*CHANNELS
};

static const size_t AUDIO_PKT_SIZE = sizeof(AudioPkt);
// ≈ 15 + 256*2*2 = 15 + 1024 = 1039 bytes — fits in one Ethernet MTU

// ── Control packet ───────────────────────────────────────────────────────────
struct __attribute__((packed)) CtrlPkt {
    PktHeader hdr;
    uint8_t   cmd;    // CtrlCmd
    uint8_t   value;  // command-specific value
    uint32_t  param;  // optional extra parameter (reserved)
};

static const size_t CTRL_PKT_SIZE = sizeof(CtrlPkt);  // 21
