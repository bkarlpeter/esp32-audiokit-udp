/*
 * audio_config.h – Shared constants for esp32_audiokit_udp.ino
 *
 * Edit values here to tune audio quality, network behaviour, level detection,
 * and the jitter buffer without touching the main sketch logic.
 */
#pragma once

// ─── Audio parameters ─────────────────────────────────────────────────────
#define SAMPLE_RATE         44100   // Hz – must match on both boards
#define NUM_CHANNELS        2       // Stereo
#define BITS                16      // Bits per sample
#define BYTES_PER_SAMPLE    (BITS / 8)

// 4 KB per UDP datagram ≈ 23 ms of stereo 44.1 kHz 16-bit PCM.
// Larger chunks → fewer packets, less overhead, smoother flow.
#define CHUNK_BYTES         4096

// ─── Network ──────────────────────────────────────────────────────────────
// Both boards use these same credentials; the master creates the AP,
// the slave connects to it.
#define AP_SSID             "AudioKit-Master"
#define AP_PASS             "audiokit1"
#define AUDIO_UDP_PORT      12345   // Audio stream datagrams
#define STATUS_UDP_PORT     12346   // Out-of-band status messages

// How long (ms) to scan for an existing master before declaring yourself one
#define DISCOVERY_SCAN_MS   400
// Maximum extra wait added to stagger simultaneous boot (derived from MAC LSB)
#define MAC_STAGGER_MS      2000

// ─── Quiet-input detection (Master only) ──────────────────────────────────
// Normalised RMS threshold below which the line-in is considered "too quiet".
// 16-bit full-scale = 32767; 0.002 ≈ -54 dBFS — increase if you want a
// less sensitive warning.
#define QUIET_THRESHOLD     0.002f
// Consecutive chunks below QUIET_THRESHOLD before a warning is emitted
// (~0.9 s at 44100 Hz / 4 KB chunks)
#define QUIET_WARN_CHUNKS   10

// ─── Gain / smoothing ─────────────────────────────────────────────────────
#define GAIN_DEFAULT        1.0f
// Maximum gain change applied per chunk (ramp avoids hiss, pops, pumping)
#define GAIN_STEP_PER_CHUNK 0.02f

// ─── Jitter buffer (Slave only) ───────────────────────────────────────────
// Ring buffer depth – 8 slots × 23 ms ≈ 185 ms of headroom to absorb
// packet-timing variation without causing audible glitches.
#define JITTER_SLOTS        8
// Minimum filled slots before the slave starts playback (pre-buffer prevents
// an underrun right at startup)
#define JITTER_PRE_BUFFER   3
// How long (ms) to wait for a missing packet before skipping it with silence
#define JITTER_SKIP_MS      60
