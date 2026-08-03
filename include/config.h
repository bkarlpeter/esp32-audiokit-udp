#pragma once

// ============================================================
// WiFi credentials — edit before flashing
// ============================================================
#define WIFI_SSID       "your_ssid"
#define WIFI_PASSWORD   "your_password"

// Static IP of the slave device.  The master sends audio here.
// Find it by opening the slave serial monitor after first boot.
#define SLAVE_IP        "192.168.1.101"

// UDP port used for the audio stream
#define AUDIO_UDP_PORT  5004

// ============================================================
// Audio parameters
// ============================================================
#define SAMPLE_RATE           44100   // Hz
#define BITS_PER_SAMPLE       16      // Bits (PCM signed)
#define AUDIO_CHANNELS        2       // 1 = mono, 2 = stereo

// Number of audio frames (samples per channel) per UDP packet.
// 256 frames × 2 ch × 2 bytes = 1 024 bytes ≈ 5.8 ms of audio.
// Increase to 512 for more stable (but higher-latency) streaming.
#define SAMPLES_PER_PACKET    256

// ── Derived — do not edit ─────────────────────────────────────
#define BYTES_PER_SAMPLE      (BITS_PER_SAMPLE / 8)
#define FRAME_SIZE            (SAMPLES_PER_PACKET * AUDIO_CHANNELS * BYTES_PER_SAMPLE)
// FRAME_SIZE = 256 * 2 * 2 = 1 024 bytes

// I2S DMA buffer — keep in sync with SAMPLES_PER_PACKET
#define DMA_BUF_LEN           SAMPLES_PER_PACKET
#define DMA_BUF_COUNT         8

// ============================================================
// Jitter buffer (slave)
// All three are adjustable at runtime via serial commands.
// ============================================================
// Packets to accumulate before starting playback.
// Lower = less latency, more susceptible to network jitter.
#define JITTER_TARGET_DEFAULT  5

// Latency-kill threshold.  When the queue exceeds this many
// packets, the oldest are dropped until depth == target.
// This bounds worst-case latency to (JITTER_MAX * 5.8 ms).
#define JITTER_MAX_DEFAULT     12

// Hard capacity of the FreeRTOS queue (absolute maximum).
#define JITTER_QUEUE_CAPACITY  20

// ============================================================
// Codec I/O selection
// ============================================================
// ADC input used by the master for line-in capture
#define MASTER_ADC_INPUT   AUDIO_HAL_ADC_INPUT_LINE2

// DAC output used by the slave for line-out playback
#define SLAVE_DAC_OUTPUT   AUDIO_HAL_DAC_OUTPUT_LINE1

// ============================================================
// Default volume (0–100)
// ============================================================
#define DEFAULT_VOLUME  70

// ============================================================
// Serial
// ============================================================
#define SERIAL_BAUD  115200
