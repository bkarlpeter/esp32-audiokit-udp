#pragma once

// ============================================================
//  USER CONFIGURATION – edit these to match your setup
// ============================================================

// WiFi network (both boards must join the same network)
#define WIFI_SSID     "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"

// Static IP addresses – assign these in your router or set
// them directly via WiFi.config() in your environment.
#define MASTER_IP  "192.168.1.100"
#define SLAVE_IP   "192.168.1.101"

// UDP ports
#define AUDIO_PORT  5555   // audio stream
#define CTRL_PORT   5556   // control/volume messages

// ============================================================
//  AUDIO SETTINGS
// ============================================================
#define SAMPLE_RATE       44100   // Hz  (change to 48000 if APLL is unstable)
#define BITS_PER_SAMPLE   16
#define CHANNELS          2       // stereo

// Samples per channel per UDP packet (power-of-two recommended).
// 256 samples/ch @ 44 100 Hz ≈ 5.8 ms per packet.
#define SAMPLES_PER_PACKET  256

// Total int16_t values carried in one audio UDP payload
#define AUDIO_SAMPLES_TOTAL  (SAMPLES_PER_PACKET * CHANNELS)

// I2S DMA configuration
// Each DMA buffer holds one packet's worth of samples.
// Fewer buffers = lower DMA latency; minimum 2 to avoid xruns.
#define I2S_DMA_BUF_LEN    AUDIO_SAMPLES_TOTAL  // samples per DMA buffer
#define I2S_DMA_BUF_COUNT  4                     // number of DMA buffers

// ============================================================
//  JITTER BUFFER  (slave only)
// ============================================================
// Maximum packets the circular buffer can store
#define JITTER_BUF_CAPACITY  16

// How many packets to pre-buffer before starting playback
#define JITTER_MIN_PREBUFFER  2

// Desired steady-state fill level (fine-tunable via serial 'b' command)
#define JITTER_TARGET_LEVEL   4

// If fill level reaches this threshold, flush + resync immediately
// (this is the "kill latency" trigger)
#define JITTER_MAX_LEVEL      8

// ============================================================
//  ES8388 HARDWARE PINS  (AI-Thinker ESP32-A1S AudioKit)
// ============================================================
#define ES8388_I2C_ADDR  0x10
#define PIN_I2C_SDA      33
#define PIN_I2C_SCL      32

// I2S bus
#define PIN_I2S_MCLK  0    // Master clock to ES8388
#define PIN_I2S_BCLK  27   // Bit clock
#define PIN_I2S_LRCK  25   // Word-select / left-right clock
#define PIN_I2S_DOUT  26   // ESP32 → ES8388 DAC data
#define PIN_I2S_DIN   35   // ES8388 ADC data → ESP32

// ============================================================
//  MISC
// ============================================================
#define DEFAULT_VOLUME  75    // 0-100
#define SERIAL_BAUD     115200
#define PKT_MAGIC       0xA1D10FF0UL   // sanity word in every packet header
