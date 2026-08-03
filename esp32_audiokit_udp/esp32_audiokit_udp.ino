// ================================================================
//  esp32_audiokit_udp.ino
//
//  Low-latency PCM audio streaming between two ESP32 AudioKit boards
//  using Phil Schatzmann's AudioTools / ESP32-AudioKit ecosystem.
//
//  MASTER role  (IS_MASTER = 1)
//    – Starts a Wi-Fi Access Point ("AudioKitLink")
//    – Reads 16-bit PCM from the codec line-in via I2S
//    – Applies a configurable input gain
//    – Measures a short-term RMS level
//    – Wraps PCM in a small header and broadcasts over UDP
//
//  SLAVE role   (IS_MASTER = 0)
//    – Connects to the master's Access Point as a Wi-Fi station
//    – Receives UDP packets and stores them in a ring-based jitter buffer
//    – Drains the jitter buffer to the codec line-out via I2S
//    – Applies adaptive gain so the slave loudness tracks the master
//
//  Both roles accept serial tuning commands at runtime (115 200 baud).
//  Type  ?  to see the command list.
//
//  Required libraries (install via Arduino Library Manager):
//    • arduino-audio-tools  (pschatzmann/arduino-audio-tools)
//    • arduino-esp32-audiokit (pschatzmann/arduino-esp32-audiokit)
//
//  Tested on AI Thinker AudioKit (ES8388 codec, ESP32).
// ================================================================

// ── Role selection ──────────────────────────────────────────────
// Set IS_MASTER to 1 on the sender board, 0 on the receiver board.
#define IS_MASTER  1

// ── Audio parameters ────────────────────────────────────────────
#define SAMPLE_RATE      22050   // Hz – 22 kHz mono keeps bandwidth low
#define CHANNELS         1       // 1 = mono
#define BITS_PER_SAMPLE  16      // 16-bit signed PCM

// ── Network / Wi-Fi ─────────────────────────────────────────────
#define AP_SSID  "AudioKitLink"
#define AP_PASS  "audiokit1"     // must be ≥ 8 chars for WPA2
#define UDP_PORT 5000
// The ESP32 SoftAP always uses 192.168.4.1 as its own address and
// hands out addresses in the 192.168.4.x subnet.  We broadcast to
// the subnet so no prior knowledge of the slave's IP is needed.
#define BCAST_IP "192.168.4.255"

// ── Transport tuning ────────────────────────────────────────────
// Bytes of PCM carried in one UDP datagram.
// 22050 Hz × 1 ch × 2 bytes = 44 100 B/s → 882 B ≈ 20 ms per packet.
#define PCM_BYTES_PER_PACKET  882

// Jitter-buffer depth: how many PCM packets are held before playback
// starts and as a ceiling.  Each slot ≈ 20 ms → 4 slots ≈ 80 ms.
#define JITTER_SLOTS  4

// ── Gain defaults ───────────────────────────────────────────────
#define DEFAULT_GAIN      1.0f   // unity gain on both sides at startup
// RMS averaging window for loudness measurement (≈ 100 ms at 22 050 Hz)
#define RMS_WINDOW        2205

// ── Libraries ───────────────────────────────────────────────────
#include "AudioKitStream.h"  // ESP32-AudioKit board support
#include "AudioTools.h"      // pschatzmann AudioTools stream API
#include <WiFi.h>
#include <WiFiUdp.h>

// ── Packet header ────────────────────────────────────────────────
// Prepended to every UDP datagram so the slave can detect gaps,
// synchronise sample counts, and track the master's current gain/RMS.
struct __attribute__((packed)) PktHeader {
    uint32_t seq;           // monotonically increasing sequence number
    uint32_t sampleIndex;   // cumulative sample count at this packet
    float    masterGain;    // master input gain at the time of capture
    float    masterRMS;     // short-term normalised RMS (0 … 1)
    uint16_t payloadBytes;  // number of PCM bytes that follow
};

static const size_t HDR_SIZE = sizeof(PktHeader);
static const size_t MAX_PKT  = HDR_SIZE + PCM_BYTES_PER_PACKET;

// ── AudioKit stream (shared by both roles) ───────────────────────
AudioKitStream kit;

// ── UDP socket (shared) ──────────────────────────────────────────
WiFiUDP udp;

// ── Runtime-tunable parameters (both roles read these) ──────────
float masterGain      = DEFAULT_GAIN;  // master input gain
float slaveGain       = DEFAULT_GAIN;  // slave output base gain
int   targetLatencyMs = 40;            // informational / future use
bool  normEnabled     = true;          // adaptive loudness on slave

// ════════════════════════════════════════════════════════════════
//  Shared helpers
// ════════════════════════════════════════════════════════════════

// Apply a scalar gain to signed 16-bit PCM samples in-place.
// Hard-clips to the int16 range to prevent wrap-around distortion.
static void applyGain(int16_t *samples, int count, float gain) {
    if (fabsf(gain - 1.0f) < 0.001f) return;   // skip unity gain
    for (int i = 0; i < count; i++) {
        float v = samples[i] * gain;
        if      (v >  32767.0f) v =  32767.0f;
        else if (v < -32768.0f) v = -32768.0f;
        samples[i] = (int16_t)v;
    }
}

// ── Serial command parser ────────────────────────────────────────

static void printHelp() {
    Serial.println(F("─── Serial tuning commands ───────────────────────"));
    Serial.println(F("  g <val>    input/output gain  (e.g.  g 1.5 )"));
    Serial.println(F("  l <ms>     latency target ms  (e.g.  l 40  )"));
    Serial.println(F("  n on|off   loudness sync       (e.g.  n on  )"));
    Serial.println(F("  ?          show this help"));
    Serial.println(F("──────────────────────────────────────────────────"));
}

static void handleSerial(const String &line) {
    if (line.length() < 1) return;
    char   cmd = line.charAt(0);
    String arg = line.substring(2);
    arg.trim();

    switch (cmd) {
        case 'g':
            masterGain = arg.toFloat();
            slaveGain  = masterGain;
            Serial.printf("[tune] gain → %.3f\n", masterGain);
            break;
        case 'l':
            targetLatencyMs = arg.toInt();
            Serial.printf("[tune] latency target → %d ms\n", targetLatencyMs);
            break;
        case 'n':
            normEnabled = (arg == "on");
            Serial.printf("[tune] normalisation → %s\n", normEnabled ? "on" : "off");
            break;
        case '?':
        default:
            printHelp();
            break;
    }
}

// ════════════════════════════════════════════════════════════════
//  MASTER – Wi-Fi AP + UDP sender
// ════════════════════════════════════════════════════════════════
#if IS_MASTER

static uint8_t  txBuf[MAX_PKT];    // reused every packet
static uint32_t txSeq       = 0;   // sequence counter
static uint32_t txSampleIdx = 0;   // cumulative sample counter

// Rolling RMS accumulators
static double   rmsAccum    = 0.0;
static int      rmsCount    = 0;
static float    currentRMS  = 0.0f;

// Set up ESP32 as a Wi-Fi Access Point and open a UDP socket.
static void setupMasterWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(200);
    Serial.printf("[net] AP  SSID: %s   IP: %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
    udp.begin(UDP_PORT);
}

// Called every loop iteration on the master.
// Flow: I2S line-in → gain → RMS measure → UDP broadcast
static void masterLoop() {
    // PCM data sits immediately after the header in txBuf.
    uint8_t  *payload = txBuf + HDR_SIZE;
    int16_t  *pcm     = (int16_t *)payload;

    // Read one packet-worth of PCM from the codec via I2S.
    // readBytes is non-blocking – if no data is ready it returns 0.
    int bytesRead = kit.readBytes(payload, PCM_BYTES_PER_PACKET);
    if (bytesRead <= 0) return;

    int samplesRead = bytesRead / sizeof(int16_t);

    // Apply configured input gain.
    applyGain(pcm, samplesRead, masterGain);

    // Accumulate RMS over RMS_WINDOW samples.
    for (int i = 0; i < samplesRead; i++) {
        double s = pcm[i] / 32768.0;
        rmsAccum += s * s;
        rmsCount++;
    }
    if (rmsCount >= RMS_WINDOW) {
        currentRMS = (float)sqrt(rmsAccum / rmsCount);
        rmsAccum   = 0.0;
        rmsCount   = 0;
    }

    // Build packet header.
    PktHeader *hdr  = (PktHeader *)txBuf;
    hdr->seq          = txSeq++;
    hdr->sampleIndex  = txSampleIdx;
    hdr->masterGain   = masterGain;
    hdr->masterRMS    = currentRMS;
    hdr->payloadBytes = (uint16_t)bytesRead;
    txSampleIdx += (uint32_t)samplesRead;

    // Broadcast to all devices on the AP subnet.
    udp.beginPacket(BCAST_IP, UDP_PORT);
    udp.write(txBuf, HDR_SIZE + (size_t)bytesRead);
    udp.endPacket();
}

#else  // ── SLAVE ──────────────────────────────────────────────────

// ════════════════════════════════════════════════════════════════
//  SLAVE – Wi-Fi STA + jitter buffer + UDP receiver
// ════════════════════════════════════════════════════════════════

// Jitter buffer: a ring of fixed-size PCM slots.
struct JitterSlot {
    uint8_t  data[PCM_BYTES_PER_PACKET];
    uint16_t len;    // actual bytes stored in this slot
    bool     valid;  // true when this slot contains unplayed audio
};

static JitterSlot jBuf[JITTER_SLOTS];
static int  jWr    = 0;      // next slot to write into
static int  jRd    = 0;      // next slot to play from
static int  jFill  = 0;      // number of currently filled slots
static bool jReady = false;  // turns true once initial fill threshold is met

static uint8_t  rxBuf[MAX_PKT];

// Adaptive output gain for loudness matching
static float  adaptiveGain  = DEFAULT_GAIN;
static bool   firstPkt      = true;
static uint32_t lastSeq     = 0;

// Connect to the master's Access Point and open the UDP listen socket.
static void setupSlaveWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASS);
    Serial.print("[net] Connecting to " AP_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[net] Connected  IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[net] ERROR – could not connect to master AP.");
    }
    udp.begin(UDP_PORT);
}

// Called every loop iteration on the slave.
// Flow: UDP receive → jitter buffer → adaptive gain → I2S line-out
static void slaveLoop() {
    // ── 1. Pull any arriving UDP datagram into the jitter buffer ─

    int pktSize = udp.parsePacket();
    if (pktSize >= (int)(HDR_SIZE + 2)) {
        int n = udp.read(rxBuf, sizeof(rxBuf));
        if (n >= (int)HDR_SIZE) {
            PktHeader *hdr = (PktHeader *)rxBuf;

            // Log packet loss for debugging.
            if (!firstPkt && hdr->seq != lastSeq + 1) {
                Serial.printf("[rx] gap: %u packet(s) lost\n",
                              (unsigned)(hdr->seq - lastSeq - 1));
            }
            lastSeq  = hdr->seq;
            firstPkt = false;

            // ── Loudness sync ─────────────────────────────────────
            // The header carries the master's current gain and RMS.
            // When normalisation is on we derive a target output gain
            // that keeps the slave's perceived loudness aligned with
            // the master, then smooth it with a simple one-pole filter
            // to avoid rapid pumping artefacts.
            if (normEnabled && hdr->masterRMS > 0.001f) {
                // 0.3 is the normalised-RMS target (≈ −10 dBFS).
                float targetGain = (0.3f / hdr->masterRMS) * hdr->masterGain;
                // Smooth with α ≈ 0.05  (≈ 20-packet attack/release)
                adaptiveGain = adaptiveGain * 0.95f + targetGain * 0.05f;
                // Hard limits prevent runaway amplification or silence.
                if (adaptiveGain < 0.1f) adaptiveGain = 0.1f;
                if (adaptiveGain > 8.0f) adaptiveGain = 8.0f;
            } else {
                adaptiveGain = slaveGain;
            }

            // ── Write into jitter buffer ──────────────────────────
            // If the buffer is full we overwrite the oldest slot to
            // prevent unbounded delay from accumulating.
            if (jFill >= JITTER_SLOTS) {
                // Advance read pointer to discard oldest entry.
                jRd   = (jRd + 1) % JITTER_SLOTS;
                jFill--;
            }
            uint16_t payBytes = hdr->payloadBytes;
            if (payBytes > PCM_BYTES_PER_PACKET)
                payBytes = PCM_BYTES_PER_PACKET;
            memcpy(jBuf[jWr].data, rxBuf + HDR_SIZE, payBytes);
            jBuf[jWr].len   = payBytes;
            jBuf[jWr].valid = true;
            jWr = (jWr + 1) % JITTER_SLOTS;
            jFill++;

            // Start draining only once the pre-roll is reached
            // (half the buffer depth = lower bound on start latency).
            if (!jReady && jFill >= JITTER_SLOTS / 2)
                jReady = true;
        }
    }

    // ── 2. Drain one slot from the jitter buffer to the codec ────

    if (jReady && jFill > 0 && jBuf[jRd].valid) {
        JitterSlot *slot  = &jBuf[jRd];
        int16_t    *pcm   = (int16_t *)slot->data;
        int         count = slot->len / sizeof(int16_t);

        // Apply adaptive gain for loudness matching.
        applyGain(pcm, count, adaptiveGain);

        // Write PCM to the codec line-out via I2S.
        kit.write(slot->data, slot->len);

        slot->valid = false;
        jRd   = (jRd + 1) % JITTER_SLOTS;
        jFill--;
    }
}

#endif  // IS_MASTER / SLAVE

// ════════════════════════════════════════════════════════════════
//  Arduino setup() – runs once at boot
// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("=== ESP32 AudioKit UDP  ==="));

    // ── Configure and start the AudioKit stream ───────────────────
    // RX_MODE = input (master reads from line-in).
    // TX_MODE = output (slave writes to line-out).
    auto cfg = kit.defaultConfig(IS_MASTER ? RX_MODE : TX_MODE);
    cfg.sample_rate      = SAMPLE_RATE;
    cfg.channels         = CHANNELS;
    cfg.bits_per_sample  = BITS_PER_SAMPLE;

#if IS_MASTER
    // Select the line-in socket as the ADC input source.
    cfg.input_device = AUDIO_HAL_ADC_INPUT_LINE2;
#else
    // Select the line-out socket as the DAC output.
    cfg.output_device = AUDIO_HAL_DAC_OUTPUT_LINE1;
#endif

    kit.begin(cfg);
    Serial.printf("[audio] %d Hz  %d ch  %d bit\n",
                  SAMPLE_RATE, CHANNELS, BITS_PER_SAMPLE);

    // ── Network setup ─────────────────────────────────────────────
#if IS_MASTER
    setupMasterWiFi();
    Serial.println(F("[role] MASTER  –  line-in → UDP broadcast"));
#else
    setupSlaveWiFi();
    Serial.println(F("[role] SLAVE   –  UDP → line-out"));
#endif

    printHelp();
}

// ════════════════════════════════════════════════════════════════
//  Arduino loop() – runs continuously
// ════════════════════════════════════════════════════════════════
void loop() {
    // Process any pending serial tuning command first.
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        handleSerial(line);
    }

    // Dispatch to the appropriate role handler.
#if IS_MASTER
    masterLoop();
#else
    slaveLoop();
#endif
}
