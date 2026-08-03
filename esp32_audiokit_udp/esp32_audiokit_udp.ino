/*
 * esp32_audiokit_udp.ino
 *
 * Single-firmware sketch for two ESP32 Audiokit boards.
 * Flash the same binary to both boards; they negotiate roles automatically.
 *
 * ── Audio flow ────────────────────────────────────────────────────────────
 *   Master  Line-In ──[I2S]──► RMS check ──► smooth gain ──► 4 KB PCM chunk ──► UDP ──►┐
 *   Slave   ◄── UDP ◄── jitter buffer ◄── smooth gain ◄──[I2S]◄── Line-Out           ◄─┘
 *
 * ── Role negotiation ──────────────────────────────────────────────────────
 *   On boot both boards scan for the "AudioKit-Master" Wi-Fi AP.
 *   • AP found   → become Slave, connect and start receiving audio.
 *   • AP missing → add a short MAC-derived stagger delay, scan once more.
 *     Still missing → become Master, create the AP and start streaming.
 *   The stagger prevents both boards electing themselves master when powered
 *   simultaneously (the one with the longer delay will find the AP).
 *
 * ── Quiet-input detection (Master only) ──────────────────────────────────
 *   Every 4 KB chunk the master computes a normalised RMS value.
 *   If RMS stays below QUIET_THRESHOLD for QUIET_WARN_CHUNKS consecutive
 *   chunks (≈0.9 s) it prints a Serial warning and broadcasts a UDP status
 *   packet so the slave can echo it too.
 *
 * ── Quality features ──────────────────────────────────────────────────────
 *   • 4 KB chunks       – fewer packets, lower overhead, smoother flow
 *   • Smooth gain ramp  – gradual per-chunk ramp avoids hiss, pops, pumping
 *   • Jitter buffer     – 8-slot ring absorbs packet-timing variation and
 *                         prevents underruns; silence substituted on dropout
 *
 * Dependencies (install via Arduino Library Manager or platformio.ini):
 *   - pschatzmann/arduino-audio-tools  (AudioTools)
 *   - pschatzmann/arduino-audiokit     (AudioKit stream + HAL)
 */

#include "audio_config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <esp_wifi.h>               // esp_read_mac
#include "AudioLibs/AudioKit.h"     // AudioKitStream (Phil Schatzmann)

// ─── Packet structures ────────────────────────────────────────────────────

// Prepended to every 4 KB audio UDP datagram
struct __attribute__((packed)) AudioHeader {
    uint32_t seq;   // Monotonically increasing sequence number
    uint32_t len;   // Payload bytes in this datagram (always CHUNK_BYTES)
};

// Broadcast by master when line-in level is below threshold
struct __attribute__((packed)) StatusPacket {
    uint8_t type;        // STATUS_QUIET_INPUT
    float   rms;
    float   threshold;
};
static constexpr uint8_t STATUS_QUIET_INPUT = 0x01;

static constexpr size_t HEADER_SIZE = sizeof(AudioHeader);
static constexpr size_t PACKET_SIZE = HEADER_SIZE + CHUNK_BYTES;

// ─── Globals ──────────────────────────────────────────────────────────────

enum class Role { UNDECIDED, MASTER, SLAVE };
static Role g_role = Role::UNDECIDED;

static AudioKitStream g_kit;        // Hardware I2S ↔ codec interface
static WiFiUDP        g_audioUdp;   // Audio stream socket
static WiFiUDP        g_statusUdp;  // Out-of-band status socket

// Master-side state
static uint32_t g_txSeq       = 0;
static float    g_masterGain  = GAIN_DEFAULT;   // Current (ramped) send gain
static int      g_quietCount  = 0;              // Consecutive quiet-chunk counter
static bool     g_quietWarned = false;

// Slave-side state – jitter buffer
struct JitterSlot {
    bool     valid;
    uint32_t seq;
    uint8_t  data[CHUNK_BYTES];
};
static JitterSlot g_jitter[JITTER_SLOTS];
static uint32_t   g_rxExpected   = 0;           // Next sequence the player expects
static float      g_slaveGain    = GAIN_DEFAULT; // Current (ramped) playback gain
static bool       g_slaveStarted = false;        // True once pre-buffer is primed
static uint32_t   g_lastPlayMs   = 0;

// ─── Audio helpers ────────────────────────────────────────────────────────

/**
 * computeRMS – normalised RMS of a 16-bit PCM buffer.
 * Returns a value in [0, 1] where 1.0 represents full-scale.
 */
static float computeRMS(const int16_t *samples, size_t nSamples) {
    if (!nSamples) return 0.0f;
    double sum = 0.0;
    for (size_t i = 0; i < nSamples; ++i) {
        double s = samples[i] / 32768.0;
        sum += s * s;
    }
    return (float)sqrt(sum / (double)nSamples);
}

/**
 * applyGain – apply gain to a 16-bit PCM buffer in place, ramping
 * currentGain toward targetGain by at most GAIN_STEP_PER_CHUNK per call.
 * The gradual ramp prevents audible clicks, hiss, or pumping artefacts.
 */
static void applyGain(int16_t *samples, size_t nSamples,
                      float &currentGain, float targetGain) {
    float stepPerSample = GAIN_STEP_PER_CHUNK / (float)nSamples;
    for (size_t i = 0; i < nSamples; ++i) {
        if      (currentGain < targetGain)
            currentGain = fminf(currentGain + stepPerSample, targetGain);
        else if (currentGain > targetGain)
            currentGain = fmaxf(currentGain - stepPerSample, targetGain);

        float v = samples[i] * currentGain;
        samples[i] = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, v));
    }
}

// ─── Role negotiation ─────────────────────────────────────────────────────

/** Returns true if AP_SSID is visible in the current Wi-Fi scan results. */
static bool scanForMasterAP() {
    int n = WiFi.scanNetworks(/*async*/false, /*hidden*/false,
                              /*passive*/false, /*max_ms_per_chan*/DISCOVERY_SCAN_MS);
    for (int i = 0; i < n; ++i) {
        if (WiFi.SSID(i) == AP_SSID) return true;
    }
    return false;
}

/**
 * negotiateRole – decide at boot whether this board is Master or Slave.
 *
 * Algorithm:
 *   1. Scan for AP_SSID.  If found → Slave.
 *   2. Wait a short delay proportional to the MAC LSB (0…MAC_STAGGER_MS).
 *      This staggers simultaneous boots so one board creates the AP first.
 *   3. Scan again.  If found → Slave.
 *   4. Otherwise → Master.
 */
static void negotiateRole() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);

    Serial.println("[ROLE] Scanning for existing master AP...");
    if (scanForMasterAP()) {
        Serial.println("[ROLE] Master AP found → SLAVE");
        g_role = Role::SLAVE;
        return;
    }

    // Derive a stagger delay from the last byte of the station MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t staggerMs = (uint32_t)((uint64_t)mac[5] * MAC_STAGGER_MS / 255UL);
    Serial.printf("[ROLE] No master found.  Stagger %u ms (MAC LSB=0x%02X)...\n",
                  staggerMs, mac[5]);
    delay(staggerMs);

    Serial.println("[ROLE] Re-scanning after stagger...");
    if (scanForMasterAP()) {
        Serial.println("[ROLE] Master AP found after stagger → SLAVE");
        g_role = Role::SLAVE;
        return;
    }

    Serial.println("[ROLE] No master detected → MASTER");
    g_role = Role::MASTER;
}

// ─── Master ───────────────────────────────────────────────────────────────

static void setupMaster() {
    Serial.println("[MASTER] Creating Wi-Fi AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[MASTER] AP ready.  IP: %s\n",
                  WiFi.softAPIP().toString().c_str());

    // Configure AudioKit for line-in capture (ADC path)
    auto cfg = g_kit.defaultConfig(RX_MODE);
    cfg.input_device    = AUDIO_HAL_ADC_INPUT_LINE1;
    cfg.sample_rate     = SAMPLE_RATE;
    cfg.channels        = NUM_CHANNELS;
    cfg.bits_per_sample = BITS;
    g_kit.begin(cfg);

    g_audioUdp.begin(AUDIO_UDP_PORT);
    g_statusUdp.begin(STATUS_UDP_PORT);

    Serial.println("[MASTER] Listening for slave.  Audio streaming active.");
}

// Static transmit buffer – avoids repeated stack allocations in the hot path
static uint8_t s_txBuf[CHUNK_BYTES];

static void loopMaster() {
    // ── Read exactly one 4 KB chunk from line-in (blocks until complete) ──
    for (size_t filled = 0; filled < CHUNK_BYTES; ) {
        int n = g_kit.readBytes(s_txBuf + filled, CHUNK_BYTES - filled);
        if (n > 0) filled += (size_t)n;
        else delay(1);  // yield briefly if nothing ready yet
    }

    int16_t *samples  = reinterpret_cast<int16_t *>(s_txBuf);
    size_t   nSamples = CHUNK_BYTES / BYTES_PER_SAMPLE;

    // ── Quiet-input detection ─────────────────────────────────────────────
    // Compute RMS every chunk; if consistently below threshold, warn once.
    float rms = computeRMS(samples, nSamples);
    if (rms < QUIET_THRESHOLD) {
        ++g_quietCount;
        if (g_quietCount >= QUIET_WARN_CHUNKS && !g_quietWarned) {
            Serial.printf("[WARN] Line-in too quiet (RMS=%.5f, threshold=%.5f)."
                          "  Check source level.\n", rms, QUIET_THRESHOLD);
            g_quietWarned = true;

            // Also send a UDP status packet so the slave can display it
            IPAddress broadcast(192, 168, 4, 255);
            StatusPacket sp{ STATUS_QUIET_INPUT, rms, QUIET_THRESHOLD };
            g_statusUdp.beginPacket(broadcast, STATUS_UDP_PORT);
            g_statusUdp.write(reinterpret_cast<uint8_t *>(&sp), sizeof(sp));
            g_statusUdp.endPacket();
        }
    } else {
        // Level recovered – reset counters so the warning fires again later
        g_quietCount  = 0;
        g_quietWarned = false;
    }

    // ── Smooth gain ───────────────────────────────────────────────────────
    // g_masterGain ramps toward GAIN_DEFAULT each chunk.
    // Adjust g_masterGain externally (e.g. via Serial) to shift send level.
    applyGain(samples, nSamples, g_masterGain, GAIN_DEFAULT);

    // ── Transmit ──────────────────────────────────────────────────────────
    AudioHeader hdr{ g_txSeq++, (uint32_t)CHUNK_BYTES };
    IPAddress   dest(192, 168, 4, 255);  // AP-subnet broadcast

    g_audioUdp.beginPacket(dest, AUDIO_UDP_PORT);
    g_audioUdp.write(reinterpret_cast<uint8_t *>(&hdr), HEADER_SIZE);
    g_audioUdp.write(s_txBuf, CHUNK_BYTES);
    g_audioUdp.endPacket();
}

// ─── Slave ────────────────────────────────────────────────────────────────

static void setupSlave() {
    Serial.println("[SLAVE] Connecting to master AP...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASS);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
        if (millis() - t0 > 20000UL) {
            Serial.println("\n[SLAVE] Connection timeout – rebooting");
            ESP.restart();
        }
    }
    Serial.printf("\n[SLAVE] Connected.  IP=%s  GW=%s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str());

    // Configure AudioKit for line-out playback (DAC path)
    auto cfg = g_kit.defaultConfig(TX_MODE);
    cfg.output_device   = AUDIO_HAL_DAC_OUTPUT_LINE1;
    cfg.sample_rate     = SAMPLE_RATE;
    cfg.channels        = NUM_CHANNELS;
    cfg.bits_per_sample = BITS;
    g_kit.begin(cfg);

    g_audioUdp.begin(AUDIO_UDP_PORT);
    g_statusUdp.begin(STATUS_UDP_PORT);

    // Initialise all jitter-buffer slots as empty
    for (int i = 0; i < JITTER_SLOTS; ++i) {
        g_jitter[i].valid = false;
        g_jitter[i].seq   = 0;
    }
    g_lastPlayMs = millis();

    Serial.println("[SLAVE] Ready.  Waiting for audio stream...");
}

// Static receive and playback buffers
static uint8_t s_rxBuf[PACKET_SIZE];
static uint8_t s_playBuf[CHUNK_BYTES];

static void loopSlave() {
    // ── Receive incoming audio packets into the jitter buffer ─────────────
    int pktLen;
    while ((pktLen = g_audioUdp.parsePacket()) > 0) {
        if ((size_t)pktLen < HEADER_SIZE + 1) { g_audioUdp.flush(); continue; }

        size_t toRead = (pktLen < (int)PACKET_SIZE) ? (size_t)pktLen : PACKET_SIZE;
        g_audioUdp.read(s_rxBuf, toRead);

        AudioHeader hdr;
        memcpy(&hdr, s_rxBuf, HEADER_SIZE);
        if (hdr.len == 0 || hdr.len > CHUNK_BYTES) continue;

        // ── Sequence resync – handles master reboot or large wrap-around ──
        // diff < -(JITTER_SLOTS): very old packet or master reboot → resync.
        // Slightly late packets (diff in [-JITTER_SLOTS, 0)) are silently
        // discarded; they are already past the playback head and useless.
        // diff >= JITTER_SLOTS*4: implausibly large forward jump → resync.
        int32_t diff = (int32_t)(hdr.seq - g_rxExpected);
        if (diff < -(int32_t)JITTER_SLOTS || diff >= (int32_t)(JITTER_SLOTS * 4)) {
            Serial.printf("[SLAVE] Seq resync (got %u, expected %u)\n",
                          hdr.seq, g_rxExpected);
            g_rxExpected   = hdr.seq;
            g_slaveStarted = false;
            for (int i = 0; i < JITTER_SLOTS; ++i) g_jitter[i].valid = false;
        }

        // Store only if the packet falls within the current jitter window
        diff = (int32_t)(hdr.seq - g_rxExpected);
        if (diff >= 0 && diff < (int32_t)JITTER_SLOTS) {
            uint32_t slot = hdr.seq % JITTER_SLOTS;
            g_jitter[slot].valid = true;
            g_jitter[slot].seq   = hdr.seq;
            memcpy(g_jitter[slot].data, s_rxBuf + HEADER_SIZE, hdr.len);
        }
    }

    // ── Consume status packets from master ────────────────────────────────
    int sPktLen;
    while ((sPktLen = g_statusUdp.parsePacket()) > 0) {
        if ((size_t)sPktLen >= sizeof(StatusPacket)) {
            StatusPacket sp;
            g_statusUdp.read(reinterpret_cast<uint8_t *>(&sp), sizeof(sp));
            if (sp.type == STATUS_QUIET_INPUT) {
                Serial.printf("[STATUS] Master: line-in too quiet "
                              "(RMS=%.5f, threshold=%.5f)\n",
                              sp.rms, sp.threshold);
            }
        } else {
            g_statusUdp.flush();
        }
    }

    // ── Pre-buffer: hold playback until minimum slots are filled ──────────
    // This prevents an underrun right at startup before packets have arrived.
    if (!g_slaveStarted) {
        int count = 0;
        for (int i = 0; i < JITTER_SLOTS; ++i)
            if (g_jitter[i].valid) ++count;
        if (count < JITTER_PRE_BUFFER) return;
        g_slaveStarted = true;
        Serial.printf("[SLAVE] Jitter buffer primed (%d/%d slots), "
                      "starting playback\n", count, JITTER_SLOTS);
    }

    // ── Consume the next expected slot ────────────────────────────────────
    uint32_t slot = g_rxExpected % JITTER_SLOTS;
    bool      got  = g_jitter[slot].valid && (g_jitter[slot].seq == g_rxExpected);

    if (got) {
        // Happy path: packet is ready
        memcpy(s_playBuf, g_jitter[slot].data, CHUNK_BYTES);
        g_jitter[slot].valid = false;
        ++g_rxExpected;
        g_lastPlayMs = millis();
    } else if (millis() - g_lastPlayMs > (uint32_t)JITTER_SKIP_MS) {
        // Packet has been missing too long – advance and substitute silence
        memset(s_playBuf, 0, CHUNK_BYTES);
        ++g_rxExpected;
        g_lastPlayMs = millis();
        Serial.println("[SLAVE] Dropped packet – substituting silence");
    } else {
        // Still within the skip window – output silence, hold position
        memset(s_playBuf, 0, CHUNK_BYTES);
    }

    // ── Smooth gain and write to line-out ─────────────────────────────────
    int16_t *samples  = reinterpret_cast<int16_t *>(s_playBuf);
    size_t   nSamples = CHUNK_BYTES / BYTES_PER_SAMPLE;
    applyGain(samples, nSamples, g_slaveGain, GAIN_DEFAULT);

    // write() on AudioKitStream blocks until I2S accepts the data, which
    // naturally paces playback to the correct sample rate.
    for (size_t written = 0; written < CHUNK_BYTES; ) {
        int n = g_kit.write(s_playBuf + written, CHUNK_BYTES - written);
        if (n > 0) written += (size_t)n;
        else delay(1);
    }
}

// ─── Arduino entry points ─────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32 Audiokit UDP Streaming ===");
    Serial.println("    AudioTools / AudioKit by Phil Schatzmann");
    Serial.println("    Automatic master/slave role negotiation");
    Serial.println();

    negotiateRole();

    if (g_role == Role::MASTER) setupMaster();
    else                        setupSlave();
}

void loop() {
    if (g_role == Role::MASTER) loopMaster();
    else                        loopSlave();
}
