/*
 * esp32_audiokit_udp.ino
 *
 * Single-firmware sketch for two ESP32 Audiokit boards.
 * Flash the same binary to both boards; the role is set via Serial and
 * persisted in EEPROM (see "Role" section below).
 *
 * ── Audio flow ───────────────────────────────────────────────────────────
 *   Master  Line-In ──[I2S]──► RMS check ──► smooth gain ──► 4 KB PCM chunk ──► UDP ──►┐
 *   Slave   ◄── UDP ◄── jitter buffer ◄── smooth gain ◄──[I2S]◄── Line-Out           ◄─┘
 *
 * ── Role ──────────────────────────────────────────────────────────────────
 *   Role (MASTER/SLAVE) is persisted in EEPROM. Fresh boards default to
 *   MASTER. Set the role once per board via Serial: "mode master" or
 *   "mode slave" (takes effect after the next reboot).
 *
 * ── Quiet-input detection (Master only) ──────────────────────────────────
 *   Every 4 KB chunk the master computes a normalised RMS value.
 *   If RMS stays below QUIET_THRESHOLD for QUIET_WARN_CHUNKS consecutive
 *   chunks (≈0.9 s) it prints a Serial warning and broadcasts a UDP status
 *   packet so the slave can echo it too.
 *
 * ── Quality features ──────────────────────────────────────────────────────
 *   • Unicast delivery  – slave announces its IP via periodic "hello"
 *                         packets; master sends audio point-to-point
 *                         instead of broadcast (unreliable on ESP32 softAP)
 *   • Small chunks      – kept below the WiFi MTU to avoid IP fragmentation
 *   • Smooth gain ramp  – gradual per-chunk ramp avoids hiss, pops, pumping
 *   • Jitter buffer     – ring buffer absorbs packet-timing variation and
 *                         prevents underruns; silence substituted on dropout
 *
 * Dependencies (install via Arduino Library Manager or platformio.ini):
 *   - pschatzmann/arduino-audio-tools   (AudioTools)
 *   - pschatzmann/arduino-audio-driver  (AudioBoardStream + codec HAL)
 */

#include "audio_config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <EEPROM.h>
#include <cmath>    // sqrt, fminf, fmaxf
#include <cstring>  // memcpy, memset

#include "AudioTools.h"
#include "AudioTools/AudioLibs/AudioBoardStream.h"  // AudioBoardStream (Phil Schatzmann)

// ES8388-Codec-Board-Variante; bei Bedarf auf AudioKitEs8388V2 umstellen.
#define AUDIOKIT_ES8388_VARIANT 1

#define BOARD_ROLE_MASTER 1
#define BOARD_ROLE_SLAVE  2
#define EEPROM_MAGIC 0xA55A2033

// ─── Packet structures ────────────────────────────────────────────────────

// Prepended to every 4 KB audio UDP datagram
struct __attribute__((packed)) AudioHeader {
    uint32_t seq;   // Monotonically increasing sequence number
    uint32_t len;   // Payload bytes in this datagram (always CHUNK_BYTES)
};

// Sent by master when line-in level is below threshold, by slave as a
// registration "hello" so the master learns its unicast IP address, or by
// either side during a latency measurement (PING from master, PONG echo
// from slave, matched by seq).
struct __attribute__((packed)) StatusPacket {
    uint8_t  type;        // STATUS_QUIET_INPUT / STATUS_HELLO / STATUS_PING / STATUS_PONG
    float    rms;
    float    threshold;
    uint32_t seq;         // Probe sequence number, used only by PING/PONG
};
static constexpr uint8_t STATUS_QUIET_INPUT = 0x01;
static constexpr uint8_t STATUS_HELLO       = 0x02;
static constexpr uint8_t STATUS_PING        = 0x03;
static constexpr uint8_t STATUS_PONG        = 0x04;

static constexpr size_t HEADER_SIZE = sizeof(AudioHeader);
static constexpr size_t PACKET_SIZE = HEADER_SIZE + CHUNK_BYTES;

// ─── Globals ──────────────────────────────────────────────────────────────

enum class Role { UNDECIDED, MASTER, SLAVE };
static Role g_role = Role::UNDECIDED;

struct RoleConfig {
    uint32_t magic;
    uint8_t  role;       // BOARD_ROLE_MASTER or BOARD_ROLE_SLAVE
    uint8_t  latencyMs;  // Master: artificial per-chunk send delay (0-200 ms)
};
static RoleConfig g_roleConfig;
static constexpr uint32_t LATENCY_REPORT_INTERVAL_MS = 5000;

// Latency measurement (Master only, triggered via 'latency measure'):
// averages the round-trip time of several UDP probes to the slave and
// stores half of it (one-way estimate) as the new g_roleConfig.latencyMs.
static constexpr uint32_t LATENCY_PROBE_COUNT        = 20;
static constexpr uint32_t LATENCY_PROBE_INTERVAL_MS  = 30;
static constexpr uint32_t LATENCY_PROBE_TIMEOUT_MS   = 300;
static constexpr uint32_t CHUNK_DURATION_MS =
    (CHUNK_BYTES / BYTES_PER_SAMPLE / NUM_CHANNELS) * 1000UL / SAMPLE_RATE;
static uint32_t g_lastLatencyReportMs = 0;

#if AUDIOKIT_ES8388_VARIANT == 1
static AudioBoardStream g_kit(AudioKitEs8388V1);  // Hardware I2S <-> codec interface
#elif AUDIOKIT_ES8388_VARIANT == 2
static AudioBoardStream g_kit(AudioKitEs8388V2);
#else
#error "Unsupported AUDIOKIT_ES8388_VARIANT"
#endif

static WiFiUDP g_audioUdp;   // Audio stream socket
static WiFiUDP g_statusUdp;  // Out-of-band status socket

// Master-side state
static uint32_t g_txSeq       = 0;
static float    g_masterGain  = GAIN_DEFAULT;    // Current (ramped) send gain
static int      g_quietCount  = 0;               // Consecutive quiet-chunk counter
static bool     g_quietWarned = false;
static bool     g_rawDbgEnabled  = false;        // Print raw ADC peak/RMS (diagnostic)
static uint32_t g_rawDbgLastMs   = 0;

// Unicast target learned from the slave's periodic "hello" packets.
// Broadcast delivery on the ESP32 softAP is unreliable (DTIM-buffered,
// frequently dropped), so audio/status are always sent point-to-point.
static IPAddress g_slaveIp;
static bool      g_slaveKnown    = false;
static uint32_t  g_lastHelloRxMs = 0;
static constexpr uint32_t SLAVE_TIMEOUT_MS = 5000;

// Slave-side state – jitter buffer
struct JitterSlot {
    bool     valid;
    uint32_t seq;
    uint8_t  data[CHUNK_BYTES];
};
static JitterSlot g_jitter[JITTER_SLOTS];
static uint32_t   g_rxExpected    = 0;            // Next sequence the player expects
static float      g_slaveGain     = GAIN_DEFAULT; // Current (ramped) playback gain
static bool       g_slaveStarted  = false;        // True once pre-buffer is primed
static uint32_t   g_pktRxCount    = 0;             // UDP packets actually received
static uint32_t   g_pktDropCount = 0;             // Slots substituted with silence
static uint32_t   g_pktOverflowCount = 0;          // Packets discarded: buffer backlog full
static uint32_t   g_driftSkipCount   = 0;          // Extra slots skipped to drain clock-drift backlog
static uint32_t   g_lastStatsMs  = 0;
static constexpr int JITTER_HIGH_WATER = JITTER_SLOTS - 6;  // start draining backlog before it overflows
static uint32_t   g_lastPlayMs    = 0;
static uint32_t   g_lastHelloTxMs = 0;            // Last time we announced ourselves to master
static constexpr uint32_t HELLO_INTERVAL_MS = 1000;

// Master-side test tone (Serial: "testtone on|off") – generates a sine wave
// in place of Line-In, to test the pipeline without a physical audio source.
static bool          g_testToneEnabled = false;
static float         g_toneAngle       = 0.0f;
static uint32_t      g_toneNextUs      = 0;  // absolute micros() deadline for next chunk
static constexpr float TEST_TONE_FREQ_HZ   = 440.0f;
static constexpr float TEST_TONE_AMPLITUDE = 0.3f;
static constexpr float TEST_TONE_AMPLITUDE_LOW = 0.03f;  // ~ same peak as the quiet real Line-In signal
static float         g_testToneAmplitude = TEST_TONE_AMPLITUDE;
static constexpr float TEST_TONE_TWO_PI    = 6.28318530718f;
static constexpr uint32_t CHUNK_DURATION_US =
    (uint32_t)((uint64_t)(CHUNK_BYTES / BYTES_PER_SAMPLE / NUM_CHANNELS) * 1000000ULL / SAMPLE_RATE);

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

/** generateTestTone – fills an interleaved PCM buffer with a sine wave. */
static void generateTestTone(int16_t *samples, size_t nSamples) {
    size_t nFrames = nSamples / NUM_CHANNELS;
    for (size_t i = 0; i < nFrames; ++i) {
        int16_t v = (int16_t)(sinf(g_toneAngle) * g_testToneAmplitude * 32767.0f);
        for (size_t ch = 0; ch < NUM_CHANNELS; ++ch) {
            samples[i * NUM_CHANNELS + ch] = v;
        }
        g_toneAngle += TEST_TONE_TWO_PI * TEST_TONE_FREQ_HZ / SAMPLE_RATE;
        if (g_toneAngle > TEST_TONE_TWO_PI) g_toneAngle -= TEST_TONE_TWO_PI;
    }
}

/**
 * applyGain – apply gain to a 16-bit PCM buffer in place, ramping
 * currentGain toward targetGain by at most GAIN_STEP_PER_CHUNK per call.
 * The gradual ramp prevents audible clicks, hiss, or pumping artefacts.
 */
static void applyGain(int16_t *samples, size_t nSamples,
                      float &currentGain, float targetGain) {
    if (samples == nullptr || nSamples == 0) return;

    float stepPerSample = GAIN_STEP_PER_CHUNK / (float)nSamples;
    for (size_t i = 0; i < nSamples; ++i) {
        if (currentGain < targetGain) {
            currentGain = fminf(currentGain + stepPerSample, targetGain);
        } else if (currentGain > targetGain) {
            currentGain = fmaxf(currentGain - stepPerSample, targetGain);
        }

        float v = samples[i] * currentGain;
        if (v > 32767.0f) v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        samples[i] = (int16_t)v;
    }
}

// ─── Role persistence ─────────────────────────────────────────────────────

/** Loads the persisted role from EEPROM; fresh boards default to MASTER. */
static void loadRoleConfig() {
    EEPROM.begin(sizeof(RoleConfig));
    EEPROM.get(0, g_roleConfig);
    if (g_roleConfig.magic != EEPROM_MAGIC) {
        g_roleConfig.magic     = EEPROM_MAGIC;
        g_roleConfig.role      = BOARD_ROLE_MASTER;
        g_roleConfig.latencyMs = 0;
        EEPROM.put(0, g_roleConfig);
        EEPROM.commit();
    }
}

static void saveRoleConfig() {
    g_roleConfig.magic = EEPROM_MAGIC;
    EEPROM.put(0, g_roleConfig);
    EEPROM.commit();
}

/**
 * measureLatency – blocking round-trip measurement to the registered slave.
 * Sends LATENCY_PROBE_COUNT pings, waits for each pong, averages the
 * round-trip time, halves it for a one-way estimate, clamps to [0,200] ms,
 * and persists it as the new artificial send delay (g_roleConfig.latencyMs).
 * Pauses audio streaming for its short duration (a few hundred ms).
 */
static void measureLatency() {
    if (!g_slaveKnown) {
        Serial.println("[LATENCY] No slave registered - aborting measurement");
        return;
    }

    Serial.println("[LATENCY] Measuring round-trip latency to slave...");
    uint64_t sumUs = 0;
    uint32_t got    = 0;

    for (uint32_t seq = 0; seq < LATENCY_PROBE_COUNT; ++seq) {
        StatusPacket ping{ STATUS_PING, 0.0f, 0.0f, seq };
        uint32_t sentUs = micros();
        g_statusUdp.beginPacket(g_slaveIp, STATUS_UDP_PORT);
        g_statusUdp.write(reinterpret_cast<uint8_t *>(&ping), sizeof(ping));
        g_statusUdp.endPacket();

        uint32_t deadline = millis() + LATENCY_PROBE_TIMEOUT_MS;
        bool matched = false;
        while (millis() < deadline) {
            int pktLen = g_statusUdp.parsePacket();
            if (pktLen >= (int)sizeof(StatusPacket)) {
                StatusPacket resp;
                g_statusUdp.read(reinterpret_cast<uint8_t *>(&resp), sizeof(resp));
                if (resp.type == STATUS_PONG && resp.seq == seq) {
                    sumUs += (uint32_t)(micros() - sentUs);
                    ++got;
                    matched = true;
                    break;
                }
                // Ignore unrelated status packets (e.g. a hello) during the probe window
            }
        }
        if (!matched) {
            Serial.printf("[LATENCY] Probe %u/%u timed out\n", seq + 1, LATENCY_PROBE_COUNT);
        }
        delay(LATENCY_PROBE_INTERVAL_MS);
    }

    if (got == 0) {
        Serial.println("[LATENCY] Measurement failed - no responses received");
        return;
    }

    uint32_t avgRttUs   = (uint32_t)(sumUs / got);
    uint32_t oneWayMs   = (avgRttUs / 2) / 1000;
    uint8_t  newLatency = (uint8_t)constrain(oneWayMs, 0, 200);

    g_roleConfig.latencyMs = newLatency;
    saveRoleConfig();

    Serial.printf("[LATENCY] %u/%u probes answered, avg RTT=%.1f ms, stored latency=%u ms\n",
                  got, LATENCY_PROBE_COUNT, avgRttUs / 1000.0f, newLatency);
}

/** Handles "mode master" / "mode slave" / "info" / "help" Serial commands. */
static void handleSerialCommands() {
    static String input;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (input.length() > 0) {
                input.trim();
                int spaceIdx = input.indexOf(' ');
                String cmd = (spaceIdx >= 0) ? input.substring(0, spaceIdx) : input;
                String arg = (spaceIdx >= 0) ? input.substring(spaceIdx + 1) : "";

                if (cmd == "mode") {
                    if (arg == "master") {
                        g_roleConfig.role = BOARD_ROLE_MASTER;
                        saveRoleConfig();
                        Serial.println("Role set to MASTER - rebooting...");
                        delay(200);
                        ESP.restart();
                    } else if (arg == "slave") {
                        g_roleConfig.role = BOARD_ROLE_SLAVE;
                        saveRoleConfig();
                        Serial.println("Role set to SLAVE - rebooting...");
                        delay(200);
                        ESP.restart();
                    } else {
                        Serial.println("Use: mode master|slave");
                    }
                } else if (cmd == "info") {
                    Serial.print("Persisted role: ");
                    Serial.println(g_roleConfig.role == BOARD_ROLE_MASTER ? "MASTER" : "SLAVE");
                    Serial.print("Active role: ");
                    Serial.println(g_role == Role::MASTER ? "MASTER" : "SLAVE");
                    Serial.print("Latency delay: ");
                    Serial.print(g_roleConfig.latencyMs);
                    Serial.println(" ms");
                } else if (cmd == "gain") {
                    if (arg.length() > 0) {
                        int pct = constrain(arg.toInt(), 0, 100);
                        g_kit.setInputVolume(pct);
                        Serial.printf("Input PGA gain set to %d%%\n", pct);
                    } else {
                        Serial.println("Use: gain <0-100>");
                    }
                } else if (cmd == "rawdbg") {
                    if (arg == "on") {
                        g_rawDbgEnabled = true;
                        Serial.println("Raw ADC peak/RMS debug ON");
                    } else if (arg == "off") {
                        g_rawDbgEnabled = false;
                        Serial.println("Raw ADC peak/RMS debug OFF");
                    } else {
                        Serial.println("Use: rawdbg on|off");
                    }
                } else if (cmd == "latency") {
                    if (arg == "measure") {
                        measureLatency();
                        input = "";
                        continue;
                    } else if (arg == "+") {
                        g_roleConfig.latencyMs = (uint8_t)constrain(g_roleConfig.latencyMs + 5, 0, 200);
                    } else if (arg == "-") {
                        g_roleConfig.latencyMs = (uint8_t)constrain((int)g_roleConfig.latencyMs - 5, 0, 200);
                    } else if (arg.length() > 0) {
                        g_roleConfig.latencyMs = (uint8_t)constrain(arg.toInt(), 0, 200);
                    } else {
                        Serial.println("Use: latency +|-|measure|<0-200>");
                        input = "";
                        continue;
                    }
                    saveRoleConfig();
                    Serial.print("Latency delay set to ");
                    Serial.print(g_roleConfig.latencyMs);
                    Serial.println(" ms");
                } else if (cmd == "help") {
                    Serial.println("Commands: mode master|slave, latency +|-|measure|<0-200>, testtone on|low|off, gain <0-100>, rawdbg on|off, info, help");
                } else if (cmd == "testtone") {
                    if (arg == "on") {
                        g_testToneEnabled  = true;
                        g_testToneAmplitude = TEST_TONE_AMPLITUDE;
                        Serial.println("Test tone ON (440 Hz sine, replaces Line-In)");
                    } else if (arg == "low") {
                        g_testToneEnabled  = true;
                        g_testToneAmplitude = TEST_TONE_AMPLITUDE_LOW;
                        Serial.println("Test tone ON, low amplitude (simulates quiet Line-In level)");
                    } else if (arg == "off") {
                        g_testToneEnabled = false;
                        Serial.println("Test tone OFF (using Line-In)");
                    } else {
                        Serial.println("Use: testtone on|low|off");
                    }
                } else {
                    Serial.println("Unknown command (try 'help')");
                }
            }
            input = "";
        } else {
            if (input.length() < 120) {   // prevent unbounded growth on malformed input
                input += c;
            }
        }
    }
}

// ─── Master ───────────────────────────────────────────────────────────────

static void setupMaster() {
    Serial.println("[MASTER] Creating Wi-Fi AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.setSleep(false);  // keep radio awake so broadcast frames aren't delayed/dropped
    Serial.printf("[MASTER] AP ready.  IP: %s\n", WiFi.softAPIP().toString().c_str());

    // Configure AudioKit for line-in capture (ADC path)
    auto cfg = g_kit.defaultConfig(RX_MODE);
    cfg.input_device    = ADC_INPUT_LINE2;
    cfg.sample_rate     = SAMPLE_RATE;
    cfg.channels        = NUM_CHANNELS;
    cfg.bits_per_sample = BITS;
    cfg.sd_active       = false;
    if (!g_kit.begin(cfg)) {
        Serial.println("[MASTER] Audio init failed");
        while (true) delay(1000);
    }
    g_kit.setInputVolume(INPUT_GAIN_PERCENT);  // moderate PGA gain - less self-noise than the 24dB default

    g_audioUdp.begin(AUDIO_UDP_PORT);
    g_statusUdp.begin(STATUS_UDP_PORT);

    Serial.println("[MASTER] Listening for slave.  Audio streaming active.");
}

// Static transmit buffer – avoids repeated stack allocations in the hot path
static uint8_t s_txBuf[CHUNK_BYTES];

static void loopMaster() {
    // ── Learn/refresh the slave's unicast IP from its "hello" packets ──────
    int sPktLen;
    while ((sPktLen = g_statusUdp.parsePacket()) > 0) {
        if ((size_t)sPktLen >= sizeof(StatusPacket)) {
            StatusPacket sp;
            g_statusUdp.read(reinterpret_cast<uint8_t *>(&sp), sizeof(sp));
            if (sp.type == STATUS_HELLO) {
                IPAddress from = g_statusUdp.remoteIP();
                if (!g_slaveKnown || from != g_slaveIp) {
                    Serial.printf("[MASTER] Slave registered: %s\n", from.toString().c_str());
                }
                g_slaveIp       = from;
                g_slaveKnown    = true;
                g_lastHelloRxMs = millis();
            }
        } else {
            g_statusUdp.flush();
        }
    }

    if (g_slaveKnown && millis() - g_lastHelloRxMs > SLAVE_TIMEOUT_MS) {
        Serial.println("[MASTER] Slave timed out - pausing audio send");
        g_slaveKnown = false;
    }

    // ── Read exactly one 4 KB chunk from line-in (blocks until complete) ──
    if (g_testToneEnabled) {
        generateTestTone(reinterpret_cast<int16_t *>(s_txBuf), CHUNK_BYTES / BYTES_PER_SAMPLE);
        // Pace to real time using an absolute microsecond schedule (avoids
        // drift from millisecond truncation and from per-chunk overhead).
        uint32_t now = micros();
        if (g_toneNextUs == 0) g_toneNextUs = now;
        int32_t remaining = (int32_t)(g_toneNextUs - now);
        if (remaining > 0) {
            delayMicroseconds((uint32_t)remaining);
        } else if (remaining < -(int32_t)(CHUNK_DURATION_US * 4)) {
            // Fell far behind (e.g. Serial/network stall) – resync instead of bursting
            g_toneNextUs = now;
        }
        g_toneNextUs += CHUNK_DURATION_US;
    } else {
        for (size_t filled = 0; filled < CHUNK_BYTES;) {
            int n = g_kit.readBytes(s_txBuf + filled, CHUNK_BYTES - filled);
            if (n > 0) filled += (size_t)n;
            else delay(1);  // yield briefly if nothing ready yet
        }
    }

    int16_t *samples  = reinterpret_cast<int16_t *>(s_txBuf);
    size_t   nSamples = CHUNK_BYTES / BYTES_PER_SAMPLE;

    // ── Quiet-input detection (skipped for the synthetic test tone) ───────
    if (!g_testToneEnabled) {
        float rms = computeRMS(samples, nSamples);
        if (rms < QUIET_THRESHOLD) {
            ++g_quietCount;
            if (g_quietCount >= QUIET_WARN_CHUNKS && !g_quietWarned) {
                Serial.printf("[WARN] Line-in too quiet (RMS=%.5f, threshold=%.5f).  Check source level.\n",
                              rms, QUIET_THRESHOLD);
                g_quietWarned = true;

                // Also send a UDP status packet so the slave can display it
                if (g_slaveKnown) {
                    StatusPacket sp{ STATUS_QUIET_INPUT, rms, QUIET_THRESHOLD, 0 };
                    g_statusUdp.beginPacket(g_slaveIp, STATUS_UDP_PORT);
                    g_statusUdp.write(reinterpret_cast<uint8_t *>(&sp), sizeof(sp));
                    g_statusUdp.endPacket();
                }
            }
        } else {
            g_quietCount  = 0;
            g_quietWarned = false;
        }

        // ── Smooth gain ─────────────────────────────────────────────────
        applyGain(samples, nSamples, g_masterGain, GAIN_DEFAULT);

        if (g_rawDbgEnabled && millis() - g_rawDbgLastMs >= 500) {
            g_rawDbgLastMs = millis();
            int16_t peak = 0;
            for (size_t i = 0; i < nSamples; ++i) {
                int16_t a = samples[i] < 0 ? -samples[i] : samples[i];
                if (a > peak) peak = a;
            }
            Serial.printf("[RAWDBG] peak=%d rms=%.5f\n", peak, rms);
        }
    }

    // ── Artificial send delay (manually tunable via 'latency' command) ────
    if (g_roleConfig.latencyMs > 0) delay(g_roleConfig.latencyMs);

    if (millis() - g_lastLatencyReportMs >= LATENCY_REPORT_INTERVAL_MS) {
        g_lastLatencyReportMs = millis();
        Serial.printf("[MASTER] Latency delay: %u ms\n", g_roleConfig.latencyMs);
    }

    // ── Transmit (unicast – no slave registered yet means nothing to send) ─
    if (!g_slaveKnown) return;

    AudioHeader hdr{ g_txSeq++, (uint32_t)CHUNK_BYTES };

    g_audioUdp.beginPacket(g_slaveIp, AUDIO_UDP_PORT);
    g_audioUdp.write(reinterpret_cast<uint8_t *>(&hdr), HEADER_SIZE);
    g_audioUdp.write(s_txBuf, CHUNK_BYTES);
    g_audioUdp.endPacket();
}

// ─── Slave ────────────────────────────────────────────────────────────────

static void setupSlave() {
    Serial.println("[SLAVE] Connecting to master AP...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // disable modem-sleep – power save causes missed frames
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
    cfg.output_device   = DAC_OUTPUT_ALL;
    cfg.sample_rate     = SAMPLE_RATE;
    cfg.channels        = NUM_CHANNELS;
    cfg.bits_per_sample = BITS;
    cfg.sd_active       = false;
    if (!g_kit.begin(cfg)) {
        Serial.println("[SLAVE] Audio init failed");
        while (true) delay(1000);
    }

    g_audioUdp.begin(AUDIO_UDP_PORT);
    g_statusUdp.begin(STATUS_UDP_PORT);

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
    // ── Reconnect if the AP dropped (e.g. master rebooted) ──────────────
    static uint32_t s_lastWifiCheckMs = 0;
    if (millis() - s_lastWifiCheckMs >= 2000) {
        s_lastWifiCheckMs = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[SLAVE] WiFi disconnected - reconnecting...");
            WiFi.disconnect();
            WiFi.begin(AP_SSID, AP_PASS);
            g_slaveStarted = false;  // re-prime the jitter buffer once we recover
        }
    }

    // ── Announce ourselves to the master so it knows our unicast IP ───────
    if (millis() - g_lastHelloTxMs >= HELLO_INTERVAL_MS) {
        g_lastHelloTxMs = millis();
        StatusPacket sp{ STATUS_HELLO, 0.0f, 0.0f, 0 };
        g_statusUdp.beginPacket(WiFi.gatewayIP(), STATUS_UDP_PORT);
        g_statusUdp.write(reinterpret_cast<uint8_t *>(&sp), sizeof(sp));
        g_statusUdp.endPacket();
    }

    // ── Receive incoming audio packets into the jitter buffer ─────────────
    int pktLen;
    while ((pktLen = g_audioUdp.parsePacket()) > 0) {
        if ((size_t)pktLen < HEADER_SIZE + 1) {
            g_audioUdp.flush();
            continue;
        }

        size_t toRead = (pktLen < (int)PACKET_SIZE) ? (size_t)pktLen : PACKET_SIZE;
        g_audioUdp.read(s_rxBuf, toRead);
        ++g_pktRxCount;

        AudioHeader hdr;
        memcpy(&hdr, s_rxBuf, HEADER_SIZE);
        if (hdr.len == 0 || hdr.len > CHUNK_BYTES) continue;

        int32_t diff = (int32_t)(hdr.seq - g_rxExpected);
        if (diff < -(int32_t)JITTER_SLOTS || diff >= (int32_t)(JITTER_SLOTS * 4)) {
            Serial.printf("[SLAVE] Seq resync (got %u, expected %u)\n", hdr.seq, g_rxExpected);
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

            // copy only actual payload
            memcpy(g_jitter[slot].data, s_rxBuf + HEADER_SIZE, hdr.len);

            // zero-fill tail if packet shorter than CHUNK_BYTES
            if (hdr.len < CHUNK_BYTES) {
                memset(g_jitter[slot].data + hdr.len, 0, CHUNK_BYTES - hdr.len);
            }
        } else if (diff >= (int32_t)JITTER_SLOTS) {
            // Backlog already full (Master clock running slightly faster than Slave
            // playback) - packet is too far ahead to store, silently lost otherwise.
            ++g_pktOverflowCount;
        }
    }

    // ── Consume status packets from master ────────────────────────────────
    int sPktLen;
    while ((sPktLen = g_statusUdp.parsePacket()) > 0) {
        if ((size_t)sPktLen >= sizeof(StatusPacket)) {
            IPAddress pingFrom = g_statusUdp.remoteIP();
            StatusPacket sp;
            g_statusUdp.read(reinterpret_cast<uint8_t *>(&sp), sizeof(sp));
            if (sp.type == STATUS_QUIET_INPUT) {
                Serial.printf("[STATUS] Master: line-in too quiet (RMS=%.5f, threshold=%.5f)\n",
                              sp.rms, sp.threshold);
            } else if (sp.type == STATUS_PING) {
                // Echo back immediately so the master can measure round-trip time
                StatusPacket pong{ STATUS_PONG, 0.0f, 0.0f, sp.seq };
                g_statusUdp.beginPacket(pingFrom, STATUS_UDP_PORT);
                g_statusUdp.write(reinterpret_cast<uint8_t *>(&pong), sizeof(pong));
                g_statusUdp.endPacket();
            }
        } else {
            g_statusUdp.flush();
        }
    }

    // ── Pre-buffer: hold playback until minimum slots are filled ──────────
    if (!g_slaveStarted) {
        int count = 0;
        for (int i = 0; i < JITTER_SLOTS; ++i)
            if (g_jitter[i].valid) ++count;
        if (count < JITTER_PRE_BUFFER) return;
        g_slaveStarted = true;
        Serial.printf("[SLAVE] Jitter buffer primed (%d/%d slots), starting playback\n",
                      count, JITTER_SLOTS);
    }

    // ── Periodic buffer-depth report (approximate playback latency) ───────
    if (millis() - g_lastLatencyReportMs >= LATENCY_REPORT_INTERVAL_MS) {
        g_lastLatencyReportMs = millis();
        int count = 0;
        for (int i = 0; i < JITTER_SLOTS; ++i)
            if (g_jitter[i].valid) ++count;
        Serial.printf("[SLAVE] Buffer depth: %d/%d slots (~%u ms)\n",
                      count, JITTER_SLOTS, count * CHUNK_DURATION_MS);
    }

    // ── Consume the next expected slot ────────────────────────────────────
    // Drift compensation: if the backlog is piling up (Master's ADC clock runs
    // marginally faster than the Slave's DAC clock), proactively drop one extra
    // buffered slot so the backlog drains gradually instead of overflowing every
    // few seconds (which caused periodic multi-packet loss bursts / crackling).
    int depthNow = 0;
    for (int i = 0; i < JITTER_SLOTS; ++i)
        if (g_jitter[i].valid) ++depthNow;
    if (depthNow > JITTER_HIGH_WATER && g_jitter[g_rxExpected % JITTER_SLOTS].valid) {
        g_jitter[g_rxExpected % JITTER_SLOTS].valid = false;
        ++g_rxExpected;
        ++g_driftSkipCount;
    }

    uint32_t slot = g_rxExpected % JITTER_SLOTS;
    bool got = g_jitter[slot].valid && (g_jitter[slot].seq == g_rxExpected);

    if (got) {
        memcpy(s_playBuf, g_jitter[slot].data, CHUNK_BYTES);
        g_jitter[slot].valid = false;
        ++g_rxExpected;
        g_lastPlayMs = millis();
    } else if (millis() - g_lastPlayMs > (uint32_t)JITTER_SKIP_MS) {
        memset(s_playBuf, 0, CHUNK_BYTES);
        ++g_rxExpected;
        g_lastPlayMs = millis();
        ++g_pktDropCount;
    } else {
        memset(s_playBuf, 0, CHUNK_BYTES);
    }

    if (millis() - g_lastStatsMs >= 1000) {
        g_lastStatsMs = millis();
        Serial.printf("[STATS] rx=%u drops=%u overflow=%u driftSkip=%u rssi=%d dBm\n",
                      g_pktRxCount, g_pktDropCount, g_pktOverflowCount, g_driftSkipCount, WiFi.RSSI());
        g_pktRxCount       = 0;
        g_pktDropCount     = 0;
        g_pktOverflowCount = 0;
        g_driftSkipCount   = 0;
    }

    // ── Smooth gain and write to line-out ─────────────────────────────────
    int16_t *samples  = reinterpret_cast<int16_t *>(s_playBuf);
    size_t   nSamples = CHUNK_BYTES / BYTES_PER_SAMPLE;
    applyGain(samples, nSamples, g_slaveGain, GAIN_DEFAULT);

    // write() on AudioBoardStream blocks until I2S accepts the data.
    for (size_t written = 0; written < CHUNK_BYTES;) {
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
    Serial.println("    AudioTools / AudioBoardStream by Phil Schatzmann");
    Serial.println();

    loadRoleConfig();
    Serial.print("[ROLE] Persisted role: ");
    Serial.println(g_roleConfig.role == BOARD_ROLE_MASTER ? "MASTER" : "SLAVE");
    Serial.println("[ROLE] Change with 'mode master' / 'mode slave' via Serial (takes effect after reboot)");

    g_role = (g_roleConfig.role == BOARD_ROLE_SLAVE) ? Role::SLAVE : Role::MASTER;

    if (g_role == Role::MASTER) setupMaster();
    else                        setupSlave();
}

void loop() {
    handleSerialCommands();
    if (g_role == Role::MASTER) loopMaster();
    else                        loopSlave();
}
