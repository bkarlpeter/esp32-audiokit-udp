/**
 * esp32-audiokit-udp
 * ══════════════════
 * Two AI-Thinker ESP32-A1S AudioKit boards exchange a live audio stream
 * over WiFi/UDP.
 *
 *  Master  – captures audio from the line-in jack (ES8388 ADC → I2S → UDP)
 *  Slave   – plays the audio on the line-out jack  (UDP → jitter buffer →
 *             I2S → ES8388 DAC)
 *
 * Both devices connect to the same WiFi network.  Static IPs are configured
 * in include/config.h.  The build environment selects the role:
 *
 *   pio run -e master -t upload
 *   pio run -e slave  -t upload
 *
 * Serial commands (115200 baud, send with newline)
 * ────────────────────────────────────────────────
 *   v <0-100>   Set volume
 *               Master: adjusts ADC PGA gain + syncs slave volume via UDP
 *               Slave:  adjusts DAC output volume locally
 *   f           Flush the slave jitter buffer (kills accumulated latency)
 *               Master: sends CMD_FLUSH control packet to slave
 *               Slave:  flushes local buffer immediately
 *   b <1-14>    Set jitter-buffer target depth (packets)
 *               Smaller = lower latency, more sensitive to jitter
 *   s           Print statistics (packet counts, buffer level, WiFi RSSI)
 *   l           Print estimated end-to-end latency
 *   h           Print this command list
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <driver/i2s.h>
#include "config.h"
#include "audio_packet.h"
#include "es8388.h"

#if defined(SLAVE_MODE)
#include "jitter_buffer.h"
#endif

// ── Compile-time sanity ───────────────────────────────────────────────────────
#if defined(MASTER_MODE) && defined(SLAVE_MODE)
#error "Define exactly one of MASTER_MODE or SLAVE_MODE — not both."
#endif
#if !defined(MASTER_MODE) && !defined(SLAVE_MODE)
#error "Define MASTER_MODE=1 or SLAVE_MODE=1 in platformio.ini build_flags."
#endif

// ── Global objects ────────────────────────────────────────────────────────────
static WiFiUDP audioUdp;
static WiFiUDP ctrlUdp;

#if defined(MASTER_MODE)
static uint32_t  txSeq      = 0;
static uint8_t   masterVol  = DEFAULT_VOLUME;
static IPAddress slaveAddr;
#endif

#if defined(SLAVE_MODE)
static JitterBuffer jitterBuf;
static uint8_t      slaveVol     = DEFAULT_VOLUME;
static bool         slaveRunning = true;

// Pre-allocated silence & output buffers
static int16_t silence[AUDIO_SAMPLES_TOTAL] = {};
static int16_t outBuf [AUDIO_SAMPLES_TOTAL];
#endif

// ── WiFi ──────────────────────────────────────────────────────────────────────
static void connectWiFi() {
    Serial.printf("[WiFi] Connecting to \"%s\"", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print('.');
    }
    Serial.printf("\n[WiFi] Connected — IP %s\n",
                  WiFi.localIP().toString().c_str());
}

// ── I2S initialisation ────────────────────────────────────────────────────────
static void initI2S() {
    i2s_config_t cfg = {};
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = I2S_DMA_BUF_COUNT;
    cfg.dma_buf_len          = I2S_DMA_BUF_LEN;
    cfg.use_apll             = true;   // use APLL for accurate sample rate
    cfg.tx_desc_auto_clear   = true;   // auto-zero TX DMA on underrun

#if defined(MASTER_MODE)
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
#else
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
#endif

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = PIN_I2S_MCLK;
    pins.bck_io_num   = PIN_I2S_BCLK;
    pins.ws_io_num    = PIN_I2S_LRCK;
    pins.data_out_num = PIN_I2S_DOUT;
    pins.data_in_num  = PIN_I2S_DIN;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pins));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_0));
    Serial.println("[I2S] Initialised");
}

// ── Control packet sender (master only) ──────────────────────────────────────
#if defined(MASTER_MODE)
static void sendCtrl(CtrlCmd cmd, uint8_t value, uint32_t param = 0) {
    static CtrlPkt pkt;
    pkt.hdr.magic        = PKT_MAGIC;
    pkt.hdr.type         = PKT_CTRL;
    pkt.hdr.seq          = txSeq++;
    pkt.hdr.timestamp_us = (uint32_t)micros();
    pkt.hdr.payload_len  = (uint16_t)(CTRL_PKT_SIZE - PKT_HEADER_SIZE);
    pkt.cmd              = cmd;
    pkt.value            = value;
    pkt.param            = param;

    ctrlUdp.beginPacket(slaveAddr, CTRL_PORT);
    ctrlUdp.write((const uint8_t*)&pkt, CTRL_PKT_SIZE);
    ctrlUdp.endPacket();
}
#endif

// ── Serial command handler ────────────────────────────────────────────────────
static void handleSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    const char  cmd    = (char)tolower(line.charAt(0));
    const int   argInt = (line.length() > 1) ? line.substring(1).toInt() : 0;

    switch (cmd) {

        // ── Volume ────────────────────────────────────────────────────────
        case 'v': {
            uint8_t vol = (uint8_t)constrain(argInt, 0, 100);
#if defined(MASTER_MODE)
            masterVol = vol;
            // On master, volume maps to ADC PGA gain (0-8 steps)
            codec.setInputGain((uint8_t)map(vol, 0, 100, 0, 8));
            // Synchronise slave loudness via control packet
            sendCtrl(CMD_VOLUME, vol);
            Serial.printf("[Master] Volume → %u/100 (synced to slave)\n", vol);
#else
            slaveVol = vol;
            codec.setVolume(vol);
            Serial.printf("[Slave] Volume → %u/100\n", vol);
#endif
            break;
        }

        // ── Flush jitter buffer ───────────────────────────────────────────
        case 'f': {
#if defined(MASTER_MODE)
            sendCtrl(CMD_FLUSH, 0);
            Serial.println("[Master] CMD_FLUSH sent to slave");
#else
            jitterBuf.flush();
            i2s_zero_dma_buffer(I2S_NUM_0);
            Serial.println("[Slave] Buffer and DMA flushed locally");
#endif
            break;
        }

        // ── Jitter-buffer target depth ────────────────────────────────────
        case 'b': {
            int n = constrain(argInt, 1, JITTER_BUF_CAPACITY - 2);
#if defined(MASTER_MODE)
            sendCtrl(CMD_BUFSIZE, (uint8_t)n);
            Serial.printf("[Master] CMD_BUFSIZE=%d sent to slave\n", n);
#else
            jitterBuf.setTargetLevel(n);
            Serial.printf("[Slave] Buffer target → %d packets (%.1f ms)\n",
                          n, (float)n * SAMPLES_PER_PACKET * 1000.0f / SAMPLE_RATE);
#endif
            break;
        }

        // ── Statistics ────────────────────────────────────────────────────
        case 's': {
#if defined(MASTER_MODE)
            Serial.println("=== Master Statistics ===");
            Serial.printf("  TX packets : %lu\n", (unsigned long)txSeq);
            Serial.printf("  Volume     : %u/100\n", masterVol);
            Serial.printf("  Slave IP   : %s\n",  slaveAddr.toString().c_str());
            Serial.printf("  WiFi RSSI  : %d dBm\n", WiFi.RSSI());
#else
            Serial.println("=== Slave Statistics ===");
            Serial.printf("  Volume     : %u/100\n", slaveVol);
            Serial.printf("  Buf level  : %d / %d packets\n",
                          jitterBuf.level(), JITTER_BUF_CAPACITY);
            Serial.printf("  RX packets : %lu\n",
                          (unsigned long)jitterBuf.getRxCount());
            Serial.printf("  Lost pkts  : %lu\n",
                          (unsigned long)jitterBuf.getLossCount());
            Serial.printf("  Dropped    : %lu\n",
                          (unsigned long)jitterBuf.getDropCount());
            Serial.printf("  Underruns  : %lu\n",
                          (unsigned long)jitterBuf.getUnderruns());
            Serial.printf("  WiFi RSSI  : %d dBm\n", WiFi.RSSI());
#endif
            break;
        }

        // ── Latency estimate ──────────────────────────────────────────────
        case 'l': {
            const float pkt_ms = (float)SAMPLES_PER_PACKET * 1000.0f / SAMPLE_RATE;
            const float dma_ms = (float)I2S_DMA_BUF_COUNT * pkt_ms;
#if defined(MASTER_MODE)
            Serial.printf("[Master] Packet period : %.1f ms\n", pkt_ms);
            Serial.printf("[Master] DMA depth     : %.1f ms (%d buffers)\n",
                          dma_ms, I2S_DMA_BUF_COUNT);
#else
            const float jit_ms  = (float)jitterBuf.level() * pkt_ms;
            const float pre_ms  = (float)JITTER_MIN_PREBUFFER * pkt_ms;
            Serial.printf("[Slave] Packet period   : %.1f ms\n", pkt_ms);
            Serial.printf("[Slave] Jitter buf now  : %d pkts / %.1f ms\n",
                          jitterBuf.level(), jit_ms);
            Serial.printf("[Slave] Min pre-buffer  : %d pkts / %.1f ms\n",
                          JITTER_MIN_PREBUFFER, pre_ms);
            Serial.printf("[Slave] DMA depth       : %.1f ms (%d buffers)\n",
                          dma_ms, I2S_DMA_BUF_COUNT);
            Serial.printf("[Slave] Est. total      : ~%.0f ms\n",
                          pre_ms + dma_ms + 2.0f /* WiFi est. */);
#endif
            break;
        }

        // ── Help ──────────────────────────────────────────────────────────
        case 'h':
        default:
            Serial.println("=== Commands (send with newline) ===");
            Serial.println("  v <0-100>  Set volume");
            Serial.println("  f          Flush slave jitter buffer (kill latency)");
            Serial.println("  b <1-14>   Set jitter-buffer target depth (packets)");
            Serial.println("  s          Show statistics");
            Serial.println("  l          Show latency estimate");
            Serial.println("  h          Show this help");
            break;
    }
}

// ── Master transmit loop ──────────────────────────────────────────────────────
#if defined(MASTER_MODE)

static AudioPkt txPkt;

static void masterTransmit() {
    size_t bytes_read = 0;

    // i2s_read blocks until the DMA buffer has filled with the requested
    // number of bytes — this naturally paces the TX loop at the sample rate.
    esp_err_t err = i2s_read(I2S_NUM_0,
                             txPkt.samples,
                             sizeof(txPkt.samples),
                             &bytes_read,
                             portMAX_DELAY);

    if (err != ESP_OK || bytes_read != sizeof(txPkt.samples)) return;

    txPkt.hdr.magic        = PKT_MAGIC;
    txPkt.hdr.type         = PKT_AUDIO;
    txPkt.hdr.seq          = txSeq++;
    txPkt.hdr.timestamp_us = (uint32_t)micros();
    txPkt.hdr.payload_len  = (uint16_t)sizeof(txPkt.samples);

    audioUdp.beginPacket(slaveAddr, AUDIO_PORT);
    audioUdp.write((const uint8_t*)&txPkt, AUDIO_PKT_SIZE);
    audioUdp.endPacket();
}

#endif  // MASTER_MODE

// ── Slave receive helpers ─────────────────────────────────────────────────────
#if defined(SLAVE_MODE)

static void drainAudioUDP() {
    int sz;
    static AudioPkt rxPkt;
    while ((sz = audioUdp.parsePacket()) > 0) {
        if (sz < (int)AUDIO_PKT_SIZE) { audioUdp.flush(); continue; }
        int n = audioUdp.read((uint8_t*)&rxPkt, sizeof(rxPkt));
        if (n != (int)AUDIO_PKT_SIZE)                 continue;
        if (rxPkt.hdr.magic != PKT_MAGIC)             continue;
        if (rxPkt.hdr.type  != PKT_AUDIO)             continue;
        jitterBuf.push(rxPkt);
    }
}

static void drainCtrlUDP() {
    int sz;
    static CtrlPkt cp;
    while ((sz = ctrlUdp.parsePacket()) > 0) {
        if (sz < (int)CTRL_PKT_SIZE) { ctrlUdp.flush(); continue; }
        int n = ctrlUdp.read((uint8_t*)&cp, sizeof(cp));
        if (n < (int)CTRL_PKT_SIZE)      continue;
        if (cp.hdr.magic != PKT_MAGIC)   continue;
        if (cp.hdr.type  != PKT_CTRL)    continue;

        switch (cp.cmd) {
            case CMD_VOLUME:
                slaveVol = cp.value;
                codec.setVolume(cp.value);
                Serial.printf("[Slave] Volume synced: %u/100\n", cp.value);
                break;
            case CMD_FLUSH:
                jitterBuf.flush();
                i2s_zero_dma_buffer(I2S_NUM_0);
                Serial.println("[Slave] Buffer flushed by master");
                break;
            case CMD_BUFSIZE:
                jitterBuf.setTargetLevel(cp.value);
                Serial.printf("[Slave] Buffer target → %u packets\n", cp.value);
                break;
            case CMD_START:
                slaveRunning = true;
                Serial.println("[Slave] Stream started");
                break;
            case CMD_STOP:
                slaveRunning = false;
                Serial.println("[Slave] Stream stopped");
                break;
            default:
                break;
        }
    }
}

// ── Slave output loop ─────────────────────────────────────────────────────────

static void slaveOutput() {
    size_t written = 0;

    if (!slaveRunning) {
        // Not running — output silence with short timeout to keep draining UDP
        i2s_write(I2S_NUM_0, silence, sizeof(silence), &written,
                  pdMS_TO_TICKS(10));
        return;
    }

    if (!jitterBuf.isReady()) {
        // Pre-buffering — output silence but don't block long so UDP keeps
        // being drained
        i2s_write(I2S_NUM_0, silence, sizeof(silence), &written,
                  pdMS_TO_TICKS(10));
        return;
    }

    // Normal playback: pop one packet and write to I2S DMA.
    // portMAX_DELAY blocks until the DMA ring accepts the data, which
    // naturally paces the loop at the audio sample rate.
    bool ok = jitterBuf.pop(outBuf);
    const int16_t* src = ok ? outBuf : silence;
    i2s_write(I2S_NUM_0, src, sizeof(outBuf), &written, portMAX_DELAY);
}

#endif  // SLAVE_MODE

// ── setup() ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

#if defined(MASTER_MODE)
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  ESP32 AudioKit UDP — MASTER          ║");
    Serial.println("╚══════════════════════════════════════╝");
#else
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  ESP32 AudioKit UDP — SLAVE           ║");
    Serial.println("╚══════════════════════════════════════╝");
#endif
    Serial.println("Type 'h' for command help.");

    // Connect to WiFi
    connectWiFi();

    // Initialise ES8388 codec over I2C
    if (!codec.begin(PIN_I2C_SDA, PIN_I2C_SCL, ES8388_I2C_ADDR)) {
        Serial.println("[ERROR] ES8388 not detected on I2C bus! Check wiring.");
        // Continue anyway — I2S may still be useful for debugging.
    }

#if defined(MASTER_MODE)
    codec.configADC();
    codec.setInputGain((uint8_t)map(masterVol, 0, 100, 0, 8));
    slaveAddr.fromString(SLAVE_IP);
    audioUdp.begin(AUDIO_PORT);  // listen port for future ACK / RTT packets
    ctrlUdp.begin(CTRL_PORT);
    Serial.printf("[Master] Streaming to slave at %s:%u\n", SLAVE_IP, AUDIO_PORT);
#else
    codec.configDAC();
    codec.setVolume(DEFAULT_VOLUME);
    jitterBuf.init(JITTER_TARGET_LEVEL);
    audioUdp.begin(AUDIO_PORT);
    ctrlUdp.begin(CTRL_PORT);
    Serial.printf("[Slave]  Listening on :%u (audio) :%u (ctrl)\n",
                  AUDIO_PORT, CTRL_PORT);
#endif

    // Initialise I2S last (after codec is ready to receive/provide clocks)
    initI2S();

    Serial.println("[Setup] Ready.");
    Serial.printf("  Sample rate  : %d Hz\n", SAMPLE_RATE);
    Serial.printf("  Packet size  : %u samples/ch  (%.1f ms)\n",
                  SAMPLES_PER_PACKET,
                  (float)SAMPLES_PER_PACKET * 1000.0f / SAMPLE_RATE);
#if defined(SLAVE_MODE)
    Serial.printf("  Jitter buf   : target %d pkts, max %d pkts, flush at %d\n",
                  JITTER_TARGET_LEVEL, JITTER_BUF_CAPACITY, JITTER_MAX_LEVEL);
#endif
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop() {
    handleSerial();

#if defined(MASTER_MODE)
    masterTransmit();   // blocks ~5.8 ms on i2s_read, then sends one UDP pkt
#else
    drainAudioUDP();    // non-blocking: drain all waiting audio packets
    drainCtrlUDP();     // non-blocking: process any control packets
    slaveOutput();      // write one frame to I2S (blocks ~5.8 ms in steady state)
#endif
}
