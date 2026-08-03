#ifndef MASTER_MODE

// Helper macros for embedding compile-time constants in string literals
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x)       STRINGIFY_INNER(x)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <AudioKitStream.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "config.h"
#include "audio_packet.h"
#include "slave.h"

// ── Packet stored in the FreeRTOS jitter queue ────────────────
struct QueuedPacket {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t  volume;
    uint16_t len;
    uint8_t  data[FRAME_SIZE];
};

// ── Shared RTOS objects ───────────────────────────────────────
static QueueHandle_t     audioQueue;
static SemaphoreHandle_t cfgMutex;   // guards the config variables below

// Config — written by slave_loop (serial), read by both tasks
static volatile uint8_t  cfg_target  = JITTER_TARGET_DEFAULT;
static volatile uint8_t  cfg_max     = JITTER_MAX_DEFAULT;
static volatile uint8_t  cfg_volume  = DEFAULT_VOLUME;
static volatile bool     cfg_vol_new = false;  // pending volume apply

// Statistics — written by tasks, read by slave_loop (serial)
static volatile uint32_t stat_rx     = 0;
static volatile uint32_t stat_drop   = 0;
static volatile uint32_t stat_play   = 0;
static volatile uint32_t stat_silence = 0;

// AudioKit instance used exclusively from audio_task (Core 1)
static AudioKitStream kit;

// ── Network task (Core 0) ─────────────────────────────────────
//
// Receives UDP packets and pushes them into the jitter queue.
// Implements latency kill: when the queue exceeds cfg_max, it
// drains oldest packets down to cfg_target before inserting.
// ─────────────────────────────────────────────────────────────
static void network_task(void* /*param*/) {
    WiFiUDP udp;
    udp.begin(AUDIO_UDP_PORT);

    // Static buffers — kept off the task stack
    static uint8_t rx_buf[sizeof(AudioPacketHeader) + FRAME_SIZE];
    static QueuedPacket pkt;

    Serial.printf("[slave] net_task  core=%d\n", xPortGetCoreID());

    for (;;) {
        int pkt_bytes = udp.parsePacket();
        if (pkt_bytes <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int n = udp.read(rx_buf, sizeof(rx_buf));
        if (n < (int)sizeof(AudioPacketHeader)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        const AudioPacketHeader* hdr =
            reinterpret_cast<const AudioPacketHeader*>(rx_buf);

        if (hdr->magic != PACKET_MAGIC_AUDIO ||
            hdr->payload_len == 0 ||
            hdr->payload_len > FRAME_SIZE) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        stat_rx++;

        // ── Volume synchronisation ────────────────────────────
        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        if (hdr->volume != cfg_volume) {
            cfg_volume  = hdr->volume;
            cfg_vol_new = true;
        }
        uint8_t cur_max    = cfg_max;
        uint8_t cur_target = cfg_target;
        xSemaphoreGive(cfgMutex);

        // ── Latency kill ──────────────────────────────────────
        // If the queue has grown beyond the kill threshold, drop
        // oldest packets until the depth returns to target.
        // This prevents stale audio from ever playing and bounds
        // the worst-case latency to (cfg_target * frame_ms).
        while ((int)uxQueueMessagesWaiting(audioQueue) >= cur_max) {
            QueuedPacket dummy;
            xQueueReceive(audioQueue, &dummy, 0);
            stat_drop++;
        }
        // After a latency-kill drain, also reset to target depth
        // so the audio task immediately has fresh material.
        while ((int)uxQueueMessagesWaiting(audioQueue) > cur_target) {
            QueuedPacket dummy;
            xQueueReceive(audioQueue, &dummy, 0);
            stat_drop++;
        }

        // ── Enqueue fresh packet ──────────────────────────────
        pkt.seq          = hdr->seq;
        pkt.timestamp_ms = hdr->timestamp_ms;
        pkt.volume       = hdr->volume;
        pkt.len          = hdr->payload_len;
        memcpy(pkt.data, rx_buf + sizeof(AudioPacketHeader),
               hdr->payload_len);

        // Non-blocking send; if queue is somehow still full, drop.
        if (xQueueSend(audioQueue, &pkt, 0) != pdTRUE) {
            stat_drop++;
        }
    }
}

// ── Audio task (Core 1) ───────────────────────────────────────
//
// Pops packets from the jitter queue and writes PCM to I2S.
// Plays silence while buffering and on underrun.
// Applies volume changes received from master via shared state.
// ─────────────────────────────────────────────────────────────
static void audio_task(void* /*param*/) {
    static uint8_t     silence[FRAME_SIZE] = {};
    static QueuedPacket pkt;

    uint8_t applied_vol = DEFAULT_VOLUME;
    bool    buffering   = true;

    Serial.printf("[slave] audio_task  core=%d\n", xPortGetCoreID());

    for (;;) {
        // ── Apply pending volume change ───────────────────────
        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        bool    vol_pending = cfg_vol_new;
        uint8_t new_vol     = cfg_volume;
        uint8_t cur_target  = cfg_target;
        xSemaphoreGive(cfgMutex);

        if (vol_pending && new_vol != applied_vol) {
            applied_vol = new_vol;
            // arduino-audio-tools: float 0.0–1.0
            kit.setVolume((float)new_vol / 100.0f);
            xSemaphoreTake(cfgMutex, portMAX_DELAY);
            cfg_vol_new = false;
            xSemaphoreGive(cfgMutex);
        }

        // ── Buffering gate ────────────────────────────────────
        int depth = (int)uxQueueMessagesWaiting(audioQueue);

        if (buffering) {
            if (depth < cur_target) {
                // Not enough data yet — output silence
                kit.write(silence, FRAME_SIZE);
                stat_silence++;
                continue;
            }
            buffering = false;   // Buffer reached target; start playing
        }

        // ── Playback ──────────────────────────────────────────
        // Wait up to one frame period for the next packet.
        // On timeout (underrun), play silence and re-buffer.
        const TickType_t frame_ticks =
            pdMS_TO_TICKS((SAMPLES_PER_PACKET * 1000UL) / SAMPLE_RATE + 2);

        if (xQueueReceive(audioQueue, &pkt, frame_ticks) == pdTRUE) {
            kit.write(pkt.data, pkt.len);
            stat_play++;

            // Re-enter buffering if queue has emptied completely
            if (uxQueueMessagesWaiting(audioQueue) == 0) {
                buffering = true;
            }
        } else {
            // Underrun: play silence and wait for buffer to refill
            kit.write(silence, FRAME_SIZE);
            stat_silence++;
            buffering = true;
        }
    }
}

// ── Serial command handler (called from slave_loop) ───────────

static void process_serial(const String& cmd) {
    if (cmd.startsWith("buf=")) {
        int v = cmd.substring(4).toInt();
        if (v < 1 || v >= JITTER_QUEUE_CAPACITY) {
            Serial.printf("[slave] ERROR: buf must be 1–%d\n",
                          JITTER_QUEUE_CAPACITY - 1);
            return;
        }
        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        cfg_target = (uint8_t)v;
        // Keep max sane relative to new target
        if (cfg_max <= cfg_target) cfg_max = cfg_target + 1;
        xSemaphoreGive(cfgMutex);
        Serial.printf("[slave] Jitter target → %d pkts (~%.0f ms)\n",
                      v, (float)v * SAMPLES_PER_PACKET / SAMPLE_RATE * 1000.0f);

    } else if (cmd.startsWith("max=")) {
        int v = cmd.substring(4).toInt();
        if (v < 1 || v > JITTER_QUEUE_CAPACITY) {
            Serial.printf("[slave] ERROR: max must be 1–%d\n",
                          JITTER_QUEUE_CAPACITY);
            return;
        }
        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        cfg_max = (uint8_t)v;
        // Keep target sane relative to new max
        if (cfg_target >= cfg_max) cfg_target = cfg_max > 1 ? cfg_max - 1 : 1;
        xSemaphoreGive(cfgMutex);
        Serial.printf("[slave] Latency-kill threshold → %d pkts (~%.0f ms)\n",
                      v, (float)v * SAMPLES_PER_PACKET / SAMPLE_RATE * 1000.0f);

    } else if (cmd.startsWith("vol=")) {
        int v = cmd.substring(4).toInt();
        if (v < 0 || v > 100) {
            Serial.println("[slave] ERROR: vol must be 0–100");
            return;
        }
        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        cfg_volume  = (uint8_t)v;
        cfg_vol_new = true;
        xSemaphoreGive(cfgMutex);
        Serial.printf("[slave] Local volume override → %d\n", v);

    } else if (cmd == "stats") {
        int depth = (int)uxQueueMessagesWaiting(audioQueue);
        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        uint8_t t = cfg_target, m = cfg_max, v = cfg_volume;
        xSemaphoreGive(cfgMutex);
        Serial.printf(
            "[slave] RX=%u  Played=%u  Dropped=%u  Silence=%u  "
            "Buf=%d  Target=%d  Max=%d  Vol=%d\n",
            stat_rx, stat_play, stat_drop, stat_silence,
            depth, t, m, v);

    } else if (cmd == "help") {
        Serial.println("[slave] Commands:");
        Serial.println("  buf=N   jitter-buffer target in packets (default "
                       STRINGIFY(JITTER_TARGET_DEFAULT) ")");
        Serial.println("          Lower = less latency, more risk of stuttering");
        Serial.println("  max=N   latency-kill threshold in packets (default "
                       STRINGIFY(JITTER_MAX_DEFAULT) ")");
        Serial.println("          When buf > max, oldest packets are dropped");
        Serial.println("          Must be > buf");
        Serial.println("  vol=N   local volume override 0–100");
        Serial.println("  stats   show RX / playback statistics");
        Serial.println("  help    show this help");

    } else {
        Serial.printf("[slave] Unknown: '%s'  — type 'help'\n", cmd.c_str());
    }
}

// ── Setup ─────────────────────────────────────────────────────

void slave_setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("\n[slave] Booting…");

    // Create RTOS primitives
    audioQueue = xQueueCreate(JITTER_QUEUE_CAPACITY, sizeof(QueuedPacket));
    cfgMutex   = xSemaphoreCreateMutex();
    if (!audioQueue || !cfgMutex) {
        Serial.println("[slave] FATAL: RTOS object creation failed");
        for (;;) delay(1000);
    }

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[slave] Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print('.');
    }
    Serial.printf("\n[slave] Connected. IP: %s\n",
                  WiFi.localIP().toString().c_str());
    Serial.printf("[slave] Enter this IP as SLAVE_IP in config.h on the master.\n");

    // Configure AudioKit for line-out playback (TX only)
    AudioKitStreamConfig cfg = kit.defaultConfig(TX_MODE);
    cfg.sample_rate           = SAMPLE_RATE;
    cfg.bits_per_sample       = BITS_PER_SAMPLE;
    cfg.channels              = AUDIO_CHANNELS;
    cfg.dac_output            = SLAVE_DAC_OUTPUT;
    cfg.buffer_size           = DMA_BUF_LEN;
    cfg.buffer_count          = DMA_BUF_COUNT;
    kit.begin(cfg);
    kit.setVolume((float)DEFAULT_VOLUME / 100.0f);

    float frame_ms = (float)SAMPLES_PER_PACKET / SAMPLE_RATE * 1000.0f;
    Serial.printf("[slave] Listening on UDP port %d  frame=%.1f ms  "
                  "target=%d pkts  max=%d pkts\n",
                  AUDIO_UDP_PORT, frame_ms,
                  JITTER_TARGET_DEFAULT, JITTER_MAX_DEFAULT);
    Serial.println("[slave] Type 'help' for serial commands.");

    // Launch tasks — network on Core 0 (with WiFi), audio on Core 1
    xTaskCreatePinnedToCore(network_task, "net_task",  8192, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(audio_task,  "audio_task", 8192, nullptr, 2, nullptr, 1);
    // slave_loop() (serial handler) runs in the Arduino task on Core 1
    // at priority 1.  audio_task at priority 2 preempts it while active.
}

// ── Loop (serial handler, Core 1, priority 1) ─────────────────

void slave_loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) process_serial(cmd);
    }
    delay(10);   // yield so audio_task can run between I2S writes
}

#endif // !MASTER_MODE
