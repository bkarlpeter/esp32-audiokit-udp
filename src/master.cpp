#ifdef MASTER_MODE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <AudioKitStream.h>
#include "config.h"
#include "audio_packet.h"
#include "master.h"

// ── Module-private state ──────────────────────────────────────

static AudioKitStream kit;
static WiFiUDP        udp;

// Slave destination — can be changed at runtime with ip=x.x.x.x
static char slave_ip[32] = SLAVE_IP;

static uint32_t pkt_seq        = 0;
static uint8_t  current_volume = DEFAULT_VOLUME;

// Running statistics
static uint32_t stat_tx_pkts  = 0;
static uint32_t stat_tx_bytes = 0;

// Reusable buffers (static to avoid repeated stack allocation)
static uint8_t audio_buf[FRAME_SIZE];
static uint8_t tx_buf[sizeof(AudioPacketHeader) + FRAME_SIZE];

// ── Helpers ───────────────────────────────────────────────────

static void apply_volume(uint8_t v) {
    current_volume = v;
    // arduino-audio-tools uses float 0.0–1.0; fall back to
    // kit.setVolume(v) if your library version expects 0–100.
    kit.setVolume((float)v / 100.0f);
}

// ── Serial command handler ────────────────────────────────────

static void process_serial(const String& cmd) {
    if (cmd.startsWith("vol=")) {
        int v = cmd.substring(4).toInt();
        if (v < 0 || v > 100) {
            Serial.println("[master] ERROR: vol must be 0–100");
            return;
        }
        apply_volume((uint8_t)v);
        Serial.printf("[master] Volume → %d (will sync to slave on next packet)\n", v);

    } else if (cmd.startsWith("ip=")) {
        String ip_str = cmd.substring(3);
        ip_str.trim();
        ip_str.toCharArray(slave_ip, sizeof(slave_ip));
        Serial.printf("[master] Slave IP → %s\n", slave_ip);

    } else if (cmd == "stats") {
        Serial.printf("[master] IP=%s  Slave=%s:%d  Seq=%u  TX=%u pkts / %u bytes  Vol=%d\n",
            WiFi.localIP().toString().c_str(),
            slave_ip, AUDIO_UDP_PORT,
            pkt_seq, stat_tx_pkts, stat_tx_bytes, current_volume);

    } else if (cmd == "help") {
        Serial.println("[master] Commands:");
        Serial.println("  vol=N       set master volume 0-100 (instantly synced to slave)");
        Serial.println("  ip=x.x.x.x change slave destination IP at runtime");
        Serial.println("  stats       show TX statistics");
        Serial.println("  help        show this help");

    } else {
        Serial.printf("[master] Unknown command: '%s'  — type 'help'\n", cmd.c_str());
    }
}

// ── Setup ─────────────────────────────────────────────────────

void master_setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("\n[master] Booting…");

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[master] Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print('.');
    }
    Serial.printf("\n[master] Connected. IP: %s\n",
                  WiFi.localIP().toString().c_str());

    udp.begin(AUDIO_UDP_PORT);

    // Configure AudioKit for line-in capture (RX only)
    AudioKitStreamConfig cfg = kit.defaultConfig(RX_MODE);
    cfg.sample_rate           = SAMPLE_RATE;
    cfg.bits_per_sample       = BITS_PER_SAMPLE;
    cfg.channels              = AUDIO_CHANNELS;
    cfg.adc_input             = MASTER_ADC_INPUT;
    cfg.buffer_size           = DMA_BUF_LEN;
    cfg.buffer_count          = DMA_BUF_COUNT;
    kit.begin(cfg);
    apply_volume(DEFAULT_VOLUME);

    Serial.printf("[master] Streaming line-in → %s:%d  frame=%d bytes (~%.1f ms)\n",
                  slave_ip, AUDIO_UDP_PORT, FRAME_SIZE,
                  (float)SAMPLES_PER_PACKET / SAMPLE_RATE * 1000.0f);
    Serial.println("[master] Type 'help' for serial commands.");
}

// ── Loop ──────────────────────────────────────────────────────

void master_loop() {
    // 1. Handle serial commands (non-blocking check)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) process_serial(cmd);
    }

    // 2. Read one audio frame from line-in via I2S DMA
    //    readBytes blocks until FRAME_SIZE bytes are available.
    int n = kit.readBytes((char*)audio_buf, FRAME_SIZE);
    if (n <= 0) return;

    // 3. Build the packet: header + raw PCM payload
    AudioPacketHeader* hdr = reinterpret_cast<AudioPacketHeader*>(tx_buf);
    hdr->magic        = PACKET_MAGIC_AUDIO;
    hdr->seq          = pkt_seq++;
    hdr->timestamp_ms = millis();
    hdr->volume       = current_volume;   // slave will apply this volume
    hdr->pad          = 0;
    hdr->payload_len  = static_cast<uint16_t>(n);
    memcpy(tx_buf + sizeof(AudioPacketHeader), audio_buf, n);

    // 4. Fire-and-forget UDP send to slave
    udp.beginPacket(slave_ip, AUDIO_UDP_PORT);
    udp.write(tx_buf, sizeof(AudioPacketHeader) + n);
    udp.endPacket();

    stat_tx_pkts++;
    stat_tx_bytes += sizeof(AudioPacketHeader) + (uint32_t)n;
}

#endif // MASTER_MODE
