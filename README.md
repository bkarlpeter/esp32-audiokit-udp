# esp32-audiokit-udp

Real-time, low-latency audio link between two **AI-Thinker ESP32-A1S AudioKit** boards over UDP/WiFi.

| Role | Audio flow |
|------|-----------|
| **Master** | Analog LINE IN → WiFi UDP |
| **Slave**  | WiFi UDP → Analog LINE OUT |

## Features

- **PCM audio streaming** — 44 100 Hz · 16-bit · stereo over UDP
- **Jitter buffer with latency kill** — the slave buffers incoming packets; when the queue depth exceeds a configurable threshold, all stale packets are dropped to guarantee bounded, low latency
- **Volume synchronisation** — every audio packet carries the master's current volume; the slave applies it to the ES8388 codec immediately, with no extra control channel needed
- **Serial fine-tuning** — adjust volume, jitter-target, and latency-kill threshold at runtime on either device without reflashing

## Hardware

- 2× AI-Thinker ESP32-A1S AudioKit (ES8388 codec)
- Both devices on the same WiFi network (2.4 GHz)
- Audio source connected to master **LINE IN** (3.5 mm jack)
- Amplifier or headphones connected to slave **LINE OUT / headphone** jack

## Quick Start

### 1. Prerequisites

Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

### 2. Configure

Edit **`include/config.h`**:

```c
#define WIFI_SSID       "your_ssid"
#define WIFI_PASSWORD   "your_password"
#define SLAVE_IP        "192.168.x.x"   // IP of the slave (see step 3)
```

### 3. Flash

```bash
# Flash the slave first — open its serial monitor to find its IP.
pio run -e slave -t upload
pio device monitor          # note the IP printed on boot

# Update SLAVE_IP in config.h, then flash master.
pio run -e master -t upload
pio device monitor
```

---

## Serial Commands

### Master (`pio run -e master`)

| Command | Description |
|---------|-------------|
| `vol=N` | Set master volume 0–100. Synced to slave on the next packet. |
| `ip=x.x.x.x` | Change slave destination IP at runtime. |
| `stats` | Show TX packet count, bytes, and current volume. |
| `help` | List all commands. |

### Slave (`pio run -e slave`)

| Command | Description |
|---------|-------------|
| `buf=N` | Jitter-buffer target depth in packets (default **5**). Lower = less latency; raise if playback stutters. |
| `max=N` | Latency-kill threshold in packets (default **12**). When queue exceeds this, oldest packets are dropped until depth == target. Must be > `buf`. |
| `vol=N` | Local volume override 0–100 (overrides master sync until next packet). |
| `stats` | Show RX / playback / drop counts and current buffer state. |
| `help` | List all commands. |

---

## Timing Reference

At 44 100 Hz · 16-bit · stereo, one UDP packet carries **256 samples ≈ 5.8 ms** of audio.

| Parameter | Default | Latency |
|-----------|:-------:|---------|
| `buf` (target) | 5 pkts | ≈ 29 ms |
| `max` (kill)   | 12 pkts | ≈ 70 ms (hard ceiling) |

Both values are adjustable live via serial. A good starting point for a clean LAN is `buf=3 max=7`.

---

## Architecture

```
Master (Core 0+1)                       Slave (Core 0 / Core 1)
─────────────────                       ───────────────────────
ADC (line-in)
  │ I2S / DMA
AudioKitStream::readBytes()
  │ FRAME_SIZE bytes
Build AudioPacketHeader                 net_task (Core 0)
  │  .magic, .seq, .timestamp_ms          WiFiUDP::parsePacket()
  │  .volume (current master vol)           → validate header
  │  .payload_len                           → volume sync (cfgMutex)
UDP sendPacket ──────────────────────→    → latency kill (drain to target)
                                          → xQueueSend(audioQueue)
                                                   │ FreeRTOS queue
                                          audio_task (Core 1)
                                            xQueueReceive(audioQueue)
                                            AudioKitStream::write()
                                              │ I2S / DMA
                                            DAC (line-out)
```

### Latency-kill mechanism

The network task checks the queue depth on every received packet.  
If `depth >= cfg_max` the task discards packets from the **front** of the queue (oldest audio) until `depth == cfg_target`, then inserts the freshest packet.  
This means the slave is always playing audio that is at most `cfg_target × frame_ms` old, regardless of how long the WiFi link was degraded.

### Volume synchronisation

`master_loop()` embeds `current_volume` (0–100) in every `AudioPacketHeader::volume` field.  
`network_task()` compares `hdr->volume` with the last known value; on a change it sets a shared flag.  
`audio_task()` picks up the flag before each I2S write and calls `kit.setVolume(new_vol / 100.0f)`.  
The result: turning the master volume knob via `vol=N` is reflected on the slave within one packet interval (< 10 ms on a clean LAN).

---

## Board Configuration

The build flag `-DAUDIOKIT_BOARD=5` selects the AI-Thinker ESP32-A1S v2.2 (ES8388).  
If your board does not initialise or produces silence, try `-DAUDIOKIT_BOARD=6` (v2.3) in `platformio.ini`.  
See [`arduino-audiokit/src/AudioKitConfig.h`](https://github.com/pschatzmann/arduino-audiokit/blob/main/src/AudioKitConfig.h) for the full board list.

## Library Versions

| Library | Tested version |
|---------|---------------|
| `pschatzmann/arduino-audiokit` | ^1.3.0 |
| `pschatzmann/arduino-audio-tools` | ^0.9.0 |

`AudioKitStream::setVolume(float)` expects **0.0–1.0**.  
If your installed version uses the integer `setVolume(int)` API (0–100), replace `kit.setVolume((float)v / 100.0f)` with `kit.setVolume(v)` in `master.cpp` and `slave.cpp`.

