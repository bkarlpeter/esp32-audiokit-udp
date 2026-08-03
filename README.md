# esp32-audiokit-udp

Stream live audio between two **AI-Thinker ESP32-A1S AudioKit** boards over
WiFi/UDP with low latency, automatic latency correction, and master-controlled
loudness synchronisation.

```
Line-In ──► [MASTER] ──UDP──► [SLAVE] ──► Line-Out / Headphones
                ▲
           Serial monitor (fine-tuning)
```

---

## Features

| Feature | Detail |
|---|---|
| **Audio streaming** | 44 100 Hz, 16-bit, stereo PCM over LAN UDP |
| **Latency killer** | Jitter buffer flushes automatically when depth exceeds threshold; instant re-sync |
| **Loudness sync** | Master volume changes are forwarded to the slave via control packets |
| **Serial fine-tuning** | Volume, buffer depth, flush, and stats available from both boards |
| **Gap detection** | Sequence-number tracking triggers immediate re-sync on packet loss |

---

## Hardware

- 2 × AI-Thinker ESP32-A1S AudioKit (ES8388 codec)
- WiFi LAN (both boards on the same network / SSID)
- Line-level audio source connected to the **master** board's Line-In jack
- Speakers / amplifier connected to the **slave** board's Line-Out or headphone jack

### Wiring (AudioKit pin-out)

All pins below are hard-wired on the AudioKit PCB and require no extra wiring.

| Signal | GPIO |
|---|---|
| I²C SDA (ES8388 config) | 33 |
| I²C SCL (ES8388 config) | 32 |
| I²S MCLK | 0 |
| I²S BCLK | 27 |
| I²S LRCK | 25 |
| I²S DOUT (ESP32 → DAC) | 26 |
| I²S DIN  (ADC → ESP32) | 35 |

---

## Quick Start

### 1 · Install PlatformIO

```bash
pip install platformio
```

### 2 · Configure

Edit `include/config.h`:

```c
#define WIFI_SSID     "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
#define MASTER_IP     "192.168.1.100"   // fixed IP of the master board
#define SLAVE_IP      "192.168.1.101"   // fixed IP of the slave board
```

Assign static IP leases to both boards in your router (matched to their MAC
addresses), or configure `WiFi.config()` before `WiFi.begin()` in `main.cpp`.

### 3 · Flash

```bash
# Flash the master board
pio run -e master -t upload

# Flash the slave board
pio run -e slave -t upload
```

### 4 · Monitor

Open two terminal sessions (one per board):

```bash
pio device monitor   # 115200 baud
```

---

## Serial Commands

Both boards accept commands on the serial monitor (115200 baud). Terminate
each command with a newline (`Enter`).

| Command | Description |
|---|---|
| `v <0-100>` | **Set volume** — on master: adjusts ADC gain and syncs slave volume via UDP; on slave: sets DAC output volume locally |
| `f` | **Flush jitter buffer** — on master: sends `CMD_FLUSH` control packet to slave; on slave: flushes local buffer and resets DMA |
| `b <1-14>` | **Buffer depth** — set jitter-buffer target (number of packets). Smaller = lower latency, higher risk of underruns |
| `s` | **Statistics** — RX/TX packet counts, buffer level, packet loss, WiFi RSSI |
| `l` | **Latency estimate** — pre-buffer, DMA depth, and total estimated delay |
| `h` | **Help** — print command list |

### Example session (slave)

```
> v 80
[Slave] Volume → 80/100

> s
=== Slave Statistics ===
  Volume     : 80/100
  Buf level  : 4 / 16 packets
  RX packets : 12473
  Lost pkts  : 0
  Dropped    : 0
  Underruns  : 1
  WiFi RSSI  : -52 dBm

> l
[Slave] Packet period   : 5.8 ms
[Slave] Jitter buf now  : 4 pkts / 23.2 ms
[Slave] Min pre-buffer  : 2 pkts / 11.6 ms
[Slave] DMA depth       : 23.2 ms (4 buffers)
[Slave] Est. total      : ~37 ms
```

---

## How Latency Killing Works

1. **Minimum pre-buffer** (`JITTER_MIN_PREBUFFER = 2` packets ≈ 12 ms) —
   playback starts only after this many packets are queued, providing a
   short cushion against WiFi jitter.

2. **Overflow flush** — if the jitter buffer fills to `JITTER_MAX_LEVEL`
   (default 8 packets ≈ 46 ms), it is flushed immediately and playback
   restarts from the next incoming packet.  This caps the maximum latency.

3. **Gap detection** — if the slave receives a sequence number higher than
   expected (indicating dropped packets), the buffer is flushed and
   re-synchronised.

4. **Manual flush** — type `f` on either board (or the master sends
   `CMD_FLUSH`) to reset the slave buffer on demand, instantly bringing
   latency back to the minimum pre-buffer level.

Typical end-to-end latency on a quiet LAN: **~35–45 ms**.

---

## Tuning Parameters (`include/config.h`)

| Parameter | Default | Effect |
|---|---|---|
| `SAMPLE_RATE` | 44100 | Audio sample rate. Change to 48000 if APLL is unstable |
| `SAMPLES_PER_PACKET` | 256 | Samples per channel per UDP packet. Smaller = lower per-packet latency but higher packet rate |
| `I2S_DMA_BUF_COUNT` | 4 | Number of DMA ring buffers. Fewer = lower DMA latency |
| `JITTER_MIN_PREBUFFER` | 2 | Packets to buffer before starting playback |
| `JITTER_TARGET_LEVEL` | 4 | Desired steady-state buffer depth |
| `JITTER_MAX_LEVEL` | 8 | Flush threshold (latency killer trigger) |

All of `JITTER_*` can also be changed at runtime via the serial `b` command.

---

## Project Structure

```
platformio.ini          Build environments (master / slave)
include/
  config.h              All user-tunable settings
  audio_packet.h        UDP packet structs (AudioPkt, CtrlPkt)
  es8388.h              ES8388 codec driver interface
  jitter_buffer.h       Jitter buffer interface
src/
  main.cpp              Application entry point (setup / loop)
  es8388.cpp            ES8388 I²C register configuration
  jitter_buffer.cpp     Thread-safe circular jitter buffer
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| No audio on slave | Wrong slave IP or slave not on same WiFi | Check `SLAVE_IP` in config.h; run `s` on both boards |
| Glitches / crackles | WiFi jitter | Increase `JITTER_TARGET_LEVEL` or run `b 6` on slave |
| Growing latency | Clock drift (master/slave crystal mismatch) | Run `f` to flush; consider reducing `JITTER_MAX_LEVEL` |
| ES8388 not detected | I²C wiring | Verify SDA=33, SCL=32 and that ES8388_I2C_ADDR=0x10 |
| Sample rate inaccurate | APLL issue | Change `SAMPLE_RATE` to 48000 in config.h |

