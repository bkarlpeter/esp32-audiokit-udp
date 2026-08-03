# esp32-audiokit-udp

Low-latency PCM audio streaming between two **ESP32 AudioKit** boards using
Phil Schatzmann's [AudioTools](https://github.com/pschatzmann/arduino-audio-tools) /
[ESP32-AudioKit](https://github.com/pschatzmann/arduino-esp32-audiokit) ecosystem.

```
Master (AP)  ──UDP broadcast──▶  Slave (STA)
line-in ──I2S──▶ ESP32           ESP32 ──I2S──▶ line-out
```

---

## Features

| Feature | Detail |
|---|---|
| **Roles** | Master (sender) and Slave (receiver) in one sketch file |
| **Network** | Master acts as Wi-Fi AP; Slave connects as a station |
| **Transport** | Raw 16-bit mono PCM over UDP – no codec, minimal latency |
| **Packet header** | Sequence number, sample index, master gain, RMS level |
| **Jitter buffer** | Ring buffer on the slave (~80 ms start-up depth, tunable) |
| **Loudness sync** | Slave derives adaptive gain from master RMS metadata |
| **Serial tuning** | Live parameter changes without reflashing |

---

## Required libraries

Install both libraries via the **Arduino Library Manager** or the Arduino IDE
*Manage Libraries* dialog:

1. **arduino-audio-tools** – `pschatzmann/arduino-audio-tools`
2. **arduino-esp32-audiokit** – `pschatzmann/arduino-esp32-audiokit`

---

## Hardware

Tested on the **AI Thinker AudioKit v2.2** (ESP32, ES8388 codec).
Should also work on boards with the AC101 codec — the `AudioKitStream`
driver auto-detects the codec.

| Connector | Master | Slave |
|---|---|---|
| Audio in  | line-in socket | — |
| Audio out | — | line-out socket |
| USB / UART | serial monitor 115 200 baud | serial monitor 115 200 baud |

---

## Quick start

### 1 – Flash the master

Open `esp32_audiokit_udp/esp32_audiokit_udp.ino` in the Arduino IDE.

Ensure the role flag at the top of the file reads:

```cpp
#define IS_MASTER  1
```

Select **ESP32 Dev Module** (or AI Thinker ESP32-A1S), then upload.

### 2 – Flash the slave

Change the flag:

```cpp
#define IS_MASTER  0
```

Upload to the second board.

### 3 – Connect audio cables

- Master: plug an audio source into the **line-in** socket.
- Slave: connect speakers or amplifier to the **line-out** socket.

### 4 – Power on

- The master starts a Wi-Fi AP named **`AudioKitLink`** (password `audiokit1`).
- The slave automatically connects to that AP.
- Audio streaming begins as soon as the jitter buffer pre-rolls (~80 ms).

---

## Serial tuning commands (115 200 baud)

Type a command in the serial monitor on either board and press **Enter**.

| Command | Effect | Example |
|---|---|---|
| `g <val>` | Set gain (master: input gain; slave: base output gain) | `g 1.5` |
| `l <ms>` | Set latency target in milliseconds (informational) | `l 40` |
| `n on\|off` | Enable / disable adaptive loudness normalisation on slave | `n off` |
| `?` | Show help | `?` |

---

## Audio / network flow

```
MASTER
  I2S line-in  ──readBytes()──▶  applyGain()  ──▶  measureRMS()
               ──▶  PktHeader{seq, sampleIdx, masterGain, masterRMS}
               ──▶  UDP.beginPacket(broadcast) ──▶  sendPacket()

SLAVE
  UDP.parsePacket()  ──▶  PktHeader decode
    ├─ sequence gap?  →  log loss warning
    ├─ update adaptiveGain (smoothed loudness match)
    └─ push PCM into jitter ring-buffer

  jitterBuffer.drain()  ──▶  applyGain(adaptiveGain)
                        ──▶  I2S line-out  ──▶  speaker
```

---

## Tuning notes

- **Latency**: lower `JITTER_SLOTS` or `PCM_BYTES_PER_PACKET` to reduce latency
  at the cost of more frequent dropouts on a congested network.
- **Bandwidth**: `PCM_BYTES_PER_PACKET = 882` at 22 050 Hz mono ≈ 44 kB/s,
  well within the capacity of a direct AP link.
- **Sample rate**: change `SAMPLE_RATE` to 44100 for higher quality; the
  bandwidth doubles to ~176 kB/s and packets grow accordingly.
- **Loudness sync**: the slave smooths gain changes over ~20 packets
  (`α = 0.05`) to prevent pumping.  Disable with `n off` if not needed.
