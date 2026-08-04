# esp32-audiokit-udp

Low-latency, artefact-free audio streaming between two ESP32 Audiokit boards over Wi-Fi UDP.
Flash the **same firmware** to both boards; they negotiate roles automatically at boot.

---

## Features

| Feature | Description |
|---|---|
| Single firmware | One `.ino` for both boards — no manual role selection |
| Auto role negotiation | MAC-staggered AP scan elects master/slave at boot |
| Master as Wi-Fi AP | Direct board-to-board link, no router required |
| Raw PCM over UDP | No compression; lossless 44.1 kHz 16-bit stereo |
| 4 KB audio chunks | ~23 ms per packet — less overhead, smoother flow |
| Quiet-input detection | RMS check every chunk; Serial + UDP warning when line-in is too low |
| Smooth gain ramp | Per-chunk gain ramp prevents hiss, pops, and pumping |
| Jitter buffer | 8-slot ring (~185 ms) absorbs packet-timing variation; silence on dropout |
| AudioTools compatible | Uses Phil Schatzmann's AudioKit stream and AudioTools ecosystem |

---

## Audio flow

```
Master  Line-In ──[I2S]──► RMS check ──► smooth gain ──► 4 KB PCM ──► UDP ──►┐
Slave   ◄── UDP ◄── jitter buffer ◄── smooth gain ◄──[I2S]◄── Line-Out      ◄─┘
```

---

## Role negotiation

Both boards run the same code.  On every boot:

1. Board scans for Wi-Fi AP `AudioKit-Master`.
2. **AP found** → board becomes **Slave** and connects.
3. **AP not found** → board waits a short delay derived from its MAC address
   (prevents simultaneous-boot collision), then scans again.
4. Still not found → board becomes **Master**, creates the AP, and starts streaming.

Because the stagger is deterministic (based on each board's unique MAC), the
board with the shorter stagger will create the AP first; the other will find it
on the second scan.

---

## Quiet-input detection

The master computes an RMS value for every 4 KB chunk captured from line-in.
If the level stays below `QUIET_THRESHOLD` (≈ −54 dBFS) for `QUIET_WARN_CHUNKS`
consecutive chunks (≈ 0.9 s), it:

- Prints a warning on the Serial monitor:
  ```
  [WARN] Line-in too quiet (RMS=0.00031, threshold=0.00200).  Check source level.
  ```
- Broadcasts a UDP status packet to the slave, which also prints it:
  ```
  [STATUS] Master: line-in too quiet (RMS=0.00031, threshold=0.00200)
  ```

The warning fires once per quiet episode and resets when the level recovers.

---

## Hardware requirements

- 2× ESP32 Audiokit boards (ES8388 codec)
- Audio source connected to **Line-In** on the master board
- Speakers / amplifier connected to **Line-Out** on the slave board

---

## Dependencies

Install via the Arduino Library Manager or add to `platformio.ini`:

| Library | Author | Purpose |
|---|---|---|
| `arduino-audio-tools` | pschatzmann | Core audio stream infrastructure |
| `arduino-audiokit` | pschatzmann | `AudioKitStream` HAL for the ES8388 codec |

---

## Configuration

All tunable constants are in `audio_config.h`:

| Constant | Default | Description |
|---|---|---|
| `SAMPLE_RATE` | 44100 | Sample rate (Hz) |
| `CHUNK_BYTES` | 4096 | UDP payload size (~23 ms per packet) |
| `AP_SSID` | `AudioKit-Master` | Wi-Fi AP name |
| `AP_PASS` | `audiokit1` | Wi-Fi AP password |
| `QUIET_THRESHOLD` | 0.002 | Normalised RMS below which input is "too quiet" |
| `QUIET_WARN_CHUNKS` | 10 | Consecutive quiet chunks before a warning |
| `GAIN_STEP_PER_CHUNK` | 0.02 | Max gain change per chunk (smoothing rate) |
| `JITTER_SLOTS` | 8 | Jitter buffer depth (8 × ~23 ms ≈ 185 ms) |
| `JITTER_PRE_BUFFER` | 3 | Minimum slots before playback starts |
| `JITTER_SKIP_MS` | 60 | Ms to wait for a missing packet before silence |
| `MAC_STAGGER_MS` | 2000 | Max extra delay for boot-collision prevention |

---

## Serial output examples

**Master:**
```
=== ESP32 Audiokit UDP Streaming ===
    AudioTools / AudioKit by Phil Schatzmann
    Automatic master/slave role negotiation

[ROLE] Scanning for existing master AP...
[ROLE] No master found.  Stagger 823 ms (MAC LSB=0xA5)...
[ROLE] Re-scanning after stagger...
[ROLE] No master detected → MASTER
[MASTER] Creating Wi-Fi AP...
[MASTER] AP ready.  IP: 192.168.4.1
[MASTER] Listening for slave.  Audio streaming active.
[WARN] Line-in too quiet (RMS=0.00031, threshold=0.00200).  Check source level.
```

**Slave:**
```
=== ESP32 Audiokit UDP Streaming ===
    AudioTools / AudioKit by Phil Schatzmann
    Automatic master/slave role negotiation

[ROLE] Scanning for existing master AP...
[ROLE] Master AP found → SLAVE
[SLAVE] Connecting to master AP...
..
[SLAVE] Connected.  IP=192.168.4.2  GW=192.168.4.1
[SLAVE] Ready.  Waiting for audio stream...
[SLAVE] Jitter buffer primed (3/8 slots), starting playback
[STATUS] Master: line-in too quiet (RMS=0.00031, threshold=0.00200)
```
