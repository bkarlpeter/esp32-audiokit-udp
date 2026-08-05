testtone off# esp32-audiokit-udp

Low-latency, artefact-free audio streaming between two ESP32 Audiokit boards
(ES8388 codec) over Wi-Fi UDP. One board captures Line-In audio and streams
it to a second board's Line-Out — used to drive two remote speakers in sync.

Flash the **same firmware** to both boards. The role (Master/Slave) is not
auto-negotiated at boot — it is set once per board via a Serial command and
persisted in EEPROM.

---

## Architecture

```
Master  Line-In ──[I2S]──► RMS/quiet check ──► smooth gain ──► 1 KB PCM chunk ──► UDP (unicast) ──►┐
Slave   ◄── UDP ◄── jitter buffer (32 slots) ◄── smooth gain ◄──[I2S]◄── Line-Out                 ◄─┘
```

- **Master**: creates a Wi-Fi Access Point (`AudioKit-Master`), reads Line-In,
  sends 1 KB PCM chunks (~5.8 ms each) via **unicast** UDP to the Slave.
- **Slave**: connects to the Master's AP, sends periodic "hello" packets so
  the Master learns its IP, buffers incoming audio in a jitter ring buffer,
  and plays it out on Line-Out.
- Broadcast delivery was tried first and abandoned — it is unreliable on the
  ESP32 softAP (DTIM-buffered, frequently dropped). All audio/status traffic
  is unicast, point-to-point, once the Slave has registered.

---

## Hardware requirements

- 2× ESP32 Audiokit boards (ES8388 codec)
- Audio source connected to **Line-In** on the Master board
- Speakers/amplifier connected to **Line-Out** on the Slave board

---

## First-time setup

1. Flash the identical firmware (`esp32_audiokit_udp.ino`) to **both** boards.
2. Open a Serial monitor (115200 baud) on each board.
3. Assign roles explicitly (fresh boards default to MASTER):
   ```
   mode master   ← on the board connected to the audio source
   mode slave    ← on the board connected to the speakers
   ```
   Each command reboots the board and persists the role in EEPROM.
4. Power both boards. The Slave connects to the Master's AP automatically;
   watch for `[SLAVE] Connected.` and `[SLAVE] Jitter buffer primed...`.

---

## Serial commands

| Command | Effect |
|---|---|
| `mode master` / `mode slave` | Sets and persists the board's role, then reboots |
| `info` | Prints persisted role, active role, and current latency setting |
| `latency +` / `latency -` | Adjust artificial per-chunk send delay by ±5 ms (Master only) |
| `latency <0-200>` | Set an absolute artificial delay in ms (Master only) |
| `testtone on` / `testtone low` / `testtone off` | Replace Line-In with a generated 440 Hz sine wave (Master only) — `on` = normal level, `low` = quiet level (simulates faint real audio) — for testing the pipeline without a physical audio source |
| `gain <0-100>` | Live-set the Master's ES8388 Mic-PGA input gain (0-100 -> 0-24dB), no reflash needed |
| `rawdbg on` / `rawdbg off` | Master prints raw ADC `peak`/`rms` of the captured buffer every 500 ms — use to spot clipping (peak=32767) or measure the analog noise floor |
| `help` | Lists all commands |

The Slave also prints a periodic one-line summary:
`[STATS] rx=<pkts/s> drops=<n> overflow=<n> driftSkip=<n> rssi=<dBm>`
(`rx`≈172-175/s is healthy at 44.1 kHz/1 KB chunks; `drops`/`overflow` should be 0 in normal operation).

`latency` exists **on purpose** for the planned latency-correction feature
(see Roadmap) and is **not locked** — but see the warning below.

---

## Configuration (`audio_config.h`)

| Constant | Default | Description |
|---|---|---|
| `SAMPLE_RATE` | 44100 | Sample rate (Hz) |
| `CHUNK_BYTES` | 1024 | UDP payload size (~5.8 ms per packet); kept below the WiFi MTU to avoid IP fragmentation |
| `AP_SSID` / `AP_PASS` | `AudioKit-Master` / `audiokit1` | Wi-Fi AP credentials |
| `AUDIO_UDP_PORT` / `STATUS_UDP_PORT` | 12345 / 12346 | UDP ports for audio and status/hello traffic |
| `QUIET_THRESHOLD` | 0.002 | Normalised RMS below which Line-In is "too quiet" (≈ −54 dBFS) |
| `QUIET_WARN_CHUNKS` | 10 | Consecutive quiet chunks before a warning fires |
| `GAIN_STEP_PER_CHUNK` | 0.02 | Max gain change per chunk (ramp avoids hiss/pops/pumping) |
| `JITTER_SLOTS` | 32 | Jitter buffer depth (~185 ms headroom) |
| `JITTER_PRE_BUFFER` | 8 | Minimum filled slots before playback starts |
| `JITTER_SKIP_MS` | 60 | Ms to wait for a missing packet before substituting silence |
| `INPUT_GAIN_PERCENT` | 70 | Master's ES8388 Mic-PGA gain in % (0-100 -> 0-24dB); overridable at runtime via the `gain` Serial command |
| `JITTER_HIGH_WATER` | `JITTER_SLOTS - 6` (26) | Backlog level at which the Slave proactively skips one extra buffered slot to drain clock drift before the buffer overflows |

---

## Diagnostic tooling

Two Python scripts (`pyserial` required) automate hardware-in-the-loop testing
without a manual terminal — useful for both boards at once:

- `serial_monitor.py <seconds>` — reads both `COM3`/`COM4` in parallel, prints
  timestamped lines from each. Update the `PORTS` list if your COM ports differ.
- `send_cmd.py <COM_PORT> "<command>"` — sends a single Serial command to one
  board (e.g. `python send_cmd.py COM4 "latency 0"`) and prints the response.

---

## ⚠️ Do NOT do this (lessons learned the hard way)

- **Do not leave a non-zero `latency` value set "just for a quick test".**
  It is persisted in EEPROM and survives reboots/reflashing. A forgotten
  117 ms test value was the root cause of a session-long "Dropped packet"
  bug that had nothing to do with networking, chunk size, or WiFi power-save.
  Always run `info` after testing and reset with `latency 0` when done.
- **Do not assume COM-port-to-role mapping stays stable.** Which physical
  USB port maps to Master vs. Slave can change across sessions/reconnects.
  Always verify via the boot log (`[ROLE] Persisted role: ...`), never assume.
- **Do not repeatedly reflash the Master while the Slave stays running**
  without expecting a temporary outage. Every Master reboot restarts its
  Wi-Fi AP, which drops the Slave's association. The Slave now auto-detects
  this (`WiFi.status() != WL_CONNECTED`) and reconnects/re-registers within
  ~2 seconds — but older builds without that fix would hang forever with
  zero audio until manually rebooted.
- **Do not chase chunk-size or power-save theories first** when packets are
  dropping. Check `info` for a stale `latency` value and check `slaveKnown`
  registration state before assuming a deeper networking issue — both are
  cheap to rule out and were the actual causes of every drop bug seen so far.
- **Do not use large chunks (>~1.4 KB).** Anything close to/above the WiFi
  MTU risks IP fragmentation; losing one fragment silently drops the whole
  packet with no MAC-layer retry on broadcast/AP traffic.

---

## What is decisive for success

1. **Unicast, not broadcast.** The Slave must register via "hello" packets
   before the Master will ever send audio (`g_slaveKnown` gate). If audio is
   silent, check this first (`slaveKnown`/`seq` were exposed as temporary
   diagnostics in `loopMaster()` — add them back if debugging).
2. **`latency` must be `0` in normal operation.** Confirm with `info` before
   troubleshooting anything else.
3. **The jitter buffer must stay in its "healthy" range** (a few slots out
   of 32, not empty and not maxed out) — visible via the periodic
   `[SLAVE] Buffer depth: x/32 slots` log. Persistent 0/32 means no packets
   are arriving (registration/WiFi problem); persistent near-32/32 means the
   Master is sending faster than the Slave consumes (pacing problem).
4. **WiFi auto-reconnect on the Slave** (`loopSlave()`, checked every 2 s) is
   what makes the system self-healing after a Master restart — do not remove
   this without replacing it with an equivalent recovery mechanism.
5. **Chunk size must stay below the WiFi MTU** (currently 1 KB) to avoid
   silent fragmentation drops.

---

## Signal-quality investigation: the "Wasserfall Rauschen" (waterfall noise) bug

A persistent broadband/static noise at the Slave's Line-Out, with real music
sounding very faint underneath it, took a long diagnostic session to resolve.
Documented here so the same theories aren't re-tested from scratch next time.

### Root cause

**Hardware defect in the ADC / Line-In circuit of one specific board.**
Swapping which physical board acts as Master (`mode master`/`mode slave` +
moving the Line-In/Line-Out cables accordingly) made the waterfall noise
disappear almost completely. It was never a software, gain, or network issue.

➡️ **Fix:** Use the board that tested clean as Master (input) permanently.
Do not use the other board for audio capture until its ADC/Line-In hardware
(solder joints, ES8388 chip, connector) has been inspected/repaired.

A minor residual artifact (light "rippling"/occasional scratches, much less
severe than the original noise) remained after the swap — most likely the
jitter-buffer clock-drift compensation (see below) and not yet fully tuned.

### What was tried and did **NOT** help (ruled out, in this order)

1. **Master-side ES8388 Mic-PGA gain** — tried 24dB (library default), 0dB,
   and 18dB (70%, matching an older working project). All three sounded
   identical; later confirmed quantitatively with `rawdbg`: `peak`/`rms`
   values were statistically the same at `gain 0` and `gain 70` with the
   same input. **The PGA gain register has no measurable effect on the
   noise.**
2. **Source/cable/power supply** — tested with Line-In unplugged, with a
   powerbank instead of USB power, and at different source volumes. No
   change to the noise character.
3. **Digital pipeline / Slave output stage** — verified clean using the
   synthetic test tone (`testtone on` and `testtone low`, i.e. loud and
   quiet synthetic sine waves that bypass the ADC entirely). Both sounded
   completely clean, proving the UDP transport, jitter buffer, Slave gain
   ramp, DAC, and amplifier were **not** the source.
4. **UDP packet loss** — initially found `[SLAVE] Dropped packet` firing
   almost continuously. Added `[STATS] rx/drops/overflow/driftSkip/rssi`
   counters; root cause was a stale sequence-number state after repeated
   Master reflashes during the debugging session itself, not a real
   networking problem. A clean reboot brought `drops`/`overflow` to 0 —
   **packet loss was a red herring specific to the debugging session**, not
   the actual field bug (though the added drift compensation is a genuine
   improvement, see below).
5. **`ADC_INPUT_LINE2` vs `ADC_INPUT_LINE1`** — tried switching the ADC
   input mux, since the codec's `LINE1` selection is treated as the *mic*
   path and `LINE2` as *line-in* per the driver's own semantics
   (`arduino-audio-driver`'s `AudioDriver.h`). Confirmed `LINE2` was already
   the correct choice (switching to `LINE1` went silent). No effect on the
   noise.
6. **Input clipping** — `rawdbg` did reveal genuine full-scale clipping
   (`peak=32767`) at louder passages, even at `gain 0`, meaning the source
   volume itself was too hot for the ADC's input range. Turning the source
   volume down eliminated the clipping — but the waterfall noise remained
   unchanged. **Clipping was a real, separate problem, but not the cause of
   the waterfall noise.**

### What actually helped

- **Swapping Master/Slave roles between the two physical boards** — isolated
  the noise to one board's hardware (see Root cause above).
- **Jitter-buffer clock-drift compensation** (new): the Master's ADC clock
  runs marginally faster than the Slave's DAC clock (a few % — matches a
  historically documented ~1.7% drift on this hardware). Without
  compensation, the jitter buffer backlog grows continuously until it
  overflows every few seconds, causing periodic multi-packet-loss bursts.
  Now, once the backlog exceeds `JITTER_HIGH_WATER` (`JITTER_SLOTS - 6`),
  the Slave proactively drops one extra buffered slot per chunk to drain the
  backlog gradually instead of hitting a hard overflow. This is a real,
  worthwhile fix independent of the hardware-defect finding above.
- **Lowering the source volume to avoid ADC clipping** — a genuine quality
  improvement (removes harsh clipping distortion on loud passages), even
  though it wasn't the cause of the waterfall noise itself.

### Diagnostic technique that made this tractable

- `rawdbg on` on the Master prints raw ADC `peak`/`rms` every 500 ms —
  essential for telling clipping (`peak` pinned at 32767) apart from a
  quiet/noisy signal, and for confirming gain changes actually reach the
  hardware.
- `gain <0-100>` changes the PGA live without a reflash — lets you A/B
  multiple gain settings against the same `rawdbg` output in one session
  instead of a compile/flash cycle per test.
- `testtone low` isolates the Slave/output side from the Master/input side
  using the *same* transport pipeline, without needing a physical signal
  generator.
- Swapping roles/boards outright is a cheap, decisive way to distinguish a
  hardware defect on one board from a systemic software/environmental issue
  — do this **early**, before spending a long session tuning software
  parameters, if a symptom doesn't respond to any config change.

---

## Roadmap

- Web-based control page (planned) to adjust the `latency` value at runtime,
  for correcting audible latency differences between two remote speakers in
  normal operation — this is the intended long-term use of the `latency`
  command, which is why it is kept unlocked rather than removed.
