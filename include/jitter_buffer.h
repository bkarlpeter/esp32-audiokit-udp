#pragma once
#include <stdint.h>
#include "audio_packet.h"
#include "config.h"

/**
 * Thread-safe circular jitter buffer for the slave board.
 *
 * Latency-killer features
 * ────────────────────────
 * 1. Minimum pre-buffer: playback begins only after JITTER_MIN_PREBUFFER
 *    packets are queued, guaranteeing a small cushion.
 * 2. Overflow flush: when the fill level reaches JITTER_MAX_LEVEL the buffer
 *    is flushed and the pre-buffer phase restarts, preventing latency from
 *    accumulating indefinitely.
 * 3. Gap detection: a jump in sequence numbers indicates lost packets; the
 *    buffer flushes and resynchronises immediately.
 * 4. Explicit flush: the master can request a flush via a control packet
 *    (CMD_FLUSH), or the user can trigger one with the serial 'f' command.
 *
 * Thread safety
 * ─────────────
 * push() and pop() are called from different execution contexts (UDP receive
 * vs. I2S output) and are protected by a FreeRTOS mutex.
 */
class JitterBuffer {
public:
    /** Initialise the buffer; call once from setup(). */
    void init(int target_level = JITTER_TARGET_LEVEL);

    /** Reset all counters and state (does not free the mutex). */
    void reset();

    /**
     * Insert a received audio packet.
     * Returns false if the packet was silently dropped (duplicate, old, or
     * buffer at hard capacity).
     */
    bool push(const AudioPkt& pkt);

    /**
     * Consume the next packet's samples into `output`.
     * `output` must point to AUDIO_SAMPLES_TOTAL int16_t elements.
     * Returns false (and writes silence) when the buffer is empty (underrun).
     */
    bool pop(int16_t* output);

    /**
     * Flush all buffered packets and re-enter the pre-buffer phase.
     * Safe to call from any context.
     */
    void flush();

    // ── Accessors ────────────────────────────────────────────────────────────
    /** Current fill level in packets. */
    int  level() const;

    /** True once the pre-buffer has been satisfied; false after underrun/flush. */
    bool isReady();

    /** Change the desired steady-state fill level (1 … CAPACITY/2). */
    void setTargetLevel(int n);
    int  getTargetLevel() const { return _target; }

    // Lifetime statistics
    uint32_t getRxCount()   const { return _rx_count; }
    uint32_t getDropCount() const { return _drop_count; }
    uint32_t getLossCount() const { return _loss_count; }
    uint32_t getUnderruns() const { return _underruns; }

private:
    static const int CAPACITY = JITTER_BUF_CAPACITY;

    AudioPkt _buf[CAPACITY];
    int      _head     = 0;   // next write slot
    int      _tail     = 0;   // next read slot
    int      _count    = 0;   // packets currently buffered
    bool     _started  = false;
    int      _target   = JITTER_TARGET_LEVEL;

    uint32_t _next_seq  = 0;
    bool     _seq_init  = false;

    uint32_t _rx_count   = 0;
    uint32_t _drop_count = 0;
    uint32_t _loss_count = 0;
    uint32_t _underruns  = 0;

    void* _mutex = nullptr;   // SemaphoreHandle_t stored as void* to avoid
                              // pulling FreeRTOS headers into this header

    void lock();
    void unlock();
};
