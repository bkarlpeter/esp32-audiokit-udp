#include "jitter_buffer.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

// ── Lock helpers ──────────────────────────────────────────────────────────────

void JitterBuffer::lock() {
    if (_mutex) xSemaphoreTake((SemaphoreHandle_t)_mutex, portMAX_DELAY);
}

void JitterBuffer::unlock() {
    if (_mutex) xSemaphoreGive((SemaphoreHandle_t)_mutex);
}

// ── Public API ────────────────────────────────────────────────────────────────

void JitterBuffer::init(int target_level) {
    _mutex  = (void*)xSemaphoreCreateMutex();
    _target = constrain(target_level, 1, CAPACITY / 2);
    reset();
}

void JitterBuffer::reset() {
    lock();
    _head      = 0;
    _tail      = 0;
    _count     = 0;
    _started   = false;
    _seq_init  = false;
    _next_seq  = 0;
    _rx_count  = 0;
    _drop_count= 0;
    _loss_count= 0;
    _underruns = 0;
    unlock();
}

void JitterBuffer::flush() {
    lock();
    _head     = 0;
    _tail     = 0;
    _count    = 0;
    _started  = false;
    // Keep _seq_init / _next_seq so the next incoming packet is accepted
    // even if its sequence number is ahead of the flushed data.
    _seq_init = false;
    unlock();
    Serial.println("[JitterBuf] Flushed — re-buffering");
}

bool JitterBuffer::push(const AudioPkt& pkt) {
    lock();

    ++_rx_count;

    // ── Sequence-gap detection (loss / reorder) ───────────────────────────
    if (_seq_init) {
        if (pkt.hdr.seq > _next_seq) {
            uint32_t lost = pkt.hdr.seq - _next_seq;
            _loss_count  += lost;
            Serial.printf("[JitterBuf] Gap: expected %lu, got %lu (%lu lost) – flushing\n",
                          (unsigned long)_next_seq,
                          (unsigned long)pkt.hdr.seq,
                          (unsigned long)lost);
            // Inline reset while still holding the lock (avoids calling the
            // public flush() which would attempt to re-acquire the mutex).
            _head     = 0;
            _tail     = 0;
            _count    = 0;
            _started  = false;
            _seq_init = false;
        } else if (pkt.hdr.seq < _next_seq) {
            // Old duplicate packet – silently discard
            ++_drop_count;
            unlock();
            return false;
        }
    }
    _next_seq = pkt.hdr.seq + 1;
    _seq_init = true;

    // ── Latency-killer: buffer grown too large → flush and resync ─────────
    if (_count >= JITTER_MAX_LEVEL) {
        Serial.printf("[JitterBuf] Overflow (%d pkts) – killing latency, flushing\n",
                      _count);
        _head     = 0;
        _tail     = 0;
        _count    = 0;
        _started  = false;
        _seq_init = true;   // keep current seq so we don't re-trigger
        // Store the packet we just received as the first slot
    }

    // ── Hard-capacity guard ───────────────────────────────────────────────
    if (_count >= CAPACITY) {
        ++_drop_count;
        unlock();
        return false;
    }

    // ── Store ─────────────────────────────────────────────────────────────
    _buf[_head] = pkt;
    _head = (_head + 1) % CAPACITY;
    ++_count;

    unlock();
    return true;
}

bool JitterBuffer::pop(int16_t* output) {
    lock();

    if (_count == 0) {
        ++_underruns;
        _started = false;   // re-enter pre-buffer phase after underrun
        unlock();
        memset(output, 0, AUDIO_SAMPLES_TOTAL * sizeof(int16_t));
        return false;
    }

    memcpy(output, _buf[_tail].samples, AUDIO_SAMPLES_TOTAL * sizeof(int16_t));
    _tail  = (_tail + 1) % CAPACITY;
    --_count;

    unlock();
    return true;
}

bool JitterBuffer::isReady() {
    lock();
    if (_started) {
        unlock();
        return true;
    }
    if (_count >= JITTER_MIN_PREBUFFER) {
        _started = true;
        int lvl  = _count;
        unlock();
        Serial.printf("[JitterBuf] Pre-buffer satisfied (%d pkts) – starting playback\n",
                      lvl);
        return true;
    }
    unlock();
    return false;
}

int JitterBuffer::level() const {
    return _count;
}

void JitterBuffer::setTargetLevel(int n) {
    _target = constrain(n, 1, CAPACITY / 2);
}
