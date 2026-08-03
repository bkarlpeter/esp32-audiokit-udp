#pragma once
#include <stdint.h>
#include <Wire.h>
#include "config.h"

/**
 * Minimal ES8388 codec driver for the AI-Thinker ESP32-A1S AudioKit.
 *
 * Audio data flows over I2S (ESP32 = I2S master, ES8388 = I2S slave).
 * Codec configuration uses I2C (Wire).
 *
 * Master role: configure ADC  → reads line-in (LIN2/RIN2 jack)
 * Slave  role: configure DAC  → drives line-out / headphone outputs
 */
class ES8388 {
public:
    /**
     * Initialise I2C, soft-reset the chip, and configure common settings.
     * Must be called before configADC() / configDAC().
     *
     * @param sda      I2C SDA pin
     * @param scl      I2C SCL pin
     * @param i2c_addr ES8388 I2C address (default 0x10)
     * @return true if the chip responds on the bus
     */
    bool begin(int sda, int scl, uint8_t i2c_addr = 0x10);

    /**
     * Configure the ADC path (line-in recording) — call on the master board.
     * Routes LIN2/RIN2 (the line-in socket on AudioKit) to the I2S output.
     */
    void configADC();

    /**
     * Configure the DAC path (line-out playback) — call on the slave board.
     * Routes I2S input to LOUT1/ROUT1 (line-out) and LOUT2/ROUT2 (headphone).
     */
    void configDAC();

    /**
     * Set the DAC analogue output volume.
     * @param vol  0 (mute) … 100 (0 dB / maximum)
     */
    void setVolume(uint8_t vol);

    /**
     * Set the ADC PGA input gain.
     * @param gain  0 … 8  (0 dB … +24 dB in 3 dB steps)
     */
    void setInputGain(uint8_t gain);

    /** Return the last volume value set via setVolume(). */
    uint8_t getVolume() const { return _vol; }

private:
    uint8_t _addr = 0x10;
    uint8_t _vol  = DEFAULT_VOLUME;

    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg (uint8_t reg);
};

// Application-wide singleton (defined in es8388.cpp)
extern ES8388 codec;
