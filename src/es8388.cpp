#include "es8388.h"
#include <Arduino.h>

ES8388 codec;

// ── ES8388 register addresses ─────────────────────────────────────────────────
static const uint8_t R_CONTROL1      = 0x00;
static const uint8_t R_CONTROL2      = 0x01;
static const uint8_t R_CHIPPOWER     = 0x02;
static const uint8_t R_ADCPOWER      = 0x03;
static const uint8_t R_DACPOWER      = 0x04;
static const uint8_t R_ANAVOLMANAGE  = 0x07;
static const uint8_t R_MASTERMODE    = 0x08;
static const uint8_t R_ADCCONTROL1   = 0x09;  // PGA gain
static const uint8_t R_ADCCONTROL2   = 0x0A;  // input select
static const uint8_t R_ADCCONTROL3   = 0x0B;
static const uint8_t R_ADCCONTROL4   = 0x0C;  // ADC I2S format
static const uint8_t R_ADCCONTROL5   = 0x0D;  // over-sampling ratio
static const uint8_t R_ADCCONTROL6   = 0x0E;  // high-pass filter
static const uint8_t R_ADCCONTROL10  = 0x12;  // ALC
static const uint8_t R_ADCCONTROL11  = 0x13;
static const uint8_t R_ADCCONTROL12  = 0x14;
static const uint8_t R_ADCCONTROL13  = 0x15;
static const uint8_t R_ADCCONTROL14  = 0x16;
static const uint8_t R_DACCONTROL1   = 0x17;  // DAC I2S format
static const uint8_t R_DACCONTROL2   = 0x18;
static const uint8_t R_DACCONTROL3   = 0x19;  // soft-mute / soft-ramp
static const uint8_t R_DACCONTROL4   = 0x1A;  // LDACVOL (digital)
static const uint8_t R_DACCONTROL5   = 0x1B;  // RDACVOL (digital)
static const uint8_t R_DACCONTROL8   = 0x1E;  // LOUT1VOL (analogue)
static const uint8_t R_DACCONTROL9   = 0x1F;  // ROUT1VOL
static const uint8_t R_DACCONTROL10  = 0x20;  // LOUT2VOL
static const uint8_t R_DACCONTROL11  = 0x21;  // ROUT2VOL
static const uint8_t R_DACCONTROL16  = 0x26;  // DAC mixer input select
static const uint8_t R_DACCONTROL17  = 0x27;  // LOUT1 mixer
static const uint8_t R_DACCONTROL18  = 0x28;  // LOUT2 mixer
static const uint8_t R_DACCONTROL20  = 0x2A;  // ROUT1 mixer
static const uint8_t R_DACCONTROL21  = 0x2B;  // ROUT2 mixer

// ── I2C helpers ───────────────────────────────────────────────────────────────

void ES8388::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t ES8388::readReg(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_addr, 1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool ES8388::begin(int sda, int scl, uint8_t i2c_addr) {
    _addr = i2c_addr;
    Wire.begin(sda, scl);

    // Soft-reset
    writeReg(R_CONTROL1, 0x80);
    delay(20);
    writeReg(R_CONTROL1, 0x00);

    // Power down all blocks while configuring
    writeReg(R_CHIPPOWER, 0xFF);
    writeReg(R_ADCPOWER,  0xFF);
    writeReg(R_DACPOWER,  0xC0);

    // Reference / bias voltage
    writeReg(R_CONTROL1,    0x05);  // VMIDSEL=01 (500 Ω divider)
    writeReg(R_CONTROL2,    0x40);  // ENVCM: enable common-mode reference
    writeReg(R_ANAVOLMANAGE, 0x7B); // default analogue management

    // I2S slave mode: all clocks (MCLK/BCLK/LRCK) come from ESP32
    writeReg(R_MASTERMODE, 0x00);

    // Verify presence
    return (readReg(R_CONTROL1) != 0xFF);
}

void ES8388::configADC() {
    // Input selection: LIN2 / RIN2 (the line-in socket on AudioKit)
    // ADCCONTROL2 bits [7:6]=LINSEL, [5:4]=RINSEL  01 = LIN2/RIN2
    writeReg(R_ADCCONTROL2, 0x50);  // LINSEL=01, RINSEL=01

    // PGA: 0 dB initial gain
    writeReg(R_ADCCONTROL1, 0x00);

    // ADC I2S format: standard I2S, 16-bit word length
    // [7:6]=00 (I2S), [5:3]=010 (16-bit), [2:0]=000
    writeReg(R_ADCCONTROL4, 0x0C);

    // ADC filter / mode
    writeReg(R_ADCCONTROL3, 0x02);  // DS_OFF
    writeReg(R_ADCCONTROL5, 0x02);  // 256 Fs over-sampling
    writeReg(R_ADCCONTROL6, 0x35);  // high-pass filter on

    // Disable ALC (automatic level control)
    writeReg(R_ADCCONTROL10, 0x38);
    writeReg(R_ADCCONTROL11, 0xB0);
    writeReg(R_ADCCONTROL12, 0x32);
    writeReg(R_ADCCONTROL13, 0x06);
    writeReg(R_ADCCONTROL14, 0xD2);

    // Power up ADC
    writeReg(R_ADCPOWER,  0x00);  // ADCL + ADCR on
    writeReg(R_CHIPPOWER, 0x00);  // normal operation

    Serial.println("[ES8388] ADC (line-in) configured");
}

void ES8388::configDAC() {
    // DAC I2S format: standard I2S, 16-bit
    // [7:6]=00 (I2S), [5:3]=011 (16-bit), [2:0]=000
    writeReg(R_DACCONTROL1, 0x18);

    // Soft-ramp and zero-crossing enabled; no soft-mute
    writeReg(R_DACCONTROL3, 0x09);

    // Digital volume: 0 dB (loudness is controlled by the analogue regs below)
    writeReg(R_DACCONTROL4, 0x00);  // LDACVOL = 0 dB
    writeReg(R_DACCONTROL5, 0x00);  // RDACVOL = 0 dB

    // Mixer: route DACL → LOUT1, DACR → ROUT1 (and LOUT2/ROUT2 = headphone)
    writeReg(R_DACCONTROL16, 0x00);  // DACL/R input to mixer
    writeReg(R_DACCONTROL17, 0x90);  // LOUT1: enable, source = DACL
    writeReg(R_DACCONTROL18, 0x90);  // LOUT2: enable, source = DACL
    writeReg(R_DACCONTROL20, 0x90);  // ROUT1: enable, source = DACR
    writeReg(R_DACCONTROL21, 0x90);  // ROUT2: enable, source = DACR

    // Set initial analogue output volume
    setVolume(DEFAULT_VOLUME);

    // Power up DAC and all four output stages
    writeReg(R_DACPOWER,  0x3C);  // DACL + DACR + LOUT1 + ROUT1 + LOUT2 + ROUT2
    writeReg(R_CHIPPOWER, 0x00);  // normal operation

    Serial.println("[ES8388] DAC (line-out) configured");
}

void ES8388::setVolume(uint8_t vol) {
    if (vol > 100) vol = 100;
    _vol = vol;

    // Analogue output volume registers are 6-bit [5:0].
    // 0x00 = mute / minimum, 0x21 (33) = 0 dB maximum.
    uint8_t reg_val = (uint8_t)((vol * 33U) / 100U);

    writeReg(R_DACCONTROL8,  reg_val);  // LOUT1VOL
    writeReg(R_DACCONTROL9,  reg_val);  // ROUT1VOL
    writeReg(R_DACCONTROL10, reg_val);  // LOUT2VOL  (headphone)
    writeReg(R_DACCONTROL11, reg_val);  // ROUT2VOL
}

void ES8388::setInputGain(uint8_t gain) {
    if (gain > 8) gain = 8;
    // ADCCONTROL1 [7:4] = PGAGAINL, [3:0] = PGAGAINR  (3 dB / step)
    uint8_t g = gain & 0x0F;
    writeReg(R_ADCCONTROL1, (g << 4) | g);
}
