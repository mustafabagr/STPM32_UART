// STPM32_UART.h - UART driver for the ST STPM32 single-phase metering IC (ESP32 / Arduino).
//
// Scope: register I/O, scaling to physical units, configuration persisted in NVS,
// sag/swell/overcurrent protection and INT1 status handling.
// Application policy (anti-creep, power factor, energy accumulation) is intentionally
// left to the caller.
//
// SPDX-License-Identifier: MIT

#ifndef STPM32_UART_H
#define STPM32_UART_H

#if !defined(ARDUINO_ARCH_ESP32)
#error "STPM32_UART requires the ESP32 Arduino core (FreeRTOS, Preferences, HardwareSerial::updateBaudRate)."
#endif

#include <Arduino.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ------------------- CONSTANTS -------------------
#define STPM_FRAME_LEN        5
#define STPM_TIMEOUT_MS       50
#define STPM_UART_TIMEOUT_MS  20    // STPM-side frame timeout (ms, 0 = off). One frame at 9600 baud is ~5.2 ms
#define STPM_LINK_QUIET_MS    30    // Silence between baud probes (must be > STPM_UART_TIMEOUT_MS)
#define STPM_V_REF            1.18f // STPM32 internal voltage reference (datasheet Table 12)
#define STPM_CFG_VERSION      1     // NVS configuration blob version (bump when STPM32_BootConfig changes)

// Default scale factors, measured on the reference hardware
// (810 kOhm voltage divider, 500 uOhm shunt, current gain x16).
// Your board will differ: call setScale() before begin() and/or calibrate with setVoltageTrim()/setCurrentTrim().
constexpr float STPM_DEFAULT_V_LSB     = 0.031049f;  // V per LSB of the V1 RMS register
constexpr float STPM_DEFAULT_I_LSB_X16 = 0.001341f;  // A per LSB of the C1 RMS register at gain x16

// ==================== REGISTER MAP ====================

union DSP_CR1_Register {
    static constexpr uint8_t ADDRESS = 0x00;
    uint32_t all; // Full 32-bit value written to the chip
    struct {
        uint32_t CLRSS_TO1 : 4;  // Bits 0-3   Duration of the reset signal that clears the primary sag/swell registers
        uint32_t ClearSS1 : 1;   // Bit 4      Clears primary sag/swell time registers and history bits. Auto-resets to 0.
        uint32_t ENVREF1 : 1;    // Bit 5      Internal voltage reference for primary channel (1 = enabled, default). Keep 1 unless an external VREF is fitted.
        uint32_t TC1 : 3;        // Bits 6-8   Temperature compensation coefficient for VREF1 (default 0x2)
        uint32_t reserved1 : 8;  // Bits 9-16
        uint32_t AEM1 : 1;       // Bit 17     Apparent energy mode: 0 = apparent RMS power, 1 = apparent vectorial power
        uint32_t APM1 : 1;       // Bit 18     Apparent vectorial power mode: 0 = fundamental, 1 = active power
        uint32_t BHPFV1 : 1;     // Bit 19     Bypass HPF on primary voltage channel (0 = filter enabled; keep 0 to remove DC offset)
        uint32_t BHPFC1 : 1;     // Bit 20     Bypass HPF on primary current channel (0 = filter enabled)
        uint32_t ROC1 : 1;       // Bit 21     Rogowski coil integrator (1 = enabled). Must be 0 for shunt or CT sensors.
        uint32_t reserved2 : 2;  // Bits 22-23
        uint32_t LPW1 : 4;       // Bits 24-27 LED1 pulse rate divider (default 0x4)
        uint32_t LPS1 : 2;       // Bits 28-29 LED1 power source: 00 active, 01 fundamental, 10 reactive, 11 apparent
        uint32_t LCS1 : 2;       // Bits 30-31 LED1 channel: 00 primary, 01 secondary, 10 cumulative, 11 sigma-delta bitstream
    } bit;
};
union DSP_CR3_Register {
    static constexpr uint8_t ADDRESS = 0x04;
    uint32_t all;
    struct {
        uint32_t TIME_VALUE : 14;  // Bits 0-13  Sag time threshold
        uint32_t ZCR_SEL : 2;      // Bits 14-15 Zero-crossing source (00: V1, 01: C1, ...)
        uint32_t ZCR_EN : 1;       // Bit 16     1 = ZCR output, 0 = CLK output
        uint32_t TMP_TOL : 2;      // Bits 17-18 Tamper tolerance
        uint32_t TMP_EN : 1;       // Bit 19     Tamper enable
        uint32_t SW_Reset : 1;     // Bit 20     Software reset
        uint32_t SW_Latch1 : 1;    // Bit 21     Manual latch, channel 1
        uint32_t SW_Latch2 : 1;    // Bit 22     Manual latch, channel 2
        uint32_t SW_AutoLatch : 1; // Bit 23     1 = automatic latch at 7.8 kHz
        uint32_t LED1_OFF : 1;     // Bit 24
        uint32_t LED2_OFF : 1;     // Bit 25
        uint32_t EN_CUM : 1;       // Bit 26     Cumulative calculation method
        uint32_t REF_FREQ : 1;     // Bit 27     0 = 50 Hz, 1 = 60 Hz
        uint32_t reserved : 4;     // Bits 28-31
    } bit;
};
union DSP_CR4_Register {
    static constexpr uint8_t ADDRESS = 0x06;
    uint32_t all;
    struct {
        uint32_t PHC2 : 10;     // Bits 0-9   Channel 2 current phase compensation
        uint32_t PHV2 : 2;      // Bits 10-11 Channel 2 voltage phase compensation
        uint32_t PHC1 : 10;     // Bits 12-21 Channel 1 current phase compensation (e.g. for CTs)
        uint32_t PHV1 : 2;      // Bits 22-23 Channel 1 voltage phase compensation
        uint32_t reserved : 8;  // Bits 24-31
    } bit;
};
union DSP_CR5_Register {
    static constexpr uint8_t ADDRESS = 0x08;
    uint32_t all;
    struct {
        uint32_t CHV1 : 12;     // Bits 0-11  Voltage calibration (default 0x800)
        uint32_t SWV_THR1 : 10; // Bits 12-21 Voltage swell threshold (0x3FF = disabled)
        uint32_t SAG_THR1 : 10; // Bits 22-31 Voltage sag threshold (0 = disabled)
    } bit;
};
union DSP_CR6_Register {
    static constexpr uint8_t ADDRESS = 0x0A;
    uint32_t all; // Default: 0x003FF800
    struct {
        uint32_t CHC1 : 12;     // Bits 0-11  Current calibration (default 0x800)
        uint32_t SWC_THR1 : 10; // Bits 12-21 Current swell (overcurrent) threshold (0x3FF = disabled)
        uint32_t reserved : 10; // Bits 22-31 Must be 0
    } bit;
};
union DSP_CR9_Register {
    static constexpr uint8_t ADDRESS = 0x10;
    uint32_t all; // Default: 0x00000FFF
    struct {
        uint32_t AH_UP1 : 12; // Bits 0-11
        uint32_t OFA1 : 10;   // Bits 12-21 Active power offset
        uint32_t OFAF1 : 10;  // Bits 22-31 Fundamental active power offset
    } bit;
};
union DSP_CR10_Register {
    static constexpr uint8_t ADDRESS = 0x12;
    uint32_t all; // Default: 0x00000FFF
    struct {
        uint32_t AH_DOWN1 : 12; // Bits 0-11
        uint32_t OFR1 : 10;     // Bits 12-21 Reactive power offset
        uint32_t OFS1 : 10;     // Bits 22-31 Apparent power offset
    } bit;
};
union DFE_CR1_Register {
    static constexpr uint8_t ADDRESS = 0x18;
    uint32_t all; // Default: 0x0F270327
    struct {
        uint32_t enV1 : 1;       // Bit 0      Voltage channel enable
        uint32_t reserved1 : 15; // Bits 1-15  Do not modify
        uint32_t enC1 : 1;       // Bit 16     Current channel enable
        uint32_t reserved2 : 9;  // Bits 17-25 Do not modify
        uint32_t GAIN1 : 2;      // Bits 26-27 Current gain (0:x2 1:x4 2:x8 3:x16, default x16)
        uint32_t reserved3 : 4;  // Bits 28-31
    } bit;
};
// DSP_IRQ1 (mask) and DSP_SR1 (status) share the same bit layout (datasheet Table 55/57).
// Mask bit = 1 means the event is routed to the INT1 pin (1 = enabled, 0 = masked).
union DSP_IRQ1_Register {
    static constexpr uint8_t ADDRESS = 0x1C;
    uint32_t all;

    struct {
        uint32_t power_sign_flags : 8;     // Bits 0-7   Total + PH2 power signs / overflows
        uint32_t ph2_irq_flags : 4;        // Bits 8-11
        uint32_t ph1_power_irq : 4;        // Bits 12-15 PH1 power signs (level-triggered!)
        uint32_t ph1_energy_overflow : 4;  // Bits 16-19

        // --- C1 (current) ---
        uint32_t c1_stuck : 1;             // Bit 20
        uint32_t ah1_accumulation : 1;     // Bit 21  Ah accumulation (NOT overcurrent)
        uint32_t c1_swell_detected : 1;    // Bit 22  Overcurrent started
        uint32_t c1_swell_end : 1;         // Bit 23  Overcurrent ended

        // --- V1 (voltage) ---
        uint32_t v1_stuck : 1;             // Bit 24
        uint32_t v1_period_err : 1;        // Bit 25
        uint32_t v1_sag_detected : 1;      // Bit 26  Sag started
        uint32_t v1_sag_end : 1;           // Bit 27
        uint32_t v1_swell_detected : 1;    // Bit 28  Swell started
        uint32_t v1_swell_end : 1;         // Bit 29
        uint32_t tamper_detected : 1;      // Bit 30  Not available on STPM32 (STPM33/34 only)
        uint32_t tamper_end : 1;           // Bit 31
    } bit;
};

// Common bit masks for the IRQ mask and the SR1 status register
namespace STPM_IRQ {
    constexpr uint32_t C1_STUCK = 1UL << 20;
    constexpr uint32_t AH1 = 1UL << 21;
    constexpr uint32_t C1_SWELL = 1UL << 22;
    constexpr uint32_t C1_SWELL_END = 1UL << 23;
    constexpr uint32_t V1_STUCK = 1UL << 24;
    constexpr uint32_t V1_PERIOD_ERR = 1UL << 25;
    constexpr uint32_t V1_SAG = 1UL << 26;
    constexpr uint32_t V1_SAG_END = 1UL << 27;
    constexpr uint32_t V1_SWELL = 1UL << 28;
    constexpr uint32_t V1_SWELL_END = 1UL << 29;

    constexpr uint32_t VOLTAGE_EVENTS = V1_SAG | V1_SAG_END | V1_SWELL | V1_SWELL_END;
    constexpr uint32_t CURRENT_EVENTS = C1_SWELL | C1_SWELL_END;
    // All status bits handled by this driver (contiguous bits 20-29)
    constexpr uint32_t ALL_ALARMS = C1_STUCK | AH1 | CURRENT_EVENTS | V1_STUCK | V1_PERIOD_ERR | VOLTAGE_EVENTS;
}
union DSP_SR1_Register {
    static constexpr uint8_t ADDRESS = 0x20;
    uint32_t all;

    struct {
        uint32_t power_signs : 16;       // Bits 0-15  Power signs, overflows
        uint32_t energy_overflows : 4;   // Bits 16-19 Energy overflows
        uint32_t c1_stuck : 1;           // Bit 20     Current ADC stuck
        uint32_t ah1_accumulation : 1;   // Bit 21
        uint32_t c1_swell_detected : 1;  // Bit 22     Overcurrent
        uint32_t c1_swell_end : 1;       // Bit 23
        uint32_t v1_stuck : 1;           // Bit 24     Voltage ADC stuck
        uint32_t v1_period_err : 1;      // Bit 25     Line period / frequency error
        uint32_t v1_sag_detected : 1;    // Bit 26     Voltage sag
        uint32_t v1_sag_end : 1;         // Bit 27
        uint32_t v1_swell_detected : 1;  // Bit 28     Voltage swell
        uint32_t v1_swell_end : 1;       // Bit 29
        uint32_t tamper_primary : 1;     // Bit 30
        uint32_t tamper_wrong_conn : 1;  // Bit 31
    } bit;
};
union DSP_EV1_Register {
    static constexpr uint8_t ADDRESS = 0x2A; // Live event register (not latched like SR1, updated at 7.8 kHz)
    uint32_t all;

    // Datasheet Table 20. The 4-bit *_ev fields are an event HISTORY: LSB = 1 -> event active now;
    // when the event ends the field shifts left (LSB = 0); a new event shifts again and sets LSB = 1.
    struct {
        uint32_t total_flags : 4;      // Bits 0-3   PH1+PH2 total power sign / overflow
        uint32_t ph1_sign : 4;         // Bits 4-7   Active, fundamental, reactive, apparent power sign
        uint32_t ph1_overflow : 4;     // Bits 8-11  Energy overflows
        uint32_t c1_zcr : 1;           // Bit 12     Current zero crossing
        uint32_t c1_stuck : 1;         // Bit 13
        uint32_t c1_ah : 1;            // Bit 14
        uint32_t swc1_ev : 4;          // Bits 15-18 Current swell history
        uint32_t v1_zcr : 1;           // Bit 19     Voltage zero crossing
        uint32_t v1_stuck : 1;         // Bit 20
        uint32_t v1_period_err : 1;    // Bit 21
        uint32_t swv1_ev : 4;          // Bits 22-25 Voltage swell history
        uint32_t sag1_ev : 4;          // Bits 26-29 Voltage sag history
        uint32_t reserved : 2;         // Bits 30-31
    } bit;
};
union DSP_REG1_Register {
    static constexpr uint8_t ADDRESS = 0x2E;
    uint32_t all;

    struct {
        uint32_t ph1_period : 12; // Bits 0-11  Line period (8 us per LSB)
        uint32_t padding : 20;    // Bits 12-31
    } bit;
};
union DSP_REG2_Register {
    static constexpr uint8_t ADDRESS = 0x30; // Voltage waveform sample
    uint32_t all;

    struct {
        uint32_t v1_data : 24; // Bits 0-23
        uint32_t padding : 8;  // Bits 24-31
    } bit;
};
union DSP_REG3_Register {
    static constexpr uint8_t ADDRESS = 0x32; // Current waveform sample
    uint32_t all;

    struct {
        uint32_t c1_data : 24; // Bits 0-23
        uint32_t padding : 8;  // Bits 24-31
    } bit;
};
union DSP_REG6_Register {
    static constexpr uint8_t ADDRESS = 0x38; // Fundamental voltage sample
    uint32_t all;

    struct {
        uint32_t v1_fund : 24; // Bits 0-23
        uint32_t padding : 8;  // Bits 24-31
    } bit;
};
union DSP_REG7_Register {
    static constexpr uint8_t ADDRESS = 0x3A; // Fundamental current sample
    uint32_t all;

    struct {
        uint32_t c1_fund : 24; // Bits 0-23
        uint32_t padding : 8;  // Bits 24-31
    } bit;
};
union DSP_REG14_Register {
    static constexpr uint8_t ADDRESS = 0x48; // RMS data
    uint32_t all;

    struct {
        uint32_t v1_rms_data : 15; // Bits 0-14  Voltage RMS (raw)
        uint32_t c1_rms_data : 17; // Bits 15-31 Current RMS (raw)
    } bit;
};
union DSP_REG16_Register {
    static constexpr uint8_t ADDRESS = 0x4C;
    uint32_t all;
    struct {
        uint32_t swv1_time : 15; // Voltage swell duration
        uint32_t padding1 : 1;
        uint32_t sag1_time : 15; // Voltage sag duration
        uint32_t padding2 : 1;
    } bit;
};
union DSP_REG17_Register {
    static constexpr uint8_t ADDRESS = 0x4E;
    uint32_t all;
    struct {
        uint32_t swc1_time : 15; // Current swell duration
        uint32_t padding1 : 1;
        uint32_t c1_pha : 12;    // Phase angle between V and I
        uint32_t padding2 : 4;
    } bit;
};
union PH1_REG1_Register {
    static constexpr uint8_t ADDRESS = 0x54; // PH1 active energy
    uint32_t all;
    struct { uint32_t active_energy : 32; } bit;
};
union PH1_REG2_Register {
    static constexpr uint8_t ADDRESS = 0x56; // PH1 fundamental energy
    uint32_t all;
    struct { uint32_t fundamental_energy : 32; } bit;
};
union PH1_REG3_Register {
    static constexpr uint8_t ADDRESS = 0x58; // PH1 reactive energy
    uint32_t all;
    struct { uint32_t reactive_energy : 32; } bit;
};
union PH1_REG4_Register {
    static constexpr uint8_t ADDRESS = 0x5A; // PH1 apparent energy
    uint32_t all;
    struct { uint32_t apparent_energy : 32; } bit;
};
union PH1_REG12_Register {
    static constexpr uint8_t ADDRESS = 0x6A; // PH1 ampere-hour accumulator
    uint32_t all;
    struct { uint32_t ah_acc : 32; } bit;
};
union PH1_REG5_Register {
    static constexpr uint8_t ADDRESS = 0x5C; // PH1 active power
    uint32_t all;
    struct {
        uint32_t active_power : 29; // Bits 0-28 (signed)
        uint32_t padding : 3;       // Bits 29-31
    } bit;
};
union PH1_REG6_Register {
    static constexpr uint8_t ADDRESS = 0x5E; // PH1 fundamental active power
    uint32_t all;
    struct {
        uint32_t fundamental_power : 29;
        uint32_t padding : 3;
    } bit;
};
union PH1_REG7_Register {
    static constexpr uint8_t ADDRESS = 0x60; // PH1 reactive power
    uint32_t all;
    struct {
        uint32_t reactive_power : 29;
        uint32_t padding : 3;
    } bit;
};
union PH1_REG8_Register {
    static constexpr uint8_t ADDRESS = 0x62; // PH1 apparent RMS power
    uint32_t all;
    struct {
        uint32_t apparent_rms_power : 29;
        uint32_t padding : 3;
    } bit;
};
union PH1_REG9_Register {
    static constexpr uint8_t ADDRESS = 0x64; // PH1 apparent vectorial power
    uint32_t all;
    struct {
        uint32_t apparent_vectorial_power : 29;
        uint32_t padding : 3;
    } bit;
};
union PH1_REG10_Register {
    static constexpr uint8_t ADDRESS = 0x66; // PH1 momentary active power
    uint32_t all;
    struct {
        uint32_t momentary_active_power : 29;
        uint32_t padding : 3;
    } bit;
};
union PH1_REG11_Register {
    static constexpr uint8_t ADDRESS = 0x68; // PH1 momentary fundamental power
    uint32_t all;
    struct {
        uint32_t momentary_fundamental_power : 29;
        uint32_t padding : 3;
    } bit;
};
union TOT_REG1_Register {
    static constexpr uint8_t ADDRESS = 0x84; // Total active energy
    uint32_t all;
    struct { uint32_t total_active_energy : 32; } bit;
};
union TOT_REG2_Register {
    static constexpr uint8_t ADDRESS = 0x86; // Total fundamental energy
    uint32_t all;
    struct { uint32_t total_fundamental_energy : 32; } bit;
};
union TOT_REG3_Register {
    static constexpr uint8_t ADDRESS = 0x88; // Total reactive energy
    uint32_t all;
    struct { uint32_t total_reactive_energy : 32; } bit;
};
union TOT_REG4_Register {
    static constexpr uint8_t ADDRESS = 0x8A; // Total apparent energy
    uint32_t all;
    struct { uint32_t total_apparent_energy : 32; } bit;
};


// Configuration block kept in NVS (flash) and written to the chip at every boot.
// Defaults: ALL protection and interrupt sources are off (SAG_THR = 0, SWV/SWC_THR = 0x3FF, IRQ mask = 0).
// Enable them with setVoltageProtection() / setCurrentProtection().
struct STPM32_BootConfig {
    uint32_t version = STPM_CFG_VERSION;
    uint32_t cr1_0x00 = 0x040000A0;  // ENVREF1=1, TC1=2, LPW1=4, HPFs enabled, ROC1=0 (shunt/CT)
    uint32_t cr3_0x04 = 0x008104E0;  // TIME_VALUE=0x4E0 (~10 ms), ZCR_EN, S/W auto-latch (7.8 kHz)
    uint32_t cr4_0x06 = 0x00000000;  // No phase compensation
    uint32_t cr5_0x08 = 0x003FF800;  // CHV1=0x800, SWV_THR1=0x3FF (swell off), SAG_THR1=0 (sag off)
    uint32_t cr6_0x0A = 0x003FF800;  // CHC1=0x800, SWC_THR1=0x3FF (overcurrent off)
    uint32_t cr9_0x10 = 0x00000FFF;  // Active power offset / AH_UP
    uint32_t cr10_0x12 = 0x00000FFF; // Reactive power offset / AH_DOWN
    uint32_t dfe1_0x18 = 0x0F270327; // GAIN1=x16, voltage and current channels enabled
    uint32_t irq1_0x1C = 0x00000000; // INT1 mask (1 = enabled). Default: nothing routed to INT1
};

// Raw measurement set read under a single lock (physical units, no anti-creep applied)
struct StpmSnapshot {
    float vRms = 0;               // V
    float iRms = 0;               // A
    float pActive = 0;            // W   wideband active power (PH1_REG5)
    float pFund = 0;              // W   fundamental active power (PH1_REG6)
    float qReactive = 0;          // var reactive power (PH1_REG7, signed)
    float sApparent = 0;          // VA  apparent power (PH1_REG8)
    uint32_t energyRaw = 0;       // PH1_REG1 active energy counter (mod 2^32, counts up/down)
    uint32_t reactEnergyRaw = 0;  // PH1_REG3 reactive energy counter (+ inductive, - capacitive)
    uint32_t appEnergyRaw = 0;    // PH1_REG4 apparent energy counter
    uint32_t rawRms = 0;          // DSP_REG14 raw (diagnostics)
    float lineHz = 0;             // Line frequency from DSP_REG1 period
};

class STPMClass {
public:
    explicit STPMClass(HardwareSerial& serial) : _serial(serial) {}

    // Sets the scale factors for your hardware. Call BEFORE begin().
    //   vLsb         : volts per LSB of the V1 RMS register
    //   iLsbAtGain16 : amperes per LSB of the C1 RMS register at current gain x16
    // Power LSB is derived as 16 * vLsb * iLsb (datasheet). Thresholds stored in NVS are
    // encoded with these factors, so change them only before begin().
    void setScale(float vLsb, float iLsbAtGain16);

    // Starts UART, NVS and the configuration; creates the internal mutex.
    // The chip starts at 9600 baud; the current rate (9600 or targetBaud) is detected and
    // switched to targetBaud if needed. nvsNamespace: Preferences namespace (max 15 chars),
    // use a different one per instance if you drive more than one chip.
    bool begin(int rxPin, int txPin, uint32_t targetBaud = 9600, const char* nvsNamespace = "stpm_cfg");
    uint8_t recoverLink();                      // 0: failed, 1: OK, 2: OK + configuration re-applied
    uint32_t baud() const { return _baud; }
    bool linkOk() const { return _linkOk; }

    // --- SETTINGS (persisted to NVS) ---
    bool setCurrentGain(uint8_t gainCode);      // 0:x2 1:x4 2:x8 3:x16
    bool setLEDPulseRate(uint8_t lpw_code);     // 0..15
    bool setTamperProtection(bool enable);      // STPM32 has no tamper module -> always false
    bool setSagSwellThresholds(float sagVolts, float swellVolts);   // thresholds only, IRQ mask unchanged
    // Calibration: 'measured' must be a RAW value (no anti-creep) read with the current CHV/CHC setting
    bool setVoltageTrim(float expectedVoltage, float measuredVoltage);
    bool setCurrentTrim(float expectedCurrent, float measuredCurrent);

    // --- MEASUREMENT (raw physical values, no policy) ---
    bool readSnapshot(StpmSnapshot& out);        // all measurements under one lock
    bool readVoltageAndCurrent(float& voltageOut, float& currentOut);
    bool readActivePower(float& powerOut);
    bool readApparentPower(float& powerOut);
    double energyLsbWh() const;                  // Wh per count of PH1_REG1

    // --- LOW-LEVEL READ/WRITE (thread-safe) ---
    bool readRegister32(uint8_t addrLSB, uint32_t* valueOut);
    bool writeRegister32(uint8_t addrLSB, uint32_t value);

    // --- PROTECTION ---
    bool setVoltageProtection(bool enable, float sagVolts, float swellVolts);
    bool setCurrentProtection(bool enable, float maxAmps);
    void getProtectionSettings(bool& v_en, float& sagVolts, float& swellVolts, bool& c_en, float& maxAmps);

    // --- INTERRUPT / STATUS ---
    bool readStatus(uint32_t& sr1);                 // DSP_SR1 (latched status)
    bool clearStatus(uint32_t mask);                // Clears the given bits in SR1
    bool readLiveEvents(uint32_t& ev1);             // DSP_EV1 (live)
    bool clearSagSwellHistory();                    // DSP_CR1.ClearSS1
    uint32_t irqMask();

private:
    HardwareSerial& _serial;
    Preferences _preferences;
    STPM32_BootConfig _activeConfig;
    SemaphoreHandle_t _mutex = nullptr;   // recursive mutex: UART + _activeConfig + NVS

    float _vLsbBase = STPM_DEFAULT_V_LSB;
    float _iLsbBase16 = STPM_DEFAULT_I_LSB_X16;
    float _v_lsb = 1;   // V / LSB (RMS register)
    float _i_lsb = 1;   // A / LSB (RMS register), scaled with current gain
    float _p_lsb = 1;   // W / LSB (power registers)
    float _current_gain = 16.0f;

    // Low-level UART / CRC
    uint8_t byteReverse(uint8_t n);
    uint8_t crc8CalcRaw(const uint8_t* buf, uint8_t len);
    uint8_t crc8ForUart(const uint8_t* buf4);
    bool stpmTransfer(uint8_t readAddr, uint8_t writeAddr, uint8_t dataLSB, uint8_t dataMSB, uint8_t respOut[4]);
    bool writeRegister16(uint8_t addr, uint16_t value);
    bool readPower29(uint8_t addr, float& out);

    // UART link
    uint32_t _baud = 0;          // current working rate (0 = no link)
    uint32_t _targetBaud = 9600;
    volatile bool _linkOk = false;
    static uint16_t baudDivisor(uint32_t baud);
    void drainRx(uint32_t quietMs);
    bool probeBaud(uint32_t baud);
    bool establishLink(uint32_t targetBaud);
    bool applyConfig();
    bool configMatches();

    // Configuration helpers
    void loadConfig();
    bool persist();
    bool verifyRegister(uint8_t addr, uint32_t expected);
    void updateScaling();
    uint32_t encodeSagThr(float vRms) const;
    uint32_t encodeSwellVThr(float vRms) const;
    uint32_t encodeSwcThr(float aRms) const;
    float decodeVThr(uint32_t code) const;
    float decodeSwcThr(uint32_t code) const;
};

#endif // STPM32_UART_H
