// STPM32_UART.cpp - UART driver for the ST STPM32 single-phase metering IC (ESP32 / Arduino).
// SPDX-License-Identifier: MIT

#include "STPM32_UART.h"
#include <math.h>

namespace {

    constexpr float SQRT2 = 1.41421356f;

    // RAII lock for the recursive mutex. The UART link, _activeConfig and NVS may be used from
    // several tasks (loop, network handlers, an alarm task...), so every public method locks.
    class StpmLock {
    public:
        explicit StpmLock(SemaphoreHandle_t m) : _m(m) { if (_m) xSemaphoreTakeRecursive(_m, portMAX_DELAY); }
        ~StpmLock() { if (_m) xSemaphoreGiveRecursive(_m); }
        StpmLock(const StpmLock&) = delete;
        StpmLock& operator=(const StpmLock&) = delete;
    private:
        SemaphoreHandle_t _m;
    };

} // namespace

// ------------------- LOW-LEVEL UART AND CRC -------------------

uint8_t STPMClass::byteReverse(uint8_t n) {
    n = ((n >> 1) & 0x55) | ((n << 1) & 0xAA);
    n = ((n >> 2) & 0x33) | ((n << 2) & 0xCC);
    n = ((n >> 4) & 0x0F) | ((n << 4) & 0xF0);
    return n;
}

uint8_t STPMClass::crc8CalcRaw(const uint8_t* buf, uint8_t len) {
    const uint8_t poly = 0x07;
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t data = buf[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t tmp = data ^ crc;
            crc <<= 1;
            if (tmp & 0x80) crc ^= poly;
            data <<= 1;
        }
    }
    return crc;
}

// In UART mode the STPM32 computes the CRC-8 on bit-reversed bytes.
uint8_t STPMClass::crc8ForUart(const uint8_t* buf4) {
    uint8_t rev[4];
    for (uint8_t i = 0; i < 4; i++) rev[i] = byteReverse(buf4[i]);
    uint8_t crcOnRev = crc8CalcRaw(rev, 4);
    return byteReverse(crcOnRev);
}

bool STPMClass::stpmTransfer(uint8_t readAddr, uint8_t writeAddr, uint8_t dataLSB, uint8_t dataMSB, uint8_t respOut[4]) {
    uint8_t tx[STPM_FRAME_LEN] = { readAddr, writeAddr, dataLSB, dataMSB, 0 };
    tx[4] = crc8ForUart(tx);

    while (_serial.available()) _serial.read();
    _serial.write(tx, STPM_FRAME_LEN);
    _serial.flush();

    // Stream::readBytes() busy-waits and starves lower-priority tasks.
    // Sleep one tick per iteration instead (~5 ms per frame at 9600 baud).
    uint8_t rx[STPM_FRAME_LEN];
    const uint32_t start = millis();
    while (_serial.available() < STPM_FRAME_LEN) {
        if (millis() - start >= STPM_TIMEOUT_MS) return false;
        vTaskDelay(1);
    }
    for (uint8_t k = 0; k < STPM_FRAME_LEN; k++) rx[k] = (uint8_t)_serial.read();
    if (crc8ForUart(rx) != rx[4]) return false;

    memcpy(respOut, rx, 4);
    return true;
}

// ------------------- REGISTER READ / WRITE -------------------
// A read takes 2 transactions (set pointer + fetch data). If another task gets in between,
// the pointer is corrupted and the WRONG register is returned -> the whole sequence is locked.

bool STPMClass::readRegister32(uint8_t addrLSB, uint32_t* valueOut) {
    if (valueOut == nullptr) return false;
    StpmLock lock(_mutex);
    uint8_t resp[4];
    if (!stpmTransfer(addrLSB, 0xFF, 0xFF, 0xFF, resp)) return false;
    if (!stpmTransfer(0xFF, 0xFF, 0xFF, 0xFF, resp)) return false;

    *valueOut = (uint32_t)resp[0] | ((uint32_t)resp[1] << 8) | ((uint32_t)resp[2] << 16) | ((uint32_t)resp[3] << 24);
    return true;
}

bool STPMClass::writeRegister16(uint8_t addr, uint16_t value) {
    uint8_t resp[4];
    return stpmTransfer(0xFF, addr, value & 0xFF, (value >> 8) & 0xFF, resp);
}

bool STPMClass::writeRegister32(uint8_t addrLSB, uint32_t value) {
    StpmLock lock(_mutex);
    bool ok = writeRegister16(addrLSB, value & 0xFFFF);
    ok = writeRegister16(addrLSB + 1, (value >> 16) & 0xFFFF) && ok;
    return ok;
}

bool STPMClass::verifyRegister(uint8_t addr, uint32_t expected) {
    uint32_t v;
    return readRegister32(addr, &v) && (v == expected);
}

// ------------------- CONFIGURATION (NVS) -------------------

bool STPMClass::persist() {
    return _preferences.putBytes("config", &_activeConfig, sizeof(STPM32_BootConfig)) == sizeof(STPM32_BootConfig);
}

void STPMClass::loadConfig() {
    if (_preferences.getBytesLength("config") == sizeof(STPM32_BootConfig)) {
        STPM32_BootConfig tmp;
        _preferences.getBytes("config", &tmp, sizeof(tmp));
        if (tmp.version == STPM_CFG_VERSION) {
            _activeConfig = tmp;
            return;
        }
    }
    // Nothing stored, or an incompatible version -> start from the built-in defaults
    _activeConfig = STPM32_BootConfig();
    persist();
}

// ------------------- SCALING (LSB) -------------------
// Datasheet Table 14: LSB_IRMS ~ 1/AI, so i_lsb changes with the current gain.
// The threshold registers use the same LSBs and must be re-encoded when the gain changes.

void STPMClass::setScale(float vLsb, float iLsbAtGain16) {
    StpmLock lock(_mutex);
    if (vLsb > 0.0f) _vLsbBase = vLsb;
    if (iLsbAtGain16 > 0.0f) _iLsbBase16 = iLsbAtGain16;
    updateScaling();
}

void STPMClass::updateScaling() {
    static const float gainTable[4] = { 2.0f, 4.0f, 8.0f, 16.0f };
    DFE_CR1_Register dfe;
    dfe.all = _activeConfig.dfe1_0x18;
    _current_gain = gainTable[dfe.bit.GAIN1 & 0x3];

    _v_lsb = _vLsbBase;
    _i_lsb = _iLsbBase16 * (16.0f / _current_gain);
    _p_lsb = _v_lsb * _i_lsb * 16.0f;                    // datasheet: LSB_P = 16 * LSB_V * LSB_I
}

// Threshold <-> register conversion (datasheet Table 15-17):
//   SAG/SWV_THR = peak_voltage / (32  * LSB_VRMS),  peak = RMS * sqrt(2)
//   SWC_THR     = peak_current / (128 * LSB_IRMS)
// The registers are 10 bits wide and would wrap (mod 1024) -> clamp.
uint32_t STPMClass::encodeSagThr(float vRms) const {
    if (vRms <= 0.0f) return 0;                          // 0 = sag detection off
    float c = (vRms * SQRT2 / _v_lsb) / 32.0f;
    if (c > 1023.0f) c = 1023.0f;
    return (uint32_t)lroundf(c);
}

uint32_t STPMClass::encodeSwellVThr(float vRms) const {
    float c = (vRms * SQRT2 / _v_lsb) / 32.0f;
    if (c > 1023.0f) c = 1023.0f;                        // 0x3FF = swell detection off
    if (c < 1.0f) c = 1.0f;
    return (uint32_t)lroundf(c);
}

uint32_t STPMClass::encodeSwcThr(float aRms) const {
    float c = (aRms * SQRT2 / _i_lsb) / 128.0f;
    if (c > 1023.0f) c = 1023.0f;                        // 0x3FF = overcurrent detection off
    if (c < 1.0f) c = 1.0f;
    return (uint32_t)lroundf(c);
}

float STPMClass::decodeVThr(uint32_t code) const {
    return (code * 32.0f * _v_lsb) / SQRT2;
}

float STPMClass::decodeSwcThr(uint32_t code) const {
    return (code * 128.0f * _i_lsb) / SQRT2;
}

// ------------------- UART LINK (baud negotiation) -------------------
// The STPM32 powers up at 9600 baud. If the ESP32 resets after switching to a higher rate
// (OTA, watchdog, brownout) the STPM stays at that rate, so both rates are probed at boot.
// Bytes sent at the wrong rate can shift the STPM's 5-byte frame alignment and by default
// there is no timeout to recover from that -> on the first successful exchange the US_REG1
// timeout field is set, so partial frames are dropped after STPM_UART_TIMEOUT_MS of silence.

namespace {
    constexpr uint8_t US_REG1_ADDR = 0x24;   // [15:0] CRC settings, [23:16] timeout (ms)
    constexpr uint8_t US_REG2_ADDR = 0x26;   // [15:0] baud divisor, [23:16] frame delay
    constexpr uint32_t STPM_DEFAULT_BAUD = 9600;
}

// Datasheet Eq. 19-21 / Table 32: BRDIV = 16 MHz / (16 * BR); BRRI = integer part [15:4], BRRF = fraction*16 [3:0]
uint16_t STPMClass::baudDivisor(uint32_t baud) {
    const double div = 16000000.0 / (16.0 * (double)baud);
    uint32_t bri = (uint32_t)div;
    uint32_t brf = (uint32_t)lround((div - (double)bri) * 16.0);
    if (brf >= 16) { bri++; brf = 0; }
    return (uint16_t)((bri << 4) | brf);     // 9600 -> 0x683, 115200 -> 0x08B (matches Table 32)
}

void STPMClass::drainRx(uint32_t quietMs) {
    vTaskDelay(pdMS_TO_TICKS(quietMs));
    while (_serial.available()) _serial.read();
}

bool STPMClass::probeBaud(uint32_t baud) {
    _serial.updateBaudRate(baud);
    drainRx(STPM_LINK_QUIET_MS);             // let any partial frame on the STPM side time out
    uint32_t v = 0;
    for (uint8_t k = 0; k < 2; k++) {
        if (readRegister32(US_REG2_ADDR, &v) && (v & 0xFFFF) == baudDivisor(baud)) {
            _baud = baud;
            return true;
        }
        drainRx(STPM_LINK_QUIET_MS);
    }
    return false;
}

bool STPMClass::establishLink(uint32_t targetBaud) {
    StpmLock lock(_mutex);
    _linkOk = false;

    // 1) Find the current rate. Try 9600 first: after a cold start the STPM is at 9600 and
    //    has no timeout configured yet, so it must not receive bytes at a wrong rate.
    const uint32_t candidates[2] = { STPM_DEFAULT_BAUD, targetBaud };
    bool found = false;
    for (uint8_t attempt = 0; attempt < 2 && !found; attempt++) {
        for (uint8_t k = 0; k < 2 && !found; k++) {
            if (k == 1 && candidates[1] == candidates[0]) break;
            found = probeBaud(candidates[k]);
        }
    }
    if (!found) {
        _serial.updateBaudRate(STPM_DEFAULT_BAUD);
        _baud = 0;
        return false;
    }

    // 2) Frame sync safety: US_REG1[23:16] = timeout (ms). [15:0] (CRC settings) unchanged.
    if (STPM_UART_TIMEOUT_MS > 0) {
        writeRegister16(US_REG1_ADDR + 1, (uint16_t)(STPM_UART_TIMEOUT_MS & 0xFF));
    }

    // 3) Switch to the target rate if needed. The reply to this write may arrive at mixed rates -> ignored.
    if (_baud != targetBaud) {
        writeRegister16(US_REG2_ADDR, baudDivisor(targetBaud));
        _serial.flush();
        if (!probeBaud(targetBaud)) {
            // The switch did not take; the STPM may still be at the old rate
            if (!probeBaud(STPM_DEFAULT_BAUD)) { _baud = 0; return false; }
        }
    }
    _linkOk = true;
    return true;
}

// Writes the full configuration in a safe order (mask off -> clear -> write -> wait -> mask on).
bool STPMClass::applyConfig() {
    StpmLock lock(_mutex);
    bool ok = writeRegister32(DSP_IRQ1_Register::ADDRESS, 0);
    ok = clearStatus(0xFFFFFFFF) && ok;
    ok = clearSagSwellHistory() && ok;

    ok = writeRegister32(DSP_CR1_Register::ADDRESS, _activeConfig.cr1_0x00) && ok;
    ok = writeRegister32(DSP_CR3_Register::ADDRESS, _activeConfig.cr3_0x04) && ok;
    ok = writeRegister32(DSP_CR4_Register::ADDRESS, _activeConfig.cr4_0x06) && ok;
    ok = writeRegister32(DSP_CR5_Register::ADDRESS, _activeConfig.cr5_0x08) && ok;
    ok = writeRegister32(DSP_CR6_Register::ADDRESS, _activeConfig.cr6_0x0A) && ok;
    ok = writeRegister32(DSP_CR9_Register::ADDRESS, _activeConfig.cr9_0x10) && ok;
    ok = writeRegister32(DSP_CR10_Register::ADDRESS, _activeConfig.cr10_0x12) && ok;
    ok = writeRegister32(DFE_CR1_Register::ADDRESS, _activeConfig.dfe1_0x18) && ok;

    vTaskDelay(pdMS_TO_TICKS(30));
    ok = writeRegister32(DSP_IRQ1_Register::ADDRESS, _activeConfig.irq1_0x1C) && ok;
    return ok && configMatches();
}

// Some CR1/CR3 bits self-clear, so only these registers are verified.
bool STPMClass::configMatches() {
    StpmLock lock(_mutex);
    return verifyRegister(DSP_CR5_Register::ADDRESS, _activeConfig.cr5_0x08) &&
           verifyRegister(DSP_CR6_Register::ADDRESS, _activeConfig.cr6_0x0A) &&
           verifyRegister(DSP_IRQ1_Register::ADDRESS, _activeConfig.irq1_0x1C);
}

// ------------------- STARTUP -------------------

bool STPMClass::begin(int rxPin, int txPin, uint32_t targetBaud, const char* nvsNamespace) {
    if (!_mutex) _mutex = xSemaphoreCreateRecursiveMutex();
    StpmLock lock(_mutex);

    _targetBaud = targetBaud;
    _serial.begin(STPM_DEFAULT_BAUD, SERIAL_8N1, rxPin, txPin);
    _serial.setTimeout(STPM_TIMEOUT_MS);

    _preferences.begin(nvsNamespace, false);
    loadConfig();
    updateScaling();

    if (!establishLink(_targetBaud)) return false;
    // The STPM32 stays powered across ESP32 resets; stale SR1 bits must not fire INT1 right away,
    // so the configuration is re-applied in a safe order at every boot.
    return applyConfig();
}

// Call when communication is lost (e.g. several failed reads in a row).
// Returns: 0 = not recovered, 1 = link restored, 2 = link restored and the STPM had lost its
// configuration (it was reset) -> re-applied; any energy counter reference should be reset.
uint8_t STPMClass::recoverLink() {
    StpmLock lock(_mutex);
    if (!establishLink(_targetBaud)) return 0;
    if (configMatches()) return 1;
    return applyConfig() ? 2 : 0;
}

// ------------------- CALIBRATION TRIM -------------------
// Datasheet Table 37: CHV = 14336 * XV/VAV - 12288 (generalised below for the current CHV value)

bool STPMClass::setVoltageTrim(float expectedVoltage, float measuredVoltage) {
    if (expectedVoltage <= 0.0f || measuredVoltage <= 0.0f) return false;
    StpmLock lock(_mutex);

    DSP_CR5_Register cr5;
    cr5.all = _activeConfig.cr5_0x08;

    float ratio = expectedVoltage / measuredVoltage;
    float calc = ((float)cr5.bit.CHV1 + 12288.0f) * ratio - 12288.0f;
    if (calc < 0.0f) calc = 0.0f;
    if (calc > 4095.0f) calc = 4095.0f;
    cr5.bit.CHV1 = (uint32_t)lroundf(calc);

    if (!writeRegister32(DSP_CR5_Register::ADDRESS, cr5.all)) return false;
    _activeConfig.cr5_0x08 = cr5.all;
    return persist();
}

bool STPMClass::setCurrentTrim(float expectedCurrent, float measuredCurrent) {
    if (expectedCurrent <= 0.0f || measuredCurrent <= 0.0f) return false;
    StpmLock lock(_mutex);

    DSP_CR6_Register cr6;
    cr6.all = _activeConfig.cr6_0x0A;

    float ratio = expectedCurrent / measuredCurrent;
    float calc = ((float)cr6.bit.CHC1 + 12288.0f) * ratio - 12288.0f;
    if (calc < 0.0f) calc = 0.0f;
    if (calc > 4095.0f) calc = 4095.0f;
    cr6.bit.CHC1 = (uint32_t)lroundf(calc);

    if (!writeRegister32(DSP_CR6_Register::ADDRESS, cr6.all)) return false;
    _activeConfig.cr6_0x0A = cr6.all;
    return persist();
}

// ------------------- SETTINGS -------------------

bool STPMClass::setCurrentGain(uint8_t gainCode) {
    if (gainCode > 3) return false;
    StpmLock lock(_mutex);

    DSP_CR6_Register cr6;
    cr6.all = _activeConfig.cr6_0x0A;
    bool swcEnabled = (cr6.bit.SWC_THR1 != 0x3FF);
    float maxAmps = decodeSwcThr(cr6.bit.SWC_THR1);      // convert to amperes with the OLD LSB

    DFE_CR1_Register dfe;
    dfe.all = _activeConfig.dfe1_0x18;
    dfe.bit.GAIN1 = gainCode;
    if (!writeRegister32(DFE_CR1_Register::ADDRESS, dfe.all)) return false;
    _activeConfig.dfe1_0x18 = dfe.all;

    updateScaling();                                     // i_lsb and p_lsb for the new gain

    if (swcEnabled) {                                    // re-encode the overcurrent threshold with the new LSB
        cr6.bit.SWC_THR1 = encodeSwcThr(maxAmps);
        if (!writeRegister32(DSP_CR6_Register::ADDRESS, cr6.all)) return false;
        _activeConfig.cr6_0x0A = cr6.all;
    }
    return persist();
}

bool STPMClass::setLEDPulseRate(uint8_t lpw_code) {
    if (lpw_code > 15) return false;
    StpmLock lock(_mutex);

    DSP_CR1_Register cr1;
    cr1.all = _activeConfig.cr1_0x00;
    cr1.bit.LPW1 = lpw_code;

    if (!writeRegister32(DSP_CR1_Register::ADDRESS, cr1.all)) return false;
    _activeConfig.cr1_0x00 = cr1.all;
    return persist();
}

bool STPMClass::setTamperProtection(bool enable) {
    (void)enable;
    return false;   // The tamper module exists only on STPM33/STPM34 (datasheet 8.4.11)
}

bool STPMClass::setSagSwellThresholds(float sagVolts, float swellVolts) {
    if (sagVolts < 0.0f || swellVolts <= sagVolts) return false;
    StpmLock lock(_mutex);

    DSP_CR5_Register cr5;
    cr5.all = _activeConfig.cr5_0x08;
    cr5.bit.SAG_THR1 = encodeSagThr(sagVolts);
    cr5.bit.SWV_THR1 = encodeSwellVThr(swellVolts);

    if (!writeRegister32(DSP_CR5_Register::ADDRESS, cr5.all)) return false;
    _activeConfig.cr5_0x08 = cr5.all;
    return persist();
}

// ------------------- MEASUREMENT -------------------
// The driver returns RAW physical values. Anti-creep / PF / energy policy belongs to the caller.

bool STPMClass::readVoltageAndCurrent(float& voltageOut, float& currentOut) {
    StpmLock lock(_mutex);
    uint32_t rawVal;
    if (!readRegister32(DSP_REG14_Register::ADDRESS, &rawVal)) return false;

    DSP_REG14_Register rms;
    rms.all = rawVal;
    voltageOut = (float)rms.bit.v1_rms_data * _v_lsb;
    currentOut = (float)rms.bit.c1_rms_data * _i_lsb;
    return true;
}

// PH1_REG5..REG8: [28:0] signed (29-bit two's complement)
bool STPMClass::readPower29(uint8_t addr, float& out) {
    StpmLock lock(_mutex);
    uint32_t raw;
    if (!readRegister32(addr, &raw)) return false;
    raw &= 0x1FFFFFFFUL;
    int32_t v = (raw & 0x10000000UL) ? (int32_t)(raw | 0xE0000000UL) : (int32_t)raw;
    out = (float)v * _p_lsb;
    return true;
}

bool STPMClass::readActivePower(float& powerOut) {
    return readPower29(PH1_REG5_Register::ADDRESS, powerOut);
}

bool STPMClass::readApparentPower(float& powerOut) {
    return readPower29(PH1_REG8_Register::ADDRESS, powerOut);
}

bool STPMClass::readSnapshot(StpmSnapshot& out) {
    StpmLock lock(_mutex);
    StpmSnapshot s;
    uint32_t rms = 0, energy = 0;
    if (!readRegister32(DSP_REG14_Register::ADDRESS, &rms)) return false;
    if (!readPower29(PH1_REG5_Register::ADDRESS, s.pActive)) return false;
    if (!readPower29(PH1_REG6_Register::ADDRESS, s.pFund)) return false;
    if (!readPower29(PH1_REG7_Register::ADDRESS, s.qReactive)) return false;
    if (!readPower29(PH1_REG8_Register::ADDRESS, s.sApparent)) return false;
    if (!readRegister32(PH1_REG1_Register::ADDRESS, &energy)) return false;
    uint32_t period = 0, qEnergy = 0, sEnergy = 0;
    if (!readRegister32(DSP_REG1_Register::ADDRESS, &period)) return false;
    if (!readRegister32(PH1_REG3_Register::ADDRESS, &qEnergy)) return false;
    if (!readRegister32(PH1_REG4_Register::ADDRESS, &sEnergy)) return false;

    DSP_REG14_Register r; r.all = rms;
    s.rawRms = rms;
    s.vRms = (float)r.bit.v1_rms_data * _v_lsb;
    s.iRms = (float)r.bit.c1_rms_data * _i_lsb;
    s.energyRaw = energy;
    s.reactEnergyRaw = qEnergy;
    s.appEnergyRaw = sEnergy;
    DSP_REG1_Register pr; pr.all = period;
    s.lineHz = pr.bit.ph1_period ? 125000.0f / (float)pr.bit.ph1_period : 0.0f;   // 8 us per tick
    out = s;
    return true;
}

// Datasheet: LSB_E = LSB_P * 2^11 / (3600 * DCLK) [Wh], DCLK = 7812.5 Hz
// (Verified on hardware: counter overflow at 2^32 * LSB_E matched the measured point.)
double STPMClass::energyLsbWh() const {
    return (double)_p_lsb * 2048.0 / (3600.0 * 7812.5);
}

// ------------------- INTERRUPT / STATUS -------------------

bool STPMClass::readStatus(uint32_t& sr1) {
    return readRegister32(DSP_SR1_Register::ADDRESS, &sr1);
}

// SR1 bits stay latched after the event ends, and INT1 stays active while any unmasked bit is set.
// Datasheet: "cleared to zero by a write operation". For selective clearing write 0 = clear,
// 1 = keep, so events that arrive between the read and the clear are not lost.
bool STPMClass::clearStatus(uint32_t mask) {
    return writeRegister32(DSP_SR1_Register::ADDRESS, ~mask);
}

uint32_t STPMClass::irqMask() {
    StpmLock lock(_mutex);
    return _activeConfig.irq1_0x1C;
}

bool STPMClass::readLiveEvents(uint32_t& ev1) {
    return readRegister32(DSP_EV1_Register::ADDRESS, &ev1);
}

bool STPMClass::clearSagSwellHistory() {
    StpmLock lock(_mutex);
    DSP_CR1_Register cr1;
    cr1.all = _activeConfig.cr1_0x00;
    cr1.bit.ClearSS1 = 1;                                // self-clearing; not stored in the cache
    return writeRegister32(DSP_CR1_Register::ADDRESS, cr1.all);
}

// ------------------- PROTECTION -------------------
// IRQ mask: 1 = enabled.
// Sequence: mask off -> clear stale latched status -> write new threshold -> wait for DSP -> mask on.
// This way a stale event does not fire INT1 immediately, but a condition that is REALLY present
// when the threshold is written (e.g. voltage already above the swell threshold) correctly asserts INT1.

bool STPMClass::setVoltageProtection(bool enable, float sagVolts, float swellVolts) {
    if (sagVolts < 0.0f || swellVolts <= sagVolts) return false;
    StpmLock lock(_mutex);

    uint32_t maskOff = _activeConfig.irq1_0x1C & ~STPM_IRQ::VOLTAGE_EVENTS;
    if (!writeRegister32(DSP_IRQ1_Register::ADDRESS, maskOff)) return false;
    _activeConfig.irq1_0x1C = maskOff;
    if (!clearStatus(STPM_IRQ::VOLTAGE_EVENTS)) return false;

    DSP_CR5_Register cr5;
    cr5.all = _activeConfig.cr5_0x08;
    cr5.bit.SAG_THR1 = encodeSagThr(sagVolts);
    cr5.bit.SWV_THR1 = encodeSwellVThr(swellVolts);
    if (!writeRegister32(DSP_CR5_Register::ADDRESS, cr5.all)) return false;
    _activeConfig.cr5_0x08 = cr5.all;

    if (enable) {
        delay(30);   // sag time filter (~10 ms) + a few samples
        uint32_t maskOn = maskOff | STPM_IRQ::VOLTAGE_EVENTS;
        if (!writeRegister32(DSP_IRQ1_Register::ADDRESS, maskOn)) return false;
        _activeConfig.irq1_0x1C = maskOn;
    }
    return persist();
}

bool STPMClass::setCurrentProtection(bool enable, float maxAmps) {
    if (maxAmps <= 0.0f) return false;
    StpmLock lock(_mutex);

    uint32_t maskOff = _activeConfig.irq1_0x1C & ~STPM_IRQ::CURRENT_EVENTS;
    if (!writeRegister32(DSP_IRQ1_Register::ADDRESS, maskOff)) return false;
    _activeConfig.irq1_0x1C = maskOff;
    if (!clearStatus(STPM_IRQ::CURRENT_EVENTS)) return false;

    DSP_CR6_Register cr6;
    cr6.all = _activeConfig.cr6_0x0A;
    cr6.bit.SWC_THR1 = encodeSwcThr(maxAmps);
    if (!writeRegister32(DSP_CR6_Register::ADDRESS, cr6.all)) return false;
    _activeConfig.cr6_0x0A = cr6.all;

    if (enable) {
        delay(30);
        uint32_t maskOn = maskOff | STPM_IRQ::CURRENT_EVENTS;
        if (!writeRegister32(DSP_IRQ1_Register::ADDRESS, maskOn)) return false;
        _activeConfig.irq1_0x1C = maskOn;
    }
    return persist();
}

void STPMClass::getProtectionSettings(bool& v_en, float& sagVolts, float& swellVolts, bool& c_en, float& maxAmps) {
    StpmLock lock(_mutex);
    DSP_CR5_Register cr5;   cr5.all = _activeConfig.cr5_0x08;
    DSP_CR6_Register cr6;   cr6.all = _activeConfig.cr6_0x0A;
    DSP_IRQ1_Register irq;  irq.all = _activeConfig.irq1_0x1C;

    sagVolts = decodeVThr(cr5.bit.SAG_THR1);
    swellVolts = decodeVThr(cr5.bit.SWV_THR1);
    maxAmps = decodeSwcThr(cr6.bit.SWC_THR1);

    // 1 = enabled
    v_en = (irq.bit.v1_sag_detected == 1) || (irq.bit.v1_swell_detected == 1);
    c_en = (irq.bit.c1_swell_detected == 1);
}
