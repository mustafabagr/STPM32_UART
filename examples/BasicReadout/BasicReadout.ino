// BasicReadout - prints voltage, current, power, energy and line frequency once per second.
//
// Wiring (UART mode): STPM32 TXD -> ESP32 RX pin, STPM32 RXD -> ESP32 TX pin, common GND.
// The STPM32 enters UART mode when SCS is HIGH while EN goes HIGH (see the datasheet).
// WARNING: STPM32 boards are connected to mains voltage. Use an isolated USB connection
// or an isolated supply when the board is live.

#include <STPM32_UART.h>

constexpr int STPM_RX_PIN = 16;   // ESP32 pin receiving from STPM32 TXD
constexpr int STPM_TX_PIN = 17;   // ESP32 pin sending to STPM32 RXD

HardwareSerial StpmSerial(1);     // UART1 exists on every ESP32 variant
STPMClass stpm(StpmSerial);

void setup() {
  Serial.begin(115200);

  // Replace with the scale factors of YOUR hardware (see README, "Scaling").
  stpm.setScale(STPM_DEFAULT_V_LSB, STPM_DEFAULT_I_LSB_X16);

  if (!stpm.begin(STPM_RX_PIN, STPM_TX_PIN)) {
    Serial.println("STPM32 not responding - check wiring, EN/SCS levels and power.");
  } else {
    Serial.printf("STPM32 link OK at %lu baud\n", (unsigned long)stpm.baud());
  }
}

void loop() {
  static uint32_t lastPrint = 0;
  static uint8_t failures = 0;
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  StpmSnapshot s;
  if (!stpm.readSnapshot(s)) {
    Serial.println("Read failed");
    if (++failures >= 3) {
      failures = 0;
      uint8_t r = stpm.recoverLink();
      Serial.printf("recoverLink() -> %u\n", r);
    }
    return;
  }
  failures = 0;

  // The energy register wraps around (mod 2^32), so accumulate the signed difference
  // between readings. Read at least once per wrap period at your maximum power.
  static bool haveLast = false;
  static uint32_t lastRaw = 0;
  static double wh = 0;
  if (haveLast) wh += (double)(int32_t)(s.energyRaw - lastRaw) * stpm.energyLsbWh();
  lastRaw = s.energyRaw;
  haveLast = true;
  Serial.printf("V=%.2f V  I=%.4f A  P=%.2f W  Q=%.2f var  S=%.2f VA  f=%.2f Hz  E=%.4f Wh\n",
                s.vRms, s.iRms, s.pActive, s.qReactive, s.sApparent, s.lineHz, wh);
}
