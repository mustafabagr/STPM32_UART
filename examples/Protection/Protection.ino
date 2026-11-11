// Protection - voltage sag/swell and overcurrent detection on the STPM32 INT1 pin.
//
// INT1 is active-HIGH: it idles LOW and goes HIGH when an enabled event is latched in SR1.
// It stays HIGH until the latched bits are cleared, so the handler reads SR1 and clears
// exactly the bits it has handled.

#include <STPM32_UART.h>

constexpr int STPM_RX_PIN = 16;
constexpr int STPM_TX_PIN = 17;
constexpr int STPM_INT1_PIN = 4;

constexpr float SAG_VOLTS = 180.0f;
constexpr float SWELL_VOLTS = 250.0f;
constexpr float MAX_AMPS = 10.0f;

HardwareSerial StpmSerial(1);
STPMClass stpm(StpmSerial);

volatile bool int1Fired = false;
void IRAM_ATTR onInt1() { int1Fired = true; }

void setup() {
  Serial.begin(115200);
  pinMode(STPM_INT1_PIN, INPUT);

  if (!stpm.begin(STPM_RX_PIN, STPM_TX_PIN)) {
    Serial.println("STPM32 not responding");
    return;
  }

  // Settings are stored in NVS and re-applied at every boot.
  stpm.setVoltageProtection(true, SAG_VOLTS, SWELL_VOLTS);
  stpm.setCurrentProtection(true, MAX_AMPS);

  attachInterrupt(digitalPinToInterrupt(STPM_INT1_PIN), onInt1, RISING);
  // The condition may already be present at startup (INT1 already HIGH, no rising edge).
  if (digitalRead(STPM_INT1_PIN) == HIGH) int1Fired = true;

  Serial.println("Protection armed");
}

void loop() {
  if (!int1Fired) return;
  int1Fired = false;

  uint32_t sr1 = 0;
  if (!stpm.readStatus(sr1)) {
    int1Fired = true;   // retry on the next loop
    return;
  }

  if (sr1 & STPM_IRQ::V1_SAG)        Serial.println("Voltage SAG started");
  if (sr1 & STPM_IRQ::V1_SAG_END)    Serial.println("Voltage SAG ended");
  if (sr1 & STPM_IRQ::V1_SWELL)      Serial.println("Voltage SWELL started");
  if (sr1 & STPM_IRQ::V1_SWELL_END)  Serial.println("Voltage SWELL ended");
  if (sr1 & STPM_IRQ::C1_SWELL)      Serial.println("OVERCURRENT started");
  if (sr1 & STPM_IRQ::C1_SWELL_END)  Serial.println("OVERCURRENT ended");

  // Clear only what was handled; events arriving in between stay latched.
  const uint32_t handled = sr1 & STPM_IRQ::ALL_ALARMS;
  stpm.clearStatus(handled);
  if (handled & STPM_IRQ::VOLTAGE_EVENTS) stpm.clearSagSwellHistory();

  // If INT1 is still HIGH, another event is pending.
  if (digitalRead(STPM_INT1_PIN) == HIGH) int1Fired = true;
}
