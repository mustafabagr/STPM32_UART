// Calibration - trims the voltage and current gain against a reference meter.
//
// Apply a stable load, read the true values from a reference meter and type in the Serial Monitor:
//   v 230.0   -> voltage reads 230.0 V
//   i 4.350   -> current reads 4.350 A
//   r         -> print the current reading
// The trim (CHV1 / CHC1) is stored in NVS and re-applied at every boot.

#include <STPM32_UART.h>

constexpr int STPM_RX_PIN = 16;
constexpr int STPM_TX_PIN = 17;

HardwareSerial StpmSerial(1);
STPMClass stpm(StpmSerial);

// Averages a few readings to reduce noise
bool averagedReading(float& v, float& i) {
  float sv = 0, si = 0;
  const int n = 8;
  for (int k = 0; k < n; k++) {
    float vv, ii;
    if (!stpm.readVoltageAndCurrent(vv, ii)) return false;
    sv += vv;
    si += ii;
    delay(100);
  }
  v = sv / n;
  i = si / n;
  return true;
}

void setup() {
  Serial.begin(115200);
  if (!stpm.begin(STPM_RX_PIN, STPM_TX_PIN)) {
    Serial.println("STPM32 not responding");
    return;
  }
  Serial.println("Commands: 'v <volts>', 'i <amps>', 'r'");
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  const char cmd = line.charAt(0);
  const float ref = line.substring(1).toFloat();

  float v, i;
  if (!averagedReading(v, i)) {
    Serial.println("Read failed");
    return;
  }

  if (cmd == 'v' && ref > 0) {
    Serial.printf("Measured %.3f V, reference %.3f V -> %s\n", v, ref,
                  stpm.setVoltageTrim(ref, v) ? "trim saved" : "trim FAILED");
  } else if (cmd == 'i' && ref > 0) {
    Serial.printf("Measured %.4f A, reference %.4f A -> %s\n", i, ref,
                  stpm.setCurrentTrim(ref, i) ? "trim saved" : "trim FAILED");
  } else {
    Serial.printf("V=%.3f V  I=%.4f A\n", v, i);
  }
}
