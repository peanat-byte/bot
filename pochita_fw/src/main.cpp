#include <Arduino.h>

#include "bluetooth_receiver.h"
#include "motor_pwm.h"
#include "sensorless_foc.h"

// Sensorless FOC TODOs:
// - Current-only samples do not contain enough information to determine rotor
//   position; the observer also needs commanded phase voltage.
// - Replace the motor and ADC placeholders in sensorless_foc.cpp with measured
//   values before enabling FOC_OUTPUT_ENABLED.
// - Add synchronized 20 kHz ADC sampling and validate the open-loop startup
//   ramp before connecting the inverter to a motor.

namespace {
constexpr uint8_t CURRENT_PINS[] = {1, 2, 3};
constexpr uint8_t VOLTAGE_PINS[] = {4, 5, 6};

void printPacket(const bluetooth_receiver::ControlPacket& packet) {
  Serial.print((packet.switches >> 0) & 1);
  Serial.print(',');
  Serial.print((packet.switches >> 1) & 1);
  Serial.print(',');
  Serial.print(packet.leftHorizontal);
  Serial.print(',');
  Serial.print(packet.leftVertical);
  Serial.print(',');
  Serial.print(packet.rightHorizontal);
  Serial.print(',');
  Serial.print(packet.rightVertical);
  Serial.print(',');
  Serial.print((packet.switches >> 2) & 1);
  Serial.print(',');
  Serial.print((packet.switches >> 3) & 1);
  Serial.print(',');
  Serial.println((packet.switches >> 4) & 1);

  Serial.print(F("Current: "));
  Serial.print(analogRead(CURRENT_PINS[0]));
  Serial.print(',');
  Serial.print(analogRead(CURRENT_PINS[1]));
  Serial.print(',');
  Serial.println(analogRead(CURRENT_PINS[2]));

  Serial.print(F("Voltage: "));
  Serial.print(analogRead(VOLTAGE_PINS[0]));
  Serial.print(',');
  Serial.print(analogRead(VOLTAGE_PINS[1]));
  Serial.print(',');
  Serial.println(analogRead(VOLTAGE_PINS[2]));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  motor_pwm::begin();
  sensorless_foc::begin();
  bluetooth_receiver::begin();
}

void loop() {

}
