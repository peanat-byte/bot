#include <Arduino.h>

#include "bluetooth_receiver.h"
#include "motor_pwm.h"
#include "sensorless_foc.h"

// Sensorless FOC TODOs:
// - Current-only samples do not contain enough information to determine rotor
//   position; the observer also needs commanded phase voltage.
// - Replace the motor and ADC placeholders in sensorless_foc.cpp with measured
//   values before enabling focOutputEnabled.
// - Add synchronized 20 kHz ADC sampling and validate the open-loop startup
//   ramp before connecting the inverter to a motor.
namespace {
constexpr uint8_t currentPins[] = {1, 2, 3};
constexpr uint8_t voltagePins[] = {4, 5, 6};

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
  Serial.print(analogRead(currentPins[0]));
  Serial.print(',');
  Serial.print(analogRead(currentPins[1]));
  Serial.print(',');
  Serial.println(analogRead(currentPins[2]));

  Serial.print(F("Voltage: "));
  Serial.print(analogRead(voltagePins[0]));
  Serial.print(',');
  Serial.print(analogRead(voltagePins[1]));
  Serial.print(',');
  Serial.println(analogRead(voltagePins[2]));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("BLE receiver starting"));

  motor_pwm::begin();
  sensorless_foc::begin();
  bluetooth_receiver::begin();
}

void loop() {
  bluetooth_receiver::ControlPacket packet;
  if (bluetooth_receiver::takePacket(packet)) {
    printPacket(packet);
  }

  sensorless_foc::update();
  bluetooth_receiver::update();
  if (!bluetooth_receiver::isConnected()) {
    motor_pwm::disable();
  }
}
