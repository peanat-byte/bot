#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "bluetooth_receiver.h"
#include "drive.h"
#include "encoder.h"
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
constexpr TickType_t FOC_TASK_DELAY = pdMS_TO_TICKS(1);
constexpr TickType_t BLUETOOTH_TASK_DELAY = pdMS_TO_TICKS(10);
constexpr TickType_t DRIVE_TASK_DELAY = pdMS_TO_TICKS(10);
constexpr TickType_t DRIVE_COMMAND_TIMEOUT = pdMS_TO_TICKS(100);

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
}

void sensorlessFocTask(void*) {
  for (;;) {
    sensorless_foc::update();
    vTaskDelay(FOC_TASK_DELAY);
  }
}

void bluetoothTask(void*) {
  for (;;) {
    bluetooth_receiver::update();
    vTaskDelay(BLUETOOTH_TASK_DELAY);
  }
}

void driveTask(void*) {
  drive::begin();
  encoder::begin();
  TickType_t lastCommandTime = xTaskGetTickCount();

  for (;;) {
    bluetooth_receiver::ControlPacket packet;
    if (bluetooth_receiver::takePacket(packet)) {
      drive::setJoystickOutput(packet.rightHorizontal, packet.rightVertical);
      lastCommandTime = xTaskGetTickCount();
    } else if (xTaskGetTickCount() - lastCommandTime > DRIVE_COMMAND_TIMEOUT) {
      drive::stop();
    }
    vTaskDelay(DRIVE_TASK_DELAY);
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  // motor_pwm::begin();
  // sensorless_foc::begin();
  bluetooth_receiver::begin();
  //
  // if (xTaskCreate(sensorlessFocTask, "sensorless_foc", 4096, nullptr, 2, nullptr) != pdPASS) {
  //   Serial.println(F("Failed to create sensorless FOC task"));
  // }
  if (xTaskCreate(bluetoothTask, "bluetooth", 4096, nullptr, 1, nullptr) != pdPASS) {
    Serial.println(F("Failed to create Bluetooth task"));
  }
  if (xTaskCreate(driveTask, "drive", 4096, nullptr, 1, nullptr) != pdPASS) {
    Serial.println(F("Failed to create drive task"));
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
