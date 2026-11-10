#pragma once

#include <Arduino.h>

namespace motor_pwm {

void begin();
void setDuty(uint8_t phaseA, uint8_t phaseB, uint8_t phaseC);
void disable();

}  // namespace motor_pwm
