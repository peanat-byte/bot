#include "drive.h"

#include <Arduino.h>
#include <driver/mcpwm.h>

namespace drive {
namespace {
constexpr uint8_t LEFT_MOTOR_IN1 = 39;
constexpr uint8_t LEFT_MOTOR_IN2 = 40;
constexpr uint8_t RIGHT_MOTOR_IN1 = 41;
constexpr uint8_t RIGHT_MOTOR_IN2 = 42;
constexpr float MAX_DUTY_PERCENT = 20.0f;
constexpr uint16_t JOYSTICK_CENTER = 2000;
constexpr uint16_t JOYSTICK_MAXIMUM = 4000;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr mcpwm_unit_t PWM_UNIT = MCPWM_UNIT_1;
constexpr mcpwm_timer_t LEFT_PWM_TIMER = MCPWM_TIMER_0;
constexpr mcpwm_timer_t RIGHT_PWM_TIMER = MCPWM_TIMER_1;
}  // namespace

void begin() {
  mcpwm_gpio_init(PWM_UNIT, MCPWM0A, LEFT_MOTOR_IN1);
  mcpwm_gpio_init(PWM_UNIT, MCPWM0B, LEFT_MOTOR_IN2);
  mcpwm_gpio_init(PWM_UNIT, MCPWM1A, RIGHT_MOTOR_IN1);
  mcpwm_gpio_init(PWM_UNIT, MCPWM1B, RIGHT_MOTOR_IN2);

  mcpwm_config_t config = {};
  config.frequency = PWM_FREQUENCY;
  config.cmpr_a = 0.0f;
  config.cmpr_b = 0.0f;
  config.duty_mode = MCPWM_DUTY_MODE_0;
  config.counter_mode = MCPWM_UP_COUNTER;
  ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, LEFT_PWM_TIMER, &config));
  ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, RIGHT_PWM_TIMER, &config));
  stop();
}

void setMotorDuty(mcpwm_timer_t timer, float dutyPercent, bool forward) {
  const mcpwm_generator_t activeGenerator = forward ? MCPWM_GEN_A : MCPWM_GEN_B;
  const mcpwm_generator_t inactiveGenerator = forward ? MCPWM_GEN_B : MCPWM_GEN_A;

  ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, timer, activeGenerator, dutyPercent));
  ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, timer, inactiveGenerator, 0.0f));
}

void setJoystickOutput(uint16_t horizontal, uint16_t vertical) {
  horizontal = min(horizontal, JOYSTICK_MAXIMUM);
  vertical = min(vertical, JOYSTICK_MAXIMUM);

  const bool forward = vertical > JOYSTICK_CENTER;
  const float baseDuty = static_cast<float>(abs(static_cast<int32_t>(vertical) - JOYSTICK_CENTER)) / JOYSTICK_CENTER * MAX_DUTY_PERCENT;
  float leftDuty = baseDuty;
  float rightDuty = baseDuty;

  if (horizontal < JOYSTICK_CENTER) {
    leftDuty *= static_cast<float>(horizontal) / JOYSTICK_CENTER;
  } else if (horizontal > JOYSTICK_CENTER) {
    rightDuty *= static_cast<float>(JOYSTICK_MAXIMUM - horizontal) / JOYSTICK_CENTER;
  }

  setMotorDuty(LEFT_PWM_TIMER, leftDuty, forward);
  setMotorDuty(RIGHT_PWM_TIMER, rightDuty, forward);
}

void stop() {
  ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, LEFT_PWM_TIMER, MCPWM_GEN_A, 0.0f));
  ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, LEFT_PWM_TIMER, MCPWM_GEN_B, 0.0f));
  ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, RIGHT_PWM_TIMER, MCPWM_GEN_A, 0.0f));
  ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, RIGHT_PWM_TIMER, MCPWM_GEN_B, 0.0f));
}
}  // namespace drive
