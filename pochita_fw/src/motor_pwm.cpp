#include "motor_pwm.h"

#include <driver/mcpwm.h>

namespace motor_pwm {
namespace {
constexpr uint8_t PHASE_PWM_PINS[] = {16, 17, 18};
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr mcpwm_unit_t PWM_UNIT = MCPWM_UNIT_0;
constexpr mcpwm_timer_t PWM_TIMERS[] = {MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_2};

}  // namespace

void begin() {
  for (size_t phase = 0; phase < 3; ++phase) {
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, static_cast<mcpwm_io_signals_t>(MCPWM0A + phase * 2), PHASE_PWM_PINS[phase]));

    mcpwm_config_t config = {};
    config.frequency = PWM_FREQUENCY;
    config.cmpr_a = 0.0f;
    config.cmpr_b = 0.0f;
    config.duty_mode = MCPWM_DUTY_MODE_0;
    config.counter_mode = MCPWM_UP_COUNTER;
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, PWM_TIMERS[phase], &config));
  }
  disable();
}

void setDuty(uint8_t phaseA, uint8_t phaseB, uint8_t phaseC) {
  const uint8_t duty[] = {phaseA, phaseB, phaseC};
  for (size_t phase = 0; phase < 3; ++phase) {
    const float dutyPercent = static_cast<float>(duty[phase]) * 100.0f / 255.0f;
    ESP_ERROR_CHECK(mcpwm_set_duty(PWM_UNIT, PWM_TIMERS[phase], MCPWM_GEN_A, dutyPercent));
  }
}

void disable() {
  setDuty(0, 0, 0);
}

}  // namespace motor_pwm
