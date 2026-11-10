#include "motor_pwm.h"

#include <driver/mcpwm_prelude.h>

namespace motor_pwm {
namespace {
constexpr uint8_t phasePwmPins[] = {16, 17, 18};
constexpr uint32_t pwmFrequency = 20000;
constexpr uint32_t pwmResolutionHz = 10000000;
constexpr uint32_t pwmPeriodTicks = pwmResolutionHz / pwmFrequency;

mcpwm_timer_handle_t pwmTimer = nullptr;
mcpwm_comparator_handle_t pwmComparators[3] = {};
mcpwm_generator_handle_t pwmGenerators[3] = {};

}  // namespace

void begin() {
  mcpwm_timer_config_t timerConfig = {};
  timerConfig.group_id = 0;
  timerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timerConfig.resolution_hz = pwmResolutionHz;
  timerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timerConfig.period_ticks = pwmPeriodTicks;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &pwmTimer));

  for (size_t phase = 0; phase < 3; ++phase) {
    mcpwm_operator_handle_t pwmOperator = nullptr;
    mcpwm_operator_config_t operatorConfig = {};
    operatorConfig.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &pwmOperator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(pwmOperator, pwmTimer));

    mcpwm_comparator_config_t comparatorConfig = {};
    comparatorConfig.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(
        pwmOperator, &comparatorConfig, &pwmComparators[phase]));

    mcpwm_generator_config_t generatorConfig = {};
    generatorConfig.gen_gpio_num = phasePwmPins[phase];
    ESP_ERROR_CHECK(mcpwm_new_generator(
        pwmOperator, &generatorConfig, &pwmGenerators[phase]));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        pwmGenerators[phase],
        {MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
         MCPWM_GEN_ACTION_HIGH}));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        pwmGenerators[phase],
        {MCPWM_TIMER_DIRECTION_UP, pwmComparators[phase],
         MCPWM_GEN_ACTION_LOW}));
  }

  ESP_ERROR_CHECK(mcpwm_timer_enable(pwmTimer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(
      pwmTimer, MCPWM_TIMER_START_NO_STOP));
  disable();
}

void setDuty(uint8_t phaseA, uint8_t phaseB, uint8_t phaseC) {
  const uint8_t duty[] = {phaseA, phaseB, phaseC};
  for (size_t phase = 0; phase < 3; ++phase) {
    const uint32_t compareTicks =
        (static_cast<uint32_t>(duty[phase]) * pwmPeriodTicks) / 255;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
        pwmComparators[phase], compareTicks));
  }
}

void disable() {
  setDuty(0, 0, 0);
}

}  // namespace motor_pwm
