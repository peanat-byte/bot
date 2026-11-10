#include "sensorless_foc.h"

#include <math.h>

#include "motor_pwm.h"

namespace sensorless_foc {
namespace {
// TODO: Replace placeholders with measured values before enabling PWM output:
// - current sensor zero offsets
// - voltage divider and ADC calibration
// - motor phase inductance
// - verified phase resistance and DC bus voltage
// - current and voltage protection limits
constexpr uint8_t CURRENT_PINS[] = {1, 2};
constexpr uint8_t VOLTAGE_PINS[] = {4, 5};
constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float ADC_MAXIMUM_CODE = 4095.0f;
constexpr float CURRENT_BIAS_VOLTAGE = 1.65f;
constexpr float CURRENT_AMPLIFIER_GAIN = 10.0f;
constexpr float SHUNT_RESISTANCE = 0.002f;
constexpr float MOTOR_RESISTANCE = 45.0f;
constexpr float MOTOR_INDUCTANCE = 1.0e-6f;  // Placeholder.
constexpr float DC_BUS_VOLTAGE = 24.0f;
constexpr uint8_t POLE_PAIRS = 7;

constexpr float CONTROL_FREQUENCY = 20000.0f;
constexpr float CONTROL_PERIOD = 1.0f / CONTROL_FREQUENCY;
constexpr uint32_t CONTROL_PERIOD_MICROS = 1000000UL / 20000UL;
constexpr float ONE_OVER_SQRT_THREE = 0.57735026919f;
constexpr float SQRT_THREE_OVER_TWO = 0.86602540378f;

// Keep this false until the observer and protections are validated on hardware.
constexpr bool FOC_OUTPUT_ENABLED = false;
constexpr float TEST_IQ_AMPS = 0.5f;
constexpr float MAXIMUM_VOLTAGE = DC_BUS_VOLTAGE * 0.45f;
constexpr uint32_t ALIGNMENT_TIME_MICROS = 300000;
constexpr float OPEN_LOOP_START_SPEED = 20.0f;
constexpr float OPEN_LOOP_ACCELERATION = 3000.0f;

struct AlphaBeta {
  float alpha;
  float beta;
};

struct PhaseCurrents {
  float a;
  float b;
  float c;
};

struct PIController {
  float integral;
  float kp;
  float ki;
  float limit;
};

float electricalAngle = 0.0f;
float openLoopSpeed = 0.0f;
float observerAngle = 0.0f;
float observerSpeed = 0.0f;
float backEmfAlpha = 0.0f;
float backEmfBeta = 0.0f;
float estimatedCurrentAlpha = 0.0f;
float estimatedCurrentBeta = 0.0f;
uint32_t startTimeMicros = 0;
uint32_t lastUpdateMicros = 0;
bool observerLocked = false;

PIController idController = {0.0f, 0.0f, 0.0f, MAXIMUM_VOLTAGE};
PIController iqController = {0.0f, 0.0f, 0.0f, MAXIMUM_VOLTAGE};

float wrapAngle(float angle) {
  while (angle >= TWO_PI) {
    angle -= TWO_PI;
  }
  while (angle < 0.0f) {
    angle += TWO_PI;
  }
  return angle;
}

float clamp(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float adcVoltage(uint8_t pin) {
  return analogRead(pin) * ADC_REFERENCE_VOLTAGE / ADC_MAXIMUM_CODE;
}

float readCurrentAmps(uint8_t pin) {
  return (adcVoltage(pin) - CURRENT_BIAS_VOLTAGE) /
         (CURRENT_AMPLIFIER_GAIN * SHUNT_RESISTANCE);
}

PhaseCurrents readCurrents() {
  const float phaseA = readCurrentAmps(CURRENT_PINS[0]);
  const float phaseB = readCurrentAmps(CURRENT_PINS[1]);
  return {phaseA, phaseB, -(phaseA + phaseB)};
}

AlphaBeta clarke(const PhaseCurrents& currents) {
  return {currents.a,
          (currents.a + 2.0f * currents.b) * ONE_OVER_SQRT_THREE};
}

AlphaBeta rotateToDq(const AlphaBeta& value, float angle) {
  const float sine = sinf(angle);
  const float cosine = cosf(angle);
  return {value.alpha * cosine + value.beta * sine,
          -value.alpha * sine + value.beta * cosine};
}

AlphaBeta rotateToAlphaBeta(const AlphaBeta& value, float angle) {
  const float sine = sinf(angle);
  const float cosine = cosf(angle);
  return {value.alpha * cosine - value.beta * sine,
          value.alpha * sine + value.beta * cosine};
}

float runPI(PIController& controller, float error, float dt) {
  controller.integral =
      clamp(controller.integral + controller.ki * error * dt,
            -controller.limit, controller.limit);
  return clamp(controller.kp * error + controller.integral,
               -controller.limit, controller.limit);
}

AlphaBeta commandedPhaseVoltage(uint8_t dutyA, uint8_t dutyB, uint8_t dutyC) {
  const float a = static_cast<float>(dutyA) / 255.0f;
  const float b = static_cast<float>(dutyB) / 255.0f;
  const float c = static_cast<float>(dutyC) / 255.0f;
  return {DC_BUS_VOLTAGE * (a - 0.5f * (b + c)),
          DC_BUS_VOLTAGE * ONE_OVER_SQRT_THREE * (b - c)};
}

void slidingModeObserver(const AlphaBeta& voltage, const AlphaBeta& current, float dt) {
  const float currentErrorAlpha = current.alpha - estimatedCurrentAlpha;
  const float currentErrorBeta = current.beta - estimatedCurrentBeta;
  const float observerGain = 8.0f;
  const float backEmfGain = 0.25f;

  const float correctionAlpha =
      clamp(observerGain * currentErrorAlpha, -1.0f, 1.0f);
  const float correctionBeta =
      clamp(observerGain * currentErrorBeta, -1.0f, 1.0f);

  estimatedCurrentAlpha +=
      (voltage.alpha - MOTOR_RESISTANCE * estimatedCurrentAlpha -
       backEmfAlpha - correctionAlpha) /
      MOTOR_INDUCTANCE * dt;
  estimatedCurrentBeta +=
      (voltage.beta - MOTOR_RESISTANCE * estimatedCurrentBeta -
       backEmfBeta - correctionBeta) /
      MOTOR_INDUCTANCE * dt;

  backEmfAlpha += backEmfGain * correctionAlpha * dt;
  backEmfBeta += backEmfGain * correctionBeta * dt;

  const float emfMagnitude = hypotf(backEmfAlpha, backEmfBeta);
  if (emfMagnitude > 0.2f) {
    const float measuredAngle = atan2f(-backEmfAlpha, backEmfBeta);
    const float phaseError = atan2f(sinf(measuredAngle - observerAngle), cosf(measuredAngle - observerAngle));
    observerSpeed += 1000.0f * phaseError * dt;
    observerSpeed = clamp(observerSpeed, -20000.0f, 20000.0f);
    observerAngle = wrapAngle(observerAngle + observerSpeed * dt);
    observerLocked = true;
  }
}

void calculateSvpwm(const AlphaBeta& voltage, uint8_t& dutyA, uint8_t& dutyB, uint8_t& dutyC) {
  const float normalizedAlpha = clamp(voltage.alpha / DC_BUS_VOLTAGE, -0.5f, 0.5f);
  const float normalizedBeta = clamp(voltage.beta / DC_BUS_VOLTAGE, -0.5f, 0.5f);
  const float phaseA = normalizedAlpha;
  const float phaseB = -0.5f * normalizedAlpha + SQRT_THREE_OVER_TWO * normalizedBeta;
  const float phaseC = -0.5f * normalizedAlpha - SQRT_THREE_OVER_TWO * normalizedBeta;
  const float maximum = fmaxf(phaseA, fmaxf(phaseB, phaseC));
  const float minimum = fminf(phaseA, fminf(phaseB, phaseC));
  const float commonMode = -0.5f * (maximum + minimum);

  dutyA = static_cast<uint8_t>(clamp((phaseA + commonMode + 0.5f) * 255.0f, 0.0f, 255.0f));
  dutyB = static_cast<uint8_t>(clamp((phaseB + commonMode + 0.5f) * 255.0f, 0.0f, 255.0f));
  dutyC = static_cast<uint8_t>(clamp((phaseC + commonMode + 0.5f) * 255.0f, 0.0f, 255.0f));
}

void updateControl(float dt) {
  const PhaseCurrents currents = readCurrents();
  const AlphaBeta currentAlphaBeta = clarke(currents);
  const AlphaBeta voltage = commandedPhaseVoltage(128, 128, 128);
  slidingModeObserver(voltage, currentAlphaBeta, dt);

  if (micros() - startTimeMicros < ALIGNMENT_TIME_MICROS) {
    electricalAngle = 0.0f;
  } else if (!observerLocked) {
    openLoopSpeed = fminf(openLoopSpeed + OPEN_LOOP_ACCELERATION * dt,
                          2000.0f);
    electricalAngle = wrapAngle(electricalAngle + openLoopSpeed * dt);
  } else {
    electricalAngle = observerAngle;
  }

  const AlphaBeta dqCurrent = rotateToDq(currentAlphaBeta, electricalAngle);
  const float vd = runPI(idController, -dqCurrent.alpha, dt);
  const float vq = runPI(iqController, TEST_IQ_AMPS - dqCurrent.beta, dt);
  const AlphaBeta alphaBetaVoltage = rotateToAlphaBeta({vd, vq}, electricalAngle);

  uint8_t dutyA;
  uint8_t dutyB;
  uint8_t dutyC;
  calculateSvpwm(alphaBetaVoltage, dutyA, dutyB, dutyC);

  if (FOC_OUTPUT_ENABLED) {
    motor_pwm::setDuty(dutyA, dutyB, dutyC);
  } else {
    motor_pwm::disable();
  }
}
}  // namespace

void begin() {
  analogReadResolution(12);
  idController.integral = 0.0f;
  idController.kp = 0.0f;
  idController.ki = MOTOR_INDUCTANCE * CONTROL_FREQUENCY;
  idController.limit = MAXIMUM_VOLTAGE;
  iqController.integral = 0.0f;
  iqController.kp = 0.0f;
  iqController.ki = MOTOR_INDUCTANCE * CONTROL_FREQUENCY;
  iqController.limit = MAXIMUM_VOLTAGE;
  startTimeMicros = micros();
  lastUpdateMicros = startTimeMicros;
}

void update() {
  const uint32_t now = micros();
  if (now - lastUpdateMicros < CONTROL_PERIOD_MICROS) {
    return;
  }
  lastUpdateMicros += CONTROL_PERIOD_MICROS;
  updateControl(CONTROL_PERIOD);
}

}  // namespace sensorless_foc
