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
// constexpr uint8_t VOLTAGE_PINS[] = {4, 5};

constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float ADC_MAXIMUM_CODE = 4095.0f;
constexpr float CURRENT_BIAS_VOLTAGE = 1.65f;
constexpr float CURRENT_AMPLIFIER_GAIN = 10.0f;
constexpr float SHUNT_RESISTANCE = 0.002f;
constexpr float MOTOR_RESISTANCE = 45.0f;
constexpr float MOTOR_INDUCTANCE = 1.0e-6f;  // Placeholder.
constexpr float MOTOR_FLUX_LINKAGE = 0.01f;  // Placeholder.
constexpr float DC_BUS_VOLTAGE = 24.0f;
constexpr uint8_t POLE_PAIRS = 7;

constexpr float CONTROL_FREQUENCY = 20000.0f;
constexpr float CONTROL_PERIOD = 1.0f / CONTROL_FREQUENCY;
constexpr uint32_t CONTROL_PERIOD_MICROS = 1000000UL / 20000UL;
constexpr float ONE_OVER_SQRT_THREE = 0.57735026919f;
constexpr float SQRT_THREE_OVER_TWO = 0.86602540378f;
constexpr float BEMF_FILTER_TIME_CONSTANT = 0.001f;
constexpr float PLL_PROPORTIONAL_GAIN = 100.0f;
constexpr float PLL_INTEGRAL_GAIN = 1000.0f;

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
float filteredBackEmfD = 0.0f;
float filteredBackEmfQ = 0.0f;
float previousCurrentAlpha = 0.0f;
float previousCurrentBeta = 0.0f;
float pllCorrection = 0.0f;
AlphaBeta previousVoltage = {0.0f, 0.0f};
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

void pllEstimator(const AlphaBeta& voltage, const AlphaBeta& current, float dt) {
  const AlphaBeta currentDerivative = {
      (current.alpha - previousCurrentAlpha) / dt,
      (current.beta - previousCurrentBeta) / dt};
  previousCurrentAlpha = current.alpha;
  previousCurrentBeta = current.beta;

  const AlphaBeta backEmf = {
      voltage.alpha - MOTOR_RESISTANCE * current.alpha -
          MOTOR_INDUCTANCE * currentDerivative.alpha,
      voltage.beta - MOTOR_RESISTANCE * current.beta -
          MOTOR_INDUCTANCE * currentDerivative.beta};
  const float filterCoefficient =
      dt / (BEMF_FILTER_TIME_CONSTANT + dt);
  const AlphaBeta backEmfDq = rotateToDq(backEmf, observerAngle);
  filteredBackEmfD +=
      filterCoefficient * (backEmfDq.alpha - filteredBackEmfD);
  filteredBackEmfQ +=
      filterCoefficient * (backEmfDq.beta - filteredBackEmfQ);

  const float emfMagnitude =
      hypotf(filteredBackEmfD, filteredBackEmfQ);
  if (emfMagnitude > 0.2f) {
    const float phaseError = filteredBackEmfD;
    pllCorrection += PLL_INTEGRAL_GAIN * phaseError * dt;
    pllCorrection = clamp(pllCorrection, -20000.0f, 20000.0f);
    observerSpeed =
        filteredBackEmfQ / MOTOR_FLUX_LINKAGE +
        PLL_PROPORTIONAL_GAIN * phaseError + pllCorrection;
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
  pllEstimator(previousVoltage, currentAlphaBeta, dt);

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
  previousVoltage = alphaBetaVoltage;

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
  electricalAngle = 0.0f;
  openLoopSpeed = 0.0f;
  observerAngle = 0.0f;
  observerSpeed = 0.0f;
  filteredBackEmfD = 0.0f;
  filteredBackEmfQ = 0.0f;
  previousCurrentAlpha = 0.0f;
  previousCurrentBeta = 0.0f;
  pllCorrection = 0.0f;
  previousVoltage = {0.0f, 0.0f};
  observerLocked = false;
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
