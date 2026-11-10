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
constexpr uint8_t currentPins[] = {1, 2};
constexpr uint8_t voltagePins[] = {4, 5};
constexpr float adcReferenceVoltage = 3.3f;
constexpr float adcMaximumCode = 4095.0f;
constexpr float currentBiasVoltage = 1.65f;
constexpr float currentAmplifierGain = 10.0f;
constexpr float shuntResistance = 0.002f;
constexpr float motorResistance = 45.0f;
constexpr float motorInductance = 1.0e-6f;  // Placeholder.
constexpr float dcBusVoltage = 24.0f;
constexpr uint8_t polePairs = 7;

constexpr float controlFrequency = 20000.0f;
constexpr float controlPeriod = 1.0f / controlFrequency;
constexpr uint32_t controlPeriodMicros = 1000000UL / 20000UL;
constexpr float twoPi = 6.28318530718f;
constexpr float oneOverSqrtThree = 0.57735026919f;
constexpr float sqrtThreeOverTwo = 0.86602540378f;

// Keep this false until the observer and protections are validated on hardware.
constexpr bool focOutputEnabled = false;
constexpr float testIqAmps = 0.5f;
constexpr float maximumVoltage = dcBusVoltage * 0.45f;
constexpr uint32_t alignmentTimeMicros = 300000;
constexpr float openLoopStartSpeed = 20.0f;
constexpr float openLoopAcceleration = 3000.0f;

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
  float integral = 0.0f;
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

PIController idController{0.0f, 0.0f, 0.0f, maximumVoltage};
PIController iqController{0.0f, 0.0f, 0.0f, maximumVoltage};

float wrapAngle(float angle) {
  while (angle >= twoPi) {
    angle -= twoPi;
  }
  while (angle < 0.0f) {
    angle += twoPi;
  }
  return angle;
}

float clamp(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float adcVoltage(uint8_t pin) {
  return analogRead(pin) * adcReferenceVoltage / adcMaximumCode;
}

float readCurrentAmps(uint8_t pin) {
  return (adcVoltage(pin) - currentBiasVoltage) /
         (currentAmplifierGain * shuntResistance);
}

PhaseCurrents readCurrents() {
  const float phaseA = readCurrentAmps(currentPins[0]);
  const float phaseB = readCurrentAmps(currentPins[1]);
  return {phaseA, phaseB, -(phaseA + phaseB)};
}

AlphaBeta clarke(const PhaseCurrents& currents) {
  return {currents.a, (currents.a + 2.0f * currents.b) * oneOverSqrtThree};
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

AlphaBeta commandedPhaseVoltage(uint8_t dutyA, uint8_t dutyB,
                                uint8_t dutyC) {
  const float a = static_cast<float>(dutyA) / 255.0f;
  const float b = static_cast<float>(dutyB) / 255.0f;
  const float c = static_cast<float>(dutyC) / 255.0f;
  return {dcBusVoltage * (a - 0.5f * (b + c)),
          dcBusVoltage * oneOverSqrtThree * (b - c)};
}

void slidingModeObserver(const AlphaBeta& voltage, const AlphaBeta& current,
                         float dt) {
  const float currentErrorAlpha = current.alpha - estimatedCurrentAlpha;
  const float currentErrorBeta = current.beta - estimatedCurrentBeta;
  const float observerGain = 8.0f;
  const float backEmfGain = 0.25f;

  const float correctionAlpha =
      clamp(observerGain * currentErrorAlpha, -1.0f, 1.0f);
  const float correctionBeta =
      clamp(observerGain * currentErrorBeta, -1.0f, 1.0f);

  estimatedCurrentAlpha +=
      (voltage.alpha - motorResistance * estimatedCurrentAlpha -
       backEmfAlpha - correctionAlpha) /
      motorInductance * dt;
  estimatedCurrentBeta +=
      (voltage.beta - motorResistance * estimatedCurrentBeta -
       backEmfBeta - correctionBeta) /
      motorInductance * dt;

  backEmfAlpha += backEmfGain * correctionAlpha * dt;
  backEmfBeta += backEmfGain * correctionBeta * dt;

  const float emfMagnitude = hypotf(backEmfAlpha, backEmfBeta);
  if (emfMagnitude > 0.2f) {
    const float measuredAngle =
        atan2f(-backEmfAlpha, backEmfBeta);
    const float phaseError =
        atan2f(sinf(measuredAngle - observerAngle),
               cosf(measuredAngle - observerAngle));
    observerSpeed += 1000.0f * phaseError * dt;
    observerSpeed = clamp(observerSpeed, -20000.0f, 20000.0f);
    observerAngle = wrapAngle(observerAngle + observerSpeed * dt);
    observerLocked = true;
  }
}

void calculateSvpwm(const AlphaBeta& voltage, uint8_t& dutyA,
                    uint8_t& dutyB, uint8_t& dutyC) {
  const float normalizedAlpha = clamp(voltage.alpha / dcBusVoltage, -0.5f, 0.5f);
  const float normalizedBeta = clamp(voltage.beta / dcBusVoltage, -0.5f, 0.5f);
  const float phaseA = normalizedAlpha;
  const float phaseB = -0.5f * normalizedAlpha +
                       sqrtThreeOverTwo * normalizedBeta;
  const float phaseC = -0.5f * normalizedAlpha -
                       sqrtThreeOverTwo * normalizedBeta;
  const float maximum = fmaxf(phaseA, fmaxf(phaseB, phaseC));
  const float minimum = fminf(phaseA, fminf(phaseB, phaseC));
  const float commonMode = -0.5f * (maximum + minimum);

  dutyA = static_cast<uint8_t>(clamp((phaseA + commonMode + 0.5f) * 255.0f,
                                     0.0f, 255.0f));
  dutyB = static_cast<uint8_t>(clamp((phaseB + commonMode + 0.5f) * 255.0f,
                                     0.0f, 255.0f));
  dutyC = static_cast<uint8_t>(clamp((phaseC + commonMode + 0.5f) * 255.0f,
                                     0.0f, 255.0f));
}

void updateControl(float dt) {
  const PhaseCurrents currents = readCurrents();
  const AlphaBeta currentAlphaBeta = clarke(currents);
  const AlphaBeta voltage = commandedPhaseVoltage(128, 128, 128);
  slidingModeObserver(voltage, currentAlphaBeta, dt);

  if (micros() - startTimeMicros < alignmentTimeMicros) {
    electricalAngle = 0.0f;
  } else if (!observerLocked) {
    openLoopSpeed = fminf(openLoopSpeed + openLoopAcceleration * dt,
                          2000.0f);
    electricalAngle = wrapAngle(electricalAngle + openLoopSpeed * dt);
  } else {
    electricalAngle = observerAngle;
  }

  const AlphaBeta dqCurrent = rotateToDq(currentAlphaBeta, electricalAngle);
  const float vd = runPI(idController, -dqCurrent.alpha, dt);
  const float vq = runPI(iqController, testIqAmps - dqCurrent.beta, dt);
  const AlphaBeta alphaBetaVoltage =
      rotateToAlphaBeta({vd, vq}, electricalAngle);

  uint8_t dutyA;
  uint8_t dutyB;
  uint8_t dutyC;
  calculateSvpwm(alphaBetaVoltage, dutyA, dutyB, dutyC);

  if (focOutputEnabled) {
    motor_pwm::setDuty(dutyA, dutyB, dutyC);
  } else {
    motor_pwm::disable();
  }
}
}  // namespace

void begin() {
  analogReadResolution(12);
  idController = {0.0f, motorInductance * controlFrequency, 0.0f,
                  maximumVoltage};
  iqController = {0.0f, motorInductance * controlFrequency, 0.0f,
                  maximumVoltage};
  startTimeMicros = micros();
  lastUpdateMicros = startTimeMicros;
}

void update() {
  const uint32_t now = micros();
  if (now - lastUpdateMicros < controlPeriodMicros) {
    return;
  }
  lastUpdateMicros += controlPeriodMicros;
  updateControl(controlPeriod);
}

}  // namespace sensorless_foc
