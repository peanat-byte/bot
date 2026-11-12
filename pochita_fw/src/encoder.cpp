#include "encoder.h"

#include <Arduino.h>

namespace encoder {
namespace {
constexpr uint8_t LEFT_ENC_A = 21;
constexpr uint8_t LEFT_ENC_B = 38;
constexpr uint8_t RIGHT_ENC_A = 20;
constexpr uint8_t RIGHT_ENC_B = 37;
constexpr float TICKS_PER_MECHANICAL_REVOLUTION = 1024.0f;

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

constexpr int8_t QUADRATURE_DELTAS[16] = {0,  -1, 1,  0, 1,  0, 0,  -1, -1, 0,  0,  1, 0, 1, -1, 0};

void updateLeftEncoder() {
  const uint8_t state = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  leftCount += QUADRATURE_DELTAS[(leftState << 2) | state];
  leftState = state;
}

void updateRightEncoder() {
  const uint8_t state = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
  rightCount += QUADRATURE_DELTAS[(rightState << 2) | state];
  rightState = state;
}
}  // namespace

void begin() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  leftState = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  rightState = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), updateRightEncoder, CHANGE);
}

int32_t leftPosition() {
  noInterrupts();
  const int32_t position = leftCount;
  interrupts();
  return position;
}

int32_t rightPosition() {
  noInterrupts();
  const int32_t position = rightCount;
  interrupts();
  return position;
}

float leftAngleRadians() {
  return static_cast<float>(leftPosition()) * TWO_PI / TICKS_PER_MECHANICAL_REVOLUTION;
}

float rightAngleRadians() {
  return static_cast<float>(rightPosition()) * TWO_PI / TICKS_PER_MECHANICAL_REVOLUTION;
}
}  // namespace encoder
