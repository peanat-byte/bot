#pragma once

#include <Arduino.h>

namespace bluetooth_receiver {

struct __attribute__((packed)) ControlPacket {
  uint16_t leftHorizontal;
  uint16_t leftVertical;
  uint16_t rightHorizontal;
  uint16_t rightVertical;
  uint8_t switches;
};

static_assert(sizeof(ControlPacket) == 9, "Unexpected control packet size");

void begin();
void update();
bool isConnected();
bool takePacket(ControlPacket& packet);

}  // namespace bluetooth_receiver
