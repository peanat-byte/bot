#pragma once

#include <stdint.h>

namespace drive {

void begin();
void setJoystickOutput(uint16_t horizontal, uint16_t vertical);
void stop();

}  // namespace drive