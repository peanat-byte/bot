#pragma once

#include <stdint.h>

namespace encoder {

void begin();
int32_t leftPosition();
int32_t rightPosition();
float leftAngleRadians();
float rightAngleRadians();

}  // namespace encoder
