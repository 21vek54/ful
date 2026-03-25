#pragma once

#include <stdint.h>

#include "comm/i2c_bus.h"

namespace comm {

void fillI2cStatusFrame(I2cStatusFrame &frame);

} // namespace comm
