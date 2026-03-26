#pragma once

#include "app/conveyor_status.h"

namespace app {

ConveyorStatusInputs readConveyorStatusInputs();
ConveyorStatusSnapshot readConveyorStatusSnapshot();

} // namespace app
