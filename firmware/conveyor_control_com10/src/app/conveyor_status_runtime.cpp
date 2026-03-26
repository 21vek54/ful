#include "app/conveyor_status_runtime.h"

#include <Arduino.h>

#include "core/pins.h"
#include "groups/infeed/infeed_group.h"
#include "groups/outfeed/outfeed_group.h"
#include "groups/sealer/sealer_group.h"
#include "legacy/program1.h"
#include "legacy/shift_control.h"

namespace {

bool isConveyorFlagUp()
{
    return digitalRead(PIN_FLAG) == HIGH;
}

bool isConveyorSensorConflict()
{
    return shiftIsSensorZTriggered() && shiftIsSensorCTriggered();
}

} // namespace

namespace app {

ConveyorStatusInputs readConveyorStatusInputs()
{
    ConveyorStatusInputs inputs = {};
    inputs.infeed = groups::infeed::readInfeedStatus();
    inputs.outfeed = groups::outfeed::readOutfeedStatus();
    inputs.sealer = groups::sealer::readSealerStatus();
    inputs.sensorConflict = isConveyorSensorConflict();
    inputs.program1Active = program1GetStateCode() != 0;
    inputs.batchReady = program1IsBatchReadyForManipulator();
    inputs.flagUp = isConveyorFlagUp();
    return inputs;
}

ConveyorStatusSnapshot readConveyorStatusSnapshot()
{
    return buildConveyorStatusSnapshot(readConveyorStatusInputs());
}

} // namespace app
