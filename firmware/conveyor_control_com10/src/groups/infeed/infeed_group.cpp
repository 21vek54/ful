#include "groups/infeed/infeed_group.h"

#include "legacy/program1.h"
#include "legacy/shift_control.h"

// Thin runtime hooks currently implemented in main.cpp.
bool isPlateAtSensorFiltered();

namespace groups::infeed {

void initInfeedGroup()
{
}

void processInfeedGroup()
{
}

InfeedStatus readInfeedStatus()
{
    InfeedStatus status = {};
    status.sensorPlateDetected = isPlateAtSensorFiltered();
    status.shiftCalibrated = shiftIsCalibrated();

    if (program1IsPositionalMotionActive() || shiftHookIsPositionalMotionActive()) {
        status.runState = InfeedRunState::Positioning;
    } else if (program1IsManualMotionActive() || program1IsCycle2Active() ||
               shiftHookIsMotionActive() || shiftHookIsCycle2Active()) {
        status.runState = InfeedRunState::Running;
    } else {
        status.runState = InfeedRunState::Idle;
    }

    status.busy = status.runState != InfeedRunState::Idle;
    status.ready = status.shiftCalibrated && !status.busy;
    status.alarm = false;

    return status;
}

} // namespace groups::infeed
