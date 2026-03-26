#pragma once

#include <stdint.h>

#include "groups/infeed/infeed_group.h"
#include "groups/outfeed/outfeed_group.h"
#include "groups/sealer/sealer_group.h"

namespace app {

struct ConveyorStatusInputs {
    groups::infeed::InfeedStatus infeed = {};
    groups::outfeed::OutfeedStatus outfeed = {};
    groups::sealer::SealerStatus sealer = {};
    bool sensorConflict = false;
    bool program1Active = false;
    bool batchReady = false;
    bool flagUp = false;
};

struct ConveyorStatusSnapshot {
    uint16_t statusWord = 0;
    uint16_t errorWord = 0;
    uint16_t extra0 = 0;
    uint16_t extra1 = 0;
};

ConveyorStatusSnapshot buildConveyorStatusSnapshot(const ConveyorStatusInputs &inputs);
bool isConveyorBusy(const ConveyorStatusInputs &inputs);
bool isConveyorBusy(const ConveyorStatusSnapshot &snapshot);
uint16_t encodeVfdTickDuration10Ms(uint32_t durationMs);

} // namespace app
