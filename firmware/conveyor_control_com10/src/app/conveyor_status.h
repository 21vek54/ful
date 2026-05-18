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
    uint8_t program1StateCode = 0;
    uint8_t program1PassIndex = 0;
    bool batchReady = false;
    bool feedSideEmptyStrict = false;
    bool feedSideEmptyValid = false;
    int8_t feedBufferCount = -1;
    bool feedBufferCountKnown = false;
    int8_t feedIn2Pairs = -1;
    bool feedIn2PairsKnown = false;
    int8_t sealerPlateCount = -1;
    bool sealerPlateCountKnown = false;
    bool program1AbortRecoveryRequired = false;
    bool flagUp = false;
    bool post7ManualRecoveryRequired = false;
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
