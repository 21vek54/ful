#include "comm/i2c_status_provider.h"

#include <Arduino.h>
#include <string.h>

#include "app/conveyor_status_runtime.h"
#include "app/pause_contract.h"
#include "legacy/program1.h"

namespace {

constexpr uint16_t DEVICE_KIND_CONVEYOR = 1;
constexpr uint16_t FEED_SIDE_EMPTY_STRICT_BIT = 1U << 14;
constexpr uint16_t FEED_SIDE_EMPTY_VALID_BIT = 1U << 15;

uint32_t checksumI2cStatusFrame(const comm::I2cStatusFrame &frame)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
    uint32_t sum = 0x13572468UL;
    for (size_t i = 0; i < sizeof(comm::I2cStatusFrame) - sizeof(frame.checksum); i++) {
        sum = (sum << 5) | (sum >> 27);
        sum ^= bytes[i];
    }
    return sum;
}

} // namespace

namespace comm {

void fillI2cStatusFrame(I2cStatusFrame &frame)
{
    const app::ConveyorStatusInputs statusInputs = app::readConveyorStatusInputs();
    const app::ConveyorStatusSnapshot statusSnapshot = app::buildConveyorStatusSnapshot(statusInputs);

    memset(&frame, 0, sizeof(frame));
    frame.magic = I2C_FRAME_MAGIC;
    frame.protoVer = I2C_FRAME_PROTO_VER;
    frame.deviceKind = DEVICE_KIND_CONVEYOR;
    frame.deviceIdEcho = I2C_DEVICE_ADDRESS;
    frame.statusWord = statusSnapshot.statusWord;
    frame.errorWord = statusSnapshot.errorWord;
    frame.extra0 = statusSnapshot.extra0;
    frame.extra1 = statusSnapshot.extra1;
    frame.extra2 = static_cast<uint16_t>(statusInputs.program1StateCode) |
                   (static_cast<uint16_t>(statusInputs.program1PassIndex & 0x3FU) << 8);
    if (statusInputs.feedSideEmptyStrict) {
        frame.extra2 |= FEED_SIDE_EMPTY_STRICT_BIT;
    }
    if (statusInputs.feedSideEmptyValid) {
        frame.extra2 |= FEED_SIDE_EMPTY_VALID_BIT;
    }

    frame.extra3 = static_cast<uint16_t>(program1GetBatchReadySequence() & 0x7FFFU);
    if (statusInputs.batchReady) {
        frame.extra3 |= 0x8000U;
    }
    frame.heartbeatMs = millis();
    app::pauseContractApplyToI2cFrame(frame);
    frame.checksum = checksumI2cStatusFrame(frame);
}

} // namespace comm
