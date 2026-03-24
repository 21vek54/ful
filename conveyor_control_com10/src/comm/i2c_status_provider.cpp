#include "comm/i2c_status_provider.h"

#include <Arduino.h>
#include <string.h>

#include "app/conveyor_status_runtime.h"
#include "legacy/program1.h"

namespace {

constexpr uint16_t DEVICE_KIND_CONVEYOR = 1;

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
    const app::ConveyorStatusSnapshot statusSnapshot = app::readConveyorStatusSnapshot();

    memset(&frame, 0, sizeof(frame));
    frame.magic = I2C_FRAME_MAGIC;
    frame.protoVer = I2C_FRAME_PROTO_VER;
    frame.deviceKind = DEVICE_KIND_CONVEYOR;
    frame.deviceIdEcho = I2C_DEVICE_ADDRESS;
    frame.statusWord = statusSnapshot.statusWord;
    frame.errorWord = statusSnapshot.errorWord;
    frame.extra0 = statusSnapshot.extra0;
    frame.extra1 = statusSnapshot.extra1;
    frame.extra2 = static_cast<uint16_t>(program1GetStateCode()) |
                   (static_cast<uint16_t>(program1GetPassIndex()) << 8);
    frame.extra3 = static_cast<uint16_t>(program1GetBatchReadySequence() & 0x7FFFU);
    if (program1IsBatchReadyForManipulator()) {
        frame.extra3 |= 0x8000U;
    }
    frame.heartbeatMs = millis();
    frame.checksum = checksumI2cStatusFrame(frame);
}

} // namespace comm
