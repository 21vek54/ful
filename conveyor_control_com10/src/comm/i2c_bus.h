#pragma once

#include <stddef.h>
#include <stdint.h>

namespace comm {

enum class I2cLinkState : uint8_t {
    Uninitialized = 0,
    Ready,
    Error
};

struct I2cBusCounters {
    uint32_t rxCount = 0;
    uint32_t txCount = 0;
    uint8_t lastRxLen = 0;
};

struct __attribute__((packed)) I2cStatusFrame {
    uint16_t magic;
    uint16_t protoVer;
    uint16_t deviceKind;
    uint16_t deviceIdEcho;
    uint16_t statusWord;
    uint16_t errorWord;
    uint16_t extra0;
    uint16_t extra1;
    uint16_t extra2;
    uint16_t extra3;
    uint32_t heartbeatMs;
    uint32_t checksum;
};

constexpr uint16_t I2C_FRAME_MAGIC = 0x4659;
constexpr uint16_t I2C_FRAME_PROTO_VER = 1;
constexpr size_t I2C_FRAME_SIZE = 28;
constexpr uint8_t I2C_DEVICE_ADDRESS = 12;

void initI2cBus();
void processI2cBus();
void printI2cBanner();
I2cLinkState getI2cLinkState();
I2cBusCounters getI2cBusCounters();

} // namespace comm
