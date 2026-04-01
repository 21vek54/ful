#include "comm/i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "app/cli_handler.h"
#include "core/pins.h"
#include "comm/i2c_status_provider.h"

namespace {

constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr int PIN_I2C_SDA = PIN_RS485_RX;
constexpr int PIN_I2C_SCL = PIN_RS485_TX;
constexpr uint8_t I2C_MANAGED_POKE_CMD = 0xA5;
constexpr uint8_t I2C_MANAGED_TEXT_CMD = 0xA6;
constexpr size_t I2C_MANAGED_TEXT_MAX_LEN = 64;
constexpr uint8_t I2C_MANAGED_QUEUE_SIZE = 4;

struct ManagedCommandSlot {
    char text[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};
    uint8_t len = 0;
};

ManagedCommandSlot g_i2cCommandQueue[I2C_MANAGED_QUEUE_SIZE] = {};
volatile uint8_t g_i2cCommandHead = 0;
volatile uint8_t g_i2cCommandTail = 0;
volatile uint8_t g_i2cCommandCount = 0;
volatile uint32_t g_i2cDroppedCommandCount = 0;
uint32_t g_i2cDroppedCommandCountLogged = 0;
comm::I2cBusCounters g_i2cCounters = {};
comm::I2cLinkState g_i2cLinkState = comm::I2cLinkState::Uninitialized;

void queueManagedCommand(const char *text, uint8_t len)
{
    if (text == nullptr || len == 0) {
        return;
    }

    if (g_i2cCommandCount >= I2C_MANAGED_QUEUE_SIZE) {
        g_i2cDroppedCommandCount = static_cast<uint32_t>(g_i2cDroppedCommandCount + 1U);
        return;
    }

    ManagedCommandSlot &slot = g_i2cCommandQueue[g_i2cCommandTail];
    memcpy(slot.text, text, len);
    slot.text[len] = '\0';
    slot.len = len;

    g_i2cCommandTail =
        static_cast<uint8_t>((static_cast<uint8_t>(g_i2cCommandTail + 1U)) % I2C_MANAGED_QUEUE_SIZE);
    g_i2cCommandCount = static_cast<uint8_t>(g_i2cCommandCount + 1U);
}

bool popManagedCommand(char *out, uint8_t &lenOut)
{
    lenOut = 0;
    if (out == nullptr || g_i2cCommandCount == 0) {
        return false;
    }

    ManagedCommandSlot &slot = g_i2cCommandQueue[g_i2cCommandHead];
    lenOut = slot.len;
    if (lenOut == 0) {
        slot.text[0] = '\0';
    } else {
        memcpy(out, slot.text, static_cast<size_t>(lenOut) + 1U);
    }

    slot.text[0] = '\0';
    slot.len = 0;
    g_i2cCommandHead =
        static_cast<uint8_t>((static_cast<uint8_t>(g_i2cCommandHead + 1U)) % I2C_MANAGED_QUEUE_SIZE);
    g_i2cCommandCount = static_cast<uint8_t>(g_i2cCommandCount - 1U);
    return true;
}

void onI2cReceive(int len)
{
    g_i2cCounters.lastRxLen = static_cast<uint8_t>(len < 0 ? 0 : len);
    if (len <= 0) {
        return;
    }

    g_i2cCounters.rxCount++;
    const int opcode = Wire.read();
    if (opcode == I2C_MANAGED_POKE_CMD) {
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        return;
    }

    if (opcode == I2C_MANAGED_TEXT_CMD) {
        char local[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};
        size_t count = 0;
        while (Wire.available() > 0 && count < I2C_MANAGED_TEXT_MAX_LEN) {
            local[count++] = static_cast<char>(Wire.read());
        }
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        local[count] = '\0';
        queueManagedCommand(local, static_cast<uint8_t>(count));
        return;
    }

    while (Wire.available() > 0) {
        (void)Wire.read();
    }
}

void onI2cRequest()
{
    comm::I2cStatusFrame frame = {};
    comm::fillI2cStatusFrame(frame);
    Wire.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
    g_i2cCounters.txCount++;
}

} // namespace

static_assert(sizeof(comm::I2cStatusFrame) == comm::I2C_FRAME_SIZE, "Unexpected I2cStatusFrame size");

namespace comm {

void initI2cBus()
{
    if (!Wire.begin(I2C_DEVICE_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ)) {
        g_i2cLinkState = I2cLinkState::Error;
        return;
    }
    Wire.onReceive(onI2cReceive);
    Wire.onRequest(onI2cRequest);
    g_i2cLinkState = I2cLinkState::Ready;
}

void processI2cBus()
{
    char local[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};
    uint8_t len = 0;
    uint32_t droppedTotal = 0;

    noInterrupts();
    const bool hasCommand = popManagedCommand(local, len);
    droppedTotal = g_i2cDroppedCommandCount;
    interrupts();

    if (droppedTotal != g_i2cDroppedCommandCountLogged) {
        const uint32_t droppedDelta = droppedTotal - g_i2cDroppedCommandCountLogged;
        g_i2cDroppedCommandCountLogged = droppedTotal;
        Serial.print("I2C CMD WARN: dropped ");
        Serial.print(droppedDelta);
        Serial.print(", total=");
        Serial.println(droppedTotal);
    }

    if (!hasCommand || len == 0) {
        return;
    }

    String line(local);
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    Serial.print("I2C CMD: ");
    Serial.println(line);
    handleCommand(line);
}

void printI2cBanner()
{
    Serial.println("I2C status: addr 12, SDA=GPIO16, SCL=GPIO17.");
}

I2cLinkState getI2cLinkState()
{
    return g_i2cLinkState;
}

I2cBusCounters getI2cBusCounters()
{
    return g_i2cCounters;
}

} // namespace comm
