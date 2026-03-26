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

volatile bool g_i2cCommandPending = false;
volatile uint8_t g_i2cPendingCommandLen = 0;
char g_i2cPendingCommand[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};
comm::I2cBusCounters g_i2cCounters = {};
comm::I2cLinkState g_i2cLinkState = comm::I2cLinkState::Uninitialized;

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
        size_t count = 0;
        while (Wire.available() > 0 && count < I2C_MANAGED_TEXT_MAX_LEN) {
            g_i2cPendingCommand[count++] = static_cast<char>(Wire.read());
        }
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        g_i2cPendingCommand[count] = '\0';
        g_i2cPendingCommandLen = static_cast<uint8_t>(count);
        g_i2cCommandPending = count > 0;
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
    if (!g_i2cCommandPending) {
        return;
    }

    char local[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};
    noInterrupts();
    const uint8_t len = g_i2cPendingCommandLen;
    memcpy(local, g_i2cPendingCommand, len);
    local[len] = '\0';
    g_i2cPendingCommand[0] = '\0';
    g_i2cPendingCommandLen = 0;
    g_i2cCommandPending = false;
    interrupts();

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
