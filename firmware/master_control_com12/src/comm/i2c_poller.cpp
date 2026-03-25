// Этот файл реализует минимальный I2C poller нового master.
// Он отвечает за Wire.begin и ping двух целевых плат по таймеру.
// Его роль: поддерживать актуальный online/offline статус плат для MQTT.

#include "comm/i2c_poller.h"

#include <Wire.h>

#include "pins.h"

namespace {

constexpr unsigned long I2C_POLL_INTERVAL_MS = 1000;

unsigned long g_lastI2cPollMs = 0;

bool probeI2cDevice(uint8_t id) {
    // Упрощенная проверка: считаем устройство доступным, если адрес ACK на шине I2C.
    // Это проверка только присутствия/доступности, без валидации протокола устройства.
    Wire.beginTransmission(id);
    return Wire.endTransmission() == 0;
}

}  // namespace

void beginI2c() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
}

void pollBoards(RuntimeState &state, unsigned long nowMs) {
    if ((nowMs - g_lastI2cPollMs) < I2C_POLL_INTERVAL_MS) {
        return;
    }

    g_lastI2cPollMs = nowMs;

    bool changed = false;
    for (BoardState &board : state.boards) {
        const bool online = probeI2cDevice(board.id);
        if (board.online != online) {
            board.online = online;
            changed = true;
        }
    }

    if (changed) {
        state.forceStatusPublish = true;
    }
}
