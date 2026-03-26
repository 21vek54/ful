#include "groups/sealer/sealer_group.h"

#include <Arduino.h>

#include "core/pins.h"

namespace groups::sealer {

namespace {

constexpr bool SEAL_START_ACTIVE_LEVEL = LOW;   // Типовой модуль реле для ESP32
constexpr bool SEAL_START_INACTIVE_LEVEL = HIGH;
constexpr bool SEAL_DONE_ACTIVE_LEVEL = LOW;    // GPIO34 без внутренней подтяжки, нужна внешняя
constexpr uint32_t SEAL_START_PULSE_MS_DEFAULT = 300U;
constexpr uint32_t SEAL_START_PULSE_MS_MIN = 50U;
constexpr uint32_t SEAL_START_PULSE_MS_MAX = 5000U;

struct SealIoState {
    bool startPulseActive = false;
    bool startOutputActive = false;
    bool doneLastActive = false;
    uint32_t doneLastRiseMs = 0;
    uint32_t startPulseStartedMs = 0;
    uint32_t startPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;
};

SealIoState g_sealIo;

void writeStartOutputRaw(bool active)
{
    digitalWrite(PIN_RELAY_2, active ? SEAL_START_ACTIVE_LEVEL : SEAL_START_INACTIVE_LEVEL);
    g_sealIo.startOutputActive = active;
}

bool isDoneActiveRaw()
{
    return digitalRead(PIN_SENSOR_EXT_1) == SEAL_DONE_ACTIVE_LEVEL;
}

} // namespace

void initSealerGroup()
{
    pinMode(PIN_RELAY_2, OUTPUT);
    pinMode(PIN_SENSOR_EXT_1, INPUT);
    writeStartOutputRaw(false);
    g_sealIo.doneLastActive = isDoneActiveRaw();
}

void processSealerGroup()
{
    const bool doneActive = isDoneActiveRaw();
    if (doneActive != g_sealIo.doneLastActive) {
        g_sealIo.doneLastActive = doneActive;
        if (doneActive) {
            g_sealIo.doneLastRiseMs = millis();
            Serial.println("SEAL: cycle complete input became active.");
        } else {
            Serial.println("SEAL: cycle complete input released.");
        }
    }

    if (!g_sealIo.startPulseActive) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_sealIo.startPulseStartedMs) < g_sealIo.startPulseDurationMs) {
        return;
    }

    writeStartOutputRaw(false);
    g_sealIo.startPulseActive = false;
    Serial.println("SEAL: start pulse finished.");
}

SealerStatus readSealerStatus()
{
    SealerStatus status = {};
    status.startOutputActive = g_sealIo.startOutputActive;
    status.startPulseActive = g_sealIo.startPulseActive;
    status.doneInputActive = isDoneActiveRaw();
    status.startPulseDurationMs = g_sealIo.startPulseDurationMs;

    if (status.startPulseActive) {
        status.runState = SealerRunState::StartPulseActive;
    } else if (status.startOutputActive) {
        status.runState = SealerRunState::WaitDone;
    } else {
        status.runState = SealerRunState::Idle;
    }
    status.busy = status.runState != SealerRunState::Idle;
    status.ready = !status.busy;
    status.alarm = false;

    return status;
}

void setStartOutput(bool active)
{
    g_sealIo.startPulseActive = false;
    writeStartOutputRaw(active);
}

bool startPulse(uint32_t pulseMs)
{
    if (g_sealIo.startPulseActive) {
        return false;
    }
    if (pulseMs < SEAL_START_PULSE_MS_MIN || pulseMs > SEAL_START_PULSE_MS_MAX) {
        return false;
    }

    g_sealIo.startPulseDurationMs = pulseMs;
    g_sealIo.startPulseStartedMs = millis();
    g_sealIo.startPulseActive = true;
    writeStartOutputRaw(true);
    return true;
}

bool isStartPulseActive()
{
    return g_sealIo.startPulseActive;
}

uint32_t getStartPulseDefaultMs()
{
    return SEAL_START_PULSE_MS_DEFAULT;
}

uint32_t getStartPulseMinMs()
{
    return SEAL_START_PULSE_MS_MIN;
}

uint32_t getStartPulseMaxMs()
{
    return SEAL_START_PULSE_MS_MAX;
}

} // namespace groups::sealer
