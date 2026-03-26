#include "app/motion_control.h"

#include <Arduino.h>

#include "app/cycle_control.h"
#include "core/pins.h"
#include "core/settings.h"
#include "legacy/shift_control.h"

namespace {

struct MotionState {
    bool active = false;
    bool pulseHigh = false;
    bool continuous = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t nominalDelayUs = 1000;
    uint32_t currentDelayUs = 1000;
    uint32_t startDelayUs = 1000;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct PosMotionState {
    bool active = false;
    bool pulseHigh = false;
    bool continuous = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t delayUs = core::g_settings.posRunDelayUs;
    uint32_t currentDelayUs = core::g_settings.posRunDelayUs;
    uint32_t startDelayUs = core::g_settings.posRunDelayUs;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
};

MotionState g_motion;
PosMotionState g_posMotion;
SensorFilterState g_sensorFilter;
bool g_sensorStreamEnabled = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_totalStepsCounter = 0;

void updateDebouncedSensorFilter(
    SensorFilterState &state,
    bool rawState,
    uint32_t debounceMs)
{
    const uint32_t nowMs = millis();

    if (!state.initialized) {
        state.initialized = true;
        state.lastRawPlateDetected = rawState;
        state.stablePlateDetected = rawState;
        state.lastRawChangeMs = nowMs;
        return;
    }

    if (rawState != state.lastRawPlateDetected) {
        state.lastRawPlateDetected = rawState;
        state.lastRawChangeMs = nowMs;
    }

    if (rawState != state.stablePlateDetected &&
        (uint32_t)(nowMs - state.lastRawChangeMs) >= debounceMs) {
        state.stablePlateDetected = rawState;
    }
}

uint32_t interpolateDelayUs(uint32_t progressSteps, uint32_t rampSteps, uint32_t startDelayUs, uint32_t nominalDelayUs)
{
    if (rampSteps == 0 || progressSteps >= rampSteps || startDelayUs <= nominalDelayUs) {
        return nominalDelayUs;
    }

    const uint32_t delta = startDelayUs - nominalDelayUs;
    const uint32_t dec = static_cast<uint32_t>((static_cast<uint64_t>(delta) * progressSteps) / rampSteps);
    return startDelayUs - dec;
}

void updateMotionProfileDelay()
{
    if (!g_motion.active) {
        return;
    }

    const uint32_t rampSteps = g_motion.rampSteps == 0 ? 1 : g_motion.rampSteps;

    uint32_t accelProgress = g_motion.stepsDone;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    uint32_t targetDelayUs = interpolateDelayUs(
        accelProgress, rampSteps, g_motion.startDelayUs, g_motion.nominalDelayUs);

    if (g_motion.stepsTotal != UINT32_MAX) {
        uint32_t remainingSteps = 0;
        if (g_motion.stepsTotal > g_motion.stepsDone) {
            remainingSteps = g_motion.stepsTotal - g_motion.stepsDone;
        }

        uint32_t decelProgress = remainingSteps;
        if (decelProgress > rampSteps) {
            decelProgress = rampSteps;
        }

        const uint32_t decelDelayUs = interpolateDelayUs(
            decelProgress, rampSteps, g_motion.startDelayUs, g_motion.nominalDelayUs);
        if (decelDelayUs > targetDelayUs) {
            targetDelayUs = decelDelayUs;
        }
    }

    if (targetDelayUs < g_motion.nominalDelayUs) {
        targetDelayUs = g_motion.nominalDelayUs;
    }
    g_motion.currentDelayUs = targetDelayUs;
}

void writePulseInactive()
{
    digitalWrite(PIN_STEP_PUL, core::hw_config::PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void writePosPulseInactive()
{
    digitalWrite(PIN_POS_PUL, core::hw_config::PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void updatePositionalMotionProfileDelay()
{
    if (!g_posMotion.active) {
        return;
    }

    const uint32_t rampSteps = g_posMotion.rampSteps == 0 ? 1 : g_posMotion.rampSteps;

    uint32_t accelProgress = g_posMotion.stepsDone;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    uint32_t targetDelayUs = interpolateDelayUs(
        accelProgress, rampSteps, g_posMotion.startDelayUs, g_posMotion.delayUs);

    uint32_t remainingSteps = 0;
    if (g_posMotion.stepsTotal > g_posMotion.stepsDone) {
        remainingSteps = g_posMotion.stepsTotal - g_posMotion.stepsDone;
    }

    uint32_t decelProgress = remainingSteps;
    if (decelProgress > rampSteps) {
        decelProgress = rampSteps;
    }

    const uint32_t decelDelayUs = interpolateDelayUs(
        decelProgress, rampSteps, g_posMotion.startDelayUs, g_posMotion.delayUs);
    if (decelDelayUs > targetDelayUs) {
        targetDelayUs = decelDelayUs;
    }

    if (targetDelayUs < g_posMotion.delayUs) {
        targetDelayUs = g_posMotion.delayUs;
    }
    g_posMotion.currentDelayUs = targetDelayUs;
}

bool readSensorPlateRaw()
{
    return digitalRead(PIN_SENSOR) == HIGH;
}

} // namespace

void startMotionProfiled(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (nominalDelayUs == 0) {
        nominalDelayUs = 1;
    }

    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.stepsTotal = stepsTotal;
    g_motion.stepsDone = 0;
    g_motion.nominalDelayUs = nominalDelayUs;
    g_motion.currentDelayUs = nominalDelayUs;

    uint32_t startDelayUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(nominalDelayUs) * core::g_settings.motionStartDelayMultNum) / core::g_settings.motionStartDelayMultDen);
    if (startDelayUs <= nominalDelayUs) {
        startDelayUs = nominalDelayUs + 1;
    }
    g_motion.startDelayUs = startDelayUs;

    g_motion.rampSteps = core::g_settings.motionRampStepsDefault;
    if (stepsTotal != UINT32_MAX) {
        uint32_t halfSteps = stepsTotal / 2U;
        if (halfSteps < g_motion.rampSteps) {
            g_motion.rampSteps = halfSteps;
        }
        if (g_motion.rampSteps == 0) {
            g_motion.rampSteps = 1;
        }
    }

    g_motion.lastPulseStartUs = micros();
    updateMotionProfileDelay();
}

void armMotionStopAfterSteps(uint32_t stepsToStop)
{
    if (!g_motion.active) {
        return;
    }

    if (stepsToStop == 0) {
        stopMotion();
        return;
    }

    if (g_motion.stepsDone > (UINT32_MAX - stepsToStop)) {
        g_motion.stepsTotal = UINT32_MAX;
    } else {
        g_motion.stepsTotal = g_motion.stepsDone + stepsToStop;
    }

    g_motion.rampSteps = core::g_settings.motionRampStepsDefault;
    uint32_t halfSteps = stepsToStop / 2U;
    if (halfSteps < g_motion.rampSteps) {
        g_motion.rampSteps = halfSteps;
    }
    if (g_motion.rampSteps == 0) {
        g_motion.rampSteps = 1;
    }

    updateMotionProfileDelay();
}

void stopPositionalMotion()
{
    writePosPulseInactive();
    g_posMotion.active = false;
    g_posMotion.pulseHigh = false;
    g_posMotion.continuous = false;
    g_posMotion.stepsTotal = 0;
    g_posMotion.stepsDone = 0;
    Serial.println("P: EVA25 stopped.");
}

void stopMotion()
{
    writePulseInactive();
    g_motion.active = false;
    g_motion.pulseHigh = false;
    g_motion.continuous = false;
}

void startConstantMotion(uint32_t steps, uint32_t delayUs)
{
    if (g_motion.active) {
        Serial.println("Ошибка: двигатель уже в движении.");
        return;
    }

    if (steps == 0) {
        Serial.println("Шагов 0: движение не требуется.");
        return;
    }

    startMotionProfiled(steps, delayUs);

    Serial.print("Запуск: шагов=");
    Serial.print(steps);
    Serial.print(", задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс, направление=вправо");
}

void startMainContinuousMotion(uint32_t delayUs)
{
    if (g_motion.active) {
        Serial.println("MAIN: мотор PUL13 уже в движении.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    writePulseInactive();
    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.continuous = true;
    g_motion.stepsTotal = 0;
    g_motion.stepsDone = 0;
    g_motion.nominalDelayUs = delayUs;
    g_motion.currentDelayUs = delayUs;
    g_motion.startDelayUs = delayUs;
    g_motion.rampSteps = 0;
    g_motion.lastPulseStartUs = micros();

    Serial.print("MAIN: непрерывный запуск PUL13, задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (g_posMotion.active) {
        Serial.println("POS: позиционный мотор уже в движении.");
        return;
    }

    if (stepsTotal == 0U) {
        Serial.println("POS: шагов 0, движение не требуется.");
        return;
    }

    if (nominalDelayUs == 0U) {
        nominalDelayUs = 1U;
    }

    writePosPulseInactive();

    g_posMotion.active = true;
    g_posMotion.pulseHigh = false;
    g_posMotion.continuous = false;
    g_posMotion.stepsTotal = stepsTotal;
    g_posMotion.stepsDone = 0;
    g_posMotion.delayUs = nominalDelayUs;
    g_posMotion.currentDelayUs = nominalDelayUs;

    uint32_t startDelayUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(nominalDelayUs) * core::g_settings.motionStartDelayMultNum) / core::g_settings.motionStartDelayMultDen);
    if (startDelayUs <= nominalDelayUs) {
        startDelayUs = nominalDelayUs + 1U;
    }
    g_posMotion.startDelayUs = startDelayUs;

    g_posMotion.rampSteps = core::g_settings.motionRampStepsDefault;
    uint32_t halfSteps = stepsTotal / 2U;
    if (halfSteps < g_posMotion.rampSteps) {
        g_posMotion.rampSteps = halfSteps;
    }
    if (g_posMotion.rampSteps == 0U) {
        g_posMotion.rampSteps = 1U;
    }

    g_posMotion.lastPulseStartUs = micros();
    updatePositionalMotionProfileDelay();

    Serial.print("POS: профилированный ход, шагов=");
    Serial.print(stepsTotal);
    Serial.print(", расстояние=");
    Serial.print(static_cast<float>(stepsTotal) / static_cast<float>(core::g_settings.pulsesPerMm), 1);
    Serial.print(" мм");
    Serial.print(", полка=");
    Serial.print(nominalDelayUs);
    Serial.println(" мкс.");
}

void processPositionalMotion()
{
    if (!g_posMotion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_posMotion.pulseHigh) {
        if ((uint32_t)(nowUs - g_posMotion.lastPulseStartUs) >= g_posMotion.currentDelayUs) {
            digitalWrite(PIN_POS_PUL, core::hw_config::PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_posMotion.pulseHigh = true;
            g_posMotion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_posMotion.lastPulseStartUs) >= core::hw_config::STEP_PULSE_WIDTH_US) {
        writePosPulseInactive();
        g_posMotion.pulseHigh = false;
        g_posMotion.stepsDone++;
        if (g_posMotion.continuous) {
            return;
        }
        if (g_posMotion.stepsDone >= g_posMotion.stepsTotal) {
            stopPositionalMotion();
            Serial.println("POS: движение завершено.");
        } else {
            updatePositionalMotionProfileDelay();
        }
    }
}

void startPositionalContinuousMotion(uint32_t delayUs)
{
    if (g_posMotion.active) {
        Serial.println("POS: мотор PUL32 уже в движении.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    writePosPulseInactive();
    g_posMotion.active = true;
    g_posMotion.pulseHigh = false;
    g_posMotion.continuous = true;
    g_posMotion.stepsTotal = 0;
    g_posMotion.stepsDone = 0;
    g_posMotion.delayUs = delayUs;
    g_posMotion.currentDelayUs = delayUs;
    g_posMotion.startDelayUs = delayUs;
    g_posMotion.rampSteps = 0;
    g_posMotion.lastPulseStartUs = micros();

    Serial.print("POS: непрерывный запуск PUL32, задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void processMotion()
{
    if (!g_motion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_motion.pulseHigh) {
        if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= g_motion.currentDelayUs) {
            digitalWrite(PIN_STEP_PUL, core::hw_config::PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_motion.pulseHigh = true;
            g_motion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= core::hw_config::STEP_PULSE_WIDTH_US) {
        writePulseInactive();
        g_motion.pulseHigh = false;
        g_motion.stepsDone++;
        g_totalStepsCounter++;

        if (g_motion.continuous) {
            return;
        }

        if (g_motion.stepsDone >= g_motion.stepsTotal) {
            stopMotion();
            if (!cycle2IsActive()) {
                Serial.println("Движение завершено.");
            }
        } else {
            updateMotionProfileDelay();
        }
    }
}

void updateSensorFilter()
{
    updateDebouncedSensorFilter(g_sensorFilter, readSensorPlateRaw(), core::g_settings.sensorDebounceMs);
}

bool isPlateAtSensorFiltered()
{
    return g_sensorFilter.stablePlateDetected;
}

void printSensorState()
{
    const bool isPlateAtSensor = isPlateAtSensorFiltered();
    Serial.print("Датчик E18-D50NK: ");
    Serial.println(isPlateAtSensor ? "тарелка на флаге (HIGH)" : "тарелки нет (LOW)");

    Serial.print("Геркон Z GPIO33: ");
    Serial.println(shiftIsSensorZTriggered() ? "замкнут (LOW)" : "разомкнут (HIGH)");
    Serial.print("Геркон C GPIO25: ");
    Serial.println(shiftIsSensorCTriggered() ? "замкнут (LOW)" : "разомкнут (HIGH)");
}

void processSensorStream()
{
    if (!g_sensorStreamEnabled) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_lastSensorPrintMs) >= core::g_settings.sensorPrintIntervalMs) {
        g_lastSensorPrintMs = nowMs;
        printSensorState();
    }
}

bool cliToggleSensorStream()
{
    g_sensorStreamEnabled = !g_sensorStreamEnabled;
    g_lastSensorPrintMs = millis();
    return g_sensorStreamEnabled;
}

bool motionMainIsActive()
{
    return g_motion.active;
}

bool motionPositionalIsActive()
{
    return g_posMotion.active;
}

uint32_t motionTotalSteps()
{
    return g_totalStepsCounter;
}
