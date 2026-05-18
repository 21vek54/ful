// Группа подвода (infeed): отдает статус и сервисную эмуляцию прохода тарелки
// через датчик E18 на уровне raw-сигнала для штатной фильтрации и runtime-логики.
#include "groups/infeed/infeed_group.h"

#include <Arduino.h>

#include "legacy/program1.h"
#include "legacy/shift_control.h"

// Thin runtime hook implemented in motion_control.cpp.
bool isPlateAtSensorFiltered();

namespace groups::infeed {

namespace {

constexpr uint32_t INFEED_EMU_ON_MS_DEFAULT = 180U;
constexpr uint32_t INFEED_EMU_OFF_MS_DEFAULT = 260U;
constexpr uint32_t INFEED_EMU_ON_MS_MIN = 90U;
constexpr uint32_t INFEED_EMU_ON_MS_MAX = 10000U;
constexpr uint32_t INFEED_EMU_OFF_MS_MIN = 50U;
constexpr uint32_t INFEED_EMU_OFF_MS_MAX = 10000U;
constexpr uint32_t INFEED_EMU_PASS_COUNT_MAX = 1000U;
constexpr uint8_t INFEED_EMU_REALISTIC_BURST_SIZE = 6U;
constexpr uint32_t INFEED_EMU_REALISTIC_BURST_GAP_MS = 1050U;
constexpr uint32_t INFEED_EMU_REALISTIC_START_ON_SENSOR_MS = 620U;
constexpr uint32_t INFEED_EMU_REALISTIC_ON_PATTERN_MS[] = {
    132U, 146U, 124U, 141U, 150U, 129U,
};
constexpr uint32_t INFEED_EMU_REALISTIC_OFF_PATTERN_MS[] = {
    205U, 182U, 228U, 198U, 216U, 191U,
};

enum class EmuPhase : uint8_t {
    Idle = 0,
    HighPulse,
    LowGap
};

struct PlatePassEmuState {
    bool autoEnabled = false;
    bool rawActive = false;
    EmuPhase phase = EmuPhase::Idle;
    InfeedEmuProfile profile = InfeedEmuProfile::Generic;
    bool realisticStartOnSensor = false;
    bool realisticStartPending = false;
    uint8_t realisticBurstPos = 0U;
    uint32_t onMs = INFEED_EMU_ON_MS_DEFAULT;
    uint32_t offMs = INFEED_EMU_OFF_MS_DEFAULT;
    uint32_t phaseTargetMs = INFEED_EMU_ON_MS_DEFAULT;
    uint32_t pendingPasses = 0;
    uint32_t passSeq = 0;
    uint32_t phaseStartedMs = 0;
};

PlatePassEmuState g_platePassEmu = {};

bool isTimingValid(uint32_t onMs, uint32_t offMs)
{
    if (onMs < INFEED_EMU_ON_MS_MIN || onMs > INFEED_EMU_ON_MS_MAX) {
        return false;
    }
    if (offMs < INFEED_EMU_OFF_MS_MIN || offMs > INFEED_EMU_OFF_MS_MAX) {
        return false;
    }
    return true;
}

bool shouldRunSequence()
{
    return g_platePassEmu.autoEnabled || g_platePassEmu.pendingPasses > 0U;
}

const char *profileToText(InfeedEmuProfile profile)
{
    switch (profile) {
        case InfeedEmuProfile::Generic:
            return "generic";
        case InfeedEmuProfile::Realistic:
            return "realistic";
    }
    return "unknown";
}

uint32_t selectRealisticHighDurationMs(bool &startOnSensorPulse)
{
    startOnSensorPulse = false;
    if (g_platePassEmu.realisticStartPending) {
        g_platePassEmu.realisticStartPending = false;
        startOnSensorPulse = true;
        return INFEED_EMU_REALISTIC_START_ON_SENSOR_MS;
    }

    constexpr uint8_t patternCount = static_cast<uint8_t>(
        sizeof(INFEED_EMU_REALISTIC_ON_PATTERN_MS) / sizeof(INFEED_EMU_REALISTIC_ON_PATTERN_MS[0]));
    const uint8_t burstIndex = g_platePassEmu.realisticBurstPos % patternCount;
    return INFEED_EMU_REALISTIC_ON_PATTERN_MS[burstIndex];
}

uint32_t selectRealisticLowDurationMs(bool &batchGapSelected)
{
    batchGapSelected = false;
    if (g_platePassEmu.realisticBurstPos >= INFEED_EMU_REALISTIC_BURST_SIZE) {
        g_platePassEmu.realisticBurstPos = 0U;
        batchGapSelected = true;
        return INFEED_EMU_REALISTIC_BURST_GAP_MS;
    }

    constexpr uint8_t patternCount = static_cast<uint8_t>(
        sizeof(INFEED_EMU_REALISTIC_OFF_PATTERN_MS) / sizeof(INFEED_EMU_REALISTIC_OFF_PATTERN_MS[0]));
    const uint8_t burstIndex = (g_platePassEmu.realisticBurstPos == 0U)
        ? 0U
        : static_cast<uint8_t>((g_platePassEmu.realisticBurstPos - 1U) % patternCount);
    return INFEED_EMU_REALISTIC_OFF_PATTERN_MS[burstIndex];
}

uint32_t peekRealisticLowDurationMs()
{
    if (g_platePassEmu.realisticBurstPos >= INFEED_EMU_REALISTIC_BURST_SIZE) {
        return INFEED_EMU_REALISTIC_BURST_GAP_MS;
    }
    constexpr uint8_t patternCount = static_cast<uint8_t>(
        sizeof(INFEED_EMU_REALISTIC_OFF_PATTERN_MS) / sizeof(INFEED_EMU_REALISTIC_OFF_PATTERN_MS[0]));
    const uint8_t burstIndex = (g_platePassEmu.realisticBurstPos == 0U)
        ? 0U
        : static_cast<uint8_t>((g_platePassEmu.realisticBurstPos - 1U) % patternCount);
    return INFEED_EMU_REALISTIC_OFF_PATTERN_MS[burstIndex];
}

void startHighPhase(uint32_t nowMs, const char *source)
{
    bool realisticStartOnSensorPulse = false;
    uint32_t highDurationMs = g_platePassEmu.onMs;
    if (g_platePassEmu.profile == InfeedEmuProfile::Realistic) {
        highDurationMs = selectRealisticHighDurationMs(realisticStartOnSensorPulse);
    }

    g_platePassEmu.phase = EmuPhase::HighPulse;
    g_platePassEmu.rawActive = true;
    g_platePassEmu.phaseStartedMs = nowMs;
    g_platePassEmu.phaseTargetMs = highDurationMs;
    g_platePassEmu.onMs = highDurationMs;
    if (g_platePassEmu.pendingPasses > 0U) {
        g_platePassEmu.pendingPasses--;
    }
    g_platePassEmu.passSeq++;
    if (g_platePassEmu.profile == InfeedEmuProfile::Realistic) {
        if (g_platePassEmu.realisticBurstPos < 0xFFU) {
            g_platePassEmu.realisticBurstPos++;
        }
        g_platePassEmu.offMs = peekRealisticLowDurationMs();
    }

    Serial.print("INFEED EMU: pass #");
    Serial.print(g_platePassEmu.passSeq);
    Serial.print(" ACTIVE, source=");
    Serial.print((source != nullptr && source[0] != '\0') ? source : "n/a");
    Serial.print(", profile=");
    Serial.print(profileToText(g_platePassEmu.profile));
    if (g_platePassEmu.profile == InfeedEmuProfile::Realistic && realisticStartOnSensorPulse) {
        Serial.print(", start_on_sensor=yes");
    }
    Serial.print(", on_ms=");
    Serial.print(highDurationMs);
    Serial.print(", next_off_ms=");
    Serial.println(g_platePassEmu.offMs);
}

void startLowGap(uint32_t nowMs)
{
    bool realisticBatchGap = false;
    uint32_t lowDurationMs = g_platePassEmu.offMs;
    if (g_platePassEmu.profile == InfeedEmuProfile::Realistic) {
        lowDurationMs = selectRealisticLowDurationMs(realisticBatchGap);
        g_platePassEmu.offMs = lowDurationMs;
    }

    g_platePassEmu.phase = EmuPhase::LowGap;
    g_platePassEmu.rawActive = false;
    g_platePassEmu.phaseStartedMs = nowMs;
    g_platePassEmu.phaseTargetMs = lowDurationMs;

    Serial.print("INFEED EMU: pass #");
    Serial.print(g_platePassEmu.passSeq);
    Serial.print(" INACTIVE, off_ms=");
    Serial.print(lowDurationMs);
    if (realisticBatchGap) {
        Serial.print(", batch_gap=yes");
    }
    Serial.println(".");
}

void stopSequenceNow(const char *reason)
{
    const bool wasActive =
        g_platePassEmu.rawActive ||
        g_platePassEmu.phase != EmuPhase::Idle ||
        g_platePassEmu.pendingPasses > 0U ||
        g_platePassEmu.autoEnabled;

    g_platePassEmu.autoEnabled = false;
    g_platePassEmu.rawActive = false;
    g_platePassEmu.phase = EmuPhase::Idle;
    g_platePassEmu.profile = InfeedEmuProfile::Generic;
    g_platePassEmu.realisticStartOnSensor = false;
    g_platePassEmu.realisticStartPending = false;
    g_platePassEmu.realisticBurstPos = 0U;
    g_platePassEmu.pendingPasses = 0;
    g_platePassEmu.phaseTargetMs = g_platePassEmu.offMs;
    g_platePassEmu.phaseStartedMs = millis();

    if (!wasActive) {
        return;
    }

    if (reason != nullptr && reason[0] != '\0') {
        Serial.print("INFEED EMU: ");
        Serial.println(reason);
    }
}

} // namespace

void initInfeedGroup()
{
    g_platePassEmu = {};
    g_platePassEmu.onMs = INFEED_EMU_ON_MS_DEFAULT;
    g_platePassEmu.offMs = INFEED_EMU_OFF_MS_DEFAULT;
    g_platePassEmu.phaseTargetMs = INFEED_EMU_ON_MS_DEFAULT;
    g_platePassEmu.phaseStartedMs = millis();
}

void processInfeedGroup()
{
    const uint32_t nowMs = millis();

    if (g_platePassEmu.phase == EmuPhase::HighPulse) {
        if ((uint32_t)(nowMs - g_platePassEmu.phaseStartedMs) >= g_platePassEmu.phaseTargetMs) {
            startLowGap(nowMs);
        }
        return;
    }

    if (g_platePassEmu.phase == EmuPhase::LowGap) {
        if ((uint32_t)(nowMs - g_platePassEmu.phaseStartedMs) < g_platePassEmu.phaseTargetMs) {
            return;
        }

        if (shouldRunSequence()) {
            startHighPhase(nowMs, g_platePassEmu.autoEnabled ? "AUTO" : "PASS");
            return;
        }

        g_platePassEmu.phase = EmuPhase::Idle;
        g_platePassEmu.phaseStartedMs = nowMs;
        Serial.println("INFEED EMU: sequence completed.");
        return;
    }

    if (shouldRunSequence()) {
        startHighPhase(nowMs, g_platePassEmu.autoEnabled ? "AUTO" : "PASS");
    }
}

InfeedStatus readInfeedStatus()
{
    InfeedStatus status = {};
    status.sensorPlateDetected = isPlateAtSensorFiltered();
    status.shiftCalibrated = shiftIsCalibrated();

    if (program1IsPositionalMotionActive() || shiftHookIsPositionalMotionActive()) {
        status.runState = InfeedRunState::Positioning;
    } else if (program1IsManualMotionActive() || program1IsCycle2Active() ||
               shiftHookIsMotionActive() || shiftHookIsCycle2Active()) {
        status.runState = InfeedRunState::Running;
    } else {
        status.runState = InfeedRunState::Idle;
    }

    status.busy = status.runState != InfeedRunState::Idle;
    status.ready = status.shiftCalibrated && !status.busy;
    status.alarm = false;

    return status;
}

InfeedEmuStatus readInfeedEmuStatus()
{
    InfeedEmuStatus status = {};
    status.autoEnabled = g_platePassEmu.autoEnabled;
    status.sequenceActive =
        g_platePassEmu.phase != EmuPhase::Idle ||
        g_platePassEmu.pendingPasses > 0U ||
        g_platePassEmu.autoEnabled;
    status.rawSensorActive = g_platePassEmu.rawActive;
    status.profile = g_platePassEmu.profile;
    status.realisticStartOnSensor = g_platePassEmu.realisticStartOnSensor;
    status.realisticStartPending = g_platePassEmu.realisticStartPending;
    status.passSeq = g_platePassEmu.passSeq;
    status.pendingPasses = g_platePassEmu.pendingPasses;
    status.onMs = g_platePassEmu.onMs;
    status.offMs = g_platePassEmu.offMs;
    if (g_platePassEmu.phase == EmuPhase::Idle) {
        status.phaseElapsedMs = 0U;
    } else {
        status.phaseElapsedMs = millis() - g_platePassEmu.phaseStartedMs;
    }
    return status;
}

bool setPlatePassEmulationAuto(bool enabled, uint32_t onMs, uint32_t offMs)
{
    if (!enabled) {
        if (g_platePassEmu.autoEnabled) {
            g_platePassEmu.autoEnabled = false;
            Serial.println("INFEED EMU: AUTO OFF.");
        }
        return true;
    }

    if (!isTimingValid(onMs, offMs)) {
        return false;
    }

    g_platePassEmu.onMs = onMs;
    g_platePassEmu.offMs = offMs;
    g_platePassEmu.phaseTargetMs = onMs;
    g_platePassEmu.profile = InfeedEmuProfile::Generic;
    g_platePassEmu.realisticStartOnSensor = false;
    g_platePassEmu.realisticStartPending = false;
    g_platePassEmu.realisticBurstPos = 0U;
    g_platePassEmu.autoEnabled = true;

    Serial.print("INFEED EMU: AUTO ON, on_ms=");
    Serial.print(onMs);
    Serial.print(", off_ms=");
    Serial.println(offMs);
    return true;
}

bool setPlatePassEmulationRealisticAuto(bool enabled, bool startOnSensor)
{
    if (!enabled) {
        if (g_platePassEmu.profile == InfeedEmuProfile::Realistic ||
            g_platePassEmu.autoEnabled ||
            g_platePassEmu.pendingPasses > 0U ||
            g_platePassEmu.phase != EmuPhase::Idle ||
            g_platePassEmu.rawActive) {
            stopSequenceNow("REALISTIC OFF, sequence canceled.");
        }
        return true;
    }

    g_platePassEmu.profile = InfeedEmuProfile::Realistic;
    g_platePassEmu.autoEnabled = true;
    g_platePassEmu.pendingPasses = 0U;
    g_platePassEmu.rawActive = false;
    g_platePassEmu.phase = EmuPhase::Idle;
    g_platePassEmu.realisticStartOnSensor = startOnSensor;
    g_platePassEmu.realisticStartPending = startOnSensor;
    g_platePassEmu.realisticBurstPos = 0U;
    g_platePassEmu.onMs = INFEED_EMU_REALISTIC_ON_PATTERN_MS[0];
    g_platePassEmu.offMs = INFEED_EMU_REALISTIC_OFF_PATTERN_MS[0];
    g_platePassEmu.phaseTargetMs = g_platePassEmu.onMs;
    g_platePassEmu.phaseStartedMs = millis();

    Serial.print("INFEED EMU REALISTIC: ON, burst_size=");
    Serial.print(INFEED_EMU_REALISTIC_BURST_SIZE);
    Serial.print(", start_on_sensor=");
    Serial.print(startOnSensor ? "yes" : "no");
    Serial.print(", on_pattern_ms=");
    Serial.print(g_platePassEmu.onMs);
    Serial.print(".., off_pattern_ms=");
    Serial.print(g_platePassEmu.offMs);
    Serial.print(".., burst_gap_ms=");
    Serial.println(INFEED_EMU_REALISTIC_BURST_GAP_MS);

    if (startOnSensor) {
        Serial.print("INFEED EMU REALISTIC: first pulse emulates plate already at E18, hold_ms=");
        Serial.println(INFEED_EMU_REALISTIC_START_ON_SENSOR_MS);
    }

    return true;
}

bool queuePlatePassEmulation(uint32_t count, uint32_t onMs, uint32_t offMs)
{
    if (count == 0U || count > INFEED_EMU_PASS_COUNT_MAX) {
        return false;
    }
    if (!isTimingValid(onMs, offMs)) {
        return false;
    }
    if (g_platePassEmu.pendingPasses > (INFEED_EMU_PASS_COUNT_MAX - count)) {
        return false;
    }

    g_platePassEmu.onMs = onMs;
    g_platePassEmu.offMs = offMs;
    g_platePassEmu.phaseTargetMs = onMs;
    g_platePassEmu.profile = InfeedEmuProfile::Generic;
    g_platePassEmu.realisticStartOnSensor = false;
    g_platePassEmu.realisticStartPending = false;
    g_platePassEmu.realisticBurstPos = 0U;
    g_platePassEmu.pendingPasses += count;

    Serial.print("INFEED EMU: PASS queued, count=");
    Serial.print(count);
    Serial.print(", pending=");
    Serial.print(g_platePassEmu.pendingPasses);
    Serial.print(", on_ms=");
    Serial.print(onMs);
    Serial.print(", off_ms=");
    Serial.println(offMs);
    return true;
}

void stopPlatePassEmulation()
{
    stopSequenceNow("AUTO/PASS OFF, sequence canceled.");
}

bool isPlateSensorEmulationRawActive()
{
    return g_platePassEmu.rawActive;
}

uint32_t getPlatePassEmuDefaultOnMs()
{
    return INFEED_EMU_ON_MS_DEFAULT;
}

uint32_t getPlatePassEmuDefaultOffMs()
{
    return INFEED_EMU_OFF_MS_DEFAULT;
}

uint32_t getPlatePassEmuMinOnMs()
{
    return INFEED_EMU_ON_MS_MIN;
}

uint32_t getPlatePassEmuMaxOnMs()
{
    return INFEED_EMU_ON_MS_MAX;
}

uint32_t getPlatePassEmuMinOffMs()
{
    return INFEED_EMU_OFF_MS_MIN;
}

uint32_t getPlatePassEmuMaxOffMs()
{
    return INFEED_EMU_OFF_MS_MAX;
}

uint32_t getPlatePassEmuMaxCount()
{
    return INFEED_EMU_PASS_COUNT_MAX;
}

} // namespace groups::infeed
