// Группа запайщика: управляет импульсом START, ожиданием DONE и сервисной
// одноразовой эмуляцией DONE для отладочных интеграционных прогонов.
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
constexpr uint32_t SEAL_DONE_FILTER_DEBOUNCE_MS = 500U;
constexpr uint32_t SEAL_WAIT_DONE_HEARTBEAT_MS = 5000U;
constexpr uint32_t SEAL_DONE_EMU_DELAY_MS_DEFAULT = 3000U;
constexpr uint32_t SEAL_DONE_EMU_DELAY_MS_MAX = 600000U;
constexpr uint32_t SEAL_DONE_EMU_HOLD_MS_DEFAULT = 150U;
constexpr uint32_t SEAL_DONE_EMU_HOLD_MS_MIN = 10U;
constexpr uint32_t SEAL_DONE_EMU_HOLD_MS_MAX = 5000U;

struct SealIoState {
    bool startPulseActive = false;
    bool startOutputActive = false;
    bool waitingDone = false;
    bool doneRawLastActive = false;
    bool doneFilteredActive = false;
    bool doneEffectiveLastActive = false;
    bool doneSyntheticPending = false;
    bool doneSyntheticHoldActive = false;
    bool lastCompletionSynthetic = false;
    uint8_t completionSeq = 0;
    uint32_t startPulseStartedMs = 0;
    uint32_t startPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;
    uint32_t waitDoneEnteredMs = 0;
    uint32_t waitDoneLastHeartbeatMs = 0;
    uint32_t waitDoneHeartbeatSeq = 0;
    uint32_t ignoredRiseOutsideWaitDoneCount = 0;
    uint32_t ignoredRiseOutsideWaitDoneLastMs = 0;
    uint32_t doneRawRiseCount = 0;
    uint32_t doneRawFallCount = 0;
    uint32_t doneEffectiveRiseCount = 0;
    uint32_t doneEffectiveFallCount = 0;
    uint32_t doneRawLastRiseMs = 0;
    uint32_t doneRawLastFallMs = 0;
    uint32_t doneEffectiveLastRiseMs = 0;
    uint32_t doneEffectiveLastFallMs = 0;
    uint32_t doneRawLastChangeMs = 0;
    uint32_t lastCompletionMs = 0;
    uint32_t doneSyntheticArmedAtMs = 0;
    uint32_t doneSyntheticHoldStartedMs = 0;
    uint32_t doneSyntheticDelayMs = 0;
    uint32_t doneSyntheticHoldMs = 0;
};

SealIoState g_sealIo;

void writeStartOutputRaw(bool active)
{
    digitalWrite(PIN_RELAY_2, active ? SEAL_START_ACTIVE_LEVEL : SEAL_START_INACTIVE_LEVEL);
    g_sealIo.startOutputActive = active;
}

bool readDonePinLevelHigh()
{
    return digitalRead(PIN_SENSOR_EXT_1) == HIGH;
}

bool isDoneActiveRawFromPinLevel(bool pinLevelHigh)
{
    return pinLevelHigh ? (SEAL_DONE_ACTIVE_LEVEL == HIGH)
                        : (SEAL_DONE_ACTIVE_LEVEL == LOW);
}

bool isDoneActiveRaw()
{
    return isDoneActiveRawFromPinLevel(readDonePinLevelHigh());
}

uint32_t elapsedMs(uint32_t nowMs, uint32_t startMs)
{
    return static_cast<uint32_t>(nowMs - startMs);
}

bool isDoneActiveFiltered()
{
    const bool doneRawActive = isDoneActiveRaw();
    if (doneRawActive == g_sealIo.doneFilteredActive) {
        return g_sealIo.doneFilteredActive;
    }
    if (SEAL_DONE_FILTER_DEBOUNCE_MS == 0U) {
        return doneRawActive;
    }

    const uint32_t nowMs = millis();
    if (elapsedMs(nowMs, g_sealIo.doneRawLastChangeMs) >= SEAL_DONE_FILTER_DEBOUNCE_MS) {
        return doneRawActive;
    }
    return g_sealIo.doneFilteredActive;
}

bool isDoneActiveEffective()
{
    return isDoneActiveFiltered() || g_sealIo.doneSyntheticHoldActive;
}

const char *pinLevelText(bool pinHigh)
{
    return pinHigh ? "HIGH" : "LOW";
}

const char *waitDoneBlockReasonText(SealerWaitDoneBlockReason reason)
{
    switch (reason) {
        case SealerWaitDoneBlockReason::None:
            return "none";
        case SealerWaitDoneBlockReason::NotWaitingDone:
            return "not_waiting_done";
        case SealerWaitDoneBlockReason::WaitingDoneSignalInactive:
            return "waiting_done_inactive";
        case SealerWaitDoneBlockReason::WaitingDoneSyntheticPending:
            return "waiting_done_synth_pending";
        case SealerWaitDoneBlockReason::WaitingDoneSignalAlreadyActiveNoEdge:
            return "waiting_done_active_no_new_edge";
    }
    return "unknown";
}

SealerWaitDoneBlockReason currentWaitDoneBlockReason(bool doneEffectiveActive)
{
    if (!g_sealIo.waitingDone) {
        return SealerWaitDoneBlockReason::NotWaitingDone;
    }
    if (g_sealIo.doneSyntheticPending) {
        return SealerWaitDoneBlockReason::WaitingDoneSyntheticPending;
    }
    if (!doneEffectiveActive) {
        return SealerWaitDoneBlockReason::WaitingDoneSignalInactive;
    }
    return SealerWaitDoneBlockReason::WaitingDoneSignalAlreadyActiveNoEdge;
}

void printWaitDoneHeartbeat(uint32_t nowMs, bool doneRawActive, bool doneEffectiveActive)
{
    const SealerWaitDoneBlockReason reason = currentWaitDoneBlockReason(doneEffectiveActive);
    Serial.print("SEAL WAIT_DONE: age_ms=");
    Serial.print(elapsedMs(nowMs, g_sealIo.waitDoneEnteredMs));
    Serial.print(", done_raw=");
    Serial.print(doneRawActive ? "active" : "inactive");
    Serial.print(", done_effective=");
    Serial.print(doneEffectiveActive ? "active" : "inactive");
    Serial.print(", block=");
    Serial.print(waitDoneBlockReasonText(reason));
    Serial.print(", completion_seq=");
    Serial.print(g_sealIo.completionSeq);
    Serial.print(", emu_pending=");
    Serial.print(g_sealIo.doneSyntheticPending ? "yes" : "no");
    Serial.print(", emu_hold=");
    Serial.println(g_sealIo.doneSyntheticHoldActive ? "yes" : "no");
}

void clearDoneEmulationState()
{
    g_sealIo.doneSyntheticPending = false;
    g_sealIo.doneSyntheticHoldActive = false;
    g_sealIo.doneSyntheticArmedAtMs = 0;
    g_sealIo.doneSyntheticHoldStartedMs = 0;
    g_sealIo.doneSyntheticDelayMs = 0;
    g_sealIo.doneSyntheticHoldMs = 0;
}

void cancelDoneEmulationInternal(const char *reason)
{
    const bool hadEmulation = g_sealIo.doneSyntheticPending || g_sealIo.doneSyntheticHoldActive;
    clearDoneEmulationState();

    if (!hadEmulation) {
        return;
    }
    if (reason == nullptr || reason[0] == '\0') {
        return;
    }

    Serial.print("SEAL EMU: ");
    Serial.println(reason);
}

uint32_t getDoneSyntheticRemainingMs(uint32_t nowMs)
{
    if (g_sealIo.doneSyntheticPending) {
        const uint32_t elapsed = elapsedMs(nowMs, g_sealIo.doneSyntheticArmedAtMs);
        if (elapsed >= g_sealIo.doneSyntheticDelayMs) {
            return 0U;
        }
        return g_sealIo.doneSyntheticDelayMs - elapsed;
    }

    if (g_sealIo.doneSyntheticHoldActive) {
        const uint32_t elapsed = elapsedMs(nowMs, g_sealIo.doneSyntheticHoldStartedMs);
        if (elapsed >= g_sealIo.doneSyntheticHoldMs) {
            return 0U;
        }
        return g_sealIo.doneSyntheticHoldMs - elapsed;
    }

    return 0U;
}

void registerDoneCompletion(bool synthetic, uint32_t nowMs)
{
    g_sealIo.waitingDone = false;
    g_sealIo.waitDoneEnteredMs = 0;
    g_sealIo.waitDoneLastHeartbeatMs = 0;
    g_sealIo.waitDoneHeartbeatSeq = 0;
    g_sealIo.lastCompletionSynthetic = synthetic;
    g_sealIo.lastCompletionMs = nowMs;
    ++g_sealIo.completionSeq;

    if (synthetic) {
        Serial.println("SEAL: cycle complete (synthetic DONE).");
        return;
    }

    Serial.println("SEAL: cycle complete input became active.");
}

void processDoneEmulation(uint32_t nowMs)
{
    if (g_sealIo.doneSyntheticPending && isDoneActiveRaw()) {
        cancelDoneEmulationInternal("pending ONCE canceled: DONE input is already active.");
        return;
    }

    if (g_sealIo.doneSyntheticPending &&
        elapsedMs(nowMs, g_sealIo.doneSyntheticArmedAtMs) >= g_sealIo.doneSyntheticDelayMs) {
        g_sealIo.doneSyntheticPending = false;
        g_sealIo.doneSyntheticHoldActive = true;
        g_sealIo.doneSyntheticHoldStartedMs = nowMs;

        Serial.print("SEAL EMU: synthetic DONE active for ");
        Serial.print(g_sealIo.doneSyntheticHoldMs);
        Serial.println(" ms.");
    }

    if (!g_sealIo.doneSyntheticHoldActive) {
        return;
    }
    if (elapsedMs(nowMs, g_sealIo.doneSyntheticHoldStartedMs) < g_sealIo.doneSyntheticHoldMs) {
        return;
    }

    Serial.println("SEAL EMU: synthetic DONE released.");
    clearDoneEmulationState();
}

} // namespace

void initSealerGroup()
{
    pinMode(PIN_RELAY_2, OUTPUT);
    pinMode(PIN_SENSOR_EXT_1, INPUT);

    g_sealIo = {};
    g_sealIo.startPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;

    writeStartOutputRaw(false);
    const bool donePinLevelHigh = readDonePinLevelHigh();
    const bool doneRawActive = isDoneActiveRawFromPinLevel(donePinLevelHigh);
    g_sealIo.doneRawLastActive = doneRawActive;
    g_sealIo.doneFilteredActive = doneRawActive;
    g_sealIo.doneEffectiveLastActive = doneRawActive;
}

void processSealerGroup()
{
    const uint32_t nowMs = millis();
    if (g_sealIo.doneSyntheticPending && !g_sealIo.waitingDone) {
        cancelDoneEmulationInternal("pending ONCE canceled: completion already reached.");
    }
    processDoneEmulation(nowMs);

    const bool donePinLevelHigh = readDonePinLevelHigh();
    const bool doneRawActive = isDoneActiveRawFromPinLevel(donePinLevelHigh);
    bool doneFilteredActive = g_sealIo.doneFilteredActive;

    if (doneRawActive != g_sealIo.doneRawLastActive) {
        g_sealIo.doneRawLastActive = doneRawActive;
        g_sealIo.doneRawLastChangeMs = nowMs;
        if (doneRawActive) {
            ++g_sealIo.doneRawRiseCount;
            g_sealIo.doneRawLastRiseMs = nowMs;
        } else {
            ++g_sealIo.doneRawFallCount;
            g_sealIo.doneRawLastFallMs = nowMs;
        }

        Serial.print("SEAL DONE RAW: ");
        Serial.print(doneRawActive ? "active" : "released");
        Serial.print(", pin=");
        Serial.println(pinLevelText(donePinLevelHigh));
    }

    if (doneFilteredActive != doneRawActive) {
        if (SEAL_DONE_FILTER_DEBOUNCE_MS == 0U ||
            elapsedMs(nowMs, g_sealIo.doneRawLastChangeMs) >= SEAL_DONE_FILTER_DEBOUNCE_MS) {
            doneFilteredActive = doneRawActive;
            g_sealIo.doneFilteredActive = doneFilteredActive;
        }
    }

    const bool doneEffectiveActive = doneFilteredActive || g_sealIo.doneSyntheticHoldActive;

    if (doneEffectiveActive != g_sealIo.doneEffectiveLastActive) {
        g_sealIo.doneEffectiveLastActive = doneEffectiveActive;
        if (doneEffectiveActive) {
            ++g_sealIo.doneEffectiveRiseCount;
            g_sealIo.doneEffectiveLastRiseMs = nowMs;
            const bool completionSynthetic = g_sealIo.doneSyntheticHoldActive && !doneRawActive;
            if (g_sealIo.waitingDone) {
                registerDoneCompletion(completionSynthetic, nowMs);
                if (!completionSynthetic && g_sealIo.doneSyntheticPending) {
                    cancelDoneEmulationInternal("pending ONCE canceled: physical DONE arrived first.");
                }
            } else {
                ++g_sealIo.ignoredRiseOutsideWaitDoneCount;
                g_sealIo.ignoredRiseOutsideWaitDoneLastMs = nowMs;
                Serial.println("SEAL: DONE effective rise outside WaitDone, completion ignored.");
            }
        } else {
            ++g_sealIo.doneEffectiveFallCount;
            g_sealIo.doneEffectiveLastFallMs = nowMs;
            Serial.println("SEAL DONE EFF: released.");
        }
    }

    if (g_sealIo.waitingDone &&
        elapsedMs(nowMs, g_sealIo.waitDoneLastHeartbeatMs) >= SEAL_WAIT_DONE_HEARTBEAT_MS) {
        g_sealIo.waitDoneLastHeartbeatMs = nowMs;
        ++g_sealIo.waitDoneHeartbeatSeq;
        printWaitDoneHeartbeat(nowMs, doneRawActive, doneEffectiveActive);
    }

    if (!g_sealIo.startPulseActive) {
        return;
    }
    if (elapsedMs(nowMs, g_sealIo.startPulseStartedMs) < g_sealIo.startPulseDurationMs) {
        return;
    }

    writeStartOutputRaw(false);
    g_sealIo.startPulseActive = false;
    g_sealIo.waitingDone = true;
    g_sealIo.waitDoneEnteredMs = nowMs;
    g_sealIo.waitDoneLastHeartbeatMs = nowMs;
    g_sealIo.waitDoneHeartbeatSeq = 0;
    g_sealIo.lastCompletionSynthetic = false;
    if (doneEffectiveActive) {
        Serial.println("SEAL: WaitDone entered while DONE is already active; waiting for new effective edge.");
    }
    Serial.println("SEAL: start pulse finished.");
}

SealerStatus readSealerStatus()
{
    const uint32_t nowMs = millis();
    const bool donePinLevelHigh = readDonePinLevelHigh();
    const bool doneRawActive = isDoneActiveRawFromPinLevel(donePinLevelHigh);
    const bool doneFilteredActive = isDoneActiveFiltered();
    const bool doneEffectiveActive = doneFilteredActive || g_sealIo.doneSyntheticHoldActive;
    const SealerWaitDoneBlockReason waitDoneReason = currentWaitDoneBlockReason(doneEffectiveActive);

    SealerStatus status = {};
    status.startOutputActive = g_sealIo.startOutputActive;
    status.startPulseActive = g_sealIo.startPulseActive;
    status.doneInputActive = doneEffectiveActive;
    status.doneRawInputActive = doneRawActive;
    status.doneFilteredInputActive = doneFilteredActive;
    status.donePinLevelHigh = donePinLevelHigh;
    status.doneActiveLevelLow = SEAL_DONE_ACTIVE_LEVEL == LOW;
    status.doneSyntheticPending = g_sealIo.doneSyntheticPending;
    status.doneSyntheticHoldActive = g_sealIo.doneSyntheticHoldActive;
    status.lastCompletionSynthetic = g_sealIo.lastCompletionSynthetic;
    status.waitDoneActive = g_sealIo.waitingDone;
    status.completionBlocked = g_sealIo.waitingDone;
    status.completionSeq = g_sealIo.completionSeq;
    status.startPulseDurationMs = g_sealIo.startPulseDurationMs;
    status.waitDoneAgeMs = g_sealIo.waitingDone ? elapsedMs(nowMs, g_sealIo.waitDoneEnteredMs) : 0U;
    status.waitDoneEnteredMs = g_sealIo.waitDoneEnteredMs;
    status.waitDoneHeartbeatSeq = g_sealIo.waitDoneHeartbeatSeq;
    status.doneRawRiseCount = g_sealIo.doneRawRiseCount;
    status.doneRawFallCount = g_sealIo.doneRawFallCount;
    status.doneEffectiveRiseCount = g_sealIo.doneEffectiveRiseCount;
    status.doneEffectiveFallCount = g_sealIo.doneEffectiveFallCount;
    status.ignoredRiseOutsideWaitDoneCount = g_sealIo.ignoredRiseOutsideWaitDoneCount;
    status.ignoredRiseOutsideWaitDoneLastMs = g_sealIo.ignoredRiseOutsideWaitDoneLastMs;
    status.doneRawLastRiseMs = g_sealIo.doneRawLastRiseMs;
    status.doneRawLastFallMs = g_sealIo.doneRawLastFallMs;
    status.doneEffectiveLastRiseMs = g_sealIo.doneEffectiveLastRiseMs;
    status.doneEffectiveLastFallMs = g_sealIo.doneEffectiveLastFallMs;
    status.lastCompletionMs = g_sealIo.lastCompletionMs;
    status.doneFilterDebounceMs = SEAL_DONE_FILTER_DEBOUNCE_MS;
    status.doneSyntheticDelayMs = g_sealIo.doneSyntheticDelayMs;
    status.doneSyntheticHoldMs = g_sealIo.doneSyntheticHoldMs;
    status.doneSyntheticRemainingMs = getDoneSyntheticRemainingMs(nowMs);
    status.waitDoneBlockReason = waitDoneReason;

    if (status.startPulseActive) {
        status.runState = SealerRunState::StartPulseActive;
    } else if (g_sealIo.waitingDone) {
        status.runState = SealerRunState::WaitDone;
    } else {
        status.runState = SealerRunState::Idle;
    }
    status.busy = status.runState != SealerRunState::Idle;
    status.ready = !status.busy && !status.doneInputActive;
    status.alarm = false;

    return status;
}

void setStartOutput(bool active)
{
    g_sealIo.startPulseActive = false;
    g_sealIo.waitingDone = false;
    g_sealIo.waitDoneEnteredMs = 0;
    g_sealIo.waitDoneLastHeartbeatMs = 0;
    g_sealIo.waitDoneHeartbeatSeq = 0;
    cancelDoneEmulationInternal("ONCE canceled by manual OUT command.");
    writeStartOutputRaw(active);
}

bool startPulse(uint32_t pulseMs)
{
    if (g_sealIo.startPulseActive ||
        g_sealIo.waitingDone ||
        g_sealIo.doneSyntheticPending ||
        isDoneActiveEffective()) {
        return false;
    }
    if (pulseMs < SEAL_START_PULSE_MS_MIN || pulseMs > SEAL_START_PULSE_MS_MAX) {
        return false;
    }

    g_sealIo.startPulseDurationMs = pulseMs;
    g_sealIo.startPulseStartedMs = millis();
    g_sealIo.startPulseActive = true;
    g_sealIo.waitingDone = false;
    g_sealIo.waitDoneEnteredMs = 0;
    g_sealIo.waitDoneLastHeartbeatMs = 0;
    g_sealIo.waitDoneHeartbeatSeq = 0;
    g_sealIo.lastCompletionSynthetic = false;
    writeStartOutputRaw(true);
    return true;
}

bool scheduleDoneEmulationOnce(uint32_t delayMs, uint32_t holdMs)
{
    if (!g_sealIo.waitingDone || g_sealIo.startPulseActive) {
        return false;
    }
    if (g_sealIo.doneSyntheticPending || g_sealIo.doneSyntheticHoldActive) {
        return false;
    }
    if (isDoneActiveRaw()) {
        return false;
    }
    if (delayMs > SEAL_DONE_EMU_DELAY_MS_MAX) {
        return false;
    }
    if (holdMs < SEAL_DONE_EMU_HOLD_MS_MIN || holdMs > SEAL_DONE_EMU_HOLD_MS_MAX) {
        return false;
    }

    g_sealIo.doneSyntheticPending = true;
    g_sealIo.doneSyntheticHoldActive = false;
    g_sealIo.doneSyntheticArmedAtMs = millis();
    g_sealIo.doneSyntheticHoldStartedMs = 0;
    g_sealIo.doneSyntheticDelayMs = delayMs;
    g_sealIo.doneSyntheticHoldMs = holdMs;

    Serial.print("SEAL EMU: ONCE armed, delay=");
    Serial.print(delayMs);
    Serial.print(" ms, hold=");
    Serial.print(holdMs);
    Serial.println(" ms.");
    return true;
}

bool cancelDoneEmulation()
{
    const bool hadEmulation = g_sealIo.doneSyntheticPending || g_sealIo.doneSyntheticHoldActive;
    cancelDoneEmulationInternal("ONCE canceled.");
    return hadEmulation;
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

uint32_t getDoneEmuDelayDefaultMs()
{
    return SEAL_DONE_EMU_DELAY_MS_DEFAULT;
}

uint32_t getDoneEmuDelayMaxMs()
{
    return SEAL_DONE_EMU_DELAY_MS_MAX;
}

uint32_t getDoneEmuHoldDefaultMs()
{
    return SEAL_DONE_EMU_HOLD_MS_DEFAULT;
}

uint32_t getDoneEmuHoldMinMs()
{
    return SEAL_DONE_EMU_HOLD_MS_MIN;
}

uint32_t getDoneEmuHoldMaxMs()
{
    return SEAL_DONE_EMU_HOLD_MS_MAX;
}

} // namespace groups::sealer
