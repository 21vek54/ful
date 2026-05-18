#pragma once

#include <stdint.h>

// Группа запайщика (sealer).
// Модуль владеет своим состоянием и обработкой START/DONE.
// Внешний код работает через прямой API без callback-hook делегирования.

namespace groups::sealer {

enum class SealerRunState : uint8_t {
    Idle = 0,
    StartPulseActive,
    WaitDone
};

// Причина, по которой completion сейчас не может быть подтвержден.
enum class SealerWaitDoneBlockReason : uint8_t {
    None = 0,
    NotWaitingDone,
    WaitingDoneSignalInactive,
    WaitingDoneSyntheticPending,
    WaitingDoneSignalAlreadyActiveNoEdge
};

struct SealerStatus {
    bool startOutputActive = false;
    bool startPulseActive = false;
    bool doneInputActive = false;
    bool doneRawInputActive = false;
    bool doneFilteredInputActive = false;
    bool donePinLevelHigh = false;
    bool doneActiveLevelLow = true;
    bool doneSyntheticPending = false;
    bool doneSyntheticHoldActive = false;
    bool doneSyntheticAutoEnabled = false;
    bool lastCompletionSynthetic = false;
    bool waitDoneActive = false;
    bool completionBlocked = false;
    bool ready = false;
    bool busy = false;
    bool alarm = false;
    uint8_t completionSeq = 0;
    uint32_t startPulseDurationMs = 0;
    uint32_t waitDoneAgeMs = 0;
    uint32_t waitDoneEnteredMs = 0;
    uint32_t waitDoneHeartbeatSeq = 0;
    uint32_t doneRawRiseCount = 0;
    uint32_t doneRawFallCount = 0;
    uint32_t doneEffectiveRiseCount = 0;
    uint32_t doneEffectiveFallCount = 0;
    uint32_t ignoredRiseOutsideWaitDoneCount = 0;
    uint32_t ignoredRiseOutsideWaitDoneLastMs = 0;
    uint32_t doneRawLastRiseMs = 0;
    uint32_t doneRawLastFallMs = 0;
    uint32_t doneEffectiveLastRiseMs = 0;
    uint32_t doneEffectiveLastFallMs = 0;
    uint32_t lastCompletionMs = 0;
    uint32_t doneFilterDebounceMs = 0;
    uint32_t doneSyntheticDelayMs = 0;
    uint32_t doneSyntheticHoldMs = 0;
    uint32_t doneSyntheticRemainingMs = 0;
    uint32_t doneSyntheticAutoDelayMs = 0;
    uint32_t doneSyntheticAutoHoldMs = 0;
    SealerWaitDoneBlockReason waitDoneBlockReason = SealerWaitDoneBlockReason::None;
    SealerRunState runState = SealerRunState::Idle;
};

// Инициализация и периодическая обработка группы.
void initSealerGroup();
void processSealerGroup();
SealerStatus readSealerStatus();

// Прямое управление выходом старта запайщика (ручной режим).
// Вызов сбрасывает активный стартовый импульс, если он был.
void setStartOutput(bool active);

// Запуск стартового импульса на заданную длительность.
// Допустимый диапазон: [getStartPulseMinMs()..getStartPulseMaxMs()].
// Возвращает false, если импульс уже активен или длительность вне диапазона.
bool startPulse(uint32_t pulseMs);
bool scheduleDoneEmulationOnce(uint32_t delayMs, uint32_t holdMs);
bool cancelDoneEmulation();
bool setAutoDoneEmulation(bool enabled, uint32_t delayMs, uint32_t holdMs);
bool isAutoDoneEmulationEnabled();
uint32_t getAutoDoneEmulationDelayMs();
uint32_t getAutoDoneEmulationHoldMs();

// Сервисные значения и флаги для внешнего слоя.
bool isStartPulseActive();
uint32_t getStartPulseDefaultMs();
uint32_t getStartPulseMinMs();
uint32_t getStartPulseMaxMs();
uint32_t getDoneEmuDelayDefaultMs();
uint32_t getDoneEmuDelayMaxMs();
uint32_t getDoneEmuHoldDefaultMs();
uint32_t getDoneEmuHoldMinMs();
uint32_t getDoneEmuHoldMaxMs();

} // namespace groups::sealer
