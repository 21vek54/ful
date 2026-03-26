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

struct SealerStatus {
    bool startOutputActive = false;
    bool startPulseActive = false;
    bool doneInputActive = false;
    bool ready = false;
    bool busy = false;
    bool alarm = false;
    uint32_t startPulseDurationMs = 0;
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

// Сервисные значения и флаги для внешнего слоя.
bool isStartPulseActive();
uint32_t getStartPulseDefaultMs();
uint32_t getStartPulseMinMs();
uint32_t getStartPulseMaxMs();

} // namespace groups::sealer
