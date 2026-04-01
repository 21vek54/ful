#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ ВОССТАНОВЛЕНИЯ ПОСЛЕ СБРОСА ПИТАНИЯ
// =============================================

// Структура для хранения состояния рабочего цикла
struct WorkCycleState {
    uint8_t currentStep;        // Текущий шаг (0-8)
    uint32_t startTime;         // Время начала цикла
    bool interrupted;           // Флаг прерывания
};

// Инициализация системы восстановления
void recoveryInit();

// Сохранение текущего состояния
void saveWorkCycleState(uint8_t step, bool interrupted = false);

// Загрузка сохраненного состояния
bool loadWorkCycleState(uint8_t &step, bool &interrupted);

// Очистка сохраненного состояния
void clearWorkCycleState();

// Очистка всего состояния системы
void clearSystemState();

// Проверка наличия сохраненного состояния
bool hasSavedWorkCycleState();

// Сохранение системных параметров
void saveSystemParameters();

// Загрузка системных параметров
void loadSystemParameters();

// Печать информации о восстановлении
void printRecoveryInfo();