#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors.h"

// =============================================
// МОДУЛЬ УПРАВЛЕНИЯ ПНЕВМАТИКОЙ (МОСФЕТЫ)
// =============================================

// Инициализация пневматики
void pneumoInit();

// Базовые команды управления
void pneumoZUp();
void pneumoZDown();
void pneumoGripOpen();
void pneumoGripClose();

// Безопасные команды с ожиданием датчика (усиленные)
bool pneumoZUpSafe(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);
bool pneumoZDownSafe(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);
bool pneumoGripOpenSafe(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);
bool pneumoGripCloseSafe(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);

// Обработчик команд с Serial
bool pneumoHandleCommand(char cmd);

// Вывод статуса пневматики
void pneumoPrintStatus();

// Сброс в безопасное состояние (закрыть и поднять)
void pneumoSafePosition();

// ===== УСИЛЕННЫЕ ФУНКЦИИ С ПРОВЕРКАМИ =====
// Безопасные функции с двойной проверкой
bool pneumoZUpSafeEnhanced(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);
bool pneumoZDownSafeEnhanced(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);
bool pneumoGripOpenSafeEnhanced(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);
bool pneumoGripCloseSafeEnhanced(unsigned long timeout_ms = PNEUMO_TIMEOUT_MS);