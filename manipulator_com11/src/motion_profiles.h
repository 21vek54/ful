#pragma once
#include <Arduino.h>
#include "config.h"
#include "system_state.h"

// =============================================
// МОДУЛЬ ПРОФИЛЕЙ ДВИЖЕНИЯ
// =============================================

// Структура сегмента профиля
struct SpeedProfileSegment {
    float percent;              // Процент от общего расстояния
    unsigned long delay_us;     // Задержка между шагами (микросекунды)
};

// Фиксированные профили (из оригинального кода)
extern const char* RIGHT_PROFILE;
extern const char* LEFT_PROFILE;

// Вспомогательные функции
void safeStopMovement();
bool checkMovementSafety();

// Парсинг строки профиля
bool parseSpeedProfile(const String& profile_str, SpeedProfileSegment segments[], int& segment_count);

// Выполнение движения по профилю (с опциональным требованием концевика)
bool executeSpeedProfile(const String& profile_str, unsigned long total_steps, bool direction, bool require_limit = true);

// Движение вправо по фиксированному профилю
bool moveRightProfile(unsigned long total_steps);

// Движение влево по фиксированному профилю
bool moveLeftProfile(unsigned long total_steps);

// Функция для произвольного профиля (не требует концевика)
bool moveCustomProfile(const String& profile_str, unsigned long total_steps, bool direction);

// ===== БЕЗОПАСНЫЕ БЛОКИРУЮЩИЕ ФУНКЦИИ ДВИЖЕНИЯ =====
bool moveRightSafe(unsigned long timeout_ms = 30000);
bool moveLeftSafe(unsigned long timeout_ms = 30000);

// Функции медленного движения для восстановления - УДАЛЕНЫ ИЗ ЭТОГО ФАЙЛА
// Они находятся в work_cycle.h