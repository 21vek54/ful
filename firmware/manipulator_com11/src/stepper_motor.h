#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ УПРАВЛЕНИЯ ШАГОВЫМ ДВИГАТЕЛЕМ
// =============================================

// Инициализация шагового двигателя
void stepperInit();

// Включение/выключение драйвера
void stepperEnable(bool enable);

// Установка направления
// direction: HIGH - влево, LOW - вправо (согласно вашей механике)
void stepperSetDirection(bool direction);

// Выполнение одного шага с заданной задержкой
void stepperStep(unsigned long delay_us);

// Движение на заданное количество шагов с постоянной скоростью
void stepperMoveSteps(unsigned long steps, unsigned long delay_us);

// Проверка аварийного сигнала драйвера (ALM)
bool stepperIsAlarm();

// Движение до срабатывания концевика
unsigned long stepperMoveToLimit(bool direction, unsigned long max_steps, unsigned long delay_us);

// Движение с S-профилем (для калибровки)
unsigned long stepperMoveWithSProfile(bool direction, unsigned long plateau_steps);

// Сброс драйвера через реле (аварийный перезапуск)
void stepperHardReset();

// Попытка восстановления после аварии драйвера
bool stepperRecoverFromAlarm();