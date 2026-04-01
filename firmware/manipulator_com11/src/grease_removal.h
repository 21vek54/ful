#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ ПРОЦЕДУРЫ УДАЛЕНИЯ СМАЗКИ
// Равномерное распределение смазки после обслуживания
// =============================================

// Состояние процедуры
extern bool greaseRemovalInProgress;
extern int currentGreaseDistance;
extern bool currentDirection; // true - влево, false - вправо
extern unsigned long greaseCycleCounter;

// Запуск процедуры удаления смазки
void startGreaseRemoval();

// Остановка процедуры
void stopGreaseRemoval();

// Обновление состояния процедуры (вызывать в loop)
void updateGreaseRemoval();

// Выполнение одного движения процедуры
bool performGreaseMovement();

// Проверка готовности к процедуре
bool isReadyForGreaseRemoval();

// Вывод статуса процедуры
void printGreaseRemovalStatus();