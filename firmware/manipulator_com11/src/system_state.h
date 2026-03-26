#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ ГЛОБАЛЬНОГО СОСТОЯНИЯ СИСТЕМЫ
// =============================================

// Основные флаги состояния
extern bool isMoving;
extern bool moveDirection;
extern bool isCalibrated;
extern bool calibrationInProgress;
extern bool workCycleInProgress;
extern bool greaseRemovalInProgress; // ДОБАВЛЕНА ЭТА СТРОКА

// Геометрические параметры
extern unsigned long travelDistance;
extern float travelDistanceMM;
extern float stepsPerMM;

// Инициализация состояния системы
void initSystemState();

// Сброс состояния
void resetSystemState();

// Экстренная остановка
void emergencyStop();

// Вывод полной информации о системе (статус + EEPROM)
void printSystemInfo();

// Проверка безопасности системы
bool isSystemSafe();

// Получение состояния системы в виде строки
String getSystemStatusString();

// УПРОЩЕННАЯ проверка начального состояния (без вывода в Serial)
bool checkInitialStateSimple();