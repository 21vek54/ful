#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "pneumatics.h"
#include "motion_profiles.h"
#include "system_state.h"

// =============================================
// МОДУЛЬ РАБОЧЕГО ЦИКЛА МАНИПУЛЯТОРА
// =============================================

// Функция проверки начального состояния
bool checkWorkCycleInitialState();

// Функция выполнения одного рабочего цикла
bool executeWorkCycle();

// Функция запуска рабочего цикла (с проверкой начального состояния)
bool startWorkCycle(bool fromRecovery = false);

// Функция проверки, выполняется ли рабочий цикл
bool isWorkCycleInProgress();
uint8_t getWorkCycleCurrentStep();
bool isWorkCycleStep7Reached();

// Функция восстановления рабочего цикла после сбоя
bool recoverWorkCycle();

// Функция проверки состояния для заданного шага
bool verifyStepState(uint8_t step);

// Функция для проверки, был ли выполнен определенный шаг
bool checkStepCompleted(uint8_t step);

// Вспомогательная функция для перемещения к нужному шагу
bool moveToStep(uint8_t targetStep);

// Функция выполнения конкретного шага
bool executeStep(uint8_t step);

// ===== НОВЫЕ ФУНКЦИИ ДЛЯ БЕЗОПАСНОГО ВОССТАНОВЛЕНИЯ =====

// Функция медленного движения к правому концевику
bool slowMoveToRightEnd();

// Функция медленного движения к левому концевику
bool slowMoveToLeftEnd();

// Функция безопасного восстановления для каждого шага EEPROM
bool safeRecoveryForStep(uint8_t savedStep);

// Функция анализа текущего состояния
int getCurrentPosition();

// Функция определения требуемого положения для шага
int getRequiredPositionForStep(uint8_t step);

// Функция для медленного движения к концевику
bool slowMoveToSwitch(bool toLeft, unsigned long maxSteps = 50000); // Аргумент по умолчанию ТОЛЬКО ЗДЕСЬ
