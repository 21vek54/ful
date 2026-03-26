#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ ТЕСТА ОТКАЗОУСТОЙЧИВОСТИ
// Циклическое тестирование всей системы
// =============================================

// Состояние теста
extern bool testInProgress;
extern int testCyclesRemaining;
extern unsigned long testCycleCounter;
extern unsigned long testStartTime;
extern unsigned long testLastMovementTime;

// Запуск теста
void startEnduranceTest(int cycles);

// Остановка теста
void stopEnduranceTest();

// Обновление состояния теста (вызывать в loop)
void updateEnduranceTest();

// Принудительное завершение теста
void completeEnduranceTest(bool success);

// Выполнение одного цикла теста
bool performTestCycle();

// Вывод статистики теста
void printTestStatistics();

// Проверка готовности к тесту
bool isReadyForTest();

// Тестовая последовательность T (полный цикл)
bool runTSequenceOnce();