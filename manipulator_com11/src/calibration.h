#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ КАЛИБРОВКИ СИСТЕМЫ
// Измерение расстояния между концевиками с S-профилем
// =============================================

// Флаги состояния системы (объявлены в system_state.h, но нужны здесь)
extern bool isCalibrated;
extern unsigned long travelDistance;
extern float travelDistanceMM;
extern float stepsPerMM;

// Основная функция калибровки
void calibrateDistance();

// Проверка точности позиционирования концевика
bool checkSwitchAccuracy(bool direction, const char* switchName, unsigned long checkDistance);

// Измерение расстояния от текущей позиции до концевика с S-профилем
// direction: true - к правому, false - к левому
unsigned long measureDistanceWithSProfile(bool toRight);

// Отъезд от концевика на безопасное расстояние
void moveAwayFromSwitch(bool direction, const char* switchName, unsigned long distance = CALIBRATION_SAFETY_DISTANCE);

// Движение к концевику для калибровки (поиск нулевой точки)
bool moveToSwitchForCalibration(bool toRight);

// Установка пользовательского расстояния (ручной ввод)
void setManualDistance(unsigned long steps, float mm);

// Получение текущих калибровочных данных
unsigned long getCalibratedDistance();
float getStepsPerMM();

// Проверка готовности к калибровке
bool isReadyForCalibration();

// Сброс калибровки
void resetCalibration();