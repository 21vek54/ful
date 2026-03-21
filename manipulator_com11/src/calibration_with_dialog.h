#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ КАЛИБРОВКИ С ДИАЛОГОВЫМ ИНТЕРФЕЙСОМ
// =============================================

// Функция калибровки с диалогом подтверждения
void calibrateDistanceWithDialog();

// Функция быстрой калибровки (без диалога)
void calibrateDistanceQuick();

// Функция калибровки для внешнего подтверждения (web/I2C), без serial-диалога Y/N.
void calibrateDistanceWithExternalConfirmation();

// Функция установки ручного расстояния с сохранением
void setManualDistanceWithSave(unsigned long steps, float mm);

// Функция для диалога подтверждения
bool askCalibrationConfirmation(const char* question, unsigned long timeout_ms = 10000);
