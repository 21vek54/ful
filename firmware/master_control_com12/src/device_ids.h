#pragma once

// Этот файл хранит идентификаторы I2C-устройств нового master.
// Сейчас master работает только с conveyor(12) и manipulator(13).

#include <Arduino.h>

constexpr uint8_t CONVEYOR_ID = 12;
constexpr uint8_t MANIPULATOR_ID = 13;
