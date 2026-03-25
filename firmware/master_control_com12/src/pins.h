#pragma once

// Этот файл хранит минимальную аппаратную привязку нового master.
// Сейчас здесь только I2C-пины, нужные для связи с платами 12 и 13.

#include <Arduino.h>

constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
