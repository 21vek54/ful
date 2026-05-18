#pragma once

#include <Arduino.h>

// Документация:
// docs/conveyor_wiring.md  - схемы подключения
// docs/conveyor_pinout.md  - таблица пинов

// Основной конвейер
constexpr uint8_t PIN_STEP_PUL = 13;

// Каретка сдвига
constexpr uint8_t PIN_SHIFT_DIR = 14;
constexpr uint8_t PIN_SHIFT_PUL = 23;
constexpr uint8_t PIN_SHIFT_SENSOR_Z = 33;
constexpr uint8_t PIN_SHIFT_SENSOR_C = 25;

// Пневматика и датчики
constexpr uint8_t PIN_FLAG = 27;
constexpr uint8_t PIN_SENSOR = 26;

// Позиционный конвейер
constexpr uint8_t PIN_POS_PUL = 32;

// Группа отвода
constexpr uint8_t PIN_STEP2_PUL = 19;
constexpr uint8_t PIN_RELAY_1 = 21;

// Группа запайщика
constexpr uint8_t PIN_RELAY_2 = 22;
constexpr uint8_t PIN_SENSOR_EXT_1 = 34;

// Локальная шина статуса (старые имена оставлены для совместимости)
constexpr uint8_t PIN_RS485_RX = 16;
constexpr uint8_t PIN_RS485_TX = 17;
constexpr uint8_t PIN_RS485_DE_RE = 18;

// Резервные входы (на ESP32 эти пины работают только как входы)
constexpr uint8_t PIN_SENSOR_EXT_2 = 35;
constexpr uint8_t PIN_SENSOR_EXT_3 = 36;
constexpr uint8_t PIN_SENSOR_EXT_4 = 39;
