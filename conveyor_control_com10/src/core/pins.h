#pragma once

#include <Arduino.h>

// Documentation:
// docs/conveyor_wiring.md  - wiring schemes
// docs/conveyor_pinout.md  - pin map table

// Main conveyor
constexpr uint8_t PIN_STEP_PUL = 13;

// Shift carriage
constexpr uint8_t PIN_SHIFT_DIR = 14;
constexpr uint8_t PIN_SHIFT_PUL = 23;
constexpr uint8_t PIN_SHIFT_SENSOR_Z = 33;
constexpr uint8_t PIN_SHIFT_SENSOR_C = 25;

// Pneumatics and sensors
constexpr uint8_t PIN_FLAG = 27;
constexpr uint8_t PIN_SENSOR = 26;

// Positional conveyor
constexpr uint8_t PIN_POS_PUL = 32;

// Outfeed group
constexpr uint8_t PIN_STEP2_PUL = 19;
constexpr uint8_t PIN_RELAY_1 = 21;

// Sealer group
constexpr uint8_t PIN_RELAY_2 = 22;
constexpr uint8_t PIN_SENSOR_EXT_1 = 34;

// Local status bus (legacy names kept for compatibility)
constexpr uint8_t PIN_RS485_RX = 16;
constexpr uint8_t PIN_RS485_TX = 17;
constexpr uint8_t PIN_RS485_DE_RE = 18;

// Reserve inputs (input-only pins on ESP32)
constexpr uint8_t PIN_SENSOR_EXT_2 = 35;
constexpr uint8_t PIN_SENSOR_EXT_3 = 36;
constexpr uint8_t PIN_SENSOR_EXT_4 = 39;
