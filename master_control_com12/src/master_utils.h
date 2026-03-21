#pragma once

#include <Arduino.h>

#include "master_types.h"

const char *rs485SerialConfigName(uint32_t serialConfig);
void printBytesHex(const uint8_t *data, size_t len);
String bytesToHexString(const uint8_t *data, size_t len, size_t maxBytes = 24);
const char *mbResultCode(MbResult result);
bool parseU16(String token, uint16_t &value);
bool parseU32(String token, uint32_t &value);
bool parseI32(String token, int32_t &value);
bool parseF32(String token, float &value);
void normalizeCommandWhitespace(String &s);
String nextToken(String &s);
bool mapToFlagCommand(String input, String &mapped);
