#include <Arduino.h>

#include "master_utils.h"

const char *rs485SerialConfigName(uint32_t serialConfig)
{
    switch (serialConfig) {
        case SERIAL_8E1:
            return "8E1";
        case SERIAL_8N1:
            return "8N1";
        case SERIAL_8N2:
            return "8N2";
        default:
            return "custom";
    }
}

void printBytesHex(const uint8_t *data, size_t len)
{
    if (data == nullptr || len == 0) {
        Serial.println("<none>");
        return;
    }

    for (size_t i = 0; i < len; i++) {
        if (i > 0) {
            Serial.print(" ");
        }
        if (data[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(data[i], HEX);
    }
    Serial.println();
}

String bytesToHexString(const uint8_t *data, size_t len, size_t maxBytes)
{
    if (data == nullptr || len == 0) {
        return "<none>";
    }

    String out;
    const size_t limit = (len < maxBytes) ? len : maxBytes;
    out.reserve(limit * 3 + 8);

    for (size_t i = 0; i < limit; i++) {
        if (i > 0) {
            out += " ";
        }
        if (data[i] < 0x10) {
            out += "0";
        }
        out += String(data[i], HEX);
    }

    if (len > limit) {
        out += " ...";
    }

    return out;
}

const char *mbResultCode(MbResult result)
{
    switch (result) {
        case MbResult::Ok:
            return "ok";
        case MbResult::ArgError:
            return "arg";
        case MbResult::Timeout:
            return "timeout";
        case MbResult::ProtocolError:
            return "protocol";
        case MbResult::CrcError:
            return "crc";
        case MbResult::Exception:
            return "exception";
        default:
            return "unknown";
    }
}

bool parseU16(String token, uint16_t &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    char *end = nullptr;
    const unsigned long v = strtoul(token.c_str(), &end, 0);
    if (end == token.c_str() || *end != '\0' || v > 0xFFFFUL) {
        return false;
    }

    value = static_cast<uint16_t>(v);
    return true;
}

bool parseU32(String token, uint32_t &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    char *end = nullptr;
    const unsigned long v = strtoul(token.c_str(), &end, 0);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }

    value = static_cast<uint32_t>(v);
    return true;
}

bool parseI32(String token, int32_t &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    char *end = nullptr;
    const long v = strtol(token.c_str(), &end, 0);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }

    value = static_cast<int32_t>(v);
    return true;
}

bool parseF32(String token, float &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    token.replace(',', '.');
    char *end = nullptr;
    const float v = strtof(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }

    value = v;
    return true;
}

void normalizeCommandWhitespace(String &s)
{
    s.replace("\r", " ");
    s.replace("\n", " ");
    s.replace("\t", " ");
    s.replace("\xC2\xA0", " ");
    s.replace("\xE2\x80\x87", " ");
    s.replace("\xE2\x80\xAF", " ");

    while (s.indexOf("  ") >= 0) {
        s.replace("  ", " ");
    }
    s.trim();
}

String nextToken(String &s)
{
    normalizeCommandWhitespace(s);
    if (s.isEmpty()) {
        return "";
    }

    const int split = s.indexOf(' ');
    if (split < 0) {
        String t = s;
        s = "";
        return t;
    }

    String t = s.substring(0, split);
    s = s.substring(split + 1);
    s.trim();
    return t;
}

bool mapToFlagCommand(String input, String &mapped)
{
    input.trim();
    input.toUpperCase();

    if (input == "W" || input == "UP" || input == "FLAG UP") {
        mapped = "W";
        return true;
    }

    if (input == "S" || input == "DOWN" || input == "FLAG DOWN") {
        mapped = "S";
        return true;
    }

    return false;
}
