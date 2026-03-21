#include "master_modbus_vfd.h"

#include <Arduino.h>

#include "master_utils.h"
#include "pins.h"

extern String g_lastCommandResult;
extern uint8_t g_modbusSlaveId;
extern uint32_t g_rs485Baud;
extern uint32_t g_rs485SerialConfig;
extern uint32_t g_mbReqCount;
extern uint32_t g_mbOkCount;
extern uint32_t g_mbErrCount;
extern uint8_t g_mbLastException;
extern float g_vfdBaseHz;

void rs485ScanResetAll();

namespace {

uint16_t modbusCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

void rs485SendBytes(const uint8_t *data, size_t len)
{
    rs485SetTransmitMode();
    Serial2.write(data, len);
    Serial2.flush();
    delayMicroseconds(TX_SETTLE_US);
    rs485SetReceiveMode();
}

bool freqHzToRawU16(float hz, uint16_t &rawU16, int32_t &rawSigned)
{
    if (g_vfdBaseHz < 0.01F) {
        return false;
    }

    rawSigned = static_cast<int32_t>(lroundf((hz / g_vfdBaseHz) * VFD_COMM_SCALE));
    if (rawSigned < VFD_FREQ_RAW_MIN || rawSigned > VFD_FREQ_RAW_MAX) {
        return false;
    }
    rawU16 = static_cast<uint16_t>(static_cast<int16_t>(rawSigned));
    return true;
}

bool vfdWritePersistentParam(uint8_t slaveId, uint16_t reg, uint16_t value, const char *label)
{
    Serial.print("VFD setup: ");
    Serial.print(label);
    Serial.print(" = ");
    Serial.print(value);
    Serial.print(" -> ");
    const MbResult r = modbusWriteSingleRetryToSlave(slaveId, reg, value, MODBUS_RETRY_COUNT);
    printMbResult(r);
    return r == MbResult::Ok;
}

bool vfdReadPersistentParam(uint8_t slaveId, uint16_t reg, uint16_t &valueOut)
{
    return modbusReadHoldingRetryFromSlave(slaveId, reg, 1, &valueOut, MODBUS_RETRY_COUNT, true) == MbResult::Ok;
}

void printVfdPersistentParam(uint8_t slaveId, const char *label, uint16_t reg)
{
    uint16_t value = 0;
    const bool ok = vfdReadPersistentParam(slaveId, reg, value);
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" = ");
    if (ok) {
        Serial.println(value);
    } else {
        Serial.println("<read error>");
    }
}

size_t rs485ReadAvailableFrame(
    uint8_t *dst,
    size_t capacity,
    uint32_t firstByteTimeoutMs,
    uint32_t interByteTimeoutMs,
    uint32_t totalTimeoutMs,
    bool *hitTotalTimeoutOut = nullptr)
{
    if (dst == nullptr || capacity == 0) {
        return 0;
    }

    size_t len = 0;
    uint32_t startedMs = millis();
    uint32_t lastByteMs = startedMs;
    bool gotAny = false;

    while (true) {
        const int c = Serial2.read();
        if (c >= 0) {
            if (len < capacity) {
                dst[len++] = static_cast<uint8_t>(c);
            }
            lastByteMs = millis();
            gotAny = true;
            continue;
        }

        const uint32_t now = millis();
        if ((uint32_t)(now - startedMs) >= totalTimeoutMs) {
            if (hitTotalTimeoutOut != nullptr) {
                *hitTotalTimeoutOut = true;
            }
            break;
        }
        if (!gotAny) {
            if ((uint32_t)(now - startedMs) >= firstByteTimeoutMs) {
                break;
            }
        } else if ((uint32_t)(now - lastByteMs) >= interByteTimeoutMs) {
            break;
        }
        delay(1);
    }

    return len;
}

} // namespace

void rs485SetReceiveMode()
{
    digitalWrite(PIN_RS485_DE_RE, LOW);
}

void rs485SetTransmitMode()
{
    digitalWrite(PIN_RS485_DE_RE, HIGH);
}

void rs485DrainRx()
{
    while (Serial2.available() > 0) {
        (void)Serial2.read();
    }
}

bool parseRs485SerialConfigToken(String token, uint32_t &serialConfigOut)
{
    token.trim();
    token.toUpperCase();

    if (token == "8E1") {
        serialConfigOut = SERIAL_8E1;
        return true;
    }
    if (token == "8N1") {
        serialConfigOut = SERIAL_8N1;
        return true;
    }
    if (token == "8N2") {
        serialConfigOut = SERIAL_8N2;
        return true;
    }

    return false;
}

void rs485ApplyUart(uint32_t baud, uint32_t serialConfig)
{
    Serial2.flush();
    Serial2.end();
    delay(10);
    Serial2.begin(baud, serialConfig, PIN_RS485_RX, PIN_RS485_TX);
    g_rs485Baud = baud;
    g_rs485SerialConfig = serialConfig;
    rs485SetReceiveMode();
    rs485DrainRx();
    delay(10);
}

void printRs485Uart()
{
    Serial.print("RS485 UART: ");
    Serial.print(g_rs485Baud);
    Serial.print(" ");
    Serial.println(rs485SerialConfigName(g_rs485SerialConfig));
}

bool rs485ReadExact(uint8_t *dst, size_t len, uint32_t timeoutMs)
{
    size_t readBytes = 0;
    const uint32_t startMs = millis();

    while (readBytes < len) {
        const int c = Serial2.read();
        if (c >= 0) {
            dst[readBytes++] = static_cast<uint8_t>(c);
            continue;
        }

        if ((uint32_t)(millis() - startMs) >= timeoutMs) {
            return false;
        }
        delay(1);
    }

    return true;
}

void appendCrc(uint8_t *frame, size_t payloadLen)
{
    const uint16_t crc = modbusCrc16(frame, payloadLen);
    frame[payloadLen] = static_cast<uint8_t>(crc & 0xFF);
    frame[payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

bool checkFrameCrc(const uint8_t *frame, size_t frameLen)
{
    if (frameLen < 4) {
        return false;
    }

    const uint16_t calc = modbusCrc16(frame, frameLen - 2);
    const uint16_t got = static_cast<uint16_t>(frame[frameLen - 2]) |
                         (static_cast<uint16_t>(frame[frameLen - 1]) << 8);
    return calc == got;
}

MbResult modbusReadHoldingFromSlave(
    uint8_t slaveId,
    uint16_t reg,
    uint16_t count,
    uint16_t *outRegs,
    uint8_t *exceptionOut,
    bool countAsRequest)
{
    if (slaveId == 0 || count == 0 || count > MODBUS_MAX_READ_REGS || outRegs == nullptr) {
        return MbResult::ArgError;
    }

    uint8_t req[8] = {
        slaveId,
        MODBUS_FUNC_READ_HOLDING,
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((count >> 8) & 0xFF),
        static_cast<uint8_t>(count & 0xFF),
        0,
        0
    };
    appendCrc(req, 6);

    rs485DrainRx();
    rs485SendBytes(req, sizeof(req));
    if (countAsRequest) {
        g_mbReqCount++;
    }

    uint8_t hdr[3] = {};
    if (!rs485ReadExact(hdr, sizeof(hdr), MODBUS_TIMEOUT_MS)) {
        return MbResult::Timeout;
    }

    if (hdr[0] != slaveId) {
        return MbResult::ProtocolError;
    }

    if (hdr[1] == static_cast<uint8_t>(MODBUS_FUNC_READ_HOLDING | 0x80)) {
        uint8_t tail[2] = {};
        if (!rs485ReadExact(tail, sizeof(tail), MODBUS_TIMEOUT_MS)) {
            return MbResult::Timeout;
        }

        uint8_t frame[5] = {hdr[0], hdr[1], hdr[2], tail[0], tail[1]};
        if (!checkFrameCrc(frame, sizeof(frame))) {
            return MbResult::CrcError;
        }

        if (exceptionOut != nullptr) {
            *exceptionOut = hdr[2];
        }
        return MbResult::Exception;
    }

    if (hdr[1] != MODBUS_FUNC_READ_HOLDING) {
        return MbResult::ProtocolError;
    }

    const uint8_t byteCount = hdr[2];
    if (byteCount != static_cast<uint8_t>(count * 2U)) {
        return MbResult::ProtocolError;
    }

    uint8_t tail[(MODBUS_MAX_READ_REGS * 2U) + 2U] = {};
    const size_t tailLen = static_cast<size_t>(byteCount) + 2U;
    if (!rs485ReadExact(tail, tailLen, MODBUS_TIMEOUT_MS)) {
        return MbResult::Timeout;
    }

    uint8_t frame[3 + (MODBUS_MAX_READ_REGS * 2U) + 2U] = {};
    frame[0] = hdr[0];
    frame[1] = hdr[1];
    frame[2] = hdr[2];
    memcpy(&frame[3], tail, tailLen);

    if (!checkFrameCrc(frame, 3U + tailLen)) {
        return MbResult::CrcError;
    }

    for (uint16_t i = 0; i < count; i++) {
        const uint8_t hi = tail[i * 2U];
        const uint8_t lo = tail[i * 2U + 1U];
        outRegs[i] = (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo);
    }

    return MbResult::Ok;
}

MbResult modbusReadHolding(uint16_t reg, uint16_t count, uint16_t *outRegs)
{
    return modbusReadHoldingFromSlave(g_modbusSlaveId, reg, count, outRegs, &g_mbLastException, true);
}

MbResult modbusWriteSingleToSlave(uint8_t slaveId, uint16_t reg, uint16_t value, bool countAsRequest)
{
    uint8_t req[8] = {
        slaveId,
        MODBUS_FUNC_WRITE_SINGLE,
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        0,
        0
    };
    appendCrc(req, 6);

    rs485DrainRx();
    rs485SendBytes(req, sizeof(req));
    if (countAsRequest) {
        g_mbReqCount++;
    }

    uint8_t resp[8] = {};
    if (!rs485ReadExact(resp, sizeof(resp), MODBUS_TIMEOUT_MS)) {
        return MbResult::Timeout;
    }

    if (resp[0] != slaveId) {
        return MbResult::ProtocolError;
    }

    if (resp[1] == static_cast<uint8_t>(MODBUS_FUNC_WRITE_SINGLE | 0x80)) {
        g_mbLastException = resp[2];
        if (!checkFrameCrc(resp, 5)) {
            return MbResult::CrcError;
        }
        return MbResult::Exception;
    }

    if (resp[1] != MODBUS_FUNC_WRITE_SINGLE) {
        return MbResult::ProtocolError;
    }

    if (!checkFrameCrc(resp, sizeof(resp))) {
        return MbResult::CrcError;
    }

    if (memcmp(req, resp, sizeof(req) - 2U) != 0) {
        return MbResult::ProtocolError;
    }

    return MbResult::Ok;
}

MbResult modbusWriteSingle(uint16_t reg, uint16_t value)
{
    return modbusWriteSingleToSlave(g_modbusSlaveId, reg, value, true);
}

MbResult modbusReadHoldingRetryFromSlave(
    uint8_t slaveId,
    uint16_t reg,
    uint16_t count,
    uint16_t *outRegs,
    uint8_t attempts,
    bool countAsRequest)
{
    if (attempts == 0) {
        attempts = 1;
    }

    MbResult last = MbResult::ArgError;
    for (uint8_t i = 0; i < attempts; i++) {
        last = modbusReadHoldingFromSlave(slaveId, reg, count, outRegs, &g_mbLastException, countAsRequest);
        if (last == MbResult::Ok || last == MbResult::Exception || last == MbResult::ArgError) {
            return last;
        }
        delay(20);
    }
    return last;
}

MbResult modbusWriteSingleRetryToSlave(uint8_t slaveId, uint16_t reg, uint16_t value, uint8_t attempts)
{
    if (attempts == 0) {
        attempts = 1;
    }

    MbResult last = MbResult::ArgError;
    for (uint8_t i = 0; i < attempts; i++) {
        last = modbusWriteSingleToSlave(slaveId, reg, value, true);
        if (last == MbResult::Ok || last == MbResult::Exception || last == MbResult::ArgError) {
            return last;
        }
        delay(20);
    }
    return last;
}

MbResult modbusWriteSingleRetry(uint16_t reg, uint16_t value, uint8_t attempts)
{
    if (attempts == 0) {
        attempts = 1;
    }

    MbResult last = MbResult::ArgError;
    for (uint8_t i = 0; i < attempts; i++) {
        last = modbusWriteSingle(reg, value);
        if (last == MbResult::Ok || last == MbResult::Exception || last == MbResult::ArgError) {
            return last;
        }
        delay(20);
    }
    return last;
}

MbResult modbusReadHoldingRetry(uint16_t reg, uint16_t count, uint16_t *outRegs, uint8_t attempts)
{
    if (attempts == 0) {
        attempts = 1;
    }

    MbResult last = MbResult::ArgError;
    for (uint8_t i = 0; i < attempts; i++) {
        last = modbusReadHolding(reg, count, outRegs);
        if (last == MbResult::Ok || last == MbResult::Exception || last == MbResult::ArgError) {
            return last;
        }
        delay(20);
    }
    return last;
}

bool hzToPanelRawU16(float hz, float scale, uint16_t &rawOut)
{
    if (hz < 0.0F || hz > 400.0F || scale <= 0.0F) {
        return false;
    }

    const int32_t raw = static_cast<int32_t>(lroundf(hz * scale));
    if (raw < 0 || raw > 0xFFFF) {
        return false;
    }

    rawOut = static_cast<uint16_t>(raw);
    return true;
}

MbResult readVfdFrequencyScale(float &scaleOut)
{
    uint16_t dec = 0;
    const MbResult r = modbusReadHoldingRetryFromSlave(g_modbusSlaveId, VFD_REG_FREQ_DECIMALS, 1, &dec, MODBUS_RETRY_COUNT, true);
    if (r != MbResult::Ok) {
        return r;
    }

    if (dec == 1) {
        scaleOut = 10.0F;
    } else if (dec == 2) {
        scaleOut = 100.0F;
    } else {
        scaleOut = 10.0F;
    }
    return MbResult::Ok;
}

MbResult writeVfdHzSetpoint(float hz, float scale, uint16_t &rawOut)
{
    if (!hzToPanelRawU16(hz, scale, rawOut)) {
        return MbResult::ArgError;
    }

    MbResult r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_COMM_FREQ_HZ_SET, rawOut, MODBUS_RETRY_COUNT);
    printMbResult(r);
    if (r != MbResult::Ok) {
        return r;
    }

    r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_PANEL_FREQ_SET, rawOut, MODBUS_RETRY_COUNT);
    printMbResult(r);
    return r;
}

void printVfdConfigSummary(uint8_t slaveId)
{
    Serial.print("VFD CFG for slave ");
    Serial.println(slaveId);
    printVfdPersistentParam(slaveId, "P0-04 control source", VFD_REG_P0_04);
    printVfdPersistentParam(slaveId, "P0-06 frequency source", VFD_REG_P0_06);
    printVfdPersistentParam(slaveId, "P0-11 panel setpoint", VFD_REG_PANEL_FREQ_SET);
    printVfdPersistentParam(slaveId, "P8-00 baud", VFD_REG_P8_00);
    printVfdPersistentParam(slaveId, "P8-01 format", VFD_REG_P8_01);
    printVfdPersistentParam(slaveId, "P8-02 slave id", VFD_REG_P8_02);
    printVfdPersistentParam(slaveId, "P8-05 protocol", VFD_REG_P8_05);
    printVfdPersistentParam(slaveId, "P8-06 remote monitor", VFD_REG_P8_06);
}

bool setupVr70ForMasterRs485(uint8_t targetSlaveId)
{
    if (targetSlaveId == 0U || targetSlaveId > 247U) {
        Serial.println("VFD SETUPRS: slave id must be 1..247.");
        g_lastCommandResult = "VFD SETUPRS failed: invalid slave id";
        return false;
    }

    const uint8_t initialSlaveId = g_modbusSlaveId;
    const uint32_t initialBaud = g_rs485Baud;
    const uint32_t initialSerialConfig = g_rs485SerialConfig;

    Serial.print("VFD SETUPRS: target slave id ");
    Serial.println(targetSlaveId);
    if (g_rs485Baud != RS485_BAUD || g_rs485SerialConfig != SERIAL_8E1) {
        Serial.println("VFD SETUPRS: applying 9600 8E1 on master RS485 UART.");
        rs485ApplyUart(RS485_BAUD, SERIAL_8E1);
    }

    bool ok = true;
    ok = ok && vfdWritePersistentParam(initialSlaveId, VFD_REG_P8_00, VFD_RS485_BAUD_9600, "P8-00 baud=9600");
    ok = ok && vfdWritePersistentParam(initialSlaveId, VFD_REG_P8_01, VFD_RS485_FORMAT_8E1, "P8-01 format=8E1");
    ok = ok && vfdWritePersistentParam(initialSlaveId, VFD_REG_P8_05, VFD_RS485_PROTO_MODBUS_STD, "P8-05 protocol=standard Modbus");
    ok = ok && vfdWritePersistentParam(initialSlaveId, VFD_REG_P8_06, VFD_RS485_REMOTE_MONITOR_ENABLE, "P8-06 remote monitor=enabled");
    if (!ok) {
        g_lastCommandResult = "VFD SETUPRS failed: P8 stage";
        if (g_rs485Baud != initialBaud || g_rs485SerialConfig != initialSerialConfig) {
            rs485ApplyUart(initialBaud, initialSerialConfig);
        }
        return false;
    }

    if (targetSlaveId != initialSlaveId) {
        if (!vfdWritePersistentParam(initialSlaveId, VFD_REG_P8_02, targetSlaveId, "P8-02 slave id")) {
            g_lastCommandResult = "VFD SETUPRS failed: slave id write";
            if (g_rs485Baud != initialBaud || g_rs485SerialConfig != initialSerialConfig) {
                rs485ApplyUart(initialBaud, initialSerialConfig);
            }
            return false;
        }
        delay(50);
        g_modbusSlaveId = targetSlaveId;
        rs485ScanResetAll();
    }

    ok = ok && vfdWritePersistentParam(g_modbusSlaveId, VFD_REG_P0_06, VFD_FREQ_SOURCE_RS485, "P0-06 frequency via RS485");
    ok = ok && vfdWritePersistentParam(g_modbusSlaveId, VFD_REG_P0_04, VFD_CTRL_SOURCE_RS485, "P0-04 control via RS485");
    if (!ok) {
        g_lastCommandResult = "VFD SETUPRS failed: P0 stage";
        return false;
    }

    Serial.print("VFD SETUPRS: host slave id switched to ");
    Serial.println(g_modbusSlaveId);
    printVfdConfigSummary(g_modbusSlaveId);
    g_lastCommandResult = "VFD SETUPRS executed";
    return true;
}

void printMbResult(MbResult result)
{
    switch (result) {
        case MbResult::Ok:
            g_mbOkCount++;
            Serial.println("MODBUS: OK");
            break;
        case MbResult::ArgError:
            g_mbErrCount++;
            Serial.println("MODBUS: arg error.");
            break;
        case MbResult::Timeout:
            g_mbErrCount++;
            Serial.println("MODBUS: timeout.");
            break;
        case MbResult::ProtocolError:
            g_mbErrCount++;
            Serial.println("MODBUS: protocol error.");
            break;
        case MbResult::CrcError:
            g_mbErrCount++;
            Serial.println("MODBUS: CRC error.");
            break;
        case MbResult::Exception:
            g_mbErrCount++;
            Serial.print("MODBUS: exception 0x");
            Serial.println(g_mbLastException, HEX);
            break;
        default:
            g_mbErrCount++;
            Serial.println("MODBUS: unknown error.");
            break;
    }
}

void handleCommandMbRaw(String args)
{
    String regTok = nextToken(args);
    String cntTok = nextToken(args);

    uint16_t reg = 0;
    if (!parseU16(regTok, reg)) {
        Serial.println("Usage: MBRAW <reg> [count]");
        return;
    }

    uint16_t count = 1;
    if (!cntTok.isEmpty() && !parseU16(cntTok, count)) {
        Serial.println("Usage: MBRAW <reg> [count]");
        return;
    }

    if (count == 0 || count > MODBUS_MAX_READ_REGS) {
        Serial.print("Count must be 1..");
        Serial.println(MODBUS_MAX_READ_REGS);
        return;
    }

    uint8_t req[8] = {
        g_modbusSlaveId,
        MODBUS_FUNC_READ_HOLDING,
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((count >> 8) & 0xFF),
        static_cast<uint8_t>(count & 0xFF),
        0,
        0
    };
    appendCrc(req, 6);

    uint8_t rx[MBRAW_MAX_RX_LEN] = {};

    Serial.print("MBRAW uart=");
    Serial.print(g_rs485Baud);
    Serial.print(" ");
    Serial.print(rs485SerialConfigName(g_rs485SerialConfig));
    Serial.print(", id=");
    Serial.print(g_modbusSlaveId);
    Serial.print(", reg=0x");
    Serial.print(reg, HEX);
    Serial.print(", count=");
    Serial.println(count);

    Serial.print("MBRAW TX[");
    Serial.print(sizeof(req));
    Serial.print("]: ");
    printBytesHex(req, sizeof(req));

    rs485DrainRx();
    rs485SendBytes(req, sizeof(req));
    g_mbReqCount++;

    bool hitTotalTimeout = false;
    const size_t rxLen = rs485ReadAvailableFrame(
        rx,
        sizeof(rx),
        MBRAW_FIRST_BYTE_TIMEOUT_MS,
        MBRAW_INTER_BYTE_TIMEOUT_MS,
        MBRAW_TOTAL_TIMEOUT_MS,
        &hitTotalTimeout);
    Serial.print("MBRAW RX[");
    Serial.print(rxLen);
    Serial.print("]: ");
    printBytesHex(rx, rxLen);

    const String txHex = bytesToHexString(req, sizeof(req));
    const String rxHex = bytesToHexString(rx, rxLen);

    if (rxLen == 0) {
        Serial.println("MBRAW verdict: timeout, no bytes received.");
        g_lastCommandResult = "MBRAW timeout; tx=" + txHex + "; rx=<none>";
        return;
    }

    if (hitTotalTimeout) {
        Serial.println("MBRAW verdict: continuous RX stream, no silent gap.");
        g_lastCommandResult = "MBRAW continuous-rx; tx=" + txHex + "; rx=" + rxHex;
        return;
    }

    if (rxLen == sizeof(req) && memcmp(rx, req, sizeof(req)) == 0) {
        Serial.println("MBRAW verdict: exact TX echo received.");
        g_lastCommandResult = "MBRAW echo; tx=" + txHex + "; rx=" + rxHex;
        return;
    }

    if (rxLen >= 3 && rx[0] == g_modbusSlaveId && rx[1] == MODBUS_FUNC_READ_HOLDING) {
        const uint8_t byteCount = rx[2];
        const size_t expectedLen = static_cast<size_t>(byteCount) + 5U;
        Serial.print("MBRAW looks like read response header, expected len=");
        Serial.println(expectedLen);
        if (rxLen == expectedLen) {
            Serial.print("MBRAW CRC: ");
            Serial.println(checkFrameCrc(rx, rxLen) ? "OK" : "BAD");
            g_lastCommandResult = "MBRAW read-like; rx=" + rxHex + "; crc=" +
                String(checkFrameCrc(rx, rxLen) ? "OK" : "BAD");
        } else {
            Serial.println("MBRAW CRC: skipped, frame length is incomplete/mismatched.");
            g_lastCommandResult = "MBRAW read-like; rx=" + rxHex + "; len=" + String(rxLen) +
                "; expected=" + String(expectedLen);
        }
        return;
    }

    if (rxLen >= 5 && rx[0] == g_modbusSlaveId && rx[1] == static_cast<uint8_t>(MODBUS_FUNC_READ_HOLDING | 0x80)) {
        Serial.print("MBRAW exception frame CRC: ");
        Serial.println(checkFrameCrc(rx, rxLen) ? "OK" : "BAD");
        g_lastCommandResult = "MBRAW exception-like; rx=" + rxHex + "; crc=" +
            String(checkFrameCrc(rx, rxLen) ? "OK" : "BAD");
        return;
    }

    if (rxLen >= 8 && memcmp(rx, req, 6) == 0) {
        Serial.println("MBRAW verdict: response starts like TX request, likely local echo.");
        g_lastCommandResult = "MBRAW likely echo; tx=" + txHex + "; rx=" + rxHex;
        return;
    }

    Serial.println("MBRAW verdict: bytes received, but frame does not match expected Modbus response.");
    g_lastCommandResult = "MBRAW weird; tx=" + txHex + "; rx=" + rxHex;
}
