#pragma once

#include <Arduino.h>

#include "master_types.h"

inline constexpr uint32_t RS485_BAUD = 9600;
inline constexpr uint32_t TX_SETTLE_US = 150;
inline constexpr size_t RS485_BUFFER_MAX_LEN = 96;
inline constexpr bool RS485_TEXT_SNIFFER_ENABLED = false;

inline constexpr uint32_t MODBUS_TIMEOUT_MS = 300;
inline constexpr uint16_t MODBUS_MAX_READ_REGS = 32;
inline constexpr uint8_t MODBUS_FUNC_READ_HOLDING = 0x03;
inline constexpr uint8_t MODBUS_FUNC_WRITE_SINGLE = 0x06;
inline constexpr uint32_t MBRAW_FIRST_BYTE_TIMEOUT_MS = 500;
inline constexpr uint32_t MBRAW_INTER_BYTE_TIMEOUT_MS = 60;
inline constexpr uint32_t MBRAW_TOTAL_TIMEOUT_MS = 2000;
inline constexpr size_t MBRAW_MAX_RX_LEN = 96;
inline constexpr uint8_t MODBUS_RETRY_COUNT = 3;

inline constexpr uint8_t RS485_DIAG_SELF_ID = 10;
inline constexpr uint8_t RS485_DIAG_MAGIC_0 = 0xFA;
inline constexpr uint8_t RS485_DIAG_MAGIC_1 = 0x55;
inline constexpr uint8_t RS485_DIAG_CMD_PING = 0x01;
inline constexpr uint8_t RS485_DIAG_CMD_PONG = 0x02;
inline constexpr size_t RS485_DIAG_FRAME_LEN = 8;
inline constexpr uint32_t RS485_DIAG_TIMEOUT_MS = 300;

inline constexpr uint16_t VFD_REG_CMD = 0x2000;
inline constexpr uint16_t VFD_REG_STATUS = 0x3000;
inline constexpr uint16_t VFD_REG_FREQ_SET = 0x1000;
inline constexpr uint16_t VFD_REG_COMM_FREQ_HZ_SET = 0x9000;
inline constexpr uint16_t VFD_REG_FREQ_RUN = 0x1002;
inline constexpr uint16_t VFD_REG_FAULT = 0x8000;
inline constexpr uint16_t VFD_REG_PANEL_FREQ_SET = 0xF00B;
inline constexpr uint16_t VFD_REG_FREQ_DECIMALS = 0xF014;
inline constexpr uint16_t VFD_REG_P0_04 = 0xF004;
inline constexpr uint16_t VFD_REG_P0_06 = 0xF006;
inline constexpr uint16_t VFD_REG_P8_00 = 0xF800;
inline constexpr uint16_t VFD_REG_P8_01 = 0xF801;
inline constexpr uint16_t VFD_REG_P8_02 = 0xF802;
inline constexpr uint16_t VFD_REG_P8_05 = 0xF805;
inline constexpr uint16_t VFD_REG_P8_06 = 0xF806;

inline constexpr uint16_t VFD_CMD_FWD = 0x0001;
inline constexpr uint16_t VFD_CMD_REV = 0x0002;
inline constexpr uint16_t VFD_CMD_STOP_DEC = 0x0006;
inline constexpr uint16_t VFD_CMD_RESET = 0x0007;

inline constexpr int32_t VFD_FREQ_RAW_MIN = -10000;
inline constexpr int32_t VFD_FREQ_RAW_MAX = 10000;
inline constexpr float VFD_RUN_FREQ_SCALE = 10.0F;
inline constexpr float VFD_COMM_SCALE = 10000.0F;
inline constexpr uint16_t VFD_CTRL_SOURCE_RS485 = 2;
inline constexpr uint16_t VFD_FREQ_SOURCE_RS485 = 7;
inline constexpr uint16_t VFD_RS485_BAUD_9600 = 5;
inline constexpr uint16_t VFD_RS485_FORMAT_8E1 = 1;
inline constexpr uint16_t VFD_RS485_PROTO_MODBUS_STD = 0;
inline constexpr uint16_t VFD_RS485_REMOTE_MONITOR_ENABLE = 0;

void rs485SetReceiveMode();
void rs485SetTransmitMode();
void rs485DrainRx();
bool parseRs485SerialConfigToken(String token, uint32_t &serialConfigOut);
void rs485ApplyUart(uint32_t baud, uint32_t serialConfig);
void printRs485Uart();
bool rs485ReadExact(uint8_t *dst, size_t len, uint32_t timeoutMs);
void appendCrc(uint8_t *frame, size_t payloadLen);
bool checkFrameCrc(const uint8_t *frame, size_t frameLen);

MbResult modbusReadHoldingFromSlave(
    uint8_t slaveId,
    uint16_t reg,
    uint16_t count,
    uint16_t *outRegs,
    uint8_t *exceptionOut = nullptr,
    bool countAsRequest = true);

MbResult modbusReadHolding(uint16_t reg, uint16_t count, uint16_t *outRegs);
MbResult modbusWriteSingleToSlave(uint8_t slaveId, uint16_t reg, uint16_t value, bool countAsRequest = true);
MbResult modbusWriteSingle(uint16_t reg, uint16_t value);
MbResult modbusReadHoldingRetryFromSlave(
    uint8_t slaveId,
    uint16_t reg,
    uint16_t count,
    uint16_t *outRegs,
    uint8_t attempts,
    bool countAsRequest = true);
MbResult modbusWriteSingleRetryToSlave(uint8_t slaveId, uint16_t reg, uint16_t value, uint8_t attempts);
MbResult modbusWriteSingleRetry(uint16_t reg, uint16_t value, uint8_t attempts);
MbResult modbusReadHoldingRetry(uint16_t reg, uint16_t count, uint16_t *outRegs, uint8_t attempts);

bool hzToPanelRawU16(float hz, float scale, uint16_t &rawOut);
MbResult readVfdFrequencyScale(float &scaleOut);
MbResult writeVfdHzSetpoint(float hz, float scale, uint16_t &rawOut);
void printVfdConfigSummary(uint8_t slaveId);
bool setupVr70ForMasterRs485(uint8_t targetSlaveId);
void printMbResult(MbResult result);
void handleCommandMbRaw(String args);
