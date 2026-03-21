#include <Arduino.h>

#ifndef NODE_ID
#define NODE_ID 3
#endif

#ifndef TARGET_ID
#define TARGET_ID 4
#endif

#ifndef AUTO_PING_ENABLED
#define AUTO_PING_ENABLED 0
#endif

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t RS485_BAUD = 9600;
constexpr uint8_t PIN_RS485_RX = 16;
constexpr uint8_t PIN_RS485_TX = 17;
constexpr uint8_t PIN_RS485_DE_RE = 18;
constexpr uint32_t AUTO_PING_INTERVAL_MS = 1000;
constexpr uint32_t RESPONSE_TIMEOUT_MS = 300;
constexpr uint32_t READ_GAP_MS = 20;
constexpr uint32_t TX_SETTLE_US = 120;
constexpr uint8_t FRAME_MAGIC_0 = 0xFA;
constexpr uint8_t FRAME_MAGIC_1 = 0x55;
constexpr uint8_t CMD_PING = 0x01;
constexpr uint8_t CMD_PONG = 0x02;
constexpr size_t FRAME_LEN = 8;

struct Counters {
  uint32_t pingTx = 0;
  uint32_t pingRx = 0;
  uint32_t pongTx = 0;
  uint32_t pongRx = 0;
  uint32_t crcErr = 0;
  uint32_t timeoutErr = 0;
  uint32_t protoErr = 0;
} g_stats;

uint8_t g_token = 0;
uint32_t g_lastPingMs = 0;
bool g_autoPingEnabled = (AUTO_PING_ENABLED != 0);
String g_cmdBuffer;

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001U) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001U);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

void buildFrame(uint8_t dst, uint8_t src, uint8_t cmd, uint8_t token, uint8_t *frame) {
  frame[0] = FRAME_MAGIC_0;
  frame[1] = FRAME_MAGIC_1;
  frame[2] = dst;
  frame[3] = src;
  frame[4] = cmd;
  frame[5] = token;
  const uint16_t crc = crc16(frame, 6);
  frame[6] = static_cast<uint8_t>(crc & 0xFFU);
  frame[7] = static_cast<uint8_t>((crc >> 8) & 0xFFU);
}

bool isValidFrame(const uint8_t *frame) {
  if (frame[0] != FRAME_MAGIC_0 || frame[1] != FRAME_MAGIC_1) {
    return false;
  }
  const uint16_t got = static_cast<uint16_t>(frame[6]) |
                       (static_cast<uint16_t>(frame[7]) << 8);
  return crc16(frame, 6) == got;
}

void printHex(const char *prefix, const uint8_t *frame) {
  Serial.print(prefix);
  for (size_t i = 0; i < FRAME_LEN; i++) {
    if (i > 0) {
      Serial.print(' ');
    }
    if (frame[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame[i], HEX);
  }
  Serial.println();
}

void drainRs485() {
  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
}

void rs485SetReceiveMode() {
  digitalWrite(PIN_RS485_DE_RE, LOW);
}

void rs485SetTransmitMode() {
  digitalWrite(PIN_RS485_DE_RE, HIGH);
}

bool readExact(uint8_t *dst, size_t len, uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  size_t offset = 0;
  while (offset < len) {
    const int c = Serial2.read();
    if (c >= 0) {
      dst[offset++] = static_cast<uint8_t>(c);
      continue;
    }
    if ((uint32_t)(millis() - startMs) >= timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

void sendFrame(uint8_t dst, uint8_t cmd, uint8_t token) {
  uint8_t frame[FRAME_LEN] = {};
  buildFrame(dst, NODE_ID, cmd, token, frame);
  rs485SetTransmitMode();
  Serial2.write(frame, sizeof(frame));
  Serial2.flush();
  delayMicroseconds(TX_SETTLE_US);
  rs485SetReceiveMode();
  printHex("RS TX: ", frame);
}

void printStats() {
  Serial.print("stats: ping_tx=");
  Serial.print(g_stats.pingTx);
  Serial.print(" ping_rx=");
  Serial.print(g_stats.pingRx);
  Serial.print(" pong_tx=");
  Serial.print(g_stats.pongTx);
  Serial.print(" pong_rx=");
  Serial.print(g_stats.pongRx);
  Serial.print(" crc_err=");
  Serial.print(g_stats.crcErr);
  Serial.print(" timeout_err=");
  Serial.print(g_stats.timeoutErr);
  Serial.print(" proto_err=");
  Serial.println(g_stats.protoErr);
}

void handleIncomingFrame(const uint8_t *frame) {
  printHex("RS RX: ", frame);

  if (!isValidFrame(frame)) {
    g_stats.crcErr++;
    Serial.println("frame: crc error");
    return;
  }

  const uint8_t dst = frame[2];
  const uint8_t src = frame[3];
  const uint8_t cmd = frame[4];
  const uint8_t token = frame[5];

  if (dst != NODE_ID && dst != 0xFFU) {
    return;
  }

  if (cmd == CMD_PING) {
    g_stats.pingRx++;
    Serial.print("ping rx from ");
    Serial.print(src);
    Serial.print(", token=");
    Serial.println(token);
    sendFrame(src, CMD_PONG, token);
    g_stats.pongTx++;
    return;
  }

  if (cmd == CMD_PONG) {
    g_stats.pongRx++;
    Serial.print("pong rx from ");
    Serial.print(src);
    Serial.print(", token=");
    Serial.println(token);
    return;
  }

  g_stats.protoErr++;
  Serial.print("frame: unknown cmd 0x");
  Serial.println(cmd, HEX);
}

bool receiveOneFrame(uint32_t timeoutMs, uint8_t *frame) {
  const uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < timeoutMs) {
    const int first = Serial2.peek();
    if (first < 0) {
      delay(1);
      continue;
    }
    if (static_cast<uint8_t>(first) != FRAME_MAGIC_0) {
      (void)Serial2.read();
      continue;
    }
    if (!readExact(frame, FRAME_LEN, READ_GAP_MS)) {
      delay(1);
      continue;
    }
    return true;
  }
  return false;
}

void sendPingAndWait() {
  const uint8_t token = static_cast<uint8_t>(++g_token);
  drainRs485();
  sendFrame(TARGET_ID, CMD_PING, token);
  g_stats.pingTx++;

  uint8_t frame[FRAME_LEN] = {};
  if (!receiveOneFrame(RESPONSE_TIMEOUT_MS, frame)) {
    g_stats.timeoutErr++;
    Serial.println("ping: timeout");
    return;
  }

  handleIncomingFrame(frame);
  if (!isValidFrame(frame) ||
      frame[2] != NODE_ID ||
      frame[3] != TARGET_ID ||
      frame[4] != CMD_PONG ||
      frame[5] != token) {
    g_stats.protoErr++;
    Serial.println("ping: unexpected reply");
    return;
  }

  Serial.print("ping: ok from ");
  Serial.println(TARGET_ID);
}

void processRs485() {
  while (Serial2.available() >= static_cast<int>(FRAME_LEN)) {
    if (Serial2.peek() != FRAME_MAGIC_0) {
      (void)Serial2.read();
      continue;
    }
    uint8_t frame[FRAME_LEN] = {};
    if (!readExact(frame, FRAME_LEN, READ_GAP_MS)) {
      return;
    }
    handleIncomingFrame(frame);
  }
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  PING          - send ping to target and wait for pong");
  Serial.println("  AUTO ON       - enable auto ping");
  Serial.println("  AUTO OFF      - disable auto ping");
  Serial.println("  AUTO STATUS   - show auto ping state");
  Serial.println("  STATS         - print counters");
  Serial.println("  H             - help");
}

void handleConsoleLine(String line) {
  line.trim();
  if (line.isEmpty()) {
    return;
  }

  String cmd = line;
  String args;
  const int sp = cmd.indexOf(' ');
  if (sp > 0) {
    args = cmd.substring(sp + 1);
    cmd = cmd.substring(0, sp);
  } else if (sp == 0) {
    cmd = "";
  }

  cmd.toUpperCase();
  args.trim();
  args.toUpperCase();

  if (cmd == "PING") {
    sendPingAndWait();
    return;
  }

  if (cmd == "AUTO") {
    if (args == "ON") {
      g_autoPingEnabled = true;
      Serial.println("auto: on");
      return;
    }
    if (args == "OFF") {
      g_autoPingEnabled = false;
      Serial.println("auto: off");
      return;
    }
    Serial.print("auto: ");
    Serial.println(g_autoPingEnabled ? "on" : "off");
    return;
  }

  if (cmd == "STATS") {
    printStats();
    return;
  }

  printHelp();
}

void processConsole() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      if (!g_cmdBuffer.isEmpty()) {
        handleConsoleLine(g_cmdBuffer);
        g_cmdBuffer = "";
      }
      continue;
    }
    g_cmdBuffer += ch;
  }
}

void maybeAutoPing() {
  if (!g_autoPingEnabled) {
    return;
  }
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastPingMs) < AUTO_PING_INTERVAL_MS) {
    return;
  }
  g_lastPingMs = now;
  sendPingAndWait();
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_RS485_DE_RE, OUTPUT);
  rs485SetReceiveMode();
  Serial2.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  delay(200);

  Serial.println();
  Serial.println("=== RS485 AutoDir Test ===");
  Serial.print("node_id=");
  Serial.print(NODE_ID);
  Serial.print(" target_id=");
  Serial.print(TARGET_ID);
  Serial.print(" auto_ping=");
  Serial.println(g_autoPingEnabled ? "on" : "off");
  Serial.println("UART2: RX=16 TX=17 DE/RE=18 9600 8N1");
  printHelp();
}

void loop() {
  processConsole();
  processRs485();
  maybeAutoPing();
}
