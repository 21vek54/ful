#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint16_t FRAME_MAGIC = 0xA55A;
constexpr uint8_t ROLE_ID_MASTER = 1;
constexpr uint8_t ROLE_ID_CONVEYOR = 12;
constexpr uint8_t ROLE_ID_MANIPULATOR = 13;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint32_t POLL_INTERVAL_MS = 1000;
constexpr size_t FRAME_SIZE = 20;

struct __attribute__((packed)) I2cFrame {
  uint16_t magic;
  uint8_t role_id;
  uint8_t flags;
  uint32_t boot_nonce;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t checksum;
};

static_assert(sizeof(I2cFrame) == FRAME_SIZE, "Unexpected I2cFrame size");

uint32_t checksumFrame(const I2cFrame& frame) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&frame);
  uint32_t sum = 0x13572468u;
  for (size_t i = 0; i < sizeof(I2cFrame) - sizeof(frame.checksum); ++i) {
    sum = (sum << 5) | (sum >> 27);
    sum ^= bytes[i];
  }
  return sum;
}

void finalizeFrame(I2cFrame& frame) {
  frame.magic = FRAME_MAGIC;
  frame.checksum = checksumFrame(frame);
}

bool frameLooksValid(const I2cFrame& frame, uint8_t expectedRole) {
  return frame.magic == FRAME_MAGIC &&
         frame.role_id == expectedRole &&
         checksumFrame(frame) == frame.checksum;
}

uint32_t makeBootNonce(uint8_t roleId) {
  return (static_cast<uint32_t>(roleId) << 24) ^ micros() ^ 0x5A3C9E17u;
}

void printFrame(const char* prefix, const I2cFrame& frame) {
  Serial.print(prefix);
  Serial.print("magic=0x");
  Serial.print(frame.magic, HEX);
  Serial.print(" role=");
  Serial.print(frame.role_id);
  Serial.print(" flags=");
  Serial.print(frame.flags);
  Serial.print(" boot=0x");
  Serial.print(frame.boot_nonce, HEX);
  Serial.print(" tx=");
  Serial.print(frame.tx_count);
  Serial.print(" rx=");
  Serial.print(frame.rx_count);
  Serial.print(" csum=0x");
  Serial.println(frame.checksum, HEX);
}

#if defined(ROLE_MASTER)

constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

struct SlaveState {
  const char* name;
  uint8_t address;
  uint8_t expectedRole;
  uint32_t polls = 0;
  uint32_t ok = 0;
  uint32_t invalid = 0;
  bool online = false;
  uint32_t lastSeenMs = 0;
  I2cFrame lastFrame{};
};

SlaveState g_conveyor{"conveyor", ROLE_ID_CONVEYOR, ROLE_ID_CONVEYOR};
SlaveState g_manipulator{"manipulator", ROLE_ID_MANIPULATOR, ROLE_ID_MANIPULATOR};
uint32_t g_masterTxCount = 0;
uint32_t g_lastPollMs = 0;
bool g_autoPoll = true;

bool readFrame(uint8_t address, I2cFrame& frame) {
  memset(&frame, 0, sizeof(frame));
  const size_t want = sizeof(frame);
  const size_t got = Wire.requestFrom(static_cast<int>(address), static_cast<int>(want), true);
  if (got != want) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    return false;
  }
  uint8_t* dst = reinterpret_cast<uint8_t*>(&frame);
  for (size_t i = 0; i < want && Wire.available() > 0; ++i) {
    dst[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool pokeSlave(uint8_t address) {
  I2cFrame tx{};
  tx.magic = FRAME_MAGIC;
  tx.role_id = ROLE_ID_MASTER;
  tx.flags = 1;
  tx.boot_nonce = 0x4D535452u;
  tx.tx_count = ++g_masterTxCount;
  finalizeFrame(tx);

  Wire.beginTransmission(address);
  const uint8_t* src = reinterpret_cast<const uint8_t*>(&tx);
  Wire.write(src, sizeof(tx));
  return Wire.endTransmission(true) == 0;
}

bool pollSlave(SlaveState& slave) {
  slave.polls++;
  if (!pokeSlave(slave.address)) {
    slave.invalid++;
    slave.online = false;
    memset(&slave.lastFrame, 0, sizeof(slave.lastFrame));
    return false;
  }
  if (!readFrame(slave.address, slave.lastFrame)) {
    slave.invalid++;
    slave.online = false;
    return false;
  }
  if (!frameLooksValid(slave.lastFrame, slave.expectedRole)) {
    slave.invalid++;
    slave.online = false;
    return false;
  }
  slave.ok++;
  slave.online = true;
  slave.lastSeenMs = millis();
  return true;
}

void printSlaveState(const SlaveState& slave) {
  Serial.print(slave.name);
  Serial.print(": polls=");
  Serial.print(slave.polls);
  Serial.print(" ok=");
  Serial.print(slave.ok);
  Serial.print(" invalid=");
  Serial.print(slave.invalid);
  Serial.print(" online=");
  Serial.println(slave.online ? "yes" : "no");
  printFrame("  last_raw: ", slave.lastFrame);
}

void pollAll() {
  const bool convOk = pollSlave(g_conveyor);
  const bool manipOk = pollSlave(g_manipulator);
  Serial.print("POLL ");
  Serial.print(millis());
  Serial.print("ms | conveyor=");
  Serial.print(convOk ? "OK" : "BAD");
  Serial.print(" | manipulator=");
  Serial.println(manipOk ? "OK" : "BAD");
}

void printHelp() {
  Serial.println("I2C smoke-test master");
  Serial.println("Commands:");
  Serial.println("  POLL      - poll conveyor and manipulator once");
  Serial.println("  STATE     - print counters and last frames");
  Serial.println("  AUTO ON   - enable periodic polling");
  Serial.println("  AUTO OFF  - disable periodic polling");
  Serial.println("  H         - help");
}

void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.isEmpty()) {
    return;
  }
  if (line == "POLL") {
    pollAll();
    return;
  }
  if (line == "STATE") {
    printSlaveState(g_conveyor);
    printSlaveState(g_manipulator);
    return;
  }
  if (line == "AUTO ON") {
    g_autoPoll = true;
    Serial.println("AUTO: ON");
    return;
  }
  if (line == "AUTO OFF") {
    g_autoPoll = false;
    Serial.println("AUTO: OFF");
    return;
  }
  if (line == "H" || line == "HELP" || line == "?") {
    printHelp();
    return;
  }
  Serial.print("Unknown command: ");
  Serial.println(line);
}

void appSetup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);

  Serial.println();
  Serial.println("I2C smoke-test: master");
  Serial.print("SDA=");
  Serial.print(PIN_I2C_SDA);
  Serial.print(" SCL=");
  Serial.println(PIN_I2C_SCL);
  printHelp();
}

void appLoop() {
  static String line;
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleLine(line);
      line = "";
    } else {
      line += ch;
    }
  }

  if (g_autoPoll && millis() - g_lastPollMs >= POLL_INTERVAL_MS) {
    g_lastPollMs = millis();
    pollAll();
  }
}

#else

#if defined(ROLE_CONVEYOR)
constexpr uint8_t ROLE_ID = ROLE_ID_CONVEYOR;
constexpr const char* ROLE_NAME = "conveyor";
constexpr uint8_t I2C_ADDRESS = ROLE_ID_CONVEYOR;
#elif defined(ROLE_MANIPULATOR)
constexpr uint8_t ROLE_ID = ROLE_ID_MANIPULATOR;
constexpr const char* ROLE_NAME = "manipulator";
constexpr uint8_t I2C_ADDRESS = ROLE_ID_MANIPULATOR;
#else
#error Role is not defined
#endif

constexpr int PIN_I2C_SDA = 16;
constexpr int PIN_I2C_SCL = 17;

volatile bool g_haveNewRx = false;
volatile uint32_t g_validRx = 0;
volatile uint32_t g_invalidRx = 0;
volatile uint32_t g_txPrepared = 0;
volatile uint32_t g_lastRxLen = 0;
I2cFrame g_txFrame{};
I2cFrame g_rxFrame{};
uint32_t g_bootNonce = 0;
bool g_trace = false;

void prepareFrame() {
  g_txFrame.magic = FRAME_MAGIC;
  g_txFrame.role_id = ROLE_ID;
  g_txFrame.flags = g_trace ? 1 : 0;
  g_txFrame.boot_nonce = g_bootNonce;
  g_txFrame.tx_count = ++g_txPrepared;
  g_txFrame.rx_count = g_validRx;
  finalizeFrame(g_txFrame);
}

void onReceiveI2c(int count) {
  g_lastRxLen = static_cast<uint32_t>(count);
  memset(&g_rxFrame, 0, sizeof(g_rxFrame));
  uint8_t* dst = reinterpret_cast<uint8_t*>(&g_rxFrame);
  size_t idx = 0;
  while (Wire.available() > 0 && idx < sizeof(g_rxFrame)) {
    dst[idx++] = static_cast<uint8_t>(Wire.read());
  }
  if (idx == sizeof(g_rxFrame) && frameLooksValid(g_rxFrame, ROLE_ID_MASTER)) {
    g_validRx++;
  } else {
    g_invalidRx++;
  }
  prepareFrame();
  g_haveNewRx = true;
}

void onRequestI2c() {
  Wire.write(reinterpret_cast<const uint8_t*>(&g_txFrame), sizeof(g_txFrame));
}

void printState() {
  Serial.print(ROLE_NAME);
  Serial.print(": valid_rx=");
  Serial.print(g_validRx);
  Serial.print(" invalid_rx=");
  Serial.print(g_invalidRx);
  Serial.print(" tx_prepared=");
  Serial.print(g_txPrepared);
  Serial.print(" last_rx_len=");
  Serial.print(g_lastRxLen);
  Serial.print(" boot=0x");
  Serial.println(g_bootNonce, HEX);
  printFrame("  tx_frame: ", g_txFrame);
  printFrame("  rx_frame: ", g_rxFrame);
}

void printHelp() {
  Serial.print("I2C smoke-test: ");
  Serial.println(ROLE_NAME);
  Serial.println("Commands:");
  Serial.println("  STATE      - print counters and current frames");
  Serial.println("  TRACE ON   - print valid RX frames");
  Serial.println("  TRACE OFF  - disable RX tracing");
  Serial.println("  H          - help");
}

void handleLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.isEmpty()) {
    return;
  }
  if (line == "STATE") {
    printState();
    return;
  }
  if (line == "TRACE ON") {
    g_trace = true;
    g_txFrame.flags = 1;
    finalizeFrame(g_txFrame);
    Serial.println("TRACE: ON");
    return;
  }
  if (line == "TRACE OFF") {
    g_trace = false;
    g_txFrame.flags = 0;
    finalizeFrame(g_txFrame);
    Serial.println("TRACE: OFF");
    return;
  }
  if (line == "H" || line == "HELP" || line == "?") {
    printHelp();
    return;
  }
  Serial.print("Unknown command: ");
  Serial.println(line);
}

void appSetup() {
  Serial.begin(115200);
  delay(300);

  g_bootNonce = makeBootNonce(ROLE_ID);
  prepareFrame();
  memset(&g_rxFrame, 0, sizeof(g_rxFrame));

  Wire.begin(I2C_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.onReceive(onReceiveI2c);
  Wire.onRequest(onRequestI2c);

  Serial.println();
  Serial.print("I2C smoke-test: ");
  Serial.println(ROLE_NAME);
  Serial.print("ADDR=0x");
  Serial.print(I2C_ADDRESS, HEX);
  Serial.print(" SDA=");
  Serial.print(PIN_I2C_SDA);
  Serial.print(" SCL=");
  Serial.println(PIN_I2C_SCL);
  printHelp();
}

void appLoop() {
  static String line;
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleLine(line);
      line = "";
    } else {
      line += ch;
    }
  }

  if (g_haveNewRx) {
    g_haveNewRx = false;
    if (g_trace) {
      if (frameLooksValid(g_rxFrame, ROLE_ID_MASTER)) {
        printFrame("RX: ", g_rxFrame);
      } else {
        printFrame("RX_BAD: ", g_rxFrame);
      }
    }
  }
}

#endif

}  // namespace

void setup() {
  appSetup();
}

void loop() {
  appLoop();
}
