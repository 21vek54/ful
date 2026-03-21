#include <Arduino.h>
#include <SPI.h>

#include <driver/spi_master.h>
#include <driver/spi_slave.h>

namespace {

constexpr uint16_t FRAME_MAGIC = 0xA55A;
constexpr uint8_t ROLE_ID_MASTER = 1;
constexpr uint8_t ROLE_ID_CONVEYOR = 12;
constexpr uint8_t ROLE_ID_MANIPULATOR = 13;
constexpr uint32_t POLL_INTERVAL_MS = 1000;
constexpr uint32_t SPI_CLOCK_HZ = 10000;
constexpr size_t FRAME_SIZE = 20;
constexpr size_t FRAME_BITS = FRAME_SIZE * 8;

struct __attribute__((packed)) SpiFrame {
  uint16_t magic;
  uint8_t role_id;
  uint8_t flags;
  uint32_t boot_nonce;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t checksum;
};

static_assert(sizeof(SpiFrame) == FRAME_SIZE, "Unexpected SpiFrame size");

uint32_t checksumFrame(const SpiFrame& frame) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&frame);
  uint32_t sum = 0x13572468u;
  for (size_t i = 0; i < sizeof(SpiFrame) - sizeof(frame.checksum); ++i) {
    sum = (sum << 5) | (sum >> 27);
    sum ^= bytes[i];
  }
  return sum;
}

bool frameLooksValid(const SpiFrame& frame, uint8_t expectedRole) {
  if (frame.magic != FRAME_MAGIC) {
    return false;
  }
  if (frame.role_id != expectedRole) {
    return false;
  }
  return checksumFrame(frame) == frame.checksum;
}

void finalizeFrame(SpiFrame& frame) {
  frame.magic = FRAME_MAGIC;
  frame.checksum = checksumFrame(frame);
}

void printFrame(const char* prefix, const SpiFrame& frame) {
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

uint32_t makeBootNonce(uint8_t roleId) {
  return (static_cast<uint32_t>(roleId) << 24) ^ micros() ^ 0x5A3C9E17u;
}

#if defined(ROLE_MASTER)

constexpr uint8_t PIN_SPI_SCK = 14;
constexpr uint8_t PIN_SPI_MOSI = 23;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_CS_CONVEYOR = 25;
constexpr uint8_t PIN_CS_MANIPULATOR = 26;

SPIClass spi(VSPI);

struct SlaveState {
  const char* name;
  uint8_t expectedRole;
  uint8_t csPin;
  uint32_t polls = 0;
  uint32_t ok = 0;
  uint32_t invalid = 0;
  uint32_t transfers = 0;
  uint32_t lastBootNonce = 0;
  uint32_t lastTxCount = 0;
  uint32_t lastRxCount = 0;
  uint32_t lastSeenMs = 0;
  bool online = false;
  SpiFrame lastFrame{};
};

SlaveState g_conveyor{"conveyor", ROLE_ID_CONVEYOR, PIN_CS_CONVEYOR};
SlaveState g_manipulator{"manipulator", ROLE_ID_MANIPULATOR, PIN_CS_MANIPULATOR};
uint32_t g_masterTxCount = 0;
uint32_t g_lastPollMs = 0;
bool g_autoPoll = true;

bool transferToSlave(SlaveState& slave) {
  SpiFrame tx{};
  tx.magic = FRAME_MAGIC;
  tx.role_id = ROLE_ID_MASTER;
  tx.flags = 0x01;
  tx.boot_nonce = 0x4D535452u;
  tx.tx_count = ++g_masterTxCount;
  tx.rx_count = slave.polls;
  finalizeFrame(tx);

  SpiFrame rx{};
  SPISettings settings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0);

  spi.beginTransaction(settings);
  digitalWrite(slave.csPin, LOW);
  delayMicroseconds(10);
  spi.transferBytes(reinterpret_cast<uint8_t*>(&tx), reinterpret_cast<uint8_t*>(&rx), sizeof(SpiFrame));
  delayMicroseconds(10);
  digitalWrite(slave.csPin, HIGH);
  spi.endTransaction();

  slave.polls++;
  slave.transfers++;
  slave.lastFrame = rx;

  if (frameLooksValid(rx, slave.expectedRole)) {
    slave.ok++;
    slave.online = true;
    slave.lastBootNonce = rx.boot_nonce;
    slave.lastTxCount = rx.tx_count;
    slave.lastRxCount = rx.rx_count;
    slave.lastSeenMs = millis();
    return true;
  }

  slave.invalid++;
  slave.online = false;
  return false;
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
  Serial.print(slave.online ? "yes" : "no");
  Serial.print(" boot=0x");
  Serial.print(slave.lastBootNonce, HEX);
  Serial.print(" tx=");
  Serial.print(slave.lastTxCount);
  Serial.print(" rx=");
  Serial.println(slave.lastRxCount);
  if (!slave.online) {
    printFrame("  last_raw: ", slave.lastFrame);
  }
}

void pollAll() {
  const bool convOk = transferToSlave(g_conveyor);
  const bool manipOk = transferToSlave(g_manipulator);

  Serial.print("POLL ");
  Serial.print(millis());
  Serial.print("ms | conveyor=");
  Serial.print(convOk ? "OK" : "BAD");
  Serial.print(" tx=");
  Serial.print(g_conveyor.lastTxCount);
  Serial.print(" rx=");
  Serial.print(g_conveyor.lastRxCount);
  Serial.print(" | manipulator=");
  Serial.print(manipOk ? "OK" : "BAD");
  Serial.print(" tx=");
  Serial.print(g_manipulator.lastTxCount);
  Serial.print(" rx=");
  Serial.println(g_manipulator.lastRxCount);
}

void printHelp() {
  Serial.println("SPI smoke-test master");
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

  pinMode(PIN_CS_CONVEYOR, OUTPUT);
  pinMode(PIN_CS_MANIPULATOR, OUTPUT);
  digitalWrite(PIN_CS_CONVEYOR, HIGH);
  digitalWrite(PIN_CS_MANIPULATOR, HIGH);

  spi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  Serial.println();
  Serial.println("SPI smoke-test: master");
  Serial.print("SCK=");
  Serial.print(PIN_SPI_SCK);
  Serial.print(" MOSI=");
  Serial.print(PIN_SPI_MOSI);
  Serial.print(" MISO=");
  Serial.print(PIN_SPI_MISO);
  Serial.print(" CS_CONVEYOR=");
  Serial.print(PIN_CS_CONVEYOR);
  Serial.print(" CS_MANIP=");
  Serial.println(PIN_CS_MANIPULATOR);
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
constexpr int PIN_SPI_MISO = 18;
constexpr int PIN_SPI_MOSI = 17;
constexpr int PIN_SPI_SCLK = 16;
constexpr int PIN_SPI_CS = 21;
#elif defined(ROLE_MANIPULATOR)
constexpr uint8_t ROLE_ID = ROLE_ID_MANIPULATOR;
constexpr const char* ROLE_NAME = "manipulator";
constexpr int PIN_SPI_MISO = 13;
constexpr int PIN_SPI_MOSI = 17;
constexpr int PIN_SPI_SCLK = 16;
constexpr int PIN_SPI_CS = 4;
#else
#error Role is not defined
#endif

DMA_ATTR SpiFrame g_txFrame{};
DMA_ATTR SpiFrame g_rxFrame{};
uint32_t g_validRx = 0;
uint32_t g_invalidRx = 0;
uint32_t g_txPrepared = 0;
uint32_t g_bootNonce = 0;
uint32_t g_emptyRx = 0;
uint32_t g_shortRx = 0;
uint32_t g_waitTimeouts = 0;
size_t g_lastTransBits = 0;
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

void printState() {
  Serial.print(ROLE_NAME);
  Serial.print(": valid_rx=");
  Serial.print(g_validRx);
  Serial.print(" invalid_rx=");
  Serial.print(g_invalidRx);
  Serial.print(" empty_rx=");
  Serial.print(g_emptyRx);
  Serial.print(" short_rx=");
  Serial.print(g_shortRx);
  Serial.print(" wait_to=");
  Serial.print(g_waitTimeouts);
  Serial.print(" tx_prepared=");
  Serial.print(g_txPrepared);
  Serial.print(" last_bits=");
  Serial.print(g_lastTransBits);
  Serial.print(" boot=0x");
  Serial.println(g_bootNonce, HEX);
  printFrame("  tx_frame: ", g_txFrame);
  printFrame("  rx_frame: ", g_rxFrame);
}

void printHelp() {
  Serial.print("SPI smoke-test: ");
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

  spi_bus_config_t buscfg{};
  buscfg.mosi_io_num = PIN_SPI_MOSI;
  buscfg.miso_io_num = PIN_SPI_MISO;
  buscfg.sclk_io_num = PIN_SPI_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = sizeof(SpiFrame);

  spi_slave_interface_config_t slvcfg{};
  slvcfg.mode = 0;
  slvcfg.spics_io_num = PIN_SPI_CS;
  slvcfg.queue_size = 1;
  slvcfg.flags = 0;

  // For a 20-byte frame on classic ESP32, DMA in mode 0 is unnecessary and can
  // tighten MISO timing enough to corrupt the tail of the frame.
  const esp_err_t busErr = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);
  Serial.println();
  Serial.print("SPI smoke-test: ");
  Serial.println(ROLE_NAME);
  Serial.print("SCK=");
  Serial.print(PIN_SPI_SCLK);
  Serial.print(" MOSI=");
  Serial.print(PIN_SPI_MOSI);
  Serial.print(" MISO=");
  Serial.print(PIN_SPI_MISO);
  Serial.print(" CS=");
  Serial.println(PIN_SPI_CS);
  Serial.print("spi_slave_initialize: ");
  Serial.println(busErr == ESP_OK ? "OK" : "FAIL");
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

  spi_slave_transaction_t trans{};
  memset(&g_rxFrame, 0, sizeof(g_rxFrame));
  trans.length = FRAME_BITS;
  trans.tx_buffer = &g_txFrame;
  trans.rx_buffer = &g_rxFrame;

  const esp_err_t err = spi_slave_transmit(SPI2_HOST, &trans, pdMS_TO_TICKS(250));
  if (err == ESP_OK) {
    g_lastTransBits = trans.trans_len;
    if (trans.trans_len == 0) {
      g_emptyRx++;
    } else if (trans.trans_len != FRAME_BITS) {
      g_shortRx++;
      g_invalidRx++;
      if (g_trace) {
        Serial.print("RX_LEN_BAD: bits=");
        Serial.println(g_lastTransBits);
        printFrame("RX_PARTIAL: ", g_rxFrame);
      }
    } else if (frameLooksValid(g_rxFrame, ROLE_ID_MASTER)) {
      g_validRx++;
      if (g_trace) {
        printFrame("RX: ", g_rxFrame);
      }
    } else {
      g_invalidRx++;
      if (g_trace) {
        printFrame("RX_BAD: ", g_rxFrame);
      }
    }
    prepareFrame();
  } else if (err == ESP_ERR_TIMEOUT) {
    g_waitTimeouts++;
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
