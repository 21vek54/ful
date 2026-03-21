#include <Arduino.h>

namespace {

#if defined(ROLE_MASTER)
constexpr uint8_t PIN_SCK = 14;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_CS_CONVEYOR = 25;
constexpr uint8_t PIN_CS_MANIPULATOR = 26;
constexpr const char* ROLE_NAME = "master";
#elif defined(ROLE_CONVEYOR)
constexpr uint8_t PIN_SCK = 16;
constexpr uint8_t PIN_MOSI = 17;
constexpr uint8_t PIN_MISO = 18;
constexpr uint8_t PIN_CS = 21;
constexpr const char* ROLE_NAME = "conveyor";
#elif defined(ROLE_MANIPULATOR)
constexpr uint8_t PIN_SCK = 16;
constexpr uint8_t PIN_MOSI = 17;
constexpr uint8_t PIN_MISO = 13;
constexpr uint8_t PIN_CS = 4;
constexpr const char* ROLE_NAME = "manipulator";
#else
#error Role is not defined
#endif

void printHelp() {
  Serial.print("SPI line-check: ");
  Serial.println(ROLE_NAME);
  Serial.println("Commands:");
#if defined(ROLE_MASTER)
  Serial.println("  SET SCK 0|1");
  Serial.println("  SET MOSI 0|1");
  Serial.println("  SET CSC 0|1");
  Serial.println("  SET CSM 0|1");
  Serial.println("  READ");
  Serial.println("  STATE");
#else
  Serial.println("  READ");
  Serial.println("  MISO IN");
  Serial.println("  MISO 0");
  Serial.println("  MISO 1");
  Serial.println("  STATE");
#endif
  Serial.println("  H");
}

#if defined(ROLE_MASTER)
void setPinLevel(uint8_t pin, int level) {
  digitalWrite(pin, level ? HIGH : LOW);
}

void printState() {
  Serial.print("STATE SCK=");
  Serial.print(digitalRead(PIN_SCK));
  Serial.print(" MOSI=");
  Serial.print(digitalRead(PIN_MOSI));
  Serial.print(" CSC=");
  Serial.print(digitalRead(PIN_CS_CONVEYOR));
  Serial.print(" CSM=");
  Serial.print(digitalRead(PIN_CS_MANIPULATOR));
  Serial.print(" MISO=");
  Serial.println(digitalRead(PIN_MISO));
}

void handleLine(String line) {
  line.trim();
  if (line.isEmpty()) {
    return;
  }
  String upper = line;
  upper.toUpperCase();

  if (upper == "READ" || upper == "STATE") {
    printState();
    return;
  }
  if (upper == "H" || upper == "HELP" || upper == "?") {
    printHelp();
    return;
  }

  int lastSpace = upper.lastIndexOf(' ');
  if (upper.startsWith("SET ") && lastSpace > 0) {
    const String signal = upper.substring(4, lastSpace);
    const int level = upper.substring(lastSpace + 1).toInt();
    if (signal == "SCK") {
      setPinLevel(PIN_SCK, level);
    } else if (signal == "MOSI") {
      setPinLevel(PIN_MOSI, level);
    } else if (signal == "CSC") {
      setPinLevel(PIN_CS_CONVEYOR, level);
    } else if (signal == "CSM") {
      setPinLevel(PIN_CS_MANIPULATOR, level);
    } else {
      Serial.print("Unknown signal: ");
      Serial.println(signal);
      return;
    }
    printState();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
}

void appSetup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_CS_CONVEYOR, OUTPUT);
  pinMode(PIN_CS_MANIPULATOR, OUTPUT);
  pinMode(PIN_MISO, INPUT);

  digitalWrite(PIN_SCK, LOW);
  digitalWrite(PIN_MOSI, LOW);
  digitalWrite(PIN_CS_CONVEYOR, LOW);
  digitalWrite(PIN_CS_MANIPULATOR, LOW);

  Serial.println();
  Serial.println("SPI line-check: master");
  Serial.print("SCK=");
  Serial.print(PIN_SCK);
  Serial.print(" MOSI=");
  Serial.print(PIN_MOSI);
  Serial.print(" MISO=");
  Serial.print(PIN_MISO);
  Serial.print(" CSC=");
  Serial.print(PIN_CS_CONVEYOR);
  Serial.print(" CSM=");
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
}

#else

enum class MisoMode {
  Input,
  OutputLow,
  OutputHigh,
};

MisoMode g_misoMode = MisoMode::Input;

void applyMisoMode() {
  if (g_misoMode == MisoMode::Input) {
    pinMode(PIN_MISO, INPUT);
  } else {
    pinMode(PIN_MISO, OUTPUT);
    digitalWrite(PIN_MISO, g_misoMode == MisoMode::OutputHigh ? HIGH : LOW);
  }
}

void printState() {
  Serial.print("STATE SCK=");
  Serial.print(digitalRead(PIN_SCK));
  Serial.print(" MOSI=");
  Serial.print(digitalRead(PIN_MOSI));
  Serial.print(" CS=");
  Serial.print(digitalRead(PIN_CS));
  Serial.print(" MISO_MODE=");
  switch (g_misoMode) {
    case MisoMode::Input:
      Serial.print("IN");
      break;
    case MisoMode::OutputLow:
      Serial.print("OUT0");
      break;
    case MisoMode::OutputHigh:
      Serial.print("OUT1");
      break;
  }
  Serial.print(" MISO_READ=");
  Serial.println(digitalRead(PIN_MISO));
}

void handleLine(String line) {
  line.trim();
  if (line.isEmpty()) {
    return;
  }
  String upper = line;
  upper.toUpperCase();

  if (upper == "READ" || upper == "STATE") {
    printState();
    return;
  }
  if (upper == "MISO IN") {
    g_misoMode = MisoMode::Input;
    applyMisoMode();
    printState();
    return;
  }
  if (upper == "MISO 0") {
    g_misoMode = MisoMode::OutputLow;
    applyMisoMode();
    printState();
    return;
  }
  if (upper == "MISO 1") {
    g_misoMode = MisoMode::OutputHigh;
    applyMisoMode();
    printState();
    return;
  }
  if (upper == "H" || upper == "HELP" || upper == "?") {
    printHelp();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
}

void appSetup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_CS, INPUT);
  g_misoMode = MisoMode::Input;
  applyMisoMode();

  Serial.println();
  Serial.print("SPI line-check: ");
  Serial.println(ROLE_NAME);
  Serial.print("SCK=");
  Serial.print(PIN_SCK);
  Serial.print(" MOSI=");
  Serial.print(PIN_MOSI);
  Serial.print(" MISO=");
  Serial.print(PIN_MISO);
  Serial.print(" CS=");
  Serial.println(PIN_CS);
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
}

#endif

}  // namespace

void setup() {
  appSetup();
}

void loop() {
  appLoop();
}
