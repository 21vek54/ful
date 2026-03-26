#include "config.h"
#include "system_state.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "pneumatics.h"
#include "motion_profiles.h"
#include "calibration.h"
#include "calibration_with_dialog.h"
#include "endurance_test.h"
#include "work_cycle.h"
#include "recovery.h"
#include "grease_removal.h"
#include <Wire.h>

// =============================================
// ГЛАВНЫЙ ФАЙЛ СИСТЕМЫ УПРАВЛЕНИЯ МАНИПУЛЯТОРОМ
// Версия: 7 (все вывходы из аварииного режима сделаны)
// =============================================

// Буфер для ввода команд
String inputBuffer = "";

// Флаги для управления состоянием
bool systemInitialized = false;

// I2C status bus to master
static constexpr uint32_t I2C_CLOCK_HZ = 100000;
static constexpr uint8_t I2C_DEVICE_ADDRESS = 13;
static constexpr int PIN_I2C_SDA = PIN_RS485_RX;
static constexpr int PIN_I2C_SCL = PIN_RS485_TX;
static constexpr uint8_t I2C_MANAGED_POKE_CMD = 0xA5;
static constexpr uint8_t I2C_MANAGED_TEXT_CMD = 0xA6;
static constexpr size_t I2C_MANAGED_TEXT_MAX_LEN = 64;
static constexpr uint16_t I2C_FRAME_MAGIC = 0x4659;
static constexpr uint16_t I2C_FRAME_PROTO_VER = 1;
static constexpr size_t I2C_FRAME_SIZE = 28;
static constexpr uint16_t DEVICE_KIND_MANIPULATOR = 2;
static constexpr uint16_t STATUS_READY_BIT = 1U << 0;
static constexpr uint16_t STATUS_BUSY_BIT = 1U << 1;
static constexpr uint16_t STATUS_SAFE_BIT = 1U << 2;
static constexpr uint16_t STATUS_ALARM_BIT = 1U << 3;
static constexpr uint16_t STATUS_CALIBRATED_BIT = 1U << 4;
static constexpr uint16_t STATUS_STEP7_READY_BIT = 1U << 5;
static constexpr uint16_t ERROR_DRIVER_ALARM_BIT = 1U << 0;
static constexpr uint16_t ERROR_LIMITS_CONFLICT_BIT = 1U << 1;
static constexpr uint16_t ERROR_Z_CONFLICT_BIT = 1U << 2;
static constexpr uint16_t ERROR_GRIP_CONFLICT_BIT = 1U << 3;
static constexpr uint16_t ERROR_NOT_CALIBRATED_BIT = 1U << 4;
static constexpr uint16_t MODE_CODE_IDLE = 1;
static constexpr uint16_t MODE_CODE_BUSY = 2;
static constexpr uint16_t MODE_CODE_ERROR = 3;
static constexpr uint16_t JOB_CODE_NONE = 0;
static constexpr uint16_t JOB_CODE_WORK = 1;
static constexpr uint16_t JOB_CODE_TEST = 2;
static constexpr uint16_t JOB_CODE_GREASE = 3;
static constexpr uint16_t JOB_CODE_CALIB = 4;
static constexpr uint16_t JOB_CODE_MANUAL = 5;
struct __attribute__((packed)) I2cStatusFrame {
    uint16_t magic;
    uint16_t protoVer;
    uint16_t deviceKind;
    uint16_t deviceIdEcho;
    uint16_t statusWord;
    uint16_t errorWord;
    uint16_t extra0;
    uint16_t extra1;
    uint16_t extra2;
    uint16_t extra3;
    uint32_t heartbeatMs;
    uint32_t checksum;
};

static_assert(sizeof(I2cStatusFrame) == I2C_FRAME_SIZE, "Unexpected I2cStatusFrame size");

static uint32_t g_i2cRxCount = 0;
static uint32_t g_i2cTxCount = 0;
static uint8_t g_i2cLastRxLen = 0;
static volatile bool g_i2cCommandPending = false;
static volatile uint8_t g_i2cPendingCommandLen = 0;
static char g_i2cPendingCommand[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};

// =====================================================
// ДОБАВЛЕНО: логика авто-восстановления (как команда V)
// =====================================================
static uint8_t g_autoRecoverAttempts = 0;
static unsigned long g_lastAutoRecoverAttemptMs = 0;

static bool g_autoRecoverPending = false;
static unsigned long g_autoRecoverRunAtMs = 0;
static const __FlashStringHelper* g_autoRecoverReason = F("");

void processCommand(String command);

// Проверка: можно ли сейчас запускать recoverWorkCycle()
static bool canAutoRecoverNow() {
    updateAllSensors();

    if (!hasSavedWorkCycleState()) return false;
    if (!isCalibrated) return false;

    // система не должна быть занята
    if (isMoving) return false;
    if (workCycleInProgress) return false;
    if (testInProgress) return false;
    if (greaseRemovalInProgress) return false;
    if (calibrationInProgress) return false;

    // нельзя при ALM или конфликтах датчиков
    if (stepperIsAlarm()) return false;
    if (isAnyConflict()) return false;

    return true;
}

static void scheduleAutoRecover(const __FlashStringHelper* reason, unsigned long delayMs) {
    g_autoRecoverPending = true;
    g_autoRecoverRunAtMs = millis() + delayMs;
    g_autoRecoverReason = reason;

    Serial.println();
    Serial.println(F("=== АВТОВОССТАНОВЛЕНИЕ: запланировано ==="));
    Serial.print(F("Причина: "));
    Serial.println(reason);
    Serial.print(F("Старт через, мс: "));
    Serial.println(delayMs);
}

static void tryRunAutoRecoverNow() {
    if (!g_autoRecoverPending) return;
    if ((long)(millis() - g_autoRecoverRunAtMs) < 0) return; // ещё рано

    // кулдаун + лимит попыток
    if (g_autoRecoverAttempts >= AUTO_RECOVER_MAX_ATTEMPTS) {
        Serial.println(F("Автовосстановление: лимит попыток исчерпан. Используйте V вручную."));
        g_autoRecoverPending = false;
        return;
    }
    if (millis() - g_lastAutoRecoverAttemptMs < AUTO_RECOVER_COOLDOWN_MS) {
        return; // ждём кулдаун
    }

    if (!canAutoRecoverNow()) {
        // условия пока не выполнены — оставляем pending, попробуем позже
        return;
    }

    g_autoRecoverAttempts++;
    g_lastAutoRecoverAttemptMs = millis();

    Serial.println();
    Serial.println(F("=== АВТОВОССТАНОВЛЕНИЕ (как команда V) ==="));
    Serial.print(F("Причина: "));
    Serial.println(g_autoRecoverReason);
    Serial.println(F("Запуск recoverWorkCycle()..."));

    // важный момент: дадим системе сохранить параметры (как в ручной V)
    saveSystemParameters();
    delay(50);

    // запускаем восстановление
    recoverWorkCycle();

    // после запуска считаем задачу выполненной (если восстановление прервётся — оно само сохранит шаг в EEPROM)
    g_autoRecoverPending = false;
}

// =============================================
// ФУНКЦИИ ОБРАБОТКИ КОМАНД
// =============================================

static bool isManipulatorBusy() {
    return isMoving || workCycleInProgress || testInProgress || greaseRemovalInProgress || calibrationInProgress;
}

static uint16_t getManipulatorModeCode() {
    if (stepperIsAlarm() || isAnyConflict()) return MODE_CODE_ERROR;
    if (isManipulatorBusy()) return MODE_CODE_BUSY;
    return MODE_CODE_IDLE;
}

static uint16_t getManipulatorJobCode() {
    if (workCycleInProgress) return JOB_CODE_WORK;
    if (testInProgress) return JOB_CODE_TEST;
    if (greaseRemovalInProgress) return JOB_CODE_GREASE;
    if (calibrationInProgress) return JOB_CODE_CALIB;
    if (isMoving) return JOB_CODE_MANUAL;
    return JOB_CODE_NONE;
}

static uint16_t getManipulatorStatusWord() {
    uint16_t word = 0;
    const bool safe = isSystemSafe();
    const bool alarm = stepperIsAlarm() || isAnyConflict();

    if (!alarm) {
        word |= STATUS_READY_BIT;
    }
    if (isManipulatorBusy()) {
        word |= STATUS_BUSY_BIT;
    }
    if (safe) {
        word |= STATUS_SAFE_BIT;
    }
    if (alarm) {
        word |= STATUS_ALARM_BIT;
    }
    if (isCalibrated) {
        word |= STATUS_CALIBRATED_BIT;
    }
    if (isWorkCycleStep7Reached()) {
        word |= STATUS_STEP7_READY_BIT;
    }
    return word;
}

static uint16_t getManipulatorErrorWord() {
    uint16_t word = 0;
    if (stepperIsAlarm()) word |= ERROR_DRIVER_ALARM_BIT;
    if (isBothLimitsPressed()) word |= ERROR_LIMITS_CONFLICT_BIT;
    if (isZConflict()) word |= ERROR_Z_CONFLICT_BIT;
    if (isGripConflict()) word |= ERROR_GRIP_CONFLICT_BIT;
    if (!isCalibrated) word |= ERROR_NOT_CALIBRATED_BIT;
    return word;
}

static uint16_t getManipulatorSensorBits() {
    updateAllSensors();
    uint16_t bits = 0;
    if (isLeftLimitPressed()) bits |= (1U << 0);
    if (isRightLimitPressed()) bits |= (1U << 1);
    if (isZUp()) bits |= (1U << 2);
    if (isZDown()) bits |= (1U << 3);
    if (isGripOpen()) bits |= (1U << 4);
    if (isGripClosed()) bits |= (1U << 5);
    if (stepperIsAlarm()) bits |= (1U << 6);
    return bits;
}

static uint16_t getManipulatorConflictBits() {
    updateAllSensors();
    uint16_t bits = 0;
    if (isBothLimitsPressed()) bits |= (1U << 0);
    if (isZConflict()) bits |= (1U << 1);
    if (isGripConflict()) bits |= (1U << 2);
    return bits;
}

static uint32_t checksumI2cStatusFrame(const I2cStatusFrame& frame) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&frame);
    uint32_t sum = 0x13572468UL;
    for (size_t i = 0; i < sizeof(I2cStatusFrame) - sizeof(frame.checksum); i++) {
        sum = (sum << 5) | (sum >> 27);
        sum ^= bytes[i];
    }
    return sum;
}

static void fillI2cStatusFrame(I2cStatusFrame& frame) {
    memset(&frame, 0, sizeof(frame));
    frame.magic = I2C_FRAME_MAGIC;
    frame.protoVer = I2C_FRAME_PROTO_VER;
    frame.deviceKind = DEVICE_KIND_MANIPULATOR;
    frame.deviceIdEcho = I2C_DEVICE_ADDRESS;
    frame.statusWord = getManipulatorStatusWord();
    frame.errorWord = getManipulatorErrorWord();
    frame.extra0 = getManipulatorModeCode();
    frame.extra1 = getManipulatorJobCode();
    frame.extra2 = getManipulatorSensorBits();
    frame.extra3 = (getManipulatorConflictBits() & 0x00FFU) |
                   (static_cast<uint16_t>(getWorkCycleCurrentStep()) << 8);
    frame.heartbeatMs = millis();
    frame.checksum = checksumI2cStatusFrame(frame);
}

static void onI2cReceive(int len) {
    g_i2cLastRxLen = static_cast<uint8_t>(len < 0 ? 0 : len);
    if (len <= 0) {
        return;
    }

    g_i2cRxCount++;
    const int opcode = Wire.read();
    if (opcode == I2C_MANAGED_POKE_CMD) {
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        return;
    }

    if (opcode == I2C_MANAGED_TEXT_CMD) {
        size_t count = 0;
        while (Wire.available() > 0 && count < I2C_MANAGED_TEXT_MAX_LEN) {
            g_i2cPendingCommand[count++] = static_cast<char>(Wire.read());
        }
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        g_i2cPendingCommand[count] = '\0';
        g_i2cPendingCommandLen = static_cast<uint8_t>(count);
        g_i2cCommandPending = count > 0;
        return;
    }

    while (Wire.available() > 0) {
        (void)Wire.read();
    }
}

static void onI2cRequest() {
    I2cStatusFrame frame = {};
    fillI2cStatusFrame(frame);
    Wire.write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    g_i2cTxCount++;
}

static void initI2cStatusBus() {
    Wire.begin(I2C_DEVICE_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    Wire.onReceive(onI2cReceive);
    Wire.onRequest(onI2cRequest);
}

static void processPendingI2cCommand() {
    if (!g_i2cCommandPending) {
        return;
    }

    char local[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};
    noInterrupts();
    const uint8_t len = g_i2cPendingCommandLen;
    memcpy(local, g_i2cPendingCommand, len);
    local[len] = '\0';
    g_i2cPendingCommand[0] = '\0';
    g_i2cPendingCommandLen = 0;
    g_i2cCommandPending = false;
    interrupts();

    String line(local);
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    Serial.print("I2C CMD: ");
    Serial.println(line);
    processCommand(line);
}

void handleProfileCommand() {
    Serial.println("Введите профиль скорости в формате:");
    Serial.println("{процент1;задержка1:процент2;задержка2:...}");
    Serial.println("Пример: {33.33;1000:33.33;300:33.34;800}");
    Serial.println("Ожидание ввода...");

    // Ждем ввода профиля
    while (!Serial.available()) {
        delay(10);
    }

    String profile = Serial.readStringUntil('\n');
    profile.trim();

    if (profile.length() > 0) {
        Serial.print("Выполнение профиля: ");
        Serial.println(profile);

        // Заменяем запятые на точки для парсинга
        profile.replace(',', '.');

        // Определяем направление
        bool direction;
        if (isLeftLimitPressed()) {
            direction = LOW;  // Вправо
            Serial.println("Направление: вправо");
        } else if (isRightLimitPressed()) {
            direction = HIGH; // Влево
            Serial.println("Направление: влево");
        } else {
            direction = LOW;  // По умолчанию вправо
            Serial.println("Направление: вправо (по умолчанию)");
        }

        // Выполняем движение по профилю (без требования концевика)
        executeSpeedProfile(profile, travelDistance, direction, false);
    }
}

void handleTestCommand() {
    Serial.println("Введите количество циклов для теста (1-1000):");

    // Ждем ввода количества циклов
    unsigned long timeout = millis();
    while (!Serial.available() && (millis() - timeout) < 10000) {
        delay(10);
    }

    if (!Serial.available()) {
        Serial.println("Таймаут ввода. Используйте: T 10 (например)");
        return;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();
    int cycles = input.toInt();

    if (cycles > 0 && cycles <= MAX_TEST_CYCLES) {
        startEnduranceTest(cycles);
    } else {
        Serial.print("Ошибка: количество циклов должно быть от 1 до ");
        Serial.println(MAX_TEST_CYCLES);
    }
}

void processCommand(String command) {
    command.trim();
    if (command.length() == 0) return;

    String upperCommand = command;
    upperCommand.toUpperCase();

    char cmd = command.charAt(0);

    // Проверяем, не выполняется ли рабочий цикл или тест
    if ((workCycleInProgress || testInProgress || greaseRemovalInProgress) &&
        cmd != 'Z' && cmd != 'z' && cmd != 'S' && cmd != 's' && cmd != 'X' && cmd != 'x') {
        Serial.println("Система занята: выполняется автоматический процесс");
        return;
    }

    // Обработка команд пневматики (строчные и прописные)
    if (pneumoHandleCommand(cmd)) {
        return;
    }

    // Приводим команду к верхнему регистру для остальных команд
    cmd = toupper(cmd);

    switch (cmd) {
        case 'R': // Рабочий цикл
            if (!isMoving && isCalibrated && !testInProgress && !greaseRemovalInProgress) {
                Serial.println("Запуск рабочего цикла...");
                // Перед началом убеждаемся, что предыдущие данные сохранены
                saveSystemParameters();
                delay(50); // Даем время для сохранения
                startWorkCycle();
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving) {
                Serial.println("Ошибка: система уже движется");
            } else if (testInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: выполняется тест отказоустойчивости или процедура удаления смазки");
            }
            break;

        case 'V': // Восстановление цикла
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Проверка возможности восстановления...");
                // Сохраняем текущие параметры перед восстановлением
                saveSystemParameters();
                delay(50); // Даем время для сохранения
                recoverWorkCycle();
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'D': // Движение вправо
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Движение вправо");
                // Сохраняем состояние перед движением
                saveSystemParameters();
                delay(50);
                if (moveRightSafe()) {
                    Serial.println("приехал вправо");
                } else {
                    Serial.println("Ошибка движения вправо");
                }
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'A': // Движение влево
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Движение влево");
                // Сохраняем состояние перед движением
                saveSystemParameters();
                delay(50);
                if (moveLeftSafe()) {
                    Serial.println("приехал влево");
                } else {
                    Serial.println("Ошибка движения влево");
                }
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'P': // Движение по произвольному профилю
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Команда: движение по произвольному профилю");
                // Сохраняем состояние перед движением
                saveSystemParameters();
                delay(50);
                handleProfileCommand();
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'K': // Калибровка с диалогом
            if (!isMoving && !calibrationInProgress && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                // Сохраняем текущее состояние перед калибровкой
                saveSystemParameters();
                delay(50);
                if (upperCommand == "KWEB" || upperCommand == "K!") {
                    calibrateDistanceWithExternalConfirmation();
                } else {
                    calibrateDistanceWithDialog();
                }
            } else if (calibrationInProgress) {
                Serial.println("Ошибка: калибровка уже выполняется");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'T': // Тест отказоустойчивости
            if (!isMoving && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                // Сохраняем состояние перед тестом
                saveSystemParameters();
                delay(50);
                handleTestCommand();
            } else if (testInProgress) {
                Serial.println("Ошибка: тест уже выполняется");
            } else if (isMoving || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'U': // Процедура удаления смазки - ДОБАВЛЕНА ЭТА КОМАНДА
            if (!isMoving && !greaseRemovalInProgress && !testInProgress && !workCycleInProgress) {
                Serial.println("Процедура удаления смазки");
                // Сохраняем состояние перед процедурой
                saveSystemParameters();
                delay(50);
                startGreaseRemoval();
            } else if (greaseRemovalInProgress) {
                Serial.println("Ошибка: процедура удаления смазки уже выполняется");
            } else if (isMoving || testInProgress || workCycleInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'Z': // Экстренная остановка
            Serial.println("Экстренная остановка");
            // Сохраняем состояние перед остановкой
            saveSystemParameters();
            delay(50);
            emergencyStop();
            stopEnduranceTest();
            stopGreaseRemoval(); // ДОБАВЛЕНА ЭТА СТРОКА
            break;

        case 'X': // Аварийный сброс драйвера
            if (!isMoving && !workCycleInProgress && !testInProgress && !greaseRemovalInProgress) {
                Serial.println("Аварийный сброс драйвера через реле...");
                stepperHardReset();
            } else if (isMoving || workCycleInProgress || testInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'G': // Статус датчиков
            printAllSensorsStatus();
            break;

        case 'I': // Информация о системе (статус + EEPROM)
            printSystemInfo();
            break;

        case '0': // Сброс системы
            Serial.println("Сброс состояния системы...");
            resetSystemState();
            break;

        case 'C': // Очистка EEPROM
            Serial.println("Очистка EEPROM...");
            clearSystemState();
            break;

        case '?': // Помощь
            printHelp();
            break;

        default:
            Serial.print("Неизвестная команда: ");
            Serial.println(cmd);
            printHelp();
            break;
    }
}

// =============================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// =============================================

void printWelcomeMessage() {
    Serial.println("\n" + String(50, '='));
    Serial.println("СИСТЕМА УПРАВЛЕНИЯ МАНИПУЛЯТОРОМ v6.1");
    Serial.println("Добавлено: процедура удаления смазки (U)");
    Serial.println("Добавлено: аварийный сброс драйвера (X)");
    Serial.println("ESP32 + DM556 + Пневматика + Реле сброса");
    Serial.println(String(50, '='));
}

void printHelp() {
    Serial.println("\n" + String(40, '='));
    Serial.println("СПРАВКА ПО КОМАНДАМ (регистр не важен)");
    Serial.println(String(40, '='));

    Serial.println("\nИНФОРМАЦИЯ И ДИАГНОСТИКА:");
    Serial.println("  ?        - эта справка");
    Serial.println("  G/g      - статус всех датчиков (герконов)");
    Serial.println("  I/i      - информация о системе + EEPROM");

    Serial.println("\nАВТОМАТИЧЕСКИЕ ПРОЦЕССЫ:");
    Serial.println("  R/r      - рабочий цикл манипулятор");
    Serial.println("  V/v      - восстановление после сбоя (автоматическое)");
    Serial.println("  K/k      - калибровка расстояния (с диалогом)");
    Serial.println("  KWEB     - калибровка для web/I2C (без Y/N в Serial)");
    Serial.println("  T/t      - тест отказоустойчивости");
    Serial.println("  U/u      - процедура удаления смазки (после обслуживания)");
    Serial.println("  P/p      - движение по произвольному профилю");

    Serial.println("\nРУЧНОЕ УПРАВЛЕНИЕ:");
    Serial.println("  W/w      - вертикаль ВВЕРХ");
    Serial.println("  S/s      - вертикаль ВНИЗ");
    Serial.println("  A/a      - движение ВЛЕВО");
    Serial.println("  D/d      - движение ВПРАВО");
    Serial.println("  Q/q      - захват ОТКРЫТЬ");
    Serial.println("  E/e      - захват ЗАКРЫТЬ");

    Serial.println("\nУПРАВЛЕНИЕ СИСТЕМОЙ:");
    Serial.println("  0        - сброс состояния системы");
    Serial.println("  C/c      - очистка EEPROM (полный сброс)");
    Serial.println("  Z/z      - экстренная остановка");
    Serial.println("  X/x      - аварийный сброс драйвера (через реле)");
    Serial.println("  I2C status - addr 13, SDA=GPIO16, SCL=GPIO17");

    Serial.println(String(40, '='));
    Serial.println("Готов к работе. Введите команду...");
}

void handleSerialCommands() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        // Если это символ новой строки - обрабатываем команду
        if (incomingChar == '\n' || incomingChar == '\r') {
            processCommand(inputBuffer);
            inputBuffer = "";
        } else {
            // Добавляем символ в буфер
            inputBuffer += incomingChar;
        }
    }
}

// =============================================
// ОСНОВНЫЕ ФУНКЦИИ ARDUINO
// =============================================

void setup() {
    // Инициализация Serial порта
    Serial.begin(115200);
    delay(1000);

    initI2cStatusBus();

    printWelcomeMessage();
    Serial.println("I2C status ready: addr 13, SDA=GPIO16, SCL=GPIO17");

    // Инициализация всех модулей
    Serial.println("\nИнициализация системы...");

    // Сначала инициализируем восстановление
    recoveryInit();
    Serial.println("✓ Система восстановления инициализирована");

    initAllSensors();
    Serial.println("✓ Датчики инициализированы");

    stepperInit();
    Serial.println("✓ Шаговый двигатель инициализирован с аппаратным сбросом");

    pneumoInit();
    Serial.println("✓ Пневматика инициализирована");

    initSystemState();
    Serial.println("✓ Состояние системы инициализировано");

    // Первоначальное обновление датчиков
    updateAllSensors();
    delay(100);
    updateAllSensors();

    // Проверка аварийных состояний
    Serial.println("\nПроверка системы...");

    bool errors = false;

    if (stepperIsAlarm()) {
        Serial.println("⚠️  ВНИМАНИЕ: Драйвер в аварийном состоянии (ALM)");
        Serial.println("  Выполняем автоматический сброс...");
        if (stepperRecoverFromAlarm()) {
            Serial.println("✓ Драйвер успешно восстановлен");
        } else {
            Serial.println("✗ Не удалось восстановить драйвер автоматически");
            Serial.println("  Используйте команду X для аппаратного сброса");
            errors = true;
        }
    } else {
        Serial.println("✓ Драйвер: норма");
    }

    if (isBothLimitsPressed()) {
        Serial.println("⚠️  ВНИМАНИЕ: Оба концевика нажаты одновременно!");
        errors = true;
    } else {
        Serial.println("✓ Концевики: норма");
    }

    if (isGripConflict()) {
        Serial.println("⚠️  ВНИМАНИЕ: Конфликт датчиков захвата!");
        errors = true;
    }

    if (isZConflict()) {
        Serial.println("⚠️  ВНИМАНИЕ: Конфликт датчиков вертикали!");
        errors = true;
    }

    // Определение начального положения
    Serial.print("\nНачальное положение: ");
    if (isLeftLimitPressed()) {
        Serial.println("левый концевик");
    } else if (isRightLimitPressed()) {
        Serial.println("правый концевик");
    } else {
        Serial.println("в середине пути");
    }

    // Проверка восстановления
    Serial.println("\n=== ПРОВЕРКА ВОССТАНОВЛЕНИЯ ===");
    if (hasSavedWorkCycleState()) {
        Serial.println("⚠️  Обнаружено сохраненное состояние рабочего цикла!");

#if AUTO_RECOVER_ON_BOOT
        // На старте — подождём, чтобы всё стабилизировалось
        scheduleAutoRecover(F("старт контроллера"), AUTO_RECOVER_ON_BOOT_DELAY_MS);
#else
        Serial.println("Для восстановления введите команду V");
#endif

    } else {
        Serial.println("✓ Сохраненного состояния нет, готов к новому циклу");
    }

    // Проверка готовности к рабочему циклу
    Serial.print("\nГотовность к рабочему циклу: ");
    if (checkWorkCycleInitialState()) {
        Serial.println("✓ ГОТОВ");
    } else {
        Serial.println("✗ ТРЕБУЕТСЯ НАСТРОЙКА");
    }

    // Проверка калибровки
    if (isCalibrated) {
        Serial.print("✓ Калибровка выполнена: ");
        Serial.print(travelDistance);
        Serial.print(" шагов (");
        Serial.print(travelDistanceMM, 1);
        Serial.println(" мм)");
    } else {
        Serial.println("⚠️  Калибровка не выполнена. Используйте команду K");
    }

    // Рекомендации
    if (errors) {
        Serial.println("\n⚠️  Обнаружены ошибки! Исправьте их перед использованием.");
    } else {
        Serial.println("\n✓ Система готова к работе");
    }

    Serial.println("\n" + String(50, '='));
    Serial.println("Основные команды:");
    Serial.println("R - рабочий цикл, V - восстановление, I - информация");
    Serial.println("K - калибровка, U - удаление смазки, X - сброс драйвера, ? - справка");
    Serial.println(String(50, '='));

    systemInitialized = true;
}

void loop() {
    // Проверка инициализации
    if (!systemInitialized) {
        return;
    }

    // Обновление состояния всех датчиков (каждые 10мс)
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate >= 10) {
        updateAllSensors();
        lastSensorUpdate = millis();
    }

    // Автоматическая проверка аварии драйвера (каждые 5 секунд)
    static unsigned long lastAlarmCheck = 0;
    static bool wasAlarm = false;

    if (millis() - lastAlarmCheck >= 5000) {
        bool alarmNow = stepperIsAlarm();

        if (alarmNow) {
            wasAlarm = true;

            Serial.println("\n⚠️  ВНИМАНИЕ: Драйвер в аварийном состоянии!");
            Serial.println("Попытка автоматического восстановления...");

            bool recovered = stepperRecoverFromAlarm();
            if (recovered) {
                Serial.println("✓ Драйвер успешно восстановлен");

#if AUTO_RECOVER_AFTER_ALM
                // Сбрасываем счётчик попыток, чтобы после ALM разрешить авто-V
                g_autoRecoverAttempts = 0;
                g_lastAutoRecoverAttemptMs = 0;

                // Планируем авто-V через 1 секунду (как ты и хотел)
                scheduleAutoRecover(F("выход из ALM (успешное восстановление)"), AUTO_RECOVER_AFTER_ALM_DELAY_MS);
#endif

            } else {
                Serial.println("✗ Не удалось восстановить драйвер автоматически");
                Serial.println("Используйте команду X для аппаратного сброса");
            }
        } else {
            if (wasAlarm) {
                // был ALM, а теперь нет (вариант, если ALM ушёл сам)
                wasAlarm = false;

#if AUTO_RECOVER_AFTER_ALM
                g_autoRecoverAttempts = 0;
                g_lastAutoRecoverAttemptMs = 0;
                scheduleAutoRecover(F("выход из ALM (снялось без recover)"), AUTO_RECOVER_AFTER_ALM_DELAY_MS);
#endif
            }
        }

        lastAlarmCheck = millis();
    }

#if AUTO_RECOVER_IN_LOOP
    // если есть запланированное восстановление — попробуем выполнить
    tryRunAutoRecoverNow();

    // если не запланировано, но EEPROM содержит шаг — можно планировать “мягко”
    // (например, если питание моргнуло и появился сохранённый шаг)
    if (!g_autoRecoverPending && hasSavedWorkCycleState()) {
        // Не спамим — планируем без лишнего вывода/часто не планируем
        scheduleAutoRecover(F("loop: найден сохранённый шаг"), 200); // небольшой стартовый лаг
    }
#endif

    // Обновление теста отказоустойчивости (если выполняется)
    updateEnduranceTest();

    // Обновление процедуры удаления смазки (если выполняется)
    updateGreaseRemoval();
    processPendingI2cCommand();

    // Обработка команд с Serial порта
    handleSerialCommands();

    // Небольшая задержка для стабильности
    delay(1);
}
