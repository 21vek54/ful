#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <Wire.h>

#include "pins.h"
#include "program1.h"
#include "shift_control.h"

constexpr char FW_VERSION[] = "v1.17";
#if defined(DEVICE_ROLE_CONVEYOR)
constexpr char DEVICE_NAME[] = "Управление конвейерами";
#else
constexpr char DEVICE_NAME[] = "Conveyor controller";
#endif

constexpr bool FLAG_UP_LEVEL = HIGH;
constexpr bool FLAG_DOWN_LEVEL = LOW;
constexpr bool PULSE_ACTIVE_LEVEL = HIGH;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint8_t I2C_DEVICE_ADDRESS = 12;
constexpr int PIN_I2C_SDA = PIN_RS485_RX;
constexpr int PIN_I2C_SCL = PIN_RS485_TX;
constexpr uint8_t I2C_MANAGED_POKE_CMD = 0xA5;
constexpr uint8_t I2C_MANAGED_TEXT_CMD = 0xA6;
constexpr size_t I2C_MANAGED_TEXT_MAX_LEN = 64;
constexpr uint16_t I2C_FRAME_MAGIC = 0x4659;
constexpr uint16_t I2C_FRAME_PROTO_VER = 1;
constexpr size_t I2C_FRAME_SIZE = 28;

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

constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
constexpr uint32_t CONTINUOUS_STEP_DELAY_US = 1000;
constexpr uint32_t POS_RUN_DELAY_US = 1000;
constexpr uint32_t STEP2_RUN_DELAY_US = 1000;
constexpr uint32_t STEP2_DEFAULT_STEPS = 5000;
constexpr uint32_t STEP2_DIVERT_STEPS = 920;
constexpr uint32_t SENSOR_PRINT_INTERVAL_MS = 500;
constexpr uint32_t SENSOR_DEBOUNCE_MS = 80;
constexpr uint32_t MANUAL_MOVE_DELAY_US = 1500; // Фиксированная задержка для команд A/D
constexpr bool RELAY_ACTIVE_LEVEL = LOW;        // Типовой модуль реле для ESP32
constexpr bool RELAY_INACTIVE_LEVEL = HIGH;
constexpr uint32_t VFD_RELAY_TEST_DURATION_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t VFD_TICK_DEFAULT_DURATION_MS = 2190U;
constexpr uint32_t VFD_TICK_MIN_DURATION_MS = 100U;
constexpr uint32_t VFD_TICK_MAX_DURATION_MS = 600000U;
constexpr bool SEAL_START_ACTIVE_LEVEL = RELAY_ACTIVE_LEVEL;
constexpr bool SEAL_START_INACTIVE_LEVEL = RELAY_INACTIVE_LEVEL;
constexpr bool SEAL_DONE_ACTIVE_LEVEL = LOW; // GPIO34 без внутренней подтяжки, нужна внешняя
constexpr uint32_t SEAL_START_PULSE_MS_DEFAULT = 300U;
constexpr uint32_t SEAL_START_PULSE_MS_MIN = 50U;
constexpr uint32_t SEAL_START_PULSE_MS_MAX = 5000U;
constexpr uint8_t OTVOD_CYCLE_DEFAULT_TOTAL = 3U;
constexpr uint8_t OTVOD_CYCLE_MIN_TOTAL = 1U;
constexpr uint8_t OTVOD_CYCLE_MAX_TOTAL = 9U;
constexpr uint32_t OTVOD_CYCLE_STEP_PHASE_TIMEOUT_MS = 5000U;
constexpr uint32_t OTVOD_CYCLE_VFD_START_TIMEOUT_MS = 2000U;
constexpr uint32_t OTVOD_CYCLE_VFD_TIMEOUT_MARGIN_MS = 4000U;
constexpr uint32_t OTVOD_CYCLE_VFD_TIMEOUT_FALLBACK_MS = 15000U;
constexpr uint32_t OTVOD_CYCLE_STEP_TO_MAIN_SETTLE_MS = 250U;
constexpr uint32_t OTVOD_CYCLE_MAIN_TO_STEP_SETTLE_MS = 250U;
constexpr uint16_t DEVICE_KIND_CONVEYOR = 1;
constexpr uint16_t STATUS_READY_BIT = 1U << 0;
constexpr uint16_t STATUS_BUSY_BIT = 1U << 1;
constexpr uint16_t STATUS_SAFE_BIT = 1U << 2;
constexpr uint16_t STATUS_ALARM_BIT = 1U << 3;
constexpr uint16_t STATUS_CALIBRATED_BIT = 1U << 4;
constexpr uint16_t STATUS_PROGRAM1_ACTIVE_BIT = 1U << 5;
constexpr uint16_t STATUS_BATCH_READY_BIT = 1U << 6;
constexpr uint16_t STATUS_STEP2_ACTIVE_BIT = 1U << 7;
constexpr uint16_t ERROR_SENSOR_CONFLICT_BIT = 1U << 0;
constexpr uint16_t ERROR_NOT_CALIBRATED_BIT = 1U << 1;
constexpr uint16_t FLAG_STATE_UNKNOWN = 0;
constexpr uint16_t FLAG_STATE_DOWN = 1;
constexpr uint16_t FLAG_STATE_UP = 2;
constexpr uint16_t MOTION_STATE_IDLE = 0;
constexpr uint16_t MOTION_STATE_RUNNING = 1;
constexpr uint16_t MOTION_STATE_POSITIONING = 2;
constexpr uint32_t MOTION_START_DELAY_MULT_NUM = 2;
constexpr uint32_t MOTION_START_DELAY_MULT_DEN = 1;
constexpr uint32_t MOTION_RAMP_STEPS_DEFAULT = 150;
constexpr uint32_t PULSES_PER_MM = 5;           // 2000 импульсов = 400 мм => 5 имп/мм
constexpr uint32_t C3_COMMAND_DISTANCE_MM = 184U;
constexpr uint32_t C3_COMMAND_STEPS = C3_COMMAND_DISTANCE_MM * PULSES_PER_MM;
constexpr uint32_t C3_COMMAND_DELAY_US = 1200U;

constexpr uint32_t C2_MOVE_DELAY_US = 1500;
constexpr uint32_t C2_CENTER_STEPS = 0U * PULSES_PER_MM; // 0 мм, центрирование отключено
constexpr uint32_t C2_FLAG_REOPEN_STEPS = 80U * PULSES_PER_MM; // 80 мм
constexpr uint32_t C2_PLATE_DIAMETER_MM = 150U;
constexpr uint32_t C2_TARGET_GAP_MM = 34U;
constexpr uint32_t C2_FORMULA_BASE_MM = 150U; // Первая часть формулы 2
constexpr uint32_t C2_FORMULA_GAP_MM = 34U;   // Вторая часть формулы 2
constexpr uint32_t C2_RELEASE_TARGET_STEPS =
    (C2_FORMULA_BASE_MM + C2_FORMULA_GAP_MM) * PULSES_PER_MM; // 150 + 34 мм
constexpr uint32_t C2_FINAL_AFTER_SECOND_LEAVE_STEPS = 100U * PULSES_PER_MM; // 100 мм

struct MotionState {
    bool active = false;
    bool pulseHigh = false;
    bool continuous = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t nominalDelayUs = 1000;
    uint32_t currentDelayUs = 1000;
    uint32_t startDelayUs = 1000;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct PosMotionState {
    bool active = false;
    bool pulseHigh = false;
    bool continuous = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t delayUs = POS_RUN_DELAY_US;
    uint32_t currentDelayUs = POS_RUN_DELAY_US;
    uint32_t startDelayUs = POS_RUN_DELAY_US;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct Step2MotionState {
    bool active = false;
    bool pulseHigh = false;
    bool continuous = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t delayUs = STEP2_RUN_DELAY_US;
    uint32_t lastPulseStartUs = 0;
};

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
};

struct TimedRelayRunState {
    bool active = false;
    uint32_t startedMs = 0;
    uint32_t durationMs = 0;
};

struct SealIoState {
    bool startPulseActive = false;
    bool startOutputActive = false;
    bool doneLastActive = false;
    uint32_t doneLastRiseMs = 0;
    uint32_t startPulseStartedMs = 0;
    uint32_t startPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;
};

enum class C2State : uint8_t {
    Idle,
    SeekFirstPlate,
    CenterFirstPlate,
    TrackFirstLeaveAndOpen,
    SeekSecondPlate,
    CenterSecondPlate,
    WaitSecondReleaseTiming,
    MoveAfterSecondRelease,
    WaitSecondLeave,
    FinalMove
};

enum class OtvodCyclePhase : uint8_t {
    Idle = 0,
    StepRunning,
    WaitMainStart,
    MainRunning,
    WaitNextStep,
    Completed,
    Aborted
};

struct OtvodCycleState {
    bool active = false;
    bool readyForBatch = true;
    bool stepRunning = false;
    bool stepObservedActive = false;
    bool vfdRunning = false;
    bool vfdObservedActive = false;
    uint8_t totalCycles = OTVOD_CYCLE_DEFAULT_TOTAL;
    uint8_t stepRunsStarted = 0;
    uint8_t stepRunsCompleted = 0;
    uint8_t stepCompletionSeqBase = 0;
    uint8_t vfdRunsStarted = 0;
    uint8_t vfdRunsCompleted = 0;
    uint32_t stepCommandSteps = STEP2_DIVERT_STEPS;
    uint32_t stepStartedMs = 0;
    uint32_t stepCompletedMs = 0;
    uint32_t stepTriggerDelayMs = OTVOD_CYCLE_STEP_TO_MAIN_SETTLE_MS;
    uint32_t vfdStartedMs = 0;
    uint32_t vfdCompletedMs = 0;
    uint32_t vfdDoneDelayMs = 0;
    uint32_t vfdTriggerDelayMs = OTVOD_CYCLE_MAIN_TO_STEP_SETTLE_MS;
    OtvodCyclePhase phase = OtvodCyclePhase::Idle;
    String lastEvent = "idle";
};

String g_cmdBuffer;
Preferences g_preferences;
Preferences g_settingsPreferences;
MotionState g_motion;
PosMotionState g_posMotion;
Step2MotionState g_step2Motion;
uint8_t g_step2CompletedSeq = 0;
SensorFilterState g_sensorFilter;
bool g_sensorStreamEnabled = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_totalStepsCounter = 0;
bool g_program1BufferReady = false;
bool g_program1StorageReady = false;
bool g_settingsStorageReady = false;
TimedRelayRunState g_vfdRelayRun;
uint32_t g_vfdTickDurationMs = VFD_TICK_DEFAULT_DURATION_MS;
OtvodCycleState g_otvodCycle;
SealIoState g_sealIo;

bool g_c2Active = false;
C2State g_c2State = C2State::Idle;
bool g_c2FirstLeaveCaptured = false;
bool g_c2SecondLeaveCaptured = false;
uint32_t g_c2CenterStartStep = 0;
uint32_t g_c2CenterStepsActive = C2_CENTER_STEPS;
uint32_t g_c2FirstFlagDownStep = 0;
uint32_t g_c2FirstLeaveStep = 0;
uint32_t g_c2FirstUTSteps = 0;
uint32_t g_c2SecondFlagDownStep = 0;
uint32_t g_c2SecondLeaveStep = 0;
uint32_t g_c2FinalStartStep = 0;
uint32_t g_c2FinalMoveSteps = 0;
uint32_t g_c2CycleStartStep = 0;
uint32_t g_c2CycleStartMs = 0;
uint32_t g_c2SecondReleaseSteps = 0;
uint32_t g_i2cRxCount = 0;
uint32_t g_i2cTxCount = 0;
uint8_t g_i2cLastRxLen = 0;
volatile bool g_i2cCommandPending = false;
volatile uint8_t g_i2cPendingCommandLen = 0;
char g_i2cPendingCommand[I2C_MANAGED_TEXT_MAX_LEN + 1] = {};

bool isPlateAtSensorFiltered();
void stopMotion();
void startCycle2();
void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs);
void processPositionalMotion();
void processStep2Motion();
void stopPositionalMotion();
void stopStep2Motion();
void startStep2Motion(uint32_t steps, uint32_t delayUs);
void updateSensorFilter();
void initProgram1Storage();
void setProgram1BufferReady(bool ready);
void initSettingsStorage();
void initI2cStatusBus();
void processPendingI2cCommand();
void processTimedRelayRun();
void processOtvodCycle();
bool isConveyorBusy();
void processSealIo();

void writeRelayOutput(bool enabled)
{
    digitalWrite(PIN_RELAY_1, enabled ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
}

void sealWriteStartOutput(bool active)
{
    digitalWrite(PIN_RELAY_2, active ? SEAL_START_ACTIVE_LEVEL : SEAL_START_INACTIVE_LEVEL);
    g_sealIo.startOutputActive = active;
}

bool sealIsDoneActive()
{
    return digitalRead(PIN_SENSOR_EXT_1) == SEAL_DONE_ACTIVE_LEVEL;
}

void stopTimedRelayRun(const char *reason)
{
    if (!g_vfdRelayRun.active) {
        return;
    }

    writeRelayOutput(false);
    Serial.print("VFD: реле GPIO21 выключено");
    if (reason != nullptr && reason[0] != '\0') {
        Serial.print(" (");
        Serial.print(reason);
        Serial.print(")");
    }
    Serial.println(".");
    g_vfdRelayRun.active = false;
    g_vfdRelayRun.durationMs = 0;
    g_vfdRelayRun.startedMs = 0;
}

void startTimedRelayRun(uint32_t durationMs)
{
    if (g_vfdRelayRun.active) {
        stopTimedRelayRun("перезапуск");
    }

    writeRelayOutput(true);
    g_vfdRelayRun.active = true;
    g_vfdRelayRun.startedMs = millis();
    g_vfdRelayRun.durationMs = durationMs;

    Serial.print("VFD: реле GPIO21 включено на ");
    Serial.print(durationMs / 1000U);
    Serial.println(" сек.");
}

void printHelp()
{
    Serial.println("Команды конвейера:");
    Serial.println("  Основной конвейер:");
    Serial.println("    D 200      - ручной ход вправо на 200 мм, задержка 1500 мкс");
    Serial.println("    MAINSTART  - непрерывные импульсы на PUL13, задержка 1000 мкс");
    Serial.println("    MAINSTOP   - остановить PUL13");
    Serial.println("    1          - автоцикл: 2 -> C+(3+Z) -> 2 -> C+(3+Z) -> 2 -> C+Z");
    Serial.println("    2          - рабочий ход: формула 150+34-UT, добег 100 мм");
    Serial.println("    3          - позиционный ход вправо на 184 мм (PUL32), профиль, 1200 мкс");
    Serial.println("    P 300      - позиционный ход вправо на 300 мм (PUL32), профиль, 1000 мкс");
    Serial.println("    POSSTART   - непрерывные импульсы на PUL32, задержка 1000 мкс");
    Serial.println("    POSSTOP    - остановить PUL32");
    Serial.println("  Сдвиг тарелки:");
    Serial.println("    C          - сдвиг к C: 500 шагов, DIR14/PUL23, задержка 500 мкс");
    Serial.println("    Z          - возврат к Z: 500 шагов, DIR14/PUL23, задержка 300 мкс");
    Serial.println("    CZ         - калибровка хода по герконам Z=GPIO33 и C=GPIO25");
    Serial.println("  Отводной 2-ручейковый конвейер:");
    Serial.println("    T2 [steps] - STEP2 на GPIO19, по умолчанию 5000 шагов, задержка 1000 мкс");
    Serial.println("    OTVOD [steps] - STEP2 на GPIO19, по умолчанию 920 шагов");
    Serial.println("    STEP2START - непрерывные импульсы на GPIO19, задержка 1000 мкс");
    Serial.println("    STEP2STOP  - остановить GPIO19");
    Serial.println("    STOP2      - остановить STEP2");
    Serial.println("    OTCYCLE START [cycles] [steps] - локальный цикл отвода: OTVOD -> основной, по умолчанию 3x920");
    Serial.println("    OTCYCLE STATUS               - показать состояние локального цикла отвода");
    Serial.println("    OTCYCLE STOP                 - остановить локальный цикл отвода");
    Serial.println("  Реле / частотник:");
    Serial.println("    R1ON       - включить реле 1 (GPIO21)");
    Serial.println("    R1OFF      - выключить реле 1");
    Serial.println("    VFD5MIN    - включить реле 1 на 5 минут");
    Serial.println("    VFDSTOP    - выключить реле частотника");
    Serial.println("    VFDTICK SHOW      - показать время прокрутки за 1 такт");
    Serial.println("    VFDTICK RUN       - выполнить 1 такт по сохраненному времени");
    Serial.println("    VFDTICK TEST 2.19 - проверить основной конвейер на 2.19 сек");
    Serial.println("    VFDTICK SAVE 2.19 - записать время прокрутки за 1 такт");
    Serial.println("  Запайщик:");
    Serial.println("    SEAL START [ms] - импульс старта на GPIO22, по умолчанию 300 мс");
    Serial.println("    SEAL STATUS     - показать состояние START/DONE");
    Serial.println("    SEAL OUT ON     - вручную включить старт запайщика");
    Serial.println("    SEAL OUT OFF    - вручную выключить старт запайщика");
    Serial.println("  Флаг и датчики:");
    Serial.println("    W          - флаг вверх");
    Serial.println("    S          - флаг вниз");
    Serial.println("    E          - вкл/выкл поток датчиков: E18 + герконы Z/C, каждые 0.5 сек");
    Serial.println("  Сервис:");
    Serial.println("    H          - помощь");
    Serial.println("    I2C status - addr 12, SDA=GPIO16, SCL=GPIO17");
    Serial.println("Команды не чувствительны к регистру.");
}

bool parseUnsigned(const String &s, uint32_t &value)
{
    if (s.isEmpty()) {
        return false;
    }

    uint32_t acc = 0;
    for (size_t i = 0; i < s.length(); i++) {
        const char c = s[i];
        if (c < '0' || c > '9') {
            return false;
        }
        const uint32_t digit = static_cast<uint32_t>(c - '0');
        if (acc > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        acc = (acc * 10U) + digit;
    }
    value = acc;
    return true;
}

bool parseSecondsToDurationMs(const String &source, uint32_t &durationMs)
{
    String normalized = source;
    normalized.trim();
    normalized.replace(',', '.');
    if (normalized.isEmpty()) {
        return false;
    }

    bool hasDigit = false;
    bool hasDot = false;
    for (size_t i = 0; i < normalized.length(); i++) {
        const char c = normalized[i];
        if (c >= '0' && c <= '9') {
            hasDigit = true;
            continue;
        }
        if (c == '.' && !hasDot) {
            hasDot = true;
            continue;
        }
        return false;
    }

    if (!hasDigit) {
        return false;
    }

    const float seconds = normalized.toFloat();
    if (!(seconds > 0.0f)) {
        return false;
    }

    const float durationMsFloat = seconds * 1000.0f;
    if (!(durationMsFloat >= static_cast<float>(VFD_TICK_MIN_DURATION_MS)) ||
        durationMsFloat > static_cast<float>(VFD_TICK_MAX_DURATION_MS)) {
        return false;
    }

    durationMs = static_cast<uint32_t>(lroundf(durationMsFloat));
    return true;
}

void updateDebouncedSensorFilter(
    SensorFilterState &state,
    bool rawState,
    uint32_t debounceMs)
{
    const uint32_t nowMs = millis();

    if (!state.initialized) {
        state.initialized = true;
        state.lastRawPlateDetected = rawState;
        state.stablePlateDetected = rawState;
        state.lastRawChangeMs = nowMs;
        return;
    }

    if (rawState != state.lastRawPlateDetected) {
        state.lastRawPlateDetected = rawState;
        state.lastRawChangeMs = nowMs;
    }

    if (rawState != state.stablePlateDetected &&
        (uint32_t)(nowMs - state.lastRawChangeMs) >= debounceMs) {
        state.stablePlateDetected = rawState;
    }
}

bool parseDistanceMmArgs(String args, const String &cmd, uint32_t &distanceMm)
{
    args.trim();
    if (args.isEmpty()) {
        return false;
    }

    // Поддержка вариантов вида "A 200" и "A A 200" / "D D 200"
    const int splitPos = args.indexOf(' ');
    if (splitPos > 0) {
        String first = args.substring(0, splitPos);
        first.trim();
        first.toUpperCase();
        if (first == cmd) {
            args = args.substring(splitPos + 1);
            args.trim();
        }
    }

    return parseUnsigned(args, distanceMm);
}

uint32_t interpolateDelayUs(uint32_t progressSteps, uint32_t rampSteps, uint32_t startDelayUs, uint32_t nominalDelayUs)
{
    if (rampSteps == 0 || progressSteps >= rampSteps || startDelayUs <= nominalDelayUs) {
        return nominalDelayUs;
    }

    const uint32_t delta = startDelayUs - nominalDelayUs;
    const uint32_t dec = static_cast<uint32_t>((static_cast<uint64_t>(delta) * progressSteps) / rampSteps);
    return startDelayUs - dec;
}

void updateMotionProfileDelay()
{
    if (!g_motion.active) {
        return;
    }

    const uint32_t rampSteps = g_motion.rampSteps == 0 ? 1 : g_motion.rampSteps;

    uint32_t accelProgress = g_motion.stepsDone;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    uint32_t targetDelayUs = interpolateDelayUs(
        accelProgress, rampSteps, g_motion.startDelayUs, g_motion.nominalDelayUs);

    if (g_motion.stepsTotal != UINT32_MAX) {
        uint32_t remainingSteps = 0;
        if (g_motion.stepsTotal > g_motion.stepsDone) {
            remainingSteps = g_motion.stepsTotal - g_motion.stepsDone;
        }

        uint32_t decelProgress = remainingSteps;
        if (decelProgress > rampSteps) {
            decelProgress = rampSteps;
        }

        const uint32_t decelDelayUs = interpolateDelayUs(
            decelProgress, rampSteps, g_motion.startDelayUs, g_motion.nominalDelayUs);
        if (decelDelayUs > targetDelayUs) {
            targetDelayUs = decelDelayUs;
        }
    }

    if (targetDelayUs < g_motion.nominalDelayUs) {
        targetDelayUs = g_motion.nominalDelayUs;
    }
    g_motion.currentDelayUs = targetDelayUs;
}

void startMotionProfiled(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (nominalDelayUs == 0) {
        nominalDelayUs = 1;
    }

    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.stepsTotal = stepsTotal;
    g_motion.stepsDone = 0;
    g_motion.nominalDelayUs = nominalDelayUs;
    g_motion.currentDelayUs = nominalDelayUs;

    uint32_t startDelayUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(nominalDelayUs) * MOTION_START_DELAY_MULT_NUM) / MOTION_START_DELAY_MULT_DEN);
    if (startDelayUs <= nominalDelayUs) {
        startDelayUs = nominalDelayUs + 1;
    }
    g_motion.startDelayUs = startDelayUs;

    g_motion.rampSteps = MOTION_RAMP_STEPS_DEFAULT;
    if (stepsTotal != UINT32_MAX) {
        uint32_t halfSteps = stepsTotal / 2U;
        if (halfSteps < g_motion.rampSteps) {
            g_motion.rampSteps = halfSteps;
        }
        if (g_motion.rampSteps == 0) {
            g_motion.rampSteps = 1;
        }
    }

    g_motion.lastPulseStartUs = micros();
    updateMotionProfileDelay();
}

void armMotionStopAfterSteps(uint32_t stepsToStop)
{
    if (!g_motion.active) {
        return;
    }

    if (stepsToStop == 0) {
        stopMotion();
        return;
    }

    if (g_motion.stepsDone > (UINT32_MAX - stepsToStop)) {
        g_motion.stepsTotal = UINT32_MAX;
    } else {
        g_motion.stepsTotal = g_motion.stepsDone + stepsToStop;
    }

    g_motion.rampSteps = MOTION_RAMP_STEPS_DEFAULT;
    uint32_t halfSteps = stepsToStop / 2U;
    if (halfSteps < g_motion.rampSteps) {
        g_motion.rampSteps = halfSteps;
    }
    if (g_motion.rampSteps == 0) {
        g_motion.rampSteps = 1;
    }

    updateMotionProfileDelay();
}

void writePulseInactive()
{
    digitalWrite(PIN_STEP_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void writePosPulseInactive()
{
    digitalWrite(PIN_POS_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void writeStep2PulseInactive()
{
    digitalWrite(PIN_STEP2_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}


void initProgram1Storage()
{
    g_program1StorageReady = g_preferences.begin("p1buf", false);
    if (!g_program1StorageReady) {
        Serial.println("P1: EEPROM storage unavailable, buffer flag reset.");
        g_program1BufferReady = false;
        return;
    }

    g_program1BufferReady = g_preferences.getBool("ready", false);
}

void setProgram1BufferReady(bool ready)
{
    g_program1BufferReady = ready;
    if (!g_program1StorageReady) {
        return;
    }
    g_preferences.putBool("ready", ready);
}

void initSettingsStorage()
{
    g_settingsStorageReady = g_settingsPreferences.begin("convcfg", false);
    if (!g_settingsStorageReady) {
        Serial.println("CFG: storage unavailable, время прокрутки = 2.19 сек по умолчанию.");
        g_vfdTickDurationMs = VFD_TICK_DEFAULT_DURATION_MS;
        return;
    }

    uint32_t savedDurationMs = g_settingsPreferences.getUInt("vfdTickMs", VFD_TICK_DEFAULT_DURATION_MS);
    if (savedDurationMs < VFD_TICK_MIN_DURATION_MS || savedDurationMs > VFD_TICK_MAX_DURATION_MS) {
        savedDurationMs = VFD_TICK_DEFAULT_DURATION_MS;
    }

    g_vfdTickDurationMs = savedDurationMs;
}

void printVfdTickSetting()
{
    Serial.print("Прокрутка основного конвейера за 1 такт: ");
    Serial.print(static_cast<float>(g_vfdTickDurationMs) / 1000.0f, 2);
    Serial.println(" сек.");
}

void saveVfdTickSetting()
{
    if (!g_settingsStorageReady) {
        Serial.println("VFDTICK: запись недоступна, storage не открыт.");
        return;
    }

    g_settingsPreferences.putUInt("vfdTickMs", g_vfdTickDurationMs);
    Serial.print("VFDTICK: сохранено ");
    Serial.print(static_cast<float>(g_vfdTickDurationMs) / 1000.0f, 2);
    Serial.println(" сек.");
}

void printSealStatus()
{
    Serial.print("SEAL: start_out=");
    Serial.print(g_sealIo.startOutputActive ? "on" : "off");
    Serial.print(", done=");
    Serial.print(sealIsDoneActive() ? "active" : "inactive");
    Serial.print(", start_pin=");
    Serial.print(PIN_RELAY_2);
    Serial.print(", done_pin=");
    Serial.print(PIN_SENSOR_EXT_1);
    Serial.print(", pulse_ms=");
    Serial.println(g_sealIo.startPulseDurationMs);
}

const char *otvodCyclePhaseName(OtvodCyclePhase phase)
{
    switch (phase) {
        case OtvodCyclePhase::StepRunning:
            return "step_running";
        case OtvodCyclePhase::WaitMainStart:
            return "wait_main_start";
        case OtvodCyclePhase::MainRunning:
            return "main_running";
        case OtvodCyclePhase::WaitNextStep:
            return "wait_next_step";
        case OtvodCyclePhase::Completed:
            return "completed";
        case OtvodCyclePhase::Aborted:
            return "aborted";
        case OtvodCyclePhase::Idle:
        default:
            return "idle";
    }
}

void printOtvodCycleStatus()
{
    Serial.print("OTCYCLE: active=");
    Serial.print(g_otvodCycle.active ? "yes" : "no");
    Serial.print(", phase=");
    Serial.print(otvodCyclePhaseName(g_otvodCycle.phase));
    Serial.print(", ready=");
    Serial.print(g_otvodCycle.readyForBatch ? "yes" : "no");
    Serial.print(", step=");
    Serial.print(g_otvodCycle.stepRunsCompleted);
    Serial.print("/");
    Serial.print(g_otvodCycle.stepRunsStarted);
    Serial.print("/");
    Serial.print(g_otvodCycle.totalCycles);
    Serial.print(", main=");
    Serial.print(g_otvodCycle.vfdRunsCompleted);
    Serial.print("/");
    Serial.print(g_otvodCycle.vfdRunsStarted);
    Serial.print(", steps=");
    Serial.print(g_otvodCycle.stepCommandSteps);
    Serial.print(", tick_ms=");
    Serial.print(g_otvodCycle.vfdDoneDelayMs > 0 ? g_otvodCycle.vfdDoneDelayMs : g_vfdTickDurationMs);
    Serial.print(", last=");
    Serial.println(g_otvodCycle.lastEvent);
}

void stopOtvodCycleOutputs()
{
    if (g_step2Motion.active) {
        stopStep2Motion();
    }
    if (g_vfdRelayRun.active) {
        stopTimedRelayRun("остановка OTCYCLE");
    } else {
        writeRelayOutput(false);
    }
}

void finishOtvodCycle(const String &result)
{
    g_otvodCycle.active = false;
    g_otvodCycle.stepRunning = false;
    g_otvodCycle.stepObservedActive = false;
    g_otvodCycle.vfdRunning = false;
    g_otvodCycle.vfdObservedActive = false;
    g_otvodCycle.readyForBatch = true;
    g_otvodCycle.phase = OtvodCyclePhase::Completed;
    g_otvodCycle.lastEvent = result;
    Serial.println(result);
}

void abortOtvodCycle(const String &reason, bool stopOutputs)
{
    if (stopOutputs) {
        stopOtvodCycleOutputs();
    }

    g_otvodCycle.active = false;
    g_otvodCycle.stepRunning = false;
    g_otvodCycle.stepObservedActive = false;
    g_otvodCycle.vfdRunning = false;
    g_otvodCycle.vfdObservedActive = false;
    g_otvodCycle.readyForBatch = false;
    g_otvodCycle.phase = OtvodCyclePhase::Aborted;
    g_otvodCycle.lastEvent = reason;
    Serial.println(reason);
}

bool startOtvodCycleStep(uint8_t cycleIndex, uint32_t nowMs)
{
    if (g_step2Motion.active || g_vfdRelayRun.active) {
        Serial.println("OTCYCLE: нельзя запустить двухручейковый отвод, механика занята.");
        return false;
    }

    startStep2Motion(g_otvodCycle.stepCommandSteps, STEP2_RUN_DELAY_US);
    if (!g_step2Motion.active) {
        Serial.println("OTCYCLE: запуск OTVOD не подтвердился.");
        return false;
    }

    g_otvodCycle.stepRunning = true;
    g_otvodCycle.stepObservedActive = false;
    g_otvodCycle.stepRunsStarted = cycleIndex;
    g_otvodCycle.stepCompletionSeqBase = g_step2CompletedSeq;
    g_otvodCycle.stepStartedMs = nowMs;
    g_otvodCycle.stepCompletedMs = 0;
    g_otvodCycle.phase = OtvodCyclePhase::StepRunning;
    g_otvodCycle.readyForBatch = false;
    g_otvodCycle.lastEvent = "OTCYCLE: двухручейковый старт " + String(cycleIndex) + "/" + String(g_otvodCycle.totalCycles);
    Serial.println(g_otvodCycle.lastEvent);
    return true;
}

bool startOtvodCycleMainTick(uint8_t cycleIndex, uint32_t nowMs)
{
    if (g_vfdTickDurationMs == 0U) {
        Serial.println("OTCYCLE: время 1 такта не настроено.");
        return false;
    }
    if (g_vfdRelayRun.active || g_step2Motion.active) {
        Serial.println("OTCYCLE: нельзя запустить основной отвод, механика занята.");
        return false;
    }

    startTimedRelayRun(g_vfdTickDurationMs);
    if (!g_vfdRelayRun.active) {
        Serial.println("OTCYCLE: запуск основного отвода не подтвердился.");
        return false;
    }

    g_otvodCycle.vfdRunning = true;
    g_otvodCycle.vfdObservedActive = true;
    g_otvodCycle.vfdRunsStarted = cycleIndex;
    g_otvodCycle.vfdStartedMs = nowMs;
    g_otvodCycle.vfdCompletedMs = 0;
    g_otvodCycle.vfdDoneDelayMs = g_vfdTickDurationMs;
    g_otvodCycle.phase = OtvodCyclePhase::MainRunning;
    g_otvodCycle.lastEvent = "OTCYCLE: основной отвод старт " + String(cycleIndex) + "/" + String(g_otvodCycle.totalCycles);
    Serial.println(g_otvodCycle.lastEvent);
    return true;
}

bool startOtvodCycle(uint8_t totalCycles, uint32_t stepSteps)
{
    if (g_otvodCycle.active) {
        Serial.println("OTCYCLE: уже выполняется.");
        return false;
    }

    if (totalCycles < OTVOD_CYCLE_MIN_TOTAL || totalCycles > OTVOD_CYCLE_MAX_TOTAL || stepSteps == 0U) {
        Serial.println("OTCYCLE: неверные параметры запуска.");
        return false;
    }

    if (isConveyorBusy()) {
        Serial.println("OTCYCLE: конвейер занят, сначала остановите текущий процесс.");
        return false;
    }

    g_otvodCycle = OtvodCycleState{};
    g_otvodCycle.active = true;
    g_otvodCycle.readyForBatch = false;
    g_otvodCycle.totalCycles = totalCycles;
    g_otvodCycle.stepCommandSteps = stepSteps;
    g_otvodCycle.vfdDoneDelayMs = g_vfdTickDurationMs;
    g_otvodCycle.lastEvent = "OTCYCLE init";

    Serial.print("OTCYCLE: init step=");
    Serial.print(stepSteps);
    Serial.print(", tick=");
    Serial.print(g_vfdTickDurationMs);
    Serial.print(" ms, cycles=");
    Serial.println(totalCycles);

    const uint32_t nowMs = millis();
    if (!startOtvodCycleStep(1U, nowMs)) {
        g_otvodCycle = OtvodCycleState{};
        Serial.println("OTCYCLE: старт первого прохода не удался.");
        return false;
    }

    Serial.print("OTCYCLE: запущено циклов=");
    Serial.print(totalCycles);
    Serial.print(", OTVOD=");
    Serial.print(stepSteps);
    Serial.println(", основной=1 такт.");
    return true;
}

bool program1IsShiftCalibrated()
{
    return shiftIsCalibrated();
}

bool program1IsSystemBusy()
{
    return g_motion.active || g_c2Active || g_posMotion.active;
}

bool program1IsCycle2Active()
{
    return g_c2Active;
}

bool program1IsManualMotionActive()
{
    return g_motion.active;
}

bool program1IsPositionalMotionActive()
{
    return g_posMotion.active;
}

bool program1GetBufferReady()
{
    return g_program1BufferReady;
}

void program1ConsumeBuffer()
{
    setProgram1BufferReady(false);
}

void program1MarkBufferReady()
{
    setProgram1BufferReady(true);
}

void program1StartCycle2()
{
    startCycle2();
}

void program1StartPositionalPass()
{
    startPositionalProfiledMotion(C3_COMMAND_STEPS, C3_COMMAND_DELAY_US);
}

bool program1RunShiftStageC()
{
    return shiftRunProgramStageC();
}

bool program1RunShiftStageZ()
{
    return shiftRunProgramStageZ();
}

void shiftHookUpdateMainSensorFilter()
{
    updateSensorFilter();
}

void shiftHookProcessPositionalMotion()
{
    processPositionalMotion();
}

bool shiftHookIsMotionActive()
{
    return g_motion.active;
}

bool shiftHookIsCycle2Active()
{
    return g_c2Active;
}

bool shiftHookIsPositionalMotionActive()
{
    return g_posMotion.active;
}

void stopPositionalMotion()
{
    writePosPulseInactive();
    g_posMotion.active = false;
    g_posMotion.pulseHigh = false;
    g_posMotion.continuous = false;
    g_posMotion.stepsTotal = 0;
    g_posMotion.stepsDone = 0;
    Serial.println("P: EVA25 stopped.");
}

void updatePositionalMotionProfileDelay()
{
    if (!g_posMotion.active) {
        return;
    }

    const uint32_t rampSteps = g_posMotion.rampSteps == 0 ? 1 : g_posMotion.rampSteps;

    uint32_t accelProgress = g_posMotion.stepsDone;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    uint32_t targetDelayUs = interpolateDelayUs(
        accelProgress, rampSteps, g_posMotion.startDelayUs, g_posMotion.delayUs);

    uint32_t remainingSteps = 0;
    if (g_posMotion.stepsTotal > g_posMotion.stepsDone) {
        remainingSteps = g_posMotion.stepsTotal - g_posMotion.stepsDone;
    }

    uint32_t decelProgress = remainingSteps;
    if (decelProgress > rampSteps) {
        decelProgress = rampSteps;
    }

    const uint32_t decelDelayUs = interpolateDelayUs(
        decelProgress, rampSteps, g_posMotion.startDelayUs, g_posMotion.delayUs);
    if (decelDelayUs > targetDelayUs) {
        targetDelayUs = decelDelayUs;
    }

    if (targetDelayUs < g_posMotion.delayUs) {
        targetDelayUs = g_posMotion.delayUs;
    }
    g_posMotion.currentDelayUs = targetDelayUs;
}

void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (g_posMotion.active) {
        Serial.println("POS: позиционный мотор уже в движении.");
        return;
    }

    if (stepsTotal == 0U) {
        Serial.println("POS: шагов 0, движение не требуется.");
        return;
    }

    if (nominalDelayUs == 0U) {
        nominalDelayUs = 1U;
    }

    writePosPulseInactive();

    g_posMotion.active = true;
    g_posMotion.pulseHigh = false;
    g_posMotion.continuous = false;
    g_posMotion.stepsTotal = stepsTotal;
    g_posMotion.stepsDone = 0;
    g_posMotion.delayUs = nominalDelayUs;
    g_posMotion.currentDelayUs = nominalDelayUs;

    uint32_t startDelayUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(nominalDelayUs) * MOTION_START_DELAY_MULT_NUM) / MOTION_START_DELAY_MULT_DEN);
    if (startDelayUs <= nominalDelayUs) {
        startDelayUs = nominalDelayUs + 1U;
    }
    g_posMotion.startDelayUs = startDelayUs;

    g_posMotion.rampSteps = MOTION_RAMP_STEPS_DEFAULT;
    uint32_t halfSteps = stepsTotal / 2U;
    if (halfSteps < g_posMotion.rampSteps) {
        g_posMotion.rampSteps = halfSteps;
    }
    if (g_posMotion.rampSteps == 0U) {
        g_posMotion.rampSteps = 1U;
    }

    g_posMotion.lastPulseStartUs = micros();
    updatePositionalMotionProfileDelay();

    Serial.print("POS: профилированный ход, шагов=");
    Serial.print(stepsTotal);
    Serial.print(", расстояние=");
    Serial.print(static_cast<float>(stepsTotal) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.print(" мм");
    Serial.print(", полка=");
    Serial.print(nominalDelayUs);
    Serial.println(" мкс.");
}

void processPositionalMotion()
{
    if (!g_posMotion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_posMotion.pulseHigh) {
        if ((uint32_t)(nowUs - g_posMotion.lastPulseStartUs) >= g_posMotion.currentDelayUs) {
            digitalWrite(PIN_POS_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_posMotion.pulseHigh = true;
            g_posMotion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_posMotion.lastPulseStartUs) >= STEP_PULSE_WIDTH_US) {
        writePosPulseInactive();
        g_posMotion.pulseHigh = false;
        g_posMotion.stepsDone++;
        if (g_posMotion.continuous) {
            return;
        }
        if (g_posMotion.stepsDone >= g_posMotion.stepsTotal) {
            stopPositionalMotion();
            Serial.println("POS: движение завершено.");
        } else {
            updatePositionalMotionProfileDelay();
        }
    }
}

void stopMotion()
{
    writePulseInactive();
    g_motion.active = false;
    g_motion.pulseHigh = false;
    g_motion.continuous = false;
}

void startConstantMotion(uint32_t steps, uint32_t delayUs)
{
    if (g_motion.active) {
        Serial.println("Ошибка: двигатель уже в движении.");
        return;
    }

    if (steps == 0) {
        Serial.println("Шагов 0: движение не требуется.");
        return;
    }

    startMotionProfiled(steps, delayUs);

    Serial.print("Запуск: шагов=");
    Serial.print(steps);
    Serial.print(", задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс, направление=вправо");
}

void startMainContinuousMotion(uint32_t delayUs)
{
    if (g_motion.active) {
        Serial.println("MAIN: мотор PUL13 уже в движении.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    writePulseInactive();
    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.continuous = true;
    g_motion.stepsTotal = 0;
    g_motion.stepsDone = 0;
    g_motion.nominalDelayUs = delayUs;
    g_motion.currentDelayUs = delayUs;
    g_motion.startDelayUs = delayUs;
    g_motion.rampSteps = 0;
    g_motion.lastPulseStartUs = micros();

    Serial.print("MAIN: непрерывный запуск PUL13, задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void startPositionalContinuousMotion(uint32_t delayUs)
{
    if (g_posMotion.active) {
        Serial.println("POS: мотор PUL32 уже в движении.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    writePosPulseInactive();
    g_posMotion.active = true;
    g_posMotion.pulseHigh = false;
    g_posMotion.continuous = true;
    g_posMotion.stepsTotal = 0;
    g_posMotion.stepsDone = 0;
    g_posMotion.delayUs = delayUs;
    g_posMotion.currentDelayUs = delayUs;
    g_posMotion.startDelayUs = delayUs;
    g_posMotion.rampSteps = 0;
    g_posMotion.lastPulseStartUs = micros();

    Serial.print("POS: непрерывный запуск PUL32, задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void printC2Prefix()
{
    Serial.print("C2: ");
    Serial.print(g_totalStepsCounter - g_c2CycleStartStep);
    Serial.print(" шагов ");
}

void startCycle2Internal()
{
    if (g_motion.active || g_c2Active) {
        Serial.println("C2: ошибка, двигатель уже в движении.");
        return;
    }

    const bool plateAlreadyOnSensor = g_sensorFilter.stablePlateDetected;

    digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
    startMotionProfiled(UINT32_MAX, C2_MOVE_DELAY_US);

    g_c2Active = true;
    g_c2State = plateAlreadyOnSensor ? C2State::CenterFirstPlate : C2State::SeekFirstPlate;
    g_c2FirstLeaveCaptured = false;
    g_c2SecondLeaveCaptured = false;
    g_c2CenterStartStep = g_totalStepsCounter;
    g_c2CenterStepsActive = C2_CENTER_STEPS;
    g_c2FirstFlagDownStep = g_totalStepsCounter;
    g_c2FirstLeaveStep = g_totalStepsCounter;
    g_c2FirstUTSteps = 0;
    g_c2SecondFlagDownStep = g_totalStepsCounter;
    g_c2SecondLeaveStep = g_totalStepsCounter;
    g_c2FinalStartStep = g_totalStepsCounter;
    g_c2FinalMoveSteps = 0;
    g_c2CycleStartStep = g_totalStepsCounter;
    g_c2CycleStartMs = millis();
    g_c2SecondReleaseSteps = 0;

    printC2Prefix();
    Serial.println("старт. Цель: 1-я тарелка в упоре, 2-я с зазором 34 мм.");
    if (plateAlreadyOnSensor) {
        printC2Prefix();
        if (g_c2CenterStepsActive == 0U) {
            Serial.println("тарелка уже на датчике, старт без центрирования (флаг поднят).");
        } else {
            Serial.println("тарелка уже на датчике, выполняем стартовое центрирование +20 мм (флаг поднят).");
        }
    }
}

void stopStep2Motion()
{
    writeStep2PulseInactive();
    g_step2Motion.active = false;
    g_step2Motion.pulseHigh = false;
    g_step2Motion.continuous = false;
}

void startStep2Motion(uint32_t steps, uint32_t delayUs)
{
    if (g_step2Motion.active) {
        Serial.println("STEP2: тест уже выполняется.");
        return;
    }

    if (steps == 0) {
        Serial.println("STEP2: шагов 0, запуск не требуется.");
        return;
    }

    if (delayUs == 0) {
        delayUs = 1;
    }

    g_step2Motion.active = true;
    g_step2Motion.pulseHigh = false;
    g_step2Motion.continuous = false;
    g_step2Motion.stepsTotal = steps;
    g_step2Motion.stepsDone = 0;
    g_step2Motion.delayUs = delayUs;
    g_step2Motion.lastPulseStartUs = micros();

    Serial.print("STEP2: запуск, GPIO19, шагов=");
    Serial.print(steps);
    Serial.print(", задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void startStep2ContinuousMotion(uint32_t delayUs)
{
    if (g_step2Motion.active) {
        Serial.println("STEP2: GPIO19 уже в движении.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    writeStep2PulseInactive();
    g_step2Motion.active = true;
    g_step2Motion.pulseHigh = false;
    g_step2Motion.continuous = true;
    g_step2Motion.stepsTotal = 0;
    g_step2Motion.stepsDone = 0;
    g_step2Motion.delayUs = delayUs;
    g_step2Motion.lastPulseStartUs = micros();

    Serial.print("STEP2: непрерывный запуск GPIO19, задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void processStep2Motion()
{
    if (!g_step2Motion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_step2Motion.pulseHigh) {
        if ((uint32_t)(nowUs - g_step2Motion.lastPulseStartUs) >= g_step2Motion.delayUs) {
            digitalWrite(PIN_STEP2_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_step2Motion.pulseHigh = true;
            g_step2Motion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_step2Motion.lastPulseStartUs) >= STEP_PULSE_WIDTH_US) {
        writeStep2PulseInactive();
        g_step2Motion.pulseHigh = false;
        g_step2Motion.stepsDone++;
        if (g_step2Motion.continuous) {
            return;
        }
        if (g_step2Motion.stepsDone >= g_step2Motion.stepsTotal) {
            g_step2CompletedSeq = static_cast<uint8_t>(g_step2CompletedSeq + 1U);
            stopStep2Motion();
            Serial.println("STEP2: движение завершено.");
        }
    }
}

void startCycle2()
{
    startCycle2Internal();
}

void captureC2FirstLeaveIfNeeded()
{
    if (g_c2FirstLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c2FirstLeaveCaptured = true;
    g_c2FirstLeaveStep = g_totalStepsCounter;
    g_c2FirstUTSteps = g_totalStepsCounter - g_c2FirstFlagDownStep;

    printC2Prefix();
    Serial.print("1-я тарелка ушла с датчика, UT=");
    Serial.print(g_c2FirstUTSteps);
    Serial.print(" шагов (");
    Serial.print(static_cast<float>(g_c2FirstUTSteps) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.println(" мм).");

    uint32_t releaseWindowSteps = 0;
    if (g_c2FirstUTSteps < C2_RELEASE_TARGET_STEPS) {
        releaseWindowSteps = C2_RELEASE_TARGET_STEPS - g_c2FirstUTSteps;
    }
    g_c2SecondReleaseSteps = releaseWindowSteps;
}

void captureC2SecondLeaveIfNeeded()
{
    if (g_c2SecondLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c2SecondLeaveCaptured = true;
    g_c2SecondLeaveStep = g_totalStepsCounter;
    printC2Prefix();
    Serial.println("2-я тарелка ушла с датчика.");
}

void scheduleC2FinalMove()
{
    g_c2FinalMoveSteps = C2_FINAL_AFTER_SECOND_LEAVE_STEPS;
    g_c2FinalStartStep = g_totalStepsCounter;
    g_c2State = C2State::FinalMove;
    armMotionStopAfterSteps(g_c2FinalMoveSteps);

    printC2Prefix();
    Serial.print("финальный добег ");
    Serial.print(static_cast<float>(g_c2FinalMoveSteps) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.println(" мм (минимум для 2-й тарелки).");
}

void processCycle2()
{
    if (!g_c2Active || g_c2State == C2State::Idle) {
        return;
    }

    if (!g_motion.active) {
        if (g_c2State == C2State::FinalMove) {
            g_c2Active = false;
            g_c2State = C2State::Idle;
            const uint32_t cycleDurationMs = millis() - g_c2CycleStartMs;
            printC2Prefix();
            Serial.print("Движение завершено. время цикла ");
            Serial.print(cycleDurationMs);
            Serial.print(" мс (");
            Serial.print(static_cast<float>(cycleDurationMs) / 1000.0F, 1);
            Serial.println(" с).");
            return;
        }

        g_c2Active = false;
        g_c2State = C2State::Idle;
        printC2Prefix();
        Serial.println("сценарий прерван (двигатель остановлен вне C2).");
        return;
    }

    switch (g_c2State) {
        case C2State::SeekFirstPlate:
            if (isPlateAtSensorFiltered()) {
                g_c2CenterStartStep = g_totalStepsCounter;
                g_c2State = C2State::CenterFirstPlate;
                printC2Prefix();
                if (g_c2CenterStepsActive == 0U) {
                    Serial.println("1-я тарелка найдена, центрирование пропущено.");
                } else {
                    Serial.println("1-я тарелка найдена, центрирование +20 мм.");
                }
            }
            return;

        case C2State::CenterFirstPlate:
            if ((uint32_t)(g_totalStepsCounter - g_c2CenterStartStep) >= g_c2CenterStepsActive) {
                digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                g_c2FirstFlagDownStep = g_totalStepsCounter;
                g_c2FirstLeaveCaptured = false;
                g_c2FirstUTSteps = 0;
                g_c2State = C2State::TrackFirstLeaveAndOpen;
                printC2Prefix();
                Serial.println("флаг опущен для 1-й тарелки.");
            }
            return;

        case C2State::TrackFirstLeaveAndOpen:
            captureC2FirstLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c2FirstFlagDownStep) >= C2_FLAG_REOPEN_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
                g_c2State = C2State::SeekSecondPlate;
                printC2Prefix();
                Serial.println("флаг поднят, поиск 2-й тарелки.");
            }
            return;

        case C2State::SeekSecondPlate:
            captureC2FirstLeaveIfNeeded();

            if (isPlateAtSensorFiltered()) {
                g_c2CenterStartStep = g_totalStepsCounter;
                g_c2State = C2State::CenterSecondPlate;
                printC2Prefix();
                if (g_c2CenterStepsActive == 0U) {
                    Serial.println("2-я тарелка найдена, центрирование пропущено.");
                } else {
                    Serial.println("2-я тарелка найдена, центрирование +20 мм.");
                }
            }
            return;

        case C2State::CenterSecondPlate:
            captureC2FirstLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c2CenterStartStep) >= g_c2CenterStepsActive) {
                g_c2State = C2State::WaitSecondReleaseTiming;
                printC2Prefix();
                if (g_c2CenterStepsActive == 0U) {
                    Serial.println("2-я тарелка без центрирования, расчет момента отпускания.");
                } else {
                    Serial.println("2-я тарелка центрирована, расчет момента отпускания.");
                }
            }
            return;

        case C2State::WaitSecondReleaseTiming:
            captureC2FirstLeaveIfNeeded();

            if (!g_c2FirstLeaveCaptured) {
                return;
            }

            {
                const uint32_t releaseAfterFirstLeaveSteps = g_c2SecondReleaseSteps;

                if ((uint32_t)(g_totalStepsCounter - g_c2FirstLeaveStep) >= releaseAfterFirstLeaveSteps) {
                    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                    g_c2SecondFlagDownStep = g_totalStepsCounter;
                    g_c2SecondLeaveCaptured = false;
                    g_c2State = C2State::MoveAfterSecondRelease;
                    printC2Prefix();
                    Serial.println("флаг опущен для 2-й тарелки.");
                }
            }
            return;

        case C2State::MoveAfterSecondRelease:
            captureC2SecondLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c2SecondFlagDownStep) >= C2_FLAG_REOPEN_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);

                if (g_c2SecondLeaveCaptured) {
                    scheduleC2FinalMove();
                } else {
                    g_c2State = C2State::WaitSecondLeave;
                    printC2Prefix();
                    Serial.println("ждем уход 2-й тарелки с датчика.");
                }
            }
            return;

        case C2State::WaitSecondLeave:
            captureC2SecondLeaveIfNeeded();
            if (g_c2SecondLeaveCaptured) {
                scheduleC2FinalMove();
            }
            return;

        case C2State::FinalMove:
            return;

        case C2State::Idle:
        default:
            return;
    }
}

void processMotion()
{
    if (!g_motion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_motion.pulseHigh) {
        if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= g_motion.currentDelayUs) {
            digitalWrite(PIN_STEP_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_motion.pulseHigh = true;
            g_motion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= STEP_PULSE_WIDTH_US) {
        writePulseInactive();
        g_motion.pulseHigh = false;
        g_motion.stepsDone++;
        g_totalStepsCounter++;

        if (g_motion.continuous) {
            return;
        }

        if (g_motion.stepsDone >= g_motion.stepsTotal) {
            stopMotion();
            if (!g_c2Active) {
                Serial.println("Движение завершено.");
            }
        } else {
            updateMotionProfileDelay();
        }
    }
}

bool readSensorPlateRaw()
{
    return digitalRead(PIN_SENSOR) == HIGH;
}

void updateSensorFilter()
{
    updateDebouncedSensorFilter(g_sensorFilter, readSensorPlateRaw(), SENSOR_DEBOUNCE_MS);
}

bool isPlateAtSensorFiltered()
{
    return g_sensorFilter.stablePlateDetected;
}

void printSensorState()
{
    const bool isPlateAtSensor = isPlateAtSensorFiltered();
    Serial.print("Датчик E18-D50NK: ");
    Serial.println(isPlateAtSensor ? "тарелка на флаге (HIGH)" : "тарелки нет (LOW)");

    Serial.print("Геркон Z GPIO33: ");
    Serial.println(shiftIsSensorZTriggered() ? "замкнут (LOW)" : "разомкнут (HIGH)");
    Serial.print("Геркон C GPIO25: ");
    Serial.println(shiftIsSensorCTriggered() ? "замкнут (LOW)" : "разомкнут (HIGH)");
}

void processSensorStream()
{
    if (!g_sensorStreamEnabled) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_lastSensorPrintMs) >= SENSOR_PRINT_INTERVAL_MS) {
        g_lastSensorPrintMs = nowMs;
        printSensorState();
    }
}

void splitCommandLine(const String &line, String &cmd, String &args)
{
    int splitPos = line.indexOf(' ');
    if (splitPos < 0) {
        splitPos = line.indexOf('\t');
    }

    cmd = line;
    args = "";
    if (splitPos > 0) {
        cmd = line.substring(0, splitPos);
        args = line.substring(splitPos + 1);
        args.trim();
    }
    cmd.toUpperCase();
}

bool parseMoveDistanceSteps(const String &args, const String &cmd, const char *example, uint32_t &steps)
{
    uint32_t distanceMm = 0;
    if (!parseDistanceMmArgs(args, cmd, distanceMm)) {
        Serial.print("Format error. Example: ");
        Serial.println(example);
        return false;
    }
    if (distanceMm > (UINT32_MAX / PULSES_PER_MM)) {
        Serial.println("Error: distance is too large.");
        return false;
    }

    steps = distanceMm * PULSES_PER_MM;
    return true;
}

bool tryHandleDistanceMoveCommand(const String &cmd, const String &args)
{
    uint32_t steps = 0;

    if (cmd == "D") {
        if (!parseMoveDistanceSteps(args, cmd, "D 200", steps)) {
            return true;
        }
        startConstantMotion(steps, MANUAL_MOVE_DELAY_US);
        return true;
    }

    if (cmd == "P") {
        if (!parseMoveDistanceSteps(args, cmd, "P 300", steps)) {
            return true;
        }
        startPositionalProfiledMotion(steps, POS_RUN_DELAY_US);
        return true;
    }

    return false;
}

bool tryHandleProgramCommand(const String &cmd)
{
    if (cmd == "1") {
        startProgram1();
        return true;
    }

    if (cmd == "2") {
        startCycle2();
        return true;
    }

    if (cmd == "3") {
        startPositionalProfiledMotion(C3_COMMAND_STEPS, C3_COMMAND_DELAY_US);
        return true;
    }

    return false;
}

bool tryHandleShiftCommand(const String &cmd)
{
    if (cmd == "CZ") {
        shiftCalibrateTravel();
        return true;
    }

    if (cmd == "C") {
        shiftRunMoveCommandC();
        return true;
    }

    if (cmd == "Z") {
        shiftRunMoveCommandZ();
        return true;
    }

    return false;
}

bool tryHandleServiceCommand(const String &cmd, const String &args)
{
    if (cmd == "SEAL") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (subCmd.isEmpty() || subCmd == "HELP" || subCmd == "H") {
            Serial.println("SEAL commands:");
            Serial.println("  SEAL START [ms]");
            Serial.println("  SEAL STATUS");
            Serial.println("  SEAL OUT ON");
            Serial.println("  SEAL OUT OFF");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "STATE") {
            printSealStatus();
            return true;
        }

        if (subCmd == "OUT") {
            String mode;
            String unused;
            splitCommandLine(subArgs, mode, unused);
            if (mode == "ON") {
                g_sealIo.startPulseActive = false;
                sealWriteStartOutput(true);
                Serial.println("SEAL OUT ON");
                return true;
            }
            if (mode == "OFF") {
                g_sealIo.startPulseActive = false;
                sealWriteStartOutput(false);
                Serial.println("SEAL OUT OFF");
                return true;
            }
            Serial.println("Usage: SEAL OUT <ON|OFF>");
            return true;
        }

        if (subCmd == "START" || subCmd == "RUN") {
            uint32_t pulseMs = SEAL_START_PULSE_MS_DEFAULT;
            if (!subArgs.isEmpty()) {
                if (!parseUnsigned(subArgs, pulseMs) ||
                    pulseMs < SEAL_START_PULSE_MS_MIN ||
                    pulseMs > SEAL_START_PULSE_MS_MAX) {
                    Serial.println("Usage: SEAL START [50..5000]");
                    return true;
                }
            }

            if (g_sealIo.startPulseActive) {
                Serial.println("SEAL START ignored: pulse already active.");
                return true;
            }

            g_sealIo.startPulseDurationMs = pulseMs;
            g_sealIo.startPulseStartedMs = millis();
            g_sealIo.startPulseActive = true;
            sealWriteStartOutput(true);

            Serial.print("SEAL START: pulse on GPIO");
            Serial.print(PIN_RELAY_2);
            Serial.print(" for ");
            Serial.print(pulseMs);
            Serial.println(" ms.");
            return true;
        }

        Serial.println("Unknown SEAL subcommand. Use: SEAL HELP");
        return true;
    }

    if (cmd == "OTCYCLE") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (subCmd.isEmpty() || subCmd == "HELP" || subCmd == "H") {
            Serial.println("OTCYCLE commands:");
            Serial.println("  OTCYCLE START [cycles] [steps]");
            Serial.println("  OTCYCLE STATUS");
            Serial.println("  OTCYCLE STOP");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "STATE") {
            printOtvodCycleStatus();
            return true;
        }

        if (subCmd == "STOP") {
            if (!g_otvodCycle.active) {
                Serial.println("OTCYCLE: не выполняется.");
                return true;
            }
            abortOtvodCycle("OTCYCLE: остановлено пользователем.", true);
            return true;
        }

        if (subCmd == "START" || subCmd == "RUN") {
            uint32_t cyclesRaw = OTVOD_CYCLE_DEFAULT_TOTAL;
            uint32_t stepsRaw = STEP2_DIVERT_STEPS;

            String cyclesTok;
            String stepsTok;
            splitCommandLine(subArgs, cyclesTok, stepsTok);

            if (!cyclesTok.isEmpty() && !parseUnsigned(cyclesTok, cyclesRaw)) {
                Serial.println("Format error. Example: OTCYCLE START 3 920");
                return true;
            }
            if (!stepsTok.isEmpty() && !parseUnsigned(stepsTok, stepsRaw)) {
                Serial.println("Format error. Example: OTCYCLE START 3 920");
                return true;
            }
            if (cyclesRaw > 255U) {
                Serial.println("Format error. Example: OTCYCLE START 3 920");
                return true;
            }
            (void)startOtvodCycle(static_cast<uint8_t>(cyclesRaw), stepsRaw);
            return true;
        }

        Serial.println("Format error. Use: OTCYCLE START [cycles] [steps] | STATUS | STOP");
        return true;
    }

    if (cmd == "R1ON") {
        if (g_otvodCycle.active) {
            Serial.println("OTCYCLE: активен, ручное включение реле запрещено. Используйте OTCYCLE STOP.");
            return true;
        }
        writeRelayOutput(true);
        g_vfdRelayRun.active = false;
        Serial.println("R1: включено.");
        return true;
    }

    if (cmd == "R1OFF") {
        if (g_otvodCycle.active) {
            abortOtvodCycle("OTCYCLE: остановлено командой R1OFF.", true);
            return true;
        }
        if (g_vfdRelayRun.active) {
            stopTimedRelayRun("ручной стоп");
            return true;
        }
        writeRelayOutput(false);
        Serial.println("R1: выключено.");
        return true;
    }

    if (cmd == "VFD5MIN") {
        if (g_otvodCycle.active) {
            Serial.println("OTCYCLE: активен, VFD5MIN запрещен. Используйте OTCYCLE STOP.");
            return true;
        }
        if (!args.isEmpty()) {
            Serial.println("Format error. Example: VFD5MIN");
            return true;
        }
        startTimedRelayRun(VFD_RELAY_TEST_DURATION_MS);
        return true;
    }

    if (cmd == "VFDSTOP") {
        if (g_otvodCycle.active) {
            abortOtvodCycle("OTCYCLE: остановлено командой VFDSTOP.", true);
            return true;
        }
        if (!g_vfdRelayRun.active) {
            writeRelayOutput(false);
            Serial.println("VFD: реле уже выключено.");
            return true;
        }
        stopTimedRelayRun("команда VFDSTOP");
        return true;
    }

    if (cmd == "VFDTICK") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (subCmd.isEmpty() || subCmd == "SHOW" || subCmd == "GET") {
            if (!subArgs.isEmpty()) {
                Serial.println("Format error. Example: VFDTICK SHOW");
                return true;
            }
            printVfdTickSetting();
            return true;
        }

        if (subCmd == "RUN") {
            if (g_otvodCycle.active) {
                Serial.println("OTCYCLE: активен, ручной VFDTICK RUN запрещен. Используйте OTCYCLE STOP.");
                return true;
            }
            if (!subArgs.isEmpty()) {
                Serial.println("Format error. Example: VFDTICK RUN");
                return true;
            }
            Serial.print("VFDTICK: рабочий такт на ");
            Serial.print(static_cast<float>(g_vfdTickDurationMs) / 1000.0f, 2);
            Serial.println(" сек.");
            startTimedRelayRun(g_vfdTickDurationMs);
            return true;
        }

        uint32_t durationMs = 0;
        if (!parseSecondsToDurationMs(subArgs, durationMs)) {
            Serial.println("Format error. Example: VFDTICK TEST 2.19");
            return true;
        }

        if (subCmd == "TEST") {
            if (g_otvodCycle.active) {
                Serial.println("OTCYCLE: активен, VFDTICK TEST запрещен. Используйте OTCYCLE STOP.");
                return true;
            }
            Serial.print("VFDTICK: проверка на ");
            Serial.print(static_cast<float>(durationMs) / 1000.0f, 2);
            Serial.println(" сек.");
            startTimedRelayRun(durationMs);
            return true;
        }

        if (subCmd == "SAVE") {
            if (g_otvodCycle.active) {
                Serial.println("OTCYCLE: активен, VFDTICK SAVE запрещен до завершения цикла.");
                return true;
            }
            g_vfdTickDurationMs = durationMs;
            saveVfdTickSetting();
            return true;
        }

        Serial.println("Format error. Examples: VFDTICK SHOW | VFDTICK RUN | VFDTICK TEST 2.19 | VFDTICK SAVE 2.19");
        return true;
    }

    if (cmd == "W") {
        digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
        Serial.println("Flag: UP.");
        return true;
    }

    if (cmd == "MAINSTART") {
        startMainContinuousMotion(CONTINUOUS_STEP_DELAY_US);
        return true;
    }

    if (cmd == "MAINSTOP") {
        if (!g_motion.active) {
            Serial.println("MAIN: PUL13 уже остановлен.");
            return true;
        }
        stopMotion();
        Serial.println("MAIN: PUL13 остановлен.");
        return true;
    }

    if (cmd == "POSSTART") {
        startPositionalContinuousMotion(CONTINUOUS_STEP_DELAY_US);
        return true;
    }

    if (cmd == "POSSTOP") {
        if (!g_posMotion.active) {
            Serial.println("POS: PUL32 уже остановлен.");
            return true;
        }
        stopPositionalMotion();
        return true;
    }

    if (cmd == "S") {
        digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
        Serial.println("Flag: DOWN.");
        return true;
    }

    if (cmd == "E") {
        g_sensorStreamEnabled = !g_sensorStreamEnabled;
        g_lastSensorPrintMs = millis();
        Serial.println(g_sensorStreamEnabled ? "Sensor stream ON." : "Sensor stream OFF.");
        return true;
    }

    if (cmd == "STOP2") {
        if (g_otvodCycle.active) {
            abortOtvodCycle("OTCYCLE: остановлено командой STOP2.", true);
            return true;
        }
        if (!g_step2Motion.active) {
            Serial.println("STEP2: тест не запущен.");
            return true;
        }
        stopStep2Motion();
        Serial.println("STEP2: остановлено.");
        return true;
    }

    if (cmd == "T2" || cmd == "STEP2") {
        if (g_otvodCycle.active) {
            Serial.println("OTCYCLE: активен, ручной STEP2 запрещен. Используйте OTCYCLE STOP.");
            return true;
        }
        uint32_t steps = STEP2_DEFAULT_STEPS;
        if (!args.isEmpty() && !parseUnsigned(args, steps)) {
            Serial.println("Format error. Example: T2 5000");
            return true;
        }
        startStep2Motion(steps, STEP2_RUN_DELAY_US);
        return true;
    }

    if (cmd == "OTVOD") {
        if (g_otvodCycle.active) {
            Serial.println("OTCYCLE: активен, ручной OTVOD запрещен. Используйте OTCYCLE STOP.");
            return true;
        }
        uint32_t steps = STEP2_DIVERT_STEPS;
        if (!args.isEmpty() && !parseUnsigned(args, steps)) {
            Serial.println("Format error. Example: OTVOD 920");
            return true;
        }
        startStep2Motion(steps, STEP2_RUN_DELAY_US);
        return true;
    }

    if (cmd == "STEP2START") {
        if (g_otvodCycle.active) {
            Serial.println("OTCYCLE: активен, STEP2START запрещен. Используйте OTCYCLE STOP.");
            return true;
        }
        startStep2ContinuousMotion(CONTINUOUS_STEP_DELAY_US);
        return true;
    }

    if (cmd == "STEP2STOP") {
        if (g_otvodCycle.active) {
            abortOtvodCycle("OTCYCLE: остановлено командой STEP2STOP.", true);
            return true;
        }
        if (!g_step2Motion.active) {
            Serial.println("STEP2: GPIO19 уже остановлен.");
            return true;
        }
        stopStep2Motion();
        Serial.println("STEP2: GPIO19 остановлен.");
        return true;
    }

    if (cmd == "H") {
        printHelp();
        return true;
    }

    return false;
}

void handleCommand(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    String cmd;
    String args;
    splitCommandLine(line, cmd, args);

    if (g_otvodCycle.active) {
        const bool commandAllowedDuringOtvodCycle =
            (cmd == "OTCYCLE" || cmd == "H" || cmd == "E" ||
             cmd == "VFDTICK" || cmd == "R1OFF" || cmd == "VFDSTOP" ||
             cmd == "STOP2" || cmd == "STEP2STOP");
        if (!commandAllowedDuringOtvodCycle) {
            Serial.println("OTCYCLE: цикл активен, команда запрещена. Используйте OTCYCLE STATUS или OTCYCLE STOP.");
            return;
        }
    }

    if (tryHandleDistanceMoveCommand(cmd, args) ||
        tryHandleProgramCommand(cmd) ||
        tryHandleShiftCommand(cmd) ||
        tryHandleServiceCommand(cmd, args)) {
        return;
    }

    Serial.println("Unknown command. Use H for help.");
}

void readSerialCommands()
{
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_cmdBuffer.isEmpty()) {
                handleCommand(g_cmdBuffer);
                g_cmdBuffer = "";
            }
            continue;
        }

        g_cmdBuffer += ch;
    }
}

void processTimedRelayRun()
{
    if (!g_vfdRelayRun.active) {
        return;
    }

    const uint32_t elapsedMs = static_cast<uint32_t>(millis() - g_vfdRelayRun.startedMs);
    if (elapsedMs < g_vfdRelayRun.durationMs) {
        return;
    }

    stopTimedRelayRun("таймер истек");
}

void processSealIo()
{
    const bool doneActive = sealIsDoneActive();
    if (doneActive != g_sealIo.doneLastActive) {
        g_sealIo.doneLastActive = doneActive;
        if (doneActive) {
            g_sealIo.doneLastRiseMs = millis();
            Serial.println("SEAL: cycle complete input became active.");
        } else {
            Serial.println("SEAL: cycle complete input released.");
        }
    }

    if (!g_sealIo.startPulseActive) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_sealIo.startPulseStartedMs) < g_sealIo.startPulseDurationMs) {
        return;
    }

    sealWriteStartOutput(false);
    g_sealIo.startPulseActive = false;
    Serial.println("SEAL: start pulse finished.");
}

void processOtvodCycle()
{
    if (!g_otvodCycle.active) {
        return;
    }

    const uint32_t nowMs = millis();

    if (g_otvodCycle.stepRunning) {
        if ((uint32_t)(nowMs - g_otvodCycle.stepStartedMs) > OTVOD_CYCLE_STEP_PHASE_TIMEOUT_MS) {
            abortOtvodCycle("OTCYCLE: ошибка, STEP2 не завершился по таймауту.", true);
            return;
        }

        const bool step2CompletedLatched =
            g_step2CompletedSeq != g_otvodCycle.stepCompletionSeqBase;
        if (g_step2Motion.active) {
            g_otvodCycle.stepObservedActive = true;
        }

        if (step2CompletedLatched || (!g_step2Motion.active && g_otvodCycle.stepObservedActive)) {
            g_otvodCycle.stepRunning = false;
            g_otvodCycle.stepObservedActive = false;
            g_otvodCycle.stepRunsCompleted++;
            g_otvodCycle.stepCompletedMs = nowMs;
            g_otvodCycle.phase = OtvodCyclePhase::WaitMainStart;
            g_otvodCycle.readyForBatch =
                g_otvodCycle.stepRunsCompleted >= g_otvodCycle.totalCycles;

            Serial.print("OTCYCLE: двухручейковый завершен ");
            Serial.print(g_otvodCycle.stepRunsCompleted);
            Serial.print("/");
            Serial.println(g_otvodCycle.totalCycles);

            if (g_otvodCycle.readyForBatch) {
                g_otvodCycle.lastEvent = "OTCYCLE: двухручейковый готов принять новую партию.";
                Serial.println(g_otvodCycle.lastEvent);
            } else {
                g_otvodCycle.lastEvent = "OTCYCLE: ожидание старта основного отвода.";
            }
        }
    }

    if (g_otvodCycle.active &&
        !g_otvodCycle.stepRunning &&
        g_otvodCycle.stepRunsStarted > g_otvodCycle.vfdRunsStarted &&
        !g_otvodCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodCycle.stepCompletedMs);
        if (elapsedMs >= g_otvodCycle.stepTriggerDelayMs) {
            if (!startOtvodCycleMainTick(g_otvodCycle.stepRunsStarted, nowMs)) {
                abortOtvodCycle("OTCYCLE: ошибка запуска основного отвода.", true);
                return;
            }
            return;
        }
    }

    if (g_otvodCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodCycle.vfdStartedMs);

        if (g_vfdRelayRun.active) {
            g_otvodCycle.vfdObservedActive = true;
        } else if (g_otvodCycle.vfdObservedActive) {
            g_otvodCycle.vfdRunning = false;
            g_otvodCycle.vfdObservedActive = false;
            g_otvodCycle.vfdRunsCompleted++;
            g_otvodCycle.vfdCompletedMs = nowMs;
            g_otvodCycle.phase = OtvodCyclePhase::WaitNextStep;
            g_otvodCycle.lastEvent = "OTCYCLE: основной отвод завершен " +
                String(g_otvodCycle.vfdRunsCompleted) + "/" + String(g_otvodCycle.totalCycles);
            Serial.println(g_otvodCycle.lastEvent);
            return;
        }

        if (!g_otvodCycle.vfdObservedActive && elapsedMs > OTVOD_CYCLE_VFD_START_TIMEOUT_MS) {
            abortOtvodCycle("OTCYCLE: основной отвод не стартовал по таймауту.", true);
            return;
        }

        const uint32_t timeoutMs =
            (g_otvodCycle.vfdDoneDelayMs > 0U)
                ? (g_otvodCycle.vfdDoneDelayMs + OTVOD_CYCLE_VFD_TIMEOUT_MARGIN_MS)
                : OTVOD_CYCLE_VFD_TIMEOUT_FALLBACK_MS;
        if (elapsedMs > timeoutMs) {
            abortOtvodCycle("OTCYCLE: основной отвод не завершился по таймауту.", true);
            return;
        }
    }

    if (g_otvodCycle.active &&
        !g_otvodCycle.vfdRunning &&
        !g_otvodCycle.stepRunning &&
        g_otvodCycle.vfdRunsCompleted < g_otvodCycle.totalCycles &&
        g_otvodCycle.stepRunsStarted == g_otvodCycle.vfdRunsCompleted) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodCycle.vfdCompletedMs);
        if (elapsedMs >= g_otvodCycle.vfdTriggerDelayMs) {
            const uint8_t nextCycleIndex = static_cast<uint8_t>(g_otvodCycle.vfdRunsCompleted + 1U);
            if (!startOtvodCycleStep(nextCycleIndex, nowMs)) {
                abortOtvodCycle("OTCYCLE: ошибка запуска следующего прохода OTVOD.", true);
                return;
            }
        }
    }

    if (g_otvodCycle.active &&
        !g_otvodCycle.stepRunning &&
        !g_otvodCycle.vfdRunning &&
        g_otvodCycle.stepRunsCompleted >= g_otvodCycle.totalCycles &&
        g_otvodCycle.vfdRunsCompleted >= g_otvodCycle.totalCycles) {
        finishOtvodCycle("OTCYCLE: завершено, циклов " + String(g_otvodCycle.totalCycles) + ".");
    }
}

bool isConveyorSensorConflict()
{
    return shiftIsSensorZTriggered() && shiftIsSensorCTriggered();
}

bool isConveyorBusy()
{
    return g_motion.active ||
           g_posMotion.active ||
           g_step2Motion.active ||
           g_c2Active ||
           g_vfdRelayRun.active ||
           g_sealIo.startPulseActive ||
           g_otvodCycle.active ||
           shiftHookIsMotionActive() ||
           shiftHookIsCycle2Active() ||
           shiftHookIsPositionalMotionActive();
}

uint16_t getConveyorStatusWord()
{
    const bool busy = isConveyorBusy();
    const bool conflict = isConveyorSensorConflict();
    const bool calibrated = shiftIsCalibrated();
    const bool program1Active = program1GetStateCode() != 0;
    const bool batchReady = program1IsBatchReadyForManipulator();

    uint16_t word = 0;
    if (!conflict) {
        word |= STATUS_READY_BIT;
        word |= STATUS_SAFE_BIT;
    }
    if (busy) {
        word |= STATUS_BUSY_BIT;
    }
    if (conflict) {
        word |= STATUS_ALARM_BIT;
    }
    if (calibrated) {
        word |= STATUS_CALIBRATED_BIT;
    }
    if (program1Active) {
        word |= STATUS_PROGRAM1_ACTIVE_BIT;
    }
    if (batchReady) {
        word |= STATUS_BATCH_READY_BIT;
    }
    if (g_step2Motion.active) {
        word |= STATUS_STEP2_ACTIVE_BIT;
    }
    word |= static_cast<uint16_t>(g_step2CompletedSeq) << 8;
    return word;
}

uint16_t getConveyorErrorWord()
{
    uint16_t word = 0;
    if (isConveyorSensorConflict()) {
        word |= ERROR_SENSOR_CONFLICT_BIT;
    }
    if (!shiftIsCalibrated()) {
        word |= ERROR_NOT_CALIBRATED_BIT;
    }
    return word;
}

uint16_t getConveyorFlagState()
{
    return digitalRead(PIN_FLAG) == FLAG_UP_LEVEL ? FLAG_STATE_UP : FLAG_STATE_DOWN;
}

uint16_t getConveyorMotionState()
{
    if (g_posMotion.active || shiftHookIsPositionalMotionActive()) {
        return MOTION_STATE_POSITIONING;
    }
    if (g_motion.active || g_c2Active || g_otvodCycle.active ||
        shiftHookIsMotionActive() || shiftHookIsCycle2Active()) {
        return MOTION_STATE_RUNNING;
    }
    return MOTION_STATE_IDLE;
}

uint16_t getConveyorPackedExtra0()
{
    const uint16_t flagState = static_cast<uint16_t>(getConveyorFlagState() & 0x00FFU);
    const uint16_t motionState = static_cast<uint16_t>(getConveyorMotionState() & 0x00FFU);
    return static_cast<uint16_t>(flagState | (motionState << 8));
}

uint16_t getConveyorVfdTickDurationEncoded()
{
    uint32_t duration10Ms = (g_vfdTickDurationMs + 5U) / 10U;
    if (duration10Ms > 0xFFFFU) {
        duration10Ms = 0xFFFFU;
    }
    return static_cast<uint16_t>(duration10Ms);
}

uint32_t checksumI2cStatusFrame(const I2cStatusFrame &frame)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
    uint32_t sum = 0x13572468UL;
    for (size_t i = 0; i < sizeof(I2cStatusFrame) - sizeof(frame.checksum); i++) {
        sum = (sum << 5) | (sum >> 27);
        sum ^= bytes[i];
    }
    return sum;
}

void fillI2cStatusFrame(I2cStatusFrame &frame)
{
    memset(&frame, 0, sizeof(frame));
    frame.magic = I2C_FRAME_MAGIC;
    frame.protoVer = I2C_FRAME_PROTO_VER;
    frame.deviceKind = DEVICE_KIND_CONVEYOR;
    frame.deviceIdEcho = I2C_DEVICE_ADDRESS;
    frame.statusWord = getConveyorStatusWord();
    frame.errorWord = getConveyorErrorWord();
    frame.extra0 = getConveyorPackedExtra0();
    frame.extra1 = getConveyorVfdTickDurationEncoded();
    frame.extra2 = static_cast<uint16_t>(program1GetStateCode()) |
                   (static_cast<uint16_t>(program1GetPassIndex()) << 8);
    frame.extra3 = static_cast<uint16_t>(program1GetBatchReadySequence() & 0x7FFFU);
    if (program1IsBatchReadyForManipulator()) {
        frame.extra3 |= 0x8000U;
    }
    frame.heartbeatMs = millis();
    frame.checksum = checksumI2cStatusFrame(frame);
}

void onI2cReceive(int len)
{
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

void onI2cRequest()
{
    I2cStatusFrame frame = {};
    fillI2cStatusFrame(frame);
    Wire.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
    g_i2cTxCount++;
}

void initI2cStatusBus()
{
    Wire.begin(I2C_DEVICE_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    Wire.onReceive(onI2cReceive);
    Wire.onRequest(onI2cRequest);
}

void processPendingI2cCommand()
{
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
    handleCommand(line);
}

void initHardwarePins()
{
    pinMode(PIN_STEP_PUL, OUTPUT);
    pinMode(PIN_STEP2_PUL, OUTPUT);
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    shiftInitHardwarePins();
    pinMode(PIN_FLAG, OUTPUT);
    pinMode(PIN_SENSOR, INPUT_PULLUP);
    pinMode(PIN_POS_PUL, OUTPUT);
    pinMode(PIN_SENSOR_EXT_1, INPUT);
}

void initHardwareStates()
{
    writePulseInactive();
    writeStep2PulseInactive();
    writeRelayOutput(false);
    sealWriteStartOutput(false);
    g_sealIo.doneLastActive = sealIsDoneActive();
    shiftInitHardwareStates();
    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
    writePosPulseInactive();
}

void initRuntimeState()
{
    initProgram1Storage();
    initSettingsStorage();
    updateSensorFilter();
    shiftUpdateSensorFilters();
    homeShiftToZOnStartup();
}

void printStartupBanner()
{
    Serial.println();
    Serial.print("=== ");
    Serial.print(DEVICE_NAME);
    Serial.println(" ===");
    Serial.print("Версия прошивки: ");
    Serial.println(FW_VERSION);
    Serial.println("Плата: ESP32 Dev Module");
    Serial.println("DM542: PUL=13");
    Serial.println("DM542 positional: PUL=GPIO32");
    Serial.println("STEP2 / отводной: PUL=GPIO19, команды T2 / OTVOD.");
    Serial.println("Запущено два конвейера + сдвиг тарелки.");
    Serial.println("Сдвиг тарелки: DIR=GPIO14, PUL=GPIO23, Z=GPIO33, C=GPIO25.");
    Serial.println("I2C status: addr 12, SDA=GPIO16, SCL=GPIO17.");
    Serial.println("Сдвиг по умолчанию: фиксированный ход 2255 шагов.");
    Serial.print("P1 buffer: ");
    Serial.println(g_program1BufferReady ? "есть 2 тарелки." : "пусто.");
    Serial.print("Реле частотника: GPIO21, активный уровень=");
    Serial.println(RELAY_ACTIVE_LEVEL == HIGH ? "HIGH" : "LOW");
    printVfdTickSetting();
    Serial.print("Запайщик: START=GPIO");
    Serial.print(PIN_RELAY_2);
    Serial.print(", DONE=GPIO");
    Serial.print(PIN_SENSOR_EXT_1);
    Serial.println(" (GPIO34 без внутренней подтяжки, нужна внешняя).");
    Serial.println("Sensor: GPIO26 (E18-D50NK)");
    Serial.println("Флаг вверх: GPIO27=HIGH");
    Serial.println("Команды не чувствительны к регистру.");
    printHelp();
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    initHardwarePins();
    initHardwareStates();
    initI2cStatusBus();
    initRuntimeState();
    printStartupBanner();
}

void loop()
{
    updateSensorFilter();
    shiftUpdateSensorFilters();
    processPendingI2cCommand();
    readSerialCommands();

    processCycle2();
    processMotion();
    processPositionalMotion();
    processStep2Motion();
    processProgram1();
    processSensorStream();
    processTimedRelayRun();
    processSealIo();
    processOtvodCycle();
}

