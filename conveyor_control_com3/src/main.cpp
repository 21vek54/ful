#include <Arduino.h>
#include <Preferences.h>

#include "pins.h"
#include "program1.h"
#include "shift_control.h"

constexpr char FW_VERSION[] = "v1.11";
#if defined(DEVICE_ROLE_CONVEYOR)
constexpr char DEVICE_NAME[] = "Управление конвейерами";
#else
constexpr char DEVICE_NAME[] = "Conveyor controller";
#endif

constexpr bool FLAG_UP_LEVEL = HIGH;
constexpr bool FLAG_DOWN_LEVEL = LOW;
constexpr bool PULSE_ACTIVE_LEVEL = HIGH;

constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
constexpr uint32_t POS_RUN_DELAY_US = 1000;
constexpr uint32_t SENSOR_PRINT_INTERVAL_MS = 500;
constexpr uint32_t SENSOR_DEBOUNCE_MS = 80;
constexpr uint32_t MANUAL_MOVE_DELAY_US = 1500; // Фиксированная задержка для команд A/D
constexpr uint32_t RS485_BAUD = 115200;
constexpr size_t RS485_CMD_MAX_LEN = 48;
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
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t delayUs = POS_RUN_DELAY_US;
    uint32_t currentDelayUs = POS_RUN_DELAY_US;
    uint32_t startDelayUs = POS_RUN_DELAY_US;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
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

String g_cmdBuffer;
String g_rs485CmdBuffer;
Preferences g_preferences;
MotionState g_motion;
PosMotionState g_posMotion;
SensorFilterState g_sensorFilter;
bool g_sensorStreamEnabled = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_totalStepsCounter = 0;
bool g_program1BufferReady = false;
bool g_program1StorageReady = false;

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

bool isPlateAtSensorFiltered();
void stopMotion();
void startCycle2();
void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs);
void processPositionalMotion();
void updateSensorFilter();
void initProgram1Storage();
void setProgram1BufferReady(bool ready);
void initRs485();
void processRs485Commands();

void printHelp()
{
    Serial.println("Команды:");
    Serial.println("  D 200        - вправо, 200 мм (задержка 1500 мкс)");
    Serial.println("  1            - автоцикл: 2 -> C+(3+Z) -> 2 -> C+(3+Z) -> 2 -> C+Z");
    Serial.println("  2            - рабочий ход: без центрирования, формула 150+34-UT, добег 100 мм");
    Serial.println("  3            - позиционный (PUL32): вправо 184 мм, профиль, полка 1200 мкс");
    Serial.println("  W            - флаг вверх");
    Serial.println("  S            - флаг вниз");
    Serial.println("  E            - вкл/выкл поток датчиков (E18 + 2 геркона, каждые 0.5 сек)");
    Serial.println("  P 300        - позиционный (PUL32): вправо 300 мм, профиль, полка 1000 мкс");
    Serial.println("  C            - сдвиг тарелки: 500 шагов (DIR14/PUL23), задержка 500 мкс, S-профиль, длинное торможение");
    Serial.println("  Z            - возврат тарелки: 500 шагов (DIR14/PUL23), задержка 300 мкс, плавный ход");
    Serial.println("  CZ           - калибровка хода сдвига по герконам Z=33 и C=25");
    Serial.println("  RS485        - удаленно: W/S, UP/DOWN, FLAG UP/FLAG DOWN");
    Serial.println("  H            - помощь");
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
    Serial.println(shiftIsSensorZTriggered() ? "замкнут (HIGH)" : "разомкнут (LOW)");
    Serial.print("Геркон C GPIO25: ");
    Serial.println(shiftIsSensorCTriggered() ? "замкнут (HIGH)" : "разомкнут (LOW)");
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

bool tryHandleServiceCommand(const String &cmd)
{
    if (cmd == "W") {
        digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
        Serial.println("Flag: UP.");
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

    if (tryHandleDistanceMoveCommand(cmd, args) ||
        tryHandleProgramCommand(cmd) ||
        tryHandleShiftCommand(cmd) ||
        tryHandleServiceCommand(cmd)) {
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

bool tryHandleRs485FlagCommand(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return false;
    }

    line.toUpperCase();

    if (line == "W" || line == "UP" || line == "FLAG UP") {
        digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
        Serial.println("RS485: Flag UP.");
        return true;
    }

    if (line == "S" || line == "DOWN" || line == "FLAG DOWN") {
        digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
        Serial.println("RS485: Flag DOWN.");
        return true;
    }

    return false;
}

void processRs485Commands()
{
    while (Serial2.available() > 0) {
        const char ch = static_cast<char>(Serial2.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_rs485CmdBuffer.isEmpty()) {
                if (!tryHandleRs485FlagCommand(g_rs485CmdBuffer)) {
                    Serial.print("RS485: unknown cmd '");
                    Serial.print(g_rs485CmdBuffer);
                    Serial.println("'. Allowed: W/S, UP/DOWN, FLAG UP/DOWN.");
                }
                g_rs485CmdBuffer = "";
            }
            continue;
        }

        g_rs485CmdBuffer += ch;
        if (g_rs485CmdBuffer.length() > RS485_CMD_MAX_LEN) {
            g_rs485CmdBuffer = "";
            Serial.println("RS485: command too long, buffer cleared.");
        }
    }
}

void initRs485()
{
    pinMode(PIN_RS485_DE_RE, OUTPUT);
    digitalWrite(PIN_RS485_DE_RE, LOW); // Режим приема (RE=0, DE=0)
    Serial2.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
}

void initHardwarePins()
{
    pinMode(PIN_STEP_PUL, OUTPUT);
    shiftInitHardwarePins();
    pinMode(PIN_FLAG, OUTPUT);
    pinMode(PIN_SENSOR, INPUT_PULLUP);
    pinMode(PIN_POS_PUL, OUTPUT);
}

void initHardwareStates()
{
    writePulseInactive();
    shiftInitHardwareStates();
    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
    writePosPulseInactive();
}

void initRuntimeState()
{
    initProgram1Storage();
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
    Serial.println("Запущено два конвейера + сдвиг тарелки.");
    Serial.println("Сдвиг тарелки: DIR=GPIO14, PUL=GPIO23, Z=GPIO33, C=GPIO25.");
    Serial.println("RS485: RX=GPIO16, TX=GPIO17, DE/RE=GPIO18, 115200 8N1.");
    Serial.println("Сдвиг по умолчанию: фиксированный ход 2255 шагов.");
    Serial.print("P1 buffer: ");
    Serial.println(g_program1BufferReady ? "есть 2 тарелки." : "пусто.");
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
    initRs485();
    initRuntimeState();
    printStartupBanner();
}

void loop()
{
    updateSensorFilter();
    shiftUpdateSensorFilters();
    readSerialCommands();
    processRs485Commands();

    processCycle2();
    processMotion();
    processPositionalMotion();
    processProgram1();
    processSensorStream();
}
