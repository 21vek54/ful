#include <Arduino.h>

#include "pins.h"
#include "shift_control.h"

namespace {

constexpr bool PULSE_ACTIVE_LEVEL = HIGH;
constexpr bool SHIFT_DIR_C_LEVEL = HIGH;
constexpr bool SHIFT_DIR_Z_LEVEL = LOW;

constexpr uint32_t SHIFT_COMMAND_STEPS = 500;
constexpr uint32_t SHIFT_COMMAND_DELAY_C_US = 300;
constexpr uint32_t SHIFT_COMMAND_DELAY_Z_US = 50;
constexpr uint32_t SHIFT_PULSE_WIDTH_US = 120;
constexpr uint32_t SHIFT_START_DELAY_C_US = 3000;
constexpr uint32_t SHIFT_START_DELAY_Z_US = 1000;
constexpr uint32_t SHIFT_RAMP_STEPS_C = 500;
constexpr uint32_t SHIFT_RAMP_STEPS_Z = 180;
constexpr uint32_t SHIFT_DECEL_STEPS_C = 1000;
constexpr uint32_t SHIFT_SENSOR_DEBOUNCE_MS = 25;
constexpr uint32_t SHIFT_LEAVE_SENSOR_MAX_STEPS = 300;
constexpr uint32_t SHIFT_CAL_SEARCH_MAX_STEPS = 12000;
constexpr uint32_t SHIFT_CAL_MOVE_MARGIN_STEPS = 50;
constexpr uint32_t SHIFT_TRAVEL_STEPS_FIXED = 2255;

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
};

SensorFilterState g_shiftSensorZFilter;
SensorFilterState g_shiftSensorCFilter;
bool g_shiftCalibrated = true;
uint32_t g_shiftTravelSteps = SHIFT_TRAVEL_STEPS_FIXED;

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

uint32_t interpolateDelayUs(uint32_t progressSteps, uint32_t rampSteps, uint32_t startDelayUs, uint32_t nominalDelayUs)
{
    if (rampSteps == 0U || progressSteps >= rampSteps || startDelayUs <= nominalDelayUs) {
        return nominalDelayUs;
    }

    const uint32_t delta = startDelayUs - nominalDelayUs;
    const uint32_t dec = static_cast<uint32_t>((static_cast<uint64_t>(delta) * progressSteps) / rampSteps);
    return startDelayUs - dec;
}

void writeShiftPulseInactive()
{
    digitalWrite(PIN_SHIFT_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void serviceShiftBackground(bool allowPositionalOverlap)
{
    shiftHookUpdateMainSensorFilter();
    shiftUpdateSensorFilters();
    if (allowPositionalOverlap) {
        shiftHookProcessPositionalMotion();
    }
}

void delayShiftUs(uint32_t delayUs, bool allowPositionalOverlap)
{
    const uint32_t startUs = micros();
    while ((uint32_t)(micros() - startUs) < delayUs) {
        serviceShiftBackground(allowPositionalOverlap);
    }
}

uint32_t getShiftNominalDelayUs(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? SHIFT_COMMAND_DELAY_Z_US : SHIFT_COMMAND_DELAY_C_US;
}

uint32_t getShiftStartDelayUs(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? SHIFT_START_DELAY_Z_US : SHIFT_START_DELAY_C_US;
}

uint32_t interpolateSmoothDelayUs(
    uint32_t progressSteps,
    uint32_t rampSteps,
    uint32_t startDelayUs,
    uint32_t nominalDelayUs)
{
    if (rampSteps == 0U || progressSteps >= rampSteps || startDelayUs <= nominalDelayUs) {
        return nominalDelayUs;
    }

    const uint32_t delta = startDelayUs - nominalDelayUs;
    constexpr uint32_t kScale = 4096U;

    const uint64_t x = (static_cast<uint64_t>(progressSteps) * kScale) / rampSteps;
    const uint64_t easeScaled =
        ((3ULL * x * x * kScale) - (2ULL * x * x * x)) /
        (static_cast<uint64_t>(kScale) * kScale);

    const uint32_t dec = static_cast<uint32_t>(
        (static_cast<uint64_t>(delta) * easeScaled) / kScale);
    return startDelayUs - dec;
}

uint32_t getShiftStepDelayUs(bool dirLevel, uint32_t stepIndex, uint32_t totalPlannedSteps)
{
    const uint32_t nominalDelayUs = getShiftNominalDelayUs(dirLevel);
    const uint32_t startDelayUs = getShiftStartDelayUs(dirLevel);
    uint32_t rampSteps = (dirLevel == SHIFT_DIR_C_LEVEL) ? SHIFT_RAMP_STEPS_C : SHIFT_RAMP_STEPS_Z;
    uint32_t decelRampSteps = (dirLevel == SHIFT_DIR_C_LEVEL) ? SHIFT_DECEL_STEPS_C : SHIFT_RAMP_STEPS_Z;
    if (totalPlannedSteps != 0U) {
        const uint32_t halfSteps = totalPlannedSteps / 2U;
        if (halfSteps < rampSteps) {
            rampSteps = halfSteps;
        }
        if (halfSteps < decelRampSteps) {
            decelRampSteps = halfSteps;
        }
    }
    if (rampSteps == 0U) {
        rampSteps = 1U;
    }
    if (decelRampSteps == 0U) {
        decelRampSteps = 1U;
    }

    uint32_t accelProgress = stepIndex;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    const bool useSCurve = (dirLevel == SHIFT_DIR_C_LEVEL);
    uint32_t targetDelayUs = useSCurve
                                 ? interpolateSmoothDelayUs(
                                       accelProgress,
                                       rampSteps,
                                       startDelayUs,
                                       nominalDelayUs)
                                 : interpolateDelayUs(
                                       accelProgress,
                                       rampSteps,
                                       startDelayUs,
                                       nominalDelayUs);

    if (totalPlannedSteps != 0U) {
        uint32_t remainingSteps = 0U;
        if (totalPlannedSteps > stepIndex) {
            remainingSteps = totalPlannedSteps - stepIndex;
        }

        uint32_t decelProgress = remainingSteps;
        if (decelProgress > decelRampSteps) {
            decelProgress = decelRampSteps;
        }

        const uint32_t decelDelayUs = useSCurve
                                          ? interpolateSmoothDelayUs(
                                                decelProgress,
                                                decelRampSteps,
                                                startDelayUs,
                                                nominalDelayUs)
                                          : interpolateDelayUs(
                                                decelProgress,
                                                decelRampSteps,
                                                startDelayUs,
                                                nominalDelayUs);
        if (decelDelayUs > targetDelayUs) {
            targetDelayUs = decelDelayUs;
        }
    }

    if (targetDelayUs < nominalDelayUs) {
        targetDelayUs = nominalDelayUs;
    }

    return targetDelayUs;
}

using ShiftSensorFn = bool (*)();

bool readShiftSensorZRaw()
{
    return digitalRead(PIN_SHIFT_SENSOR_Z) == HIGH;
}

bool readShiftSensorCRaw()
{
    return digitalRead(PIN_SHIFT_SENSOR_C) == HIGH;
}

ShiftSensorFn getShiftTargetSensorFn(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? shiftIsSensorZTriggered : shiftIsSensorCTriggered;
}

ShiftSensorFn getShiftOppositeSensorFn(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? shiftIsSensorCTriggered : shiftIsSensorZTriggered;
}

bool isShiftTargetSensorTriggered(bool dirLevel)
{
    return getShiftTargetSensorFn(dirLevel)();
}

bool isShiftOppositeSensorTriggered(bool dirLevel)
{
    return getShiftOppositeSensorFn(dirLevel)();
}

void prepareShiftMove(bool dirLevel)
{
    digitalWrite(PIN_SHIFT_DIR, dirLevel);
    writeShiftPulseInactive();
    delayMicroseconds(200);
}

void pulseShiftStep(bool dirLevel, bool allowPositionalOverlap, uint32_t stepIndex, uint32_t totalPlannedSteps)
{
    const uint32_t shiftDelayUs = getShiftStepDelayUs(dirLevel, stepIndex, totalPlannedSteps);
    digitalWrite(PIN_SHIFT_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
    delayShiftUs(SHIFT_PULSE_WIDTH_US, allowPositionalOverlap);
    writeShiftPulseInactive();
    delayShiftUs(shiftDelayUs, allowPositionalOverlap);
}

bool shiftLeaveSensor(
    bool dirLevel,
    ShiftSensorFn sensorFn,
    uint32_t maxSteps,
    uint32_t &stepsDone,
    uint32_t &profileStepIndex,
    uint32_t profileTotalSteps,
    bool allowPositionalOverlap)
{
    stepsDone = 0;
    if (!sensorFn()) {
        return true;
    }

    prepareShiftMove(dirLevel);
    for (uint32_t i = 0; i < maxSteps; i++) {
        pulseShiftStep(dirLevel, allowPositionalOverlap, profileStepIndex, profileTotalSteps);
        stepsDone++;
        profileStepIndex++;
        if (!sensorFn()) {
            return true;
        }
    }

    return false;
}

bool shiftFindSensor(
    bool dirLevel,
    ShiftSensorFn sensorFn,
    uint32_t maxSteps,
    uint32_t &stepsDone,
    uint32_t &profileStepIndex,
    uint32_t profileTotalSteps,
    bool allowPositionalOverlap)
{
    stepsDone = 0;
    prepareShiftMove(dirLevel);
    for (uint32_t i = 0; i < maxSteps; i++) {
        pulseShiftStep(dirLevel, allowPositionalOverlap, profileStepIndex, profileTotalSteps);
        stepsDone++;
        profileStepIndex++;
        if (sensorFn()) {
            return true;
        }
    }

    return false;
}

bool leaveShiftSensorWithFreshProfile(
    bool dirLevel,
    ShiftSensorFn sensorFn,
    uint32_t maxSteps,
    uint32_t profileTotalSteps,
    bool allowPositionalOverlap,
    uint32_t &stepsDone)
{
    uint32_t profileStepIndex = 0;
    return shiftLeaveSensor(
        dirLevel,
        sensorFn,
        maxSteps,
        stepsDone,
        profileStepIndex,
        profileTotalSteps,
        allowPositionalOverlap);
}

bool findShiftSensorWithFreshProfile(
    bool dirLevel,
    ShiftSensorFn sensorFn,
    uint32_t maxSteps,
    uint32_t profileTotalSteps,
    bool allowPositionalOverlap,
    uint32_t &stepsDone)
{
    uint32_t profileStepIndex = 0;
    return shiftFindSensor(
        dirLevel,
        sensorFn,
        maxSteps,
        stepsDone,
        profileStepIndex,
        profileTotalSteps,
        allowPositionalOverlap);
}

uint32_t getShiftLeaveLimitSteps()
{
    uint32_t leaveLimit = SHIFT_LEAVE_SENSOR_MAX_STEPS;
    if (g_shiftCalibrated) {
        const uint32_t calibratedLimit = g_shiftTravelSteps + SHIFT_CAL_MOVE_MARGIN_STEPS;
        if (calibratedLimit > leaveLimit) {
            leaveLimit = calibratedLimit;
        }
    }
    return leaveLimit;
}

bool runFixedShiftMove(bool dirLevel, const char *name, bool allowPositionalOverlap)
{
    const uint32_t shiftNominalDelayUs = getShiftNominalDelayUs(dirLevel);
    Serial.print("SHIFT: калибровки нет, ");
    Serial.print(name);
    Serial.print(" выполнится на фиксированные ");
    Serial.print(SHIFT_COMMAND_STEPS);
    Serial.println(" шагов.");

    Serial.print("SHIFT: старт ");
    Serial.print(name);
    Serial.print(", шагов=");
    Serial.print(SHIFT_COMMAND_STEPS);
    Serial.print(", задержка=");
    Serial.print(shiftNominalDelayUs);
    Serial.println(" мкс.");

    prepareShiftMove(dirLevel);
    uint32_t profileStepIndex = 0;
    for (uint32_t i = 0; i < SHIFT_COMMAND_STEPS; i++) {
        pulseShiftStep(dirLevel, allowPositionalOverlap, profileStepIndex, SHIFT_COMMAND_STEPS);
        profileStepIndex++;
    }

    Serial.print("SHIFT: выполнено ");
    Serial.print(name);
    Serial.print(", шагов=");
    Serial.print(SHIFT_COMMAND_STEPS);
    Serial.print(", задержка=");
    Serial.print(shiftNominalDelayUs);
    Serial.println(" мкс.");
    return true;
}

bool moveShiftToTargetEdge(bool dirLevel, bool allowPositionalOverlap, uint32_t &totalSteps)
{
    const uint32_t maxSeekSteps = g_shiftTravelSteps + SHIFT_CAL_MOVE_MARGIN_STEPS;
    totalSteps = 0;

    if (isShiftOppositeSensorTriggered(dirLevel)) {
        uint32_t leaveSteps = 0;
        const uint32_t leaveLimitSteps = getShiftLeaveLimitSteps();
        uint32_t profileStepIndex = totalSteps;
        if (!shiftLeaveSensor(
                dirLevel,
                getShiftOppositeSensorFn(dirLevel),
                leaveLimitSteps,
                leaveSteps,
                profileStepIndex,
                maxSeekSteps,
                allowPositionalOverlap)) {
            Serial.print("SHIFT: не удалось уйти с противоположного края. Лимит=");
            Serial.print(leaveLimitSteps);
            Serial.println(" шагов.");
            return false;
        }
        totalSteps += leaveSteps;
    }

    uint32_t seekSteps = 0;
    uint32_t profileStepIndex = totalSteps;
    const bool reachedEdge = shiftFindSensor(
        dirLevel,
        getShiftTargetSensorFn(dirLevel),
        maxSeekSteps,
        seekSteps,
        profileStepIndex,
        maxSeekSteps,
        allowPositionalOverlap);
    totalSteps += seekSteps;
    return reachedEdge;
}

bool runShiftMoveInternal(bool dirLevel, const char *name, bool allowPositionalOverlap)
{
    if (shiftHookIsMotionActive() || shiftHookIsCycle2Active() ||
        (shiftHookIsPositionalMotionActive() && !allowPositionalOverlap)) {
        Serial.println("SHIFT: отказ, другой двигатель уже в движении.");
        return false;
    }

    if (!g_shiftCalibrated) {
        return runFixedShiftMove(dirLevel, name, allowPositionalOverlap);
    }

    const bool targetActive = isShiftTargetSensorTriggered(dirLevel);
    const bool oppositeActive = isShiftOppositeSensorTriggered(dirLevel);
    if (targetActive && oppositeActive) {
        Serial.println("SHIFT: ошибка, одновременно сработали оба геркона.");
        return false;
    }
    if (targetActive) {
        Serial.print("SHIFT: ");
        Serial.print(name);
        Serial.println(" не требуется, каретка уже в нужном крайнем положении.");
        return true;
    }

    const uint32_t shiftNominalDelayUs = getShiftNominalDelayUs(dirLevel);
    uint32_t totalSteps = 0;

    Serial.print("SHIFT: старт ");
    Serial.print(name);
    Serial.print(", откалиброванный ход=");
    Serial.print(g_shiftTravelSteps);
    Serial.print(" шагов, задержка=");
    Serial.print(shiftNominalDelayUs);
    Serial.println(" мкс.");

    const bool reachedEdge = moveShiftToTargetEdge(dirLevel, allowPositionalOverlap, totalSteps);
    if (reachedEdge) {
        Serial.print("SHIFT: выполнено ");
        Serial.print(name);
        Serial.print(", дошли до края, шагов=");
        Serial.print(totalSteps);
        Serial.println(".");
    } else {
        const uint32_t maxSeekSteps = g_shiftTravelSteps + SHIFT_CAL_MOVE_MARGIN_STEPS;
        Serial.print("SHIFT: край не найден, остановка по лимиту. Шагов=");
        Serial.print(totalSteps);
        Serial.print(", лимит=");
        Serial.print(maxSeekSteps);
        Serial.println(".");
    }

    return reachedEdge;
}

} // namespace

void shiftInitHardwarePins()
{
    pinMode(PIN_SHIFT_DIR, OUTPUT);
    pinMode(PIN_SHIFT_PUL, OUTPUT);
    pinMode(PIN_SHIFT_SENSOR_Z, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_SENSOR_C, INPUT_PULLDOWN);
}

void shiftInitHardwareStates()
{
    digitalWrite(PIN_SHIFT_DIR, SHIFT_DIR_C_LEVEL);
    writeShiftPulseInactive();
}

void shiftUpdateSensorFilters()
{
    updateDebouncedSensorFilter(
        g_shiftSensorZFilter,
        readShiftSensorZRaw(),
        SHIFT_SENSOR_DEBOUNCE_MS);
    updateDebouncedSensorFilter(
        g_shiftSensorCFilter,
        readShiftSensorCRaw(),
        SHIFT_SENSOR_DEBOUNCE_MS);
}

bool shiftIsSensorZTriggered()
{
    shiftUpdateSensorFilters();
    return g_shiftSensorZFilter.stablePlateDetected;
}

bool shiftIsSensorCTriggered()
{
    shiftUpdateSensorFilters();
    return g_shiftSensorCFilter.stablePlateDetected;
}

bool shiftIsCalibrated()
{
    return g_shiftCalibrated;
}

void shiftCalibrateTravel()
{
    if (shiftHookIsMotionActive() || shiftHookIsCycle2Active() || shiftHookIsPositionalMotionActive()) {
        Serial.println("SHIFT: отказ, другой двигатель уже в движении.");
        return;
    }

    Serial.println("SHIFT: CZ калибровка старт.");

    const bool zActive = shiftIsSensorZTriggered();
    const bool cActive = shiftIsSensorCTriggered();
    if (zActive && cActive) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, одновременно сработали Z и C. Проверь герконы.");
        return;
    }

    if (zActive) {
        Serial.println("SHIFT: каретка уже на краю Z.");
    } else {
        if (cActive) {
            uint32_t leaveCSteps = 0;
            if (!leaveShiftSensorWithFreshProfile(
                    SHIFT_DIR_Z_LEVEL,
                    shiftIsSensorCTriggered,
                    getShiftLeaveLimitSteps(),
                    getShiftLeaveLimitSteps(),
                    false,
                    leaveCSteps)) {
                g_shiftCalibrated = false;
                Serial.println("SHIFT: ошибка, не удалось уйти с края C.");
                return;
            }
            Serial.println("SHIFT: ушли с края C, ищем Z.");
        } else {
            Serial.println("SHIFT: старт между краями, ищем край Z.");
        }

        uint32_t stepsToZ = 0;
        if (!findShiftSensorWithFreshProfile(
                SHIFT_DIR_Z_LEVEL,
                shiftIsSensorZTriggered,
                SHIFT_CAL_SEARCH_MAX_STEPS,
                SHIFT_CAL_SEARCH_MAX_STEPS,
                false,
                stepsToZ)) {
            g_shiftCalibrated = false;
            Serial.println("SHIFT: ошибка, край Z не найден.");
            return;
        }
    }

    Serial.println("SHIFT: край Z найден.");

    uint32_t leaveZSteps = 0;
    if (!leaveShiftSensorWithFreshProfile(
            SHIFT_DIR_C_LEVEL,
            shiftIsSensorZTriggered,
            getShiftLeaveLimitSteps(),
            getShiftLeaveLimitSteps(),
            false,
            leaveZSteps)) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, не удалось уйти с края Z.");
        return;
    }

    uint32_t stepsToC = 0;
    if (!findShiftSensorWithFreshProfile(
            SHIFT_DIR_C_LEVEL,
            shiftIsSensorCTriggered,
            SHIFT_CAL_SEARCH_MAX_STEPS,
            SHIFT_CAL_SEARCH_MAX_STEPS,
            false,
            stepsToC)) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, край C не найден.");
        return;
    }

    g_shiftTravelSteps = leaveZSteps + stepsToC;
    if (g_shiftTravelSteps == 0) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, ход получился 0 шагов.");
        return;
    }

    g_shiftCalibrated = true;

    Serial.println("SHIFT: край C найден.");
    Serial.print("SHIFT: калибровка завершена. Ход=");
    Serial.print(g_shiftTravelSteps);
    Serial.println(" шагов.");
    Serial.println("SHIFT: каретка сейчас в крайнем положении C.");
}

void shiftRunMoveCommandC()
{
    (void)runShiftMoveInternal(SHIFT_DIR_C_LEVEL, "C (сдвиг)", false);
}

void shiftRunMoveCommandZ()
{
    (void)runShiftMoveInternal(SHIFT_DIR_Z_LEVEL, "Z (возврат)", false);
}

bool shiftRunProgramStageC()
{
    return runShiftMoveInternal(SHIFT_DIR_C_LEVEL, "C (сдвиг)", true);
}

bool shiftRunProgramStageZ()
{
    return runShiftMoveInternal(SHIFT_DIR_Z_LEVEL, "Z (возврат)", true);
}

void homeShiftToZOnStartup()
{
    if (shiftHookIsMotionActive() || shiftHookIsCycle2Active() || shiftHookIsPositionalMotionActive()) {
        Serial.println("SHIFT: стартовая привязка пропущена, система занята.");
        return;
    }

    g_shiftCalibrated = true;
    g_shiftTravelSteps = SHIFT_TRAVEL_STEPS_FIXED;

    const bool zActive = shiftIsSensorZTriggered();
    const bool cActive = shiftIsSensorCTriggered();
    if (zActive && cActive) {
        Serial.println("SHIFT: стартовая привязка невозможна, одновременно активны Z и C.");
        return;
    }

    const uint32_t homeLimitSteps = getShiftLeaveLimitSteps();

    if (zActive) {
        Serial.println("SHIFT: стартовая привязка, каретка уже на Z. Отходим и заново ловим Z.");

        uint32_t leaveSteps = 0;
        uint32_t leaveProfileStep = 0;
        if (!shiftLeaveSensor(
                SHIFT_DIR_C_LEVEL,
                shiftIsSensorZTriggered,
                homeLimitSteps,
                leaveSteps,
                leaveProfileStep,
                homeLimitSteps,
                false)) {
            Serial.println("SHIFT: ошибка стартовой привязки, не удалось уйти с Z.");
            return;
        }
    } else {
        Serial.println("SHIFT: стартовая привязка, идем к Z.");
    }

    uint32_t seekSteps = 0;
    uint32_t seekProfileStep = 0;
    if (!shiftFindSensor(
            SHIFT_DIR_Z_LEVEL,
            shiftIsSensorZTriggered,
            homeLimitSteps,
            seekSteps,
            seekProfileStep,
            homeLimitSteps,
            false)) {
        Serial.println("SHIFT: ошибка стартовой привязки, Z не найден.");
        return;
    }

    Serial.print("SHIFT: стартовая привязка завершена. Z найден, ход=");
    Serial.print(g_shiftTravelSteps);
    Serial.println(" шагов.");
}
