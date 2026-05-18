#include "app/firmware.h"

#include <Arduino.h>
#include <Preferences.h>

#include "app/cli_handler.h"
#include "app/board_monitor.h"
#include "app/conveyor_status_runtime.h"
#include "app/cycle_control.h"
#include "app/manual_runtime.h"
#include "app/motion_control.h"
#include "app/pause_contract.h"
#include "app/post7_supervisor.h"
#include "comm/i2c_bus.h"
#include "core/pins.h"
#include "core/settings.h"
#include "groups/infeed/infeed_group.h"
#include "groups/outfeed/outfeed_group.h"
#include "groups/sealer/sealer_group.h"
#include "legacy/program1.h"
#include "legacy/shift_control.h"

constexpr char FW_VERSION[] = "v1.17";
#if defined(DEVICE_ROLE_CONVEYOR)
constexpr char DEVICE_NAME[] = "Управление конвейерами";
#else
constexpr char DEVICE_NAME[] = "Conveyor controller";
#endif

Preferences g_preferences;
Preferences g_settingsPreferences;
bool g_program1BufferReady = false;
bool g_program1StorageReady = false;
bool g_settingsStorageReady = false;

bool cliIsOtvodCycleActive()
{
    return groups::outfeed::isOtvodCycleActive();
}

bool cliIsMainMotionActive()
{
    return motionMainIsActive();
}

bool cliIsPositionalMotionActive()
{
    return motionPositionalIsActive();
}

bool cliIsStep2MotionActive()
{
    return groups::outfeed::isStep2Active();
}

bool cliIsConveyorBusy()
{
    return app::isConveyorBusy(app::readConveyorStatusSnapshot());
}

void manualStartConstantMotion(uint32_t steps, uint32_t delayUs)
{
    app::invalidateConveyorFeedSideModel("manual D move");
    startConstantMotion(steps, delayUs);
}

void manualStartPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    app::invalidateConveyorFeedSideModel("manual P move");
    startPositionalProfiledMotion(stepsTotal, nominalDelayUs);
}

void manualStartCycle2()
{
    app::invalidateConveyorFeedSideModel("manual C2 start");
    startCycle2();
}

void manualShiftCalibrateTravel()
{
    app::invalidateConveyorFeedSideModel("manual CZ calibration");
    shiftCalibrateTravel();
}

void manualShiftRunMoveCommandC()
{
    app::invalidateConveyorFeedSideModel("manual shift C");
    shiftRunMoveCommandC();
}

void manualShiftRunMoveCommandZ()
{
    app::invalidateConveyorFeedSideModel("manual shift Z");
    shiftRunMoveCommandZ();
}

void manualStartMainContinuousMotion(uint32_t delayUs)
{
    app::invalidateConveyorFeedSideModel("manual MAINSTART");
    startMainContinuousMotion(delayUs);
}

void manualStartPositionalContinuousMotion(uint32_t delayUs)
{
    app::invalidateConveyorFeedSideModel("manual POSSTART");
    startPositionalContinuousMotion(delayUs);
}

void manualSetFlagUp()
{
    digitalWrite(PIN_FLAG, core::hw_config::FLAG_UP_LEVEL);
}

void manualSetFlagDown()
{
    digitalWrite(PIN_FLAG, core::hw_config::FLAG_DOWN_LEVEL);
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
        groups::outfeed::setVfdTickDurationMs(groups::outfeed::getVfdTickDefaultDurationMs());
        return;
    }

    uint32_t savedDurationMs = g_settingsPreferences.getUInt(
        "vfdTickMs", groups::outfeed::getVfdTickDefaultDurationMs());
    if (savedDurationMs < groups::outfeed::getVfdTickMinDurationMs() ||
        savedDurationMs > groups::outfeed::getVfdTickMaxDurationMs()) {
        savedDurationMs = groups::outfeed::getVfdTickDefaultDurationMs();
    }

    groups::outfeed::setVfdTickDurationMs(savedDurationMs);
}

void printVfdTickSetting()
{
    Serial.print("Прокрутка основного конвейера за 1 такт: ");
    Serial.print(static_cast<float>(groups::outfeed::getVfdTickDurationMs()) / 1000.0f, 2);
    Serial.println(" сек.");
}

void saveVfdTickSetting()
{
    if (!g_settingsStorageReady) {
        Serial.println("VFDTICK: запись недоступна, storage не открыт.");
        return;
    }

    g_settingsPreferences.putUInt("vfdTickMs", groups::outfeed::getVfdTickDurationMs());
    Serial.print("VFDTICK: сохранено ");
    Serial.print(static_cast<float>(groups::outfeed::getVfdTickDurationMs()) / 1000.0f, 2);
    Serial.println(" сек.");
}

const char *sealerRunStateToText(groups::sealer::SealerRunState state)
{
    switch (state) {
        case groups::sealer::SealerRunState::Idle:
            return "idle";
        case groups::sealer::SealerRunState::StartPulseActive:
            return "start_pulse";
        case groups::sealer::SealerRunState::WaitDone:
            return "wait_done";
    }
    return "unknown";
}

const char *sealerLastCompletionToText(const groups::sealer::SealerStatus &status)
{
    if (status.completionSeq == 0U) {
        return "none";
    }
    return status.lastCompletionSynthetic ? "synthetic" : "physical";
}

const char *sealerWaitDoneBlockReasonToText(groups::sealer::SealerWaitDoneBlockReason reason)
{
    switch (reason) {
        case groups::sealer::SealerWaitDoneBlockReason::None:
            return "none";
        case groups::sealer::SealerWaitDoneBlockReason::NotWaitingDone:
            return "not_waiting_done";
        case groups::sealer::SealerWaitDoneBlockReason::WaitingDoneSignalInactive:
            return "waiting_done_inactive";
        case groups::sealer::SealerWaitDoneBlockReason::WaitingDoneSyntheticPending:
            return "waiting_done_synth_pending";
        case groups::sealer::SealerWaitDoneBlockReason::WaitingDoneSignalAlreadyActiveNoEdge:
            return "waiting_done_active_no_new_edge";
    }
    return "unknown";
}

const char *logicLevelToText(bool levelHigh)
{
    return levelHigh ? "HIGH" : "LOW";
}

void printSealStatus()
{
    const groups::sealer::SealerStatus sealerStatus = groups::sealer::readSealerStatus();
    Serial.print("SEAL: run_state=");
    Serial.print(sealerRunStateToText(sealerStatus.runState));
    Serial.print(", start_out=");
    Serial.print(sealerStatus.startOutputActive ? "on" : "off");
    Serial.print(", done=");
    Serial.print(sealerStatus.doneInputActive ? "active" : "inactive");
    Serial.print(", done_raw=");
    Serial.print(sealerStatus.doneRawInputActive ? "active" : "inactive");
    Serial.print(", done_filtered=");
    Serial.print(sealerStatus.doneFilteredInputActive ? "active" : "inactive");
    Serial.print(", done_pin_level=");
    Serial.print(logicLevelToText(sealerStatus.donePinLevelHigh));
    Serial.print(", done_active_level=");
    Serial.print(sealerStatus.doneActiveLevelLow ? "LOW" : "HIGH");
    Serial.print(", done_filter_ms=");
    Serial.print(sealerStatus.doneFilterDebounceMs);
    Serial.print(", pulse_ms=");
    Serial.print(sealerStatus.startPulseDurationMs);
    Serial.print(", wait_done=");
    Serial.print(sealerStatus.waitDoneActive ? "yes" : "no");
    Serial.print(", wait_age_ms=");
    Serial.print(sealerStatus.waitDoneAgeMs);
    Serial.print(", wait_hb_seq=");
    Serial.print(sealerStatus.waitDoneHeartbeatSeq);
    Serial.print(", completion_blocked=");
    Serial.print(sealerStatus.completionBlocked ? "yes" : "no");
    Serial.print(", block_reason=");
    Serial.print(sealerWaitDoneBlockReasonToText(sealerStatus.waitDoneBlockReason));
    Serial.print(", completion_seq=");
    Serial.print(sealerStatus.completionSeq);
    Serial.print(", last_completion=");
    Serial.print(sealerLastCompletionToText(sealerStatus));
    Serial.print(", last_completion_ms=");
    Serial.print(sealerStatus.lastCompletionMs);
    Serial.print(", edges_raw=");
    Serial.print(sealerStatus.doneRawRiseCount);
    Serial.print("/");
    Serial.print(sealerStatus.doneRawFallCount);
    Serial.print(", edges_effective=");
    Serial.print(sealerStatus.doneEffectiveRiseCount);
    Serial.print("/");
    Serial.print(sealerStatus.doneEffectiveFallCount);
    Serial.print(", raw_last_rise_ms=");
    Serial.print(sealerStatus.doneRawLastRiseMs);
    Serial.print(", raw_last_fall_ms=");
    Serial.print(sealerStatus.doneRawLastFallMs);
    Serial.print(", eff_last_rise_ms=");
    Serial.print(sealerStatus.doneEffectiveLastRiseMs);
    Serial.print(", eff_last_fall_ms=");
    Serial.print(sealerStatus.doneEffectiveLastFallMs);
    Serial.print(", ignored_rise_outside_wait=");
    Serial.print(sealerStatus.ignoredRiseOutsideWaitDoneCount);
    Serial.print(", ignored_last_ms=");
    Serial.print(sealerStatus.ignoredRiseOutsideWaitDoneLastMs);
    Serial.print(", emu_pending=");
    Serial.print(sealerStatus.doneSyntheticPending ? "yes" : "no");
    Serial.print(", emu_hold=");
    Serial.print(sealerStatus.doneSyntheticHoldActive ? "yes" : "no");
    Serial.print(", emu_remaining_ms=");
    Serial.print(sealerStatus.doneSyntheticRemainingMs);
    Serial.print(", emu_delay_ms=");
    Serial.print(sealerStatus.doneSyntheticDelayMs);
    Serial.print(", emu_hold_ms=");
    Serial.print(sealerStatus.doneSyntheticHoldMs);
    Serial.print(", start_pin=");
    Serial.print(PIN_RELAY_2);
    Serial.print(", done_pin=");
    Serial.println(PIN_SENSOR_EXT_1);
}

void printSealStartPulseStarted(uint32_t pulseMs)
{
    Serial.print("SEAL START: pulse on GPIO");
    Serial.print(PIN_RELAY_2);
    Serial.print(" for ");
    Serial.print(pulseMs);
    Serial.println(" ms.");
}

bool program1IsShiftCalibrated()
{
    return shiftIsCalibrated();
}

bool program1IsSystemBusy()
{
    return motionMainIsActive() || cycle2IsActive() || motionPositionalIsActive();
}

bool program1IsCycle2Active()
{
    return cycle2IsActive();
}

bool program1IsManualMotionActive()
{
    return motionMainIsActive();
}

bool program1IsPositionalMotionActive()
{
    return motionPositionalIsActive();
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
    startPositionalProfiledMotion(core::g_settings.c3CommandSteps, core::g_settings.c3CommandDelayUs);
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

void shiftHookProcessI2cBus()
{
    comm::processI2cBus();
}

void shiftHookProcessOutfeedGroup()
{
    groups::outfeed::processOutfeedGroup();
}

void shiftHookProcessSealerGroup()
{
    groups::sealer::processSealerGroup();
}

void shiftHookProcessPost7Supervisor()
{
    app::post7::processPost7Supervisor();
}

bool shiftHookIsMotionActive()
{
    return motionMainIsActive();
}

bool shiftHookIsCycle2Active()
{
    return cycle2IsActive();
}

bool shiftHookIsPositionalMotionActive()
{
    return motionPositionalIsActive();
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
    digitalWrite(PIN_STEP_PUL, core::hw_config::PULSE_ACTIVE_LEVEL ? LOW : HIGH);
    shiftInitHardwareStates();
    digitalWrite(PIN_FLAG, core::hw_config::FLAG_DOWN_LEVEL);
    digitalWrite(PIN_POS_PUL, core::hw_config::PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void initRuntimeState()
{
    initProgram1Storage();
    initSettingsStorage();
    updateSensorFilter();
    shiftUpdateSensorFilters();
    homeShiftToZOnStartup();
}

void registerManualRuntimeCallbacks()
{
    app::manual::Callbacks callbacks = {};
    callbacks.isOtvodCycleActive = cliIsOtvodCycleActive;
    callbacks.isConveyorBusy = cliIsConveyorBusy;
    callbacks.isMainMotionActive = cliIsMainMotionActive;
    callbacks.isPositionalMotionActive = cliIsPositionalMotionActive;
    callbacks.isStep2MotionActive = cliIsStep2MotionActive;
    callbacks.toggleSensorStream = cliToggleSensorStream;
    callbacks.printSealStatus = printSealStatus;
    callbacks.printSealStartPulseStarted = printSealStartPulseStarted;
    callbacks.printVfdTickSetting = printVfdTickSetting;
    callbacks.saveVfdTickSetting = saveVfdTickSetting;
    callbacks.startConstantMotion = manualStartConstantMotion;
    callbacks.startPositionalProfiledMotion = manualStartPositionalProfiledMotion;
    callbacks.startProgram1 = startProgram1;
    callbacks.startCycle2 = manualStartCycle2;
    callbacks.shiftCalibrateTravel = manualShiftCalibrateTravel;
    callbacks.shiftRunMoveCommandC = manualShiftRunMoveCommandC;
    callbacks.shiftRunMoveCommandZ = manualShiftRunMoveCommandZ;
    callbacks.startMainContinuousMotion = manualStartMainContinuousMotion;
    callbacks.stopMotion = stopMotion;
    callbacks.startPositionalContinuousMotion = manualStartPositionalContinuousMotion;
    callbacks.stopPositionalMotion = stopPositionalMotion;
    callbacks.setFlagUp = manualSetFlagUp;
    callbacks.setFlagDown = manualSetFlagDown;
    app::manual::registerCallbacks(callbacks);
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
    comm::printI2cBanner();
    Serial.println("Сдвиг по умолчанию: фиксированный ход 2255 шагов.");
    Serial.print("P1 buffer: ");
    Serial.println(g_program1BufferReady ? "есть 2 тарелки." : "пусто.");
    Serial.print("Реле частотника: GPIO21, активный уровень=");
    Serial.println(groups::outfeed::getVfdRelayActiveLevel() == HIGH ? "HIGH" : "LOW");
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

void firmwareSetup()
{
    Serial.begin(115200);
    delay(300);

    initHardwarePins();
    initHardwareStates();
    groups::infeed::initInfeedGroup();
    groups::outfeed::initOutfeedGroup();
    groups::sealer::initSealerGroup();
    app::post7::initPost7Supervisor();
    comm::initI2cBus();
    initRuntimeState();
    registerManualRuntimeCallbacks();
    printStartupBanner();
}

void firmwareLoop()
{
    updateSensorFilter();
    shiftUpdateSensorFilters();
    comm::processI2cBus();
    readSerialCommands();
    app::pauseContractTick();

    processCycle2();
    processMotion();
    processPositionalMotion();
    processProgram1();
    processSensorStream();
    groups::infeed::processInfeedGroup();
    groups::outfeed::processOutfeedGroup();
    groups::sealer::processSealerGroup();
    app::post7::processPost7Supervisor();
    app::processBoardMonitor();
}
