#include "groups/outfeed/outfeed_group.h"

#include <Arduino.h>

#include "core/pins.h"

namespace groups::outfeed {

void stopStep2Motion();
void setVfdRelayOutput(bool enabled);
void stopVfdTimedRun(const char *reason);
bool isStep2Active();
bool isVfdTimedRunActive();
void startStep2Motion(uint32_t steps, uint32_t delayUs);
uint8_t getStep2CompletionSeq();
uint32_t getVfdTickDurationMs();
void startVfdTimedRun(uint32_t durationMs);
void abortOtvodCycle(const String &reason, bool stopOutputs);

namespace {

constexpr bool RELAY_ACTIVE_LEVEL = LOW;        // Типовой модуль реле для ESP32
constexpr bool RELAY_INACTIVE_LEVEL = HIGH;
constexpr bool STEP2_PULSE_ACTIVE_LEVEL = HIGH;
constexpr bool STEP2_PULSE_INACTIVE_LEVEL = LOW;
constexpr uint32_t STEP2_PULSE_WIDTH_US = 10U;
constexpr uint32_t STEP2_RUN_DELAY_US = 1000U;
constexpr uint32_t VFD_RELAY_TEST_DURATION_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t VFD_TICK_DEFAULT_DURATION_MS = 2190U;
constexpr uint32_t VFD_TICK_MIN_DURATION_MS = 100U;
constexpr uint32_t VFD_TICK_MAX_DURATION_MS = 600000U;
constexpr uint8_t OTVOD_CYCLE_DEFAULT_TOTAL = 3U;
constexpr uint8_t OTVOD_CYCLE_MIN_TOTAL = 1U;
constexpr uint8_t OTVOD_CYCLE_MAX_TOTAL = 9U;
constexpr uint32_t OTVOD_CYCLE_STEP_PHASE_TIMEOUT_MS = 5000U;
constexpr uint32_t OTVOD_CYCLE_VFD_START_TIMEOUT_MS = 2000U;
constexpr uint32_t OTVOD_CYCLE_VFD_TIMEOUT_MARGIN_MS = 4000U;
constexpr uint32_t OTVOD_CYCLE_VFD_TIMEOUT_FALLBACK_MS = 15000U;
constexpr uint32_t OTVOD_CYCLE_STEP_TO_MAIN_SETTLE_MS = 250U;
constexpr uint32_t OTVOD_CYCLE_MAIN_TO_STEP_SETTLE_MS = 250U;

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
    uint32_t stepCommandSteps = 920U;
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

struct TimedRelayRunState {
    bool active = false;
    uint32_t startedMs = 0;
    uint32_t durationMs = 0;
};

struct OutfeedRuntimeState {
    bool relayOutputActive = false;
    TimedRelayRunState timedRun = {};
    bool step2Active = false;
    bool step2PulseHigh = false;
    bool step2Continuous = false;
    uint32_t step2StepsTotal = 0;
    uint32_t step2StepsDone = 0;
    uint32_t step2DelayUs = 1000U;
    uint32_t step2LastPulseStartUs = 0;
    uint8_t step2CompletionSeq = 0;
    uint32_t vfdTickDurationMs = VFD_TICK_DEFAULT_DURATION_MS;
    OtvodCycleState otvodCycle = {};
};

OutfeedRuntimeState g_outfeedRuntime;

void writeRelayOutputRaw(bool enabled)
{
    digitalWrite(PIN_RELAY_1, enabled ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    g_outfeedRuntime.relayOutputActive = enabled;
}

void writeStep2PulseInactiveRaw()
{
    digitalWrite(PIN_STEP2_PUL, STEP2_PULSE_INACTIVE_LEVEL);
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

void stopOtvodCycleOutputs()
{
    if (g_outfeedRuntime.step2Active) {
        stopStep2Motion();
    }
    if (g_outfeedRuntime.timedRun.active) {
        stopVfdTimedRun("остановка OTCYCLE");
    } else {
        setVfdRelayOutput(false);
    }
}

void finishOtvodCycle(const String &result)
{
    g_outfeedRuntime.otvodCycle.active = false;
    g_outfeedRuntime.otvodCycle.stepRunning = false;
    g_outfeedRuntime.otvodCycle.stepObservedActive = false;
    g_outfeedRuntime.otvodCycle.vfdRunning = false;
    g_outfeedRuntime.otvodCycle.vfdObservedActive = false;
    g_outfeedRuntime.otvodCycle.readyForBatch = true;
    g_outfeedRuntime.otvodCycle.phase = OtvodCyclePhase::Completed;
    g_outfeedRuntime.otvodCycle.lastEvent = result;
    Serial.println(result);
}

bool startOtvodCycleStep(uint8_t cycleIndex, uint32_t nowMs)
{
    if (isStep2Active() || isVfdTimedRunActive()) {
        Serial.println("OTCYCLE: нельзя запустить двухручейковый отвод, механика занята.");
        return false;
    }

    startStep2Motion(g_outfeedRuntime.otvodCycle.stepCommandSteps, STEP2_RUN_DELAY_US);
    if (!isStep2Active()) {
        Serial.println("OTCYCLE: запуск OTVOD не подтвердился.");
        return false;
    }

    g_outfeedRuntime.otvodCycle.stepRunning = true;
    g_outfeedRuntime.otvodCycle.stepObservedActive = false;
    g_outfeedRuntime.otvodCycle.stepRunsStarted = cycleIndex;
    g_outfeedRuntime.otvodCycle.stepCompletionSeqBase = getStep2CompletionSeq();
    g_outfeedRuntime.otvodCycle.stepStartedMs = nowMs;
    g_outfeedRuntime.otvodCycle.stepCompletedMs = 0;
    g_outfeedRuntime.otvodCycle.phase = OtvodCyclePhase::StepRunning;
    g_outfeedRuntime.otvodCycle.readyForBatch = false;
    g_outfeedRuntime.otvodCycle.lastEvent =
        "OTCYCLE: двухручейковый старт " + String(cycleIndex) + "/" + String(g_outfeedRuntime.otvodCycle.totalCycles);
    Serial.println(g_outfeedRuntime.otvodCycle.lastEvent);
    return true;
}

bool startOtvodCycleMainTick(uint8_t cycleIndex, uint32_t nowMs)
{
    const uint32_t vfdTickDurationMs = getVfdTickDurationMs();
    if (vfdTickDurationMs == 0U) {
        Serial.println("OTCYCLE: время 1 такта не настроено.");
        return false;
    }
    if (isVfdTimedRunActive() || isStep2Active()) {
        Serial.println("OTCYCLE: нельзя запустить основной отвод, механика занята.");
        return false;
    }

    startVfdTimedRun(vfdTickDurationMs);
    if (!isVfdTimedRunActive()) {
        Serial.println("OTCYCLE: запуск основного отвода не подтвердился.");
        return false;
    }

    g_outfeedRuntime.otvodCycle.vfdRunning = true;
    g_outfeedRuntime.otvodCycle.vfdObservedActive = true;
    g_outfeedRuntime.otvodCycle.vfdRunsStarted = cycleIndex;
    g_outfeedRuntime.otvodCycle.vfdStartedMs = nowMs;
    g_outfeedRuntime.otvodCycle.vfdCompletedMs = 0;
    g_outfeedRuntime.otvodCycle.vfdDoneDelayMs = vfdTickDurationMs;
    g_outfeedRuntime.otvodCycle.phase = OtvodCyclePhase::MainRunning;
    g_outfeedRuntime.otvodCycle.lastEvent =
        "OTCYCLE: основной отвод старт " + String(cycleIndex) + "/" + String(g_outfeedRuntime.otvodCycle.totalCycles);
    Serial.println(g_outfeedRuntime.otvodCycle.lastEvent);
    return true;
}

void processOtvodCycle()
{
    if (!g_outfeedRuntime.otvodCycle.active) {
        return;
    }

    const uint32_t nowMs = millis();

    if (g_outfeedRuntime.otvodCycle.stepRunning) {
        if ((uint32_t)(nowMs - g_outfeedRuntime.otvodCycle.stepStartedMs) > OTVOD_CYCLE_STEP_PHASE_TIMEOUT_MS) {
            abortOtvodCycle("OTCYCLE: ошибка, STEP2 не завершился по таймауту.", true);
            return;
        }

        const bool step2CompletedLatched =
            getStep2CompletionSeq() != g_outfeedRuntime.otvodCycle.stepCompletionSeqBase;
        if (isStep2Active()) {
            g_outfeedRuntime.otvodCycle.stepObservedActive = true;
        }

        if (step2CompletedLatched || (!isStep2Active() && g_outfeedRuntime.otvodCycle.stepObservedActive)) {
            g_outfeedRuntime.otvodCycle.stepRunning = false;
            g_outfeedRuntime.otvodCycle.stepObservedActive = false;
            g_outfeedRuntime.otvodCycle.stepRunsCompleted++;
            g_outfeedRuntime.otvodCycle.stepCompletedMs = nowMs;
            g_outfeedRuntime.otvodCycle.phase = OtvodCyclePhase::WaitMainStart;
            g_outfeedRuntime.otvodCycle.readyForBatch =
                g_outfeedRuntime.otvodCycle.stepRunsCompleted >= g_outfeedRuntime.otvodCycle.totalCycles;

            Serial.print("OTCYCLE: двухручейковый завершен ");
            Serial.print(g_outfeedRuntime.otvodCycle.stepRunsCompleted);
            Serial.print("/");
            Serial.println(g_outfeedRuntime.otvodCycle.totalCycles);

            if (g_outfeedRuntime.otvodCycle.readyForBatch) {
                g_outfeedRuntime.otvodCycle.lastEvent = "OTCYCLE: двухручейковый готов принять новую партию.";
                Serial.println(g_outfeedRuntime.otvodCycle.lastEvent);
            } else {
                g_outfeedRuntime.otvodCycle.lastEvent = "OTCYCLE: ожидание старта основного отвода.";
            }
        }
    }

    if (g_outfeedRuntime.otvodCycle.active &&
        !g_outfeedRuntime.otvodCycle.stepRunning &&
        g_outfeedRuntime.otvodCycle.stepRunsStarted > g_outfeedRuntime.otvodCycle.vfdRunsStarted &&
        !g_outfeedRuntime.otvodCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_outfeedRuntime.otvodCycle.stepCompletedMs);
        if (elapsedMs >= g_outfeedRuntime.otvodCycle.stepTriggerDelayMs) {
            if (!startOtvodCycleMainTick(g_outfeedRuntime.otvodCycle.stepRunsStarted, nowMs)) {
                abortOtvodCycle("OTCYCLE: ошибка запуска основного отвода.", true);
                return;
            }
            return;
        }
    }

    if (g_outfeedRuntime.otvodCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_outfeedRuntime.otvodCycle.vfdStartedMs);

        if (isVfdTimedRunActive()) {
            g_outfeedRuntime.otvodCycle.vfdObservedActive = true;
        } else if (g_outfeedRuntime.otvodCycle.vfdObservedActive) {
            g_outfeedRuntime.otvodCycle.vfdRunning = false;
            g_outfeedRuntime.otvodCycle.vfdObservedActive = false;
            g_outfeedRuntime.otvodCycle.vfdRunsCompleted++;
            g_outfeedRuntime.otvodCycle.vfdCompletedMs = nowMs;
            g_outfeedRuntime.otvodCycle.phase = OtvodCyclePhase::WaitNextStep;
            g_outfeedRuntime.otvodCycle.lastEvent = "OTCYCLE: основной отвод завершен " +
                String(g_outfeedRuntime.otvodCycle.vfdRunsCompleted) + "/" + String(g_outfeedRuntime.otvodCycle.totalCycles);
            Serial.println(g_outfeedRuntime.otvodCycle.lastEvent);
            return;
        }

        if (!g_outfeedRuntime.otvodCycle.vfdObservedActive && elapsedMs > OTVOD_CYCLE_VFD_START_TIMEOUT_MS) {
            abortOtvodCycle("OTCYCLE: основной отвод не стартовал по таймауту.", true);
            return;
        }

        const uint32_t timeoutMs =
            (g_outfeedRuntime.otvodCycle.vfdDoneDelayMs > 0U)
                ? (g_outfeedRuntime.otvodCycle.vfdDoneDelayMs + OTVOD_CYCLE_VFD_TIMEOUT_MARGIN_MS)
                : OTVOD_CYCLE_VFD_TIMEOUT_FALLBACK_MS;
        if (elapsedMs > timeoutMs) {
            abortOtvodCycle("OTCYCLE: основной отвод не завершился по таймауту.", true);
            return;
        }
    }

    if (g_outfeedRuntime.otvodCycle.active &&
        !g_outfeedRuntime.otvodCycle.vfdRunning &&
        !g_outfeedRuntime.otvodCycle.stepRunning &&
        g_outfeedRuntime.otvodCycle.vfdRunsCompleted < g_outfeedRuntime.otvodCycle.totalCycles &&
        g_outfeedRuntime.otvodCycle.stepRunsStarted == g_outfeedRuntime.otvodCycle.vfdRunsCompleted) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_outfeedRuntime.otvodCycle.vfdCompletedMs);
        if (elapsedMs >= g_outfeedRuntime.otvodCycle.vfdTriggerDelayMs) {
            const uint8_t nextCycleIndex = static_cast<uint8_t>(g_outfeedRuntime.otvodCycle.vfdRunsCompleted + 1U);
            if (!startOtvodCycleStep(nextCycleIndex, nowMs)) {
                abortOtvodCycle("OTCYCLE: ошибка запуска следующего прохода OTVOD.", true);
                return;
            }
        }
    }

    if (g_outfeedRuntime.otvodCycle.active &&
        !g_outfeedRuntime.otvodCycle.stepRunning &&
        !g_outfeedRuntime.otvodCycle.vfdRunning &&
        g_outfeedRuntime.otvodCycle.stepRunsCompleted >= g_outfeedRuntime.otvodCycle.totalCycles &&
        g_outfeedRuntime.otvodCycle.vfdRunsCompleted >= g_outfeedRuntime.otvodCycle.totalCycles) {
        finishOtvodCycle("OTCYCLE: завершено, циклов " + String(g_outfeedRuntime.otvodCycle.totalCycles) + ".");
    }
}

} // namespace

void initOutfeedGroup()
{
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_STEP2_PUL, OUTPUT);
    writeRelayOutputRaw(false);
    writeStep2PulseInactiveRaw();
    g_outfeedRuntime.timedRun = TimedRelayRunState{};
    g_outfeedRuntime.step2Active = false;
    g_outfeedRuntime.step2PulseHigh = false;
    g_outfeedRuntime.step2Continuous = false;
    g_outfeedRuntime.step2StepsTotal = 0;
    g_outfeedRuntime.step2StepsDone = 0;
    g_outfeedRuntime.step2DelayUs = 1000U;
    g_outfeedRuntime.step2LastPulseStartUs = 0;
    g_outfeedRuntime.otvodCycle = OtvodCycleState{};
}

void processOutfeedGroup()
{
    if (g_outfeedRuntime.step2Active) {
        const uint32_t nowUs = micros();

        if (!g_outfeedRuntime.step2PulseHigh) {
            if ((uint32_t)(nowUs - g_outfeedRuntime.step2LastPulseStartUs) >= g_outfeedRuntime.step2DelayUs) {
                digitalWrite(PIN_STEP2_PUL, STEP2_PULSE_ACTIVE_LEVEL);
                g_outfeedRuntime.step2PulseHigh = true;
                g_outfeedRuntime.step2LastPulseStartUs = nowUs;
            }
        } else if ((uint32_t)(nowUs - g_outfeedRuntime.step2LastPulseStartUs) >= STEP2_PULSE_WIDTH_US) {
            writeStep2PulseInactiveRaw();
            g_outfeedRuntime.step2PulseHigh = false;
            g_outfeedRuntime.step2StepsDone++;

            if (!g_outfeedRuntime.step2Continuous &&
                g_outfeedRuntime.step2StepsDone >= g_outfeedRuntime.step2StepsTotal) {
                g_outfeedRuntime.step2CompletionSeq =
                    static_cast<uint8_t>(g_outfeedRuntime.step2CompletionSeq + 1U);
                stopStep2Motion();
                Serial.println("STEP2: движение завершено.");
            }
        }
    }

    if (!g_outfeedRuntime.timedRun.active) {
        processOtvodCycle();
        return;
    }

    const uint32_t elapsedMs =
        static_cast<uint32_t>(millis() - g_outfeedRuntime.timedRun.startedMs);
    if (elapsedMs >= g_outfeedRuntime.timedRun.durationMs) {
        stopVfdTimedRun("таймер истек");
    }

    processOtvodCycle();
}

OutfeedStatus readOutfeedStatus()
{
    OutfeedStatus status = {};
    status.step2Active = g_outfeedRuntime.step2Active;
    status.step2Continuous = g_outfeedRuntime.step2Continuous;
    status.step2StepsTotal = g_outfeedRuntime.step2StepsTotal;
    status.step2StepsDone = g_outfeedRuntime.step2StepsDone;
    status.step2CompletionSeq = g_outfeedRuntime.step2CompletionSeq;
    status.vfdRelayActive = g_outfeedRuntime.relayOutputActive;
    status.vfdTimedRunActive = g_outfeedRuntime.timedRun.active;
    status.localCycleActive = g_outfeedRuntime.otvodCycle.active;
    status.vfdTickDurationMs = g_outfeedRuntime.vfdTickDurationMs;
    if (status.localCycleActive) {
        status.runState = OutfeedRunState::LocalCycleRunning;
    } else if (status.step2Active) {
        status.runState = OutfeedRunState::Step2Running;
    } else if (status.vfdTimedRunActive) {
        status.runState = OutfeedRunState::MainConveyorRunning;
    } else {
        status.runState = OutfeedRunState::Idle;
    }
    status.busy = status.runState != OutfeedRunState::Idle;
    status.ready = !status.busy;
    status.alarm = false;
    return status;
}

void setVfdRelayOutput(bool enabled)
{
    g_outfeedRuntime.timedRun = TimedRelayRunState{};
    writeRelayOutputRaw(enabled);
}

bool isVfdRelayOutputActive()
{
    return g_outfeedRuntime.relayOutputActive;
}

void stopVfdTimedRun(const char *reason)
{
    if (!g_outfeedRuntime.timedRun.active) {
        return;
    }

    writeRelayOutputRaw(false);
    Serial.print("VFD: реле GPIO21 выключено");
    if (reason != nullptr && reason[0] != '\0') {
        Serial.print(" (");
        Serial.print(reason);
        Serial.print(")");
    }
    Serial.println(".");
    g_outfeedRuntime.timedRun = TimedRelayRunState{};
}

void startVfdTimedRun(uint32_t durationMs)
{
    if (g_outfeedRuntime.timedRun.active) {
        stopVfdTimedRun("перезапуск");
    }

    writeRelayOutputRaw(true);
    g_outfeedRuntime.timedRun.active = true;
    g_outfeedRuntime.timedRun.startedMs = millis();
    g_outfeedRuntime.timedRun.durationMs = durationMs;

    Serial.print("VFD: реле GPIO21 включено на ");
    Serial.print(durationMs / 1000U);
    Serial.println(" сек.");
}

bool isVfdTimedRunActive()
{
    return g_outfeedRuntime.timedRun.active;
}

uint32_t getVfdTickDurationMs()
{
    return g_outfeedRuntime.vfdTickDurationMs;
}

void setVfdTickDurationMs(uint32_t durationMs)
{
    g_outfeedRuntime.vfdTickDurationMs = durationMs;
}

void stopStep2Motion()
{
    writeStep2PulseInactiveRaw();
    g_outfeedRuntime.step2Active = false;
    g_outfeedRuntime.step2PulseHigh = false;
    g_outfeedRuntime.step2Continuous = false;
}

void startStep2Motion(uint32_t steps, uint32_t delayUs)
{
    if (g_outfeedRuntime.step2Active) {
        Serial.println("STEP2: тест уже выполняется.");
        return;
    }

    if (steps == 0U) {
        Serial.println("STEP2: шагов 0, запуск не требуется.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    g_outfeedRuntime.step2Active = true;
    g_outfeedRuntime.step2PulseHigh = false;
    g_outfeedRuntime.step2Continuous = false;
    g_outfeedRuntime.step2StepsTotal = steps;
    g_outfeedRuntime.step2StepsDone = 0;
    g_outfeedRuntime.step2DelayUs = delayUs;
    g_outfeedRuntime.step2LastPulseStartUs = micros();

    Serial.print("STEP2: запуск, GPIO19, шагов=");
    Serial.print(steps);
    Serial.print(", задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

void startStep2ContinuousMotion(uint32_t delayUs)
{
    if (g_outfeedRuntime.step2Active) {
        Serial.println("STEP2: GPIO19 уже в движении.");
        return;
    }

    if (delayUs == 0U) {
        delayUs = 1U;
    }

    writeStep2PulseInactiveRaw();
    g_outfeedRuntime.step2Active = true;
    g_outfeedRuntime.step2PulseHigh = false;
    g_outfeedRuntime.step2Continuous = true;
    g_outfeedRuntime.step2StepsTotal = 0;
    g_outfeedRuntime.step2StepsDone = 0;
    g_outfeedRuntime.step2DelayUs = delayUs;
    g_outfeedRuntime.step2LastPulseStartUs = micros();

    Serial.print("STEP2: непрерывный запуск GPIO19, задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс.");
}

bool isStep2Active()
{
    return g_outfeedRuntime.step2Active;
}

uint8_t getStep2CompletionSeq()
{
    return g_outfeedRuntime.step2CompletionSeq;
}

bool startOtvodCycle(uint8_t totalCycles, uint32_t stepSteps, bool conveyorBusy)
{
    if (g_outfeedRuntime.otvodCycle.active) {
        Serial.println("OTCYCLE: уже выполняется.");
        return false;
    }

    if (totalCycles < OTVOD_CYCLE_MIN_TOTAL || totalCycles > OTVOD_CYCLE_MAX_TOTAL || stepSteps == 0U) {
        Serial.println("OTCYCLE: неверные параметры запуска.");
        return false;
    }

    if (conveyorBusy) {
        Serial.println("OTCYCLE: конвейер занят, сначала остановите текущий процесс.");
        return false;
    }

    g_outfeedRuntime.otvodCycle = OtvodCycleState{};
    g_outfeedRuntime.otvodCycle.active = true;
    g_outfeedRuntime.otvodCycle.readyForBatch = false;
    g_outfeedRuntime.otvodCycle.totalCycles = totalCycles;
    g_outfeedRuntime.otvodCycle.stepCommandSteps = stepSteps;
    g_outfeedRuntime.otvodCycle.vfdDoneDelayMs = getVfdTickDurationMs();
    g_outfeedRuntime.otvodCycle.lastEvent = "OTCYCLE init";

    Serial.print("OTCYCLE: init step=");
    Serial.print(stepSteps);
    Serial.print(", tick=");
    Serial.print(getVfdTickDurationMs());
    Serial.print(" ms, cycles=");
    Serial.println(totalCycles);

    const uint32_t nowMs = millis();
    if (!startOtvodCycleStep(1U, nowMs)) {
        g_outfeedRuntime.otvodCycle = OtvodCycleState{};
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

void abortOtvodCycle(const String &reason, bool stopOutputs)
{
    if (stopOutputs) {
        stopOtvodCycleOutputs();
    }

    g_outfeedRuntime.otvodCycle.active = false;
    g_outfeedRuntime.otvodCycle.stepRunning = false;
    g_outfeedRuntime.otvodCycle.stepObservedActive = false;
    g_outfeedRuntime.otvodCycle.vfdRunning = false;
    g_outfeedRuntime.otvodCycle.vfdObservedActive = false;
    g_outfeedRuntime.otvodCycle.readyForBatch = false;
    g_outfeedRuntime.otvodCycle.phase = OtvodCyclePhase::Aborted;
    g_outfeedRuntime.otvodCycle.lastEvent = reason;
    Serial.println(reason);
}

bool isOtvodCycleActive()
{
    return g_outfeedRuntime.otvodCycle.active;
}

void printOtvodCycleStatus()
{
    Serial.print("OTCYCLE: active=");
    Serial.print(g_outfeedRuntime.otvodCycle.active ? "yes" : "no");
    Serial.print(", phase=");
    Serial.print(otvodCyclePhaseName(g_outfeedRuntime.otvodCycle.phase));
    Serial.print(", ready=");
    Serial.print(g_outfeedRuntime.otvodCycle.readyForBatch ? "yes" : "no");
    Serial.print(", step=");
    Serial.print(g_outfeedRuntime.otvodCycle.stepRunsCompleted);
    Serial.print("/");
    Serial.print(g_outfeedRuntime.otvodCycle.stepRunsStarted);
    Serial.print("/");
    Serial.print(g_outfeedRuntime.otvodCycle.totalCycles);
    Serial.print(", main=");
    Serial.print(g_outfeedRuntime.otvodCycle.vfdRunsCompleted);
    Serial.print("/");
    Serial.print(g_outfeedRuntime.otvodCycle.vfdRunsStarted);
    Serial.print(", steps=");
    Serial.print(g_outfeedRuntime.otvodCycle.stepCommandSteps);
    Serial.print(", tick_ms=");
    Serial.print(g_outfeedRuntime.otvodCycle.vfdDoneDelayMs > 0
        ? g_outfeedRuntime.otvodCycle.vfdDoneDelayMs
        : getVfdTickDurationMs());
    Serial.print(", last=");
    Serial.println(g_outfeedRuntime.otvodCycle.lastEvent);
}

uint32_t getVfdTickDefaultDurationMs()
{
    return VFD_TICK_DEFAULT_DURATION_MS;
}

uint32_t getVfdTickMinDurationMs()
{
    return VFD_TICK_MIN_DURATION_MS;
}

uint32_t getVfdTickMaxDurationMs()
{
    return VFD_TICK_MAX_DURATION_MS;
}

uint32_t getVfdRelayTestDurationMs()
{
    return VFD_RELAY_TEST_DURATION_MS;
}

bool getVfdRelayActiveLevel()
{
    return RELAY_ACTIVE_LEVEL;
}

bool getStep2PulseActiveLevel()
{
    return STEP2_PULSE_ACTIVE_LEVEL;
}

uint32_t getStep2PulseWidthUs()
{
    return STEP2_PULSE_WIDTH_US;
}

} // namespace groups::outfeed
