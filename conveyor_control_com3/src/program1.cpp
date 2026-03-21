#include <Arduino.h>

#include "program1.h"

namespace {

constexpr uint8_t PROGRAM1_MAX_METRICS = 12;

enum class Program1State : uint8_t {
    Idle,
    RunPass,
    WaitCycle2Done,
    WaitFinalPositionalDone,
    WaitBufferFillC2Done
};

enum class Program1MetricKind : uint8_t {
    Cycle2,
    ShiftC,
    Pos3,
    ShiftZ,
    BufferFill
};

struct Program1Metric {
    Program1MetricKind kind = Program1MetricKind::Cycle2;
    uint8_t passIndex = 0;
    uint32_t startMs = 0;
    uint32_t durationMs = 0;
    bool active = false;
    bool finished = false;
};

Program1State g_program1State = Program1State::Idle;
uint32_t g_program1StartMs = 0;
uint8_t g_program1PassIndex = 0;
bool g_program1StartedFromBuffer = false;
Program1Metric g_program1Metrics[PROGRAM1_MAX_METRICS] = {};
uint8_t g_program1MetricCount = 0;
int8_t g_program1ActiveC2Metric = -1;
int8_t g_program1ActivePosMetric = -1;

void resetProgram1Metrics()
{
    for (uint8_t i = 0; i < PROGRAM1_MAX_METRICS; i++) {
        g_program1Metrics[i] = Program1Metric();
    }
    g_program1MetricCount = 0;
    g_program1ActiveC2Metric = -1;
    g_program1ActivePosMetric = -1;
}

void printProgram1MetricLabel(const Program1Metric &metric)
{
    switch (metric.kind) {
        case Program1MetricKind::Cycle2:
            Serial.print("2 #");
            break;
        case Program1MetricKind::ShiftC:
            Serial.print("C #");
            break;
        case Program1MetricKind::Pos3:
            Serial.print("3 #");
            break;
        case Program1MetricKind::ShiftZ:
            Serial.print("Z #");
            break;
        case Program1MetricKind::BufferFill:
            Serial.print("BUF #");
            break;
        default:
            Serial.print("? #");
            break;
    }
    Serial.print(metric.passIndex);
}

int8_t beginProgram1Metric(Program1MetricKind kind, uint8_t passIndex)
{
    if (g_program1MetricCount >= PROGRAM1_MAX_METRICS) {
        Serial.println("P1: overflow metrics.");
        return -1;
    }

    Program1Metric &metric = g_program1Metrics[g_program1MetricCount];
    metric.kind = kind;
    metric.passIndex = passIndex;
    metric.startMs = millis();
    metric.durationMs = 0;
    metric.active = true;
    metric.finished = false;

    const int8_t metricId = static_cast<int8_t>(g_program1MetricCount);
    g_program1MetricCount++;
    return metricId;
}

void finishProgram1Metric(int8_t metricId)
{
    if (metricId < 0 || metricId >= static_cast<int8_t>(g_program1MetricCount)) {
        return;
    }

    Program1Metric &metric = g_program1Metrics[metricId];
    if (!metric.active || metric.finished) {
        return;
    }

    metric.durationMs = millis() - metric.startMs;
    metric.active = false;
    metric.finished = true;
}

void updateProgram1AsyncMetrics()
{
    if (g_program1ActiveC2Metric >= 0 && !program1IsCycle2Active()) {
        finishProgram1Metric(g_program1ActiveC2Metric);
        g_program1ActiveC2Metric = -1;
    }

    if (g_program1ActivePosMetric >= 0 && !program1IsPositionalMotionActive()) {
        finishProgram1Metric(g_program1ActivePosMetric);
        g_program1ActivePosMetric = -1;
    }
}

void printProgram1Summary()
{
    uint32_t sum2Ms = 0;
    uint32_t sumCMs = 0;
    uint32_t sum3Ms = 0;
    uint32_t sumZMs = 0;
    uint32_t sumBufMs = 0;

    Serial.println("P1: ===== Summary =====");
    for (uint8_t i = 0; i < g_program1MetricCount; i++) {
        const Program1Metric &metric = g_program1Metrics[i];
        Serial.print("P1: ");
        printProgram1MetricLabel(metric);
        Serial.print(" | ");
        if (!metric.finished) {
            Serial.println("RUNNING");
            continue;
        }

        Serial.print(metric.durationMs);
        Serial.println(" мс");

        switch (metric.kind) {
            case Program1MetricKind::Cycle2:
                sum2Ms += metric.durationMs;
                break;
            case Program1MetricKind::ShiftC:
                sumCMs += metric.durationMs;
                break;
            case Program1MetricKind::Pos3:
                sum3Ms += metric.durationMs;
                break;
            case Program1MetricKind::ShiftZ:
                sumZMs += metric.durationMs;
                break;
            case Program1MetricKind::BufferFill:
                sumBufMs += metric.durationMs;
                break;
            default:
                break;
        }
    }

    const uint32_t totalProgramMs = millis() - g_program1StartMs;
    Serial.print("P1: SUM 2 | ");
    Serial.print(sum2Ms);
    Serial.println(" мс");
    Serial.print("P1: SUM C | ");
    Serial.print(sumCMs);
    Serial.println(" мс");
    Serial.print("P1: SUM 3 | ");
    Serial.print(sum3Ms);
    Serial.println(" мс");
    Serial.print("P1: SUM Z | ");
    Serial.print(sumZMs);
    Serial.println(" мс");
    Serial.print("P1: SUM BUF | ");
    Serial.print(sumBufMs);
    Serial.println(" мс");
    Serial.print("P1: TOTAL | ");
    Serial.print(totalProgramMs);
    Serial.print(" мс (");
    Serial.print(static_cast<float>(totalProgramMs) / 1000.0F, 1);
    Serial.println(" с)");
}

void startProgram1Cycle2Metric(Program1MetricKind kind, uint8_t passIndex)
{
    const int8_t metricId = beginProgram1Metric(kind, passIndex);
    program1StartCycle2();
    if (program1IsCycle2Active() && metricId >= 0) {
        g_program1ActiveC2Metric = metricId;
    } else {
        finishProgram1Metric(metricId);
    }
}

void startProgram1Cycle2(uint8_t passIndex)
{
    startProgram1Cycle2Metric(Program1MetricKind::Cycle2, passIndex);
}

void startProgram1Pos3(uint8_t passIndex)
{
    if (program1IsPositionalMotionActive()) {
        Serial.print("P1: 3 #");
        Serial.print(passIndex);
        Serial.println(" не запущена, позиционный мотор уже в движении.");
        return;
    }

    const int8_t metricId = beginProgram1Metric(Program1MetricKind::Pos3, passIndex);
    program1StartPositionalPass();
    if (program1IsPositionalMotionActive() && metricId >= 0) {
        g_program1ActivePosMetric = metricId;
    } else {
        finishProgram1Metric(metricId);
    }
}

bool runProgram1ShiftStage(bool (*runStageFn)(), Program1MetricKind kind, uint8_t passIndex)
{
    const int8_t metricId = beginProgram1Metric(kind, passIndex);
    const bool ok = runStageFn();
    finishProgram1Metric(metricId);
    return ok;
}

void stopProgram1AfterFailure(const char *stepName)
{
    Serial.print("P1: аварийная остановка на шаге ");
    Serial.print(stepName);
    Serial.println(".");
    g_program1State = Program1State::Idle;
    g_program1PassIndex = 0;
    g_program1StartedFromBuffer = false;
}

bool runProgram1PassShiftSequence(uint8_t passIndex, bool withPositional)
{
    if (!runProgram1ShiftStage(program1RunShiftStageC, Program1MetricKind::ShiftC, passIndex)) {
        stopProgram1AfterFailure("C");
        return false;
    }

    if (withPositional) {
        startProgram1Pos3(passIndex);
    }

    if (!runProgram1ShiftStage(program1RunShiftStageZ, Program1MetricKind::ShiftZ, passIndex)) {
        stopProgram1AfterFailure("Z");
        return false;
    }

    return true;
}

void startProgram1BufferFillCycle()
{
    Serial.println("P1: запускаем дополнительный 2 для наполнения буфера.");
    startProgram1Cycle2Metric(Program1MetricKind::BufferFill, 1);
    if (!program1IsCycle2Active()) {
        Serial.println("P1: ошибка, не удалось запустить буферный 2.");
        g_program1State = Program1State::Idle;
        g_program1PassIndex = 0;
        g_program1StartedFromBuffer = false;
        return;
    }
    g_program1State = Program1State::WaitBufferFillC2Done;
}

void finishProgram1Run()
{
    program1MarkBufferReady();
    Serial.println("P1: программа 1 завершена.");
    Serial.println("P1: буфер пополнен, в памяти отмечено: тарелки есть.");
    printProgram1Summary();
    g_program1State = Program1State::Idle;
    g_program1PassIndex = 0;
    g_program1StartedFromBuffer = false;
}

void printProgram1PassStartMessage(uint8_t passIndex)
{
    if (passIndex == 1 && g_program1StartedFromBuffer) {
        Serial.println("P1: буферный старт. Выполняем C, затем 3 + Z.");
        return;
    }

    if (passIndex < 3) {
        Serial.print("P1: ");
        Serial.print(passIndex);
        Serial.println("-й проход 2 завершен. Выполняем C, затем 3 + Z.");
        return;
    }

    Serial.println("P1: 3-й проход 2 завершен. Финальный C + Z.");
}

void startProgram1NextCycle2(uint8_t nextPassIndex)
{
    Serial.print("P1: шаг ");
    Serial.print(nextPassIndex);
    Serial.println("/3 -> команда 2.");
    startProgram1Cycle2(nextPassIndex);
    g_program1PassIndex = nextPassIndex;
    g_program1StartedFromBuffer = false;
    g_program1State = Program1State::WaitCycle2Done;
}

void runProgram1Pass()
{
    if (g_program1PassIndex < 1 || g_program1PassIndex > 3) {
        Serial.println("P1: ошибка, неверный номер прохода.");
        g_program1State = Program1State::Idle;
        g_program1PassIndex = 0;
        g_program1StartedFromBuffer = false;
        return;
    }

    const uint8_t passIndex = g_program1PassIndex;
    printProgram1PassStartMessage(passIndex);

    const bool withPositional = passIndex < 3;
    if (!runProgram1PassShiftSequence(passIndex, withPositional)) {
        return;
    }

    g_program1StartedFromBuffer = false;

    if (passIndex < 3) {
        startProgram1NextCycle2(passIndex + 1);
        return;
    }

    if (g_program1ActivePosMetric >= 0 || program1IsPositionalMotionActive()) {
        Serial.println("P1: ждем завершение последнего этапа 3 перед наполнением буфера.");
        g_program1State = Program1State::WaitFinalPositionalDone;
        return;
    }

    startProgram1BufferFillCycle();
}

} // namespace

void startProgram1()
{
    if (g_program1State != Program1State::Idle) {
        Serial.println("P1: программа 1 уже выполняется.");
        return;
    }

    if (!program1IsShiftCalibrated()) {
        Serial.println("P1: сначала выполни CZ, чтобы откалибровать сдвиг.");
        return;
    }

    if (program1IsSystemBusy()) {
        Serial.println("P1: отказ, система уже выполняет движение.");
        return;
    }

    Serial.println("P1: старт программы 1.");
    g_program1StartMs = millis();
    resetProgram1Metrics();
    g_program1PassIndex = 1;
    g_program1StartedFromBuffer = false;

    if (program1GetBufferReady()) {
        Serial.println("P1: буфер с 2 тарелками найден, начинаем сразу с C #1.");
        program1ConsumeBuffer();
        g_program1StartedFromBuffer = true;
        g_program1State = Program1State::RunPass;
        return;
    }

    Serial.println("P1: шаг 1/3 -> команда 2.");
    startProgram1Cycle2(1);
    g_program1State = Program1State::WaitCycle2Done;
}

void processProgram1()
{
    updateProgram1AsyncMetrics();

    if (g_program1State == Program1State::Idle) {
        return;
    }

    if (program1IsCycle2Active()) {
        return;
    }

    if (program1IsManualMotionActive()) {
        return;
    }

    switch (g_program1State) {
        case Program1State::RunPass:
        case Program1State::WaitCycle2Done:
            runProgram1Pass();
            return;

        case Program1State::WaitFinalPositionalDone:
            if (g_program1ActivePosMetric >= 0 || program1IsPositionalMotionActive()) {
                return;
            }
            startProgram1BufferFillCycle();
            return;

        case Program1State::WaitBufferFillC2Done:
            finishProgram1Run();
            return;

        case Program1State::Idle:
        default:
            return;
    }
}
