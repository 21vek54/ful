#include "app/conveyor_status_runtime.h"

#include <Arduino.h>

#include "core/pins.h"
#include "groups/infeed/infeed_group.h"
#include "groups/outfeed/outfeed_group.h"
#include "groups/sealer/sealer_group.h"
#include "legacy/program1.h"
#include "legacy/shift_control.h"
#include "app/post7_supervisor.h"

namespace {

constexpr int8_t FEED_BUFFER_UNKNOWN = -1;
constexpr int8_t FEED_BUFFER_EMPTY = 0;
constexpr int8_t FEED_BUFFER_PAIR = 2;
constexpr int8_t IN2_PAIRS_UNKNOWN = -1;
constexpr int8_t IN2_PAIRS_EMPTY = 0;
constexpr int8_t IN2_PAIRS_FULL = 3;
constexpr int8_t SEALER_PLATE_COUNT_UNKNOWN = -1;
constexpr int8_t SEALER_PLATE_RESIDUAL = 6;

struct FeedSideRuntimeModel {
    bool synced = false;
    int8_t bufferCount = FEED_BUFFER_UNKNOWN;
    int8_t in2Pairs = IN2_PAIRS_UNKNOWN;
    bool program1RunActive = false;
    uint8_t lastProgramPass = 0;
};

FeedSideRuntimeModel g_feedSideModel = {};
bool g_program1AbortRecoveryRequired = false;

bool isConveyorFlagUp()
{
    return digitalRead(PIN_FLAG) == HIGH;
}

bool isConveyorSensorConflict()
{
    return shiftIsSensorZTriggered() && shiftIsSensorCTriggered();
}

bool isShiftHomeZStrict()
{
    const bool zActive = shiftIsSensorZTriggered();
    const bool cActive = shiftIsSensorCTriggered();
    return zActive && !cActive;
}

void invalidateConveyorFeedSideModelInternal(const char *reason)
{
    g_feedSideModel.synced = false;
    g_feedSideModel.bufferCount = FEED_BUFFER_UNKNOWN;
    g_feedSideModel.in2Pairs = IN2_PAIRS_UNKNOWN;
    g_feedSideModel.program1RunActive = false;
    g_feedSideModel.lastProgramPass = 0;

    if (reason != nullptr && reason[0] != '\0') {
        Serial.print("FEED STRICT: model invalidated: ");
        Serial.println(reason);
    }
}

void refreshFeedSideFlags(app::ConveyorStatusInputs &inputs)
{
    g_feedSideModel.program1RunActive = inputs.program1Active || inputs.infeed.busy;

    if (inputs.infeed.busy) {
        // While C2 is collecting plates, exact BUFFER_COUNT is not deterministic.
        g_feedSideModel.bufferCount = FEED_BUFFER_UNKNOWN;
    }

    const bool bufferPairReady = program1GetBufferReady();
    if (bufferPairReady) {
        g_feedSideModel.bufferCount = FEED_BUFFER_PAIR;
    }

    if (!bufferPairReady &&
        !g_feedSideModel.program1RunActive &&
        g_feedSideModel.bufferCount == FEED_BUFFER_PAIR) {
        invalidateConveyorFeedSideModelInternal("buffer pair flag dropped outside strict model");
    }

    const bool modelHasSnapshot =
        g_feedSideModel.synced &&
        g_feedSideModel.in2Pairs != IN2_PAIRS_UNKNOWN &&
        g_feedSideModel.bufferCount != FEED_BUFFER_UNKNOWN &&
        !inputs.sensorConflict;

    const bool strictEmpty =
        modelHasSnapshot &&
        g_feedSideModel.bufferCount == FEED_BUFFER_EMPTY &&
        g_feedSideModel.in2Pairs == IN2_PAIRS_EMPTY &&
        isShiftHomeZStrict();

    inputs.feedSideEmptyValid = modelHasSnapshot;
    inputs.feedSideEmptyStrict = strictEmpty;
    inputs.feedBufferCount = g_feedSideModel.bufferCount;
    inputs.feedBufferCountKnown = g_feedSideModel.bufferCount != FEED_BUFFER_UNKNOWN;
    inputs.feedIn2Pairs = g_feedSideModel.in2Pairs;
    inputs.feedIn2PairsKnown =
        g_feedSideModel.synced && g_feedSideModel.in2Pairs != IN2_PAIRS_UNKNOWN;
}

void refreshSealerPlateHints(app::ConveyorStatusInputs &inputs)
{
    const bool sealerResidualObserved =
        inputs.sealer.busy ||
        inputs.sealer.doneInputActive ||
        inputs.sealer.startPulseActive;

    if (sealerResidualObserved) {
        inputs.sealerPlateCount = SEALER_PLATE_RESIDUAL;
        inputs.sealerPlateCountKnown = true;
        return;
    }

    // На COM10 нет отдельного датчика "запайщик пуст/не пуст".
    // Если прямых признаков остатка нет, считаем состояние неизвестным.
    inputs.sealerPlateCount = SEALER_PLATE_COUNT_UNKNOWN;
    inputs.sealerPlateCountKnown = false;
}

} // namespace

namespace app {

ConveyorStatusInputs readConveyorStatusInputs()
{
    ConveyorStatusInputs inputs = {};
    inputs.infeed = groups::infeed::readInfeedStatus();
    inputs.outfeed = groups::outfeed::readOutfeedStatus();
    inputs.sealer = groups::sealer::readSealerStatus();
    inputs.sensorConflict = isConveyorSensorConflict();
    inputs.program1StateCode = program1GetStateCode();
    inputs.program1PassIndex = program1GetPassIndex();
    inputs.program1Active = inputs.program1StateCode != 0U;
    inputs.batchReady = program1IsBatchReadyForManipulator();
    inputs.program1AbortRecoveryRequired = g_program1AbortRecoveryRequired;
    inputs.flagUp = isConveyorFlagUp();
    inputs.post7ManualRecoveryRequired = post7::isManualRecoveryRequired();
    refreshFeedSideFlags(inputs);
    refreshSealerPlateHints(inputs);
    return inputs;
}

ConveyorStatusSnapshot readConveyorStatusSnapshot()
{
    return buildConveyorStatusSnapshot(readConveyorStatusInputs());
}

void conveyorFeedSideNoteProgram1Started(bool startedFromBuffer)
{
    if (g_feedSideModel.synced && g_feedSideModel.in2Pairs != IN2_PAIRS_EMPTY) {
        invalidateConveyorFeedSideModelInternal("program1 start with IN2 != empty");
    }

    g_feedSideModel.program1RunActive = true;
    g_feedSideModel.lastProgramPass = 0;
    g_feedSideModel.bufferCount = startedFromBuffer ? FEED_BUFFER_EMPTY : FEED_BUFFER_UNKNOWN;
}

void conveyorFeedSideNoteProgram1PassShiftCompleted(uint8_t passIndex)
{
    if (!g_feedSideModel.program1RunActive) {
        invalidateConveyorFeedSideModelInternal("pass complete without active program1");
        return;
    }

    g_feedSideModel.lastProgramPass = passIndex;
    g_feedSideModel.bufferCount = FEED_BUFFER_EMPTY;

    if (!g_feedSideModel.synced || g_feedSideModel.in2Pairs == IN2_PAIRS_UNKNOWN) {
        return;
    }

    if (g_feedSideModel.in2Pairs >= IN2_PAIRS_FULL) {
        invalidateConveyorFeedSideModelInternal("IN2 overflow on pass complete");
        return;
    }

    g_feedSideModel.in2Pairs++;
}

void conveyorFeedSideNoteBatchReadyLatched()
{
    g_feedSideModel.synced = true;
    g_feedSideModel.in2Pairs = IN2_PAIRS_FULL;
}

void conveyorFeedSideNoteProgram1Finished()
{
    g_feedSideModel.program1RunActive = false;
    g_feedSideModel.lastProgramPass = 0;
    g_feedSideModel.bufferCount = FEED_BUFFER_PAIR;
}

void conveyorFeedSideNoteProgram1Aborted()
{
    g_program1AbortRecoveryRequired = true;
    invalidateConveyorFeedSideModelInternal("program1 aborted");
}

void conveyorFeedSideNoteProgram1StoppedByPauseArm()
{
    invalidateConveyorFeedSideModelInternal("program1 stopped by pause arm");
}

void conveyorFeedSideNoteIn2Consumed()
{
    if (g_feedSideModel.program1RunActive) {
        invalidateConveyorFeedSideModelInternal("IN2 consumed while program1 active");
        return;
    }

    g_feedSideModel.synced = true;
    g_feedSideModel.in2Pairs = IN2_PAIRS_EMPTY;
}

void invalidateConveyorFeedSideModel(const char *reason)
{
    invalidateConveyorFeedSideModelInternal(reason);
}

bool conveyorProgram1AbortRecoveryRequired()
{
    return g_program1AbortRecoveryRequired;
}

void clearConveyorProgram1AbortRecoveryRequired(const char *reason)
{
    if (!g_program1AbortRecoveryRequired) {
        return;
    }

    g_program1AbortRecoveryRequired = false;
    Serial.print("P1 recovery: abort latch cleared");
    if (reason != nullptr && reason[0] != '\0') {
        Serial.print(" (");
        Serial.print(reason);
        Serial.print(")");
    }
    Serial.println(".");
}

} // namespace app
