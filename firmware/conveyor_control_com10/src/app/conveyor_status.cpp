#include "app/conveyor_status.h"

namespace app {

namespace {

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

constexpr uint16_t FLAG_STATE_DOWN = 1U;
constexpr uint16_t FLAG_STATE_UP = 2U;

constexpr uint16_t MOTION_STATE_IDLE = 0U;
constexpr uint16_t MOTION_STATE_RUNNING = 1U;
constexpr uint16_t MOTION_STATE_POSITIONING = 2U;

uint16_t getMotionState(const ConveyorStatusInputs &inputs)
{
    if (inputs.infeed.runState == groups::infeed::InfeedRunState::Positioning) {
        return MOTION_STATE_POSITIONING;
    }
    if (inputs.infeed.runState == groups::infeed::InfeedRunState::Running ||
        inputs.outfeed.runState == groups::outfeed::OutfeedRunState::LocalCycleRunning) {
        return MOTION_STATE_RUNNING;
    }
    return MOTION_STATE_IDLE;
}

} // namespace

bool isConveyorBusy(const ConveyorStatusInputs &inputs)
{
    return inputs.infeed.busy || inputs.outfeed.busy || inputs.sealer.busy;
}

ConveyorStatusSnapshot buildConveyorStatusSnapshot(const ConveyorStatusInputs &inputs)
{
    ConveyorStatusSnapshot snapshot = {};

    const bool busy = isConveyorBusy(inputs);
    const bool conflict = inputs.sensorConflict;

    if (!conflict) {
        snapshot.statusWord |= STATUS_READY_BIT;
        snapshot.statusWord |= STATUS_SAFE_BIT;
    }
    if (busy) {
        snapshot.statusWord |= STATUS_BUSY_BIT;
    }
    if (conflict) {
        snapshot.statusWord |= STATUS_ALARM_BIT;
    }
    if (inputs.infeed.shiftCalibrated) {
        snapshot.statusWord |= STATUS_CALIBRATED_BIT;
    }
    if (inputs.program1Active) {
        snapshot.statusWord |= STATUS_PROGRAM1_ACTIVE_BIT;
    }
    if (inputs.batchReady) {
        snapshot.statusWord |= STATUS_BATCH_READY_BIT;
    }
    if (inputs.outfeed.step2Active) {
        snapshot.statusWord |= STATUS_STEP2_ACTIVE_BIT;
    }
    snapshot.statusWord |= static_cast<uint16_t>(inputs.outfeed.step2CompletionSeq) << 8;

    if (conflict) {
        snapshot.errorWord |= ERROR_SENSOR_CONFLICT_BIT;
    }
    if (!inputs.infeed.shiftCalibrated) {
        snapshot.errorWord |= ERROR_NOT_CALIBRATED_BIT;
    }

    const uint16_t flagState = inputs.flagUp ? FLAG_STATE_UP : FLAG_STATE_DOWN;
    const uint16_t motionState = getMotionState(inputs);
    snapshot.extra0 = static_cast<uint16_t>(flagState | (motionState << 8));
    snapshot.extra1 = encodeVfdTickDuration10Ms(inputs.outfeed.vfdTickDurationMs);

    return snapshot;
}

uint16_t encodeVfdTickDuration10Ms(uint32_t durationMs)
{
    uint32_t duration10Ms = (durationMs + 5U) / 10U;
    if (duration10Ms > 0xFFFFU) {
        duration10Ms = 0xFFFFU;
    }
    return static_cast<uint16_t>(duration10Ms);
}

bool isConveyorBusy(const ConveyorStatusSnapshot &snapshot)
{
    return (snapshot.statusWord & STATUS_BUSY_BIT) != 0U;
}

} // namespace app
