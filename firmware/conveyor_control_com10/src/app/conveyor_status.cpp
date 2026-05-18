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
constexpr uint16_t STATUS_OUTFEED_READY_FOR_BATCH_BIT = 1U << 15;

constexpr uint16_t ERROR_SENSOR_CONFLICT_BIT = 1U << 0;
constexpr uint16_t ERROR_NOT_CALIBRATED_BIT = 1U << 1;
constexpr uint16_t ERROR_POST7_MANUAL_RECOVERY_REQUIRED_BIT = 1U << 2;

constexpr uint16_t FLAG_STATE_DOWN = 1U;
constexpr uint16_t FLAG_STATE_UP = 2U;

constexpr uint16_t MOTION_STATE_IDLE = 0U;
constexpr uint16_t MOTION_STATE_RUNNING = 1U;
constexpr uint16_t MOTION_STATE_POSITIONING = 2U;
constexpr uint16_t EXTRA0_FLAG_MASK = 0x0003U;
constexpr uint16_t EXTRA0_SEALER_SEQ_LOW_SHIFT = 2U;
constexpr uint16_t EXTRA0_SEALER_SEQ_LOW_MASK = 0x003FU;
constexpr uint16_t EXTRA0_MOTION_SHIFT = 8U;
constexpr uint16_t EXTRA0_MOTION_MASK = 0x0003U;
constexpr uint16_t EXTRA0_SEALER_BUSY_BIT = 1U << 10;
constexpr uint16_t EXTRA0_SEALER_START_PULSE_BIT = 1U << 11;
constexpr uint16_t EXTRA0_SEALER_DONE_ACTIVE_BIT = 1U << 12;
constexpr uint16_t EXTRA0_SEALER_SEQ_HIGH_SHIFT = 13U;
constexpr uint16_t EXTRA0_SEALER_SEQ_HIGH_MASK = 0x0003U;
constexpr uint16_t EXTRA0_VFD_TIMED_RUN_ACTIVE_BIT = 1U << 15;

uint16_t getMotionState(const ConveyorStatusInputs &inputs)
{
    if (inputs.infeed.runState == groups::infeed::InfeedRunState::Positioning) {
        return MOTION_STATE_POSITIONING;
    }
    if (inputs.infeed.runState == groups::infeed::InfeedRunState::Running ||
        inputs.outfeed.runState == groups::outfeed::OutfeedRunState::Step2Running ||
        inputs.outfeed.runState == groups::outfeed::OutfeedRunState::MainConveyorRunning ||
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
    const bool alarm =
        conflict ||
        inputs.post7ManualRecoveryRequired ||
        inputs.program1AbortRecoveryRequired;

    if (!alarm) {
        snapshot.statusWord |= STATUS_READY_BIT;
        snapshot.statusWord |= STATUS_SAFE_BIT;
    }
    if (busy) {
        snapshot.statusWord |= STATUS_BUSY_BIT;
    }
    if (alarm) {
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
    snapshot.statusWord |= static_cast<uint16_t>(inputs.outfeed.step2CompletionSeq & 0x7FU) << 8;
    if (inputs.outfeed.readyForBatch) {
        snapshot.statusWord |= STATUS_OUTFEED_READY_FOR_BATCH_BIT;
    }

    if (conflict) {
        snapshot.errorWord |= ERROR_SENSOR_CONFLICT_BIT;
    }
    if (!inputs.infeed.shiftCalibrated) {
        snapshot.errorWord |= ERROR_NOT_CALIBRATED_BIT;
    }
    if (inputs.post7ManualRecoveryRequired) {
        snapshot.errorWord |= ERROR_POST7_MANUAL_RECOVERY_REQUIRED_BIT;
    }

    const uint16_t flagState = inputs.flagUp ? FLAG_STATE_UP : FLAG_STATE_DOWN;
    const uint16_t motionState = getMotionState(inputs);
    const uint16_t sealerSeq = static_cast<uint16_t>(inputs.sealer.completionSeq);
    snapshot.extra0 = static_cast<uint16_t>(flagState & EXTRA0_FLAG_MASK);
    snapshot.extra0 |= static_cast<uint16_t>((sealerSeq & EXTRA0_SEALER_SEQ_LOW_MASK)
                                             << EXTRA0_SEALER_SEQ_LOW_SHIFT);
    snapshot.extra0 |= static_cast<uint16_t>((motionState & EXTRA0_MOTION_MASK)
                                             << EXTRA0_MOTION_SHIFT);
    snapshot.extra0 |= static_cast<uint16_t>(((sealerSeq >> 6) & EXTRA0_SEALER_SEQ_HIGH_MASK)
                                             << EXTRA0_SEALER_SEQ_HIGH_SHIFT);
    if (inputs.sealer.busy) {
        snapshot.extra0 |= EXTRA0_SEALER_BUSY_BIT;
    }
    if (inputs.sealer.startPulseActive) {
        snapshot.extra0 |= EXTRA0_SEALER_START_PULSE_BIT;
    }
    if (inputs.sealer.doneInputActive) {
        snapshot.extra0 |= EXTRA0_SEALER_DONE_ACTIVE_BIT;
    }
    if (inputs.outfeed.vfdTimedRunActive) {
        snapshot.extra0 |= EXTRA0_VFD_TIMED_RUN_ACTIVE_BIT;
    }
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
