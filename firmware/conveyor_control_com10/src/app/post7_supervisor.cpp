// Локальный supervisor post-step7 на COM10.
// Владеет production-стартом POST7 и локально оркестрирует
// связку OUTFEED + SEAL без раскрытия внутренней механики мастеру.
#include "app/post7_supervisor.h"

#include <Arduino.h>

#include "app/runtime_log.h"
#include "core/settings.h"
#include "groups/outfeed/outfeed_group.h"
#include "groups/sealer/sealer_group.h"

namespace app::post7 {

namespace {

struct Post7RuntimeState {
    bool active = false;
    bool expectsSealerCompletion = false;
    bool outfeedActivityObserved = false;
    bool manualRecoveryRequired = false;
    bool sealStartGuardPending = false;
    uint8_t outfeedStep2SeqBase = 0;
    uint8_t sealerCompletionSeqBase = 0;
    uint32_t startedMs = 0;
    uint32_t finishedMs = 0;
    uint32_t lastSealWaitDiagMs = 0;
    Post7Mode mode = Post7Mode::None;
    Post7Phase phase = Post7Phase::None;
    String lastEvent = "idle";
};

Post7RuntimeState g_post7Runtime = {};
constexpr uint32_t POST7_SEAL_WAIT_DIAG_PERIOD_MS = 3000U;

const char *post7ModeToText(Post7Mode mode)
{
    switch (mode) {
        case Post7Mode::Load:
            return "LOAD";
        case Post7Mode::UnloadOnly:
            return "UNLOAD_ONLY";
        case Post7Mode::None:
        default:
            return "NONE";
    }
}

const char *post7PhaseToText(Post7Phase phase)
{
    switch (phase) {
        case Post7Phase::PreSeal:
            return "pre-seal";
        case Post7Phase::SealInFlight:
            return "seal-in-flight";
        case Post7Phase::SealCompletedOutfeedPending:
            return "seal-completed-outfeed-pending";
        case Post7Phase::OutfeedOnly:
            return "outfeed-only";
        case Post7Phase::None:
        default:
            return "none";
    }
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

bool isSealCommitReachedPhase(Post7Phase phase)
{
    return phase == Post7Phase::SealInFlight ||
        phase == Post7Phase::SealCompletedOutfeedPending;
}

bool isStopRecoveryZone(Post7Phase phase)
{
    return phase == Post7Phase::SealInFlight;
}

Post7Phase resolvePost7PhaseForActiveJob(const groups::sealer::SealerStatus &sealerStatus)
{
    if (!g_post7Runtime.active) {
        return Post7Phase::None;
    }
    if (!g_post7Runtime.expectsSealerCompletion) {
        return Post7Phase::OutfeedOnly;
    }
    if (g_post7Runtime.phase == Post7Phase::PreSeal) {
        return Post7Phase::PreSeal;
    }

    const bool sealerCompletionAdvanced =
        sealerStatus.completionSeq != g_post7Runtime.sealerCompletionSeqBase;
    if (sealerCompletionAdvanced) {
        return Post7Phase::SealCompletedOutfeedPending;
    }
    return Post7Phase::SealInFlight;
}

bool startPost7Job(Post7Mode mode)
{
    if (mode != Post7Mode::Load && mode != Post7Mode::UnloadOnly) {
        Serial.println("POST7: invalid mode.");
        return false;
    }

    if (g_post7Runtime.active) {
        Serial.println("POST7: already active.");
        return false;
    }
    if (g_post7Runtime.manualRecoveryRequired) {
        Serial.println("POST7: rejected, manual recovery is required.");
        return false;
    }

    const groups::outfeed::OutfeedStatus outfeedStatus = groups::outfeed::readOutfeedStatus();
    const groups::sealer::SealerStatus sealerStatus = groups::sealer::readSealerStatus();
    if (outfeedStatus.localCycleActive) {
        Serial.println("POST7: rejected, OTCYCLE is already active.");
        return false;
    }

    if (mode == Post7Mode::Load) {
        if (sealerStatus.startPulseActive || sealerStatus.waitDoneActive) {
            Serial.println("POST7 LOAD: rejected, SEAL is already in progress.");
            return false;
        }
    }

    const uint8_t totalCycles = core::g_settings.otvodCycleDefaultTotal;
    const uint32_t stepSteps = core::g_settings.step2DivertSteps;
    if (!groups::outfeed::startOtvodCycle(totalCycles, stepSteps)) {
        Serial.println("POST7: OTCYCLE start failed.");
        return false;
    }

    const bool expectsSealerCompletion = (mode == Post7Mode::Load);

    Post7RuntimeState nextState = {};
    nextState.active = true;
    nextState.mode = mode;
    nextState.expectsSealerCompletion = expectsSealerCompletion;
    nextState.phase = expectsSealerCompletion
        ? Post7Phase::PreSeal
        : Post7Phase::OutfeedOnly;
    // Guard на один проход supervisor гарантирует честное pre-seal окно:
    // START уже принят, но commit-to-seal еще не достигнут.
    nextState.sealStartGuardPending = expectsSealerCompletion;
    nextState.outfeedStep2SeqBase = outfeedStatus.step2CompletionSeq;
    nextState.sealerCompletionSeqBase = sealerStatus.completionSeq;
    nextState.startedMs = millis();
    nextState.lastSealWaitDiagMs = nextState.startedMs;
    if (expectsSealerCompletion) {
        nextState.lastEvent = "POST7: started mode LOAD (pre-seal cancellable).";
    } else {
        nextState.lastEvent = String("POST7: started mode ") + post7ModeToText(mode);
    }
    g_post7Runtime = nextState;

    Serial.println(g_post7Runtime.lastEvent);
    return true;
}

} // namespace

void initPost7Supervisor()
{
    g_post7Runtime = Post7RuntimeState{};
}

void processPost7Supervisor()
{
    if (!g_post7Runtime.active) {
        return;
    }

    const groups::outfeed::OutfeedStatus outfeedStatus = groups::outfeed::readOutfeedStatus();
    const groups::sealer::SealerStatus sealerStatus = groups::sealer::readSealerStatus();

    const bool outfeedActivity =
        !outfeedStatus.readyForBatch ||
        outfeedStatus.localCycleActive ||
        outfeedStatus.step2Active ||
        outfeedStatus.vfdTimedRunActive ||
        (outfeedStatus.step2CompletionSeq != g_post7Runtime.outfeedStep2SeqBase);
    if (outfeedActivity) {
        g_post7Runtime.outfeedActivityObserved = true;
    }

    if (g_post7Runtime.expectsSealerCompletion &&
        g_post7Runtime.phase == Post7Phase::PreSeal) {
        if (g_post7Runtime.sealStartGuardPending) {
            g_post7Runtime.sealStartGuardPending = false;
            return;
        }

        const uint32_t pulseMs = groups::sealer::getStartPulseDefaultMs();
        if (!groups::sealer::startPulse(pulseMs)) {
            const groups::sealer::SealerStatus rejectStatus = groups::sealer::readSealerStatus();
            if (groups::outfeed::isOtvodCycleActive()) {
                groups::outfeed::abortOtvodCycle(
                    "POST7 LOAD: OTCYCLE aborted because delayed SEAL START was rejected.",
                    true);
            }
            g_post7Runtime.active = false;
            g_post7Runtime.finishedMs = millis();
            g_post7Runtime.lastEvent = "POST7 LOAD: aborted, failed to commit SEAL START.";
            Serial.println(g_post7Runtime.lastEvent);
            Serial.print("POST7 LOAD: SEAL START reject diag: pulse_ms=");
            Serial.print(pulseMs);
            Serial.print(", start_pulse_active=");
            Serial.print(rejectStatus.startPulseActive ? "yes" : "no");
            Serial.print(", wait_done_active=");
            Serial.print(rejectStatus.waitDoneActive ? "yes" : "no");
            Serial.print(", done_raw=");
            Serial.print(rejectStatus.doneRawInputActive ? "active" : "inactive");
            Serial.print(", done_filtered=");
            Serial.print(rejectStatus.doneFilteredInputActive ? "active" : "inactive");
            Serial.print(", done_effective=");
            Serial.print(rejectStatus.doneInputActive ? "active" : "inactive");
            Serial.print(", wait_block=");
            Serial.print(sealerWaitDoneBlockReasonToText(rejectStatus.waitDoneBlockReason));
            Serial.print(", auto_emu=");
            Serial.println(rejectStatus.doneSyntheticAutoEnabled ? "on" : "off");
            return;
        }

        g_post7Runtime.phase = Post7Phase::SealInFlight;
        g_post7Runtime.lastEvent = "POST7 LOAD: phase switched to seal-in-flight.";
        Serial.println(g_post7Runtime.lastEvent);
        if (app::runtime_log::isDebugEnabled()) {
            Serial.print("POST7 LOAD: SEAL START pulse ");
            Serial.print(pulseMs);
            Serial.println(" ms.");
        }
    }

    const bool sealerCompletionAdvanced =
        sealerStatus.completionSeq != g_post7Runtime.sealerCompletionSeqBase;
    if (g_post7Runtime.expectsSealerCompletion &&
        g_post7Runtime.phase == Post7Phase::SealInFlight &&
        sealerCompletionAdvanced) {
        g_post7Runtime.phase = Post7Phase::SealCompletedOutfeedPending;
        g_post7Runtime.lastEvent =
            "POST7 LOAD: phase switched to seal-completed-outfeed-pending.";
        Serial.println(g_post7Runtime.lastEvent);
    }
    if (g_post7Runtime.expectsSealerCompletion &&
        g_post7Runtime.phase == Post7Phase::SealInFlight &&
        !sealerCompletionAdvanced) {
        const uint32_t nowMs = millis();
        if (static_cast<uint32_t>(nowMs - g_post7Runtime.lastSealWaitDiagMs) >=
            POST7_SEAL_WAIT_DIAG_PERIOD_MS) {
            g_post7Runtime.lastSealWaitDiagMs = nowMs;
            Serial.print("POST7 LOAD: waiting sealer completion, wait_age_ms=");
            Serial.print(sealerStatus.waitDoneAgeMs);
            Serial.print(", start_pulse_active=");
            Serial.print(sealerStatus.startPulseActive ? "yes" : "no");
            Serial.print(", wait_done_active=");
            Serial.print(sealerStatus.waitDoneActive ? "yes" : "no");
            Serial.print(", wait_block=");
            Serial.print(sealerWaitDoneBlockReasonToText(sealerStatus.waitDoneBlockReason));
            Serial.print(", done_raw=");
            Serial.print(sealerStatus.doneRawInputActive ? "active" : "inactive");
            Serial.print(", done_filtered=");
            Serial.print(sealerStatus.doneFilteredInputActive ? "active" : "inactive");
            Serial.print(", done_effective=");
            Serial.print(sealerStatus.doneInputActive ? "active" : "inactive");
            Serial.print(", auto_emu=");
            Serial.print(sealerStatus.doneSyntheticAutoEnabled ? "on" : "off");
            Serial.print(", emu_pending=");
            Serial.print(sealerStatus.doneSyntheticPending ? "yes" : "no");
            Serial.print(", emu_hold=");
            Serial.print(sealerStatus.doneSyntheticHoldActive ? "yes" : "no");
            Serial.print(", seal_seq=");
            Serial.print(sealerStatus.completionSeq);
            Serial.print(", seal_seq_base=");
            Serial.println(g_post7Runtime.sealerCompletionSeqBase);
        }
    }
    const bool outfeedReady = outfeedStatus.readyForBatch;
    const bool cycleDone = outfeedReady &&
        (!g_post7Runtime.expectsSealerCompletion || sealerCompletionAdvanced);
    if (!cycleDone) {
        return;
    }

    g_post7Runtime.active = false;
    g_post7Runtime.phase = Post7Phase::None;
    g_post7Runtime.finishedMs = millis();
    g_post7Runtime.lastEvent = String("POST7: completed mode ") + post7ModeToText(g_post7Runtime.mode);
    Serial.println(g_post7Runtime.lastEvent);
}

bool startPost7Load()
{
    return startPost7Job(Post7Mode::Load);
}

bool startPost7UnloadOnly()
{
    return startPost7Job(Post7Mode::UnloadOnly);
}

bool stopPost7Job(const char *reason)
{
    if (!g_post7Runtime.active) {
        Serial.println("POST7: no active job.");
        return false;
    }

    if (groups::outfeed::isOtvodCycleActive()) {
        groups::outfeed::abortOtvodCycle("POST7: OTCYCLE aborted by POST7 STOP.", true);
    }

    const groups::sealer::SealerStatus sealerStatus = groups::sealer::readSealerStatus();
    const Post7Phase currentPhase = resolvePost7PhaseForActiveJob(sealerStatus);
    const bool stopNeedsRecovery = isStopRecoveryZone(currentPhase);

    g_post7Runtime.active = false;
    g_post7Runtime.phase = Post7Phase::None;
    g_post7Runtime.finishedMs = millis();
    if (stopNeedsRecovery) {
        g_post7Runtime.manualRecoveryRequired = true;
        g_post7Runtime.lastEvent = "POST7: aborted in seal-in-flight; manual recovery required.";
        Serial.println(g_post7Runtime.lastEvent);
        Serial.print("POST7 STOP diag: phase=");
        Serial.print(post7PhaseToText(currentPhase));
        Serial.print(", start_pulse_active=");
        Serial.print(sealerStatus.startPulseActive ? "yes" : "no");
        Serial.print(", wait_done_active=");
        Serial.print(sealerStatus.waitDoneActive ? "yes" : "no");
        Serial.print(", wait_block=");
        Serial.print(sealerWaitDoneBlockReasonToText(sealerStatus.waitDoneBlockReason));
        Serial.print(", done_raw=");
        Serial.print(sealerStatus.doneRawInputActive ? "active" : "inactive");
        Serial.print(", done_filtered=");
        Serial.print(sealerStatus.doneFilteredInputActive ? "active" : "inactive");
        Serial.print(", done_effective=");
        Serial.print(sealerStatus.doneInputActive ? "active" : "inactive");
        Serial.print(", auto_emu=");
        Serial.print(sealerStatus.doneSyntheticAutoEnabled ? "on" : "off");
        Serial.print(", seal_seq=");
        Serial.print(sealerStatus.completionSeq);
        Serial.print(", seal_seq_base=");
        Serial.println(g_post7Runtime.sealerCompletionSeqBase);
    } else {
        g_post7Runtime.lastEvent = String("POST7: aborted in phase ") +
            post7PhaseToText(currentPhase) + ".";
        if (reason != nullptr && reason[0] != '\0') {
            Serial.print("POST7: aborted (");
            Serial.print(reason);
            Serial.print(", phase=");
            Serial.print(post7PhaseToText(currentPhase));
            Serial.println(").");
        } else {
            Serial.println(g_post7Runtime.lastEvent);
        }
    }
    return true;
}

bool clearManualRecoveryLatch(const char *reason)
{
    if (g_post7Runtime.active) {
        Serial.println("POST7: manual recovery clear rejected while POST7 active.");
        return false;
    }
    if (!g_post7Runtime.manualRecoveryRequired) {
        Serial.println("POST7: manual recovery latch is already clear.");
        return true;
    }
    const groups::sealer::SealerStatus sealerStatus = groups::sealer::readSealerStatus();
    if (sealerStatus.startPulseActive || sealerStatus.waitDoneActive) {
        Serial.println("POST7: manual recovery clear rejected, SEAL still in-flight.");
        return false;
    }

    g_post7Runtime.manualRecoveryRequired = false;
    g_post7Runtime.lastEvent = "POST7: manual recovery latch cleared.";
    if (reason != nullptr && reason[0] != '\0') {
        Serial.print("POST7: manual recovery latch cleared (");
        Serial.print(reason);
        Serial.println(").");
    } else {
        Serial.println(g_post7Runtime.lastEvent);
    }
    return true;
}

bool isManualRecoveryRequired()
{
    return g_post7Runtime.manualRecoveryRequired;
}

Post7Status readPost7Status()
{
    const groups::outfeed::OutfeedStatus outfeedStatus = groups::outfeed::readOutfeedStatus();
    const groups::sealer::SealerStatus sealerStatus = groups::sealer::readSealerStatus();

    Post7Status status = {};
    status.active = g_post7Runtime.active;
    status.expectsSealerCompletion = g_post7Runtime.expectsSealerCompletion;
    status.phase = resolvePost7PhaseForActiveJob(sealerStatus);
    status.preSealCancellable = status.active && status.phase == Post7Phase::PreSeal;
    status.sealCommitReached = status.active && isSealCommitReachedPhase(status.phase);
    status.outfeedReadyForBatch = outfeedStatus.readyForBatch;
    status.outfeedActivityObserved = g_post7Runtime.outfeedActivityObserved;
    status.sealerCompletionAdvanced = g_post7Runtime.expectsSealerCompletion &&
        (sealerStatus.completionSeq != g_post7Runtime.sealerCompletionSeqBase);
    status.manualRecoveryRequired = g_post7Runtime.manualRecoveryRequired;
    status.outfeedStep2SeqBase = g_post7Runtime.outfeedStep2SeqBase;
    status.outfeedStep2SeqCurrent = outfeedStatus.step2CompletionSeq;
    status.sealerCompletionSeqBase = g_post7Runtime.sealerCompletionSeqBase;
    status.sealerCompletionSeqCurrent = sealerStatus.completionSeq;
    status.startedMs = g_post7Runtime.startedMs;
    status.finishedMs = g_post7Runtime.finishedMs;
    status.mode = g_post7Runtime.mode;
    return status;
}

void printPost7Status()
{
    const Post7Status status = readPost7Status();
    Serial.print("POST7: active=");
    Serial.print(status.active ? "yes" : "no");
    Serial.print(", mode=");
    Serial.print(post7ModeToText(status.mode));
    Serial.print(", phase=");
    Serial.print(post7PhaseToText(status.phase));
    Serial.print(", stop_recovery_zone=");
    Serial.print((status.active && isStopRecoveryZone(status.phase)) ? "yes" : "no");
    Serial.print(", pre_seal=");
    Serial.print(status.preSealCancellable ? "yes" : "no");
    Serial.print(", seal_commit=");
    Serial.print(status.sealCommitReached ? "yes" : "no");
    Serial.print(", outfeed_ready=");
    Serial.print(status.outfeedReadyForBatch ? "yes" : "no");
    Serial.print(", outfeed_seen_activity=");
    Serial.print(status.outfeedActivityObserved ? "yes" : "no");
    Serial.print(", outfeed_seq=");
    Serial.print(status.outfeedStep2SeqCurrent);
    Serial.print(", outfeed_seq_base=");
    Serial.print(status.outfeedStep2SeqBase);
    Serial.print(", seal_seq=");
    Serial.print(status.sealerCompletionSeqCurrent);
    Serial.print(", seal_seq_base=");
    Serial.print(status.sealerCompletionSeqBase);
    Serial.print(", seal_seq_advanced=");
    Serial.print(status.sealerCompletionAdvanced ? "yes" : "no");
    Serial.print(", manual_recovery_required=");
    Serial.print(status.manualRecoveryRequired ? "yes" : "no");
    Serial.print(", started_ms=");
    Serial.print(status.startedMs);
    Serial.print(", finished_ms=");
    Serial.print(status.finishedMs);
    Serial.print(", last=");
    Serial.println(g_post7Runtime.lastEvent);
}

} // namespace app::post7
