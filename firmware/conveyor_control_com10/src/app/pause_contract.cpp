// Реализация pause-контракта на конвейере: парсинг команд от мастера и поля для I2C snapshot.

#include "app/pause_contract.h"

#include <Arduino.h>

#include "app/conveyor_status_runtime.h"
#include "app/post7_supervisor.h"

namespace {

constexpr uint16_t ERROR_PAUSE_STATUS_ACK_VALID_BIT = 1U << 3;
constexpr uint32_t HEARTBEAT_PAUSE_EXT_VALID_BIT = 1U << 29U;
constexpr uint32_t HEARTBEAT_RESIDUAL_KNOWN_BIT = 1U << 28U;
constexpr uint32_t HEARTBEAT_PAUSE_STATE_SHIFT = 30U;
constexpr int16_t PAUSE_SEALER_RESIDUAL_PLATES = 6;
constexpr int16_t PAUSE_COUNT_UNKNOWN = -1;
constexpr uint8_t PAUSE_COUNT_WIRE_UNKNOWN = 0x0FU;
constexpr uint8_t PAUSE_COUNT_WIRE_MAX = 0x0EU;
constexpr uint32_t PAUSE_PREPARE_TIMEOUT_MS = 90000U;

uint16_t g_pauseWireEpoch = 0;
bool g_pauseWireValid = false;
app::PausePrepareState g_pausePrepareState = app::PausePrepareState::None;
app::PausePrepareWaitReason g_pausePrepareWaitReason = app::PausePrepareWaitReason::None;
uint32_t g_pausePrepareRequestedSinceMs = 0;
uint32_t g_pausePrepareWaitingSinceMs = 0;
const char *g_pauseBlockedReason = "none";

bool parseUnsignedArg(const char *s, uint32_t &out)
{
    if (s == nullptr || s[0] == '\0') {
        return false;
    }
    uint32_t acc = 0;
    for (const char *p = s; *p != '\0'; ++p) {
        const char c = *p;
        if (c < '0' || c > '9') {
            return false;
        }
        const uint32_t d = static_cast<uint32_t>(c - '0');
        if (acc > (UINT32_MAX - d) / 10U) {
            return false;
        }
        acc = acc * 10U + d;
    }
    out = acc;
    return true;
}

const char *pausePrepareStateName(app::PausePrepareState state)
{
    switch (state) {
        case app::PausePrepareState::Requested:
            return "requested";
        case app::PausePrepareState::Preparing:
            return "preparing";
        case app::PausePrepareState::Ready:
            return "ready";
        case app::PausePrepareState::BlockedFault:
            return "blocked_fault";
        case app::PausePrepareState::None:
        default:
            return "none";
    }
}

const char *pausePrepareWaitName(app::PausePrepareWaitReason reason)
{
    switch (reason) {
        case app::PausePrepareWaitReason::ActiveP1SafeBoundary:
            return "active_p1_safe_boundary";
        case app::PausePrepareWaitReason::ActivePost7SafeBoundary:
            return "active_post7_safe_boundary";
        case app::PausePrepareWaitReason::None:
        default:
            return "none";
    }
}

const char *pausePrepareDiagText(app::PausePrepareState state,
                                 app::PausePrepareWaitReason waitReason)
{
    switch (state) {
        case app::PausePrepareState::Requested:
            return "pause_prepare requested";
        case app::PausePrepareState::Preparing:
            if (waitReason == app::PausePrepareWaitReason::ActiveP1SafeBoundary) {
                return "pause_prepare waiting active P1 safe boundary";
            }
            if (waitReason == app::PausePrepareWaitReason::ActivePost7SafeBoundary) {
                return "pause_prepare waiting active POST7 safe boundary";
            }
            return "pause_prepare waiting local safe boundary";
        case app::PausePrepareState::Ready:
            return "pause_ready";
        case app::PausePrepareState::BlockedFault:
            return "pause_blocked_fault";
        case app::PausePrepareState::None:
        default:
            return "pause_none";
    }
}

uint8_t pausePrepareStateWireCode(app::PausePrepareState state)
{
    switch (state) {
        case app::PausePrepareState::Requested:
            return 0U;
        case app::PausePrepareState::Preparing:
            return 1U;
        case app::PausePrepareState::Ready:
            return 2U;
        case app::PausePrepareState::BlockedFault:
            return 3U;
        case app::PausePrepareState::None:
        default:
            return 0U;
    }
}

struct PausePrepareEval {
    app::PausePrepareState state = app::PausePrepareState::None;
    app::PausePrepareWaitReason waitReason = app::PausePrepareWaitReason::None;
    const char *blockedReason = "none";
};

PausePrepareEval evaluatePausePrepareState()
{
    PausePrepareEval eval = {};

    if (!g_pauseWireValid) {
        return eval;
    }

    const app::ConveyorStatusInputs inputs = app::readConveyorStatusInputs();
    const app::post7::Post7Status post7Status = app::post7::readPost7Status();

    const bool waitP1SafeBoundary =
        inputs.program1Active ||
        inputs.infeed.busy ||
        (inputs.program1StateCode != 0U);

    const bool waitPost7SafeBoundary =
        post7Status.active ||
        inputs.outfeed.localCycleActive ||
        inputs.outfeed.step2Active ||
        inputs.outfeed.vfdTimedRunActive ||
        inputs.sealer.busy ||
        inputs.sealer.startPulseActive;

    const bool blockedFault =
        inputs.program1AbortRecoveryRequired ||
        inputs.post7ManualRecoveryRequired ||
        inputs.sensorConflict;

    if (blockedFault) {
        eval.state = app::PausePrepareState::BlockedFault;
        if (inputs.post7ManualRecoveryRequired) {
            eval.blockedReason = "post7_manual_recovery_required";
        } else if (inputs.program1AbortRecoveryRequired) {
            eval.blockedReason = "program1_abort_recovery_required";
        } else {
            eval.blockedReason = "sensor_conflict";
        }
        return eval;
    }

    if (waitP1SafeBoundary) {
        eval.state = app::PausePrepareState::Preparing;
        eval.waitReason = app::PausePrepareWaitReason::ActiveP1SafeBoundary;
        return eval;
    }

    if (waitPost7SafeBoundary) {
        eval.state = app::PausePrepareState::Preparing;
        eval.waitReason = app::PausePrepareWaitReason::ActivePost7SafeBoundary;
        return eval;
    }

    eval.state = app::PausePrepareState::Ready;
    return eval;
}

void resetPausePrepareRuntime()
{
    g_pausePrepareState = app::PausePrepareState::None;
    g_pausePrepareWaitReason = app::PausePrepareWaitReason::None;
    g_pausePrepareRequestedSinceMs = 0;
    g_pausePrepareWaitingSinceMs = 0;
    g_pauseBlockedReason = "none";
}

void printPausePrepareTransitionIfNeeded(app::PausePrepareState prevState,
                                         app::PausePrepareWaitReason prevWaitReason,
                                         const char *prevBlockedReason)
{
    if (prevState == g_pausePrepareState &&
        prevWaitReason == g_pausePrepareWaitReason &&
        prevBlockedReason == g_pauseBlockedReason) {
        return;
    }

    switch (g_pausePrepareState) {
        case app::PausePrepareState::Preparing:
            if (g_pausePrepareWaitReason == app::PausePrepareWaitReason::ActiveP1SafeBoundary) {
                Serial.println("PAUSE: pause_prepare waiting active P1 safe boundary.");
            } else if (g_pausePrepareWaitReason == app::PausePrepareWaitReason::ActivePost7SafeBoundary) {
                Serial.println("PAUSE: pause_prepare waiting active POST7 safe boundary.");
            } else {
                Serial.println("PAUSE: pause_prepare waiting local safe boundary.");
            }
            break;
        case app::PausePrepareState::Ready:
            Serial.println("PAUSE: pause_ready.");
            break;
        case app::PausePrepareState::BlockedFault:
            Serial.print("PAUSE: pause_blocked_fault reason=");
            Serial.println(g_pauseBlockedReason);
            break;
        case app::PausePrepareState::Requested:
            Serial.println("PAUSE: pause_prepare requested.");
            break;
        case app::PausePrepareState::None:
        default:
            break;
    }
}

void refreshPausePrepareState(bool emitTransitionLogs)
{
    const app::PausePrepareState prevState = g_pausePrepareState;
    const app::PausePrepareWaitReason prevWaitReason = g_pausePrepareWaitReason;
    const char *prevBlockedReason = g_pauseBlockedReason;

    if (!g_pauseWireValid) {
        resetPausePrepareRuntime();
        return;
    }

    if (g_pausePrepareRequestedSinceMs == 0U) {
        g_pausePrepareRequestedSinceMs = millis();
    }

    PausePrepareEval eval = evaluatePausePrepareState();

    if (eval.state == app::PausePrepareState::Preparing) {
        if (g_pausePrepareState != app::PausePrepareState::Preparing ||
            g_pausePrepareWaitReason != eval.waitReason ||
            g_pausePrepareWaitingSinceMs == 0U) {
            g_pausePrepareWaitingSinceMs = millis();
        }

        const uint32_t waitElapsedMs = millis() - g_pausePrepareWaitingSinceMs;
        if (waitElapsedMs >= PAUSE_PREPARE_TIMEOUT_MS) {
            eval.state = app::PausePrepareState::BlockedFault;
            eval.waitReason = app::PausePrepareWaitReason::None;
            eval.blockedReason = "prepare_timeout";
        }
    } else {
        g_pausePrepareWaitingSinceMs = 0U;
    }

    g_pausePrepareState = eval.state;
    g_pausePrepareWaitReason = eval.waitReason;
    g_pauseBlockedReason = eval.blockedReason;

    if (emitTransitionLogs) {
        printPausePrepareTransitionIfNeeded(prevState, prevWaitReason, prevBlockedReason);
    }
}

void computeWireCounts(const app::ConveyorStatusInputs &inputs,
                       int16_t &bufferCount,
                       bool &bufferValid,
                       int16_t &sealerCount,
                       bool &sealerValid,
                       bool &residualKnown)
{
    bufferValid = inputs.feedBufferCountKnown;
    bufferCount = bufferValid ? static_cast<int16_t>(inputs.feedBufferCount) : PAUSE_COUNT_UNKNOWN;

    sealerValid = inputs.sealerPlateCountKnown;
    if (sealerValid) {
        const int16_t sealerCountRaw = static_cast<int16_t>(inputs.sealerPlateCount);
        sealerCount = sealerCountRaw > 0 ? sealerCountRaw : 0;
        if (sealerCount > PAUSE_SEALER_RESIDUAL_PLATES) {
            sealerCount = PAUSE_SEALER_RESIDUAL_PLATES;
        }
    } else {
        sealerCount = PAUSE_COUNT_UNKNOWN;
    }

    residualKnown =
        bufferValid &&
        sealerValid &&
        !inputs.infeed.busy &&
        !inputs.program1Active;
}

uint32_t packHeartbeatWithPauseExt(uint32_t millisNow,
                                   uint8_t bufferCount4,
                                   uint8_t sealerCount4,
                                   bool residualKnown,
                                   app::PausePrepareState pauseState)
{
    uint32_t hb = millisNow & 0xFFFFFU;
    hb |= static_cast<uint32_t>(bufferCount4 & 0x0FU) << 20;
    hb |= static_cast<uint32_t>(sealerCount4 & 0x0FU) << 24;
    if (residualKnown) {
        hb |= HEARTBEAT_RESIDUAL_KNOWN_BIT;
    }
    hb |= HEARTBEAT_PAUSE_EXT_VALID_BIT;
    hb |= static_cast<uint32_t>(pausePrepareStateWireCode(pauseState) & 0x03U) <<
        HEARTBEAT_PAUSE_STATE_SHIFT;
    return hb;
}

uint8_t encodeWireCount4(int16_t count, bool valid)
{
    if (!valid) {
        return PAUSE_COUNT_WIRE_UNKNOWN;
    }
    if (count <= 0) {
        return 0U;
    }
    if (count > static_cast<int16_t>(PAUSE_COUNT_WIRE_MAX)) {
        return PAUSE_COUNT_WIRE_MAX;
    }
    return static_cast<uint8_t>(count);
}

} // namespace

namespace app {

bool pauseContractHandleCommand(const char *lineIn)
{
    if (lineIn == nullptr) {
        return false;
    }

    String line(lineIn);
    line.trim();
    if (!line.length()) {
        return false;
    }

    String upper = line;
    upper.toUpperCase();
    if (!upper.startsWith("PAUSE ")) {
        return false;
    }

    String rest = upper.substring(6);
    rest.trim();
    rest.replace('\t', ' ');
    int sp = rest.indexOf(' ');
    String verb = sp < 0 ? rest : rest.substring(0, sp);
    String argStr = sp < 0 ? String("") : rest.substring(sp + 1);
    argStr.trim();

    uint32_t epoch = 0;
    if (!parseUnsignedArg(argStr.c_str(), epoch)) {
        Serial.println("PAUSE: need epoch number");
        return true;
    }

    g_pauseWireEpoch = static_cast<uint16_t>(epoch & 0x0FFFU);

    if (verb == "ARM") {
        g_pauseWireValid = true;
        g_pausePrepareState = PausePrepareState::Requested;
        g_pausePrepareWaitReason = PausePrepareWaitReason::None;
        g_pausePrepareRequestedSinceMs = millis();
        g_pausePrepareWaitingSinceMs = 0U;
        g_pauseBlockedReason = "none";

        Serial.print("PAUSE ARM epoch=");
        Serial.println(g_pauseWireEpoch);
        Serial.println("PAUSE: pause_prepare requested.");
        refreshPausePrepareState(true);
        return true;
    }

    if (verb == "STATUS") {
        refreshPausePrepareState(true);
        const ConveyorStatusInputs inputs = readConveyorStatusInputs();
        int16_t bc = 0;
        bool bv = false;
        int16_t sc = 0;
        bool sv = false;
        bool rk = false;
        computeWireCounts(inputs, bc, bv, sc, sv, rk);
        const uint8_t wireBuf = encodeWireCount4(bc, bv);
        const uint8_t wireSeal = encodeWireCount4(sc, sv);

        Serial.print("PAUSE STATUS epoch=");
        Serial.print(g_pauseWireEpoch);
        Serial.print(", armed=");
        Serial.print(g_pauseWireValid ? "yes" : "no");
        Serial.print(", state=");
        Serial.print(pausePrepareStateName(g_pausePrepareState));
        Serial.print(", wait=");
        Serial.print(pausePrepareWaitName(g_pausePrepareWaitReason));
        Serial.print(", diag=");
        Serial.print(pausePrepareDiagText(g_pausePrepareState, g_pausePrepareWaitReason));
        Serial.print(", ready=");
        Serial.print(pauseContractIsReady() ? "yes" : "no");
        Serial.print(", blocked=");
        Serial.print(pauseContractIsBlockedFault() ? "yes" : "no");
        if (pauseContractIsBlockedFault()) {
            Serial.print(", blocked_reason=");
            Serial.print(g_pauseBlockedReason);
        }
        Serial.print(", local_buffer=");
        if (inputs.feedBufferCountKnown) {
            Serial.print(inputs.feedBufferCount);
        } else {
            Serial.print("unknown");
        }
        Serial.print(", local_in2_pairs=");
        if (inputs.feedIn2PairsKnown) {
            Serial.print(inputs.feedIn2Pairs);
        } else {
            Serial.print("unknown");
        }
        Serial.print(", local_sealer=");
        if (inputs.sealerPlateCountKnown) {
            Serial.print(inputs.sealerPlateCount);
        } else {
            Serial.print("unknown");
        }
        Serial.print(", wire_buffer=");
        if (wireBuf == PAUSE_COUNT_WIRE_UNKNOWN) {
            Serial.print("unknown");
        } else {
            Serial.print(wireBuf);
        }
        Serial.print(", wire_sealer=");
        if (wireSeal == PAUSE_COUNT_WIRE_UNKNOWN) {
            Serial.print("unknown");
        } else {
            Serial.print(wireSeal);
        }
        Serial.print(", residual_known=");
        Serial.print(rk ? "yes" : "no");
        Serial.print(", prepare_elapsed_ms=");
        const uint32_t elapsed =
            g_pausePrepareRequestedSinceMs == 0U ? 0U : (millis() - g_pausePrepareRequestedSinceMs);
        Serial.println(elapsed);
        return true;
    }

    if (verb == "RELEASE") {
        g_pauseWireValid = false;
        resetPausePrepareRuntime();
        Serial.print("PAUSE RELEASE epoch=");
        Serial.println(g_pauseWireEpoch);
        return true;
    }

    Serial.println("PAUSE: use PAUSE ARM|STATUS|RELEASE <epoch>");
    return true;
}

void pauseContractTick()
{
    refreshPausePrepareState(true);
}

bool pauseContractIsArmed()
{
    return g_pauseWireValid;
}

bool pauseContractIsReady()
{
    return g_pauseWireValid && g_pausePrepareState == PausePrepareState::Ready;
}

bool pauseContractIsBlockedFault()
{
    return g_pauseWireValid && g_pausePrepareState == PausePrepareState::BlockedFault;
}

PausePrepareState pauseContractPrepareState()
{
    return g_pausePrepareState;
}

PausePrepareWaitReason pauseContractPrepareWaitReason()
{
    return g_pausePrepareWaitReason;
}

void pauseContractApplyToI2cFrame(comm::I2cStatusFrame &frame)
{
    if (!g_pauseWireValid) {
        return;
    }

    refreshPausePrepareState(false);

    frame.errorWord = static_cast<uint16_t>(frame.errorWord | ERROR_PAUSE_STATUS_ACK_VALID_BIT);
    frame.errorWord = static_cast<uint16_t>(
        frame.errorWord | static_cast<uint16_t>((g_pauseWireEpoch & 0x0FFFU) << 4));

    const ConveyorStatusInputs inputs = readConveyorStatusInputs();
    int16_t bc = 0;
    bool bv = false;
    int16_t sc = 0;
    bool sv = false;
    bool rk = false;
    computeWireCounts(inputs, bc, bv, sc, sv, rk);
    const uint8_t buf4 = encodeWireCount4(bc, bv);
    const uint8_t seal4 = encodeWireCount4(sc, sv);

    frame.heartbeatMs = packHeartbeatWithPauseExt(millis(),
                                                   buf4,
                                                   seal4,
                                                   rk,
                                                   g_pausePrepareState);
}

} // namespace app
