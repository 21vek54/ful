#include "app/cli_handler.h"
#include "app/board_monitor.h"
#include "app/conveyor_status_runtime.h"
#include "app/manual_runtime.h"
#include "app/post7_supervisor.h"
#include "app/pause_contract.h"
#include "core/settings.h"

#include <math.h>

#include "groups/infeed/infeed_group.h"
#include "groups/outfeed/outfeed_group.h"
#include "groups/sealer/sealer_group.h"
#include "legacy/program1.h"

namespace {


String g_cmdBuffer;

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

bool parseSecondsToDurationMs(const String &source, uint32_t &durationMs)
{
    String normalized = source;
    normalized.trim();
    normalized.replace(',', '.');
    if (normalized.isEmpty()) {
        return false;
    }

    bool hasDigit = false;
    bool hasDot = false;
    for (size_t i = 0; i < normalized.length(); i++) {
        const char c = normalized[i];
        if (c >= '0' && c <= '9') {
            hasDigit = true;
            continue;
        }
        if (c == '.' && !hasDot) {
            hasDot = true;
            continue;
        }
        return false;
    }

    if (!hasDigit) {
        return false;
    }

    const float seconds = normalized.toFloat();
    if (!(seconds > 0.0f)) {
        return false;
    }

    const float durationMsFloat = seconds * 1000.0f;
    if (!(durationMsFloat >= static_cast<float>(groups::outfeed::getVfdTickMinDurationMs())) ||
        durationMsFloat > static_cast<float>(groups::outfeed::getVfdTickMaxDurationMs())) {
        return false;
    }

    durationMs = static_cast<uint32_t>(durationMsFloat + 0.5f);
    return true;
}

bool parseDistanceMmArgs(String args, const String &cmd, uint32_t &distanceMm)
{
    args.trim();
    if (args.isEmpty()) {
        return false;
    }

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
    if (distanceMm > (UINT32_MAX / core::g_settings.pulsesPerMm)) {
        Serial.println("Error: distance is too large.");
        return false;
    }

    steps = distanceMm * core::g_settings.pulsesPerMm;
    return true;
}

bool parseOtcycleStartArgs(const String &args, const char *example, uint32_t &cyclesRaw, uint32_t &stepsRaw)
{
    String cyclesTok;
    String stepsTok;
    splitCommandLine(args, cyclesTok, stepsTok);

    if (!cyclesTok.isEmpty() && !parseUnsigned(cyclesTok, cyclesRaw)) {
        Serial.print("Format error. Example: ");
        Serial.println(example);
        return false;
    }
    if (!stepsTok.isEmpty() && !parseUnsigned(stepsTok, stepsRaw)) {
        Serial.print("Format error. Example: ");
        Serial.println(example);
        return false;
    }
    if (cyclesRaw > 255U) {
        Serial.print("Format error. Example: ");
        Serial.println(example);
        return false;
    }

    return true;
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

void printSealEmuOnceUsage()
{
    Serial.print("Usage: SEAL EMU ONCE [0..");
    Serial.print(groups::sealer::getDoneEmuDelayMaxMs());
    Serial.print("] [");
    Serial.print(groups::sealer::getDoneEmuHoldMinMs());
    Serial.print("..");
    Serial.print(groups::sealer::getDoneEmuHoldMaxMs());
    Serial.println("]");
}

bool parseSealEmuOnceArgs(const String &args, uint32_t &delayMs, uint32_t &holdMs)
{
    String delayTok;
    String holdTail;
    splitCommandLine(args, delayTok, holdTail);

    if (!delayTok.isEmpty() && !parseUnsigned(delayTok, delayMs)) {
        return false;
    }
    if (holdTail.isEmpty()) {
        return true;
    }

    String holdTok;
    String extra;
    splitCommandLine(holdTail, holdTok, extra);
    if (holdTok.isEmpty() || !parseUnsigned(holdTok, holdMs)) {
        return false;
    }
    extra.trim();
    return extra.isEmpty();
}

void printSealEmuStatus()
{
    const groups::sealer::SealerStatus status = groups::sealer::readSealerStatus();
    Serial.print("SEAL EMU: run_state=");
    Serial.print(sealerRunStateToText(status.runState));
    Serial.print(", pending=");
    Serial.print(status.doneSyntheticPending ? "yes" : "no");
    Serial.print(", hold=");
    Serial.print(status.doneSyntheticHoldActive ? "yes" : "no");
    Serial.print(", remaining_ms=");
    Serial.print(status.doneSyntheticRemainingMs);
    Serial.print(", delay_ms=");
    Serial.print(status.doneSyntheticDelayMs);
    Serial.print(", hold_ms=");
    Serial.print(status.doneSyntheticHoldMs);
    Serial.print(", auto=");
    Serial.print(status.doneSyntheticAutoEnabled ? "on" : "off");
    Serial.print(", auto_delay_ms=");
    Serial.print(status.doneSyntheticAutoDelayMs);
    Serial.print(", auto_hold_ms=");
    Serial.print(status.doneSyntheticAutoHoldMs);
    Serial.print(", done_raw=");
    Serial.print(status.doneRawInputActive ? "active" : "inactive");
    Serial.print(", done_filtered=");
    Serial.print(status.doneFilteredInputActive ? "active" : "inactive");
    Serial.print(", done_effective=");
    Serial.print(status.doneInputActive ? "active" : "inactive");
    Serial.print(", done_pin_level=");
    Serial.print(logicLevelToText(status.donePinLevelHigh));
    Serial.print(", done_active_level=");
    Serial.print(status.doneActiveLevelLow ? "LOW" : "HIGH");
    Serial.print(", wait_done=");
    Serial.print(status.waitDoneActive ? "yes" : "no");
    Serial.print(", wait_age_ms=");
    Serial.print(status.waitDoneAgeMs);
    Serial.print(", wait_hb_seq=");
    Serial.print(status.waitDoneHeartbeatSeq);
    Serial.print(", block_reason=");
    Serial.print(sealerWaitDoneBlockReasonToText(status.waitDoneBlockReason));
    Serial.print(", completion_blocked=");
    Serial.print(status.completionBlocked ? "yes" : "no");
    Serial.print(", completion_seq=");
    Serial.print(status.completionSeq);
    Serial.print(", last_completion=");
    Serial.print(sealerLastCompletionToText(status));
    Serial.print(", edges_raw=");
    Serial.print(status.doneRawRiseCount);
    Serial.print("/");
    Serial.print(status.doneRawFallCount);
    Serial.print(", edges_effective=");
    Serial.print(status.doneEffectiveRiseCount);
    Serial.print("/");
    Serial.print(status.doneEffectiveFallCount);
    Serial.print(", ignored_outside_wait=");
    Serial.print(status.ignoredRiseOutsideWaitDoneCount);
    Serial.print(", ignored_last_ms=");
    Serial.println(status.ignoredRiseOutsideWaitDoneLastMs);
}

void printInfeedEmuUsage()
{
    Serial.println("Usage:");
    Serial.println("  INFEED EMU ON [on_ms] [off_ms]");
    Serial.println("  INFEED EMU OFF");
    Serial.println("  INFEED EMU PASS <count> [on_ms] [off_ms]");
    Serial.println("  INFEED EMU REALISTIC ON [START_ON_SENSOR]");
    Serial.println("  INFEED EMU REALISTIC OFF");
    Serial.println("  INFEED EMU REALISTIC STATUS");
    Serial.println("  INFEED EMU STATUS");
    Serial.print("Ranges: count=1..");
    Serial.print(groups::infeed::getPlatePassEmuMaxCount());
    Serial.print(", on_ms=");
    Serial.print(groups::infeed::getPlatePassEmuMinOnMs());
    Serial.print("..");
    Serial.print(groups::infeed::getPlatePassEmuMaxOnMs());
    Serial.print(", off_ms=");
    Serial.print(groups::infeed::getPlatePassEmuMinOffMs());
    Serial.print("..");
    Serial.println(groups::infeed::getPlatePassEmuMaxOffMs());
    Serial.println("Realistic start options: START_ON_SENSOR|WITH_PLATE|NORMAL.");
}

const char *infeedEmuProfileToText(groups::infeed::InfeedEmuProfile profile)
{
    switch (profile) {
        case groups::infeed::InfeedEmuProfile::Generic:
            return "generic";
        case groups::infeed::InfeedEmuProfile::Realistic:
            return "realistic";
    }
    return "unknown";
}

bool parseInfeedEmuRealisticStartMode(const String &tokenRaw, bool &startOnSensor)
{
    String token = tokenRaw;
    token.trim();
    token.toUpperCase();

    if (token.isEmpty() ||
        token == "NORMAL" ||
        token == "NO_PLATE" ||
        token == "WITHOUT_PLATE" ||
        token == "START_CLEAR") {
        startOnSensor = false;
        return true;
    }

    if (token == "START_ON_SENSOR" ||
        token == "WITH_PLATE" ||
        token == "SENSOR_ACTIVE" ||
        token == "PLATE_ON_SENSOR") {
        startOnSensor = true;
        return true;
    }

    return false;
}

void printInfeedEmuStatus()
{
    const groups::infeed::InfeedEmuStatus status = groups::infeed::readInfeedEmuStatus();
    Serial.print("INFEED EMU: auto=");
    Serial.print(status.autoEnabled ? "on" : "off");
    Serial.print(", sequence=");
    Serial.print(status.sequenceActive ? "active" : "idle");
    Serial.print(", raw=");
    Serial.print(status.rawSensorActive ? "active" : "inactive");
    Serial.print(", profile=");
    Serial.print(infeedEmuProfileToText(status.profile));
    Serial.print(", realistic_start_on_sensor=");
    Serial.print(status.realisticStartOnSensor ? "yes" : "no");
    Serial.print(", realistic_start_pending=");
    Serial.print(status.realisticStartPending ? "yes" : "no");
    Serial.print(", pass_seq=");
    Serial.print(status.passSeq);
    Serial.print(", pending=");
    Serial.print(status.pendingPasses);
    Serial.print(", on_ms=");
    Serial.print(status.onMs);
    Serial.print(", off_ms=");
    Serial.print(status.offMs);
    Serial.print(", phase_elapsed_ms=");
    Serial.println(status.phaseElapsedMs);
}

bool parseInfeedEmuArgs(String args, uint32_t *values, size_t maxCount, size_t &outCount)
{
    outCount = 0;
    args.trim();
    if (args.isEmpty()) {
        return true;
    }

    while (!args.isEmpty()) {
        if (outCount >= maxCount) {
            return false;
        }

        String token;
        String tail;
        splitCommandLine(args, token, tail);
        if (token.isEmpty()) {
            return false;
        }
        if (!parseUnsigned(token, values[outCount])) {
            return false;
        }
        outCount++;
        args = tail;
        args.trim();
    }
    return true;
}

bool tryHandleDistanceMoveCommand(const String &cmd, const String &args)
{
    uint32_t steps = 0;

    if (cmd == "D") {
        if (!parseMoveDistanceSteps(args, cmd, "D 200", steps)) {
            return true;
        }
        app::manual::startConstantMotion(steps, core::g_settings.manualMoveDelayUs);
        return true;
    }

    if (cmd == "P") {
        if (!parseMoveDistanceSteps(args, cmd, "P 300", steps)) {
            return true;
        }
        app::manual::startPositionalProfiledMotion(steps, core::g_settings.posRunDelayUs);
        return true;
    }

    return false;
}

bool tryHandleProgramCommand(const String &cmd)
{
    if (cmd == "1") {
        app::manual::startProgram1();
        return true;
    }

    if (cmd == "2") {
        app::manual::startCycle2();
        return true;
    }

    if (cmd == "3") {
        app::manual::startPositionalProfiledMotion(core::g_settings.c3CommandSteps, core::g_settings.c3CommandDelayUs);
        return true;
    }

    return false;
}

bool tryHandleShiftCommand(const String &cmd)
{
    if (cmd == "CZ") {
        app::manual::shiftCalibrateTravel();
        return true;
    }

    if (cmd == "C") {
        app::manual::shiftRunMoveCommandC();
        return true;
    }

    if (cmd == "Z") {
        app::manual::shiftRunMoveCommandZ();
        return true;
    }

    return false;
}

bool tryHandleServiceCommand(const String &cmd, const String &args)
{
    if (cmd == "FSC") {
        app::conveyorFeedSideNoteIn2Consumed();
        Serial.println("FEED STRICT: sync IN2 consumed accepted.");
        return true;
    }

    if (cmd == "FSINV") {
        app::invalidateConveyorFeedSideModel("manual FSINV command");
        app::clearConveyorProgram1AbortRecoveryRequired("FSINV");
        Serial.println("FEED STRICT: model invalidated.");
        return true;
    }

    if (cmd == "P1REC") {
        String subCmd;
        String unusedArgs;
        splitCommandLine(args, subCmd, unusedArgs);
        if (subCmd == "CLEAR") {
            app::clearConveyorProgram1AbortRecoveryRequired("P1REC CLEAR");
            Serial.println("P1REC: abort recovery latch clear requested.");
            return true;
        }
        Serial.print("P1REC: recovery_required=");
        Serial.println(app::conveyorProgram1AbortRecoveryRequired() ? "yes" : "no");
        return true;
    }

    if (cmd == "INFEED") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);
        if (subCmd != "EMU") {
            Serial.println("Usage: INFEED EMU <ON|OFF|PASS|REALISTIC|STATUS>");
            return true;
        }

        String emuCmd;
        String emuArgs;
        splitCommandLine(subArgs, emuCmd, emuArgs);

        if (emuCmd.isEmpty() || emuCmd == "H" || emuCmd == "HELP") {
            printInfeedEmuUsage();
            return true;
        }

        if (emuCmd == "STATUS" || emuCmd == "STATE") {
            printInfeedEmuStatus();
            return true;
        }

        if (emuCmd == "OFF" || emuCmd == "STOP") {
            groups::infeed::stopPlatePassEmulation();
            return true;
        }

        if (emuCmd == "ON" || emuCmd == "RUN") {
            uint32_t values[2] = {0U, 0U};
            size_t count = 0;
            if (!parseInfeedEmuArgs(emuArgs, values, 2, count)) {
                printInfeedEmuUsage();
                return true;
            }

            uint32_t onMs = groups::infeed::getPlatePassEmuDefaultOnMs();
            uint32_t offMs = groups::infeed::getPlatePassEmuDefaultOffMs();
            if (count >= 1U) {
                onMs = values[0];
            }
            if (count >= 2U) {
                offMs = values[1];
            }

            if (!groups::infeed::setPlatePassEmulationAuto(true, onMs, offMs)) {
                printInfeedEmuUsage();
                return true;
            }
            return true;
        }

        if (emuCmd == "PASS") {
            uint32_t values[3] = {0U, 0U, 0U};
            size_t count = 0;
            if (!parseInfeedEmuArgs(emuArgs, values, 3, count) || count < 1U) {
                printInfeedEmuUsage();
                return true;
            }

            const uint32_t passCount = values[0];
            uint32_t onMs = groups::infeed::getPlatePassEmuDefaultOnMs();
            uint32_t offMs = groups::infeed::getPlatePassEmuDefaultOffMs();
            if (count >= 2U) {
                onMs = values[1];
            }
            if (count >= 3U) {
                offMs = values[2];
            }

            if (!groups::infeed::queuePlatePassEmulation(passCount, onMs, offMs)) {
                printInfeedEmuUsage();
                return true;
            }
            return true;
        }

        if (emuCmd == "REALISTIC") {
            String realisticCmd;
            String realisticArgs;
            splitCommandLine(emuArgs, realisticCmd, realisticArgs);

            if (realisticCmd.isEmpty() || realisticCmd == "H" || realisticCmd == "HELP") {
                printInfeedEmuUsage();
                return true;
            }

            if (realisticCmd == "STATUS" || realisticCmd == "STATE") {
                printInfeedEmuStatus();
                return true;
            }

            if (realisticCmd == "OFF" || realisticCmd == "STOP") {
                groups::infeed::setPlatePassEmulationRealisticAuto(false, false);
                return true;
            }

            if (realisticCmd == "ON" || realisticCmd == "RUN") {
                bool startOnSensor = false;
                if (!realisticArgs.isEmpty() && !parseInfeedEmuRealisticStartMode(realisticArgs, startOnSensor)) {
                    printInfeedEmuUsage();
                    return true;
                }

                if (!groups::infeed::setPlatePassEmulationRealisticAuto(true, startOnSensor)) {
                    printInfeedEmuUsage();
                    return true;
                }
                return true;
            }

            Serial.println("Unknown INFEED EMU REALISTIC subcommand. Use: INFEED EMU HELP");
            return true;
        }

        Serial.println("Unknown INFEED EMU subcommand. Use: INFEED EMU HELP");
        return true;
    }

    if (cmd == "P1OT" || cmd == "1OT") {
        uint32_t cyclesRaw = core::g_settings.otvodCycleDefaultTotal;
        uint32_t stepsRaw = core::g_settings.step2DivertSteps;

        if (!parseOtcycleStartArgs(args, "P1OT 3 920", cyclesRaw, stepsRaw)) {
            return true;
        }

        if (program1GetStateCode() != 0U) {
            Serial.println("P1OT: программа 1 уже выполняется.");
            return true;
        }
        if (!program1IsShiftCalibrated()) {
            Serial.println("P1OT: сначала выполни CZ, чтобы откалибровать сдвиг.");
            return true;
        }
        if (program1IsSystemBusy()) {
            Serial.println("P1OT: отказ, система уже выполняет движение.");
            return true;
        }
        if (app::manual::isConveyorBusy()) {
            Serial.println("P1OT: конвейер занят, общий запуск запрещен.");
            return true;
        }

        if (!groups::outfeed::startOtvodCycle(
                static_cast<uint8_t>(cyclesRaw), stepsRaw)) {
            return true;
        }

        Serial.println("P1OT: общий запуск OTCYCLE + 1.");
        app::manual::startProgram1();
        if (program1GetStateCode() == 0U) {
            groups::outfeed::abortOtvodCycle("P1OT: программа 1 не стартовала, OTCYCLE отменен.", true);
        }
        return true;
    }

    if (cmd == "MON" || cmd == "MONITOR") {
        String subCmd;
        String unusedArgs;
        splitCommandLine(args, subCmd, unusedArgs);

        if (subCmd.isEmpty()) {
            const bool enabled = app::toggleBoardMonitorStream();
            Serial.println(enabled ? "Board monitor ON." : "Board monitor OFF.");
            if (enabled) {
                app::printBoardMonitorSnapshot();
            }
            return true;
        }

        if (subCmd == "ON") {
            app::setBoardMonitorStreamEnabled(true);
            Serial.println("Board monitor ON.");
            app::printBoardMonitorSnapshot();
            return true;
        }

        if (subCmd == "OFF" || subCmd == "STOP") {
            app::setBoardMonitorStreamEnabled(false);
            Serial.println("Board monitor OFF.");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "NOW" || subCmd == "SHOW") {
            app::printBoardMonitorSnapshot();
            return true;
        }

        if (subCmd == "HELP" || subCmd == "H") {
            Serial.println("MONITOR commands:");
            Serial.println("  MONITOR         - вкл/выкл поток loop/heap/reset раз в 1 сек");
            Serial.println("  MONITOR ON      - включить поток");
            Serial.println("  MONITOR OFF     - выключить поток");
            Serial.println("  MONITOR STATUS  - вывести 1 строку loop/heap/reset");
            return true;
        }

        Serial.println("Usage: MONITOR | MONITOR ON | MONITOR OFF | MONITOR STATUS");
        return true;
    }

    if (cmd == "SEAL") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (app::manual::isOtvodCycleActive() &&
            !(subCmd.isEmpty() ||
              subCmd == "H" || subCmd == "HELP" ||
              subCmd == "STATUS" || subCmd == "STATE" ||
              subCmd == "EMU")) {
            Serial.println("OTCYCLE: active, only SEAL STATUS and SEAL EMU are allowed.");
            return true;
        }

        if (subCmd.isEmpty() || subCmd == "HELP" || subCmd == "H") {
            Serial.println("SEAL commands:");
            Serial.println("  SEAL START [ms]");
            Serial.println("  SEAL STATUS");
            Serial.println("  SEAL OUT ON");
            Serial.println("  SEAL OUT OFF");
            Serial.println("  SEAL EMU ONCE [delay_ms] [hold_ms]");
            Serial.println("  SEAL EMU AUTO ON [delay_ms] [hold_ms]");
            Serial.println("  SEAL EMU AUTO OFF");
            Serial.println("  SEAL EMU STATUS");
            Serial.println("  SEAL EMU CANCEL");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "STATE") {
            app::manual::printSealStatus();
            return true;
        }

        if (subCmd == "EMU") {
            String emuCmd;
            String emuArgs;
            splitCommandLine(subArgs, emuCmd, emuArgs);

            if (emuCmd.isEmpty() || emuCmd == "HELP" || emuCmd == "H") {
                Serial.println("SEAL EMU commands:");
                Serial.println("  SEAL EMU ONCE [delay_ms] [hold_ms]");
                Serial.println("  SEAL EMU AUTO ON [delay_ms] [hold_ms]");
                Serial.println("  SEAL EMU AUTO OFF");
                Serial.println("  SEAL EMU STATUS");
                Serial.println("  SEAL EMU CANCEL");
                Serial.print("Defaults: delay_ms=");
                Serial.print(groups::sealer::getDoneEmuDelayDefaultMs());
                Serial.print(", hold_ms=");
                Serial.println(groups::sealer::getDoneEmuHoldDefaultMs());
                printSealEmuOnceUsage();
                return true;
            }

            if (emuCmd == "STATUS" || emuCmd == "STATE") {
                printSealEmuStatus();
                return true;
            }

            if (emuCmd == "AUTO") {
                String autoCmd;
                String autoArgs;
                splitCommandLine(emuArgs, autoCmd, autoArgs);

                if (autoCmd.isEmpty() || autoCmd == "STATUS" || autoCmd == "STATE") {
                    printSealEmuStatus();
                    return true;
                }

                if (autoCmd == "OFF" || autoCmd == "STOP") {
                    if (!groups::sealer::setAutoDoneEmulation(
                            false,
                            groups::sealer::getAutoDoneEmulationDelayMs(),
                            groups::sealer::getAutoDoneEmulationHoldMs())) {
                        printSealEmuOnceUsage();
                    }
                    return true;
                }

                if (autoCmd == "ON" || autoCmd == "RUN") {
                    uint32_t delayMs = groups::sealer::getAutoDoneEmulationDelayMs();
                    uint32_t holdMs = groups::sealer::getAutoDoneEmulationHoldMs();
                    if (!parseSealEmuOnceArgs(autoArgs, delayMs, holdMs)) {
                        printSealEmuOnceUsage();
                        return true;
                    }
                    if (!groups::sealer::setAutoDoneEmulation(true, delayMs, holdMs)) {
                        printSealEmuOnceUsage();
                    }
                    return true;
                }

                Serial.println("Usage: SEAL EMU AUTO <ON [delay_ms] [hold_ms]|OFF|STATUS>");
                return true;
            }

            if (emuCmd == "CANCEL" || emuCmd == "STOP") {
                if (!groups::sealer::cancelDoneEmulation()) {
                    Serial.println("SEAL EMU: nothing to cancel.");
                }
                return true;
            }

            if (emuCmd == "ONCE" || emuCmd == "RUN") {
                uint32_t delayMs = groups::sealer::getDoneEmuDelayDefaultMs();
                uint32_t holdMs = groups::sealer::getDoneEmuHoldDefaultMs();
                if (!parseSealEmuOnceArgs(emuArgs, delayMs, holdMs)) {
                    printSealEmuOnceUsage();
                    return true;
                }

                const groups::sealer::SealerStatus status = groups::sealer::readSealerStatus();
                if (status.runState != groups::sealer::SealerRunState::WaitDone) {
                    Serial.println("SEAL EMU ONCE rejected: SEAL is not waiting DONE (WaitDone).");
                    return true;
                }
                if (status.doneSyntheticPending || status.doneSyntheticHoldActive) {
                    Serial.println("SEAL EMU ONCE ignored: previous ONCE is still active.");
                    return true;
                }

                if (!groups::sealer::scheduleDoneEmulationOnce(delayMs, holdMs)) {
                    printSealEmuOnceUsage();
                }
                return true;
            }

            Serial.println("Unknown SEAL EMU subcommand. Use: SEAL EMU HELP");
            return true;
        }

        if (subCmd == "OUT") {
            String mode;
            String unused;
            splitCommandLine(subArgs, mode, unused);
            if (mode == "ON") {
                groups::sealer::setStartOutput(true);
                Serial.println("SEAL OUT ON");
                return true;
            }
            if (mode == "OFF") {
                groups::sealer::setStartOutput(false);
                Serial.println("SEAL OUT OFF");
                return true;
            }
            Serial.println("Usage: SEAL OUT <ON|OFF>");
            return true;
        }

        if (subCmd == "START" || subCmd == "RUN") {
            uint32_t pulseMs = groups::sealer::getStartPulseDefaultMs();
            if (!subArgs.isEmpty()) {
                if (!parseUnsigned(subArgs, pulseMs)) {
                    Serial.println("Usage: SEAL START [50..5000]");
                    return true;
                }
            }

            if (groups::sealer::isStartPulseActive()) {
                Serial.println("SEAL START ignored: pulse already active.");
                return true;
            }

            if (!groups::sealer::startPulse(pulseMs)) {
                Serial.println("Usage: SEAL START [50..5000]");
                return true;
            }

            app::manual::printSealStartPulseStarted(pulseMs);
            return true;
        }

        Serial.println("Unknown SEAL subcommand. Use: SEAL HELP");
        return true;
    }

    if (cmd == "POST7") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (subCmd.isEmpty() || subCmd == "HELP" || subCmd == "H") {
            Serial.println("POST7 commands:");
            Serial.println("  POST7 START LOAD        - production start post-step7: local SEAL + OUTFEED");
            Serial.println("                           обычный цикл после step7: COM10 сам запускает");
            Serial.println("                           локальную post-step7 связку SEAL + OUTFEED.");
            Serial.println("  POST7 START UNLOAD_ONLY - production mandatory unload only");
            Serial.println("                           mandatory unload / unload-only: без новой запайки,");
            Serial.println("                           только локальная outfeed/post-step7 часть.");
            Serial.println("  POST7 STATUS            - production status for post-step7");
            Serial.println("                           показать текущий статус локального POST7 job:");
            Serial.println("                           active, mode, phase, outfeed_ready, sealer seq, recovery.");
            Serial.println("  POST7 STOP              - production abort post-step7");
            Serial.println("                           аварийно остановить текущий POST7 job.");
            Serial.println("                           Latch manual recovery required поднимается");
            Serial.println("                           только в фазе seal-in-flight.");
            Serial.println("  POST7 RECOVERY CLEAR    - service clear manual recovery interlock");
            Serial.println("                           сервисный сброс latch manual recovery required");
            Serial.println("                           после ручного восстановления механики.");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "STATE") {
            app::post7::printPost7Status();
            return true;
        }

        if (subCmd == "STOP") {
            (void)app::post7::stopPost7Job("POST7 STOP command");
            return true;
        }

        if (subCmd == "RECOVERY") {
            String action;
            String extra;
            splitCommandLine(subArgs, action, extra);
            if (action == "CLEAR" && extra.isEmpty()) {
                (void)app::post7::clearManualRecoveryLatch("POST7 RECOVERY CLEAR command");
                return true;
            }
            Serial.println("Usage: POST7 RECOVERY CLEAR");
            return true;
        }

        if (subCmd == "START" || subCmd == "RUN") {
            String mode;
            String extra;
            splitCommandLine(subArgs, mode, extra);
            if (!extra.isEmpty()) {
                Serial.println("Usage: POST7 START <LOAD|UNLOAD_ONLY>");
                return true;
            }
            if (mode.isEmpty()) {
                mode = "LOAD";
            }

            if (mode == "LOAD") {
                (void)app::post7::startPost7Load();
                return true;
            }
            if (mode == "UNLOAD_ONLY") {
                (void)app::post7::startPost7UnloadOnly();
                return true;
            }

            Serial.println("Usage: POST7 START <LOAD|UNLOAD_ONLY>");
            return true;
        }

        Serial.println("Unknown POST7 subcommand. Use: POST7 HELP");
        return true;
    }

    if (cmd == "OTCYCLE") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (subCmd.isEmpty() || subCmd == "HELP" || subCmd == "H") {
            Serial.println("OTCYCLE commands:");
            Serial.println("  OTCYCLE START [cycles] [steps]");
            Serial.println("  OTCYCLE STATUS");
            Serial.println("  OTCYCLE STOP");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "STATE") {
            groups::outfeed::printOtvodCycleStatus();
            return true;
        }

        if (subCmd == "STOP") {
            if (!app::manual::isOtvodCycleActive()) {
                Serial.println("OTCYCLE: не выполняется.");
                return true;
            }
            groups::outfeed::abortOtvodCycle("OTCYCLE: остановлено пользователем.", true);
            return true;
        }

        if (subCmd == "START" || subCmd == "RUN") {
            uint32_t cyclesRaw = core::g_settings.otvodCycleDefaultTotal;
            uint32_t stepsRaw = core::g_settings.step2DivertSteps;

            if (!parseOtcycleStartArgs(subArgs, "OTCYCLE START 3 920", cyclesRaw, stepsRaw)) {
                return true;
            }
            (void)groups::outfeed::startOtvodCycle(
                static_cast<uint8_t>(cyclesRaw), stepsRaw);
            return true;
        }

        Serial.println("Format error. Use: OTCYCLE START [cycles] [steps] | STATUS | STOP");
        return true;
    }

    if (cmd == "R1ON") {
        if (app::manual::isOtvodCycleActive()) {
            Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, СЂСѓС‡РЅРѕРµ РІРєР»СЋС‡РµРЅРёРµ СЂРµР»Рµ Р·Р°РїСЂРµС‰РµРЅРѕ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
            return true;
        }
        groups::outfeed::setVfdRelayOutput(true);
        Serial.println("R1: включено.");
        return true;
    }

    if (cmd == "R1OFF") {
        if (app::manual::isOtvodCycleActive()) {
            groups::outfeed::abortOtvodCycle("OTCYCLE: остановлено командой R1OFF.", true);
            return true;
        }
        if (groups::outfeed::isVfdTimedRunActive()) {
            groups::outfeed::stopVfdTimedRun("ручной стоп");
            return true;
        }
        groups::outfeed::setVfdRelayOutput(false);
        Serial.println("R1: выключено.");
        return true;
    }

    if (cmd == "VFD5MIN") {
        if (app::manual::isOtvodCycleActive()) {
            Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, VFD5MIN Р·Р°РїСЂРµС‰РµРЅ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
            return true;
        }
        if (!args.isEmpty()) {
            Serial.println("Format error. Example: VFD5MIN");
            return true;
        }
        groups::outfeed::startVfdTimedRun(groups::outfeed::getVfdRelayTestDurationMs());
        return true;
    }

    if (cmd == "VFDSTOP") {
        if (app::manual::isOtvodCycleActive()) {
            groups::outfeed::abortOtvodCycle("OTCYCLE: остановлено командой VFDSTOP.", true);
            return true;
        }
        if (!groups::outfeed::isVfdTimedRunActive()) {
            groups::outfeed::setVfdRelayOutput(false);
            Serial.println("VFD: реле уже выключено.");
            return true;
        }
        groups::outfeed::stopVfdTimedRun("команда VFDSTOP");
        return true;
    }

    if (cmd == "VFDTICK") {
        String subCmd;
        String subArgs;
        splitCommandLine(args, subCmd, subArgs);

        if (subCmd.isEmpty() || subCmd == "SHOW" || subCmd == "GET") {
            if (!subArgs.isEmpty()) {
                Serial.println("Format error. Example: VFDTICK SHOW");
                return true;
            }
            app::manual::printVfdTickSetting();
            return true;
        }

        if (subCmd == "RUN") {
            if (app::manual::isOtvodCycleActive()) {
                Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, СЂСѓС‡РЅРѕР№ VFDTICK RUN Р·Р°РїСЂРµС‰РµРЅ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
                return true;
            }
            if (!subArgs.isEmpty()) {
                Serial.println("Format error. Example: VFDTICK RUN");
                return true;
            }
            Serial.print("VFDTICK: рабочий такт на ");
            Serial.print(static_cast<float>(groups::outfeed::getVfdTickDurationMs()) / 1000.0f, 2);
            Serial.println(" сек.");
            groups::outfeed::startVfdTimedRun(groups::outfeed::getVfdTickDurationMs());
            return true;
        }

        uint32_t durationMs = 0;
        if (!parseSecondsToDurationMs(subArgs, durationMs)) {
            Serial.println("Format error. Example: VFDTICK TEST 2.19");
            return true;
        }

        if (subCmd == "TEST") {
            if (app::manual::isOtvodCycleActive()) {
                Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, VFDTICK TEST Р·Р°РїСЂРµС‰РµРЅ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
                return true;
            }
            Serial.print("VFDTICK: проверка на ");
            Serial.print(static_cast<float>(durationMs) / 1000.0f, 2);
            Serial.println(" сек.");
            groups::outfeed::startVfdTimedRun(durationMs);
            return true;
        }

        if (subCmd == "SAVE") {
            if (app::manual::isOtvodCycleActive()) {
                Serial.println("OTCYCLE: активен, VFDTICK SAVE запрещен до завершения цикла.");
                return true;
            }
            groups::outfeed::setVfdTickDurationMs(durationMs);
            app::manual::saveVfdTickSetting();
            return true;
        }

        Serial.println("Format error. Examples: VFDTICK SHOW | VFDTICK RUN | VFDTICK TEST 2.19 | VFDTICK SAVE 2.19");
        return true;
    }

    if (cmd == "W") {
        app::manual::setFlagUp();
        Serial.println("Flag: UP.");
        return true;
    }

    if (cmd == "MAINSTART") {
        app::manual::startMainContinuousMotion(core::g_settings.continuousStepDelayUs);
        return true;
    }

    if (cmd == "MAINSTOP") {
        if (!app::manual::isMainMotionActive()) {
            Serial.println("MAIN: PUL13 уже остановлен.");
            return true;
        }
        app::manual::stopMotion();
        Serial.println("MAIN: PUL13 остановлен.");
        return true;
    }

    if (cmd == "POSSTART") {
        app::manual::startPositionalContinuousMotion(core::g_settings.continuousStepDelayUs);
        return true;
    }

    if (cmd == "POSSTOP") {
        if (!app::manual::isPositionalMotionActive()) {
            Serial.println("POS: PUL32 уже остановлен.");
            return true;
        }
        app::manual::stopPositionalMotion();
        return true;
    }

    if (cmd == "S") {
        app::manual::setFlagDown();
        Serial.println("Flag: DOWN.");
        return true;
    }

    if (cmd == "E") {
        const bool enabled = app::manual::toggleSensorStream();
        Serial.println(enabled ? "Sensor stream ON." : "Sensor stream OFF.");
        return true;
    }

    if (cmd == "STOP2") {
        if (app::manual::isOtvodCycleActive()) {
            groups::outfeed::abortOtvodCycle("OTCYCLE: остановлено командой STOP2.", true);
            return true;
        }
        if (!app::manual::isStep2MotionActive()) {
            Serial.println("STEP2: тест не запущен.");
            return true;
        }
        groups::outfeed::stopStep2Motion();
        Serial.println("STEP2: остановлено.");
        return true;
    }

    if (cmd == "T2" || cmd == "STEP2") {
        if (app::manual::isOtvodCycleActive()) {
            Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, СЂСѓС‡РЅРѕР№ STEP2 Р·Р°РїСЂРµС‰РµРЅ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
            return true;
        }
        uint32_t steps = core::g_settings.step2DefaultSteps;
        if (!args.isEmpty() && !parseUnsigned(args, steps)) {
            Serial.println("Format error. Example: T2 5000");
            return true;
        }
        groups::outfeed::startStep2Motion(steps, core::g_settings.step2RunDelayUs);
        return true;
    }

    if (cmd == "OTVOD") {
        if (app::manual::isOtvodCycleActive()) {
            Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, СЂСѓС‡РЅРѕР№ OTVOD Р·Р°РїСЂРµС‰РµРЅ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
            return true;
        }
        uint32_t steps = core::g_settings.step2DivertSteps;
        if (!args.isEmpty() && !parseUnsigned(args, steps)) {
            Serial.println("Format error. Example: OTVOD 920");
            return true;
        }
        groups::outfeed::startStep2Motion(steps, core::g_settings.step2RunDelayUs);
        return true;
    }

    if (cmd == "STEP2START") {
        if (app::manual::isOtvodCycleActive()) {
            Serial.println("OTCYCLE: Р°РєС‚РёРІРµРЅ, STEP2START Р·Р°РїСЂРµС‰РµРЅ. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STOP.");
            return true;
        }
        groups::outfeed::startStep2ContinuousMotion(core::g_settings.continuousStepDelayUs);
        return true;
    }

    if (cmd == "STEP2STOP") {
        if (app::manual::isOtvodCycleActive()) {
            groups::outfeed::abortOtvodCycle("OTCYCLE: остановлено командой STEP2STOP.", true);
            return true;
        }
        if (!app::manual::isStep2MotionActive()) {
            Serial.println("STEP2: GPIO19 уже остановлен.");
            return true;
        }
        groups::outfeed::stopStep2Motion();
        Serial.println("STEP2: GPIO19 остановлен.");
        return true;
    }

    if (cmd == "H") {
        printHelp();
        return true;
    }

    return false;
}

} // namespace

void printHelp()
{
    Serial.println("Команды конвейера:");
    Serial.println("  Основной конвейер:");
    Serial.println("    D 200      - ручной ход вправо на 200 мм, задержка 1500 мкс");
    Serial.println("    MAINSTART  - непрерывные импульсы на PUL13, задержка 1000 мкс");
    Serial.println("    MAINSTOP   - остановить PUL13");
    Serial.println("    1          - автоцикл: 2 -> C+(3+Z) -> 2 -> C+(3+Z) -> 2 -> C+Z");
    Serial.println("    2          - рабочий ход: формула 150+34-UT, добег 100 мм");
    Serial.println("    3          - позиционный ход вправо на 184 мм (PUL32), профиль, 1200 мкс");
    Serial.println("    P 300      - позиционный ход вправо на 300 мм (PUL32), профиль, 1000 мкс");
    Serial.println("    POSSTART   - непрерывные импульсы на PUL32, задержка 1000 мкс");
    Serial.println("    POSSTOP    - остановить PUL32");
    Serial.println("  Сдвиг тарелки:");
    Serial.println("    C          - сдвиг к C: 500 шагов, DIR14/PUL23, задержка 500 мкс");
    Serial.println("    Z          - возврат к Z: 500 шагов, DIR14/PUL23, задержка 300 мкс");
    Serial.println("    CZ         - калибровка хода по герконам Z=GPIO33 и C=GPIO25");
    Serial.println("  Отводной 2-ручейковый конвейер:");
    Serial.println("    T2 [steps] - STEP2 на GPIO19, по умолчанию 5000 шагов, задержка 1000 мкс");
    Serial.println("    OTVOD [steps] - STEP2 на GPIO19, по умолчанию 920 шагов");
    Serial.println("    STEP2START - непрерывные импульсы на GPIO19, задержка 1000 мкс");
    Serial.println("    STEP2STOP  - остановить GPIO19");
    Serial.println("    STOP2      - остановить STEP2");
    Serial.println("    OTCYCLE START [cycles] [steps] - локальный цикл отвода: OTVOD -> основной, по умолчанию 3x920");
    Serial.println("    P1OT [cycles] [steps] - одновременно запустить 1 и OTCYCLE START, по умолчанию 3x920");
    Serial.println("    OTCYCLE STATUS               - показать состояние локального цикла отвода");
    Serial.println("    OTCYCLE STOP                 - остановить локальный цикл отвода");
    Serial.println("    POST7 START LOAD             - production start post-step7 (SEAL + OUTFEED)");
    Serial.println("    POST7 START UNLOAD_ONLY      - production mandatory unload only");
    Serial.println("    POST7 STATUS                 - production status (mode, phase, recovery)");
    Serial.println("    POST7 STOP                   - production abort post-step7");
    Serial.println("    POST7 RECOVERY CLEAR         - service clear manual recovery interlock");
    Serial.println("  Реле / частотник:");
    Serial.println("    R1ON       - включить реле 1 (GPIO21)");
    Serial.println("    R1OFF      - выключить реле 1");
    Serial.println("    VFD5MIN    - включить реле 1 на 5 минут");
    Serial.println("    VFDSTOP    - выключить реле частотника");
    Serial.println("    VFDTICK SHOW      - показать время прокрутки за 1 такт");
    Serial.println("    VFDTICK RUN       - выполнить 1 такт по сохраненному времени");
    Serial.println("    VFDTICK TEST 2.19 - проверить основной конвейер на 2.19 сек");
    Serial.println("    VFDTICK SAVE 2.19 - записать время прокрутки за 1 такт");
    Serial.println("  Запайщик:");
    Serial.println("    SEAL START [ms] - импульс старта на GPIO22, по умолчанию 300 мс");
    Serial.println("    SEAL STATUS     - показать состояние START/DONE");
    Serial.println("    SEAL OUT ON     - вручную включить старт запайщика");
    Serial.println("    SEAL OUT OFF    - вручную выключить старт запайщика");
    Serial.println("    SEAL EMU ONCE [delay_ms] [hold_ms] - service single-shot DONE emulation (WaitDone only)");
    Serial.println("    SEAL EMU AUTO ON [delay_ms] [hold_ms] - стендовый auto-DONE для каждого WaitDone");
    Serial.println("    SEAL EMU AUTO OFF - выключить стендовую auto-эмуляцию");
    Serial.println("    SEAL EMU STATUS - show DONE emulation state");
    Serial.println("    SEAL EMU CANCEL - cancel pending/active DONE emulation");
    Serial.println("  Флаг и датчики:");
    Serial.println("    W          - флаг вверх");
    Serial.println("    S          - флаг вниз");
    Serial.println("    E          - вкл/выкл поток датчиков: E18 + герконы Z/C, каждые 0.5 сек");
    Serial.println("  Сервис:");
    Serial.println("    H          - помощь");
    Serial.println("    MONITOR    - вкл/выкл поток loop/heap/reset, 1 строка в 1 сек");
    Serial.println("    MONITOR STATUS - вывести 1 строку loop/heap/reset без запуска потока");
    Serial.println("    INFEED EMU ON [on_ms] [off_ms] - auto emulation of E18 plate-pass");
    Serial.println("    INFEED EMU PASS count [on_ms] [off_ms] - burst plate-pass profile");
    Serial.println("    INFEED EMU REALISTIC ON [START_ON_SENSOR] - realistic E18 profile");
    Serial.println("    INFEED EMU REALISTIC OFF|STATUS - stop or show realistic profile");
    Serial.println("    INFEED EMU OFF|STATUS - stop or show generic plate-pass emulation");
    Serial.println("    FSC        - strict-feed sync: IN2 consumed by manipulator");
    Serial.println("    FSINV      - strict-feed invalidate model (service)");
    Serial.println("    P1REC [CLEAR] - show/clear P1 abort recovery latch");
    Serial.println("    I2C status - addr 12, SDA=GPIO16, SCL=GPIO17");
    Serial.println("Команды не чувствительны к регистру.");
}

void handleCommand(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    if (app::pauseContractHandleCommand(line.c_str())) {
        return;
    }

    String cmd;
    String args;
    splitCommandLine(line, cmd, args);

    if (app::manual::isOtvodCycleActive()) {
        const bool commandAllowedDuringOtvodCycle =
            (cmd == "OTCYCLE" || cmd == "H" || cmd == "E" ||
             cmd == "MON" || cmd == "MONITOR" ||
             cmd == "FSC" || cmd == "FSINV" || cmd == "P1REC" || cmd == "INFEED" ||
             cmd == "POST7" ||
             cmd == "SEAL" ||
             cmd == "PAUSE" ||
             cmd == "VFDTICK" || cmd == "R1OFF" || cmd == "VFDSTOP" ||
             cmd == "STOP2" || cmd == "STEP2STOP");
        if (!commandAllowedDuringOtvodCycle) {
            Serial.println("OTCYCLE: С†РёРєР» Р°РєС‚РёРІРµРЅ, РєРѕРјР°РЅРґР° Р·Р°РїСЂРµС‰РµРЅР°. РСЃРїРѕР»СЊР·СѓР№С‚Рµ OTCYCLE STATUS РёР»Рё OTCYCLE STOP.");
            return;
        }
    }

    if (tryHandleDistanceMoveCommand(cmd, args) ||
        tryHandleProgramCommand(cmd) ||
        tryHandleShiftCommand(cmd) ||
        tryHandleServiceCommand(cmd, args)) {
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
