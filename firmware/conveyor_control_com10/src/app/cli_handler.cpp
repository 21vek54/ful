#include "app/cli_handler.h"
#include "app/board_monitor.h"
#include "core/settings.h"
#include "app/manual_runtime.h"

#include <math.h>

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
                static_cast<uint8_t>(cyclesRaw), stepsRaw, app::manual::isConveyorBusy())) {
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

        if (subCmd.isEmpty() || subCmd == "HELP" || subCmd == "H") {
            Serial.println("SEAL commands:");
            Serial.println("  SEAL START [ms]");
            Serial.println("  SEAL STATUS");
            Serial.println("  SEAL OUT ON");
            Serial.println("  SEAL OUT OFF");
            return true;
        }

        if (subCmd == "STATUS" || subCmd == "STATE") {
            app::manual::printSealStatus();
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
                static_cast<uint8_t>(cyclesRaw), stepsRaw, app::manual::isConveyorBusy());
            return true;
        }

        Serial.println("Format error. Use: OTCYCLE START [cycles] [steps] | STATUS | STOP");
        return true;
    }

    if (cmd == "R1ON") {
        if (app::manual::isOtvodCycleActive()) {
            Serial.println("OTCYCLE: активен, ручное включение реле запрещено. Используйте OTCYCLE STOP.");
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
            Serial.println("OTCYCLE: активен, VFD5MIN запрещен. Используйте OTCYCLE STOP.");
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
                Serial.println("OTCYCLE: активен, ручной VFDTICK RUN запрещен. Используйте OTCYCLE STOP.");
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
                Serial.println("OTCYCLE: активен, VFDTICK TEST запрещен. Используйте OTCYCLE STOP.");
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
            Serial.println("OTCYCLE: активен, ручной STEP2 запрещен. Используйте OTCYCLE STOP.");
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
            Serial.println("OTCYCLE: активен, ручной OTVOD запрещен. Используйте OTCYCLE STOP.");
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
            Serial.println("OTCYCLE: активен, STEP2START запрещен. Используйте OTCYCLE STOP.");
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
    Serial.println("  Флаг и датчики:");
    Serial.println("    W          - флаг вверх");
    Serial.println("    S          - флаг вниз");
    Serial.println("    E          - вкл/выкл поток датчиков: E18 + герконы Z/C, каждые 0.5 сек");
    Serial.println("  Сервис:");
    Serial.println("    H          - помощь");
    Serial.println("    MONITOR    - вкл/выкл поток loop/heap/reset, 1 строка в 1 сек");
    Serial.println("    MONITOR STATUS - вывести 1 строку loop/heap/reset без запуска потока");
    Serial.println("    I2C status - addr 12, SDA=GPIO16, SCL=GPIO17");
    Serial.println("Команды не чувствительны к регистру.");
}

void handleCommand(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    String cmd;
    String args;
    splitCommandLine(line, cmd, args);

    if (app::manual::isOtvodCycleActive()) {
        const bool commandAllowedDuringOtvodCycle =
            (cmd == "OTCYCLE" || cmd == "H" || cmd == "E" ||
             cmd == "MON" || cmd == "MONITOR" ||
             cmd == "VFDTICK" || cmd == "R1OFF" || cmd == "VFDSTOP" ||
             cmd == "STOP2" || cmd == "STEP2STOP");
        if (!commandAllowedDuringOtvodCycle) {
            Serial.println("OTCYCLE: цикл активен, команда запрещена. Используйте OTCYCLE STATUS или OTCYCLE STOP.");
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
