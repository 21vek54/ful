#include "app/cycle_control.h"

#include <Arduino.h>

#include "app/runtime_log.h"
#include "app/motion_control.h"
#include "core/pins.h"
#include "core/settings.h"
#include "legacy/program1.h"

namespace {

enum class C2State : uint8_t {
    Idle,
    SeekFirstPlate,
    CenterFirstPlate,
    TrackFirstLeaveAndOpen,
    SeekSecondPlate,
    CenterSecondPlate,
    WaitSecondReleaseTiming,
    MoveAfterSecondRelease,
    WaitSecondLeave,
    FinalMove
};

bool g_c2Active = false;
C2State g_c2State = C2State::Idle;
bool g_c2FirstLeaveCaptured = false;
bool g_c2SecondLeaveCaptured = false;
uint32_t g_c2CenterStartStep = 0;
uint32_t g_c2CenterStepsActive = core::g_settings.c2CenterSteps;
uint32_t g_c2FirstFlagDownStep = 0;
uint32_t g_c2FirstLeaveStep = 0;
uint32_t g_c2FirstUTSteps = 0;
uint32_t g_c2SecondFlagDownStep = 0;
uint32_t g_c2SecondLeaveStep = 0;
uint32_t g_c2FinalStartStep = 0;
uint32_t g_c2FinalMoveSteps = 0;
uint32_t g_c2CycleStartStep = 0;
uint32_t g_c2CycleStartMs = 0;
uint32_t g_c2SecondReleaseSteps = 0;

bool shouldLogCycle2Trace()
{
    if (program1GetStateCode() == 0U) {
        return true;
    }
    return app::runtime_log::isDebugEnabled();
}

void printC2Prefix()
{
    Serial.print("C2: ");
    Serial.print(motionTotalSteps() - g_c2CycleStartStep);
    Serial.print(" шагов ");
}

void captureC2FirstLeaveIfNeeded()
{
    if (g_c2FirstLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c2FirstLeaveCaptured = true;
    g_c2FirstLeaveStep = motionTotalSteps();
    g_c2FirstUTSteps = motionTotalSteps() - g_c2FirstFlagDownStep;

    if (shouldLogCycle2Trace()) {
        printC2Prefix();
        Serial.print("1-я тарелка ушла с датчика, UT=");
        Serial.print(g_c2FirstUTSteps);
        Serial.print(" шагов (");
        Serial.print(static_cast<float>(g_c2FirstUTSteps) / static_cast<float>(core::g_settings.pulsesPerMm), 1);
        Serial.println(" мм).");
    }

    uint32_t releaseWindowSteps = 0;
    if (g_c2FirstUTSteps < core::g_settings.c2ReleaseTargetSteps) {
        releaseWindowSteps = core::g_settings.c2ReleaseTargetSteps - g_c2FirstUTSteps;
    }
    g_c2SecondReleaseSteps = releaseWindowSteps;
}

void captureC2SecondLeaveIfNeeded()
{
    if (g_c2SecondLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c2SecondLeaveCaptured = true;
    g_c2SecondLeaveStep = motionTotalSteps();
    if (shouldLogCycle2Trace()) {
        printC2Prefix();
        Serial.println("2-я тарелка ушла с датчика.");
    }
}

void scheduleC2FinalMove()
{
    g_c2FinalMoveSteps = core::g_settings.c2FinalAfterSecondLeaveSteps;
    g_c2FinalStartStep = motionTotalSteps();
    g_c2State = C2State::FinalMove;
    armMotionStopAfterSteps(g_c2FinalMoveSteps);

    if (shouldLogCycle2Trace()) {
        printC2Prefix();
        Serial.print("финальный добег ");
        Serial.print(static_cast<float>(g_c2FinalMoveSteps) / static_cast<float>(core::g_settings.pulsesPerMm), 1);
        Serial.println(" мм (минимум для 2-й тарелки).");
    }
}

void startCycle2Internal()
{
    if (motionMainIsActive() || g_c2Active) {
        Serial.println("C2: ошибка, двигатель уже в движении.");
        return;
    }

    const bool plateAlreadyOnSensor = isPlateAtSensorFiltered();

    digitalWrite(PIN_FLAG, core::hw_config::FLAG_UP_LEVEL);
    startMotionProfiled(UINT32_MAX, core::g_settings.c2MoveDelayUs);

    g_c2Active = true;
    g_c2State = plateAlreadyOnSensor ? C2State::CenterFirstPlate : C2State::SeekFirstPlate;
    g_c2FirstLeaveCaptured = false;
    g_c2SecondLeaveCaptured = false;
    g_c2CenterStartStep = motionTotalSteps();
    g_c2CenterStepsActive = core::g_settings.c2CenterSteps;
    g_c2FirstFlagDownStep = motionTotalSteps();
    g_c2FirstLeaveStep = motionTotalSteps();
    g_c2FirstUTSteps = 0;
    g_c2SecondFlagDownStep = motionTotalSteps();
    g_c2SecondLeaveStep = motionTotalSteps();
    g_c2FinalStartStep = motionTotalSteps();
    g_c2FinalMoveSteps = 0;
    g_c2CycleStartStep = motionTotalSteps();
    g_c2CycleStartMs = millis();
    g_c2SecondReleaseSteps = 0;

    if (shouldLogCycle2Trace()) {
        printC2Prefix();
        Serial.println("старт. Цель: 1-я тарелка в упоре, 2-я с зазором 34 мм.");
        if (plateAlreadyOnSensor) {
            printC2Prefix();
            if (g_c2CenterStepsActive == 0U) {
                Serial.println("тарелка уже на датчике, старт без центрирования (флаг поднят).");
            } else {
                Serial.println("тарелка уже на датчике, выполняем стартовое центрирование +20 мм (флаг поднят).");
            }
        }
    }
}

} // namespace

void startCycle2()
{
    startCycle2Internal();
}

bool cycle2IsActive()
{
    return g_c2Active;
}

void processCycle2()
{
    if (!g_c2Active || g_c2State == C2State::Idle) {
        return;
    }

    if (!motionMainIsActive()) {
        if (g_c2State == C2State::FinalMove) {
            g_c2Active = false;
            g_c2State = C2State::Idle;
            const uint32_t cycleDurationMs = millis() - g_c2CycleStartMs;
            if (shouldLogCycle2Trace()) {
                printC2Prefix();
                Serial.print("Движение завершено. время цикла ");
                Serial.print(cycleDurationMs);
                Serial.print(" мс (");
                Serial.print(static_cast<float>(cycleDurationMs) / 1000.0F, 1);
                Serial.println(" с).");
            }
            return;
        }

        g_c2Active = false;
        g_c2State = C2State::Idle;
        printC2Prefix();
        Serial.println("сценарий прерван (двигатель остановлен вне C2).");
        return;
    }

    switch (g_c2State) {
        case C2State::SeekFirstPlate:
            if (isPlateAtSensorFiltered()) {
                g_c2CenterStartStep = motionTotalSteps();
                g_c2State = C2State::CenterFirstPlate;
                if (shouldLogCycle2Trace()) {
                    printC2Prefix();
                    if (g_c2CenterStepsActive == 0U) {
                        Serial.println("1-я тарелка найдена, центрирование пропущено.");
                    } else {
                        Serial.println("1-я тарелка найдена, центрирование +20 мм.");
                    }
                }
            }
            return;

        case C2State::CenterFirstPlate:
            if ((uint32_t)(motionTotalSteps() - g_c2CenterStartStep) >= g_c2CenterStepsActive) {
                digitalWrite(PIN_FLAG, core::hw_config::FLAG_DOWN_LEVEL);
                g_c2FirstFlagDownStep = motionTotalSteps();
                g_c2FirstLeaveCaptured = false;
                g_c2FirstUTSteps = 0;
                g_c2State = C2State::TrackFirstLeaveAndOpen;
                if (shouldLogCycle2Trace()) {
                    printC2Prefix();
                    Serial.println("флаг опущен для 1-й тарелки.");
                }
            }
            return;

        case C2State::TrackFirstLeaveAndOpen:
            captureC2FirstLeaveIfNeeded();

            if ((uint32_t)(motionTotalSteps() - g_c2FirstFlagDownStep) >= core::g_settings.c2FlagReopenSteps) {
                digitalWrite(PIN_FLAG, core::hw_config::FLAG_UP_LEVEL);
                g_c2State = C2State::SeekSecondPlate;
                if (shouldLogCycle2Trace()) {
                    printC2Prefix();
                    Serial.println("флаг поднят, поиск 2-й тарелки.");
                }
            }
            return;

        case C2State::SeekSecondPlate:
            captureC2FirstLeaveIfNeeded();

            if (isPlateAtSensorFiltered()) {
                g_c2CenterStartStep = motionTotalSteps();
                g_c2State = C2State::CenterSecondPlate;
                if (shouldLogCycle2Trace()) {
                    printC2Prefix();
                    if (g_c2CenterStepsActive == 0U) {
                        Serial.println("2-я тарелка найдена, центрирование пропущено.");
                    } else {
                        Serial.println("2-я тарелка найдена, центрирование +20 мм.");
                    }
                }
            }
            return;

        case C2State::CenterSecondPlate:
            captureC2FirstLeaveIfNeeded();

            if ((uint32_t)(motionTotalSteps() - g_c2CenterStartStep) >= g_c2CenterStepsActive) {
                g_c2State = C2State::WaitSecondReleaseTiming;
                if (shouldLogCycle2Trace()) {
                    printC2Prefix();
                    if (g_c2CenterStepsActive == 0U) {
                        Serial.println("2-я тарелка без центрирования, расчет момента отпускания.");
                    } else {
                        Serial.println("2-я тарелка центрирована, расчет момента отпускания.");
                    }
                }
            }
            return;

        case C2State::WaitSecondReleaseTiming:
            captureC2FirstLeaveIfNeeded();

            if (!g_c2FirstLeaveCaptured) {
                return;
            }

            {
                const uint32_t releaseAfterFirstLeaveSteps = g_c2SecondReleaseSteps;

                if ((uint32_t)(motionTotalSteps() - g_c2FirstLeaveStep) >= releaseAfterFirstLeaveSteps) {
                    digitalWrite(PIN_FLAG, core::hw_config::FLAG_DOWN_LEVEL);
                    g_c2SecondFlagDownStep = motionTotalSteps();
                    g_c2SecondLeaveCaptured = false;
                    g_c2State = C2State::MoveAfterSecondRelease;
                    if (shouldLogCycle2Trace()) {
                        printC2Prefix();
                        Serial.println("флаг опущен для 2-й тарелки.");
                    }
                }
            }
            return;

        case C2State::MoveAfterSecondRelease:
            captureC2SecondLeaveIfNeeded();

            if ((uint32_t)(motionTotalSteps() - g_c2SecondFlagDownStep) >= core::g_settings.c2FlagReopenSteps) {
                digitalWrite(PIN_FLAG, core::hw_config::FLAG_UP_LEVEL);

                if (g_c2SecondLeaveCaptured) {
                    scheduleC2FinalMove();
                } else {
                    g_c2State = C2State::WaitSecondLeave;
                    if (shouldLogCycle2Trace()) {
                        printC2Prefix();
                        Serial.println("ждем уход 2-й тарелки с датчика.");
                    }
                }
            }
            return;

        case C2State::WaitSecondLeave:
            captureC2SecondLeaveIfNeeded();
            if (g_c2SecondLeaveCaptured) {
                scheduleC2FinalMove();
            }
            return;

        case C2State::FinalMove:
            return;

        case C2State::Idle:
        default:
            return;
    }
}
