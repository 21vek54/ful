#include "app/manual_runtime.h"

namespace app::manual {

namespace {

Callbacks g_callbacks = {};
bool g_callbacksRegistered = false;

void printUnavailable(const __FlashStringHelper *action)
{
    Serial.print(F("MANUAL: runtime action unavailable: "));
    Serial.println(action);
}

bool readBool(bool (*fn)())
{
    if (fn == nullptr) {
        return false;
    }
    return fn();
}

void callVoid(void (*fn)(), const __FlashStringHelper *action)
{
    if (fn == nullptr) {
        printUnavailable(action);
        return;
    }
    fn();
}

void callDelayAction(void (*fn)(uint32_t), uint32_t value, const __FlashStringHelper *action)
{
    if (fn == nullptr) {
        printUnavailable(action);
        return;
    }
    fn(value);
}

void callMotionAction(
    void (*fn)(uint32_t, uint32_t),
    uint32_t first,
    uint32_t second,
    const __FlashStringHelper *action)
{
    if (fn == nullptr) {
        printUnavailable(action);
        return;
    }
    fn(first, second);
}

} // namespace

void registerCallbacks(const Callbacks &callbacks)
{
    g_callbacks = callbacks;
    g_callbacksRegistered = true;
}

bool isReady()
{
    return g_callbacksRegistered;
}

bool isOtvodCycleActive()
{
    return readBool(g_callbacks.isOtvodCycleActive);
}

bool isConveyorBusy()
{
    return readBool(g_callbacks.isConveyorBusy);
}

bool isMainMotionActive()
{
    return readBool(g_callbacks.isMainMotionActive);
}

bool isPositionalMotionActive()
{
    return readBool(g_callbacks.isPositionalMotionActive);
}

bool isStep2MotionActive()
{
    return readBool(g_callbacks.isStep2MotionActive);
}

bool toggleSensorStream()
{
    return readBool(g_callbacks.toggleSensorStream);
}

void printSealStatus()
{
    callVoid(g_callbacks.printSealStatus, F("printSealStatus"));
}

void printSealStartPulseStarted(uint32_t pulseMs)
{
    callDelayAction(
        g_callbacks.printSealStartPulseStarted,
        pulseMs,
        F("printSealStartPulseStarted"));
}

void printVfdTickSetting()
{
    callVoid(g_callbacks.printVfdTickSetting, F("printVfdTickSetting"));
}

void saveVfdTickSetting()
{
    callVoid(g_callbacks.saveVfdTickSetting, F("saveVfdTickSetting"));
}

void startConstantMotion(uint32_t steps, uint32_t delayUs)
{
    callMotionAction(g_callbacks.startConstantMotion, steps, delayUs, F("startConstantMotion"));
}

void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    callMotionAction(
        g_callbacks.startPositionalProfiledMotion,
        stepsTotal,
        nominalDelayUs,
        F("startPositionalProfiledMotion"));
}

void startProgram1()
{
    callVoid(g_callbacks.startProgram1, F("startProgram1"));
}

void startCycle2()
{
    callVoid(g_callbacks.startCycle2, F("startCycle2"));
}

void shiftCalibrateTravel()
{
    callVoid(g_callbacks.shiftCalibrateTravel, F("shiftCalibrateTravel"));
}

void shiftRunMoveCommandC()
{
    callVoid(g_callbacks.shiftRunMoveCommandC, F("shiftRunMoveCommandC"));
}

void shiftRunMoveCommandZ()
{
    callVoid(g_callbacks.shiftRunMoveCommandZ, F("shiftRunMoveCommandZ"));
}

void startMainContinuousMotion(uint32_t delayUs)
{
    callDelayAction(g_callbacks.startMainContinuousMotion, delayUs, F("startMainContinuousMotion"));
}

void stopMotion()
{
    callVoid(g_callbacks.stopMotion, F("stopMotion"));
}

void startPositionalContinuousMotion(uint32_t delayUs)
{
    callDelayAction(
        g_callbacks.startPositionalContinuousMotion,
        delayUs,
        F("startPositionalContinuousMotion"));
}

void stopPositionalMotion()
{
    callVoid(g_callbacks.stopPositionalMotion, F("stopPositionalMotion"));
}

void setFlagUp()
{
    callVoid(g_callbacks.setFlagUp, F("setFlagUp"));
}

void setFlagDown()
{
    callVoid(g_callbacks.setFlagDown, F("setFlagDown"));
}

} // namespace app::manual
