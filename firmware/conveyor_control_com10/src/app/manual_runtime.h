#pragma once

#include <Arduino.h>

namespace app::manual {

struct Callbacks {
    bool (*isOtvodCycleActive)() = nullptr;
    bool (*isConveyorBusy)() = nullptr;
    bool (*isMainMotionActive)() = nullptr;
    bool (*isPositionalMotionActive)() = nullptr;
    bool (*isStep2MotionActive)() = nullptr;
    bool (*toggleSensorStream)() = nullptr;
    void (*printSealStatus)() = nullptr;
    void (*printSealStartPulseStarted)(uint32_t pulseMs) = nullptr;
    void (*printVfdTickSetting)() = nullptr;
    void (*saveVfdTickSetting)() = nullptr;
    void (*startConstantMotion)(uint32_t steps, uint32_t delayUs) = nullptr;
    void (*startPositionalProfiledMotion)(uint32_t stepsTotal, uint32_t nominalDelayUs) = nullptr;
    void (*startProgram1)() = nullptr;
    void (*startCycle2)() = nullptr;
    void (*shiftCalibrateTravel)() = nullptr;
    void (*shiftRunMoveCommandC)() = nullptr;
    void (*shiftRunMoveCommandZ)() = nullptr;
    void (*startMainContinuousMotion)(uint32_t delayUs) = nullptr;
    void (*stopMotion)() = nullptr;
    void (*startPositionalContinuousMotion)(uint32_t delayUs) = nullptr;
    void (*stopPositionalMotion)() = nullptr;
    void (*setFlagUp)() = nullptr;
    void (*setFlagDown)() = nullptr;
};

void registerCallbacks(const Callbacks &callbacks);
bool isReady();

bool isOtvodCycleActive();
bool isConveyorBusy();
bool isMainMotionActive();
bool isPositionalMotionActive();
bool isStep2MotionActive();
bool toggleSensorStream();

void printSealStatus();
void printSealStartPulseStarted(uint32_t pulseMs);
void printVfdTickSetting();
void saveVfdTickSetting();
void startConstantMotion(uint32_t steps, uint32_t delayUs);
void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs);
void startProgram1();
void startCycle2();
void shiftCalibrateTravel();
void shiftRunMoveCommandC();
void shiftRunMoveCommandZ();
void startMainContinuousMotion(uint32_t delayUs);
void stopMotion();
void startPositionalContinuousMotion(uint32_t delayUs);
void stopPositionalMotion();
void setFlagUp();
void setFlagDown();

} // namespace app::manual
