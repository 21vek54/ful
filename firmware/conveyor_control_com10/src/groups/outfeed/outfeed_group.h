#pragma once

#include <Arduino.h>
#include <stdint.h>

// Группа outfeed/VFD.
// Модуль владеет состоянием реле частотника и timed-run логикой
// без callback-hook делегирования.

namespace groups::outfeed {

enum class OutfeedRunState : uint8_t {
    Idle = 0,
    Step2Running,
    MainConveyorRunning,
    LocalCycleRunning
};

struct OutfeedStatus {
    bool step2Active = false;
    bool step2Continuous = false;
    uint32_t step2StepsTotal = 0;
    uint32_t step2StepsDone = 0;
    uint8_t step2CompletionSeq = 0;
    bool vfdRelayActive = false;
    bool vfdTimedRunActive = false;
    bool localCycleActive = false;
    bool ready = false;
    bool busy = false;
    bool alarm = false;
    uint32_t vfdTickDurationMs = 0;
    OutfeedRunState runState = OutfeedRunState::Idle;
};

// Инициализация и периодическая обработка группы.
void initOutfeedGroup();
void processOutfeedGroup();
OutfeedStatus readOutfeedStatus();

// Прямое управление реле VFD.
// setVfdRelayOutput() сбрасывает активный timed run, если он был.
void setVfdRelayOutput(bool enabled);
bool isVfdRelayOutputActive();

// Управление timed run VFD.
void startVfdTimedRun(uint32_t durationMs);
void stopVfdTimedRun(const char *reason);
bool isVfdTimedRunActive();

// Runtime-параметр "1 такт VFD" (ms).
uint32_t getVfdTickDurationMs();
void setVfdTickDurationMs(uint32_t durationMs);

// Управление STEP2 (GPIO19).
void startStep2Motion(uint32_t steps, uint32_t delayUs);
void startStep2ContinuousMotion(uint32_t delayUs);
void stopStep2Motion();
bool isStep2Active();
uint8_t getStep2CompletionSeq();

// Управление локальным циклом отвода (OTCYCLE).
bool startOtvodCycle(uint8_t totalCycles, uint32_t stepSteps, bool conveyorBusy);
void abortOtvodCycle(const String &reason, bool stopOutputs);
bool isOtvodCycleActive();
void printOtvodCycleStatus();

// Константы группы outfeed.
uint32_t getVfdTickDefaultDurationMs();
uint32_t getVfdTickMinDurationMs();
uint32_t getVfdTickMaxDurationMs();
uint32_t getVfdRelayTestDurationMs();
bool getVfdRelayActiveLevel();
bool getStep2PulseActiveLevel();
uint32_t getStep2PulseWidthUs();

} // namespace groups::outfeed
