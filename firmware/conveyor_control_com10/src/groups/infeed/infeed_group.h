#pragma once

#include <stdint.h>

// Интерфейс подводящей группы (infeed).
// Модуль отдает статус и сервисную эмуляцию прохода тарелки через E18
// через тот же сырой сенсорный путь, который используется в runtime.

namespace groups::infeed {

enum class InfeedRunState : uint8_t {
    Idle = 0,
    Running,
    Positioning
};

struct InfeedStatus {
    bool sensorPlateDetected = false;
    bool shiftCalibrated = false;
    bool ready = false;
    bool busy = false;
    bool alarm = false;
    InfeedRunState runState = InfeedRunState::Idle;
};

enum class InfeedEmuProfile : uint8_t {
    Generic = 0,
    Realistic
};

struct InfeedEmuStatus {
    bool autoEnabled = false;
    bool sequenceActive = false;
    bool rawSensorActive = false;
    InfeedEmuProfile profile = InfeedEmuProfile::Generic;
    bool realisticStartOnSensor = false;
    bool realisticStartPending = false;
    uint32_t passSeq = 0;
    uint32_t pendingPasses = 0;
    uint32_t onMs = 0;
    uint32_t offMs = 0;
    uint32_t phaseElapsedMs = 0;
};

void initInfeedGroup();
void processInfeedGroup();
InfeedStatus readInfeedStatus();
InfeedEmuStatus readInfeedEmuStatus();

bool setPlatePassEmulationAuto(bool enabled, uint32_t onMs, uint32_t offMs);
bool setPlatePassEmulationRealisticAuto(bool enabled, bool startOnSensor);
bool queuePlatePassEmulation(uint32_t count, uint32_t onMs, uint32_t offMs);
void stopPlatePassEmulation();
bool isPlateSensorEmulationRawActive();

uint32_t getPlatePassEmuDefaultOnMs();
uint32_t getPlatePassEmuDefaultOffMs();
uint32_t getPlatePassEmuMinOnMs();
uint32_t getPlatePassEmuMaxOnMs();
uint32_t getPlatePassEmuMinOffMs();
uint32_t getPlatePassEmuMaxOffMs();
uint32_t getPlatePassEmuMaxCount();

} // namespace groups::infeed
