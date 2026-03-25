#pragma once

#include <stdint.h>

// Интерфейс подводящей группы (infeed).
// Здесь только граница ответственности для будущей декомпозиции:
// текущая рабочая механика пока остается в main.cpp и legacy.

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

// Будущие точки входа группы infeed.
void initInfeedGroup();
void processInfeedGroup();
InfeedStatus readInfeedStatus();

} // namespace groups::infeed
