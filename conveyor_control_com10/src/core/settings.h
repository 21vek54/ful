#pragma once

#include <stdint.h>

namespace core {

struct ConveyorSettings {
    // Delays
    uint32_t manualMoveDelayUs = 1500;
    uint32_t continuousStepDelayUs = 1000;
    uint32_t posRunDelayUs = 1000;
    uint32_t step2RunDelayUs = 1000;
    uint32_t c3CommandDelayUs = 1200;
    uint32_t c2MoveDelayUs = 1500;

    // Steps & Distances
    uint32_t step2DefaultSteps = 5000;
    uint32_t step2DivertSteps = 920;
    uint32_t pulsesPerMm = 5;
    uint32_t c3CommandDistanceMm = 184;
    uint32_t c3CommandSteps = 184 * 5; // C3_COMMAND_DISTANCE_MM * PULSES_PER_MM
    uint32_t c2CenterSteps = 0;
    uint32_t c2FlagReopenSteps = 80 * 5; // 80 * PULSES_PER_MM
    uint32_t c2PlateDiameterMm = 150;
    uint32_t c2TargetGapMm = 34;
    uint32_t c2FormulaBaseMm = 150;
    uint32_t c2FormulaGapMm = 34;
    uint32_t c2ReleaseTargetSteps = (150 + 34) * 5; // (C2_FORMULA_BASE_MM + C2_FORMULA_GAP_MM) * PULSES_PER_MM
    uint32_t c2FinalAfterSecondLeaveSteps = 100 * 5; // 100 * PULSES_PER_MM

    // Other
    uint8_t otvodCycleDefaultTotal = 3;
    uint32_t sensorPrintIntervalMs = 500;
    uint32_t sensorDebounceMs = 80;
    uint32_t motionStartDelayMultNum = 2;
    uint32_t motionStartDelayMultDen = 1;
    uint32_t motionRampStepsDefault = 150;
};

extern ConveyorSettings g_settings;

namespace hw_config {
    constexpr bool FLAG_UP_LEVEL = true; // HIGH
    constexpr bool FLAG_DOWN_LEVEL = false; // LOW
    constexpr bool PULSE_ACTIVE_LEVEL = true; // HIGH
    constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
}

} // namespace core
