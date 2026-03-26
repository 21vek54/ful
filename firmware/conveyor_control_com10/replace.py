import os, re

main_path = 'src/main.cpp'
cli_path = 'src/app/cli_handler.cpp'

with open(main_path, 'r', encoding='utf-8') as f:
    main_content = f.read()

with open(cli_path, 'r', encoding='utf-8') as f:
    cli_content = f.read()

# Add include
if '#include "core/settings.h"' not in main_content:
    main_content = main_content.replace('#include <Arduino.h>', '#include <Arduino.h>\n#include "core/settings.h"')

if '#include "../core/settings.h"' not in cli_content:
    cli_content = cli_content.replace('#include "cli_handler.h"', '#include "cli_handler.h"\n#include "../core/settings.h"')

# Remove constants from main.cpp
constants_to_remove_main = [
    r'constexpr bool FLAG_UP_LEVEL = HIGH;\n',
    r'constexpr bool FLAG_DOWN_LEVEL = LOW;\n',
    r'constexpr bool PULSE_ACTIVE_LEVEL = HIGH;\n',
    r'constexpr uint32_t STEP_PULSE_WIDTH_US = 10;\n',
    r'constexpr uint32_t CONTINUOUS_STEP_DELAY_US = 1000;\n',
    r'constexpr uint32_t POS_RUN_DELAY_US = 1000;\n',
    r'constexpr uint32_t STEP2_RUN_DELAY_US = 1000;\n',
    r'constexpr uint32_t STEP2_DEFAULT_STEPS = 5000;\n',
    r'constexpr uint32_t STEP2_DIVERT_STEPS = 920;\n',
    r'constexpr uint32_t SENSOR_PRINT_INTERVAL_MS = 500;\n',
    r'constexpr uint32_t SENSOR_DEBOUNCE_MS = 80;\n',
    r'constexpr uint32_t MANUAL_MOVE_DELAY_US = 1500; // Фиксированная задержка для команд A/D\n',
    r'constexpr uint32_t MOTION_START_DELAY_MULT_NUM = 2;\n',
    r'constexpr uint32_t MOTION_START_DELAY_MULT_DEN = 1;\n',
    r'constexpr uint32_t MOTION_RAMP_STEPS_DEFAULT = 150;\n',
    r'constexpr uint32_t PULSES_PER_MM = 5;           // 2000 импульсов = 400 мм => 5 имп/мм\n',
    r'constexpr uint32_t C3_COMMAND_DISTANCE_MM = 184U;\n',
    r'constexpr uint32_t C3_COMMAND_STEPS = C3_COMMAND_DISTANCE_MM \* PULSES_PER_MM;\n',
    r'constexpr uint32_t C3_COMMAND_DELAY_US = 1200U;\n',
    r'constexpr uint32_t C2_MOVE_DELAY_US = 1500;\n',
    r'constexpr uint32_t C2_CENTER_STEPS = 0U \* PULSES_PER_MM; // 0 мм, центрирование отключено\n',
    r'constexpr uint32_t C2_FLAG_REOPEN_STEPS = 80U \* PULSES_PER_MM; // 80 мм\n',
    r'constexpr uint32_t C2_PLATE_DIAMETER_MM = 150U;\n',
    r'constexpr uint32_t C2_TARGET_GAP_MM = 34U;\n',
    r'constexpr uint32_t C2_FORMULA_BASE_MM = 150U; // Первая часть формулы 2\n',
    r'constexpr uint32_t C2_FORMULA_GAP_MM = 34U;   // Вторая часть формулы 2\n',
    r'constexpr uint32_t C2_RELEASE_TARGET_STEPS =\n    \(C2_FORMULA_BASE_MM \+ C2_FORMULA_GAP_MM\) \* PULSES_PER_MM;\n',
    r'constexpr uint32_t C2_FINAL_AFTER_SECOND_LEAVE_STEPS = 100U \* PULSES_PER_MM; // 100 мм\n'
]

for pattern in constants_to_remove_main:
    main_content = re.sub(pattern, '', main_content)

# Remove constants from cli_handler.cpp
constants_to_remove_cli = [
    r'constexpr uint32_t MANUAL_MOVE_DELAY_US = 1500U;\n',
    r'constexpr uint32_t CONTINUOUS_STEP_DELAY_US = 1000U;\n',
    r'constexpr uint32_t POS_RUN_DELAY_US = 1000U;\n',
    r'constexpr uint32_t STEP2_RUN_DELAY_US = 1000U;\n',
    r'constexpr uint32_t STEP2_DEFAULT_STEPS = 5000U;\n',
    r'constexpr uint32_t STEP2_DIVERT_STEPS = 920U;\n',
    r'constexpr uint8_t OTVOD_CYCLE_DEFAULT_TOTAL = 3U;\n',
    r'constexpr uint32_t PULSES_PER_MM = 5U;\n',
    r'constexpr uint32_t C3_COMMAND_DISTANCE_MM = 184U;\n',
    r'constexpr uint32_t C3_COMMAND_STEPS = C3_COMMAND_DISTANCE_MM \* PULSES_PER_MM;\n',
    r'constexpr uint32_t C3_COMMAND_DELAY_US = 1200U;\n'
]

for pattern in constants_to_remove_cli:
    cli_content = re.sub(pattern, '', cli_content)

# Replacements in main.cpp
replacements_main = {
    'FLAG_UP_LEVEL': 'core::hw_config::FLAG_UP_LEVEL',
    'FLAG_DOWN_LEVEL': 'core::hw_config::FLAG_DOWN_LEVEL',
    'PULSE_ACTIVE_LEVEL': 'core::hw_config::PULSE_ACTIVE_LEVEL',
    'STEP_PULSE_WIDTH_US': 'core::hw_config::STEP_PULSE_WIDTH_US',
    'CONTINUOUS_STEP_DELAY_US': 'core::g_settings.continuousStepDelayUs',
    'POS_RUN_DELAY_US': 'core::g_settings.posRunDelayUs',
    'STEP2_RUN_DELAY_US': 'core::g_settings.step2RunDelayUs',
    'STEP2_DEFAULT_STEPS': 'core::g_settings.step2DefaultSteps',
    'STEP2_DIVERT_STEPS': 'core::g_settings.step2DivertSteps',
    'SENSOR_PRINT_INTERVAL_MS': 'core::g_settings.sensorPrintIntervalMs',
    'SENSOR_DEBOUNCE_MS': 'core::g_settings.sensorDebounceMs',
    'MANUAL_MOVE_DELAY_US': 'core::g_settings.manualMoveDelayUs',
    'MOTION_START_DELAY_MULT_NUM': 'core::g_settings.motionStartDelayMultNum',
    'MOTION_START_DELAY_MULT_DEN': 'core::g_settings.motionStartDelayMultDen',
    'MOTION_RAMP_STEPS_DEFAULT': 'core::g_settings.motionRampStepsDefault',
    'PULSES_PER_MM': 'core::g_settings.pulsesPerMm',
    'C3_COMMAND_DISTANCE_MM': 'core::g_settings.c3CommandDistanceMm',
    'C3_COMMAND_STEPS': 'core::g_settings.c3CommandSteps',
    'C3_COMMAND_DELAY_US': 'core::g_settings.c3CommandDelayUs',
    'C2_MOVE_DELAY_US': 'core::g_settings.c2MoveDelayUs',
    'C2_CENTER_STEPS': 'core::g_settings.c2CenterSteps',
    'C2_FLAG_REOPEN_STEPS': 'core::g_settings.c2FlagReopenSteps',
    'C2_PLATE_DIAMETER_MM': 'core::g_settings.c2PlateDiameterMm',
    'C2_TARGET_GAP_MM': 'core::g_settings.c2TargetGapMm',
    'C2_FORMULA_BASE_MM': 'core::g_settings.c2FormulaBaseMm',
    'C2_FORMULA_GAP_MM': 'core::g_settings.c2FormulaGapMm',
    'C2_RELEASE_TARGET_STEPS': 'core::g_settings.c2ReleaseTargetSteps',
    'C2_FINAL_AFTER_SECOND_LEAVE_STEPS': 'core::g_settings.c2FinalAfterSecondLeaveSteps'
}

for old, new in replacements_main.items():
    main_content = re.sub(r'\b' + old + r'\b', new, main_content)

# Replacements in cli_handler.cpp
replacements_cli = {
    'MANUAL_MOVE_DELAY_US': 'core::g_settings.manualMoveDelayUs',
    'CONTINUOUS_STEP_DELAY_US': 'core::g_settings.continuousStepDelayUs',
    'POS_RUN_DELAY_US': 'core::g_settings.posRunDelayUs',
    'STEP2_RUN_DELAY_US': 'core::g_settings.step2RunDelayUs',
    'STEP2_DEFAULT_STEPS': 'core::g_settings.step2DefaultSteps',
    'STEP2_DIVERT_STEPS': 'core::g_settings.step2DivertSteps',
    'OTVOD_CYCLE_DEFAULT_TOTAL': 'core::g_settings.otvodCycleDefaultTotal',
    'PULSES_PER_MM': 'core::g_settings.pulsesPerMm',
    'C3_COMMAND_DISTANCE_MM': 'core::g_settings.c3CommandDistanceMm',
    'C3_COMMAND_STEPS': 'core::g_settings.c3CommandSteps',
    'C3_COMMAND_DELAY_US': 'core::g_settings.c3CommandDelayUs'
}

for old, new in replacements_cli.items():
    cli_content = re.sub(r'\b' + old + r'\b', new, cli_content)

with open(main_path, 'w', encoding='utf-8') as f:
    f.write(main_content)

with open(cli_path, 'w', encoding='utf-8') as f:
    f.write(cli_content)

print('Done')