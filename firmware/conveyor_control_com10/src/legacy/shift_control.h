#pragma once

void shiftHookUpdateMainSensorFilter();
void shiftHookProcessPositionalMotion();
void shiftHookProcessI2cBus();
void shiftHookProcessOutfeedGroup();
void shiftHookProcessSealerGroup();
void shiftHookProcessPost7Supervisor();
bool shiftHookIsMotionActive();
bool shiftHookIsCycle2Active();
bool shiftHookIsPositionalMotionActive();

void shiftInitHardwarePins();
void shiftInitHardwareStates();
void shiftUpdateSensorFilters();
bool shiftIsSensorZTriggered();
bool shiftIsSensorCTriggered();
bool shiftIsCalibrated();
void shiftCalibrateTravel();
void shiftRunMoveCommandC();
void shiftRunMoveCommandZ();
bool shiftRunProgramStageC();
bool shiftRunProgramStageZ();
void homeShiftToZOnStartup();
