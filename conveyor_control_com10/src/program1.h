#pragma once

bool program1IsShiftCalibrated();
bool program1IsSystemBusy();
bool program1IsCycle2Active();
bool program1IsManualMotionActive();
bool program1IsPositionalMotionActive();
bool program1GetBufferReady();
void program1ConsumeBuffer();
void program1MarkBufferReady();
uint8_t program1GetStateCode();
uint8_t program1GetPassIndex();
bool program1IsBatchReadyForManipulator();
uint16_t program1GetBatchReadySequence();
void program1StartCycle2();
void program1StartPositionalPass();
bool program1RunShiftStageC();
bool program1RunShiftStageZ();

void startProgram1();
void processProgram1();
