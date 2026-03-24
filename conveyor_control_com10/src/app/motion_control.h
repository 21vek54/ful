#pragma once

#include <stdint.h>

void startMotionProfiled(uint32_t stepsTotal, uint32_t nominalDelayUs);
void armMotionStopAfterSteps(uint32_t stepsToStop);
void stopMotion();
void startConstantMotion(uint32_t steps, uint32_t delayUs);
void startMainContinuousMotion(uint32_t delayUs);
void processMotion();

void stopPositionalMotion();
void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs);
void processPositionalMotion();
void startPositionalContinuousMotion(uint32_t delayUs);

void updateSensorFilter();
bool isPlateAtSensorFiltered();
void processSensorStream();
void printSensorState();
bool cliToggleSensorStream();

bool motionMainIsActive();
bool motionPositionalIsActive();
uint32_t motionTotalSteps();
