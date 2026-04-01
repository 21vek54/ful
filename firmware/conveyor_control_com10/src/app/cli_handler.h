#pragma once

#include <Arduino.h>

// Слой CLI/Serial-команд для conveyor_control_com10.
// Содержит разбор команд и работу с Serial без изменения механики.

void printHelp();
void handleCommand(String line);
void readSerialCommands();
