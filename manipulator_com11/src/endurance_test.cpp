#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "pneumatics.h"
#include "motion_profiles.h"
#include "endurance_test.h"
#include "system_state.h"

// Test state
bool testInProgress = false;
int testCyclesRemaining = 0;
unsigned long testCycleCounter = 0;
unsigned long testStartTime = 0;
unsigned long testLastMovementTime = 0;

// Shared system state
extern bool isCalibrated;
extern unsigned long travelDistance;
extern bool isMoving;
extern bool workCycleInProgress;

bool runTSequenceOnce() {
    if (isMoving || workCycleInProgress) {
        Serial.println("Test sequence unavailable: system is busy");
        return false;
    }

    updateAllSensors();

    if (isBothLimitsPressed()) {
        Serial.println("Test sequence aborted: limit switch conflict");
        return false;
    }

    if (stepperIsAlarm()) {
        Serial.println("Test sequence aborted: stepper driver alarm");
        return false;
    }

    if (!isRightLimitPressed()) {
        Serial.println("Test: returning to right limit...");
        if (!moveRightSafe()) {
            return false;
        }
    }

    Serial.println("Test: moving left...");
    if (!moveLeftSafe()) {
        return false;
    }

    if (TEST_DELAY_BETWEEN_MOVES > 0) {
        delay(TEST_DELAY_BETWEEN_MOVES);
    }

    Serial.println("Test: moving right...");
    if (!moveRightSafe()) {
        return false;
    }

    return true;
}

void startEnduranceTest(int cycles) {
    if (testInProgress) {
        Serial.println("Test is already running");
        return;
    }

    if (cycles <= 0 || cycles > MAX_TEST_CYCLES) {
        Serial.print("Invalid cycle count. Allowed range: 1-");
        Serial.println(MAX_TEST_CYCLES);
        return;
    }

    if (!isReadyForTest()) {
        return;
    }

    testInProgress = true;
    testCyclesRemaining = cycles;
    testCycleCounter = 0;
    testStartTime = millis();
    testLastMovementTime = 0;

    Serial.println("\n=== ENDURANCE TEST STARTED ===");
    Serial.print("Planned cycles: ");
    Serial.println(cycles);
    Serial.println("Sequence: right limit -> left limit -> right limit");
}

void stopEnduranceTest() {
    if (testInProgress) {
        Serial.println("\n!!! TEST STOPPED BY USER !!!");
        completeEnduranceTest(false);
    }
}

void updateEnduranceTest() {
    if (!testInProgress) return;

    if (!isMoving && testCyclesRemaining > 0) {
        performTestCycle();
    }
}

bool performTestCycle() {
    if (!testInProgress || testCyclesRemaining <= 0) {
        return false;
    }

    Serial.print("\n=== Cycle ");
    Serial.print(testCycleCounter + 1);
    Serial.print("/");
    Serial.print(testCycleCounter + testCyclesRemaining);
    Serial.println(" ===");

    bool success = runTSequenceOnce();

    if (!success) {
        Serial.println("!!! TEST SEQUENCE FAILED !!!");
        completeEnduranceTest(false);
        return false;
    }

    testCycleCounter++;
    testCyclesRemaining--;
    testLastMovementTime = millis();

    float progress = (float)testCycleCounter / (testCycleCounter + testCyclesRemaining) * 100.0f;
    float elapsedTime = (millis() - testStartTime) / 1000.0f / 60.0f;

    Serial.print("Progress: ");
    Serial.print(progress, 1);
    Serial.print("% | Time: ");
    Serial.print(elapsedTime, 1);
    Serial.println(" min");

    if (testCyclesRemaining <= 0) {
        completeEnduranceTest(true);
    }

    return true;
}

void completeEnduranceTest(bool success) {
    testInProgress = false;
    unsigned long testEndTime = millis();
    float totalTestTime = (testEndTime - testStartTime) / 1000.0f / 60.0f;

    Serial.println("\n=== TEST COMPLETED ===");

    if (success) {
        Serial.print("OK: completed cycles = ");
        Serial.println(testCycleCounter);
    } else {
        Serial.print("Interrupted after cycles = ");
        Serial.println(testCycleCounter);
    }

    Serial.print("Total time: ");
    Serial.print(totalTestTime, 1);
    Serial.println(" min");

    if (testCycleCounter > 0) {
        float avgCycleTime = totalTestTime * 60.0f / testCycleCounter;
        Serial.print("Average cycle time: ");
        Serial.print(avgCycleTime, 1);
        Serial.println(" sec");
    }

    testCyclesRemaining = 0;
    testCycleCounter = 0;
}

void printTestStatistics() {
    if (!testInProgress && testCycleCounter == 0) {
        Serial.println("Test has not been run");
        return;
    }

    unsigned long currentTime = millis();
    float totalTestTime = (currentTime - testStartTime) / 1000.0f / 60.0f;

    Serial.println("\n=== TEST STATISTICS ===");
    Serial.print("Completed cycles: ");
    Serial.println(testCycleCounter);

    if (testInProgress) {
        Serial.print("Remaining cycles: ");
        Serial.println(testCyclesRemaining);
        Serial.print("Progress: ");
        Serial.print((float)testCycleCounter / (testCycleCounter + testCyclesRemaining) * 100.0f, 1);
        Serial.println("%");
    }

    Serial.print("Total time: ");
    Serial.print(totalTestTime, 1);
    Serial.println(" min");

    if (testCycleCounter > 0) {
        float avgCycleTime = totalTestTime * 60.0f / testCycleCounter;
        Serial.print("Average cycle time: ");
        Serial.print(avgCycleTime, 1);
        Serial.println(" sec");
    }
}

bool isReadyForTest() {
    if (!isCalibrated) {
        Serial.println("Calibration is required");
        return false;
    }

    updateAllSensors();

    if (isBothLimitsPressed()) {
        Serial.println("Both limit switches are active");
        return false;
    }

    if (stepperIsAlarm()) {
        Serial.println("Stepper driver is in alarm state");
        return false;
    }

    return true;
}
