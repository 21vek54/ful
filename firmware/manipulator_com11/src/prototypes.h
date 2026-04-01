[file name]: prototypes.h
[file content begin]
#pragma once

// Этот файл содержит прототипы всех функций для корректной компиляции
// в Arduino IDE, которая может путаться с порядком файлов

// config.h - только define, не требует прототипов

// sensors.h
void initAllSensors();
void updateAllSensors();
bool isLeftLimitPressed();
bool isRightLimitPressed();
bool isGripOpen();
bool isGripClosed();
bool isZUp();
bool isZDown();
bool isLeftLimitPressedRaw();
bool isRightLimitPressedRaw();
bool isGripOpenRaw();
bool isGripClosedRaw();
bool isZUpRaw();
bool isZDownRaw();
bool waitForLeftLimit(bool pressed, unsigned long timeout_ms);
bool waitForRightLimit(bool pressed, unsigned long timeout_ms);
bool waitForGripOpen(unsigned long timeout_ms);
bool waitForGripClosed(unsigned long timeout_ms);
bool waitForZUp(unsigned long timeout_ms);
bool waitForZDown(unsigned long timeout_ms);
bool isBothLimitsPressed();
bool isGripConflict();
bool isZConflict();
bool isAnyConflict();
void printAllSensorsStatus();
void printSensorStatus(ReedSwitch &sensor);

// stepper_motor.h
void stepperInit();
void stepperEnable(bool enable);
void stepperSetDirection(bool direction);
void stepperStep(unsigned long delay_us);
void stepperMoveSteps(unsigned long steps, unsigned long delay_us);
bool stepperIsAlarm();
unsigned long stepperMoveToLimit(bool direction, unsigned long max_steps, unsigned long delay_us);
unsigned long stepperMoveWithSProfile(bool direction, unsigned long plateau_steps);
void stepperHardReset();
bool stepperRecoverFromAlarm();

// pneumatics.h
void pneumoInit();
void pneumoZUp();
void pneumoZDown();
void pneumoGripOpen();
void pneumoGripClose();
bool pneumoZUpSafe(unsigned long timeout_ms);
bool pneumoZDownSafe(unsigned long timeout_ms);
bool pneumoGripOpenSafe(unsigned long timeout_ms);
bool pneumoGripCloseSafe(unsigned long timeout_ms);
bool pneumoZUpSafeEnhanced(unsigned long timeout_ms);
bool pneumoZDownSafeEnhanced(unsigned long timeout_ms);
bool pneumoGripOpenSafeEnhanced(unsigned long timeout_ms);
bool pneumoGripCloseSafeEnhanced(unsigned long timeout_ms);
bool pneumoHandleCommand(char cmd);
void pneumoPrintStatus();
void pneumoSafePosition();

// motion_profiles.h
bool parseSpeedProfile(const String& profile_str, SpeedProfileSegment segments[], int& segment_count);
bool executeSpeedProfile(const String& profile_str, unsigned long total_steps, bool direction, bool require_limit);
bool moveRightProfile(unsigned long total_steps);
bool moveLeftProfile(unsigned long total_steps);
bool moveCustomProfile(const String& profile_str, unsigned long total_steps, bool direction);
bool moveRightSafe(unsigned long timeout_ms);
bool moveLeftSafe(unsigned long timeout_ms);

// calibration.h
void calibrateDistance();
bool checkSwitchAccuracy(bool direction, const char* switchName, unsigned long checkDistance);
unsigned long measureDistanceWithSProfile(bool toRight);
void moveAwayFromSwitch(bool direction, const char* switchName, unsigned long distance);
bool moveToSwitchForCalibration(bool toRight);
void setManualDistance(unsigned long steps, float mm);
unsigned long getCalibratedDistance();
float getStepsPerMM();
bool isReadyForCalibration();
void resetCalibration();

// calibration_with_dialog.h
void calibrateDistanceWithDialog();
void calibrateDistanceQuick();
void setManualDistanceWithSave(unsigned long steps, float mm);
bool askCalibrationConfirmation(const char* question, unsigned long timeout_ms = 10000);

// endurance_test.h
void startEnduranceTest(int cycles);
void stopEnduranceTest();
void updateEnduranceTest();
void completeEnduranceTest(bool success);
bool performTestCycle();
void printTestStatistics();
bool isReadyForTest();
bool runTSequenceOnce();

// system_state.h
void initSystemState();
void resetSystemState();
void emergencyStop();
void printSystemInfo();
bool isSystemSafe();
String getSystemStatusString();
bool checkInitialStateSimple();

// work_cycle.h
bool checkWorkCycleInitialState();
bool executeWorkCycle();
bool startWorkCycle(bool fromRecovery = false);
bool isWorkCycleInProgress();
uint8_t getWorkCycleCurrentStep();
bool isWorkCycleStep7Reached();
bool recoverWorkCycle();
bool verifyStepState(uint8_t step);
bool moveToStep(uint8_t targetStep);
bool executeStep(uint8_t step);
bool slowMoveToRightEnd();
bool slowMoveToLeftEnd();
bool safeRecoveryForStep(uint8_t savedStep);
int getCurrentPosition();
int getRequiredPositionForStep(uint8_t step);
bool slowMoveToSwitch(bool toLeft, unsigned long maxSteps);

// recovery.h
void recoveryInit();
void saveWorkCycleState(uint8_t step, bool interrupted = false);
bool loadWorkCycleState(uint8_t &step, bool &interrupted);
void clearWorkCycleState();
void clearSystemState();
bool hasSavedWorkCycleState();
void saveSystemParameters();
void loadSystemParameters();
void printRecoveryInfo();

// grease_removal.h - ДОБАВЛЕНЫ ЭТИ ПРОТОТИПЫ
void startGreaseRemoval();
void stopGreaseRemoval();
void updateGreaseRemoval();
bool performGreaseMovement();
bool isReadyForGreaseRemoval();
void printGreaseRemovalStatus();
