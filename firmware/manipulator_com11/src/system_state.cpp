#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "system_state.h"
#include "recovery.h"

// Инициализация глобальных переменных
bool isMoving = false;
bool moveDirection = HIGH;
bool isCalibrated = false;
bool calibrationInProgress = false;
bool workCycleInProgress = false;
// greaseRemovalInProgress теперь определена в grease_removal.cpp
// testInProgress теперь определена в endurance_test.cpp

unsigned long travelDistance = DEFAULT_STEPS;
float travelDistanceMM = DEFAULT_MM;
float stepsPerMM = DEFAULT_STEPS_PER_MM;

void initSystemState() {
    // Загрузка сохраненных значений из EEPROM уже выполнена в recoveryInit()
    Serial.println("Состояние системы инициализировано");
}

void resetSystemState() {
    isMoving = false;
    moveDirection = HIGH;
    isCalibrated = false;
    calibrationInProgress = false;
    workCycleInProgress = false;
    
    // Сброс процедуры удаления смазки
    extern bool greaseRemovalInProgress;
    greaseRemovalInProgress = false;
    
    travelDistance = DEFAULT_STEPS;
    travelDistanceMM = DEFAULT_MM;
    stepsPerMM = DEFAULT_STEPS_PER_MM;
    
    // Сохраняем сброшенное состояние в EEPROM (без лишнего вывода)
    saveSystemParameters();
    
    Serial.println("Состояние системы сброшено до заводских настроек");
}

void emergencyStop() {
    isMoving = false;
    workCycleInProgress = false;
    
    // Остановка процедуры удаления смазки
    extern bool greaseRemovalInProgress;
    greaseRemovalInProgress = false;
    // testInProgress сбрасывается в endurance_test.cpp
    
    Serial.println("\n!!! ЭКСТРЕННАЯ ОСТАНОВКА !!!");
    Serial.println("Все движения остановлены");
    Serial.println("Рабочий цикл прерван");
    Serial.println("Тест отказоустойчивости остановлен");
    Serial.println("Процедура удаления смазки остановлена");
}

// Простая проверка начального состояния без вывода в Serial
bool checkInitialStateSimple() {
    updateAllSensors();
    
    bool rightLimit = isRightLimitPressed();
    bool zUp = isZUp();
    bool gripOpen = isGripOpen();
    
    return (rightLimit && zUp && gripOpen);
}

void printSystemInfo() {
    updateAllSensors();
    
    Serial.println("\n" + String(50, '='));
    Serial.println("ПОЛНАЯ ИНФОРМАЦИЯ О СИСТЕМЕ");
    Serial.println(String(50, '='));
    
    Serial.println("\n=== СОСТОЯНИЕ СИСТЕМЫ ===");
    
    // Калибровка
    Serial.print("Калибровка: ");
    Serial.println(isCalibrated ? "✓ ВЫПОЛНЕНА" : "✗ НЕ ВЫПОЛНЕНА");
    
    if (isCalibrated) {
        Serial.print("Расстояние: ");
        Serial.print(travelDistance);
        Serial.print(" шагов (");
        Serial.print(travelDistanceMM, 1);
        Serial.print(" мм)");
        Serial.print(", коэффициент: ");
        Serial.print(stepsPerMM, 3);
        Serial.println(" шаг/мм");
    }
    
    // Движение
    Serial.print("Движение: ");
    Serial.println(isMoving ? "▶ ВЫПОЛНЯЕТСЯ" : "⏸ ОСТАНОВЛЕНО");
    
    if (isMoving) {
        Serial.print("Направление: ");
        Serial.println(moveDirection == HIGH ? "← ВЛЕВО" : "→ ВПРАВО");
    }
    
    // Процессы
    Serial.print("Рабочий цикл: ");
    Serial.println(workCycleInProgress ? "▶ ВЫПОЛНЯЕТСЯ" : "⏸ ОСТАНОВЛЕН");
    
    // Для testInProgress нужно объявить как extern
    extern bool testInProgress;
    Serial.print("Тест отказоустойчивости: ");
    Serial.println(testInProgress ? "▶ ВЫПОЛНЯЕТСЯ" : "⏸ ОСТАНОВЛЕН");
    
    // Для greaseRemovalInProgress нужно объявить как extern
    extern bool greaseRemovalInProgress;
    Serial.print("Процедура удаления смазки: ");
    Serial.println(greaseRemovalInProgress ? "▶ ВЫПОЛНЯЕТСЯ" : "⏸ ОСТАНОВЛЕНА");
    
    Serial.print("Калибровка: ");
    Serial.println(calibrationInProgress ? "▶ ВЫПОЛНЯЕТСЯ" : "⏸ ОСТАНОВЛЕНА");
    
    // Концевики
    Serial.print("Концевики: ЛЕВЫЙ=");
    Serial.print(isLeftLimitPressed() ? "НАЖАТ" : "ОТПУЩЕН");
    Serial.print(", ПРАВЫЙ=");
    Serial.println(isRightLimitPressed() ? "НАЖАТ" : "ОТПУЩЕН");
    
    // Драйвер
    Serial.print("Драйвер: ");
    if (stepperIsAlarm()) {
        Serial.println("⚠ АВАРИЯ (ALM)");
        Serial.println("  Используйте команду X для аппаратного сброса");
    } else {
        Serial.println("✓ НОРМА");
    }
    
    // Проверка безопасности
    Serial.print("Безопасность: ");
    if (isSystemSafe()) {
        Serial.println("✓ НОРМА");
    } else {
        Serial.println("⚠ НАРУШЕНА!");
    }
    
    // Состояние датчиков пневматики
    Serial.print("Захват: ");
    if (isGripOpen()) {
        Serial.print("ОТКРЫТ");
    } else if (isGripClosed()) {
        Serial.print("ЗАКРЫТ");
    } else {
        Serial.print("ПРОМЕЖУТОЧНО");
    }
    
    Serial.print(" | Вертикаль: ");
    if (isZUp()) {
        Serial.println("ВВЕРХУ");
    } else if (isZDown()) {
        Serial.println("ВНИЗУ");
    } else {
        Serial.println("ПРОМЕЖУТОЧНО");
    }
    
    // Информация о EEPROM
    printRecoveryInfo();
    
    Serial.println("\n=== РЕКОМЕНДАЦИИ ===");
    
    if (!isCalibrated) {
        Serial.println("1. Выполните калибровку (команда K)");
    }
    
    if (!checkInitialStateSimple()) {
        Serial.println("2. Приведите систему в начальное состояние:");
        Serial.println("   - Правый концевик");
        Serial.println("   - Вертикаль вверх (W)");
        Serial.println("   - Захват открыт (Q)");
    }
    
    if (isBothLimitsPressed() || isGripConflict() || isZConflict()) {
        Serial.println("3. Устраните конфликты датчиков!");
    }
    
    if (stepperIsAlarm()) {
        Serial.println("4. Устраните аварию драйвера! (команда X)");
    }
    
    Serial.println(String(50, '='));
}

bool isSystemSafe() {
    if (isBothLimitsPressed()) {
        return false;
    }
    
    if (stepperIsAlarm()) {
        return false;
    }
    
    if (isGripConflict() || isZConflict()) {
        return false;
    }
    
    return true;
}

String getSystemStatusString() {
    extern bool testInProgress;
    extern bool greaseRemovalInProgress;
    
    String status = "SYS:";
    status += isCalibrated ? "CAL:" : "UNCAL:";
    status += isMoving ? "MOV:" : "STP:";
    status += workCycleInProgress ? "WORK:" : "IDLE:";
    status += testInProgress ? "TEST:" : "NOTEST:";
    status += greaseRemovalInProgress ? "GREASE:" : "NOGREASE:";
    status += isLeftLimitPressed() ? "L_P:" : "L_R:";
    status += isRightLimitPressed() ? "R_P:" : "R_R:";
    status += stepperIsAlarm() ? "ALM" : "OK";
    return status;
}