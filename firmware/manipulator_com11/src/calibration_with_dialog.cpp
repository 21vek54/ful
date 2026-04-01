#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "calibration.h"
#include "system_state.h"
#include "recovery.h"
#include "calibration_with_dialog.h"

// Глобальные переменные
extern bool isCalibrated;
extern unsigned long travelDistance;
extern float travelDistanceMM;
extern float stepsPerMM;
extern bool calibrationInProgress;
extern bool isMoving;
extern bool testInProgress;
extern bool workCycleInProgress;

// Функция для диалога подтверждения
bool askCalibrationConfirmation(const char* question, unsigned long timeout_ms) {
    Serial.print(question);
    Serial.println(" (Y/N)");
    Serial.println("Ожидание ответа...");
    
    unsigned long startTime = millis();
    while (millis() - startTime < timeout_ms) {
        if (Serial.available()) {
            char answer = Serial.read();
            answer = toupper(answer);
            
            if (answer == 'Y') {
                Serial.println("Да");
                return true;
            } else if (answer == 'N') {
                Serial.println("Нет");
                return false;
            } else if (answer != '\n' && answer != '\r') {
                Serial.println("Неизвестный ответ. Используйте Y или N.");
            }
        }
        delay(10);
    }
    
    Serial.println("Таймаут. Ответ не получен.");
    return false;
}

// Улучшенная функция калибровки с диалогом
void calibrateDistanceWithDialog() {
    // Проверяем, не выполняется ли уже калибровка
    if (calibrationInProgress) {
        Serial.println("Калибровка уже выполняется!");
        return;
    }
    
    // Проверяем, не занята ли система
    if (isMoving || testInProgress || workCycleInProgress) {
        Serial.println("Система занята. Дождитесь завершения текущих операций.");
        return;
    }
    
    // Если калибровка уже выполнена, спрашиваем, нужно ли выполнить заново
    if (isCalibrated) {
        Serial.println("\n=== КАЛИБРОВКА УЖЕ ВЫПОЛНЕНА ===");
        Serial.print("Текущее расстояние: ");
        Serial.print(travelDistance);
        Serial.println(" шагов");
        
        if (!askCalibrationConfirmation("Хотите выполнить калибровку заново?")) {
            Serial.println("Калибровка отменена.");
            return;
        }
    }
    
    // Проверяем готовность к калибровке
    if (!isReadyForCalibration()) {
        Serial.println("Система не готова к калибровке!");
        return;
    }
    
    // Устанавливаем флаг калибровки
    calibrationInProgress = true;
    
    // Выполняем калибровку
    calibrateDistance();
    
    // Сбрасываем флаг калибровки
    calibrationInProgress = false;
}

// Функция быстрой калибровки (без диалога, для использования из других модулей)
void calibrateDistanceQuick() {
    if (calibrationInProgress) {
        return;
    }
    
    calibrationInProgress = true;
    calibrateDistance();
    calibrationInProgress = false;
}

void calibrateDistanceWithExternalConfirmation() {
    if (calibrationInProgress) {
        Serial.println("Калибровка уже выполняется!");
        return;
    }

    if (isMoving || testInProgress || workCycleInProgress) {
        Serial.println("Система занята. Дождитесь завершения текущих операций.");
        return;
    }

    if (!isReadyForCalibration()) {
        Serial.println("Система не готова к калибровке!");
        return;
    }

    if (isCalibrated) {
        Serial.println("\n=== ПОВТОРНАЯ КАЛИБРОВКА ПО ВНЕШНЕМУ ПОДТВЕРЖДЕНИЮ ===");
        Serial.print("Текущее расстояние: ");
        Serial.print(travelDistance);
        Serial.println(" шагов");
    }

    calibrationInProgress = true;
    calibrateDistance();
    calibrationInProgress = false;
}

// Функция установки ручного расстояния с сохранением
void setManualDistanceWithSave(unsigned long steps, float mm) {
    if (steps == 0) {
        Serial.println("Ошибка: расстояние не может быть 0 шагов");
        return;
    }
    
    travelDistance = steps;
    travelDistanceMM = mm;
    
    if (mm > 0) {
        stepsPerMM = (float)steps / mm;
    }
    
    isCalibrated = true;
    
    // Сохраняем в EEPROM
    saveSystemParameters();
    
    Serial.println("\n=== РУЧНАЯ КАЛИБРОВКА ===");
    Serial.print("Расстояние: ");
    Serial.print(steps);
    Serial.print(" шагов");
    
    if (mm > 0) {
        Serial.print(" (");
        Serial.print(mm);
        Serial.print(" мм, коэффициент ");
        Serial.print(stepsPerMM, 3);
        Serial.print(" шаг/мм)");
    }
    
    Serial.println("\n✓ Калибровка сохранена");
}
