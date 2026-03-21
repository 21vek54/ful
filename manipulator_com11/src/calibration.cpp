#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "calibration.h"
#include "system_state.h"
#include "recovery.h"

// Глобальные переменные (определены в system_state.cpp)
extern bool isCalibrated;
extern unsigned long travelDistance;
extern float travelDistanceMM;
extern float stepsPerMM;
extern bool calibrationInProgress;
extern bool isMoving;
extern bool testInProgress;
extern bool workCycleInProgress;

// =============================================
// ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ: УСИЛЕННОЕ ОЖИДАНИЕ ОТПУСКАНИЯ КОНЦЕВИКА
// =============================================

bool waitForSwitchRelease(bool isLeftSwitch, unsigned long timeout_ms) {
    Serial.print("  Ожидание отпускания ");
    Serial.print(isLeftSwitch ? "левого" : "правого");
    Serial.println(" концевика...");
    
    unsigned long start_time = millis();
    
    while (millis() - start_time < timeout_ms) {
        updateAllSensors();
        
        if (isLeftSwitch) {
            if (!isLeftLimitPressed()) {
                // Еще раз проверим через 50мс для уверенности
                delay(50);
                updateAllSensors();
                if (!isLeftLimitPressed()) {
                    Serial.println("  ✓ Концевик отпущен");
                    return true;
                }
            }
        } else {
            if (!isRightLimitPressed()) {
                // Еще раз проверим через 50мс для уверенности
                delay(50);
                updateAllSensors();
                if (!isRightLimitPressed()) {
                    Serial.println("  ✓ Концевик отпущен");
                    return true;
                }
            }
        }
        
        delay(10);
    }
    
    Serial.println("  ✗ Таймаут ожидания отпускания");
    return false;
}

// =============================================
// ФУНКЦИЯ КАЛИБРОВКИ
// =============================================

bool moveToSwitchForCalibration(bool toRight) {
    Serial.print("Поиск ");
    Serial.print(toRight ? "правого" : "левого");
    Serial.println(" концевика...");
    
    // Движение к концевику на медленной скорости
    unsigned long steps = 0;
    bool switch_found = false;
    unsigned long start_time = millis();
    
    stepperSetDirection(toRight ? LOW : HIGH);
    
    while (!switch_found && steps < CALIBRATION_MAX_STEPS) {
        updateAllSensors();
        
        if (toRight) {
            if (isRightLimitPressed()) {
                // Проверим стабильность в течение 50мс
                delay(50);
                updateAllSensors();
                if (isRightLimitPressed()) {
                    switch_found = true;
                    break;
                }
            }
        } else {
            if (isLeftLimitPressed()) {
                // Проверим стабильность в течение 50мс
                delay(50);
                updateAllSensors();
                if (isLeftLimitPressed()) {
                    switch_found = true;
                    break;
                }
            }
        }
        
        stepperStep(S_PROFILE_START_DELAY);
        steps++;
        
        // Таймаут
        if (millis() - start_time > 30000) {
            break;
        }
    }
    
    if (switch_found) {
        Serial.print("Найден ");
        Serial.print(toRight ? "правый" : "левый");
        Serial.print(" концевик за ");
        Serial.print(steps);
        Serial.println(" шагов");
        return true;
    } else {
        Serial.print("Не удалось найти ");
        Serial.println(toRight ? "правый концевик" : "левый концевик");
        return false;
    }
}

unsigned long measureDistanceWithSProfile(bool toRight) {
    Serial.print("Ускоренное измерение расстояния к ");
    Serial.print(toRight ? "правому" : "левому");
    Serial.println(" концевику...");
    
    stepperSetDirection(toRight ? LOW : HIGH);
    unsigned long total_steps = 0;
    bool switch_found = false;
    unsigned long start_time = millis();
    
    // Разгон
    for (unsigned long i = 0; i < S_PROFILE_ACCEL_STEPS && !switch_found; i++) {
        updateAllSensors();
        
        // Проверяем концевик каждый шаг
        if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
            // Проверим стабильность в течение 50мс
            delay(50);
            updateAllSensors();
            if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                switch_found = true;
                break;
            }
        }
        
        unsigned long current_delay = map(i, 0, S_PROFILE_ACCEL_STEPS, 
                                          S_PROFILE_START_DELAY, S_PROFILE_MIN_DELAY);
        stepperStep(current_delay);
        total_steps++;
    }
    
    // Полка (максимальная скорость)
    if (!switch_found) {
        unsigned long plateau_remaining = S_PROFILE_PLATEAU_STEPS;
        
        while (plateau_remaining > 0 && !switch_found) {
            updateAllSensors();
            
            if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                // Проверим стабильность в течение 50мс
                delay(50);
                updateAllSensors();
                if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                    switch_found = true;
                    break;
                }
            }
            
            stepperStep(S_PROFILE_MIN_DELAY);
            total_steps++;
            plateau_remaining--;
        }
    }
    
    // Торможение
    if (!switch_found) {
        for (unsigned long i = 0; i < S_PROFILE_DECEL_STEPS && !switch_found; i++) {
            updateAllSensors();
            
            if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                // Проверим стабильность в течение 50мс
                delay(50);
                updateAllSensors();
                if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                    switch_found = true;
                    break;
                }
            }
            
            unsigned long current_delay = map(i, 0, S_PROFILE_DECEL_STEPS,
                                              S_PROFILE_MIN_DELAY, S_PROFILE_START_DELAY);
            stepperStep(current_delay);
            total_steps++;
        }
    }
    
    // Завершение на медленной скорости
    if (!switch_found) {
        while (!switch_found && total_steps < CALIBRATION_MAX_STEPS) {
            updateAllSensors();
            
            if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                // Проверим стабильность в течение 50мс
                delay(50);
                updateAllSensors();
                if ((toRight && isRightLimitPressed()) || (!toRight && isLeftLimitPressed())) {
                    switch_found = true;
                    break;
                }
            }
            
            stepperStep(S_PROFILE_START_DELAY);
            total_steps++;
        }
    }
    
    if (switch_found) {
        Serial.print("Найден ");
        Serial.print(toRight ? "правый" : "левый");
        Serial.print(" концевик за ");
        Serial.print(total_steps);
        Serial.println(" шагов");
        
        return total_steps;
    } else {
        Serial.print("!!! ");
        Serial.print(toRight ? "Правый" : "Левый");
        Serial.println(" концевик не найден!");
        return 0;
    }
}

void calibrateDistance() {
    // Проверяем, не занята ли система другими процессами
    if (isMoving || testInProgress || workCycleInProgress) {
        Serial.println("Ошибка: система занята! Дождитесь завершения текущих операций.");
        return;
    }
    
    if (isCalibrated && !calibrationInProgress) {
        Serial.println("Калибровка уже выполнена. Используйте команду K для перекалибровки.");
        return;
    }
    
    Serial.println("\n=== КАЛИБРОВКА ===");
    Serial.println("Определение расстояния между концевиками...");
    
    updateAllSensors();
    
    // Проверяем начальное положение
    if (isBothLimitsPressed()) {
        Serial.println("ОШИБКА: Оба концевика нажаты! Проверьте механику.");
        return;
    }
    
    unsigned long measured_distance = 0;
    bool calibration_direction = false; // false: слева направо, true: справа налево
    
    if (isLeftLimitPressed()) {
        // Начинаем с левого концевика
        Serial.println("Начальная позиция: левый концевик");
        
        // 1. Отъезжаем от левого концевика на 1000 шагов
        Serial.println("Отъезд от левого концевика на 1000 шагов...");
        stepperSetDirection(LOW); // Вправо
        
        for (unsigned long i = 0; i < 1000; i++) {
            stepperStep(S_PROFILE_START_DELAY);
        }
        
        Serial.println("  ✓ Отъехали на 1000 шагов");
        delay(100);
        
        // 2. Ждем отпускания левого концевика
        if (!waitForSwitchRelease(true, 5000)) {
            Serial.println("ОШИБКА: левый концевик не отпускается!");
            return;
        }
        
        // 3. Возвращаемся и ищем левый концевик
        Serial.print("  Поиск левого концевика...");
        stepperSetDirection(HIGH); // Влево
        bool left_found = false;
        unsigned long steps_back = 0;
        
        while (!left_found && steps_back < 2000) {
            updateAllSensors();
            if (isLeftLimitPressed()) {
                // Двойная проверка стабильности
                delay(50);
                updateAllSensors();
                if (isLeftLimitPressed()) {
                    left_found = true;
                    break;
                }
            }
            stepperStep(S_PROFILE_START_DELAY);
            steps_back++;
        }
        
        if (left_found) {
            Serial.print(" найден за ");
            Serial.print(steps_back);
            Serial.println(" шагов");
            
            // Если нашли за 0 шагов - это ошибка!
            if (steps_back == 0) {
                Serial.println("ОШИБКА: концевик найден сразу! Механическая проблема или ложное срабатывание.");
                return;
            }
        } else {
            Serial.println(" не найден");
            return;
        }
        
        delay(200);
        
        // 4. Измеряем расстояние до правого
        calibration_direction = false; // Слева направо
        measured_distance = measureDistanceWithSProfile(true);
        
    } else if (isRightLimitPressed()) {
        // Начинаем с правого концевика
        Serial.println("Начальная позиция: правый концевик");
        
        // 1. Отъезжаем от правого концевика на 1000 шагов
        Serial.println("Отъезд от правого концевика на 1000 шагов...");
        stepperSetDirection(HIGH); // Влево
        
        for (unsigned long i = 0; i < 1000; i++) {
            stepperStep(S_PROFILE_START_DELAY);
        }
        
        Serial.println("  ✓ Отъехали на 1000 шагов");
        delay(100);
        
        // 2. Ждем отпускания правого концевика
        if (!waitForSwitchRelease(false, 5000)) {
            Serial.println("ОШИБКА: правый концевик не отпускается!");
            return;
        }
        
        // 3. Возвращаемся и ищем правый концевик
        Serial.print("  Поиск правого концевика...");
        stepperSetDirection(LOW); // Вправо
        bool right_found = false;
        unsigned long steps_back = 0;
        
        while (!right_found && steps_back < 2000) {
            updateAllSensors();
            if (isRightLimitPressed()) {
                // Двойная проверка стабильности
                delay(50);
                updateAllSensors();
                if (isRightLimitPressed()) {
                    right_found = true;
                    break;
                }
            }
            stepperStep(S_PROFILE_START_DELAY);
            steps_back++;
        }
        
        if (right_found) {
            Serial.print(" найден за ");
            Serial.print(steps_back);
            Serial.println(" шагов");
            
            // Если нашли за 0 шагов - это ошибка!
            if (steps_back == 0) {
                Serial.println("ОШИБКА: концевик найден сразу! Механическая проблема или ложное срабатывание.");
                return;
            }
        } else {
            Serial.println(" не найден");
            return;
        }
        
        delay(200);
        
        // 4. Измеряем расстояние до левого
        calibration_direction = true; // Справа налево
        measured_distance = measureDistanceWithSProfile(false);
        
    } else {
        // Каретка в середине - едем к левому концевику
        Serial.println("Начальная позиция: в середине");
        Serial.println("Сначала едем к левому концевику...");
        
        if (moveToSwitchForCalibration(false)) {
            delay(500);
            
            // Затем выполняем калибровку как для левого концевика
            // 1. Отъезжаем от левого концевика на 1000 шагов
            Serial.println("Отъезд от левого концевика на 1000 шагов...");
            stepperSetDirection(LOW); // Вправо
            
            for (unsigned long i = 0; i < 1000; i++) {
                stepperStep(S_PROFILE_START_DELAY);
            }
            
            Serial.println("  ✓ Отъехали на 1000 шагов");
            delay(100);
            
            // 2. Ждем отпускания левого концевика
            if (!waitForSwitchRelease(true, 5000)) {
                Serial.println("ОШИБКА: левый концевик не отпускается!");
                return;
            }
            
            // 3. Возвращаемся и ищем левый концевик
            Serial.print("  Поиск левого концевика...");
            stepperSetDirection(HIGH); // Влево
            bool left_found = false;
            unsigned long steps_back = 0;
            
            while (!left_found && steps_back < 2000) {
                updateAllSensors();
                if (isLeftLimitPressed()) {
                    // Двойная проверка стабильности
                    delay(50);
                    updateAllSensors();
                    if (isLeftLimitPressed()) {
                        left_found = true;
                        break;
                    }
                }
                stepperStep(S_PROFILE_START_DELAY);
                steps_back++;
            }
            
            if (left_found) {
                Serial.print(" найден за ");
                Serial.print(steps_back);
                Serial.println(" шагов");
                
                // Если нашли за 0 шагов - это ошибка!
                if (steps_back == 0) {
                    Serial.println("ОШИБКА: концевик найден сразу! Механическая проблема или ложное срабатывание.");
                    return;
                }
            } else {
                Serial.println(" не найден");
                return;
            }
            
            delay(200);
            
            // 4. Измеряем расстояние до правого
            calibration_direction = false; // Слева направо
            measured_distance = measureDistanceWithSProfile(true);
        } else {
            Serial.println("Не удалось найти левый концевик!");
            return;
        }
    }
    
    // Обработка результатов
    if (measured_distance > 0) {
        // ДОБАВЛЕНО: Отъезд на 40 шагов от найденного концевика
        Serial.println("\nОтъезд от концевика на 40 шагов...");
        
        if (calibration_direction) { // Если калибровались справа налево
            // Калибровка была от правого к левому - отъезжаем от левого влево на 40 шагов
            stepperSetDirection(HIGH); // Влево
            for (int i = 0; i < 40; i++) {
                stepperStep(S_PROFILE_START_DELAY);
            }
            Serial.println("Отъехали от левого концевика на 40 шагов влево");
        } else { // Если калибровались слева направо
            // Калибровка была от левого к правому - отъезжаем от правого вправо на 40 шагов
            stepperSetDirection(LOW); // Вправо
            for (int i = 0; i < 40; i++) {
                stepperStep(S_PROFILE_START_DELAY);
            }
            Serial.println("Отъехали от правого концевика на 40 шагов вправо");
        }
        
        delay(200);
        
        // ДОБАВЛЕНО: Увеличиваем расстояние на 80 шагов (по 40 с каждой стороны)
        travelDistance = measured_distance + 80;
        isCalibrated = true;
        
        Serial.println("\n=== КАЛИБРОВКА ЗАВЕРШЕНА ===");
        Serial.print("Исходное расстояние: ");
        Serial.print(measured_distance);
        Serial.println(" шагов");
        Serial.print("Скорректированное расстояние: ");
        Serial.print(travelDistance);
        Serial.println(" шагов (+80 шагов буфера)");
        
        // Если известно расстояние в мм, вычисляем коэффициент
        if (travelDistanceMM > 0) {
            stepsPerMM = (float)travelDistance / travelDistanceMM;
            Serial.print("Коэффициент: ");
            Serial.print(stepsPerMM, 3);
            Serial.println(" шагов/мм");
        }
        
        // Сохраняем калибровку в EEPROM (без лишнего вывода)
        saveSystemParameters();
        
    } else {
        Serial.println("\n!!! КАЛИБРОВКА НЕ УДАЛАСЬ !!!");
        Serial.println("Не удалось определить расстояние.");
    }
}

void setManualDistance(unsigned long steps, float mm) {
    travelDistance = steps;
    travelDistanceMM = mm;
    
    if (mm > 0) {
        stepsPerMM = (float)steps / mm;
    }
    
    isCalibrated = true;
    
    // Сохраняем в EEPROM (без лишнего вывода)
    saveSystemParameters();
    
    Serial.print("Ручная установка: ");
    Serial.print(steps);
    Serial.print(" шагов, ");
    Serial.print(mm);
    Serial.print(" мм, коэффициент ");
    Serial.print(stepsPerMM, 3);
    Serial.println(" шагов/мм");
}

unsigned long getCalibratedDistance() {
    return travelDistance;
}

float getStepsPerMM() {
    return stepsPerMM;
}

bool isReadyForCalibration() {
    updateAllSensors();
    
    // Нельзя калибровать если оба концевики нажаты
    if (isBothLimitsPressed()) {
        Serial.println("ОШИБКА: оба концевика нажаты!");
        return false;
    }
    
    // Проверяем аварию драйвера
    if (stepperIsAlarm()) {
        Serial.println("ОШИБКА: драйвер в аварийном состоянии!");
        return false;
    }
    
    return true;
}

void resetCalibration() {
    isCalibrated = false;
    travelDistance = DEFAULT_STEPS;
    travelDistanceMM = DEFAULT_MM;
    stepsPerMM = DEFAULT_STEPS_PER_MM;
    
    // Сохраняем в EEPROM (без лишнего вывода)
    saveSystemParameters();
    
    Serial.println("Калибровка сброшена до заводских значений");
}