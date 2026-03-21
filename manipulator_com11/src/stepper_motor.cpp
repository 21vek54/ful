#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"

// Глобальные переменные для управления движением
static bool stepperEnabled = false;

// Инициализация реле для сброса драйвера
static void initRelay() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Реле выключено (питание подано на драйвер)
    Serial.println("Реле для сброса драйвера инициализировано");
}

// Сброс драйвера через реле (аварийный перезапуск)
void stepperHardReset() {
    Serial.println("=== АВАРИЙНЫЙ СБРОС ДРАЙВЕРА ЧЕРЕЗ РЕЛЕ ===");
    
    // 1. Выключаем драйвер через ENABLE (для безопасности)
    stepperEnable(false);
    delay(100);
    
    // 2. Отключаем питание драйвера через реле
    Serial.print("Отключение питания драйвера на ");
    Serial.print(RELAY_RESET_TIME);
    Serial.println(" мс...");
    digitalWrite(RELAY_PIN, HIGH); // Включаем реле (отключаем питание)
    delay(RELAY_RESET_TIME);
    
    // 3. Включаем питание обратно
    Serial.println("Включение питания драйвера...");
    digitalWrite(RELAY_PIN, LOW); // Выключаем реле (включаем питание)
    delay(500); // Даем время на инициализацию драйвера
    
    // 4. Включаем драйвер через ENABLE
    stepperEnable(true);
    delay(100);
    
    // 5. Проверяем состояние
    updateAllSensors();
    if (stepperIsAlarm()) {
        Serial.println("⚠️  Предупреждение: драйвер все еще в аварийном состоянии после сброса!");
    } else {
        Serial.println("✓ Драйвер успешно сброшен");
    }
    
    Serial.println("==========================================");
}

// Попытка восстановления после аварии драйвера
bool stepperRecoverFromAlarm() {
    if (!stepperIsAlarm()) {
        Serial.println("Восстановление не требуется: драйвер не в аварии");
        return true;
    }
    
    Serial.println("=== ПОПЫТКА ВОССТАНОВЛЕНИЯ ДРАЙВЕРА ===");
    
    // Пробуем программный сброс через ENABLE
    Serial.println("Попытка программного сброса...");
    stepperEnable(false);
    delay(200);
    stepperEnable(true);
    delay(200);
    
    updateAllSensors();
    if (!stepperIsAlarm()) {
        Serial.println("✓ Драйвер восстановлен программным сбросом");
        return true;
    }
    
    Serial.println("Программный сброс не помог, пробуем аппаратный сброс...");
    
    // Пробуем аппаратный сброс через реле
    stepperHardReset();
    
    updateAllSensors();
    if (!stepperIsAlarm()) {
        Serial.println("✓ Драйвер восстановлен аппаратным сбросом");
        return true;
    }
    
    Serial.println("✗ Не удалось восстановить драйвер");
    return false;
}

void stepperInit() {
    // Инициализируем реле для сброса драйвера
    initRelay();
    
    // Настраиваем пины шагового двигателя
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(ALM_PIN, INPUT_PULLUP);
    
    // Включаем драйвер (активный уровень LOW)
    digitalWrite(ENABLE_PIN, LOW);
    stepperEnabled = true;
    
    Serial.println("Шаговый двигатель инициализирован");
}

void stepperEnable(bool enable) {
    digitalWrite(ENABLE_PIN, enable ? LOW : HIGH);
    stepperEnabled = enable;
    
    if (enable) {
        delay(100); // Задержка после включения драйвера
        updateAllSensors();
        
        if (stepperIsAlarm()) {
            Serial.println("⚠️  ВНИМАНИЕ: Драйвер в аварийном состоянии после включения!");
        }
    }
    
    Serial.print("Драйвер двигателя ");
    Serial.println(enable ? "включен" : "выключен");
}

void stepperSetDirection(bool direction) {
    digitalWrite(DIR_PIN, direction);
    delayMicroseconds(20); // Задержка после смены направления
}

void stepperStep(unsigned long delay_us) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delay_us / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delay_us / 2);
}

void stepperMoveSteps(unsigned long steps, unsigned long delay_us) {
    for (unsigned long i = 0; i < steps; i++) {
        stepperStep(delay_us);
    }
}

bool stepperIsAlarm() {
    return digitalRead(ALM_PIN) == LOW;
}

unsigned long stepperMoveToLimit(bool direction, unsigned long max_steps, unsigned long delay_us) {
    stepperSetDirection(direction);
    
    unsigned long steps = 0;
    bool limit_reached = false;
    
    while (!limit_reached && steps < max_steps) {
        // Проверяем концевик каждые 10 шагов
        if (steps > 0 && steps % STEP_CHECK_INTERVAL == 0) {
            updateAllSensors();
            
            if (direction == HIGH) { // Движение влево
                limit_reached = isLeftLimitPressed();
            } else { // Движение вправо
                limit_reached = isRightLimitPressed();
            }
            
            if (limit_reached) {
                break;
            }
        }
        
        stepperStep(delay_us);
        steps++;
        
        // Выводим прогресс каждые 1000 шагов
        if (steps % 1000 == 0) {
            Serial.print("  Пройдено: ");
            Serial.print(steps);
            Serial.println(" шагов");
        }
    }
    
    return steps;
}

unsigned long stepperMoveWithSProfile(bool direction, unsigned long plateau_steps) {
    // Эта функция реализована в calibration.cpp
    // Здесь возвращаем 0, так как основная реализация в другом месте
    return 0;
}