#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "pneumatics.h"

// Вспомогательная функция ожидания с проверкой конфликтов
static bool pneumoWaitForSensorEnhanced(bool (*sensor_func)(), bool (*conflict_sensor_func)(), 
                                        unsigned long timeout_ms, const char* action_name) {
    unsigned long start_time = millis();
    bool success = false;
    unsigned long stable_start = 0;
    
    Serial.print("PNEUMO: ");
    Serial.print(action_name);
    Serial.println("...");
    
    while (!success) {
        updateAllSensors();
        
        // Проверяем конфликт датчиков
        if (conflict_sensor_func && conflict_sensor_func()) {
            Serial.print("PNEUMO: КОНФЛИКТ при ");
            Serial.println(action_name);
            return false;
        }
        
        if (sensor_func()) {
            if (stable_start == 0) {
                // Впервые достигли нужного состояния
                stable_start = millis();
            }
            
            // Проверяем стабильность в течение 100мс
            if (millis() - stable_start >= 100) {
                success = true;
                break;
            }
        } else {
            // Сбрасываем счетчик стабильности
            stable_start = 0;
        }
        
        if (millis() - start_time >= timeout_ms) {
            Serial.print("PNEUMO: Таймаут при ");
            Serial.println(action_name);
            return false;
        }
        
        delay(PNEUMO_POLL_INTERVAL_MS);
    }
    
    // Пауза после срабатывания датчика
    if (success && PNEUMO_POST_DELAY_MS > 0) {
        delay(PNEUMO_POST_DELAY_MS);
    }
    
    // Дополнительная проверка через 50мс
    delay(50);
    updateAllSensors();
    
    if (!sensor_func()) {
        Serial.print("PNEUMO: Нестабильное состояние после ");
        Serial.println(action_name);
        return false;
    }
    
    Serial.print("PNEUMO: ✓ ");
    Serial.print(action_name);
    Serial.println(" успешно");
    return true;
}

void pneumoInit() {
    pinMode(GRIP_SOL_PIN, OUTPUT);
    pinMode(Z_SOL_PIN, OUTPUT);
    
    // Устанавливаем безопасное состояние (закрыто и поднято)
    digitalWrite(GRIP_SOL_PIN, LOW);   // Закрыть захват
    digitalWrite(Z_SOL_PIN, LOW);      // Поднять вертикаль
    
    Serial.println("Пневматика инициализирована");
}

void pneumoZUp() {
    digitalWrite(Z_SOL_PIN, LOW);
    // УБРАНО: Serial.println("PNEUMO: команда 'вертикаль ВВЕРХ'");
}

void pneumoZDown() {
    digitalWrite(Z_SOL_PIN, HIGH);
    // УБРАНО: Serial.println("PNEUMO: команда 'вертикаль ВНИЗ'");
}

void pneumoGripOpen() {
    digitalWrite(GRIP_SOL_PIN, HIGH);
    // УБРАНО: Serial.println("PNEUMO: команда 'захват ОТКРЫТЬ'");
}

void pneumoGripClose() {
    digitalWrite(GRIP_SOL_PIN, LOW);
    // УБРАНО: Serial.println("PNEUMO: команда 'захват ЗАКРЫТЬ'");
}

bool pneumoZUpSafe(unsigned long timeout_ms) {
    pneumoZUp();
    return waitForZUp(timeout_ms);
}

bool pneumoZDownSafe(unsigned long timeout_ms) {
    pneumoZDown();
    return waitForZDown(timeout_ms);
}

bool pneumoGripOpenSafe(unsigned long timeout_ms) {
    pneumoGripOpen();
    return waitForGripOpen(timeout_ms);
}

bool pneumoGripCloseSafe(unsigned long timeout_ms) {
    pneumoGripClose();
    return waitForGripClosed(timeout_ms);
}

// Усиленные версии с проверкой конфликтов
bool pneumoZUpSafeEnhanced(unsigned long timeout_ms) {
    pneumoZUp();
    return pneumoWaitForSensorEnhanced(isZUp, isZConflict, timeout_ms, "подъем вертикали");
}

bool pneumoZDownSafeEnhanced(unsigned long timeout_ms) {
    pneumoZDown();
    return pneumoWaitForSensorEnhanced(isZDown, isZConflict, timeout_ms, "опускание вертикали");
}

bool pneumoGripOpenSafeEnhanced(unsigned long timeout_ms) {
    pneumoGripOpen();
    return pneumoWaitForSensorEnhanced(isGripOpen, isGripConflict, timeout_ms, "открытие захвата");
}

bool pneumoGripCloseSafeEnhanced(unsigned long timeout_ms) {
    pneumoGripClose();
    return pneumoWaitForSensorEnhanced(isGripClosed, isGripConflict, timeout_ms, "закрытие захвата");
}

bool pneumoHandleCommand(char cmd) {
    // Приводим команду к верхнему регистру
    cmd = toupper(cmd);
    
    switch (cmd) {
        case 'W': // Вверх (W/w)
            if (pneumoZUpSafeEnhanced()) {
                Serial.println("PNEUMO: вертикаль ВВЕРХ - OK");
            } else {
                Serial.println("PNEUMO: вертикаль ВВЕРХ - ТАЙМАУТ");
            }
            return true;
            
        case 'S': // Вниз (S/s)
            if (pneumoZDownSafeEnhanced()) {
                Serial.println("PNEUMO: вертикаль ВНИЗ - OK");
            } else {
                Serial.println("PNEUMO: вертикаль ВНИЗ - ТАЙМАУТ");
            }
            return true;
            
        case 'Q': // Открыть (Q/q)
            if (pneumoGripOpenSafeEnhanced()) {
                Serial.println("PNEUMO: захват ОТКРЫТ - OK");
            } else {
                Serial.println("PNEUMO: захват ОТКРЫТ - ТАЙМАУТ");
            }
            return true;
            
        case 'E': // Закрыть (E/e)
            if (pneumoGripCloseSafeEnhanced()) {
                Serial.println("PNEUMO: захват ЗАКРЫТ - OK");
            } else {
                Serial.println("PNEUMO: захват ЗАКРЫТ - ТАЙМАУТ");
            }
            return true;
            
        default:
            return false; // Не команда пневматики
    }
}

void pneumoPrintStatus() {
    updateAllSensors();
    
    Serial.println("\n=== СОСТОЯНИЕ ПНЕВМАТИКИ ===");
    
    Serial.print("Захват: ");
    if (isGripOpen()) {
        Serial.print("ОТКРЫТ(f)");
    } else if (isGripClosed()) {
        Serial.print("ЗАКРЫТ(f)");
    } else {
        Serial.print("ПРОМЕЖУТОЧНО(f)");
    }
    Serial.print(" [сырое: OPEN=");
    Serial.print(isGripOpenRaw() ? "ДА" : "НЕТ");
    Serial.print(", CLOSED=");
    Serial.print(isGripClosedRaw() ? "ДА" : "НЕТ");
    Serial.print("]");
    
    Serial.print(" | Вертикаль: ");
    if (isZUp()) {
        Serial.print("ВВЕРХУ(f)");
    } else if (isZDown()) {
        Serial.print("ВНИЗУ(f)");
    } else {
        Serial.print("ПРОМЕЖУТОЧНО(f)");
    }
    Serial.print(" [сырое: UP=");
    Serial.print(isZUpRaw() ? "ДА" : "НЕТ");
    Serial.print(", DOWN=");
    Serial.print(isZDownRaw() ? "ДА" : "НЕТ");
    Serial.println("]");
    
    // Проверка конфликтов
    if (isGripConflict()) {
        Serial.println("!!! ВНИМАНИЕ: Оба датчика захвата активны!");
    }
    if (isZConflict()) {
        Serial.println("!!! ВНИМАНИЕ: Оба датчика вертикали активны!");
    }
    
    Serial.println("==============================");
}

void pneumoSafePosition() {
    Serial.println("Установка пневматики в безопасное положение...");
    pneumoGripCloseSafeEnhanced();
    pneumoZUpSafeEnhanced();
    Serial.println("Пневматика в безопасном положении");
}