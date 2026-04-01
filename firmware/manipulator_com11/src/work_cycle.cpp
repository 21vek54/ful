#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "pneumatics.h"
#include "motion_profiles.h"
#include "work_cycle.h"
#include "system_state.h"
#include "recovery.h"

// Глобальные переменные (из system_state.cpp)
extern bool isCalibrated;
extern unsigned long travelDistance;
extern bool isMoving;
extern bool workCycleInProgress;
extern bool greaseRemovalInProgress;

// Локальные переменные
static uint8_t currentStep = 0;
static bool step3ReachedLatch = false;
static bool step7ReachedLatch = false;
static bool recoveryMode = false;

void resetWorkCycleReadyLatches() {
    step3ReachedLatch = false;
    step7ReachedLatch = false;
}

// =============================================
// НОВЫЕ ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ БЕЗОПАСНОГО ВОССТАНОВЛЕНИЯ
// =============================================

// Функция медленного движения к правому концевику
bool slowMoveToRightEnd() {
    Serial.println("Медленное движение к правому концевику...");
    stepperSetDirection(LOW); // Вправо
    unsigned long steps = 0;
    bool switchFound = false;
    unsigned long startTime = millis();
    
    while (!switchFound && steps < 50000) {
        updateAllSensors();
        
        if (isRightLimitPressed()) {
            delay(50);
            updateAllSensors();
            if (isRightLimitPressed()) {
                switchFound = true;
                break;
            }
        }
        
        stepperStep(1000); // Медленная скорость (1000 мкс)
        steps++;
        
        // Выводим прогресс каждые 1000 шагов
        if (steps % 1000 == 0) {
            Serial.print("  Пройдено: ");
            Serial.print(steps);
            Serial.println(" шагов");
        }
        
        // Таймаут 30 секунд
        if (millis() - startTime > 30000) {
            Serial.println("Таймаут движения к правому концевику!");
            break;
        }
        
        // Проверка аварии драйвера
        if (stepperIsAlarm()) {
            Serial.println("Авария драйвера при медленном движении!");
            break;
        }
        
        // Проверка конфликта концевиков
        if (isBothLimitsPressed()) {
            Serial.println("Конфликт: оба концевика нажаты!");
            break;
        }
    }
    
    if (switchFound) {
        Serial.print("✓ Правый концевик достигнут за ");
        Serial.print(steps);
        Serial.println(" шагов");
        
        // Заезжаем на 40 шагов вглубь концевика
        Serial.println("Заезжаем на 40 шагов вглубь концевика...");
        stepperSetDirection(HIGH); // Влево
        for (int i = 0; i < 40; i++) {
            stepperStep(1000);
        }
        
        return true;
    }
    
    return false;
}

// Функция медленного движения к левому концевику
bool slowMoveToLeftEnd() {
    Serial.println("Медленное движение к левому концевику...");
    stepperSetDirection(HIGH); // Влево
    unsigned long steps = 0;
    bool switchFound = false;
    unsigned long startTime = millis();
    
    while (!switchFound && steps < 50000) {
        updateAllSensors();
        
        if (isLeftLimitPressed()) {
            delay(50);
            updateAllSensors();
            if (isLeftLimitPressed()) {
                switchFound = true;
                break;
            }
        }
        
        stepperStep(1000); // Медленная скорость (1000 мкс)
        steps++;
        
        // Выводим прогресс каждые 1000 шагов
        if (steps % 1000 == 0) {
            Serial.print("  Пройдено: ");
            Serial.print(steps);
            Serial.println(" шагов");
        }
        
        // Таймаут 30 секунд
        if (millis() - startTime > 30000) {
            Serial.println("Таймаут движения к левому концевику!");
            break;
        }
        
        // Проверка аварии драйвера
        if (stepperIsAlarm()) {
            Serial.println("Авария драйвера при медленном движении!");
            break;
        }
        
        // Проверка конфликта концевиков
        if (isBothLimitsPressed()) {
            Serial.println("Конфликт: оба концевика нажаты!");
            break;
        }
    }
    
    if (switchFound) {
        Serial.print("✓ Левый концевик достигнут за ");
        Serial.print(steps);
        Serial.println(" шагов");
        
        // Заезжаем на 40 шагов вглубь концевика
        Serial.println("Заезжаем на 40 шагов вглубь концевика...");
        stepperSetDirection(LOW); // Вправо
        for (int i = 0; i < 40; i++) {
            stepperStep(1000);
        }
        
        return true;
    }
    
    return false;
}

// Функция безопасного восстановления для каждого шага EEPROM
bool safeRecoveryForStep(uint8_t savedStep) {
    Serial.println("\n=== БЕЗОПАСНОЕ ВОССТАНОВЛЕНИЕ ===");
    Serial.print("Восстановление для шага ");
    Serial.println(savedStep);
    
    // Определяем текущее состояние
    updateAllSensors();
    
    bool gripClamped = isGripClosed();
    bool gripOpen = isGripOpen();
    bool zUp = isZUp();
    bool zDown = isZDown();
    bool leftPressed = isLeftLimitPressed();
    bool rightPressed = isRightLimitPressed();
    bool inMiddle = !leftPressed && !rightPressed;
    
    Serial.print("Текущее состояние: ");
    Serial.print("Захват=");
    Serial.print(gripClamped ? "ЗАЖАТ" : "РАЗЖАТ");
    Serial.print(", Вертикаль=");
    Serial.print(zUp ? "ВВЕРХУ" : (zDown ? "ВНИЗУ" : "ПРОМЕЖУТОЧНО"));
    Serial.print(", Позиция=");
    if (leftPressed) Serial.println("ЛЕВЫЙ");
    else if (rightPressed) Serial.println("ПРАВЫЙ");
    else Serial.println("СЕРЕДИНА");
    
    // Основные правила безопасности
    if (gripClamped) {
        Serial.println("⚠  Захват зажат - возможна тарелка внутри!");
        
        // Если тарелка в захвате, ПОДНИМАЕМ вертикаль перед любыми движениями
        if (!zUp) {
            Serial.println("Поднимаем вертикаль для безопасности тарелки...");
            if (!pneumoZUpSafeEnhanced()) {
                Serial.println("Ошибка: не удалось поднять вертикаль!");
                return false;
            }
            zUp = true;
            zDown = false;
        }
    } else {
        // Если тарелки нет, тоже поднимаем вертикаль для безопасности
        if (!zUp && !zDown) {
            Serial.println("Поднимаем вертикаль для безопасности конвейера...");
            if (!pneumoZUpSafeEnhanced()) {
                Serial.println("Ошибка: не удалось поднять вертикаль!");
                return false;
            }
            zUp = true;
            zDown = false;
        }
    }
    
    // Восстановление в зависимости от сохраненного шага
    switch(savedStep) {
        case 1: // Опускание (должны быть: право/низ/разжат)
            Serial.println("Требуется: право/низ/разжат");
            
            // 1. Корректируем горизонтальное положение
            if (!rightPressed) {
                Serial.println("Двигаемся к правому концевику...");
                if (!slowMoveToRightEnd()) {
                    Serial.println("Ошибка движения к правому концевику!");
                    return false;
                }
                rightPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zDown) {
                Serial.println("Опускаем вертикаль...");
                if (!pneumoZDownSafeEnhanced()) {
                    Serial.println("Ошибка опускания вертикали!");
                    return false;
                }
                zDown = true;
                zUp = false;
            }
            
            // 3. Корректируем захват
            if (!gripOpen) {
                Serial.println("Разжимаем захват...");
                if (!pneumoGripOpenSafeEnhanced()) {
                    Serial.println("Ошибка разжатия захвата!");
                    return false;
                }
                gripOpen = true;
                gripClamped = false;
            }
            
            Serial.println("✓ Восстановлено: право/низ/разжат");
            break;
            
        case 2: // Зажатие (должны быть: право/низ/зажат)
            Serial.println("Требуется: право/низ/зажат");
            
            // 1. Корректируем горизонтальное положение
            if (!rightPressed) {
                Serial.println("Двигаемся к правому концевику...");
                if (!slowMoveToRightEnd()) {
                    Serial.println("Ошибка движения к правому концевику!");
                    return false;
                }
                rightPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zDown) {
                Serial.println("Опускаем вертикаль...");
                if (!pneumoZDownSafeEnhanced()) {
                    Serial.println("Ошибка опускания вертикали!");
                    return false;
                }
                zDown = true;
                zUp = false;
            }
            
            // 3. Корректируем захват
            if (!gripClamped) {
                Serial.println("Зажимаем захват...");
                if (!pneumoGripCloseSafeEnhanced()) {
                    Serial.println("Ошибка зажатия захвата!");
                    return false;
                }
                gripClamped = true;
                gripOpen = false;
            }
            
            Serial.println("✓ Восстановлено: право/низ/зажат");
            break;
            
        case 3: // Подъем (должны быть: право/верх/зажат)
            Serial.println("Требуется: право/верх/зажат");
            
            // 1. Корректируем горизонтальное положение
            if (!rightPressed) {
                Serial.println("Двигаемся к правому концевику...");
                if (!slowMoveToRightEnd()) {
                    Serial.println("Ошибка движения к правому концевику!");
                    return false;
                }
                rightPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zUp) {
                Serial.println("Поднимаем вертикаль...");
                if (!pneumoZUpSafeEnhanced()) {
                    Serial.println("Ошибка подъема вертикали!");
                    return false;
                }
                zUp = true;
                zDown = false;
            }
            
            // 3. Проверяем захват (должен быть зажат)
            if (!gripClamped) {
                Serial.println("⚠  Захват разжат, но должен быть зажат!");
                Serial.println("Возможно, тарелка выпала. Будем считать, что ее нет.");
                // Не зажимаем, чтобы не повредить механизм
            }
            
            Serial.println("✓ Восстановлено: право/верх/зажат");
            break;
            
        case 4: // Движение влево (должны быть: лево/верх/зажат)
            Serial.println("Требуется: лево/верх/зажат");
            
            // 1. Корректируем горизонтальное положение
            if (!leftPressed) {
                Serial.println("Двигаемся к левому концевику...");
                if (!slowMoveToLeftEnd()) {
                    Serial.println("Ошибка движения к левому концевику!");
                    return false;
                }
                leftPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zUp) {
                Serial.println("Поднимаем вертикаль...");
                if (!pneumoZUpSafeEnhanced()) {
                    Serial.println("Ошибка подъема вертикали!");
                    return false;
                }
                zUp = true;
                zDown = false;
            }
            
            // 3. Проверяем захват (должен быть зажат)
            if (!gripClamped) {
                Serial.println("⚠  Захват разжат, но должен быть зажат!");
                Serial.println("Возможно, тарелка выпала. Будем считать, что ее нет.");
            }
            
            Serial.println("✓ Восстановлено: лево/верх/зажат");
            break;
            
        case 5: // Опускание (должны быть: лево/низ/зажат)
            Serial.println("Требуется: лево/низ/зажат");
            
            // 1. Корректируем горизонтальное положение
            if (!leftPressed) {
                Serial.println("Двигаемся к левому концевику...");
                if (!slowMoveToLeftEnd()) {
                    Serial.println("Ошибка движения к левому концевику!");
                    return false;
                }
                leftPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zDown) {
                Serial.println("Опускаем вертикаль...");
                if (!pneumoZDownSafeEnhanced()) {
                    Serial.println("Ошибка опускания вертикали!");
                    return false;
                }
                zDown = true;
                zUp = false;
            }
            
            // 3. Проверяем захват (должен быть зажат)
            if (!gripClamped) {
                Serial.println("⚠  Захват разжат, но должен быть зажат!");
                Serial.println("Возможно, тарелка выпала. Будем считать, что ее нет.");
            }
            
            Serial.println("✓ Восстановлено: лево/низ/зажат");
            break;
            
        case 6: // Разжатие (должны быть: лево/низ/разжат)
            Serial.println("Требуется: лево/низ/разжат");
            
            // 1. Корректируем горизонтальное положение
            if (!leftPressed) {
                Serial.println("Двигаемся к левому концевику...");
                if (!slowMoveToLeftEnd()) {
                    Serial.println("Ошибка движения к левому концевику!");
                    return false;
                }
                leftPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zDown) {
                Serial.println("Опускаем вертикаль...");
                if (!pneumoZDownSafeEnhanced()) {
                    Serial.println("Ошибка опускания вертикали!");
                    return false;
                }
                zDown = true;
                zUp = false;
            }
            
            // 3. Корректируем захват
            if (!gripOpen) {
                Serial.println("Разжимаем захват...");
                if (!pneumoGripOpenSafeEnhanced()) {
                    Serial.println("Ошибка разжатия захвата!");
                    return false;
                }
                gripOpen = true;
                gripClamped = false;
            }
            
            Serial.println("✓ Восстановлено: лево/низ/разжат");
            break;
            
        case 7: // Подъем (должны быть: лево/верх/разжат)
            Serial.println("Требуется: лево/верх/разжат");
            
            // 1. Корректируем горизонтальное положение
            if (!leftPressed) {
                Serial.println("Двигаемся к левому концевику...");
                if (!slowMoveToLeftEnd()) {
                    Serial.println("Ошибка движения к левому концевику!");
                    return false;
                }
                leftPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zUp) {
                Serial.println("Поднимаем вертикаль...");
                if (!pneumoZUpSafeEnhanced()) {
                    Serial.println("Ошибка подъема вертикали!");
                    return false;
                }
                zUp = true;
                zDown = false;
            }
            
            // 3. Корректируем захват
            if (!gripOpen) {
                Serial.println("Разжимаем захват...");
                if (!pneumoGripOpenSafeEnhanced()) {
                    Serial.println("Ошибка разжатия захвата!");
                    return false;
                }
                gripOpen = true;
                gripClamped = false;
            }
            
            Serial.println("✓ Восстановлено: лево/верх/разжат");
            break;
            
        case 8: // Движение вправо (должны быть: право/верх/разжат)
            Serial.println("Требуется: право/верх/разжат");
            
            // 1. Корректируем горизонтальное положение
            if (!rightPressed) {
                Serial.println("Двигаемся к правому концевику...");
                if (!slowMoveToRightEnd()) {
                    Serial.println("Ошибка движения к правому концевику!");
                    return false;
                }
                rightPressed = true;
            }
            
            // 2. Корректируем вертикальное положение
            if (!zUp) {
                Serial.println("Поднимаем вертикаль...");
                if (!pneumoZUpSafeEnhanced()) {
                    Serial.println("Ошибка подъема вертикали!");
                    return false;
                }
                zUp = true;
                zDown = false;
            }
            
            // 3. Корректируем захват
            if (!gripOpen) {
                Serial.println("Разжимаем захват...");
                if (!pneumoGripOpenSafeEnhanced()) {
                    Serial.println("Ошибка разжатия захвата!");
                    return false;
                }
                gripOpen = true;
                gripClamped = false;
            }
            
            Serial.println("✓ Восстановлено: право/верх/разжат");
            break;
            
        default:
            Serial.print("Неизвестный шаг для восстановления: ");
            Serial.println(savedStep);
            return false;
    }
    
    return true;
}

// =============================================
// СУЩЕСТВУЮЩИЕ ФУНКЦИИ (оставляем без изменений, кроме recoverWorkCycle)
// =============================================

bool verifyStepState(uint8_t step) {
    updateAllSensors();
    
    switch (step) {
        case 1: // право/низ/разжат
            return isRightLimitPressed() && isZDown() && isGripOpen();
        case 2: // право/низ/сжат
            return isRightLimitPressed() && isZDown() && isGripClosed();
        case 3: // право/верх/сжат
            return isRightLimitPressed() && isZUp() && isGripClosed();
        case 4: // лево/верх/сжат
            return isLeftLimitPressed() && isZUp() && isGripClosed();
        case 5: // лево/низ/сжат
            return isLeftLimitPressed() && isZDown() && isGripClosed();
        case 6: // лево/низ/разжат
            return isLeftLimitPressed() && isZDown() && isGripOpen();
        case 7: // лево/верх/разжат
            return isLeftLimitPressed() && isZUp() && isGripOpen();
        case 8: // право/верх/разжат
            return isRightLimitPressed() && isZUp() && isGripOpen();
        default:
            return false;
    }
}

bool checkStepCompleted(uint8_t step) {
    // Для каждого шага проверяем предыдущие состояния
    switch (step) {
        case 1: // Шаг 1 выполнен если мы в состоянии 1, 2, 3, 4, 5, 6, 7 или 8
            return verifyStepState(1) || verifyStepState(2) || verifyStepState(3) || 
                   verifyStepState(4) || verifyStepState(5) || verifyStepState(6) || 
                   verifyStepState(7) || verifyStepState(8);
        case 2: // Шаг 2 выполнен если мы в состоянии 2, 3, 4, 5, 6, 7 или 8
            return verifyStepState(2) || verifyStepState(3) || verifyStepState(4) || 
                   verifyStepState(5) || verifyStepState(6) || verifyStepState(7) || 
                   verifyStepState(8);
        case 3: // Шаг 3 выполнен если мы в состоянии 3, 4, 5, 6, 7 или 8
            return verifyStepState(3) || verifyStepState(4) || verifyStepState(5) || 
                   verifyStepState(6) || verifyStepState(7) || verifyStepState(8);
        case 4: // Шаг 4 выполнен если мы в состоянии 4, 5, 6, 7 или 8
            return verifyStepState(4) || verifyStepState(5) || verifyStepState(6) || 
                   verifyStepState(7) || verifyStepState(8);
        case 5: // Шаг 5 выполнен если мы в состоянии 5, 6, 7 или 8
            return verifyStepState(5) || verifyStepState(6) || verifyStepState(7) || 
                   verifyStepState(8);
        case 6: // Шаг 6 выполнен если мы в состоянии 6, 7 или 8
            return verifyStepState(6) || verifyStepState(7) || verifyStepState(8);
        case 7: // Шаг 7 выполнен если мы в состоянии 7 или 8
            return verifyStepState(7) || verifyStepState(8);
        case 8: // Шаг 8 выполнен если мы в состоянии 8
            return verifyStepState(8);
        default:
            return false;
    }
}

int getCurrentPosition() {
    updateAllSensors();
    
    if (isLeftLimitPressed() && !isRightLimitPressed()) {
        return 1; // Левый концевик
    } else if (isRightLimitPressed() && !isLeftLimitPressed()) {
        return 2; // Правый концевик
    } else if (!isLeftLimitPressed() && !isRightLimitPressed()) {
        return 0; // В середине
    } else {
        return -1; // Конфликт (оба нажаты)
    }
}

int getRequiredPositionForStep(uint8_t step) {
    switch (step) {
        case 1: // опускание (требуется право)
        case 2: // закрытие (требуется право)
        case 3: // подъем (требуется право)
            return 2; // Правый концевик
            
        case 4: // движение влево (требуется право)
            return 2; // Правый концевик
            
        case 5: // опускание (требуется лево)
        case 6: // открытие (требуется лево)
        case 7: // подъем (требуется лево)
            return 1; // Левый концевик
            
        case 8: // движение вправо (требуется лево)
            return 1; // Левый концевик
            
        default:
            return -1; // Неизвестный шаг
    }
}

// Функция для медленного движения к концевику (уже есть, оставляем)
bool slowMoveToSwitch(bool toLeft, unsigned long maxSteps) {
    Serial.print("Медленное движение к ");
    Serial.print(toLeft ? "левому" : "правому");
    Serial.println(" концевику...");
    
    stepperSetDirection(toLeft ? HIGH : LOW);
    unsigned long steps = 0;
    bool switchFound = false;
    unsigned long startTime = millis();
    
    while (!switchFound && steps < maxSteps) {
        updateAllSensors();
        
        // Проверяем концевик
        if (toLeft) {
            if (isLeftLimitPressed()) {
                delay(50);
                updateAllSensors();
                if (isLeftLimitPressed()) {
                    switchFound = true;
                    break;
                }
            }
        } else {
            if (isRightLimitPressed()) {
                delay(50);
                updateAllSensors();
                if (isRightLimitPressed()) {
                    switchFound = true;
                    break;
                }
            }
        }
        
        stepperStep(1000); // Медленная скорость (1000 мкс)
        steps++;
        
        // Проверка таймаута (30 секунд)
        if (millis() - startTime > 30000) {
            Serial.println("Таймаут медленного движения");
            break;
        }
        
        // Проверка аварии драйвера
        if (stepperIsAlarm()) {
            Serial.println("Авария драйвера при медленном движении!");
            break;
        }
    }
    
    if (switchFound) {
        Serial.print("✓ Концевик достигнут за ");
        Serial.print(steps);
        Serial.println(" шагов");
        return true;
    } else {
        Serial.println("✗ Не удалось достичь концевика");
        return false;
    }
}

// Функция корректировки положения для восстановления (обновленная)
bool adjustPositionForRecovery(uint8_t targetStep) {
    Serial.println("\n=== КОРРЕКТИРОВКА ПОЛОЖЕНИЯ ДЛЯ ВОССТАНОВЛЕНИЯ ===");
    
    // Используем новую функцию безопасного восстановления
    return safeRecoveryForStep(targetStep);
}

// Функция анализа текущего состояния и определения с какого шага продолжать
uint8_t analyzeCurrentStateAndGetStartStep(uint8_t savedStep) {
    Serial.println("\n=== АНАЛИЗ ТЕКУЩЕГО СОСТОЯНИЯ ===");
    
    // Сначала проверяем, в каком состоянии мы находимся
    for (uint8_t step = 1; step <= 8; step++) {
        if (verifyStepState(step)) {
            Serial.print("Текущее состояние соответствует шагу ");
            Serial.print(step);
            Serial.print(" (");
            switch (step) {
                case 1: Serial.print("право/низ/разжат"); break;
                case 2: Serial.print("право/низ/сжат"); break;
                case 3: Serial.print("право/верх/сжат"); break;
                case 4: Serial.print("лево/верх/сжат"); break;
                case 5: Serial.print("лево/низ/сжат"); break;
                case 6: Serial.print("лево/низ/разжат"); break;
                case 7: Serial.print("лево/верх/разжат"); break;
                case 8: Serial.print("право/верх/разжат"); break;
            }
            Serial.println(")");
            
            // Определяем, какие шаги уже выполнены
            Serial.println("Анализ выполненных шагов...");
            
            // Если мы находимся на шаге позже сохраненного
            if (step > savedStep) {
                // Проверяем, все ли промежуточные шаги выполнены
                bool allPreviousDone = true;
                for (uint8_t i = savedStep; i < step; i++) {
                    if (!checkStepCompleted(i)) {
                        allPreviousDone = false;
                        break;
                    }
                }
                
                if (allPreviousDone) {
                    // Все предыдущие шаги выполнены, продолжаем с текущего
                    Serial.print("Все предыдущие шаги выполнены, продолжаем с шага ");
                    Serial.println(step);
                    return step;
                } else {
                    // Не все шаги выполнены, начинаем с сохраненного
                    Serial.print("Не все предыдущие шаги выполнены, начинаем с сохраненного шага ");
                    Serial.println(savedStep);
                    return savedStep;
                }
            } else if (step == savedStep) {
                // На том же шаге - продолжаем со следующего
                uint8_t nextStep = (step == 8) ? 1 : step + 1;
                Serial.print("На том же шаге, продолжаем со следующего: ");
                Serial.println(nextStep);
                return nextStep;
            } else if (step < savedStep) {
                // На более раннем шаге - начинаем со следующего
                uint8_t nextStep = step + 1;
                Serial.print("На более раннем шаге, начинаем с: ");
                Serial.println(nextStep);
                return nextStep;
            }
        }
    }
    
    // Если не соответствуем ни одному состоянию
    Serial.println("Состояние не соответствует ни одному шагу");
    Serial.print("Начинаем с сохраненного шага: ");
    Serial.println(savedStep);
    return savedStep;
}

bool moveToStep(uint8_t targetStep) {
    // Эта функция перемещает манипулятор в состояние целевого шага
    // Упрощенная версия: всегда возвращаемся в начальное состояние и начинаем сначала
    Serial.println("Возврат в начальное состояние...");
    
    // Поднимаем вертикаль
    if (!isZUp()) {
        Serial.println("  Поднимаем вертикаль...");
        if (!pneumoZUpSafeEnhanced()) {
            Serial.println("  Ошибка: не удалось поднять вертикаль");
            return false;
        }
    }
    
    // Открываем захват
    if (!isGripOpen()) {
        Serial.println("  Открываем захват...");
        if (!pneumoGripOpenSafeEnhanced()) {
            Serial.println("  Ошибка: не удалось открыть захват");
            return false;
        }
    }
    
    // Двигаемся к правому концевику (если не на нем)
    if (!isRightLimitPressed()) {
        Serial.println("  Двигаемся к правому концевику...");
        if (!moveRightSafe()) {
            Serial.println("  Ошибка: не удалось добраться до правого концевика");
            return false;
        }
    }
    
    // Проверяем, что в начальном состоянии
    bool success = verifyStepState(8);
    if (success) {
        Serial.println("✓ Система в начальном состоянии");
    } else {
        Serial.println("✗ Не удалось привести систему в начальное состояние");
    }
    
    return success;
}

bool executeStep(uint8_t step) {
    switch (step) {
        case 1: // ОПУСТИТЬСЯ
            Serial.println("\n1. ОПУСКАНИЕ ВЕРТИКАЛИ...");
            if (!pneumoZDownSafeEnhanced()) {
                Serial.println("Ошибка: таймаут опускания");
                return false;
            }
            Serial.println("Состояние: право/низ/разжат");
            break;
            
        case 2: // СЖАТЬ
            Serial.println("\n2. ЗАКРЫТИЕ ЗАХВАТА...");
            if (!pneumoGripCloseSafeEnhanced()) {
                Serial.println("Ошибка: таймаут закрытия");
                pneumoZUpSafeEnhanced();
                return false;
            }
            Serial.println("Состояние: право/низ/сжат");
            break;
            
        case 3: // ПОДНЯТЬ
            Serial.println("\n3. ПОДЪЕМ ВЕРТИКАЛИ...");
            if (!pneumoZUpSafeEnhanced()) {
                Serial.println("Ошибка: таймаут подъема");
                return false;
            }
            Serial.println("Состояние: право/верх/сжат");
            break;
            
        case 4: // В ЛЕВО
            Serial.println("\n4. ДВИЖЕНИЕ ВЛЕВО...");
            if (!moveLeftSafe()) {
                Serial.println("Ошибка: движение влево не завершено");
                return false;
            }
            Serial.println("Состояние: лево/верх/сжат");
            break;
            
        case 5: // ОПУСТИТЬ
            Serial.println("\n5. ОПУСКАНИЕ ВЕРТИКАЛИ...");
            if (!pneumoZDownSafeEnhanced()) {
                Serial.println("Ошибка: таймаут опускания");
                pneumoZUpSafeEnhanced();
                return false;
            }
            Serial.println("Состояние: лево/низ/сжат");
            break;
            
        case 6: // РАЗЖАТЬ
            Serial.println("\n6. ОТКРЫТИЕ ЗАХВАТА...");
            if (!pneumoGripOpenSafeEnhanced()) {
                Serial.println("Ошибка: таймаут открытия");
                pneumoZUpSafeEnhanced();
                return false;
            }
            Serial.println("Состояние: лево/низ/разжат");
            break;
            
        case 7: // ПОДНЯТЬ
            Serial.println("\n7. ПОДЪЕМ ВЕРТИКАЛИ...");
            if (!pneumoZUpSafeEnhanced()) {
                Serial.println("Ошибка: таймаут подъема");
                return false;
            }
            Serial.println("Состояние: лево/верх/разжат");
            break;
            
        case 8: // ВПРАВО
            Serial.println("\n8. ДВИЖЕНИЕ ВПРАВО...");
            if (!moveRightSafe()) {
                Serial.println("Ошибка: движение вправо не завершено");
                return false;
            }
            Serial.println("Состояние: право/верх/разжат");
            break;
            
        default:
            Serial.print("Неизвестный шаг: ");
            Serial.println(step);
            return false;
    }
    
    // Пауза между шагами
    delay(200);
    
    // Сохраняем состояние после успешного выполнения шага
    saveWorkCycleState(step, false);
    
    return true;
}

// =============================================
// ОСНОВНЫЕ ФУНКЦИИ РАБОЧЕГО ЦИКЛА
// =============================================

bool checkWorkCycleInitialState() {
    updateAllSensors();
    
    Serial.println("\n=== ПРОВЕРКА НАЧАЛЬНОГО СОСТОЯНИЯ ===");
    
    bool rightLimit = isRightLimitPressed();
    bool zUp = isZUp();
    bool gripOpen = isGripOpen();
    
    Serial.print("Правый концевик: ");
    Serial.println(rightLimit ? "НАЖАТ" : "ОТПУЩЕН");
    
    Serial.print("Вертикаль: ");
    Serial.println(zUp ? "ВВЕРХУ" : "ВНИЗУ");
    
    Serial.print("Захват: ");
    Serial.println(gripOpen ? "ОТКРЫТ" : "ЗАКРЫТ");
    
    if (rightLimit && zUp && gripOpen) {
        Serial.println("✓ Начальное состояние корректно");
        return true;
    } else {
        Serial.println("✗ Ошибка: начальное состояние некорректно!");
        Serial.println("Требуется: право/верх/разжат");
        
        if (recoveryMode) {
            Serial.println("В режиме восстановления - пытаемся исправить...");
            return moveToStep(8);
        }
        
        return false;
    }
}

bool executeWorkCycle() {
    if (!isCalibrated) {
        Serial.println("Ошибка: калибровка не выполнена!");
        return false;
    }
    
    // Устанавливаем флаг выполнения цикла
    workCycleInProgress = true;
    currentStep = recoveryMode ? currentStep : 1;
    
    if (recoveryMode) {
        Serial.println("\n=== ПРОДОЛЖЕНИЕ РАБОЧЕГО ЦИКЛА ===");
        Serial.print("Продолжаем с шага ");
        Serial.println(currentStep);
        
        // В режиме восстановления проверяем текущее состояние
        uint8_t previousStep = currentStep - 1;
        if (previousStep > 0 && !verifyStepState(previousStep)) {
            Serial.println("Текущее состояние не соответствует ожидаемому!");
            Serial.println("Анализируем текущее состояние...");
            
            // Анализируем текущее состояние и корректируем
            uint8_t actualStep = analyzeCurrentStateAndGetStartStep(currentStep);
            currentStep = actualStep;
        }

        // Recovery продолжается уже после части цикла:
        // если стартуем строго позже 3/7 шага, соответствующие latch-и
        // должны быть подняты до входа в основной цикл шагов.
        if (currentStep > 3) {
            step3ReachedLatch = true;
        }
        if (currentStep > 7) {
            step7ReachedLatch = true;
        }
    } else {
        Serial.println("\n=== НАЧАЛО РАБОЧЕГО ЦИКЛА ===");
    }
    
    // Выполняем шаги, начиная с текущего
    for (uint8_t step = currentStep; step <= 8 && workCycleInProgress; step++) {
        currentStep = step;
        
        // Выполняем текущий шаг
        if (!executeStep(step)) {
            // Сохраняем состояние при ошибке
            saveWorkCycleState(step, true);
            workCycleInProgress = false;
            recoveryMode = false;
            resetWorkCycleReadyLatches();
            return false;
        }

        if (step == 3) {
            step3ReachedLatch = true;
        }

        if (step == 7) {
            step7ReachedLatch = true;
        }
    }
    
    // Цикл завершен успешно
    if (currentStep == 8) {
        Serial.println("\n✓ РАБОЧИЙ ЦИКЛ УСПЕШНО ЗАВЕРШЕН!");
        
        // Очищаем сохраненное состояние
        clearWorkCycleState();
        workCycleInProgress = false;
        recoveryMode = false;
        currentStep = 0;
        return true;
    }
    
    workCycleInProgress = false;
    recoveryMode = false;
    resetWorkCycleReadyLatches();
    return false;
}

bool startWorkCycle(bool fromRecovery) {
    if (workCycleInProgress) {
        Serial.println("Рабочий цикл уже выполняется!");
        return false;
    }
    
    if (!isCalibrated) {
        Serial.println("Ошибка: сначала выполните калибровку (K)!");
        return false;
    }
    
    if (isMoving) {
        Serial.println("Ошибка: система уже движется!");
        return false;
    }

    resetWorkCycleReadyLatches();
    
    // Если это восстановление, пропускаем проверку начального состояния
    recoveryMode = fromRecovery;
    
    if (!fromRecovery) {
        // Проверяем начальное состояние
        if (!checkWorkCycleInitialState()) {
            Serial.println("\nПриведите систему в начальное состояние:");
            Serial.println("1. Убедитесь, что манипулятор на правом концевике");
            Serial.println("2. Поднимите вертикаль (команда W)");
            Serial.println("3. Откройте захват (команда Q)");
            recoveryMode = false;
            return false;
        }
    }
    
    // Запускаем цикл
    bool result = executeWorkCycle();
    
    if (!result && recoveryMode) {
        // Если восстановление не удалось, сбрасываем флаги
        clearWorkCycleState();
        recoveryMode = false;
        currentStep = 0;
    }
    
    return result;
}

bool isWorkCycleInProgress() {
    return workCycleInProgress;
}

uint8_t getWorkCycleCurrentStep() {
    return currentStep;
}

bool isWorkCycleStep3Reached() {
    return step3ReachedLatch;
}

bool isWorkCycleStep7Reached() {
    return step7ReachedLatch;
}

// =============================================
// ОБНОВЛЕННАЯ ФУНКЦИЯ ВОССТАНОВЛЕНИЯ
// =============================================

bool recoverWorkCycle() {
    uint8_t savedStep;
    bool interrupted;

    if (!loadWorkCycleState(savedStep, interrupted)) {
        Serial.println("Нет сохраненного состояния для восстановления");
        return false;
    }

    if (savedStep == 0) {
        Serial.println("Сохраненный шаг 0 - восстановление не требуется");
        return false;
    }

    Serial.println("\n=== ВОССТАНОВЛЕНИЕ РАБОЧЕГО ЦИКЛА ===");
    Serial.print("Сохраненный шаг: ");
    Serial.print(savedStep);
    Serial.print(" (");

    // Описание шага
    switch (savedStep) {
        case 1: Serial.print("опускание"); break;
        case 2: Serial.print("сжатие"); break;
        case 3: Serial.print("подъем"); break;
        case 4: Serial.print("движение влево"); break;
        case 5: Serial.print("опускание"); break;
        case 6: Serial.print("разжатие"); break;
        case 7: Serial.print("подъем"); break;
        case 8: Serial.print("движение вправо"); break;
        default: Serial.print("неизвестный"); break;
    }

    Serial.print(")");
    Serial.print(", прервано: ");
    Serial.println(interrupted ? "да" : "нет");
    Serial.println("================================");

    Serial.println("Автоматическое безопасное восстановление...");

    // Выполняем безопасное восстановление для сохраненного шага
    if (!safeRecoveryForStep(savedStep)) {
        Serial.println("Не удалось безопасно восстановить состояние!");
        Serial.println("Попытка вернуться в начальное состояние...");

        // Пытаемся вернуться в начальное состояние (шаг 8)
        if (!moveToStep(8)) {
            Serial.println("Не удалось вернуться в начальное состояние!");
            Serial.println("Требуется ручное вмешательство!");
            clearWorkCycleState();
            resetWorkCycleReadyLatches();
            return false;
        }

        Serial.println("✓ Система в начальном состоянии, можно начинать новый цикл");
        clearWorkCycleState();
        resetWorkCycleReadyLatches();
        return false;
    }

    // ============================
    // ДОРАБОТКА (ВАРИАНТ 1 - БЕЗОПАСНЫЙ)
    // Если в EEPROM сохранен шаг 8, то НЕ запускаем новый цикл.
    // Только приводим систему в старт (это уже сделал safeRecoveryForStep(8)),
    // печатаем сообщение и очищаем состояние.
    // ============================
    if (savedStep == 8) {
        Serial.println("\nВосстановление: цикл уже завершён (шаг 8).");
        Serial.println("Манипулятор приведён в стартовое состояние.");
        Serial.println("Для запуска нового цикла нажмите R.");

        clearWorkCycleState();

        // Сбрасываем локальные флаги модуля цикла (на всякий случай)
        currentStep = 0;
        recoveryMode = false;
        workCycleInProgress = false;
        isMoving = false;
        resetWorkCycleReadyLatches();

        return false;
    }

    // Анализируем текущее состояние и определяем с какого шага продолжать
    uint8_t startStep = analyzeCurrentStateAndGetStartStep(savedStep);

    // Если мы уже прошли сохраненный шаг
    if (startStep != savedStep) {
        Serial.print("Сохраненный шаг уже пройден, продолжаем с шага ");
        Serial.println(startStep);
    } else {
        Serial.print("Продолжаем с сохраненного шага ");
        Serial.println(savedStep);
    }

    currentStep = startStep;
    recoveryMode = true;

    Serial.println("\n=== ЗАПУСК ВОССТАНОВЛЕНИЯ ===");

    // Перед началом восстановления убеждаемся, что данные сохранены
    saveWorkCycleState(startStep, true);
    delay(50); // Даем время для сохранения в EEPROM

    // Теперь запускаем восстановление
    bool result = startWorkCycle(true);

    // Если восстановление не удалось, сохраняем состояние для следующей попытки
    if (!result && recoveryMode) {
        Serial.println("Восстановление прервано, состояние сохранено для следующей попытки");
        saveWorkCycleState(currentStep, true);
    }

    return result;
}
