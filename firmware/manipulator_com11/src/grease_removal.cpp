#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "motion_profiles.h"
#include "grease_removal.h"
#include "system_state.h"

// Глобальные переменные процедуры
bool greaseRemovalInProgress = false;
int currentGreaseDistance = 18100;
bool currentDirection = true; // true = влево, false = вправо
unsigned long greaseCycleCounter = 0;

// Константы процедуры (определяем здесь вместо config.h)
const int GR_START_DISTANCE = 18100;
const int GR_END_DISTANCE = 2400;
const int GR_STEP_REDUCTION = 100;
const unsigned long GR_STEP_DELAY = 500; // задержка шагов между импульсам при выгонке масла

bool isReadyForGreaseRemoval() {
    // Проверяем начальное положение
    updateAllSensors();
    
    if (!isRightLimitPressed()) {
        Serial.println("Ошибка: каретка должна находиться на правом концевике!");
        return false;
    }
    
    if (isMoving) {
        Serial.println("Ошибка: система уже движется!");
        return false;
    }
    
    if (!isCalibrated) {
        Serial.println("Предупреждение: калибровка не выполнена, но процедура возможна");
        // Не блокируем, но предупреждаем
    }
    
    return true;
}

void startGreaseRemoval() {
    if (greaseRemovalInProgress) {
        Serial.println("Процедура удаления смазки уже выполняется!");
        return;
    }
    
    if (!isReadyForGreaseRemoval()) {
        return;
    }
    
    Serial.println("\n=== ЗАПУСК ПРОЦЕДУРЫ УДАЛЕНИЯ СМАЗКИ ===");
    Serial.println("Начальное положение: правый концевик");
    Serial.println("Скорость: средняя (задержка 700 мкс)");
    Serial.println("Схема: 18100← →18000 ←17900 →17800 ←17700 ... →2400");
    Serial.println("Уменьшение на 100 шагов после КАЖДОГО движения");
    
    // Инициализируем параметры
    greaseRemovalInProgress = true;
    currentGreaseDistance = GR_START_DISTANCE;
    currentDirection = true; // начинаем с движения влево
    greaseCycleCounter = 0;
    
    Serial.print("Первое движение: влево на ");
    Serial.print(currentGreaseDistance);
    Serial.println(" шагов");
    
    Serial.println("Для остановки используйте команду Z");
    Serial.println("==========================================");
}

void stopGreaseRemoval() {
    if (greaseRemovalInProgress) {
        greaseRemovalInProgress = false;
        Serial.println("\n!!! ПРОЦЕДУРА УДАЛЕНИЯ СМАЗКИ ОСТАНОВЛЕНА !!!");
        Serial.println("Текущее состояние сохранено");
    }
}

bool performGreaseMovement() {
    if (!greaseRemovalInProgress || isMoving) {
        return false;
    }
    
    // Проверяем завершение процедуры
    if (currentGreaseDistance <= GR_END_DISTANCE) {
        Serial.println("\n=== ПРОЦЕДУРА УДАЛЕНИЯ СМАЗКИ ЗАВЕРШЕНА ===");
        Serial.print("Выполнено циклов: ");
        Serial.println(greaseCycleCounter);
        Serial.println("Смазка равномерно распределена!");
        
        // Делаем паузу и возвращаемся на правый концевик
        delay(1000);
        Serial.println("Возврат на правый концевик...");
        
        // Создаем простой профиль для возврата
        String returnProfile = "{100;700}"; // Используем ту же скорость
        bool success = executeSpeedProfile(returnProfile, 2400, LOW, true); // движение вправо
        
        if (success) {
            Serial.println("✓ Успешно вернулись на правый концевик");
        } else {
            Serial.println("✗ Не удалось вернуться на правый концевик");
        }
        
        greaseRemovalInProgress = false;
        return false;
    }
    
    // Выполняем движение
    greaseCycleCounter++;
    
    Serial.print("\nЦикл ");
    Serial.print(greaseCycleCounter);
    Serial.print(": движение ");
    Serial.print(currentDirection ? "влево" : "вправо");
    Serial.print(" на ");
    Serial.print(currentGreaseDistance);
    Serial.println(" шагов");
    
    // Создаем простой профиль с постоянной скоростью
    String simpleProfile = "{100;700}"; // 100% расстояния, задержка 700 мкс (было 1000)
    
    // Выполняем движение
    bool success;
    if (currentDirection) {
        // Движение влево
        success = executeSpeedProfile(simpleProfile, currentGreaseDistance, HIGH, false);
    } else {
        // Движение вправо
        success = executeSpeedProfile(simpleProfile, currentGreaseDistance, LOW, false);
    }
    
    if (!success) {
        Serial.println("Ошибка движения! Процедура остановлена.");
        greaseRemovalInProgress = false;
        return false;
    }
    
    // Уменьшаем расстояние для следующего движения (после КАЖДОГО движения)
    currentGreaseDistance -= GR_STEP_REDUCTION;
    
    // Проверяем, не стало ли расстояние меньше конечного
    if (currentGreaseDistance < GR_END_DISTANCE) {
        currentGreaseDistance = GR_END_DISTANCE;
    }
    
    // Меняем направление для следующего движения
    currentDirection = !currentDirection;
    
    // Выводим информацию о следующем движении
    if (currentGreaseDistance > GR_END_DISTANCE) {
        Serial.print("Следующее: ");
        Serial.print(currentDirection ? "влево" : "вправо");
        Serial.print(" на ");
        Serial.print(currentGreaseDistance);
        Serial.println(" шагов");
        
        // Выводим прогресс
        float progress = 100.0 * (1.0 - (float)(currentGreaseDistance - GR_END_DISTANCE) / 
                              (GR_START_DISTANCE - GR_END_DISTANCE));
        Serial.print("Прогресс: ");
        Serial.print(progress, 1);
        Serial.println("%");
    }
    
    // Короткая пауза между движениями
    delay(300); // Немного уменьшили паузу, так как скорость движения выше
    
    return true;
}

void updateGreaseRemoval() {
    if (!greaseRemovalInProgress) return;
    
    // Если система не движется, выполняем следующее движение
    if (!isMoving) {
        performGreaseMovement();
    }
}

void printGreaseRemovalStatus() {
    if (!greaseRemovalInProgress) {
        Serial.println("Процедура удаления смазки не выполняется");
        return;
    }
    
    Serial.println("\n=== СТАТУС ПРОЦЕДУРЫ УДАЛЕНИЯ СМАЗКИ ===");
    Serial.print("Текущий цикл: ");
    Serial.println(greaseCycleCounter);
    
    Serial.print("Следующее движение: ");
    Serial.print(currentDirection ? "ВЛЕВО" : "ВПРАВО");
    Serial.print(" на ");
    Serial.print(currentGreaseDistance);
    Serial.println(" шагов");
    
    // Рассчитываем общее количество движений
    int totalMovements = (GR_START_DISTANCE - GR_END_DISTANCE) / GR_STEP_REDUCTION + 1;
    int movementsRemaining = (currentGreaseDistance - GR_END_DISTANCE) / GR_STEP_REDUCTION + 1;
    
    Serial.print("Выполнено движений: ");
    Serial.print(greaseCycleCounter);
    Serial.print(" из ~");
    Serial.println(totalMovements);
    
    Serial.print("Осталось движений: ~");
    Serial.println(movementsRemaining);
    
    float progress = 100.0 * (float)greaseCycleCounter / totalMovements;
    Serial.print("Прогресс: ");
    Serial.print(progress, 1);
    Serial.println("%");
    
    Serial.print("Скорость: ");
    Serial.print(1000000.0 / GR_STEP_DELAY, 0); // шагов в секунду
    Serial.println(" шаг/сек");
    
    Serial.println("==========================================");
}