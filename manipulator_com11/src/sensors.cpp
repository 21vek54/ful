#include <Arduino.h>
#include "config.h"
#include "sensors.h"

// ===== ОПРЕДЕЛЕНИЕ ВСЕХ ОБЪЕКТОВ ДАТЧИКОВ =====
// Минимальное время стабильности: 50 мс для концевиков, 100 мс для пневматики
ReedSwitch leftLimit = {false, false, 0, 0, 0, "Левый концевик", LIMIT_SWITCH_LEFT, 50};
ReedSwitch rightLimit = {false, false, 0, 0, 0, "Правый концевик", LIMIT_SWITCH_RIGHT, 50};
ReedSwitch gripOpen = {false, false, 0, 0, 0, "Захват открыт", GRIP_OPEN_SW, 100};
ReedSwitch gripClosed = {false, false, 0, 0, 0, "Захват закрыт", GRIP_CLOSED_SW, 100};
ReedSwitch zUp = {false, false, 0, 0, 0, "Вертикаль вверх", Z_UP_SW, 100};
ReedSwitch zDown = {false, false, 0, 0, 0, "Вертикаль вниз", Z_DOWN_SW, 100};

// ===== УСИЛЕННАЯ ФУНКЦИЯ ФИЛЬТРАЦИИ ДРЕБЕЗГА =====
static void updateSensor(ReedSwitch &sensor) {
  unsigned long currentTime = millis();

  // Проверяем, прошло ли достаточно времени с последней проверки
  if (currentTime - sensor.lastCheckTime >= DEBOUNCE_INTERVAL) {
    sensor.lastCheckTime = currentTime;

    // Читаем состояние пина (активный уровень LOW для всех герконов)
    bool currentState = (digitalRead(sensor.pin) == LOW);

    // Фильтр 1: изменилось ли сырое состояние?
    if (currentState != sensor.rawState) {
      sensor.rawState = currentState;
      sensor.consecutiveCount = 1;
      sensor.lastChangeTime = currentTime;

      // Отладка изменения сырого состояния
      // Serial.print(sensor.name);
      // Serial.print(": сырое состояние изменилось на ");
      // Serial.println(currentState ? "НАЖАТ" : "ОТПУЩЕН");
    } else {
      // Состояние не изменилось - увеличиваем счетчик
      sensor.consecutiveCount++;

      // Фильтр 2: достаточно ли одинаковых показаний?
      if (sensor.consecutiveCount >= DEBOUNCE_SAMPLES) {
        // Фильтр 3: прошло ли минимальное время стабильности?
        if (currentTime - sensor.lastChangeTime >= sensor.minStableTime) {
          // Фильтр 4: отличается ли состояние от фильтрованного?
          if (currentState != sensor.stableState) {
            sensor.stableState = currentState;

            // Логируем устойчивое изменение состояния для отладки
            Serial.print("Датчик '");
            Serial.print(sensor.name);
            Serial.print("' стабильно изменил состояние на: ");
            Serial.println(currentState ? "НАЖАТ" : "ОТПУЩЕН");
          }
        }
      }
    }
  }
}

// ===== ИНИЦИАЛИЗАЦИЯ ВСЕХ ДАТЧИКОВ =====
void initAllSensors() {
  // Настраиваем пины горизонтальных концевиков
  pinMode(LIMIT_SWITCH_LEFT, INPUT);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT);

  // Настраиваем пины датчиков пневматики
  pinMode(GRIP_OPEN_SW, INPUT_PULLUP);
  pinMode(GRIP_CLOSED_SW, INPUT_PULLUP);
  pinMode(Z_UP_SW, INPUT_PULLUP);
  pinMode(Z_DOWN_SW, INPUT_PULLUP);

  // Инициализируем все структуры
  leftLimit.stableState = leftLimit.rawState = (digitalRead(LIMIT_SWITCH_LEFT) == LOW);
  rightLimit.stableState = rightLimit.rawState = (digitalRead(LIMIT_SWITCH_RIGHT) == LOW);
  gripOpen.stableState = gripOpen.rawState = (digitalRead(GRIP_OPEN_SW) == LOW);
  gripClosed.stableState = gripClosed.rawState = (digitalRead(GRIP_CLOSED_SW) == LOW);
  zUp.stableState = zUp.rawState = (digitalRead(Z_UP_SW) == LOW);
  zDown.stableState = zDown.rawState = (digitalRead(Z_DOWN_SW) == LOW);

  Serial.println("Все датчики инициализированы с усиленной фильтрацией");
}

// ===== ОБНОВЛЕНИЕ ВСЕХ ДАТЧИКОВ =====
void updateAllSensors() {
  updateSensor(leftLimit);
  updateSensor(rightLimit);
  updateSensor(gripOpen);
  updateSensor(gripClosed);
  updateSensor(zUp);
  updateSensor(zDown);
}

// ===== БЫСТРЫЕ ПРОВЕРКИ СОСТОЯНИЙ (фильтрованные) =====
bool isLeftLimitPressed() { return leftLimit.stableState; }
bool isRightLimitPressed() { return rightLimit.stableState; }
bool isGripOpen() { return gripOpen.stableState; }
bool isGripClosed() { return gripClosed.stableState; }
bool isZUp() { return zUp.stableState; }
bool isZDown() { return zDown.stableState; }

// ===== ФУНКЦИИ ДЛЯ ОТЛАДКИ (сырые состояния) =====
bool isLeftLimitPressedRaw() { return leftLimit.rawState; }
bool isRightLimitPressedRaw() { return rightLimit.rawState; }
bool isGripOpenRaw() { return gripOpen.rawState; }
bool isGripClosedRaw() { return gripClosed.rawState; }
bool isZUpRaw() { return zUp.rawState; }
bool isZDownRaw() { return zDown.rawState; }

// ===== УСИЛЕННЫЕ ФУНКЦИИ ОЖИДАНИЯ =====
bool waitForStableState(ReedSwitch &sensor, bool desiredState, unsigned long timeout_ms) {
  unsigned long startTime = millis();
  unsigned long stableStartTime = 0;
  bool wasStable = false;

  Serial.print("Ожидание стабильного состояния '");
  Serial.print(sensor.name);
  Serial.print("' = ");
  Serial.println(desiredState ? "НАЖАТ" : "ОТПУЩЕН");

  while (millis() - startTime < timeout_ms) {
    updateSensor(sensor);

    if (sensor.stableState == desiredState) {
      if (!wasStable) {
        // Впервые достигли нужного состояния
        stableStartTime = millis();
        wasStable = true;
        Serial.print("  Достигнуто состояние, ожидаю стабильности ");
        Serial.print(sensor.minStableTime);
        Serial.println(" мс");
      }

      // Проверяем, стабильно ли состояние достаточно долго
      if (millis() - stableStartTime >= sensor.minStableTime) {
        Serial.print("  ✓ '");
        Serial.print(sensor.name);
        Serial.print("' стабильно ");
        Serial.print(desiredState ? "НАЖАТ" : "ОТПУЩЕН");
        Serial.print(" (");
        Serial.print(millis() - stableStartTime);
        Serial.println(" мс)");
        return true;
      }
    } else {
      // Сбрасываем ожидание, если состояние снова ушло
      if (wasStable) {
        Serial.println("  Состояние изменилось до достижения стабильности!");
        wasStable = false;
      }
    }

    delay(10);
  }

  Serial.print("  ✗ Таймаут ожидания '");
  Serial.print(sensor.name);
  Serial.print("' (");
  Serial.print(timeout_ms);
  Serial.println(" мс)");
  return false;
}

// Функции ожидания для конкретных датчиков
bool waitForLeftLimit(bool pressed, unsigned long timeout_ms) {
  return waitForStableState(leftLimit, pressed, timeout_ms);
}

bool waitForRightLimit(bool pressed, unsigned long timeout_ms) {
  return waitForStableState(rightLimit, pressed, timeout_ms);
}

bool waitForGripOpen(unsigned long timeout_ms) {
  return waitForStableState(gripOpen, true, timeout_ms);
}

bool waitForGripClosed(unsigned long timeout_ms) {
  return waitForStableState(gripClosed, true, timeout_ms);
}

bool waitForZUp(unsigned long timeout_ms) {
  return waitForStableState(zUp, true, timeout_ms);
}

bool waitForZDown(unsigned long timeout_ms) {
  return waitForStableState(zDown, true, timeout_ms);
}

// ===== ПРОВЕРКА КОНФЛИКТНЫХ СИТУАЦИЙ =====
bool isBothLimitsPressed() {
  return isLeftLimitPressed() && isRightLimitPressed();
}

bool isGripConflict() {
  return isGripOpen() && isGripClosed();
}

bool isZConflict() {
  return isZUp() && isZDown();
}

bool isAnyConflict() {
  return isBothLimitsPressed() || isGripConflict() || isZConflict();
}

// ===== ДИАГНОСТИКА И ОТЛАДКА =====
void printSensorStatus(ReedSwitch &sensor) {
  Serial.print(sensor.name);
  Serial.print(": ");
  Serial.print(sensor.stableState ? "НАЖАТ(f)" : "ОТПУЩЕН(f)");
  Serial.print(" [сырое: ");
  Serial.print(sensor.rawState ? "НАЖАТ(r)" : "ОТПУЩЕН(r)");
  Serial.print(", счетчик: ");
  Serial.print(sensor.consecutiveCount);
  Serial.print(", мин.стаб: ");
  Serial.print(sensor.minStableTime);
  Serial.print("мс, пин: ");
  Serial.print(sensor.pin);
  Serial.println("]");
}

void printAllSensorsStatus() {
  Serial.println("\n=== СОСТОЯНИЕ ВСЕХ ДАТЧИКОВ (усиленная фильтрация) ===");
  printSensorStatus(leftLimit);
  printSensorStatus(rightLimit);
  printSensorStatus(gripOpen);
  printSensorStatus(gripClosed);
  printSensorStatus(zUp);
  printSensorStatus(zDown);

  bool hasConflict = false;

  if (isBothLimitsPressed()) {
    Serial.println("!!! АВАРИЯ: Оба горизонтальных концевика нажаты!");
    hasConflict = true;
  }
  if (isGripConflict()) {
    Serial.println("!!! АВАРИЯ: Конфликт датчиков захвата!");
    hasConflict = true;
  }
  if (isZConflict()) {
    Serial.println("!!! АВАРИЯ: Конфликт датчиков вертикали!");
    hasConflict = true;
  }

  if (!hasConflict) {
    Serial.println("✓ Конфликтов датчиков не обнаружено");
  }

  Serial.println("==================================================");
}
