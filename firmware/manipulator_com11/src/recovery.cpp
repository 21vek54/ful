#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include "config.h"
#include "recovery.h"
#include "sensors.h"
#include "system_state.h"

// Глобальные переменные
extern bool workCycleInProgress;
extern bool isCalibrated;
extern unsigned long travelDistance;

// Объявление структуры EEPROMData (уже определена в config.h, но нужно здесь)
EEPROMData eepromData;

// =============================================
// РАБОТА С EEPROM
// =============================================

void recoveryInit() {
    // Инициализация EEPROM
    EEPROM.begin(EEPROM_SIZE);
    
    // Проверяем целостность данных
    uint16_t magic = EEPROM.read(0) | (EEPROM.read(1) << 8);
    uint8_t version = EEPROM.read(2);
    
    if (magic != EEPROM_MAGIC_NUMBER || (version != 1 && version != 2)) {
        // Нет валидных данных, инициализируем
        clearSystemState();
        Serial.println("EEPROM: нет валидных данных, инициализирован");
    } else {
        Serial.println("EEPROM: данные загружены");
        
        // Загружаем системные параметры
        loadSystemParameters();
    }
}

void saveWorkCycleState(uint8_t step, bool interrupted) {
    EEPROMData data;
    
    // Загружаем существующие данные
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        ptr[i] = EEPROM.read(EEPROM_START_ADDR + i);
    }
    
    // Обновляем поля
    data.magic = EEPROM_MAGIC_NUMBER;
    data.version = 2;  // Новая версия
    data.workCycleStep = step;
    data.savedTime = millis() / 1000;
    data.systemState = (workCycleInProgress ? 0x01 : 0x00);
    
    // Сохраняем текущие системные параметры
    data.isCalibrated = isCalibrated;
    data.travelDistance = travelDistance;
    
    // Сохраняем в EEPROM
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        EEPROM.write(EEPROM_START_ADDR + i, ptr[i]);
    }
    
    EEPROM.commit();
    
    Serial.print("Сохранено состояние цикла: шаг ");
    Serial.print(step);
    Serial.print(", прерван: ");
    Serial.println(interrupted ? "да" : "нет");
}

bool loadWorkCycleState(uint8_t &step, bool &interrupted) {
    EEPROMData data;
    
    // Читаем из EEPROM
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        ptr[i] = EEPROM.read(EEPROM_START_ADDR + i);
    }
    
    // Проверяем магическое число
    if (data.magic != EEPROM_MAGIC_NUMBER) {
        return false;
    }
    
    // Проверяем версию
    if (data.version == 1) {
        // Старая версия данных - загружаем только основные поля
        step = data.workCycleStep;
        interrupted = (data.systemState & 0x01) != 0;
        return true;
    } else if (data.version == 2) {
        // Новая версия - загружаем все поля
        step = data.workCycleStep;
        interrupted = (data.systemState & 0x01) != 0;
        return true;
    }
    
    return false;
}

void clearWorkCycleState() {
    // Не очищаем полностью, только сбрасываем шаг цикла
    EEPROMData data;
    
    // Загружаем существующие данные
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        ptr[i] = EEPROM.read(EEPROM_START_ADDR + i);
    }
    
    // Обновляем только шаг цикла
    data.workCycleStep = 0;
    data.systemState = 0;
    
    // Сохраняем обратно
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        EEPROM.write(EEPROM_START_ADDR + i, ptr[i]);
    }
    
    EEPROM.commit();
    Serial.println("Состояние рабочего цикла очищено (шаг = 0)");
}

void clearSystemState() {
    EEPROMData data;
    
    // Инициализируем структуру нулями
    memset(&data, 0, sizeof(EEPROMData));
    
    // Устанавливаем заголовочные поля
    data.magic = EEPROM_MAGIC_NUMBER;
    data.version = 2;
    data.workCycleStep = 0;
    data.savedTime = 0;
    data.systemState = 0;
    data.isCalibrated = false;
    data.travelDistance = DEFAULT_STEPS;
    
    // Сохраняем в EEPROM
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        EEPROM.write(EEPROM_START_ADDR + i, ptr[i]);
    }
    
    EEPROM.commit();
    Serial.println("Все состояние системы очищено");
}

bool hasSavedWorkCycleState() {
    uint8_t step;
    bool interrupted;
    return loadWorkCycleState(step, interrupted) && step > 0;
}

void saveSystemParameters() {
    EEPROMData data;
    
    // Загружаем существующие данные
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        ptr[i] = EEPROM.read(EEPROM_START_ADDR + i);
    }
    
    // Обновляем системные параметры
    data.magic = EEPROM_MAGIC_NUMBER;
    data.version = 2;  // Новая версия
    data.savedTime = millis() / 1000;
    
    // Сохраняем текущие системные параметры
    data.isCalibrated = isCalibrated;
    data.travelDistance = travelDistance;
    
    // Сохраняем обратно
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        EEPROM.write(EEPROM_START_ADDR + i, ptr[i]);
    }
    
    EEPROM.commit();
    
    // УБРАНО: Вывод в лог, чтобы не засорять
    // Serial.println("Системные параметры сохранены:");
    // Serial.print("  Калибровка: ");
    // Serial.println(isCalibrated ? "да" : "нет");
    // Serial.print("  Расстояние: ");
    // Serial.print(travelDistance);
    // Serial.println(" шагов");
}

void loadSystemParameters() {
    EEPROMData data;
    
    // Читаем из EEPROM
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        ptr[i] = EEPROM.read(EEPROM_START_ADDR + i);
    }
    
    // Проверяем магическое число
    if (data.magic != EEPROM_MAGIC_NUMBER) {
        Serial.println("EEPROM: нет валидных данных, используем заводские настройки");
        isCalibrated = false;
        travelDistance = DEFAULT_STEPS;
        return;
    }
    
    // Проверяем версию
    if (data.version == 1) {
        // Старая версия - только базовые поля
        Serial.println("EEPROM: версия 1, основные параметры загружены");
        // Калибровку и расстояние не загружаем из старой версии
        isCalibrated = false;
        travelDistance = DEFAULT_STEPS;
    } else if (data.version == 2) {
        // Новая версия - загружаем все параметры
        isCalibrated = data.isCalibrated;
        travelDistance = data.travelDistance;
        
        Serial.println("EEPROM: версия 2, все параметры загружены");
        Serial.print("  Калибровка: ");
        Serial.println(isCalibrated ? "да" : "нет");
        Serial.print("  Расстояние: ");
        Serial.print(travelDistance);
        Serial.println(" шагов");
        
        // Обновляем глобальные переменные в system_state.cpp
        // (они уже объявлены как extern и будут изменены напрямую)
    } else {
        Serial.print("EEPROM: неизвестная версия ");
        Serial.print(data.version);
        Serial.println(", используем заводские настройки");
        isCalibrated = false;
        travelDistance = DEFAULT_STEPS;
    }
}

void printRecoveryInfo() {
    EEPROMData data;
    
    // Читаем из EEPROM
    uint8_t *ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        ptr[i] = EEPROM.read(EEPROM_START_ADDR + i);
    }
    
    Serial.println("\n=== ИНФОРМАЦИЯ О ВОССТАНОВЛЕНИИ ===");
    Serial.print("Магическое число: 0x");
    Serial.println(data.magic, HEX);
    Serial.print("Версия данных: ");
    Serial.println(data.version);
    Serial.print("Шаг рабочего цикла: ");
    Serial.println(data.workCycleStep);
    Serial.print("Время сохранения: ");
    Serial.print(data.savedTime);
    Serial.println(" сек");
    Serial.print("Состояние системы: 0x");
    Serial.println(data.systemState, HEX);
    
    if (data.version >= 2) {
        Serial.print("Калибровка: ");
        Serial.println(data.isCalibrated ? "да" : "нет");
        Serial.print("Расстояние: ");
        Serial.print(data.travelDistance);
        Serial.println(" шагов");
    } else {
        Serial.println("Калибровка: данные отсутствуют (старая версия)");
        Serial.println("Расстояние: данные отсутствуют (старая версия)");
    }
    
    if (data.magic == EEPROM_MAGIC_NUMBER && (data.version == 1 || data.version == 2) && data.workCycleStep > 0) {
        Serial.println("✓ Есть сохраненное состояние для восстановления");
    } else {
        Serial.println("✗ Нет сохраненного состояния");
    }
    Serial.println("====================================");
}