#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================
// МОДУЛЬ РАБОТЫ СО ВСЕМИ ГЕРКОНАМИ СИСТЕМЫ
// Объединяет: концевики горизонтального перемещения
//           + датчики положения пневматики
// =============================================

// Структура фильтра для любого геркона (антидребезг)
struct ReedSwitch {
  bool stableState;        // Фильтрованное состояние (без дребезга)
  bool rawState;           // Сырое состояние (мгновенное с пина)
  int consecutiveCount;    // Счетчик одинаковых показаний
  unsigned long lastCheckTime; // Время последней проверки
  unsigned long lastChangeTime; // Время последнего изменения состояния
  const char* name;        // Имя датчика для отладки
  int pin;                 // Номер пина Arduino
  int minStableTime;       // Минимальное время стабильности (мс)
};

// ВСЕ ГЕРКОНЫ СИСТЕМЫ (объявлены в sensors.cpp)
extern ReedSwitch leftLimit;     // Левый концевой горизонтального перемещения
extern ReedSwitch rightLimit;    // Правый концевой горизонтального перемещения
extern ReedSwitch gripOpen;      // Захват открыт
extern ReedSwitch gripClosed;    // Захват закрыт
extern ReedSwitch zUp;           // Вертикаль в верхнем положении
extern ReedSwitch zDown;         // Вертикаль в нижнем положении

// ===== ИНИЦИАЛИЗАЦИЯ И ОБНОВЛЕНИЕ =====
void initAllSensors();           // Настройка пинов и инициализация структур
void updateAllSensors();         // Обновление состояний ВСЕХ датчиков (вызывать в loop)

// ===== БЫСТРЫЕ ПРОВЕРКИ СОСТОЯНИЙ (фильтрованные) =====
// Горизонтальные концевики
bool isLeftLimitPressed();       // TRUE если левый концевик нажат
bool isRightLimitPressed();      // TRUE если правый концевик нажат

// Пневматика - захват
bool isGripOpen();               // TRUE если захват полностью открыт
bool isGripClosed();             // TRUE если захват полностью закрыт

// Пневматика - вертикаль
bool isZUp();                    // TRUE если вертикаль в верхнем положении
bool isZDown();                  // TRUE если вертикаль в нижнем положении

// ===== ФУНКЦИИ ДЛЯ ОТЛАДКИ И ДИАГНОСТИКИ =====
// Возвращают мгновенные (нефильтрованные) состояния
bool isLeftLimitPressedRaw();    // Сырое состояние левого концевика
bool isRightLimitPressedRaw();   // Сырое состояние правого концевика
bool isGripOpenRaw();            // Сырое состояние датчика "открыт"
bool isGripClosedRaw();          // Сырое состояние датчика "закрыт"
bool isZUpRaw();                 // Сырое состояние датчика "верх"
bool isZDownRaw();               // Сырое состояние датчика "низ"

// ===== УСИЛЕННАЯ ФИЛЬТРАЦИЯ И ОЖИДАНИЕ =====
// Ожидание стабильного состояния датчика
bool waitForStableState(ReedSwitch &sensor, bool desiredState, unsigned long timeout_ms);

// Функции ожидания для конкретных датчиков
bool waitForLeftLimit(bool pressed, unsigned long timeout_ms = 5000);
bool waitForRightLimit(bool pressed, unsigned long timeout_ms = 5000);
bool waitForGripOpen(unsigned long timeout_ms = 3000);
bool waitForGripClosed(unsigned long timeout_ms = 3000);
bool waitForZUp(unsigned long timeout_ms = 3000);
bool waitForZDown(unsigned long timeout_ms = 3000);

// ===== ПРОВЕРКА КОНФЛИКТОВ И БЕЗОПАСНОСТИ =====
bool isBothLimitsPressed();      // TRUE если оба горизонтальных концевика нажаты
bool isGripConflict();           // TRUE если оба датчика захвата активны (не должно быть)
bool isZConflict();              // TRUE если оба датчика вертикали активны (не должно быть)
bool isAnyConflict();            // TRUE если есть любой конфликт датчиков

// ===== ДИАГНОСТИКА И ОТЛАДКА =====
void printAllSensorsStatus();    // Вывод статуса всех датчиков в Serial
void printSensorStatus(ReedSwitch &sensor); // Вывод статуса конкретного датчика