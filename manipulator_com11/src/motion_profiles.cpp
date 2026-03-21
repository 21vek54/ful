#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "motion_profiles.h"
#include "system_state.h"

// Глобальные переменные из system_state
extern bool isMoving;
extern bool isCalibrated;
extern unsigned long travelDistance;

// Фиксированные профили скорости (из оригинального кода)
const char* RIGHT_PROFILE = "{0,5;1000:0,5;977:0,5;945:0,5;901:0,5;847:0,5;781:0,5;704:0,5;617:0,5;530:0,5;442:0,5;366:0,5;300:0,5;246:0,5;202:0,5;169:0,5;147:0,5;136:0,5;131:0,5;128:0,5;125:0,5;125:1;115:1;105:75,5;100:1;105:1;115:0,5;125:0,5;128:0,5;131:0,5;136:0,5;147:0,5;169:0,5;202:0,5;246:0,5;300:0,5;366:0,5;442:0,5;530:0,5;617:0,5;704:0,5;781:0,5;847:0,5;901:0,5;945:0,5;977:0,5;1000}";
const char* LEFT_PROFILE = "{0,5;1000:0,5;977:0,5;945:0,5;901:0,5;847:0,5;781:0,5;704:0,5;617:0,5;530:0,5;442:0,5;366:0,5;300:0,5;246:0,5;202:0,5;169:0,5;147:0,5;136:0,5;131:0,5;128:0,5;125:0,5;125:1;115:1;105:75,5;100:1;105:1;115:0,5;125:0,5;128:0,5;131:0,5;136:0,5;147:0,5;169:0,5;202:0,5;246:0,5;300:0,5;366:0,5;442:0,5;530:0,5;617:0,5;704:0,5;781:0,5;847:0,5;901:0,5;945:0,5;977:0,5;1000}";

// ===== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ =====

// Безопасная остановка движения
void safeStopMovement() {
    isMoving = false;
    Serial.println("Движение остановлено");
}

// Проверка аварийных состояний во время движения
bool checkMovementSafety() {
    updateAllSensors();
    
    // Проверяем аварию драйвера
    if (stepperIsAlarm()) {
        Serial.println("!!! АВАРИЯ ДРАЙВЕРА! Остановка.");
        safeStopMovement();
        return false;
    }
    
    // Проверяем конфликты концевиков
    if (isBothLimitsPressed()) {
        Serial.println("!!! КОНФЛИКТ: оба концевика нажаты!");
        safeStopMovement();
        return false;
    }
    
    return true;
}

// ===== ОСНОВНЫЕ ФУНКЦИИ ПРОФИЛЕЙ =====

bool parseSpeedProfile(const String& profile_str, SpeedProfileSegment segments[], int& segment_count) {
    String str = profile_str;
    str.trim();
    
    if (!str.startsWith("{") || !str.endsWith("}")) {
        Serial.println("Ошибка: профиль должен быть в формате {X;Y:...}");
        return false;
    }
    
    // Заменяем запятые на точки для корректного парсинга
    str.replace(',', '.');
    
    str = str.substring(1, str.length() - 1);
    
    int start_pos = 0;
    int colon_pos = str.indexOf(':');
    segment_count = 0;
    float total_percent = 0.0;
    
    while (colon_pos != -1 && segment_count < MAX_PROFILE_SEGMENTS) {
        String segment_str = str.substring(start_pos, colon_pos);
        int semicolon_pos = segment_str.indexOf(';');
        
        if (semicolon_pos == -1) {
            Serial.println("Ошибка: неверный формат сегмента");
            return false;
        }
        
        String percent_str = segment_str.substring(0, semicolon_pos);
        String delay_str = segment_str.substring(semicolon_pos + 1);
        
        segments[segment_count].percent = percent_str.toFloat();
        segments[segment_count].delay_us = delay_str.toInt();
        
        total_percent += segments[segment_count].percent;
        segment_count++;
        
        start_pos = colon_pos + 1;
        colon_pos = str.indexOf(':', start_pos);
    }
    
    if (start_pos < str.length() && segment_count < MAX_PROFILE_SEGMENTS) {
        String segment_str = str.substring(start_pos);
        int semicolon_pos = segment_str.indexOf(';');
        
        if (semicolon_pos == -1) {
            Serial.println("Ошибка: неверный формат последнего сегмента");
            return false;
        }
        
        String percent_str = segment_str.substring(0, semicolon_pos);
        String delay_str = segment_str.substring(semicolon_pos + 1);
        
        segments[segment_count].percent = percent_str.toFloat();
        segments[segment_count].delay_us = delay_str.toInt();
        
        total_percent += segments[segment_count].percent;
        segment_count++;
    }
    
    // Проверяем сумму процентов
    if (abs(total_percent - 100.0) > 0.5) {
        Serial.print("Ошибка: сумма процентов (");
        Serial.print(total_percent, 2);
        Serial.println(") должна быть равна 100");
        return false;
    }
    
    return true;
}

// Выполнение движения по профилю (БЛОКИРУЮЩАЯ функция)
bool executeSpeedProfile(const String& profile_str, unsigned long total_steps, bool direction, bool require_limit) {
    if (isMoving) {
        Serial.println("Ошибка: движение уже выполняется");
        return false;
    }
    
    SpeedProfileSegment segments[MAX_PROFILE_SEGMENTS];
    int segment_count = 0;
    
    if (!parseSpeedProfile(profile_str, segments, segment_count)) {
        return false;
    }
    
    isMoving = true;
    Serial.print("Движение ");
    Serial.print(direction == HIGH ? "влево" : "вправо");
    Serial.print(" (");
    Serial.print(total_steps);
    Serial.println(" шагов)...");
    
    stepperSetDirection(direction);
    unsigned long steps_done = 0;
    unsigned long steps_remaining = total_steps;
    float accumulated_error = 0;
    bool limit_reached = false;
    bool target_limit_found = false;
    
    for (int seg = 0; seg < segment_count && isMoving; seg++) {
        // Вычисляем точное количество шагов для сегмента
        float exact_segment_steps = (segments[seg].percent / 100.0) * total_steps;
        exact_segment_steps += accumulated_error;
        unsigned long segment_steps = round(exact_segment_steps);
        accumulated_error = exact_segment_steps - segment_steps;
        
        // Для последнего сегмента берем все оставшиеся шаги
        if (seg == segment_count - 1) {
            segment_steps = steps_remaining;
        } else if (segment_steps > steps_remaining) {
            segment_steps = steps_remaining;
        }
        
        if (segment_steps == 0) continue;
        
        unsigned long segment_delay = segments[seg].delay_us;
        
        for (unsigned long i = 0; i < segment_steps && isMoving; i++) {
            // Проверяем безопасность
            if (!checkMovementSafety()) {
                return false;
            }
            
            updateAllSensors();
            
            // Проверяем целевой концевик
            if (direction == HIGH && isLeftLimitPressed()) {
                target_limit_found = true;
                limit_reached = true;
                break;
            } else if (direction == LOW && isRightLimitPressed()) {
                target_limit_found = true;
                limit_reached = true;
                break;
            }
            
            stepperStep(segment_delay);
            steps_done++;
            
            // УБРАН ВЫВОД ПРОГРЕССА КАЖДЫЕ 1000 ШАГОВ
            // Чтобы не засорять монитор порта
        }
        
        if (limit_reached) break;
        steps_remaining -= segment_steps;
    }
    
    isMoving = false;
    
    // Завершающие проверки
    updateAllSensors();
    
    // Если требуется концевик, проверяем его достижение
    if (require_limit) {
        if (target_limit_found) {
            Serial.print("✓ Достигнут концевик за ");
            Serial.print(steps_done);
            Serial.println(" шагов");
            return true;
        } else {
            Serial.print("✗ Конечный концевик не достигнут! ");
            Serial.print(steps_done);
            Serial.println(" шагов выполнено");
            return false;
        }
    } else {
        // Если концевик не требуется, считаем успешным завершение всех шагов
        Serial.print("✓ Движение завершено: ");
        Serial.print(steps_done);
        Serial.println(" шагов");
        return true;
    }
}

bool moveRightProfile(unsigned long total_steps) {
    String profile = String(RIGHT_PROFILE);
    profile.replace(',', '.');
    return executeSpeedProfile(profile, total_steps, LOW, true);
}

bool moveLeftProfile(unsigned long total_steps) {
    String profile = String(LEFT_PROFILE);
    profile.replace(',', '.');
    return executeSpeedProfile(profile, total_steps, HIGH, true);
}

// ===== БЕЗОПАСНЫЕ БЛОКИРУЮЩИЕ ФУНКЦИИ ДВИЖЕНИЯ =====

bool moveRightSafe(unsigned long timeout_ms) {
    Serial.println("Безопасное движение вправо...");
    
    if (!isCalibrated) {
        Serial.println("Ошибка: калибровка не выполнена");
        return false;
    }
    
    if (isMoving) {
        Serial.println("Ошибка: движение уже выполняется");
        return false;
    }
    
    // 1. Проверяем начальное состояние
    updateAllSensors();
    
    // Проверяем, что мы не на правом концевике
    if (isRightLimitPressed()) {
        Serial.println("Ошибка: правый концевик уже нажат!");
        return false;
    }
    
    // 2. Выполняем движение (блокирующее)
    unsigned long start_time = millis();
    bool result = moveRightProfile(travelDistance);
    
    // 3. Проверяем результат
    if (result) {
        Serial.println("✓ Движение вправо успешно завершено");
    } else {
        Serial.println("✗ Движение вправо не удалось");
    }
    
    // 4. Проверяем конечное состояние
    updateAllSensors();
    if (isRightLimitPressed()) {
        Serial.println("✓ Правый концевик достигнут");
    }
    
    return result;
}

bool moveLeftSafe(unsigned long timeout_ms) {
    Serial.println("Безопасное движение влево...");
    
    if (!isCalibrated) {
        Serial.println("Ошибка: калибровка не выполнена");
        return false;
    }
    
    if (isMoving) {
        Serial.println("Ошибка: движение уже выполняется");
        return false;
    }
    
    // 1. Проверяем начальное состояние
    updateAllSensors();
    
    // Проверяем, что мы не на левом концевике
    if (isLeftLimitPressed()) {
        Serial.println("Ошибка: левый концевик уже нажат!");
        return false;
    }
    
    // 2. Выполняем движение (блокирующее)
    unsigned long start_time = millis();
    bool result = moveLeftProfile(travelDistance);
    
    // 3. Проверяем результат
    if (result) {
        Serial.println("✓ Движение влево успешно завершено");
    } else {
        Serial.println("✗ Движение влево не удалось");
    }
    
    // 4. Проверяем конечное состояние
    updateAllSensors();
    if (isLeftLimitPressed()) {
        Serial.println("✓ Левый концевик достигнут");
    }
    
    return result;
}

// Функция для произвольного профиля (не требует концевика)
bool moveCustomProfile(const String& profile_str, unsigned long total_steps, bool direction) {
    if (isMoving) {
        Serial.println("Ошибка: движение уже выполняется");
        return false;
    }
    
    if (!isCalibrated) {
        Serial.println("Ошибка: калибровка не выполнена");
        return false;
    }
    
    return executeSpeedProfile(profile_str, total_steps, direction, false);
}