#pragma once

// Этот файл объявляет минимальный Wi-Fi менеджер нового master.
// Он отвечает за проверку состояния и неблокирующие попытки подключения.
// Его роль: дать стабильный Wi-Fi слой для MQTT и статуса.

#include <Arduino.h>

bool wifiReady();
void ensureWifiConnected(unsigned long nowMs);
