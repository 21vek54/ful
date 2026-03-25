// Этот файл реализует минимальный Wi-Fi менеджер нового master.
// Он отвечает за периодические retry-попытки подключения без перезагрузки.
// Его роль: поддерживать готовность сетевого канала для MQTT.

#include "net/wifi_manager.h"

#include <WiFi.h>

#if __has_include("wifi_secrets_local.h")
#include "wifi_secrets_local.h"
#else
#include "wifi_secrets_local.h.example"
#endif

namespace {

constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

unsigned long g_lastWifiAttemptMs = 0;

}  // namespace

bool wifiReady() {
    return WiFi.status() == WL_CONNECTED;
}

void ensureWifiConnected(unsigned long nowMs) {
    if (wifiReady()) {
        return;
    }

    if ((nowMs - g_lastWifiAttemptMs) < WIFI_RETRY_INTERVAL_MS) {
        return;
    }

    g_lastWifiAttemptMs = nowMs;
    Serial.println("[wifi] connect...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_LOCAL_SSID, WIFI_LOCAL_PASSWORD);
}
