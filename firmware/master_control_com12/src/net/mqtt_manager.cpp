// Этот файл реализует минимальный MQTT менеджер нового master.
// Он отвечает за reconnect по таймеру, LWT и публикацию online-маркера.
// Его роль: дать стабильный MQTT транспорт для статусных сообщений.

#include "net/mqtt_manager.h"

#include <PubSubClient.h>
#include <WiFi.h>

#include "net/wifi_manager.h"

#if __has_include("mqtt_secrets_local.h")
#include "mqtt_secrets_local.h"
#else
#include "mqtt_secrets_local.h.example"
#endif

namespace {

constexpr unsigned long MQTT_RETRY_INTERVAL_MS = 5000;
constexpr uint16_t MQTT_BUFFER_SIZE = 256;

WiFiClient g_wifiClient;
PubSubClient g_mqttClient(g_wifiClient);

unsigned long g_lastMqttAttemptMs = 0;
String g_baseTopic = MQTT_LOCAL_BASE_TOPIC;
bool g_connectedEvent = false;

}  // namespace

void beginMqtt() {
    g_mqttClient.setServer(MQTT_LOCAL_HOST, MQTT_LOCAL_PORT);
    g_mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
}

bool mqttReady() {
    return g_mqttClient.connected();
}

String makeMqttTopic(const char *suffix) {
    String topic = g_baseTopic;
    topic += "/";
    topic += suffix;
    return topic;
}

bool publishRetained(const char *suffix, const char *payload) {
    if (!mqttReady()) {
        return false;
    }
    return g_mqttClient.publish(makeMqttTopic(suffix).c_str(), payload, true);
}

void publishOnlineMarker() {
    publishRetained("online", "1");
}

bool consumeMqttConnectedEvent() {
    if (!g_connectedEvent) {
        return false;
    }
    g_connectedEvent = false;
    return true;
}

void ensureMqttConnected(unsigned long nowMs) {
    if (!wifiReady() || mqttReady()) {
        return;
    }

    if ((nowMs - g_lastMqttAttemptMs) < MQTT_RETRY_INTERVAL_MS) {
        return;
    }

    g_lastMqttAttemptMs = nowMs;

    const uint64_t chipId = ESP.getEfuseMac();
    const String clientId =
        "master_com12_" + String(static_cast<uint32_t>(chipId & 0xFFFFFFFFULL), HEX);

    Serial.println("[mqtt] connect...");

    const bool connected = g_mqttClient.connect(
        clientId.c_str(),
        MQTT_LOCAL_USER,
        MQTT_LOCAL_PASSWORD,
        makeMqttTopic("online").c_str(),
        0,
        true,
        "0");

    if (connected) {
        Serial.println("[mqtt] connected");
        g_connectedEvent = true;
        publishOnlineMarker();
    } else {
        Serial.print("[mqtt] failed, state=");
        Serial.println(g_mqttClient.state());
    }
}

void mqttLoopTick() {
    if (mqttReady()) {
        g_mqttClient.loop();
    }
}
