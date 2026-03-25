// Этот файл поднимает новый минимальный master под PlatformIO.
// Он отвечает за оркестрацию модулей Wi-Fi, MQTT и I2C-опроса плат 12/13.
// Его роль: держать setup/loop простыми и прозрачными для поддержки.

#include <Arduino.h>

#include "comm/i2c_poller.h"
#include "core/runtime_state.h"
#include "net/mqtt_manager.h"
#include "net/status_publisher.h"
#include "net/wifi_manager.h"
#include "pins.h"

namespace {

RuntimeState g_runtimeState;

void printStartupBanner() {
    Serial.println();
    Serial.println("master_control_com12 minimal");
    Serial.println("role: Wi-Fi + MQTT + I2C board online/offline");
    Serial.print("i2c sda=");
    Serial.print(PIN_I2C_SDA);
    Serial.print(" scl=");
    Serial.println(PIN_I2C_SCL);
    Serial.print("devices: ");
    Serial.print(g_runtimeState.boards[0].id);
    Serial.print(", ");
    Serial.println(g_runtimeState.boards[1].id);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);

    initRuntimeState(g_runtimeState);
    printStartupBanner();

    beginI2c();
    beginMqtt();
}

void loop() {
    const unsigned long nowMs = millis();

    ensureWifiConnected(nowMs);
    ensureMqttConnected(nowMs);
    if (consumeMqttConnectedEvent()) {
        g_runtimeState.forceStatusPublish = true;
    }

    mqttLoopTick();
    pollBoards(g_runtimeState, nowMs);
    publishStatusIfNeeded(g_runtimeState, wifiReady(), nowMs, g_runtimeState.forceStatusPublish);
}
