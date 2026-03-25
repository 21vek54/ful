#pragma once

// Этот файл объявляет минимальный MQTT менеджер нового master.
// Он отвечает за подключение, loop-обслуживание и публикации в базовые топики.
// Его роль: инкапсулировать MQTT транспорт и LWT online-маркер.

#include <Arduino.h>

void beginMqtt();
bool mqttReady();
void ensureMqttConnected(unsigned long nowMs);
void mqttLoopTick();

String makeMqttTopic(const char *suffix);
bool publishRetained(const char *suffix, const char *payload);
void publishOnlineMarker();
bool consumeMqttConnectedEvent();
