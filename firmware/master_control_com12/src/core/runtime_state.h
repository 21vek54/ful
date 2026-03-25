#pragma once

// Этот файл хранит минимальную runtime-модель нового master.
// Он отвечает за состояния плат 12/13 и флаги публикации MQTT-статуса.
// Его роль: единая точка состояния для main, I2C poller и status publisher.

#include <Arduino.h>

struct BoardState {
    uint8_t id;
    const char *key;
    bool online;
    bool lastPublishedOnline;
};

struct RuntimeState {
    BoardState boards[2];
    bool forceStatusPublish;
    unsigned long lastStatusPublishMs;
};

void initRuntimeState(RuntimeState &state);
void markBoardsPublished(RuntimeState &state);
bool hasUnpublishedBoardChanges(const RuntimeState &state);
