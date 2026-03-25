// Этот файл реализует публикацию минимального MQTT-статуса нового master.
// Он отвечает за строго плоский JSON и отправку retained в topic status.
// Его роль: стабильно отдавать online/offline для плат 12 и 13.

#include "net/status_publisher.h"

#include "core/runtime_state.h"
#include "net/mqtt_manager.h"

namespace {

constexpr unsigned long STATUS_PUBLISH_INTERVAL_MS = 5000;

const char *onlineText(bool online) {
    return online ? "online" : "offline";
}

String buildStatusPayload(const RuntimeState &state, bool wifiIsReady) {
    String payload = "{";
    payload += "\"wifi\":";
    payload += wifiIsReady ? "true" : "false";
    payload += ",\"mqtt\":";
    payload += mqttReady() ? "true" : "false";
    payload += ",\"";
    payload += state.boards[0].key;
    payload += "\":\"";
    payload += onlineText(state.boards[0].online);
    payload += "\",\"";
    payload += state.boards[1].key;
    payload += "\":\"";
    payload += onlineText(state.boards[1].online);
    payload += "\"}";
    return payload;
}

}  // namespace

bool publishStatusIfNeeded(RuntimeState &state, bool wifiIsReady, unsigned long nowMs, bool forceNow) {
    if (!mqttReady()) {
        return false;
    }

    const bool boardChanged = hasUnpublishedBoardChanges(state);
    const bool timeElapsed = (nowMs - state.lastStatusPublishMs) >= STATUS_PUBLISH_INTERVAL_MS;
    if (!forceNow && !boardChanged && !timeElapsed) {
        return false;
    }

    const String payload = buildStatusPayload(state, wifiIsReady);
    if (!publishRetained("status", payload.c_str())) {
        return false;
    }

    state.lastStatusPublishMs = nowMs;
    state.forceStatusPublish = false;
    markBoardsPublished(state);
    return true;
}
