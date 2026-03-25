#pragma once

// Этот файл объявляет публикацию минимального MQTT-статуса нового master.
// Он отвечает за JSON payload wifi/mqtt/board12/board13 и периодичность отправки.
// Его роль: единый контракт статуса для внешней системы.

#include <Arduino.h>

#include "core/runtime_state.h"

bool publishStatusIfNeeded(RuntimeState &state, bool wifiIsReady, unsigned long nowMs, bool forceNow);
