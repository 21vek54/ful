#pragma once

// Этот файл объявляет минимальный I2C poller нового master.
// Он отвечает за периодический ping плат 12/13 и обновление online/offline.
// Его роль: изолировать I2C-опрос от main и сетевых модулей.

#include <Arduino.h>

#include "core/runtime_state.h"

void beginI2c();
void pollBoards(RuntimeState &state, unsigned long nowMs);
