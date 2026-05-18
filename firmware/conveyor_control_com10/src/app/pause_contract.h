// Контракт pause для COMMON: ответы на PAUSE ARM/STATUS/RELEASE и упаковка снимка в I2C-кадр.
// Используется мастером COM12 для подтверждения pause_epoch и known residual (не только surrogate).

#pragma once

#include <stdint.h>

#include "comm/i2c_bus.h"

namespace app {

enum class PausePrepareState : uint8_t {
    None = 0,
    Requested,
    Preparing,
    Ready,
    BlockedFault
};

enum class PausePrepareWaitReason : uint8_t {
    None = 0,
    ActiveP1SafeBoundary,
    ActivePost7SafeBoundary
};

bool pauseContractHandleCommand(const char *line);
void pauseContractTick();
bool pauseContractIsArmed();
bool pauseContractIsReady();
bool pauseContractIsBlockedFault();
PausePrepareState pauseContractPrepareState();
PausePrepareWaitReason pauseContractPrepareWaitReason();

void pauseContractApplyToI2cFrame(comm::I2cStatusFrame &frame);

} // namespace app
