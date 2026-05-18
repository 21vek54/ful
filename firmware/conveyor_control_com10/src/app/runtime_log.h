// Легкий фильтр runtime-логов для COM10.
// Держит разделение по уровням ERROR/EVENT/DEBUG без тяжелой подсистемы.
#pragma once

#include <stdint.h>

#ifndef COM10_LOG_DEBUG_ENABLED
#define COM10_LOG_DEBUG_ENABLED 0
#endif

namespace app::runtime_log {

enum class Level : uint8_t {
    Error = 0,
    Event = 1,
    Debug = 2
};

inline bool isEnabled(Level level)
{
    if (level != Level::Debug) {
        return true;
    }
#if COM10_LOG_DEBUG_ENABLED
    return true;
#else
    return false;
#endif
}

inline bool isDebugEnabled()
{
    return isEnabled(Level::Debug);
}

} // namespace app::runtime_log
