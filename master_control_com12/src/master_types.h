#pragma once

#include <stdint.h>

enum class MbResult : uint8_t {
    Ok,
    ArgError,
    Timeout,
    ProtocolError,
    CrcError,
    Exception
};
