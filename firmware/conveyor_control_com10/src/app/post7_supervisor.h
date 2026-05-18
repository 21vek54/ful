#pragma once

#include <stdint.h>

// Локальный supervisor для зоны post-step7 на COM10.
// Модуль скрывает внутреннюю оркестрацию SEAL + OUTFEED
// за единым production-контрактом POST7 START.

namespace app::post7 {

enum class Post7Mode : uint8_t {
    None = 0,
    Load,
    UnloadOnly
};

enum class Post7Phase : uint8_t {
    None = 0,
    PreSeal,
    SealInFlight,
    SealCompletedOutfeedPending,
    OutfeedOnly
};

struct Post7Status {
    bool active = false;
    bool expectsSealerCompletion = false;
    bool preSealCancellable = false;
    bool sealCommitReached = false;
    Post7Phase phase = Post7Phase::None;
    bool outfeedReadyForBatch = true;
    bool outfeedActivityObserved = false;
    bool sealerCompletionAdvanced = false;
    bool manualRecoveryRequired = false;
    uint8_t outfeedStep2SeqBase = 0;
    uint8_t outfeedStep2SeqCurrent = 0;
    uint8_t sealerCompletionSeqBase = 0;
    uint8_t sealerCompletionSeqCurrent = 0;
    uint32_t startedMs = 0;
    uint32_t finishedMs = 0;
    Post7Mode mode = Post7Mode::None;
};

void initPost7Supervisor();
void processPost7Supervisor();

bool startPost7Load();
bool startPost7UnloadOnly();
bool stopPost7Job(const char *reason);
bool clearManualRecoveryLatch(const char *reason);
bool isManualRecoveryRequired();

Post7Status readPost7Status();
void printPost7Status();

} // namespace app::post7
