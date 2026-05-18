#pragma once

#include <stdint.h>

#include "app/conveyor_status.h"

namespace app {

ConveyorStatusInputs readConveyorStatusInputs();
ConveyorStatusSnapshot readConveyorStatusSnapshot();

void conveyorFeedSideNoteProgram1Started(bool startedFromBuffer);
void conveyorFeedSideNoteProgram1PassShiftCompleted(uint8_t passIndex);
void conveyorFeedSideNoteBatchReadyLatched();
void conveyorFeedSideNoteProgram1Finished();
void conveyorFeedSideNoteProgram1Aborted();
void conveyorFeedSideNoteProgram1StoppedByPauseArm();
void conveyorFeedSideNoteIn2Consumed();
bool conveyorProgram1AbortRecoveryRequired();
void clearConveyorProgram1AbortRecoveryRequired(const char *reason);
void invalidateConveyorFeedSideModel(const char *reason);

} // namespace app
