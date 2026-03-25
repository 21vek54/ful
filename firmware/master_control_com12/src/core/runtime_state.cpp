// Этот файл инициализирует и обслуживает runtime-состояние нового master.
// Он отвечает за базовые значения online/offline и отметки последней публикации.
// Его роль: дать простые функции работы с состоянием без логики транспорта.

#include "core/runtime_state.h"

#include "device_ids.h"

void initRuntimeState(RuntimeState &state) {
    state.boards[0] = {CONVEYOR_ID, "board12", false, false};
    state.boards[1] = {MANIPULATOR_ID, "board13", false, false};
    state.forceStatusPublish = true;
    state.lastStatusPublishMs = 0;
}

void markBoardsPublished(RuntimeState &state) {
    for (BoardState &board : state.boards) {
        board.lastPublishedOnline = board.online;
    }
}

bool hasUnpublishedBoardChanges(const RuntimeState &state) {
    for (const BoardState &board : state.boards) {
        if (board.online != board.lastPublishedOnline) {
            return true;
        }
    }
    return false;
}
