// Этот файл объявляет сервисный мониторинг предельной нагрузки платы конвейера.
// Он отвечает за периодическую печать loop, heap и reset-диагностики.
// Его роль: дать включаемый из CLI поток короткой сервисной телеметрии.

#pragma once

namespace app {

void processBoardMonitor();
void printBoardMonitorSnapshot();
bool toggleBoardMonitorStream();
void setBoardMonitorStreamEnabled(bool enabled);
bool isBoardMonitorStreamEnabled();

} // namespace app
