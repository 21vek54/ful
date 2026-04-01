// Этот файл реализует сервисный мониторинг предельной нагрузки платы конвейера.
// Он отвечает за расчет пауз главного цикла, min heap и reset reason.
// Его роль: дать оператору короткую сводку о том, упирается ли плата в предел.

#include "app/board_monitor.h"

#include <Arduino.h>
#include <esp_system.h>

namespace {

constexpr uint32_t BOARD_MONITOR_INTERVAL_MS = 1000U;

bool g_boardMonitorEnabled = false;
uint32_t g_lastBoardMonitorPrintMs = 0;
uint32_t g_lastLoopTickMs = 0;
uint32_t g_windowMaxLoopPauseMs = 0;

void resetMonitorWindow()
{
    g_windowMaxLoopPauseMs = 0;
}

void noteLoopIteration(uint32_t nowMs)
{
    if (g_lastLoopTickMs != 0) {
        const uint32_t loopPauseMs = nowMs - g_lastLoopTickMs;
        if (loopPauseMs > g_windowMaxLoopPauseMs) {
            g_windowMaxLoopPauseMs = loopPauseMs;
        }
    }
    g_lastLoopTickMs = nowMs;
}

const char *resetReasonText(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON:
            return "poweron";
        case ESP_RST_EXT:
            return "extpin";
        case ESP_RST_SW:
            return "restart";
        case ESP_RST_PANIC:
            return "panic";
        case ESP_RST_INT_WDT:
            return "int_wdt";
        case ESP_RST_TASK_WDT:
            return "task_wdt";
        case ESP_RST_WDT:
            return "wdt";
        case ESP_RST_DEEPSLEEP:
            return "sleep";
        case ESP_RST_BROWNOUT:
            return "brownout";
        case ESP_RST_SDIO:
            return "sdio";
        case ESP_RST_USB:
            return "usb";
        case ESP_RST_JTAG:
            return "jtag";
        case ESP_RST_EFUSE:
            return "efuse";
        case ESP_RST_PWR_GLITCH:
            return "pwr_glitch";
        case ESP_RST_CPU_LOCKUP:
            return "cpu_lockup";
        case ESP_RST_UNKNOWN:
        default:
            return "unknown";
    }
}

void printBoardMonitorSnapshotAt(uint32_t nowMs)
{
    Serial.print("MONITOR: loop_pause_max_ms=");
    Serial.print(g_windowMaxLoopPauseMs);
    Serial.print(" min_heap=");
    Serial.print(ESP.getMinFreeHeap());
    Serial.print(" reset_reason=");
    Serial.println(resetReasonText(esp_reset_reason()));

    g_lastBoardMonitorPrintMs = nowMs;
    resetMonitorWindow();
}

} // namespace

namespace app {

void processBoardMonitor()
{
    const uint32_t nowMs = millis();
    noteLoopIteration(nowMs);

    if (!g_boardMonitorEnabled) {
        return;
    }

    if ((nowMs - g_lastBoardMonitorPrintMs) < BOARD_MONITOR_INTERVAL_MS) {
        return;
    }

    printBoardMonitorSnapshotAt(nowMs);
}

void printBoardMonitorSnapshot()
{
    printBoardMonitorSnapshotAt(millis());
}

bool toggleBoardMonitorStream()
{
    setBoardMonitorStreamEnabled(!g_boardMonitorEnabled);
    return g_boardMonitorEnabled;
}

void setBoardMonitorStreamEnabled(bool enabled)
{
    g_boardMonitorEnabled = enabled;
    g_lastBoardMonitorPrintMs = 0;
    g_lastLoopTickMs = millis();
    resetMonitorWindow();
}

bool isBoardMonitorStreamEnabled()
{
    return g_boardMonitorEnabled;
}

} // namespace app
