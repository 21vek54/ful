#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>

#include "pins.h"
#include "master_modbus_vfd.h"
#include "master_types.h"
#include "master_utils.h"

#if __has_include("wifi_secrets_local.h")
#include "wifi_secrets_local.h"
#define WIFI_LOCAL_SECRETS_AVAILABLE 1
#else
#define WIFI_LOCAL_SECRETS_AVAILABLE 0
#define WIFI_LOCAL_SSID ""
#define WIFI_LOCAL_PASSWORD ""
#endif

#if __has_include("mqtt_secrets_local.h")
#include "mqtt_secrets_local.h"
#define MQTT_LOCAL_SECRETS_AVAILABLE 1
#else
#define MQTT_LOCAL_SECRETS_AVAILABLE 0
#define MQTT_LOCAL_HOST ""
#define MQTT_LOCAL_PORT 1883
#define MQTT_LOCAL_USER ""
#define MQTT_LOCAL_PASSWORD ""
#define MQTT_LOCAL_BASE_TOPIC "fyl/master_com12"
#endif

namespace {

constexpr char FW_VERSION[] = "v0.7-master-vfd-i2c-seal";
constexpr char DEVICE_NAME[] = "Master controller";

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint32_t WIFI_PROMPT_TIMEOUT_MS = 30000;
constexpr char WIFI_PREF_NAMESPACE[] = "wifi";
constexpr char WIFI_PREF_KEY_SSID[] = "ssid";
constexpr char WIFI_PREF_KEY_PASSWORD[] = "pass";
constexpr char MQTT_PREF_NAMESPACE[] = "mqtt";
constexpr char MQTT_PREF_KEY_HOST[] = "host";
constexpr char MQTT_PREF_KEY_PORT[] = "port";
constexpr char MQTT_PREF_KEY_USER[] = "user";
constexpr char MQTT_PREF_KEY_PASSWORD[] = "pass";
constexpr char MQTT_PREF_KEY_BASE_TOPIC[] = "topic";
constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
constexpr uint32_t MQTT_STATUS_PUBLISH_INTERVAL_MS = 5000;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint8_t I2C_MANAGED_POKE_CMD = 0xA5;
constexpr uint8_t I2C_MANAGED_TEXT_CMD = 0xA6;
constexpr size_t I2C_MANAGED_TEXT_MAX_LEN = 64;

constexpr uint8_t VFD_ID = 11;
constexpr uint8_t CONVEYOR_ID = 12;
constexpr uint8_t MANIPULATOR_ID = 13;
constexpr uint8_t RS485_SCAN_ID_MIN_DEFAULT = VFD_ID;
constexpr uint8_t RS485_SCAN_ID_MAX_DEFAULT = MANIPULATOR_ID;
constexpr uint8_t RS485_SCAN_TABLE_MAX_ID = 32;
constexpr uint16_t RS485_SCAN_PROBE_REG = 0x0000;
constexpr uint16_t RS485_SCAN_PROBE_COUNT = 1;
constexpr uint32_t RS485_SCAN_STEP_INTERVAL_MS = 1000;
constexpr uint32_t RS485_SCAN_STALE_MS = 15000;
constexpr uint16_t DEVICE_REG_MAGIC = 0x0000;
constexpr uint16_t DEVICE_REG_PROTO_VER = 0x0001;
constexpr uint16_t DEVICE_REG_KIND = 0x0002;
constexpr uint16_t DEVICE_REG_ID_ECHO = 0x0003;
constexpr uint16_t DEVICE_REG_STATUS_WORD = 0x0004;
constexpr uint16_t DEVICE_REG_ERROR_WORD = 0x0005;
constexpr uint16_t DEVICE_REG_HEARTBEAT_LO = 0x0006;
constexpr uint16_t DEVICE_REG_HEARTBEAT_HI = 0x0007;
constexpr uint16_t CONVEYOR_REG_FLAG_STATE = 0x0010;
constexpr uint16_t CONVEYOR_REG_MOTION_STATE = 0x0011;
constexpr uint16_t MANIPULATOR_REG_MODE_CODE = 0x0010;
constexpr uint16_t MANIPULATOR_REG_JOB_CODE = 0x0011;
constexpr uint16_t MANIPULATOR_REG_SENSOR_BITS = 0x0012;
constexpr uint16_t MANIPULATOR_REG_CONFLICT_BITS = 0x0013;
constexpr uint16_t DEVICE_MAGIC = 0x4659;
constexpr uint16_t DEVICE_PROTO_VER = 1;
constexpr uint16_t DEVICE_KIND_CONVEYOR = 1;
constexpr uint16_t DEVICE_KIND_MANIPULATOR = 2;
constexpr uint16_t CONVEYOR_STATUS_PROGRAM1_ACTIVE_BIT = 1U << 5;
constexpr uint16_t CONVEYOR_STATUS_BATCH_READY_BIT = 1U << 6;
constexpr uint16_t CONVEYOR_STATUS_STEP2_ACTIVE_BIT = 1U << 7;
constexpr uint16_t MANIPULATOR_STATUS_STEP7_READY_BIT = 1U << 5;
constexpr uint16_t MANIPULATOR_SENSOR_RIGHT_BIT = 1U << 1;
constexpr uint16_t MANIPULATOR_SENSOR_ZUP_BIT = 1U << 2;
constexpr uint16_t MANIPULATOR_SENSOR_GRIP_OPEN_BIT = 1U << 4;

struct __attribute__((packed)) ManagedDeviceI2cFrame {
    uint16_t magic;
    uint16_t protoVer;
    uint16_t deviceKind;
    uint16_t deviceIdEcho;
    uint16_t statusWord;
    uint16_t errorWord;
    uint16_t extra0;
    uint16_t extra1;
    uint16_t extra2;
    uint16_t extra3;
    uint32_t heartbeatMs;
    uint32_t checksum;
};

static_assert(sizeof(ManagedDeviceI2cFrame) == 28, "Unexpected ManagedDeviceI2cFrame size");
constexpr float VFD_CYCLE30_DEFAULT_MIN_HZ = 10.0F;
constexpr float VFD_CYCLE30_DEFAULT_MAX_HZ = 50.0F;
constexpr uint32_t VFD_CYCLE30_TOTAL_MS = 30000;
constexpr uint32_t VFD_CYCLE30_STEP_MS = 1000;
constexpr float VFD_RUN5_DEFAULT_HZ = 5.0F;
constexpr uint32_t VFD_RUN5_DURATION_MS = 5000;
constexpr uint32_t VFD_RUN5MIN_DURATION_MS = 300000;
constexpr uint32_t VFD_RUN5_LOG_STEP_MS = 1000;
constexpr float VFD_RUNCM_DEFAULT_CM = 35.0F;
constexpr float VFD_RUNCM_REF_DISTANCE_CM = 98.3F; // tuned from fact: 35 cm command moved ~40 cm
constexpr float VFD_RUNCM_REF_HZ = 5.0F;
constexpr uint32_t VFD_RUNCM_REF_DURATION_MS = VFD_RUN5_DURATION_MS;
constexpr uint32_t VFD_RUNCM_MIN_DURATION_MS = 200;
constexpr uint32_t VFD_RUNCM_MAX_DURATION_MS = 120000;
constexpr uint32_t OTVOD_WORK_STEP_STEPS = 920;
constexpr uint32_t OTVOD_WORK_STEP_TRIGGER_STEPS = 870;
constexpr uint32_t OTVOD_WORK_STEP_DELAY_US = 1000;
constexpr uint32_t OTVOD_WORK_STEP_COMPLETE_MARGIN_MS = 50;
constexpr float OTVOD_WORK_STEP_START_FACTOR = 1.1F;
constexpr uint32_t OTVOD_WORK_STEP_START_OFFSET_MS = 150;
constexpr uint32_t OTVOD_WORK_STEP_START_MIN_MS = 250;
constexpr uint8_t OTVOD_WORK_TOTAL_CYCLES = 3;
constexpr uint8_t OTVOD_WORK_MIN_CYCLES = 1;
constexpr uint8_t OTVOD_WORK_MAX_CYCLES = 9;
constexpr uint32_t OTVOD_WORK_VFD_SETTLE_MS = 250;
constexpr uint32_t OTVOD_WORK_STEP_SETTLE_POLL_MS = 50;
constexpr uint32_t OTVOD_WORK_VFD_POLL_MS = 100;
constexpr uint32_t OTVOD_WORK_STEP_PHASE_TIMEOUT_MS = 5000;
constexpr uint32_t OTVOD_WORK_VFD_START_TIMEOUT_MS = 2000;
constexpr uint32_t OTVOD_WORK_VFD_TIMEOUT_MARGIN_MS = 4000;
constexpr uint32_t OTVOD_WORK_VFD_TIMEOUT_FALLBACK_MS = 15000;
constexpr uint32_t COMMON_CYCLE_SEAL_SETTLE_MS = 1000;
constexpr bool SEAL_START_ACTIVE_LEVEL = HIGH;
constexpr bool SEAL_DONE_ACTIVE_LEVEL = LOW; // INPUT_PULLUP + relay contact to GND
constexpr uint32_t SEAL_START_PULSE_MS_DEFAULT = 300;
constexpr uint32_t SEAL_START_PULSE_MS_MIN = 50;
constexpr uint32_t SEAL_START_PULSE_MS_MAX = 5000;

struct Rs485DeviceState {
    bool everSeen = false;
    bool online = false;
    bool protocolOk = false;
    uint32_t okCount = 0;
    uint32_t errCount = 0;
    MbResult lastResult = MbResult::ArgError;
    uint8_t lastException = 0;
    uint32_t lastProbeMs = 0;
    uint32_t lastSeenMs = 0;
    uint16_t statusWord = 0;
    uint16_t errorWord = 0;
    uint16_t deviceKind = 0;
    uint16_t deviceIdEcho = 0;
    uint16_t extra0 = 0;
    uint16_t extra1 = 0;
    uint16_t extra2 = 0;
    uint16_t extra3 = 0;
    float vfdRunHz = 0.0F;
    uint16_t vfdFault = 0;
};

struct OtvodWorkCycleState {
    bool active = false;
    bool readyForBatch = true;
    bool stepRunning = false;
    bool stepObservedActive = false;
    bool stepTriggerIssued = false; // legacy field, unused by strict OTCYCLE path
    bool vfdRunning = false;
    bool vfdObservedBusy = false;
    bool vfdStopIssued = false;
    bool vfdTriggerIssued = false; // legacy field, unused by strict OTCYCLE path
    uint8_t totalCycles = OTVOD_WORK_TOTAL_CYCLES;
    uint8_t stepRunsStarted = 0;
    uint8_t stepRunsCompleted = 0;
    uint8_t stepCompletionSeqBase = 0;
    uint8_t vfdRunsStarted = 0;
    uint8_t vfdRunsCompleted = 0;
    uint32_t stepCommandSteps = OTVOD_WORK_STEP_STEPS;
    uint32_t stepStartedMs = 0;
    uint32_t stepCompletedMs = 0;
    uint32_t stepTriggerDelayMs = 0;
    uint32_t stepDoneDelayMs = 0;
    uint32_t vfdStartedMs = 0;
    uint32_t vfdCompletedMs = 0;
    uint32_t vfdTriggerDelayMs = 0;
    uint32_t vfdDoneDelayMs = 0;
    uint32_t lastPollMs = 0;
    String lastEvent = "idle";
};

enum class CommonCycleStage : uint8_t {
    Idle = 0,
    WaitInitialBatch = 1,
    WaitManipReady = 2,
    WaitManipStep7 = 3,
    WaitNextBatch = 4
};

struct CommonCycleState {
    bool active = false;
    bool pauseRequested = false;
    bool parallelLaunchDone = false;
    CommonCycleStage stage = CommonCycleStage::Idle;
    uint16_t lastConsumedBatchSeq = 0;
    uint32_t manipStarts = 0;
    uint32_t parallelStarts = 0;
    uint32_t sealStartedMs = 0;
    String manipStartReason;
    String lastEvent = "idle";
};

String g_rs485RxBuffer;
String g_serialCmdBuffer;
uint32_t g_rsTxCount = 0;
uint32_t g_rsRxCount = 0;
uint8_t g_rs485DiagToken = 0;
Preferences g_wifiPrefs;
Preferences g_mqttPrefs;

WiFiClient g_mqttNetClient;
PubSubClient g_mqttClient(g_mqttNetClient);
String g_mqttHost;
uint16_t g_mqttPort = 1883;
String g_mqttUser;
String g_mqttPassword;
String g_mqttBaseTopic = "fyl/master_com12";
String g_mqttLastCmd = "-";
uint32_t g_mqttCmdSeq = 0;
uint32_t g_mqttLastReconnectMs = 0;
uint32_t g_mqttLastStatusPublishMs = 0;

} // namespace

String g_lastCommandResult = "ожидание";
uint8_t g_modbusSlaveId = VFD_ID;
uint32_t g_rs485Baud = RS485_BAUD;
uint32_t g_rs485SerialConfig = SERIAL_8E1;
uint32_t g_mbReqCount = 0;
uint32_t g_mbOkCount = 0;
uint32_t g_mbErrCount = 0;
uint8_t g_mbLastException = 0;
float g_vfdBaseHz = 50.0F; // P0-14 equivalent used for Hz<->percent conversion.

namespace {

bool g_rs485ScanEnabled = true;
uint8_t g_rs485ScanMinId = RS485_SCAN_ID_MIN_DEFAULT;
uint8_t g_rs485ScanMaxId = RS485_SCAN_ID_MAX_DEFAULT;
uint8_t g_rs485ScanNextId = RS485_SCAN_ID_MIN_DEFAULT;
uint32_t g_rs485ScanLastStepMs = 0;
Rs485DeviceState g_rs485Devices[RS485_SCAN_TABLE_MAX_ID + 1] = {};
bool g_sealStartPulseActive = false;
bool g_sealStartOutputActive = false;
bool g_sealDoneLastActive = false;
uint32_t g_sealDoneLastRiseMs = 0;
uint32_t g_sealStartPulseStartedMs = 0;
uint32_t g_sealStartPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;
OtvodWorkCycleState g_otvodWorkCycle;
CommonCycleState g_commonCycle;

bool mqttPublishStatus(bool retained);
bool i2cSendManagedDeviceCommand(uint8_t id, const String &command);
bool startOtvodWorkCycle(uint8_t totalCycles = OTVOD_WORK_TOTAL_CYCLES,
                         uint32_t stepSteps = OTVOD_WORK_STEP_STEPS,
                         float legacyVfdDistanceCm = 0.0F);

const char *commonCycleStageName(CommonCycleStage stage)
{
    switch (stage) {
        case CommonCycleStage::WaitInitialBatch:
            return "wait_initial_batch";
        case CommonCycleStage::WaitManipReady:
            return "wait_manip_ready";
        case CommonCycleStage::WaitManipStep7:
            return "wait_manip_step7";
        case CommonCycleStage::WaitNextBatch:
            return "wait_next_batch";
        case CommonCycleStage::Idle:
        default:
            return "idle";
    }
}

bool deviceStatusBusy(const Rs485DeviceState &st)
{
    return (st.statusWord & (1U << 1)) != 0;
}

bool conveyorProgram1Active(const Rs485DeviceState &st)
{
    return (st.statusWord & CONVEYOR_STATUS_PROGRAM1_ACTIVE_BIT) != 0;
}

bool conveyorBatchReady(const Rs485DeviceState &st)
{
    return (st.statusWord & CONVEYOR_STATUS_BATCH_READY_BIT) != 0 ||
           (st.extra3 & 0x8000U) != 0;
}

uint8_t conveyorFlagState(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>(st.extra0 & 0x00FFU);
}

uint8_t conveyorMotionState(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.extra0 >> 8) & 0x00FFU);
}

uint32_t conveyorVfdTickDurationMs(const Rs485DeviceState &st)
{
    return static_cast<uint32_t>(st.extra1) * 10U;
}

bool conveyorStep2Active(const Rs485DeviceState &st)
{
    return (st.statusWord & CONVEYOR_STATUS_STEP2_ACTIVE_BIT) != 0;
}

uint8_t conveyorStep2CompletionSeq(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.statusWord >> 8) & 0x00FFU);
}

uint16_t conveyorBatchSeq(const Rs485DeviceState &st)
{
    return static_cast<uint16_t>(st.extra3 & 0x7FFFU);
}

uint8_t conveyorProgramStateCode(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>(st.extra2 & 0x00FFU);
}

uint8_t conveyorProgramPass(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.extra2 >> 8) & 0x00FFU);
}

uint16_t manipulatorSensorBits(const Rs485DeviceState &st)
{
    return static_cast<uint16_t>(st.extra2 & 0x00FFU);
}

uint16_t manipulatorConflictBits(const Rs485DeviceState &st)
{
    return static_cast<uint16_t>(st.extra3 & 0x00FFU);
}

uint8_t manipulatorWorkStep(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.extra3 >> 8) & 0x00FFU);
}

bool manipulatorStep7Ready(const Rs485DeviceState &st)
{
    return (st.statusWord & MANIPULATOR_STATUS_STEP7_READY_BIT) != 0;
}

bool manipulatorInWorkStartPose(const Rs485DeviceState &st)
{
    const uint16_t bits = manipulatorSensorBits(st);
    const uint16_t need = MANIPULATOR_SENSOR_RIGHT_BIT |
                          MANIPULATOR_SENSOR_ZUP_BIT |
                          MANIPULATOR_SENSOR_GRIP_OPEN_BIT;
    return (bits & need) == need;
}

bool manipulatorNeedsOnlyGripOpenForWorkStart(const Rs485DeviceState &st)
{
    const uint16_t bits = manipulatorSensorBits(st);
    const uint16_t posBits = MANIPULATOR_SENSOR_RIGHT_BIT | MANIPULATOR_SENSOR_ZUP_BIT;
    return (bits & posBits) == posBits &&
           (bits & MANIPULATOR_SENSOR_GRIP_OPEN_BIT) == 0;
}

void sealWriteStartOutput(bool active)
{
    digitalWrite(PIN_SEAL_START, active ? SEAL_START_ACTIVE_LEVEL : !SEAL_START_ACTIVE_LEVEL);
    g_sealStartOutputActive = active;
}

bool sealIsDoneActive()
{
    return digitalRead(PIN_SEAL_DONE) == SEAL_DONE_ACTIVE_LEVEL;
}

void printSealStatus()
{
    Serial.print("SEAL: start_out=");
    Serial.print(g_sealStartOutputActive ? "on" : "off");
    Serial.print(", done=");
    Serial.print(sealIsDoneActive() ? "active" : "inactive");
    Serial.print(", pin_start=");
    Serial.print(PIN_SEAL_START);
    Serial.print(", pin_done=");
    Serial.print(PIN_SEAL_DONE);
    Serial.print(", pulse_ms=");
    Serial.println(g_sealStartPulseDurationMs);
}

void processSealIo()
{
    const bool doneActive = sealIsDoneActive();
    if (doneActive != g_sealDoneLastActive) {
        g_sealDoneLastActive = doneActive;
        if (doneActive) {
            g_sealDoneLastRiseMs = millis();
            Serial.println("SEAL: cycle complete input became active.");
            g_lastCommandResult = "SEAL cycle completed";
        } else {
            Serial.println("SEAL: cycle complete input released.");
        }
        (void)mqttPublishStatus(false);
    }

    if (!g_sealStartPulseActive) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_sealStartPulseStartedMs) < g_sealStartPulseDurationMs) {
        return;
    }

    sealWriteStartOutput(false);
    g_sealStartPulseActive = false;
    Serial.println("SEAL: start pulse finished.");
    g_lastCommandResult = "SEAL START pulse finished";
    (void)mqttPublishStatus(false);
}

void handleCommandSeal(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        Serial.println("SEAL commands:");
        Serial.println("  SEAL START [ms]   - pulse START relay on GPIO23 (default 300 ms)");
        Serial.println("  SEAL STATUS       - print START/DONE state");
        Serial.println("  SEAL OUT ON       - force START relay ON for wiring test");
        Serial.println("  SEAL OUT OFF      - force START relay OFF");
        return;
    }

    if (sub == "STATUS" || sub == "STATE") {
        printSealStatus();
        g_lastCommandResult = "SEAL status printed";
        return;
    }

    if (sub == "OUT") {
        String mode = nextToken(args);
        mode.toUpperCase();
        if (mode == "ON") {
            g_sealStartPulseActive = false;
            sealWriteStartOutput(true);
            g_lastCommandResult = "SEAL OUT ON";
            Serial.println(g_lastCommandResult);
            return;
        }
        if (mode == "OFF") {
            g_sealStartPulseActive = false;
            sealWriteStartOutput(false);
            g_lastCommandResult = "SEAL OUT OFF";
            Serial.println(g_lastCommandResult);
            return;
        }
        Serial.println("Usage: SEAL OUT <ON|OFF>");
        g_lastCommandResult = "SEAL OUT failed: bad mode";
        return;
    }

    if (sub == "START" || sub == "RUN") {
        String pulseTok = nextToken(args);
        uint32_t pulseMs = SEAL_START_PULSE_MS_DEFAULT;
        if (!pulseTok.isEmpty()) {
            if (!parseU32(pulseTok, pulseMs) ||
                pulseMs < SEAL_START_PULSE_MS_MIN || pulseMs > SEAL_START_PULSE_MS_MAX) {
                Serial.println("Usage: SEAL START [50..5000]");
                g_lastCommandResult = "SEAL START failed: bad pulse";
                return;
            }
        }

        if (g_sealStartPulseActive) {
            g_lastCommandResult = "SEAL START ignored: pulse already active";
            Serial.println(g_lastCommandResult);
            return;
        }

        g_sealStartPulseDurationMs = pulseMs;
        g_sealStartPulseStartedMs = millis();
        g_sealStartPulseActive = true;
        sealWriteStartOutput(true);

        Serial.print("SEAL START: pulse on GPIO");
        Serial.print(PIN_SEAL_START);
        Serial.print(" for ");
        Serial.print(pulseMs);
        Serial.println(" ms.");
        g_lastCommandResult = "SEAL START pulse sent";
        return;
    }

    Serial.println("Unknown SEAL subcommand. Use: SEAL HELP");
    g_lastCommandResult = "SEAL failed: unknown subcommand";
}

void printCommonCycleStatus()
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    const uint32_t nowMs = millis();
    const uint32_t sealAgeMs = (g_sealDoneLastRiseMs == 0)
        ? 0
        : static_cast<uint32_t>(nowMs - g_sealDoneLastRiseMs);

    Serial.print("COMMON: active=");
    Serial.print(g_commonCycle.active ? "yes" : "no");
    Serial.print(", pause=");
    Serial.print(g_commonCycle.pauseRequested ? "yes" : "no");
    Serial.print(", stage=");
    Serial.print(commonCycleStageName(g_commonCycle.stage));
    Serial.print(", manip_starts=");
    Serial.print(g_commonCycle.manipStarts);
    Serial.print(", parallel_starts=");
    Serial.print(g_commonCycle.parallelStarts);
    Serial.print(", batch_seq=");
    Serial.print(g_commonCycle.lastConsumedBatchSeq);
    Serial.print(", conveyor_ready=");
    Serial.print(conveyorBatchReady(conveyor) ? "yes" : "no");
    Serial.print(", conveyor_seq=");
    Serial.print(conveyorBatchSeq(conveyor));
    Serial.print(", conveyor_p1_state=");
    Serial.print(conveyorProgramStateCode(conveyor));
    Serial.print(", conveyor_p1_pass=");
    Serial.print(conveyorProgramPass(conveyor));
    Serial.print(", manip_busy=");
    Serial.print(deviceStatusBusy(manipulator) ? "yes" : "no");
    Serial.print(", manip_step7=");
    Serial.print(manipulatorStep7Ready(manipulator) ? "yes" : "no");
    Serial.print(", manip_step=");
    Serial.print(manipulatorWorkStep(manipulator));
    Serial.print(", otvod_ready=");
    Serial.print(g_otvodWorkCycle.readyForBatch ? "yes" : "no");
    Serial.print(", seal_age_ms=");
    Serial.print(sealAgeMs);
    Serial.print(", last=");
    Serial.println(g_commonCycle.lastEvent);
}

void abortCommonCycle(const String &reason)
{
    g_commonCycle.active = false;
    g_commonCycle.pauseRequested = false;
    g_commonCycle.parallelLaunchDone = false;
    g_commonCycle.stage = CommonCycleStage::Idle;
    g_commonCycle.lastEvent = reason;
    g_lastCommandResult = reason;
    Serial.println(reason);
    (void)mqttPublishStatus(false);
}

bool sendCommonManipulatorWorkCycleCommand(const String &reason)
{
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    if (!manipulator.online || !manipulator.protocolOk) {
        g_lastCommandResult = "COMMON failed: manipulator offline";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (deviceStatusBusy(manipulator)) {
        g_lastCommandResult = "COMMON failed: manipulator busy";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (!i2cSendManagedDeviceCommand(MANIPULATOR_ID, "R")) {
        g_lastCommandResult = "COMMON failed: MAN R send error";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_commonCycle.stage = CommonCycleStage::WaitManipStep7;
    g_commonCycle.parallelLaunchDone = false;
    g_commonCycle.manipStarts++;
    g_commonCycle.manipStartReason = reason;
    g_commonCycle.lastEvent = "COMMON: manipulator start (" + reason + ")";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
    return true;
}

bool startCommonManipulatorWorkCycle(const String &reason)
{
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    if (!manipulator.online || !manipulator.protocolOk) {
        g_lastCommandResult = "COMMON failed: manipulator offline";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (deviceStatusBusy(manipulator)) {
        g_lastCommandResult = "COMMON failed: manipulator busy";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (manipulatorInWorkStartPose(manipulator)) {
        return sendCommonManipulatorWorkCycleCommand(reason);
    }
    if (manipulatorNeedsOnlyGripOpenForWorkStart(manipulator)) {
        if (!i2cSendManagedDeviceCommand(MANIPULATOR_ID, "Q")) {
            g_lastCommandResult = "COMMON failed: MAN Q send error";
            Serial.println(g_lastCommandResult);
            return false;
        }
        g_commonCycle.stage = CommonCycleStage::WaitManipReady;
        g_commonCycle.parallelLaunchDone = false;
        g_commonCycle.manipStartReason = reason;
        g_commonCycle.lastEvent = "COMMON: manipulator prepare grip open (" + reason + ")";
        g_lastCommandResult = g_commonCycle.lastEvent;
        Serial.println(g_lastCommandResult);
        (void)mqttPublishStatus(false);
        return true;
    }

    g_lastCommandResult =
        "COMMON failed: manipulator not in start pose (need right + Z up + grip open)";
    Serial.println(g_lastCommandResult);
    return false;
}

bool startCommonConveyorProgram1(const String &reason)
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    if (!conveyor.online || !conveyor.protocolOk) {
        g_lastCommandResult = "COMMON failed: conveyor offline";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "1")) {
        g_lastCommandResult = "COMMON failed: CONV 1 send error";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_commonCycle.lastEvent = "COMMON: conveyor P1 start (" + reason + ")";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
    return true;
}

bool startCommonParallelProcesses(uint32_t nowMs)
{
    if (!startCommonConveyorProgram1("parallel after step7")) {
        return false;
    }
    if (!startOtvodWorkCycle()) {
        g_lastCommandResult = "COMMON failed: OTCYCLE start";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (g_sealStartPulseActive) {
        g_lastCommandResult = "COMMON failed: SEAL pulse already active";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_sealStartPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;
    g_sealStartPulseStartedMs = nowMs;
    g_sealStartPulseActive = true;
    sealWriteStartOutput(true);

    g_commonCycle.parallelLaunchDone = true;
    g_commonCycle.parallelStarts++;
    g_commonCycle.sealStartedMs = nowMs;
    g_commonCycle.stage = CommonCycleStage::WaitNextBatch;
    g_commonCycle.lastEvent = "COMMON: step7 reached, started P1 + OTCYCLE + SEAL";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
    return true;
}

bool startCommonCycle()
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];

    if (g_commonCycle.active) {
        if (g_commonCycle.pauseRequested) {
            g_commonCycle.pauseRequested = false;
            g_commonCycle.lastEvent = "COMMON: pause released";
            g_lastCommandResult = g_commonCycle.lastEvent;
            Serial.println(g_lastCommandResult);
            (void)mqttPublishStatus(false);
            return true;
        }
        g_lastCommandResult = "COMMON already active";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_commonCycle = CommonCycleState{};
    g_commonCycle.active = true;
    g_commonCycle.stage = CommonCycleStage::WaitInitialBatch;
    g_commonCycle.lastEvent = "COMMON: waiting initial batch";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);

    if (conveyorBatchReady(conveyor)) {
        g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
        if (!startCommonManipulatorWorkCycle("initial batch ready")) {
            abortCommonCycle("COMMON failed: initial manipulator start");
            return false;
        }
        return true;
    }

    if (!deviceStatusBusy(conveyor) && !conveyorProgram1Active(conveyor)) {
        if (!startCommonConveyorProgram1("initial fill")) {
            abortCommonCycle("COMMON failed: initial P1 start");
            return false;
        }
    } else {
        (void)mqttPublishStatus(false);
    }
    return true;
}

void requestCommonCyclePause()
{
    if (!g_commonCycle.active) {
        g_lastCommandResult = "COMMON is not active";
        Serial.println(g_lastCommandResult);
        return;
    }

    g_commonCycle.pauseRequested = true;
    g_commonCycle.lastEvent = "COMMON: pause requested";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
}

void processCommonCycle()
{
    if (!g_commonCycle.active) {
        return;
    }

    const uint32_t nowMs = millis();
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];

    switch (g_commonCycle.stage) {
        case CommonCycleStage::WaitInitialBatch:
            if (!conveyor.online || !conveyor.protocolOk || !manipulator.online || !manipulator.protocolOk) {
                return;
            }
            if (!conveyorBatchReady(conveyor)) {
                if (!g_commonCycle.pauseRequested &&
                    !deviceStatusBusy(conveyor) &&
                    !conveyorProgram1Active(conveyor) &&
                    !startCommonConveyorProgram1("retry initial fill")) {
                    abortCommonCycle("COMMON failed: retry P1 start");
                }
                return;
            }
            if (g_commonCycle.pauseRequested) {
                return;
            }
            g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
            if (!startCommonManipulatorWorkCycle("initial batch ready")) {
                abortCommonCycle("COMMON failed: manipulator start");
            }
            return;

        case CommonCycleStage::WaitManipReady:
            if (!manipulator.online || !manipulator.protocolOk) {
                return;
            }
            if (deviceStatusBusy(manipulator)) {
                return;
            }
            if (!manipulatorInWorkStartPose(manipulator)) {
                return;
            }
            if (!sendCommonManipulatorWorkCycleCommand(
                    g_commonCycle.manipStartReason.isEmpty() ? String("prepared") : g_commonCycle.manipStartReason)) {
                abortCommonCycle("COMMON failed: manipulator start after prepare");
            }
            return;

        case CommonCycleStage::WaitManipStep7:
            if (!manipulator.online || !manipulator.protocolOk) {
                return;
            }
            if (!g_commonCycle.parallelLaunchDone &&
                deviceStatusBusy(manipulator) &&
                manipulatorStep7Ready(manipulator)) {
                if (!startCommonParallelProcesses(nowMs)) {
                    abortCommonCycle("COMMON failed: parallel start after step7");
                }
            }
            return;

        case CommonCycleStage::WaitNextBatch: {
            const bool nextBatchReady = conveyorBatchReady(conveyor) &&
                conveyorBatchSeq(conveyor) != 0 &&
                conveyorBatchSeq(conveyor) != g_commonCycle.lastConsumedBatchSeq;
            const bool manipulatorIdle = manipulator.online &&
                manipulator.protocolOk &&
                !deviceStatusBusy(manipulator);
            const bool sealReady = g_sealDoneLastRiseMs > g_commonCycle.sealStartedMs &&
                static_cast<uint32_t>(nowMs - g_sealDoneLastRiseMs) >= COMMON_CYCLE_SEAL_SETTLE_MS;
            const bool otvodReady = g_otvodWorkCycle.readyForBatch;

            if (!(nextBatchReady && manipulatorIdle && sealReady && otvodReady)) {
                return;
            }

            if (g_commonCycle.pauseRequested) {
                if (g_commonCycle.lastEvent != "COMMON: paused at safe point") {
                    g_commonCycle.lastEvent = "COMMON: paused at safe point";
                    g_lastCommandResult = g_commonCycle.lastEvent;
                    Serial.println(g_lastCommandResult);
                    (void)mqttPublishStatus(false);
                }
                return;
            }

            g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
            if (!startCommonManipulatorWorkCycle("next batch ready")) {
                abortCommonCycle("COMMON failed: next manipulator start");
            }
            return;
        }

        case CommonCycleStage::Idle:
        default:
            return;
    }
}

void rs485DiagPrintFrame(const char *prefix, const uint8_t *frame)
{
    Serial.print(prefix);
    printBytesHex(frame, RS485_DIAG_FRAME_LEN);
}

bool rs485DiagValidateFrame(const uint8_t *frame)
{
    if (frame == nullptr) {
        return false;
    }
    if (frame[0] != RS485_DIAG_MAGIC_0 || frame[1] != RS485_DIAG_MAGIC_1) {
        return false;
    }
    return checkFrameCrc(frame, RS485_DIAG_FRAME_LEN);
}

bool rs485HandleDiagFrame(const uint8_t *frame, bool passive)
{
    if (!rs485DiagValidateFrame(frame)) {
        return false;
    }

    const uint8_t cmd = frame[4];
    if (cmd == RS485_DIAG_CMD_PING && frame[3] == RS485_DIAG_SELF_ID) {
        uint8_t resp[RS485_DIAG_FRAME_LEN] = {};
        memcpy(resp, frame, sizeof(resp));
        resp[4] = RS485_DIAG_CMD_PONG;
        appendCrc(resp, 6);

        rs485SetTransmitMode();
        Serial2.write(resp, sizeof(resp));
        Serial2.flush();
        delayMicroseconds(TX_SETTLE_US);
        rs485SetReceiveMode();
        g_rsTxCount++;

        if (!passive) {
            rs485DiagPrintFrame("RSPING TX: ", resp);
        }
        return true;
    }

    return cmd == RS485_DIAG_CMD_PONG;
}

void rs485SendTextLine(const String &text)
{
    String line = text;
    line.trim();
    if (line.isEmpty()) {
        Serial.println("RS485: empty text ignored.");
        return;
    }

    rs485SetTransmitMode();
    Serial2.println(line);
    Serial2.flush();
    delayMicroseconds(TX_SETTLE_US);
    rs485SetReceiveMode();
    g_rsTxCount++;

    Serial.print("RS485 TX: ");
    Serial.println(line);
}

bool handleManagedDeviceConsoleCommand(uint8_t id, const char *label, String args)
{
    String command = args;
    command.trim();
    if (command.isEmpty()) {
        Serial.print("Usage: ");
        Serial.print(label);
        Serial.println(" <command>");
        g_lastCommandResult = String(label) + " command failed: empty";
        return false;
    }

    if (!i2cSendManagedDeviceCommand(id, command)) {
        Serial.print(label);
        Serial.println(": send failed.");
        g_lastCommandResult = String(label) + " command failed: send";
        return false;
    }

    Serial.print(label);
    Serial.print(": ");
    Serial.println(command);
    g_lastCommandResult = String(label) + " command sent";
    return true;
}

void handleCommandRsPing(String args)
{
    uint16_t dstId = 0;
    if (!parseU16(args, dstId) || dstId == 0 || dstId > 255) {
        Serial.println("Usage: RSPING <id>");
        return;
    }

    const uint8_t token = ++g_rs485DiagToken;
    uint8_t req[RS485_DIAG_FRAME_LEN] = {
        RS485_DIAG_MAGIC_0,
        RS485_DIAG_MAGIC_1,
        RS485_DIAG_SELF_ID,
        static_cast<uint8_t>(dstId),
        RS485_DIAG_CMD_PING,
        token,
        0,
        0
    };
    appendCrc(req, 6);

    rs485DrainRx();
    rs485SetTransmitMode();
    Serial2.write(req, sizeof(req));
    Serial2.flush();
    delayMicroseconds(TX_SETTLE_US);
    rs485SetReceiveMode();
    g_rsTxCount++;
    rs485DiagPrintFrame("RSPING TX: ", req);

    uint8_t resp[RS485_DIAG_FRAME_LEN] = {};
    if (!rs485ReadExact(resp, sizeof(resp), RS485_DIAG_TIMEOUT_MS)) {
        Serial.println("RSPING: timeout.");
        return;
    }

    g_rsRxCount++;
    rs485DiagPrintFrame("RSPING RX: ", resp);
    if (!rs485DiagValidateFrame(resp)) {
        Serial.println("RSPING: invalid frame.");
        return;
    }
    if (resp[2] != RS485_DIAG_SELF_ID || resp[3] != static_cast<uint8_t>(dstId) ||
        resp[4] != RS485_DIAG_CMD_PONG || resp[5] != token) {
        Serial.println("RSPING: unexpected reply.");
        return;
    }

    Serial.print("RSPING: OK from ");
    Serial.println(dstId);
}


bool isMbAliveResult(MbResult result)
{
    return (result == MbResult::Ok || result == MbResult::Exception);
}

uint32_t checksumManagedDeviceFrame(const ManagedDeviceI2cFrame &frame)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
    uint32_t sum = 0x13572468UL;
    for (size_t i = 0; i < sizeof(ManagedDeviceI2cFrame) - sizeof(frame.checksum); i++) {
        sum = (sum << 5) | (sum >> 27);
        sum ^= bytes[i];
    }
    return sum;
}

bool i2cPokeManagedDevice(uint8_t id)
{
    Wire.beginTransmission(id);
    Wire.write(I2C_MANAGED_POKE_CMD);
    return Wire.endTransmission(true) == 0;
}

bool i2cSendManagedDeviceCommand(uint8_t id, const String &command)
{
    String text = command;
    text.trim();
    if (text.isEmpty() || text.length() > I2C_MANAGED_TEXT_MAX_LEN) {
        return false;
    }

    Wire.beginTransmission(id);
    Wire.write(I2C_MANAGED_TEXT_CMD);
    Wire.write(reinterpret_cast<const uint8_t *>(text.c_str()), text.length());
    return Wire.endTransmission(true) == 0;
}

MbResult i2cReadManagedDeviceFrame(uint8_t id, ManagedDeviceI2cFrame &frame)
{
    memset(&frame, 0, sizeof(frame));

    if (!i2cPokeManagedDevice(id)) {
        return MbResult::Timeout;
    }

    const size_t want = sizeof(frame);
    const size_t got = Wire.requestFrom(static_cast<int>(id), static_cast<int>(want), true);
    if (got != want) {
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        return got == 0 ? MbResult::Timeout : MbResult::ProtocolError;
    }

    uint8_t *dst = reinterpret_cast<uint8_t *>(&frame);
    for (size_t i = 0; i < want; i++) {
        if (Wire.available() <= 0) {
            return MbResult::ProtocolError;
        }
        dst[i] = static_cast<uint8_t>(Wire.read());
    }

    return MbResult::Ok;
}

void rs485MarkDeviceOffline(Rs485DeviceState &st)
{
    st.online = false;
    st.protocolOk = false;
    st.deviceKind = 0;
    st.deviceIdEcho = 0;
    st.statusWord = 0;
    st.errorWord = 0;
    st.extra0 = 0;
    st.extra1 = 0;
    st.extra2 = 0;
    st.extra3 = 0;
    st.vfdRunHz = 0.0F;
    st.vfdFault = 0;
}

void rs485MarkDeviceAlive(Rs485DeviceState &st, MbResult result, uint8_t exceptionCode)
{
    const uint32_t now = millis();
    st.online = true;
    st.everSeen = true;
    st.lastResult = result;
    st.lastException = exceptionCode;
    st.okCount++;
    st.lastSeenMs = now;
}

void rs485ProbeVfd(uint8_t id)
{
    Rs485DeviceState &st = g_rs485Devices[id];
    const uint32_t now = millis();
    st.lastProbeMs = now;

    uint16_t statusWord = 0;
    uint8_t exceptionCode = 0;
    const MbResult r = modbusReadHoldingFromSlave(id, VFD_REG_STATUS, 1, &statusWord, &exceptionCode, false);
    st.lastResult = r;
    st.lastException = exceptionCode;

    if (!isMbAliveResult(r)) {
        st.errCount++;
        rs485MarkDeviceOffline(st);
        return;
    }

    rs485MarkDeviceAlive(st, r, exceptionCode);
    st.protocolOk = (r == MbResult::Ok);
    st.deviceKind = 0;
    st.deviceIdEcho = id;
    if (r != MbResult::Ok) {
        return;
    }

    st.statusWord = statusWord;

    uint16_t runReg = 0;
    if (modbusReadHoldingFromSlave(id, VFD_REG_FREQ_RUN, 1, &runReg, &exceptionCode, false) == MbResult::Ok) {
        st.vfdRunHz = static_cast<float>(static_cast<int16_t>(runReg)) / VFD_RUN_FREQ_SCALE;
    }

    uint16_t faultReg = 0;
    if (modbusReadHoldingFromSlave(id, VFD_REG_FAULT, 1, &faultReg, &exceptionCode, false) == MbResult::Ok) {
        st.vfdFault = faultReg;
    }
}

void rs485ProbeManagedDevice(uint8_t id, uint16_t expectedKind)
{
    Rs485DeviceState &st = g_rs485Devices[id];
    const uint32_t now = millis();
    st.lastProbeMs = now;

    ManagedDeviceI2cFrame frame = {};
    const MbResult r = i2cReadManagedDeviceFrame(id, frame);
    st.lastResult = r;
    st.lastException = 0;

    if (!isMbAliveResult(r)) {
        st.errCount++;
        if (!st.everSeen || (uint32_t)(now - st.lastSeenMs) > RS485_SCAN_STALE_MS) {
            rs485MarkDeviceOffline(st);
        }
        return;
    }

    rs485MarkDeviceAlive(st, r, 0);
    st.deviceKind = frame.deviceKind;
    st.deviceIdEcho = frame.deviceIdEcho;
    st.statusWord = frame.statusWord;
    st.errorWord = frame.errorWord;
    st.extra0 = frame.extra0;
    st.extra1 = frame.extra1;
    st.extra2 = frame.extra2;
    st.extra3 = frame.extra3;
    st.protocolOk = (frame.magic == DEVICE_MAGIC &&
                     frame.protoVer == DEVICE_PROTO_VER &&
                     frame.deviceKind == expectedKind &&
                     frame.deviceIdEcho == id &&
                     checksumManagedDeviceFrame(frame) == frame.checksum);
    if (!st.protocolOk) {
        st.lastResult = MbResult::ProtocolError;
        return;
    }
}

} // namespace

void rs485ScanResetAll()
{
    for (uint8_t id = 1; id <= RS485_SCAN_TABLE_MAX_ID; id++) {
        g_rs485Devices[id] = Rs485DeviceState();
    }
    g_rs485ScanNextId = g_rs485ScanMinId;
}

namespace {

uint16_t rs485OnlineCount()
{
    uint16_t count = 0;
    for (uint8_t id = g_rs485ScanMinId; id <= g_rs485ScanMaxId; id++) {
        if (g_rs485Devices[id].online) {
            count++;
        }
    }
    return count;
}

String rs485OnlineIdsCsv()
{
    String out;
    bool first = true;
    for (uint8_t id = g_rs485ScanMinId; id <= g_rs485ScanMaxId; id++) {
        if (!g_rs485Devices[id].online) {
            continue;
        }
        if (!first) {
            out += ",";
        }
        out += String(id);
        first = false;
    }
    return out;
}

void rs485ScanProbeId(uint8_t id)
{
    if (id == 0 || id > RS485_SCAN_TABLE_MAX_ID) {
        return;
    }

    if (id == VFD_ID) {
        rs485ProbeVfd(id);
        return;
    }
    if (id == CONVEYOR_ID) {
        rs485ProbeManagedDevice(id, DEVICE_KIND_CONVEYOR);
        return;
    }
    if (id == MANIPULATOR_ID) {
        rs485ProbeManagedDevice(id, DEVICE_KIND_MANIPULATOR);
        return;
    }

    uint16_t probeReg = 0;
    uint8_t exceptionCode = 0;
    const MbResult r = modbusReadHoldingFromSlave(
        id,
        RS485_SCAN_PROBE_REG,
        RS485_SCAN_PROBE_COUNT,
        &probeReg,
        &exceptionCode,
        false);

    Rs485DeviceState &st = g_rs485Devices[id];
    const uint32_t now = millis();
    st.lastProbeMs = now;
    st.lastResult = r;
    st.lastException = exceptionCode;

    if (isMbAliveResult(r)) {
        rs485MarkDeviceAlive(st, r, exceptionCode);
        return;
    }

    st.errCount++;
    if (!st.everSeen || (uint32_t)(now - st.lastSeenMs) > RS485_SCAN_STALE_MS) {
        rs485MarkDeviceOffline(st);
    }
}

void rs485ScanLoop()
{
    if (!g_rs485ScanEnabled) {
        return;
    }

    if (g_rs485ScanMinId == 0 ||
        g_rs485ScanMaxId > RS485_SCAN_TABLE_MAX_ID ||
        g_rs485ScanMinId > g_rs485ScanMaxId) {
        return;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - g_rs485ScanLastStepMs) < RS485_SCAN_STEP_INTERVAL_MS) {
        return;
    }
    g_rs485ScanLastStepMs = now;

    for (uint8_t id = g_rs485ScanMinId; id <= g_rs485ScanMaxId; id++) {
        rs485ScanProbeId(id);
    }

    for (uint8_t id = g_rs485ScanMinId; id <= g_rs485ScanMaxId; id++) {
        Rs485DeviceState &st = g_rs485Devices[id];
        if (id == VFD_ID && st.online && (uint32_t)(now - st.lastSeenMs) > RS485_SCAN_STALE_MS) {
            st.online = false;
        }
    }
}

void rs485ScanPrintStatus()
{
    Serial.print("MBSCAN: ");
    Serial.print(g_rs485ScanEnabled ? "on" : "off");
    Serial.print(", range=");
    Serial.print(g_rs485ScanMinId);
    Serial.print("..");
    Serial.print(g_rs485ScanMaxId);
    Serial.print(", online=");
    Serial.println(rs485OnlineCount());

    const String onlineIds = rs485OnlineIdsCsv();
    Serial.print("MBSCAN online IDs: ");
    Serial.println(onlineIds.isEmpty() ? "<none>" : onlineIds);

    const Rs485DeviceState &vfd = g_rs485Devices[VFD_ID];
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    Serial.print("VFD(11): online=");
    Serial.print(vfd.online ? "yes" : "no");
    Serial.print(", status=");
    Serial.print(vfd.statusWord);
    Serial.print(", run_hz=");
    Serial.print(vfd.vfdRunHz, 2);
    Serial.print(", fault=");
    Serial.println(vfd.vfdFault);

    Serial.print("Conveyor(12): online=");
    Serial.print(conveyor.online ? "yes" : "no");
    Serial.print(", kind_ok=");
    Serial.print(conveyor.protocolOk ? "yes" : "no");
    Serial.print(", status=");
    Serial.print(conveyor.statusWord);
    Serial.print(", error=");
    Serial.print(conveyor.errorWord);
    Serial.print(", flag=");
    Serial.print(conveyorFlagState(conveyor));
    Serial.print(", motion=");
    Serial.print(conveyorMotionState(conveyor));
    Serial.print(", vfd_tick_ms=");
    Serial.println(conveyorVfdTickDurationMs(conveyor));

    Serial.print("Manipulator(13): online=");
    Serial.print(manipulator.online ? "yes" : "no");
    Serial.print(", kind_ok=");
    Serial.print(manipulator.protocolOk ? "yes" : "no");
    Serial.print(", status=");
    Serial.print(manipulator.statusWord);
    Serial.print(", error=");
    Serial.print(manipulator.errorWord);
    Serial.print(", mode=");
    Serial.print(manipulator.extra0);
    Serial.print(", job=");
    Serial.println(manipulator.extra1);
}

bool handleConsoleLine(String line);

bool serialReadLineWithTimeout(String &line, uint32_t timeoutMs)
{
    line = "";
    const uint32_t startMs = millis();

    while ((uint32_t)(millis() - startMs) < timeoutMs) {
        while (Serial.available() > 0) {
            const char ch = static_cast<char>(Serial.read());
            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                line.trim();
                return true;
            }
            line += ch;
        }
        delay(5);
    }

    normalizeCommandWhitespace(line);
    return !line.isEmpty();
}

bool wifiLoadCredentials(String &ssid, String &password)
{
    ssid = "";
    password = "";
    if (!g_wifiPrefs.begin(WIFI_PREF_NAMESPACE, true)) {
        return false;
    }
    ssid = g_wifiPrefs.getString(WIFI_PREF_KEY_SSID, "");
    password = g_wifiPrefs.getString(WIFI_PREF_KEY_PASSWORD, "");
    g_wifiPrefs.end();
    ssid.trim();
    password.trim();
    return !ssid.isEmpty();
}

bool wifiSaveCredentials(const String &ssid, const String &password)
{
    String s = ssid;
    String p = password;
    s.trim();
    p.trim();
    if (s.isEmpty()) {
        return false;
    }

    if (!g_wifiPrefs.begin(WIFI_PREF_NAMESPACE, false)) {
        return false;
    }
    g_wifiPrefs.putString(WIFI_PREF_KEY_SSID, s);
    g_wifiPrefs.putString(WIFI_PREF_KEY_PASSWORD, p);
    g_wifiPrefs.end();
    return true;
}

void wifiClearCredentials()
{
    if (!g_wifiPrefs.begin(WIFI_PREF_NAMESPACE, false)) {
        return;
    }
    g_wifiPrefs.remove(WIFI_PREF_KEY_SSID);
    g_wifiPrefs.remove(WIFI_PREF_KEY_PASSWORD);
    g_wifiPrefs.end();
}

bool wifiTryConnect(const String &ssid, const String &password, const char *sourceLabel)
{
    String s = ssid;
    s.trim();
    if (s.isEmpty()) {
        return false;
    }

    Serial.print("WIFI: connect via ");
    Serial.print(sourceLabel);
    Serial.print(" -> ");
    Serial.println(s);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    WiFi.begin(s.c_str(), password.c_str());

    const uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if ((uint32_t)(millis() - startMs) >= WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("WIFI: timeout.");
            return false;
        }
        delay(250);
    }

    Serial.print("WIFI: connected, IP=");
    Serial.println(WiFi.localIP());
    return true;
}

void wifiPrintStatus()
{
    String savedSsid;
    String savedPass;
    const bool hasSaved = wifiLoadCredentials(savedSsid, savedPass);

    Serial.print("WIFI: ");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("connected, ssid='");
        Serial.print(WiFi.SSID());
        Serial.print("', ip=");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("disconnected.");
    }

    Serial.print("WIFI: saved creds: ");
    Serial.println(hasSaved ? "yes" : "no");
    Serial.print("WIFI: local file creds: ");
    Serial.println((WIFI_LOCAL_SECRETS_AVAILABLE && String(WIFI_LOCAL_SSID).length() > 0) ? "yes" : "no");
}

bool wifiPromptAndConnect(uint32_t timeoutMs)
{
    Serial.println("WIFI: enter SSID and press Enter (empty to skip).");
    String ssid;
    if (!serialReadLineWithTimeout(ssid, timeoutMs)) {
        Serial.println("WIFI: prompt timeout, skipped.");
        return false;
    }
    ssid.trim();
    if (ssid.isEmpty()) {
        Serial.println("WIFI: skipped.");
        return false;
    }

    Serial.println("WIFI: enter password and press Enter.");
    String password;
    if (!serialReadLineWithTimeout(password, timeoutMs)) {
        Serial.println("WIFI: password timeout.");
        return false;
    }

    if (!wifiTryConnect(ssid, password, "serial prompt")) {
        Serial.println("WIFI: connection failed for entered credentials.");
        return false;
    }

    if (wifiSaveCredentials(ssid, password)) {
        Serial.println("WIFI: credentials saved to NVS.");
    } else {
        Serial.println("WIFI: failed to save credentials.");
    }
    return true;
}

bool wifiTryAutoConnect(bool allowPrompt)
{
    bool connected = false;

#if WIFI_LOCAL_SECRETS_AVAILABLE
    const String localSsid = String(WIFI_LOCAL_SSID);
    const String localPassword = String(WIFI_LOCAL_PASSWORD);
    if (localSsid.length() > 0) {
        connected = wifiTryConnect(localSsid, localPassword, "local file");
    }
#endif

    if (!connected) {
        String savedSsid;
        String savedPassword;
        if (wifiLoadCredentials(savedSsid, savedPassword)) {
            connected = wifiTryConnect(savedSsid, savedPassword, "saved NVS");
        }
    }

    if (!connected && allowPrompt) {
        connected = wifiPromptAndConnect(WIFI_PROMPT_TIMEOUT_MS);
    }

    return connected;
}

void handleCommandWifi(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "HELP" || sub == "H") {
        Serial.println("WIFI commands:");
        Serial.println("  WIFI STATUS");
        Serial.println("  WIFI CONNECT              - try local file then saved NVS");
        Serial.println("  WIFI CONNECT <ssid> <pass>");
        Serial.println("  WIFI SET <ssid> <pass>    - save to NVS and connect");
        Serial.println("  WIFI PROMPT               - ask ssid/pass in serial monitor");
        Serial.println("  WIFI FORGET               - clear saved ssid/pass from NVS");
        return;
    }

    if (sub == "STATUS") {
        wifiPrintStatus();
        return;
    }

    if (sub == "PROMPT") {
        if (!wifiPromptAndConnect(WIFI_PROMPT_TIMEOUT_MS)) {
            Serial.println("WIFI: prompt/connect failed.");
        }
        return;
    }

    if (sub == "FORGET") {
        wifiClearCredentials();
        Serial.println("WIFI: saved credentials removed.");
        return;
    }

    if (sub == "CONNECT") {
        String ssid = nextToken(args);
        if (ssid.isEmpty()) {
            if (!wifiTryAutoConnect(false)) {
                Serial.println("WIFI: connect failed (no valid local/saved creds).");
            }
            return;
        }

        String pass = args;
        pass.trim();
        if (pass.isEmpty()) {
            Serial.println("Usage: WIFI CONNECT <ssid> <pass>");
            return;
        }
        if (!wifiTryConnect(ssid, pass, "console")) {
            Serial.println("WIFI: connect failed.");
        }
        return;
    }

    if (sub == "SET") {
        String ssid = nextToken(args);
        String pass = args;
        pass.trim();
        if (ssid.isEmpty() || pass.isEmpty()) {
            Serial.println("Usage: WIFI SET <ssid> <pass>");
            return;
        }

        if (!wifiSaveCredentials(ssid, pass)) {
            Serial.println("WIFI: failed to save credentials.");
            return;
        }
        Serial.println("WIFI: credentials saved.");
        if (!wifiTryConnect(ssid, pass, "saved from console")) {
            Serial.println("WIFI: saved, but connect failed.");
        }
        return;
    }

    Serial.println("Unknown WIFI subcommand. Use: WIFI HELP");
}

bool mqttLoadSettings()
{
    String host;
    uint16_t port = 1883;
    String user;
    String pass;
    String topic;
    bool hasSaved = false;

    if (g_mqttPrefs.begin(MQTT_PREF_NAMESPACE, true)) {
        host = g_mqttPrefs.getString(MQTT_PREF_KEY_HOST, "");
        port = static_cast<uint16_t>(g_mqttPrefs.getUShort(MQTT_PREF_KEY_PORT, 1883));
        user = g_mqttPrefs.getString(MQTT_PREF_KEY_USER, "");
        pass = g_mqttPrefs.getString(MQTT_PREF_KEY_PASSWORD, "");
        topic = g_mqttPrefs.getString(MQTT_PREF_KEY_BASE_TOPIC, "");
        g_mqttPrefs.end();
        hasSaved = !host.isEmpty();
    }

    if (!hasSaved && MQTT_LOCAL_SECRETS_AVAILABLE) {
        host = String(MQTT_LOCAL_HOST);
        port = static_cast<uint16_t>(MQTT_LOCAL_PORT);
        user = String(MQTT_LOCAL_USER);
        pass = String(MQTT_LOCAL_PASSWORD);
        topic = String(MQTT_LOCAL_BASE_TOPIC);
    }

    host.trim();
    user.trim();
    pass.trim();
    topic.trim();

    g_mqttHost = host;
    g_mqttPort = (port == 0) ? 1883 : port;
    g_mqttUser = user;
    g_mqttPassword = pass;
    if (!topic.isEmpty()) {
        g_mqttBaseTopic = topic;
    }

    return !g_mqttHost.isEmpty();
}

bool mqttSaveSettings()
{
    if (!g_mqttPrefs.begin(MQTT_PREF_NAMESPACE, false)) {
        return false;
    }

    g_mqttPrefs.putString(MQTT_PREF_KEY_HOST, g_mqttHost);
    g_mqttPrefs.putUShort(MQTT_PREF_KEY_PORT, g_mqttPort);
    g_mqttPrefs.putString(MQTT_PREF_KEY_USER, g_mqttUser);
    g_mqttPrefs.putString(MQTT_PREF_KEY_PASSWORD, g_mqttPassword);
    g_mqttPrefs.putString(MQTT_PREF_KEY_BASE_TOPIC, g_mqttBaseTopic);
    g_mqttPrefs.end();
    return true;
}

void mqttForgetSettings()
{
    if (!g_mqttPrefs.begin(MQTT_PREF_NAMESPACE, false)) {
        return;
    }
    g_mqttPrefs.remove(MQTT_PREF_KEY_HOST);
    g_mqttPrefs.remove(MQTT_PREF_KEY_PORT);
    g_mqttPrefs.remove(MQTT_PREF_KEY_USER);
    g_mqttPrefs.remove(MQTT_PREF_KEY_PASSWORD);
    g_mqttPrefs.remove(MQTT_PREF_KEY_BASE_TOPIC);
    g_mqttPrefs.end();
}

String mqttTopicCmd()
{
    return g_mqttBaseTopic + "/cmd";
}

String mqttTopicStatus()
{
    return g_mqttBaseTopic + "/status";
}

String mqttTopicResp()
{
    return g_mqttBaseTopic + "/resp";
}

String escapeJsonString(const String &value)
{
    String out = value;
    out.replace("\\", "\\\\");
    out.replace("\"", "\\\"");
    out.replace("\r", " ");
    out.replace("\n", " ");
    return out;
}

String mqttBuildStatusPayload()
{
    String payload = "{";
    payload += "\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
    payload += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
    payload += ",\"modbus_id\":" + String(g_modbusSlaveId);
    payload += ",\"mb_req\":" + String(g_mbReqCount);
    payload += ",\"mb_ok\":" + String(g_mbOkCount);
    payload += ",\"mb_err\":" + String(g_mbErrCount);
    payload += ",\"vfd_base_hz\":" + String(g_vfdBaseHz, 2);
    payload += ",\"rs485_scan_en\":" + String(g_rs485ScanEnabled ? "true" : "false");
    payload += ",\"rs485_scan_min\":" + String(g_rs485ScanMinId);
    payload += ",\"rs485_scan_max\":" + String(g_rs485ScanMaxId);
    payload += ",\"rs485_online\":" + String(rs485OnlineCount());
    payload += ",\"rs485_online_ids\":\"" + escapeJsonString(rs485OnlineIdsCsv()) + "\"";
    payload += ",\"seal\":{";
    payload += "\"start_pin\":" + String(PIN_SEAL_START);
    payload += ",\"done_pin\":" + String(PIN_SEAL_DONE);
    payload += ",\"start_out\":" + String(g_sealStartOutputActive ? "true" : "false");
    payload += ",\"done\":" + String(sealIsDoneActive() ? "true" : "false");
    payload += ",\"pulse_ms\":" + String(g_sealStartPulseDurationMs);
    payload += "}";
    payload += ",\"otvod_cycle\":{";
    payload += "\"active\":" + String(g_otvodWorkCycle.active ? "true" : "false");
    payload += ",\"ready\":" + String(g_otvodWorkCycle.readyForBatch ? "true" : "false");
    payload += ",\"step_started\":" + String(g_otvodWorkCycle.stepRunsStarted);
    payload += ",\"step_done\":" + String(g_otvodWorkCycle.stepRunsCompleted);
    payload += ",\"vfd_started\":" + String(g_otvodWorkCycle.vfdRunsStarted);
    payload += ",\"vfd_done\":" + String(g_otvodWorkCycle.vfdRunsCompleted);
    payload += ",\"last\":\"" + escapeJsonString(g_otvodWorkCycle.lastEvent) + "\"";
    payload += "}";
    payload += ",\"rs485_devices\":[";
    bool first = true;
    for (uint8_t id = g_rs485ScanMinId; id <= g_rs485ScanMaxId; id++) {
        const Rs485DeviceState &st = g_rs485Devices[id];
        if (!first) {
            payload += ",";
        }
        payload += "{";
        payload += "\"id\":" + String(id);
        payload += ",\"on\":" + String(st.online ? "true" : "false");
        payload += ",\"ok\":" + String(st.okCount);
        payload += ",\"err\":" + String(st.errCount);
        payload += ",\"last\":\"" + String(mbResultCode(st.lastResult)) + "\"";
        payload += ",\"ex\":" + String(st.lastException);
        payload += "}";
        first = false;
    }
    payload += "]";
    const Rs485DeviceState &vfd = g_rs485Devices[VFD_ID];
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    payload += ",\"devices\":{";
    payload += "\"vfd\":{";
    payload += "\"id\":" + String(VFD_ID);
    payload += ",\"online\":" + String(vfd.online ? "true" : "false");
    payload += ",\"status_word\":" + String(vfd.statusWord);
    payload += ",\"run_hz\":" + String(vfd.vfdRunHz, 2);
    payload += ",\"fault\":" + String(vfd.vfdFault);
    payload += "},";
    payload += "\"conveyor\":{";
    payload += "\"id\":" + String(CONVEYOR_ID);
    payload += ",\"online\":" + String(conveyor.online ? "true" : "false");
    payload += ",\"kind_ok\":" + String(conveyor.protocolOk ? "true" : "false");
    payload += ",\"status_word\":" + String(conveyor.statusWord);
    payload += ",\"error_word\":" + String(conveyor.errorWord);
    payload += ",\"flag_state\":" + String(conveyorFlagState(conveyor));
    payload += ",\"motion_state\":" + String(conveyorMotionState(conveyor));
    payload += ",\"program_state\":" + String(conveyorProgramStateCode(conveyor));
    payload += ",\"program_pass\":" + String(conveyorProgramPass(conveyor));
    payload += ",\"batch_ready\":" + String(conveyorBatchReady(conveyor) ? "true" : "false");
    payload += ",\"batch_seq\":" + String(conveyorBatchSeq(conveyor));
    payload += ",\"vfd_tick_ms\":" + String(conveyorVfdTickDurationMs(conveyor));
    payload += "},";
    payload += "\"manipulator\":{";
    payload += "\"id\":" + String(MANIPULATOR_ID);
    payload += ",\"online\":" + String(manipulator.online ? "true" : "false");
    payload += ",\"kind_ok\":" + String(manipulator.protocolOk ? "true" : "false");
    payload += ",\"status_word\":" + String(manipulator.statusWord);
    payload += ",\"error_word\":" + String(manipulator.errorWord);
    payload += ",\"mode\":" + String(manipulator.extra0);
    payload += ",\"job\":" + String(manipulator.extra1);
    payload += ",\"sensor_bits\":" + String(manipulatorSensorBits(manipulator));
    payload += ",\"conflict_bits\":" + String(manipulatorConflictBits(manipulator));
    payload += ",\"work_step\":" + String(manipulatorWorkStep(manipulator));
    payload += ",\"step7_ready\":" + String(manipulatorStep7Ready(manipulator) ? "true" : "false");
    payload += "}";
    payload += "}";
    payload += ",\"common_cycle\":{";
    payload += "\"active\":" + String(g_commonCycle.active ? "true" : "false");
    payload += ",\"pause\":" + String(g_commonCycle.pauseRequested ? "true" : "false");
    payload += ",\"stage\":\"" + String(commonCycleStageName(g_commonCycle.stage)) + "\"";
    payload += ",\"manip_starts\":" + String(g_commonCycle.manipStarts);
    payload += ",\"parallel_starts\":" + String(g_commonCycle.parallelStarts);
    payload += ",\"batch_seq\":" + String(g_commonCycle.lastConsumedBatchSeq);
    payload += ",\"last\":\"" + escapeJsonString(g_commonCycle.lastEvent) + "\"";
    payload += "}";
    payload += ",\"cmd_seq\":" + String(g_mqttCmdSeq);
    payload += ",\"last_cmd\":\"" + escapeJsonString(g_mqttLastCmd) + "\"";
    payload += "}";
    return payload;
}

bool mqttPublishStatus(bool retained = false)
{
    if (!g_mqttClient.connected()) {
        return false;
    }
    const String topic = mqttTopicStatus();
    const String payload = mqttBuildStatusPayload();
    return g_mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

String mqttBuildCommandResponsePayload(const String &cmd, bool accepted, const String &result)
{
    String payload = "{";
    payload += "\"seq\":" + String(g_mqttCmdSeq);
    payload += ",\"cmd\":\"" + escapeJsonString(cmd) + "\"";
    payload += ",\"accepted\":" + String(accepted ? "true" : "false");
    payload += ",\"result\":\"" + escapeJsonString(result) + "\"";
    payload += "}";
    return payload;
}

bool mqttPublishCommandResponse(const String &cmd, bool accepted, const String &result, bool retained = false)
{
    if (!g_mqttClient.connected()) {
        return false;
    }

    const String topic = mqttTopicResp();
    const String payload = mqttBuildCommandResponsePayload(cmd, accepted, result);
    return g_mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{
    String msg;
    msg.reserve(length + 1);
    for (unsigned int i = 0; i < length; i++) {
        msg += static_cast<char>(payload[i]);
    }
    msg.trim();

    Serial.print("MQTT RX [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(msg);

    if (msg.isEmpty()) {
        return;
    }

    String upper = msg;
    upper.toUpperCase();
    g_mqttLastCmd = msg;
    g_mqttCmdSeq++;

    if (upper == "STATUS") {
        g_lastCommandResult = "СЃС‚Р°С‚СѓСЃ РѕРїСѓР±Р»РёРєРѕРІР°РЅ";
        (void)mqttPublishStatus(false);
        (void)mqttPublishCommandResponse(msg, true, g_lastCommandResult, false);
        return;
    }

    const bool accepted = handleConsoleLine(msg);
    const String result = accepted ? g_lastCommandResult :
        (g_lastCommandResult.isEmpty() ? String("РЅРµРёР·РІРµСЃС‚РЅР°СЏ РєРѕРјР°РЅРґР°") : g_lastCommandResult);
    (void)mqttPublishCommandResponse(msg, accepted, result, false);
    (void)mqttPublishStatus(false);
}

void mqttConfigureClient()
{
    g_mqttClient.setServer(g_mqttHost.c_str(), g_mqttPort);
    g_mqttClient.setCallback(mqttCallback);
    g_mqttClient.setKeepAlive(30);
    g_mqttClient.setSocketTimeout(5);
    (void)g_mqttClient.setBufferSize(2048);
}

bool mqttConnectNow()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("MQTT: wifi disconnected.");
        return false;
    }
    if (g_mqttHost.isEmpty()) {
        Serial.println("MQTT: host is empty. Use MQTT SET <host> <port>.");
        return false;
    }

    mqttConfigureClient();
    const uint64_t mac = ESP.getEfuseMac();
    const String clientId = "master_" + String(static_cast<uint32_t>(mac & 0xFFFFFFFFULL), HEX);

    Serial.print("MQTT: connecting to ");
    Serial.print(g_mqttHost);
    Serial.print(":");
    Serial.println(g_mqttPort);

    bool ok = false;
    if (g_mqttUser.isEmpty()) {
        ok = g_mqttClient.connect(clientId.c_str());
    } else {
        ok = g_mqttClient.connect(clientId.c_str(), g_mqttUser.c_str(), g_mqttPassword.c_str());
    }

    if (!ok) {
        Serial.print("MQTT: connect failed, rc=");
        Serial.println(g_mqttClient.state());
        return false;
    }

    const String cmdTopic = mqttTopicCmd();
    g_mqttClient.subscribe(cmdTopic.c_str(), 1);
    (void)mqttPublishStatus(true);
    Serial.print("MQTT: connected. Subscribed to ");
    Serial.println(cmdTopic);
    Serial.print("MQTT: responses on ");
    Serial.println(mqttTopicResp());
    return true;
}

void mqttDisconnectNow()
{
    if (g_mqttClient.connected()) {
        g_mqttClient.disconnect();
    }
}

void mqttLoop()
{
    if (g_mqttClient.connected()) {
        g_mqttClient.loop();
        if ((uint32_t)(millis() - g_mqttLastStatusPublishMs) >= MQTT_STATUS_PUBLISH_INTERVAL_MS) {
            g_mqttLastStatusPublishMs = millis();
            (void)mqttPublishStatus(false);
        }
        return;
    }

    if (WiFi.status() != WL_CONNECTED || g_mqttHost.isEmpty()) {
        return;
    }

    if ((uint32_t)(millis() - g_mqttLastReconnectMs) < MQTT_RECONNECT_INTERVAL_MS) {
        return;
    }
    g_mqttLastReconnectMs = millis();
    (void)mqttConnectNow();
}

void mqttPrintStatus()
{
    Serial.print("MQTT: host=");
    Serial.print(g_mqttHost.isEmpty() ? "<empty>" : g_mqttHost);
    Serial.print(", port=");
    Serial.print(g_mqttPort);
    Serial.print(", topic=");
    Serial.print(g_mqttBaseTopic);
    Serial.print(", connected=");
    Serial.println(g_mqttClient.connected() ? "yes" : "no");
}

void handleCommandMqtt(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "HELP" || sub == "H") {
        Serial.println("MQTT commands:");
        Serial.println("  MQTT STATUS");
        Serial.println("  MQTT SET <host> <port> [user] [pass]");
        Serial.println("  MQTT TOPIC <base/topic>");
        Serial.println("  MQTT CONNECT");
        Serial.println("  MQTT DISCONNECT");
        Serial.println("  MQTT PUB <subtopic> <payload>");
        Serial.println("  MQTT FORGET");
        return;
    }

    if (sub == "STATUS") {
        mqttPrintStatus();
        return;
    }

    if (sub == "CONNECT") {
        (void)mqttConnectNow();
        return;
    }

    if (sub == "DISCONNECT") {
        mqttDisconnectNow();
        Serial.println("MQTT: disconnected.");
        return;
    }

    if (sub == "TOPIC") {
        String topic = args;
        topic.trim();
        if (topic.isEmpty()) {
            Serial.println("Usage: MQTT TOPIC <base/topic>");
            return;
        }
        g_mqttBaseTopic = topic;
        if (!mqttSaveSettings()) {
            Serial.println("MQTT: failed to save settings.");
        }
        Serial.print("MQTT: topic set to ");
        Serial.println(g_mqttBaseTopic);
        return;
    }

    if (sub == "SET") {
        String host = nextToken(args);
        String portTok = nextToken(args);
        if (host.isEmpty() || portTok.isEmpty()) {
            Serial.println("Usage: MQTT SET <host> <port> [user] [pass]");
            return;
        }
        uint16_t port = 0;
        if (!parseU16(portTok, port) || port == 0) {
            Serial.println("MQTT: invalid port.");
            return;
        }

        String user = nextToken(args);
        String pass = args;
        pass.trim();

        g_mqttHost = host;
        g_mqttPort = port;
        g_mqttUser = user;
        g_mqttPassword = user.isEmpty() ? "" : pass;

        if (!mqttSaveSettings()) {
            Serial.println("MQTT: failed to save settings.");
            return;
        }

        mqttDisconnectNow();
        Serial.println("MQTT: settings saved.");
        return;
    }

    if (sub == "PUB") {
        String subtopic = nextToken(args);
        String payload = args;
        payload.trim();
        if (subtopic.isEmpty() || payload.isEmpty()) {
            Serial.println("Usage: MQTT PUB <subtopic> <payload>");
            return;
        }
        if (!g_mqttClient.connected()) {
            Serial.println("MQTT: not connected.");
            return;
        }

        const String topic = g_mqttBaseTopic + "/" + subtopic;
        if (g_mqttClient.publish(topic.c_str(), payload.c_str(), false)) {
            Serial.print("MQTT: published to ");
            Serial.println(topic);
        } else {
            Serial.println("MQTT: publish failed.");
        }
        return;
    }

    if (sub == "FORGET") {
        mqttForgetSettings();
        g_mqttHost = "";
        g_mqttPort = 1883;
        g_mqttUser = "";
        g_mqttPassword = "";
        mqttDisconnectNow();
        Serial.println("MQTT: saved settings removed.");
        return;
    }

    Serial.println("Unknown MQTT subcommand. Use: MQTT HELP");
}

void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  UP / W            - conveyor flag up via I2C");
    Serial.println("  DOWN / S          - conveyor flag down via I2C");
    Serial.println("  CONV <cmd>        - send one-line command to conveyor (12) via I2C");
    Serial.println("  MAN <cmd>         - send one-line command to manipulator (13) via I2C");
    Serial.println("  SEAL <...>        - sealer START/DONE I/O (SEAL HELP)");
        Serial.println("  OTCYCLE <...>     - 3-cycle divert program for OTVOD/main tick");
    Serial.println("  COMMON <...>      - common production cycle with soft pause");
    Serial.println("  RS <text>         - send raw text line to RS485");
    Serial.println("  RSPING <id>       - send raw RS485 ping and wait for pong");
    Serial.println("  MBID <id>         - set VFD Modbus slave id (1..247)");
    Serial.println("  MBUART [baud] [fmt] - get/set RS485 UART, fmt=8E1|8N1|8N2");
    Serial.println("  MBR <reg> [cnt]   - read VFD holding regs (func 03)");
    Serial.println("  MBRAW <reg> [cnt] - raw Modbus hex dump using current id/UART");
    Serial.println("  MBW <reg> <val>   - write VFD single reg (func 06)");
    Serial.println("  MBSCAN <...>      - scan VFD via RS485 and 12/13 via I2C");
    Serial.println("  VFD <...>         - VFD quick commands (VFD HELP)");
    Serial.println("  WIFI <...>        - WiFi setup/status (WIFI HELP)");
    Serial.println("  MQTT <...>        - MQTT setup/status (MQTT HELP)");
    Serial.println("  STATE             - print RS485/I2C + Modbus counters");
    Serial.println("  H                 - help");
}

void printState()
{
    Serial.print("RS485 state: tx=");
    Serial.print(g_rsTxCount);
    Serial.print(", rx=");
    Serial.println(g_rsRxCount);
    printRs485Uart();

    Serial.print("MODBUS state: id=");
    Serial.print(g_modbusSlaveId);
    Serial.print(", req=");
    Serial.print(g_mbReqCount);
    Serial.print(", ok=");
    Serial.print(g_mbOkCount);
    Serial.print(", err=");
    Serial.println(g_mbErrCount);

    Serial.print("MBSCAN state: ");
    Serial.print(g_rs485ScanEnabled ? "on" : "off");
    Serial.print(", range=");
    Serial.print(g_rs485ScanMinId);
    Serial.print("..");
    Serial.print(g_rs485ScanMaxId);
    Serial.print(", online=");
    Serial.print(rs485OnlineCount());
    Serial.print(", ids=");
    const String onlineIds = rs485OnlineIdsCsv();
    Serial.println(onlineIds.isEmpty() ? "<none>" : onlineIds);

    const Rs485DeviceState &vfd = g_rs485Devices[VFD_ID];
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    Serial.print("VFD(11): online=");
    Serial.print(vfd.online ? "yes" : "no");
    Serial.print(", status=");
    Serial.print(vfd.statusWord);
    Serial.print(", run_hz=");
    Serial.print(vfd.vfdRunHz, 2);
    Serial.print(", fault=");
    Serial.println(vfd.vfdFault);

    Serial.print("Conveyor(12): online=");
    Serial.print(conveyor.online ? "yes" : "no");
    Serial.print(", kind_ok=");
    Serial.print(conveyor.protocolOk ? "yes" : "no");
    Serial.print(", status=");
    Serial.print(conveyor.statusWord);
    Serial.print(", error=");
    Serial.print(conveyor.errorWord);
    Serial.print(", flag=");
    Serial.print(conveyorFlagState(conveyor));
    Serial.print(", motion=");
    Serial.print(conveyorMotionState(conveyor));
    Serial.print(", vfd_tick_ms=");
    Serial.println(conveyorVfdTickDurationMs(conveyor));

    Serial.print("Manipulator(13): online=");
    Serial.print(manipulator.online ? "yes" : "no");
    Serial.print(", kind_ok=");
    Serial.print(manipulator.protocolOk ? "yes" : "no");
    Serial.print(", status=");
    Serial.print(manipulator.statusWord);
    Serial.print(", error=");
    Serial.print(manipulator.errorWord);
    Serial.print(", mode=");
    Serial.print(manipulator.extra0);
    Serial.print(", job=");
    Serial.println(manipulator.extra1);

    printSealStatus();

    Serial.print("WIFI state: ");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("connected, ip=");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("disconnected");
    }

    Serial.print("MQTT state: ");
    Serial.print(g_mqttClient.connected() ? "connected" : "disconnected");
    Serial.print(", host=");
    Serial.print(g_mqttHost.isEmpty() ? "<empty>" : g_mqttHost);
    Serial.print(", port=");
    Serial.println(g_mqttPort);
}

void handleCommandMbId(String args)
{
    String tok = nextToken(args);
    uint16_t id = 0;
    if (!parseU16(tok, id) || id == 0 || id > 247) {
        Serial.println("Usage: MBID <1..247>");
        return;
    }

    g_modbusSlaveId = static_cast<uint8_t>(id);
    Serial.print("MODBUS: slave id set to ");
    Serial.println(g_modbusSlaveId);
}

void handleCommandMbUart(String args)
{
    String baudTok = nextToken(args);
    String fmtTok = nextToken(args);

    if (baudTok.isEmpty()) {
        printRs485Uart();
        return;
    }

    uint32_t baud = 0;
    if (!parseU32(baudTok, baud) || baud < 300 || baud > 115200) {
        Serial.println("Usage: MBUART <300..115200> <8E1|8N1|8N2>");
        return;
    }

    uint32_t serialConfig = g_rs485SerialConfig;
    if (fmtTok.isEmpty() || !parseRs485SerialConfigToken(fmtTok, serialConfig)) {
        Serial.println("Usage: MBUART <300..115200> <8E1|8N1|8N2>");
        return;
    }

    rs485ApplyUart(baud, serialConfig);
    rs485ScanResetAll();
    Serial.print("MBUART applied: ");
    Serial.print(g_rs485Baud);
    Serial.print(" ");
    Serial.println(rs485SerialConfigName(g_rs485SerialConfig));
}

void handleCommandMbRead(String args)
{
    String regTok = nextToken(args);
    String cntTok = nextToken(args);

    uint16_t reg = 0;
    if (!parseU16(regTok, reg)) {
        Serial.println("Usage: MBR <reg> [count]");
        return;
    }

    uint16_t count = 1;
    if (!cntTok.isEmpty() && !parseU16(cntTok, count)) {
        Serial.println("Usage: MBR <reg> [count]");
        return;
    }

    if (count == 0 || count > MODBUS_MAX_READ_REGS) {
        Serial.print("Count must be 1..");
        Serial.println(MODBUS_MAX_READ_REGS);
        return;
    }

    uint16_t regs[MODBUS_MAX_READ_REGS] = {};
    const MbResult result = modbusReadHolding(reg, count, regs);
    printMbResult(result);
    if (result != MbResult::Ok) {
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        Serial.print("R[");
        Serial.print(reg + i);
        Serial.print("] = ");
        Serial.print(regs[i]);
        Serial.print(" (0x");
        Serial.print(regs[i], HEX);
        Serial.println(")");
    }
}

void handleCommandMbWrite(String args)
{
    String regTok = nextToken(args);
    String valTok = nextToken(args);

    uint16_t reg = 0;
    uint16_t value = 0;
    if (!parseU16(regTok, reg) || !parseU16(valTok, value)) {
        Serial.println("Usage: MBW <reg> <value>");
        return;
    }

    const MbResult result = modbusWriteSingle(reg, value);
    printMbResult(result);
}

void handleCommandMbScan(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        Serial.println("MBSCAN commands:");
        Serial.println("  MBSCAN STATUS");
        Serial.println("  MBSCAN ON");
        Serial.println("  MBSCAN OFF");
        Serial.println("  MBSCAN RANGE <min_id> <max_id>");
        Serial.println("  MBSCAN NOW [id]");
        return;
    }

    if (sub == "STATUS") {
        rs485ScanPrintStatus();
        return;
    }

    if (sub == "ON") {
        g_rs485ScanEnabled = true;
        Serial.println("MBSCAN: enabled.");
        rs485ScanPrintStatus();
        return;
    }

    if (sub == "OFF") {
        g_rs485ScanEnabled = false;
        Serial.println("MBSCAN: disabled.");
        rs485ScanPrintStatus();
        return;
    }

    if (sub == "RANGE") {
        String minTok = nextToken(args);
        String maxTok = nextToken(args);
        uint16_t minId = 0;
        uint16_t maxId = 0;
        if (!parseU16(minTok, minId) || !parseU16(maxTok, maxId)) {
            Serial.println("Usage: MBSCAN RANGE <min_id> <max_id>");
            return;
        }
        if (minId == 0 || maxId == 0 || minId > maxId || maxId > RS485_SCAN_TABLE_MAX_ID) {
            Serial.print("MBSCAN: valid range is 1..");
            Serial.println(RS485_SCAN_TABLE_MAX_ID);
            return;
        }

        g_rs485ScanMinId = static_cast<uint8_t>(minId);
        g_rs485ScanMaxId = static_cast<uint8_t>(maxId);
        g_rs485ScanNextId = g_rs485ScanMinId;
        rs485ScanResetAll();
        Serial.println("MBSCAN: range updated.");
        rs485ScanPrintStatus();
        return;
    }

    if (sub == "NOW") {
        String idTok = nextToken(args);
        uint16_t id = g_rs485ScanNextId;
        if (!idTok.isEmpty() && !parseU16(idTok, id)) {
            Serial.println("Usage: MBSCAN NOW [id]");
            return;
        }
        if (id == 0 || id > RS485_SCAN_TABLE_MAX_ID) {
            Serial.print("MBSCAN: id must be 1..");
            Serial.println(RS485_SCAN_TABLE_MAX_ID);
            return;
        }

        rs485ScanProbeId(static_cast<uint8_t>(id));
        const Rs485DeviceState &st = g_rs485Devices[id];
        Serial.print("MBSCAN NOW: id=");
        Serial.print(id);
        Serial.print(", online=");
        Serial.print(st.online ? "yes" : "no");
        Serial.print(", last=");
        Serial.print(mbResultCode(st.lastResult));
        if (st.lastResult == MbResult::Exception) {
            Serial.print(", ex=0x");
            Serial.print(st.lastException, HEX);
        }
        Serial.println();
        return;
    }

    Serial.println("Unknown MBSCAN subcommand. Use: MBSCAN HELP");
}

void printVfdUsage()
{
    Serial.println("VFD commands:");
    Serial.println("  VFD SETUPRS [id]        - configure VR70 for master RS485 (9600 8E1, Modbus, P0-04=2, P0-06=7)");
    Serial.println("  VFD CFG                 - read key VR70 configuration registers");
    Serial.println("  VFD START [hz] [dir]    - set Hz and run continuously, dir=FWD|REV");
    Serial.println("  VFD FWD                 - run forward (0x2000=1)");
    Serial.println("  VFD REV                 - run reverse (0x2000=2)");
    Serial.println("  VFD STOP                - decel stop (0x2000=6)");
    Serial.println("  VFD RESET               - fault reset (0x2000=7)");
    Serial.println("  VFD STAT                - read status (0x3000)");
    Serial.println("  VFD FAULT               - read fault code (0x8000)");
    Serial.println("  VFD FREQ <hz>           - set setpoint in Hz (via P0-11)");
    Serial.println("  VFD FREQRAW <value>     - set raw percent -10000..10000 (legacy 0x1000)");
    Serial.println("  VFD MAXHZ [hz]          - set/get base Hz used for VFD FREQ");
    Serial.println("  VFD RUNFREQ             - read run freq raw (0x1002)");
    Serial.println("  VFD FIND                - probe common RS485 UART configs for slave 11");
    Serial.println("  VFD RUN5 [hz] [dir]     - run for 5s at hz, dir=FWD|REV (default 5 Hz, FWD)");
    Serial.println("  VFD RUN5MIN [hz] [dir]  - run for 5 min at hz, dir=FWD|REV");
    Serial.println("  VFD RUN35 [hz] [dir]    - calibrated move by 35 cm");
    Serial.println("  VFD RUNCM <cm> [hz] [dir] - calibrated move, ref=98.3 cm in 5s at 5 Hz");
    Serial.println("  VFD CYCLE30 [min] [max] - 30s ramp + actual Hz monitor + STOP");
}

bool parseVfdDirectionToken(const String &token, uint16_t &cmdOut, const char *&labelOut)
{
    String dir = token;
    dir.trim();
    dir.toUpperCase();

    if (dir.isEmpty() || dir == "FWD" || dir == "RUN" || dir == "FORWARD") {
        cmdOut = VFD_CMD_FWD;
        labelOut = "FWD";
        return true;
    }

    if (dir == "REV" || dir == "REVERSE" || dir == "BACK") {
        cmdOut = VFD_CMD_REV;
        labelOut = "REV";
        return true;
    }

    return false;
}

bool computeVfdRunDurationForDistanceCm(float distanceCm, float hz, uint32_t &durationMsOut)
{
    if (!(distanceCm > 0.0F) || !(hz > 0.0F) ||
        !(VFD_RUNCM_REF_DISTANCE_CM > 0.0F) || !(VFD_RUNCM_REF_HZ > 0.0F)) {
        return false;
    }

    const float durationMs = (distanceCm / VFD_RUNCM_REF_DISTANCE_CM) *
        static_cast<float>(VFD_RUNCM_REF_DURATION_MS) * (VFD_RUNCM_REF_HZ / hz);
    if (!(durationMs >= static_cast<float>(VFD_RUNCM_MIN_DURATION_MS)) ||
        durationMs > static_cast<float>(VFD_RUNCM_MAX_DURATION_MS)) {
        return false;
    }

    durationMsOut = static_cast<uint32_t>(lroundf(durationMs));
    return true;
}

uint32_t computeStepperMoveDurationMs(uint32_t steps, uint32_t delayUs, uint32_t extraMs = 0)
{
    if (delayUs == 0U) {
        delayUs = 1U;
    }

    const uint64_t totalUs = static_cast<uint64_t>(steps) * static_cast<uint64_t>(delayUs);
    return static_cast<uint32_t>((totalUs + 999ULL) / 1000ULL) + extraMs;
}

uint32_t computeOtvodStepToVfdDelayMs(uint32_t steps)
{
    const float derivedMs = static_cast<float>(steps) * OTVOD_WORK_STEP_START_FACTOR +
        static_cast<float>(OTVOD_WORK_STEP_START_OFFSET_MS);
    const uint32_t totalDelayMs = static_cast<uint32_t>(lroundf(derivedMs));
    return max(OTVOD_WORK_STEP_START_MIN_MS, totalDelayMs);
}

bool pollOtvodConveyorState()
{
    rs485ProbeManagedDevice(CONVEYOR_ID, DEVICE_KIND_CONVEYOR);
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    return conveyor.online && conveyor.protocolOk;
}

bool pollOtvodStep2State(bool &step2ActiveOut)
{
    if (!pollOtvodConveyorState()) {
        return false;
    }

    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    step2ActiveOut = conveyorStep2Active(conveyor);
    return true;
}

bool pollOtvodVfdCycleState(bool &busyOut, uint32_t &tickDurationMsOut)
{
    if (!pollOtvodConveyorState()) {
        return false;
    }

    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    busyOut = deviceStatusBusy(conveyor);
    tickDurationMsOut = conveyorVfdTickDurationMs(conveyor);
    return true;
}

void printOtvodWorkCycleStatus()
{
    uint32_t tickDurationMs = g_otvodWorkCycle.vfdDoneDelayMs;
    if (!g_otvodWorkCycle.active && tickDurationMs == 0) {
        bool conveyorBusy = false;
        uint32_t polledTickMs = 0;
        if (pollOtvodVfdCycleState(conveyorBusy, polledTickMs) && polledTickMs > 0) {
            tickDurationMs = polledTickMs;
        }
    }

    Serial.print("OTCYCLE: active=");
    Serial.print(g_otvodWorkCycle.active ? "yes" : "no");
    Serial.print(", ready=");
    Serial.print(g_otvodWorkCycle.readyForBatch ? "yes" : "no");
    Serial.print(", step=");
    Serial.print(g_otvodWorkCycle.stepRunsCompleted);
    Serial.print("/");
    Serial.print(g_otvodWorkCycle.stepRunsStarted);
    Serial.print("/");
    Serial.print(g_otvodWorkCycle.totalCycles);
    Serial.print(", vfd=");
    Serial.print(g_otvodWorkCycle.vfdRunsCompleted);
    Serial.print("/");
    Serial.print(g_otvodWorkCycle.vfdRunsStarted);
    Serial.print(", steps=");
    Serial.print(g_otvodWorkCycle.stepCommandSteps);
    Serial.print(", tick_ms=");
    Serial.print(tickDurationMs);
    Serial.print(", last=");
    Serial.println(g_otvodWorkCycle.lastEvent);
}

bool sendOtvodWorkStepCommand(uint8_t cycleIndex, uint32_t startedMs)
{
    const String command = "OTVOD " + String(g_otvodWorkCycle.stepCommandSteps);
    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, command)) {
        Serial.println("OTCYCLE: failed to send OTVOD command to conveyor.");
        return false;
    }

    g_otvodWorkCycle.stepRunning = true;
    g_otvodWorkCycle.stepObservedActive = false;
    g_otvodWorkCycle.stepRunsStarted = cycleIndex;
    g_otvodWorkCycle.stepCompletionSeqBase = conveyorStep2CompletionSeq(g_rs485Devices[CONVEYOR_ID]);
    g_otvodWorkCycle.stepStartedMs = startedMs;
    g_otvodWorkCycle.stepCompletedMs = 0;
    g_otvodWorkCycle.lastPollMs = 0;
    g_otvodWorkCycle.readyForBatch = false;
    g_otvodWorkCycle.lastEvent = "2-СЂСѓС‡РµР№РєРѕРІС‹Р№ СЃС‚Р°СЂС‚ " + String(cycleIndex) + "/" + String(g_otvodWorkCycle.totalCycles);

    Serial.print("OTCYCLE: ");
    Serial.print(g_otvodWorkCycle.lastEvent);
    Serial.print(", OTVOD ");
    Serial.print(g_otvodWorkCycle.stepCommandSteps);
    Serial.println(" С€Р°РіРѕРІ.");
    (void)mqttPublishStatus(false);
    return true;
}

bool startOtvodWorkVfdRun(uint8_t cycleIndex)
{
    uint32_t tickDurationMs = conveyorVfdTickDurationMs(g_rs485Devices[CONVEYOR_ID]);
    if (tickDurationMs == 0) {
        bool conveyorBusy = false;
        if (!pollOtvodVfdCycleState(conveyorBusy, tickDurationMs)) {
            Serial.println("OTCYCLE: failed to read conveyor VFDTICK state.");
            return false;
        }
    }

    if (tickDurationMs == 0) {
        Serial.println("OTCYCLE: conveyor VFDTICK duration is not configured.");
        return false;
    }

    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "VFDTICK RUN")) {
        Serial.println("OTCYCLE: failed to send VFDTICK RUN to conveyor.");
        return false;
    }

    const uint32_t actualStartMs = millis();
    g_otvodWorkCycle.vfdRunning = true;
    g_otvodWorkCycle.vfdObservedBusy = false;
    g_otvodWorkCycle.vfdStopIssued = false;
    g_otvodWorkCycle.vfdRunsStarted = cycleIndex;
    g_otvodWorkCycle.vfdStartedMs = actualStartMs;
    g_otvodWorkCycle.vfdCompletedMs = 0;
    g_otvodWorkCycle.vfdDoneDelayMs = tickDurationMs;
    g_otvodWorkCycle.lastPollMs = 0;
    g_otvodWorkCycle.lastEvent = "OTVOD main start " + String(cycleIndex) + "/" + String(g_otvodWorkCycle.totalCycles);

    Serial.print("OTCYCLE: ");
    Serial.print(g_otvodWorkCycle.lastEvent);
    Serial.print(", 1 tick, ");
    Serial.print(tickDurationMs);
    Serial.println(" ms.");
    (void)mqttPublishStatus(false);
    return true;
}

void stopOtvodWorkCycleOutputs()
{
    if (g_otvodWorkCycle.vfdRunning) {
        (void)i2cSendManagedDeviceCommand(CONVEYOR_ID, "VFDSTOP");
    }
    (void)i2cSendManagedDeviceCommand(CONVEYOR_ID, "STEP2STOP");
}

void finishOtvodWorkCycle(const String &result)
{
    g_otvodWorkCycle.active = false;
    g_otvodWorkCycle.stepRunning = false;
    g_otvodWorkCycle.vfdRunning = false;
    g_otvodWorkCycle.stepObservedActive = false;
    g_otvodWorkCycle.vfdObservedBusy = false;
    g_otvodWorkCycle.vfdStopIssued = false;
    g_otvodWorkCycle.lastEvent = result;
    g_lastCommandResult = result;
    Serial.println(result);
    (void)mqttPublishStatus(false);
}

void abortOtvodWorkCycle(const String &reason, bool stopOutputs)
{
    if (stopOutputs) {
        stopOtvodWorkCycleOutputs();
    }
    g_otvodWorkCycle.readyForBatch = false;
    finishOtvodWorkCycle(reason);
}

bool startOtvodWorkCycle(uint8_t totalCycles, uint32_t stepSteps, float legacyVfdDistanceCm)
{
    if (g_otvodWorkCycle.active) {
        g_lastCommandResult = "OTCYCLE already active";
        Serial.println(g_lastCommandResult);
        return false;
    }

    if (totalCycles < OTVOD_WORK_MIN_CYCLES || totalCycles > OTVOD_WORK_MAX_CYCLES || stepSteps == 0) {
        g_lastCommandResult = "OTCYCLE failed: bad arguments";
        Serial.println(g_lastCommandResult);
        return false;
    }

    (void)legacyVfdDistanceCm;

    uint32_t vfdDoneDelayMs = conveyorVfdTickDurationMs(g_rs485Devices[CONVEYOR_ID]);
    if (vfdDoneDelayMs == 0) {
        bool conveyorBusy = false;
        if (!pollOtvodVfdCycleState(conveyorBusy, vfdDoneDelayMs)) {
            g_lastCommandResult = "OTCYCLE failed: conveyor offline";
            Serial.println(g_lastCommandResult);
            return false;
        }
    }

    if (vfdDoneDelayMs == 0) {
        g_lastCommandResult = "OTCYCLE failed: conveyor VFDTICK not configured";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_otvodWorkCycle = OtvodWorkCycleState{};
    g_otvodWorkCycle.active = true;
    g_otvodWorkCycle.readyForBatch = false;
    g_otvodWorkCycle.totalCycles = totalCycles;
    g_otvodWorkCycle.stepCommandSteps = stepSteps;
    g_otvodWorkCycle.stepDoneDelayMs = computeStepperMoveDurationMs(
        stepSteps, OTVOD_WORK_STEP_DELAY_US, OTVOD_WORK_STEP_COMPLETE_MARGIN_MS);
    {
        const uint32_t stepToVfdDelayMs = computeOtvodStepToVfdDelayMs(stepSteps);
        g_otvodWorkCycle.stepTriggerDelayMs =
            (stepToVfdDelayMs > g_otvodWorkCycle.stepDoneDelayMs)
                ? (stepToVfdDelayMs - g_otvodWorkCycle.stepDoneDelayMs)
                : 0U;
    }
    g_otvodWorkCycle.vfdDoneDelayMs = vfdDoneDelayMs;
    g_otvodWorkCycle.vfdTriggerDelayMs = OTVOD_WORK_VFD_SETTLE_MS;
    g_otvodWorkCycle.lastEvent = "OTCYCLE init";

    Serial.print("OTCYCLE: init step=");
    Serial.print(stepSteps);
    Serial.print(", main tick=");
    Serial.print(g_otvodWorkCycle.vfdDoneDelayMs);
    Serial.print(" ms, cycles=");
    Serial.print(totalCycles);
    Serial.print(", step settle=");
    Serial.print(g_otvodWorkCycle.stepTriggerDelayMs);
    Serial.print(" ms, vfd settle=");
    Serial.print(OTVOD_WORK_VFD_SETTLE_MS);
    Serial.println(" ms");

    const uint32_t nowMs = millis();
    if (!sendOtvodWorkStepCommand(1, nowMs)) {
        g_otvodWorkCycle = OtvodWorkCycleState{};
        g_otvodWorkCycle.readyForBatch = false;
        g_lastCommandResult = "OTCYCLE failed: first OTVOD start";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_lastCommandResult = "OTCYCLE started: " + String(totalCycles) +
        " cycles, OTVOD " + String(stepSteps) + ", main 1 tick";
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
    return true;
}

void processOtvodWorkCycle()
{
    if (!g_otvodWorkCycle.active) {
        return;
    }

    const uint32_t nowMs = millis();

    if (g_otvodWorkCycle.stepRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.stepStartedMs);
        if (elapsedMs >= g_otvodWorkCycle.stepDoneDelayMs) {
            g_otvodWorkCycle.stepRunning = false;
            g_otvodWorkCycle.stepRunsCompleted++;
            Serial.print("OTCYCLE: 2-СЂСѓС‡РµР№РєРѕРІС‹Р№ Р·Р°РІРµСЂС€РµРЅ ");
            Serial.print(g_otvodWorkCycle.stepRunsCompleted);
            Serial.print("/");
            Serial.println(g_otvodWorkCycle.totalCycles);
            g_otvodWorkCycle.readyForBatch =
                g_otvodWorkCycle.stepRunsCompleted >= g_otvodWorkCycle.totalCycles;

            if (g_otvodWorkCycle.readyForBatch) {
                g_otvodWorkCycle.lastEvent = "2-СЂСѓС‡РµР№РєРѕРІС‹Р№ РіРѕС‚РѕРІ РїСЂРёРЅСЏС‚СЊ РЅРѕРІСѓСЋ РїР°СЂС‚РёСЋ";
                g_lastCommandResult = g_otvodWorkCycle.lastEvent;
                Serial.println(g_lastCommandResult);
            }

            (void)mqttPublishStatus(false);
        }
    }

    if (g_otvodWorkCycle.active &&
        !g_otvodWorkCycle.stepTriggerIssued &&
        g_otvodWorkCycle.stepRunsStarted > g_otvodWorkCycle.vfdRunsStarted &&
        !g_otvodWorkCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.stepStartedMs);
        if (elapsedMs >= g_otvodWorkCycle.stepTriggerDelayMs) {
            if (!startOtvodWorkVfdRun(g_otvodWorkCycle.stepRunsStarted)) {
                abortOtvodWorkCycle("OTCYCLE failed: VFD start", true);
                return;
            }
            g_otvodWorkCycle.stepTriggerIssued = true;
            return;
        }
    }

    if (g_otvodWorkCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.vfdStartedMs);
        if (elapsedMs >= g_otvodWorkCycle.vfdDoneDelayMs) {
            const MbResult stopResult = modbusWriteSingleRetryToSlave(
                VFD_ID, VFD_REG_CMD, VFD_CMD_STOP_DEC, MODBUS_RETRY_COUNT);
            printMbResult(stopResult);
            g_otvodWorkCycle.vfdRunning = false;
            if (stopResult != MbResult::Ok) {
                abortOtvodWorkCycle("OTCYCLE failed: VFD stop", false);
                return;
            }

            g_otvodWorkCycle.vfdRunsCompleted++;
            Serial.print("OTCYCLE: РѕСЃРЅРѕРІРЅРѕР№ РѕС‚РІРѕРґРЅРѕР№ Р·Р°РІРµСЂС€РµРЅ ");
            Serial.print(g_otvodWorkCycle.vfdRunsCompleted);
            Serial.print("/");
            Serial.println(g_otvodWorkCycle.totalCycles);
            g_otvodWorkCycle.lastEvent = "РћСЃРЅРѕРІРЅРѕР№ РѕС‚РІРѕРґРЅРѕР№ Р·Р°РІРµСЂС€РµРЅ " +
                String(g_otvodWorkCycle.vfdRunsCompleted) + "/" + String(g_otvodWorkCycle.totalCycles);
            g_otvodWorkCycle.lastEvent = "OTVOD VFD completed " +
                String(g_otvodWorkCycle.vfdRunsCompleted) + "/" + String(g_otvodWorkCycle.totalCycles);
            (void)mqttPublishStatus(false);
        }
    }

    if (g_otvodWorkCycle.active &&
        !g_otvodWorkCycle.vfdRunning &&
        !g_otvodWorkCycle.stepRunning &&
        !g_otvodWorkCycle.vfdTriggerIssued &&
        g_otvodWorkCycle.vfdRunsCompleted < g_otvodWorkCycle.totalCycles &&
        g_otvodWorkCycle.stepRunsStarted == g_otvodWorkCycle.vfdRunsCompleted) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.vfdStartedMs);
        if (elapsedMs >= g_otvodWorkCycle.vfdTriggerDelayMs) {
            const uint8_t nextCycleIndex = static_cast<uint8_t>(g_otvodWorkCycle.vfdRunsCompleted + 1);
            if (!sendOtvodWorkStepCommand(nextCycleIndex, nowMs)) {
                abortOtvodWorkCycle("OTCYCLE failed: next OTVOD start", true);
                return;
            }
            g_otvodWorkCycle.vfdTriggerIssued = true;
        }
    }

    if (g_otvodWorkCycle.active &&
        !g_otvodWorkCycle.stepRunning &&
        !g_otvodWorkCycle.vfdRunning &&
        g_otvodWorkCycle.stepRunsCompleted >= g_otvodWorkCycle.totalCycles &&
        g_otvodWorkCycle.vfdRunsCompleted >= g_otvodWorkCycle.totalCycles) {
        finishOtvodWorkCycle("OTCYCLE completed: " + String(g_otvodWorkCycle.totalCycles) + " cycles finished");
    }
}

void processOtvodWorkCycleStrict()
{
    if (!g_otvodWorkCycle.active) {
        return;
    }

    const uint32_t nowMs = millis();

    if (g_otvodWorkCycle.stepRunning) {
        if ((uint32_t)(nowMs - g_otvodWorkCycle.stepStartedMs) > OTVOD_WORK_STEP_PHASE_TIMEOUT_MS) {
            abortOtvodWorkCycle("OTCYCLE failed: STEP2 timeout", true);
            return;
        }

        if ((uint32_t)(nowMs - g_otvodWorkCycle.lastPollMs) >= OTVOD_WORK_STEP_SETTLE_POLL_MS) {
            g_otvodWorkCycle.lastPollMs = nowMs;

            bool step2Active = false;
            if (pollOtvodStep2State(step2Active)) {
                const uint8_t step2CompletionSeq = conveyorStep2CompletionSeq(g_rs485Devices[CONVEYOR_ID]);
                const bool step2CompletedLatched =
                    step2CompletionSeq != g_otvodWorkCycle.stepCompletionSeqBase;
                if (step2Active) {
                    g_otvodWorkCycle.stepObservedActive = true;
                }
                if (step2CompletedLatched || (!step2Active && g_otvodWorkCycle.stepObservedActive)) {
                    g_otvodWorkCycle.stepRunning = false;
                    g_otvodWorkCycle.stepRunsCompleted++;
                    g_otvodWorkCycle.stepCompletedMs = nowMs;

                    if (step2CompletedLatched && !g_otvodWorkCycle.stepObservedActive) {
                        Serial.println("OTCYCLE: stepper completion detected by latched status.");
                    }

                    Serial.print("OTCYCLE: stepper completed ");
                    Serial.print(g_otvodWorkCycle.stepRunsCompleted);
                    Serial.print("/");
                    Serial.println(g_otvodWorkCycle.totalCycles);

                    g_otvodWorkCycle.readyForBatch =
                        g_otvodWorkCycle.stepRunsCompleted >= g_otvodWorkCycle.totalCycles;
                    if (g_otvodWorkCycle.readyForBatch) {
                        g_otvodWorkCycle.lastEvent = "OTVOD ready for next batch";
                        g_lastCommandResult = g_otvodWorkCycle.lastEvent;
                        Serial.println(g_lastCommandResult);
                    }
                    (void)mqttPublishStatus(false);
                }
            }
        }
    }

    if (g_otvodWorkCycle.active &&
        !g_otvodWorkCycle.stepRunning &&
        g_otvodWorkCycle.stepRunsStarted > g_otvodWorkCycle.vfdRunsStarted &&
        !g_otvodWorkCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.stepCompletedMs);
        if (elapsedMs >= g_otvodWorkCycle.stepTriggerDelayMs) {
            if (!startOtvodWorkVfdRun(g_otvodWorkCycle.stepRunsStarted)) {
                abortOtvodWorkCycle("OTCYCLE failed: VFDTICK start", true);
                return;
            }
            return;
        }
    }

    if (g_otvodWorkCycle.vfdRunning) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.vfdStartedMs);

        if ((uint32_t)(nowMs - g_otvodWorkCycle.lastPollMs) >= OTVOD_WORK_VFD_POLL_MS) {
            g_otvodWorkCycle.lastPollMs = nowMs;

            bool conveyorBusy = false;
            uint32_t tickDurationMs = 0;
            if (pollOtvodVfdCycleState(conveyorBusy, tickDurationMs)) {
                if (tickDurationMs > 0) {
                    g_otvodWorkCycle.vfdDoneDelayMs = tickDurationMs;
                }
                if (conveyorBusy) {
                    g_otvodWorkCycle.vfdObservedBusy = true;
                } else if (g_otvodWorkCycle.vfdObservedBusy) {
                    g_otvodWorkCycle.vfdRunning = false;
                    g_otvodWorkCycle.vfdObservedBusy = false;
                    g_otvodWorkCycle.vfdRunsCompleted++;
                    g_otvodWorkCycle.vfdCompletedMs = nowMs;
                    g_otvodWorkCycle.lastEvent = "OTVOD main completed " +
                        String(g_otvodWorkCycle.vfdRunsCompleted) + "/" + String(g_otvodWorkCycle.totalCycles);
                    Serial.println(g_otvodWorkCycle.lastEvent);
                    (void)mqttPublishStatus(false);
                    return;
                }
            }
        }

        if (!g_otvodWorkCycle.vfdObservedBusy && elapsedMs > OTVOD_WORK_VFD_START_TIMEOUT_MS) {
            abortOtvodWorkCycle("OTCYCLE failed: VFDTICK start timeout", true);
            return;
        }

        const uint32_t timeoutMs =
            (g_otvodWorkCycle.vfdDoneDelayMs > 0)
                ? (g_otvodWorkCycle.vfdDoneDelayMs + OTVOD_WORK_VFD_TIMEOUT_MARGIN_MS)
                : OTVOD_WORK_VFD_TIMEOUT_FALLBACK_MS;
        if (elapsedMs > timeoutMs) {
            abortOtvodWorkCycle("OTCYCLE failed: VFDTICK timeout", true);
            return;
        }
    }

    if (g_otvodWorkCycle.active &&
        !g_otvodWorkCycle.vfdRunning &&
        !g_otvodWorkCycle.stepRunning &&
        g_otvodWorkCycle.vfdRunsCompleted < g_otvodWorkCycle.totalCycles &&
        g_otvodWorkCycle.stepRunsStarted == g_otvodWorkCycle.vfdRunsCompleted) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_otvodWorkCycle.vfdCompletedMs);
        if (elapsedMs >= g_otvodWorkCycle.vfdTriggerDelayMs) {
            const uint8_t nextCycleIndex = static_cast<uint8_t>(g_otvodWorkCycle.vfdRunsCompleted + 1);
            if (!sendOtvodWorkStepCommand(nextCycleIndex, nowMs)) {
                abortOtvodWorkCycle("OTCYCLE failed: next OTVOD start", true);
                return;
            }
        }
    }

    if (g_otvodWorkCycle.active &&
        !g_otvodWorkCycle.stepRunning &&
        !g_otvodWorkCycle.vfdRunning &&
        g_otvodWorkCycle.stepRunsCompleted >= g_otvodWorkCycle.totalCycles &&
        g_otvodWorkCycle.vfdRunsCompleted >= g_otvodWorkCycle.totalCycles) {
        finishOtvodWorkCycle("OTCYCLE completed: " + String(g_otvodWorkCycle.totalCycles) + " cycles finished");
    }
}

void handleCommandOtCycle(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        Serial.println("OTCYCLE commands:");
        Serial.println("  OTCYCLE START [cycles] [steps] [cm_legacy] - start strict divert cycle");
        Serial.println("  OTCYCLE STOP    - stop current divert sequence");
        Serial.println("  OTCYCLE STATUS  - print current divert sequence state");
        return;
    }

    if (sub == "STATUS" || sub == "STATE") {
        printOtvodWorkCycleStatus();
        g_lastCommandResult = "OTCYCLE status printed";
        return;
    }

    if (sub == "START" || sub == "RUN") {
        uint32_t cyclesRaw = OTVOD_WORK_TOTAL_CYCLES;
        uint32_t stepsRaw = OTVOD_WORK_STEP_STEPS;
        float legacyCm = 0.0F;

        String cyclesTok = nextToken(args);
        String stepsTok = nextToken(args);
        String cmTok = nextToken(args);

        if (!cyclesTok.isEmpty() && !parseU32(cyclesTok, cyclesRaw)) {
            Serial.println("Usage: OTCYCLE START [cycles] [steps] [cm_legacy]");
            g_lastCommandResult = "OTCYCLE failed: bad cycles";
            return;
        }
        if (!stepsTok.isEmpty() && !parseU32(stepsTok, stepsRaw)) {
            Serial.println("Usage: OTCYCLE START [cycles] [steps] [cm_legacy]");
            g_lastCommandResult = "OTCYCLE failed: bad steps";
            return;
        }
        if (!cmTok.isEmpty() && !parseF32(cmTok, legacyCm)) {
            Serial.println("Usage: OTCYCLE START [cycles] [steps] [cm_legacy]");
            g_lastCommandResult = "OTCYCLE failed: bad legacy cm";
            return;
        }
        if (cyclesRaw > 255U) {
            Serial.println("Usage: OTCYCLE START [cycles] [steps] [cm_legacy]");
            g_lastCommandResult = "OTCYCLE failed: cycles out of range";
            return;
        }

        (void)startOtvodWorkCycle(static_cast<uint8_t>(cyclesRaw), stepsRaw, legacyCm);
        return;
    }

    if (sub == "STOP") {
        if (!g_otvodWorkCycle.active) {
            g_lastCommandResult = "OTCYCLE is not active";
            Serial.println(g_lastCommandResult);
            return;
        }
        abortOtvodWorkCycle("OTCYCLE stopped by user", true);
        return;
    }

    Serial.println("Unknown OTCYCLE subcommand. Use: OTCYCLE HELP");
    g_lastCommandResult = "OTCYCLE failed: unknown subcommand";
}

void handleCommandCommonCycle(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        Serial.println("COMMON commands:");
        Serial.println("  COMMON START   - start common production cycle / resume after pause");
        Serial.println("  COMMON PAUSE   - soft pause, no new manipulator cycle will start");
        Serial.println("  COMMON STATUS  - print common cycle state");
        return;
    }

    if (sub == "STATUS" || sub == "STATE") {
        printCommonCycleStatus();
        g_lastCommandResult = "COMMON status printed";
        return;
    }

    if (sub == "START" || sub == "RUN" || sub == "RESUME") {
        (void)startCommonCycle();
        return;
    }

    if (sub == "PAUSE") {
        requestCommonCyclePause();
        return;
    }

    Serial.println("Unknown COMMON subcommand. Use: COMMON HELP");
    g_lastCommandResult = "COMMON failed: unknown subcommand";
}

bool runVfdForFixedTime(float hz, uint32_t durationMs, uint16_t directionCmd, const char *directionLabel,
                        const String &commandSummary)
{
    float freqScale = 10.0F;
    const MbResult sr = readVfdFrequencyScale(freqScale);
    printMbResult(sr);
    if (sr != MbResult::Ok) {
        g_lastCommandResult = commandSummary + " failed: frequency scale read";
        Serial.println("VFD run aborted: failed to read frequency scale.");
        Serial.println(g_lastCommandResult);
        return false;
    }

    uint16_t rawU16 = 0;
    const MbResult setpointResult = writeVfdHzSetpoint(hz, freqScale, rawU16);
    if (setpointResult == MbResult::ArgError) {
        g_lastCommandResult = commandSummary + " failed: frequency out of range";
        Serial.println("VFD run aborted: frequency out of range.");
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (setpointResult != MbResult::Ok) {
        g_lastCommandResult = commandSummary + " failed: preset write";
        Serial.println("VFD run aborted at pre-set.");
        Serial.println(g_lastCommandResult);
        return false;
    }

    Serial.print(commandSummary);
    Serial.print(" start: ");
    Serial.print(directionLabel);
    Serial.print(", ");
    Serial.print(hz, 2);
    Serial.print(" Hz, ");
    Serial.print(durationMs);
    Serial.println(" ms");

    MbResult r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, directionCmd, MODBUS_RETRY_COUNT);
    printMbResult(r);
    if (r != MbResult::Ok) {
        g_lastCommandResult = commandSummary + " failed: start command";
        Serial.println("VFD run aborted at start command.");
        Serial.println(g_lastCommandResult);
        return false;
    }

    const uint32_t startedMs = millis();
    uint32_t nextLogMs = VFD_RUN5_LOG_STEP_MS;
    while (static_cast<uint32_t>(millis() - startedMs) < durationMs) {
        const uint32_t elapsedMs = static_cast<uint32_t>(millis() - startedMs);
        if (elapsedMs >= nextLogMs) {
            uint16_t runRawU16 = 0;
            const MbResult runR = modbusReadHoldingRetryFromSlave(g_modbusSlaveId, VFD_REG_FREQ_RUN, 1, &runRawU16, MODBUS_RETRY_COUNT, true);
            Serial.print("VFD run t=");
            Serial.print(nextLogMs / 1000U);
            Serial.print("s actual=");
            if (runR == MbResult::Ok) {
                const int16_t runRaw = static_cast<int16_t>(runRawU16);
                Serial.print(static_cast<float>(runRaw) / VFD_RUN_FREQ_SCALE, 2);
                Serial.println(" Hz");
            } else {
                Serial.println("n/a");
                printMbResult(runR);
            }
            nextLogMs += VFD_RUN5_LOG_STEP_MS;
        }
        delay(10);
    }

    r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_STOP_DEC, MODBUS_RETRY_COUNT);
    printMbResult(r);
    if (r == MbResult::Ok) {
        g_lastCommandResult = commandSummary + " command completed: " + String(hz, 2) + " Hz, " +
            String(durationMs) + " ms, " + String(directionLabel);
        Serial.println(g_lastCommandResult);
        return true;
    }

    g_lastCommandResult = commandSummary + " failed: stop command";
    Serial.println("VFD run stop failed.");
    Serial.println(g_lastCommandResult);
    return false;
}

void findVfdOnCommonUartSettings()
{
    struct UartProbe {
        uint8_t slaveId;
        uint32_t baud;
        uint32_t serialConfig;
    };

    static const UartProbe probes[] = {
        {1, 1200, SERIAL_8N2},
        {1, 1200, SERIAL_8E1},
        {1, 1200, SERIAL_8N1},
        {1, 2400, SERIAL_8N2},
        {1, 2400, SERIAL_8E1},
        {1, 2400, SERIAL_8N1},
        {1, 4800, SERIAL_8N2},
        {1, 4800, SERIAL_8E1},
        {1, 4800, SERIAL_8N1},
        {1, 9600, SERIAL_8N2},
        {1, 9600, SERIAL_8E1},
        {1, 9600, SERIAL_8N1},
        {1, 19200, SERIAL_8N2},
        {1, 19200, SERIAL_8E1},
        {1, 19200, SERIAL_8N1},
        {1, 38400, SERIAL_8N2},
        {1, 38400, SERIAL_8E1},
        {1, 38400, SERIAL_8N1},
        {VFD_ID, 1200, SERIAL_8N2},
        {VFD_ID, 1200, SERIAL_8E1},
        {VFD_ID, 1200, SERIAL_8N1},
        {VFD_ID, 2400, SERIAL_8N2},
        {VFD_ID, 2400, SERIAL_8E1},
        {VFD_ID, 2400, SERIAL_8N1},
        {VFD_ID, 4800, SERIAL_8N2},
        {VFD_ID, 4800, SERIAL_8E1},
        {VFD_ID, 4800, SERIAL_8N1},
        {VFD_ID, 9600, SERIAL_8N2},
        {VFD_ID, 9600, SERIAL_8E1},
        {VFD_ID, 9600, SERIAL_8N1},
        {VFD_ID, 19200, SERIAL_8N2},
        {VFD_ID, 19200, SERIAL_8E1},
        {VFD_ID, 19200, SERIAL_8N1},
        {VFD_ID, 38400, SERIAL_8N2},
        {VFD_ID, 38400, SERIAL_8E1},
        {VFD_ID, 38400, SERIAL_8N1},
    };

    const uint32_t prevBaud = g_rs485Baud;
    const uint32_t prevSerialConfig = g_rs485SerialConfig;

    Serial.println("VFD FIND: probing common RS485 UART settings for slave 1 and 11...");

    for (const UartProbe &probe : probes) {
        rs485ApplyUart(probe.baud, probe.serialConfig);

        Serial.print("  id=");
        Serial.print(probe.slaveId);
        Serial.print(" ");
        Serial.print(probe.baud);
        Serial.print(" ");
        Serial.print(rs485SerialConfigName(probe.serialConfig));
        Serial.print(" -> ");

        uint16_t reg = 0;
        uint8_t exceptionCode = 0;
        const MbResult r = modbusReadHoldingFromSlave(probe.slaveId, VFD_REG_STATUS, 1, &reg, &exceptionCode, false);

        if (isMbAliveResult(r)) {
            Serial.print("alive");
            if (r == MbResult::Ok) {
                Serial.print(", status=");
                Serial.print(reg);
            } else {
                Serial.print(", ex=0x");
                Serial.print(exceptionCode, HEX);
            }
            Serial.println();

            g_modbusSlaveId = probe.slaveId;
            rs485ScanResetAll();
            if (probe.slaveId == VFD_ID) {
                rs485ScanProbeId(VFD_ID);
            }
            Serial.print("VFD FIND: selected ");
            Serial.print("id=");
            Serial.print(g_modbusSlaveId);
            Serial.print(", ");
            Serial.print(g_rs485Baud);
            Serial.print(" ");
            Serial.println(rs485SerialConfigName(g_rs485SerialConfig));
            return;
        }

        Serial.println(mbResultCode(r));
    }

    rs485ApplyUart(prevBaud, prevSerialConfig);
    rs485ScanResetAll();
    Serial.println("VFD FIND: no response from slave 1 or 11 on common UART settings.");
}

void printVfdStatusWord(uint16_t statusWord)
{
    Serial.print("VFD STATUS: ");
    switch (statusWord) {
        case 0x0001:
            Serial.println("FORWARD");
            break;
        case 0x0002:
            Serial.println("REVERSE");
            break;
        case 0x0003:
            Serial.println("STOP");
            break;
        default:
            Serial.print("UNKNOWN (");
            Serial.print(statusWord);
            Serial.println(")");
            break;
    }
}

void handleCommandVfd(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        printVfdUsage();
        return;
    }

    if (sub == "CFG" || sub == "CONFIG") {
        printVfdConfigSummary(g_modbusSlaveId);
        return;
    }

    if (sub == "SETUPRS") {
        String idTok = nextToken(args);
        uint16_t slaveId = VFD_ID;
        if (!idTok.isEmpty() && !parseU16(idTok, slaveId)) {
            Serial.println("Usage: VFD SETUPRS [id]");
            return;
        }
        if (!setupVr70ForMasterRs485(static_cast<uint8_t>(slaveId))) {
            Serial.println("VFD SETUPRS failed. If the drive is still on factory UART settings, run VFD FIND first.");
        }
        return;
    }

    if (sub == "START") {
        String hzTok = nextToken(args);
        String dirTok = nextToken(args);

        float hz = VFD_RUN5_DEFAULT_HZ;
        if (!hzTok.isEmpty() && !parseF32(hzTok, hz)) {
            Serial.println("Usage: VFD START [hz] [FWD|REV]");
            return;
        }
        if (!(hz > 0.0F)) {
            Serial.println("Usage: VFD START [hz] [FWD|REV]");
            return;
        }

        uint16_t directionCmd = VFD_CMD_FWD;
        const char *directionLabel = "FWD";
        if (!parseVfdDirectionToken(dirTok, directionCmd, directionLabel)) {
            Serial.println("Usage: VFD START [hz] [FWD|REV]");
            return;
        }

        float freqScale = 10.0F;
        const MbResult sr = readVfdFrequencyScale(freqScale);
        if (sr != MbResult::Ok) {
            printMbResult(sr);
            g_lastCommandResult = "VFD START failed: frequency scale read";
            return;
        }

        uint16_t rawU16 = 0;
        const MbResult setpointResult = writeVfdHzSetpoint(hz, freqScale, rawU16);
        if (setpointResult == MbResult::ArgError) {
            Serial.println("VFD START: frequency out of range.");
            g_lastCommandResult = "VFD START failed: frequency out of range";
            return;
        }
        if (setpointResult != MbResult::Ok) {
            g_lastCommandResult = "VFD START failed: setpoint write";
            return;
        }

        MbResult r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, directionCmd, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r != MbResult::Ok) {
            g_lastCommandResult = "VFD START failed: run command write";
            return;
        }

        Serial.print("VFD continuous start: ");
        Serial.print(hz, 2);
        Serial.print(" Hz, ");
        Serial.println(directionLabel);
        g_lastCommandResult = "VFD START executed";
        return;
    }

    if (sub == "FWD" || sub == "RUN") {
        printMbResult(modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_FWD, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "REV") {
        printMbResult(modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_REV, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "STOP") {
        if (g_otvodWorkCycle.active) {
            abortOtvodWorkCycle("OTCYCLE stopped by VFD STOP", true);
        }
        printMbResult(modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_STOP_DEC, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "RESET") {
        printMbResult(modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_RESET, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "STAT" || sub == "STATUS") {
        uint16_t reg = 0;
        const MbResult r = modbusReadHoldingRetryFromSlave(g_modbusSlaveId, VFD_REG_STATUS, 1, &reg, MODBUS_RETRY_COUNT, true);
        printMbResult(r);
        if (r == MbResult::Ok) {
            printVfdStatusWord(reg);
        }
        return;
    }

    if (sub == "FAULT") {
        uint16_t reg = 0;
        const MbResult r = modbusReadHoldingRetryFromSlave(g_modbusSlaveId, VFD_REG_FAULT, 1, &reg, MODBUS_RETRY_COUNT, true);
        printMbResult(r);
        if (r == MbResult::Ok) {
            Serial.print("VFD FAULT: ");
            Serial.print(reg);
            Serial.print(" (0x");
            Serial.print(reg, HEX);
            Serial.println(")");
        }
        return;
    }

    if (sub == "FREQRAW") {
        String valueTok = nextToken(args);
        int32_t signedRaw = 0;
        if (!parseI32(valueTok, signedRaw)) {
            Serial.println("Usage: VFD FREQRAW <-10000..10000>");
            return;
        }

        if (signedRaw < VFD_FREQ_RAW_MIN || signedRaw > VFD_FREQ_RAW_MAX) {
            Serial.print("VFD FREQRAW out of range: ");
            Serial.print(VFD_FREQ_RAW_MIN);
            Serial.print("..");
            Serial.println(VFD_FREQ_RAW_MAX);
            return;
        }

        const uint16_t valueRawU16 = static_cast<uint16_t>(static_cast<int16_t>(signedRaw));
        printMbResult(modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_FREQ_SET, valueRawU16, MODBUS_RETRY_COUNT));
        Serial.print("VFD setpoint raw: ");
        Serial.print(signedRaw);
        Serial.print(" (~");
        Serial.print((static_cast<float>(signedRaw) / VFD_COMM_SCALE) * 100.0F, 2);
        Serial.println("%)");
        return;
    }

    if (sub == "MAXHZ") {
        String hzTok = nextToken(args);
        if (hzTok.isEmpty()) {
            Serial.print("VFD MAXHZ: ");
            Serial.println(g_vfdBaseHz, 2);
            return;
        }

        float hz = 0.0F;
        if (!parseF32(hzTok, hz) || hz <= 0.0F || hz > 400.0F) {
            Serial.println("Usage: VFD MAXHZ <0.01..400>");
            return;
        }

        g_vfdBaseHz = hz;
        Serial.print("VFD MAXHZ set to ");
        Serial.println(g_vfdBaseHz, 2);
        return;
    }

    if (sub == "FREQ") {
        String hzTok = nextToken(args);
        float hz = 0.0F;
        if (!parseF32(hzTok, hz)) {
            Serial.println("Usage: VFD FREQ <hz>");
            return;
        }

        float freqScale = 10.0F;
        const MbResult sr = readVfdFrequencyScale(freqScale);
        if (sr != MbResult::Ok) {
            printMbResult(sr);
            return;
        }

        uint16_t rawU16 = 0;
        const MbResult r = writeVfdHzSetpoint(hz, freqScale, rawU16);
        if (r == MbResult::ArgError) {
            Serial.println("VFD FREQ out of range.");
            return;
        }
        if (r == MbResult::Ok) {
            Serial.print("VFD setpoint Hz: ");
            Serial.print(hz, 2);
            Serial.print(" (raw=");
            Serial.print(rawU16);
            Serial.println(")");
        }
        return;
    }

    if (sub == "RUNFREQ") {
        uint16_t reg = 0;
        const MbResult r = modbusReadHoldingRetryFromSlave(g_modbusSlaveId, VFD_REG_FREQ_RUN, 1, &reg, MODBUS_RETRY_COUNT, true);
        printMbResult(r);
        if (r == MbResult::Ok) {
            const int16_t signedRaw = static_cast<int16_t>(reg);
            Serial.print("VFD RUN FREQ RAW: ");
            Serial.print(signedRaw);
            Serial.print(" (~");
            Serial.print(static_cast<float>(signedRaw) / VFD_RUN_FREQ_SCALE, 2);
            Serial.println(" Hz)");
        }
        return;
    }

    if (sub == "FIND") {
        findVfdOnCommonUartSettings();
        return;
    }

    if (sub == "RUN5") {
        String hzTok = nextToken(args);
        String dirTok = nextToken(args);

        float hz = VFD_RUN5_DEFAULT_HZ;
        if (!hzTok.isEmpty() && !parseF32(hzTok, hz)) {
            Serial.println("Usage: VFD RUN5 [hz] [FWD|REV]");
            return;
        }

        uint16_t directionCmd = VFD_CMD_FWD;
        const char *directionLabel = "FWD";
        if (!parseVfdDirectionToken(dirTok, directionCmd, directionLabel)) {
            Serial.println("Usage: VFD RUN5 [hz] [FWD|REV]");
            return;
        }

        (void)runVfdForFixedTime(hz, VFD_RUN5_DURATION_MS, directionCmd, directionLabel, "VFD RUN5");
        return;
    }

    if (sub == "RUN5MIN" || sub == "RUN300") {
        String hzTok = nextToken(args);
        String dirTok = nextToken(args);

        float hz = VFD_RUN5_DEFAULT_HZ;
        if (!hzTok.isEmpty() && !parseF32(hzTok, hz)) {
            Serial.println("Usage: VFD RUN5MIN [hz] [FWD|REV]");
            return;
        }

        uint16_t directionCmd = VFD_CMD_FWD;
        const char *directionLabel = "FWD";
        if (!parseVfdDirectionToken(dirTok, directionCmd, directionLabel)) {
            Serial.println("Usage: VFD RUN5MIN [hz] [FWD|REV]");
            return;
        }

        (void)runVfdForFixedTime(hz, VFD_RUN5MIN_DURATION_MS, directionCmd, directionLabel, "VFD RUN5MIN");
        return;
    }

    if (sub == "RUN35" || sub == "RUNCM" || sub == "MOVECM") {
        const bool fixed35Command = (sub == "RUN35");
        const String usage = fixed35Command ? "Usage: VFD RUN35 [hz] [FWD|REV]" :
            "Usage: VFD RUNCM <cm> [hz] [FWD|REV]";
        String cmTok = (sub == "RUN35") ? String(VFD_RUNCM_DEFAULT_CM, 1) : nextToken(args);
        String hzTok = nextToken(args);
        String dirTok = nextToken(args);

        if (cmTok.isEmpty()) {
            Serial.println(usage);
            g_lastCommandResult = "VFD RUNCM failed: missing distance";
            return;
        }

        float distanceCm = VFD_RUNCM_DEFAULT_CM;
        if (!parseF32(cmTok, distanceCm) || distanceCm <= 0.0F) {
            Serial.println(usage);
            g_lastCommandResult = "VFD RUNCM failed: bad distance";
            return;
        }

        float hz = VFD_RUN5_DEFAULT_HZ;
        if (!hzTok.isEmpty() && (!parseF32(hzTok, hz) || hz <= 0.0F)) {
            Serial.println(usage);
            g_lastCommandResult = "VFD RUNCM failed: bad frequency";
            return;
        }

        uint16_t directionCmd = VFD_CMD_FWD;
        const char *directionLabel = "FWD";
        if (!parseVfdDirectionToken(dirTok, directionCmd, directionLabel)) {
            Serial.println(usage);
            g_lastCommandResult = "VFD RUNCM failed: bad direction";
            return;
        }

        uint32_t durationMs = 0;
        if (!computeVfdRunDurationForDistanceCm(distanceCm, hz, durationMs)) {
            Serial.println("VFD RUNCM: duration out of range.");
            g_lastCommandResult = "VFD RUNCM failed: duration out of range";
            return;
        }

        Serial.print("VFD RUNCM calibration: ref ");
        Serial.print(VFD_RUNCM_REF_DISTANCE_CM, 1);
        Serial.print(" cm in ");
        Serial.print(VFD_RUNCM_REF_DURATION_MS);
        Serial.print(" ms at ");
        Serial.print(VFD_RUNCM_REF_HZ, 2);
        Serial.print(" Hz -> target ");
        Serial.print(distanceCm, 1);
        Serial.print(" cm = ");
        Serial.print(durationMs);
        Serial.println(" ms");

        const String commandSummary = fixed35Command ? "VFD RUN35" :
            ("VFD RUNCM " + String(distanceCm, 1) + " cm");
        (void)runVfdForFixedTime(hz, durationMs, directionCmd, directionLabel, commandSummary);
        return;
    }

    if (sub == "CYCLE30") {
        String minTok = nextToken(args);
        String maxTok = nextToken(args);

        float minHz = VFD_CYCLE30_DEFAULT_MIN_HZ;
        float maxHz = VFD_CYCLE30_DEFAULT_MAX_HZ;

        if (!minTok.isEmpty() && !parseF32(minTok, minHz)) {
            Serial.println("Usage: VFD CYCLE30 [minHz] [maxHz]");
            return;
        }
        if (!maxTok.isEmpty() && !parseF32(maxTok, maxHz)) {
            Serial.println("Usage: VFD CYCLE30 [minHz] [maxHz]");
            return;
        }
        if (maxHz <= minHz) {
            Serial.println("VFD CYCLE30: maxHz must be > minHz.");
            return;
        }

        uint16_t rawMinU16 = 0;
        uint16_t rawMaxU16 = 0;
        float freqScale = 10.0F;
        const MbResult sr = readVfdFrequencyScale(freqScale);
        if (sr != MbResult::Ok) {
            printMbResult(sr);
            return;
        }

        if (!hzToPanelRawU16(minHz, freqScale, rawMinU16) ||
            !hzToPanelRawU16(maxHz, freqScale, rawMaxU16)) {
            Serial.println("VFD CYCLE30: frequency out of range.");
            return;
        }

        Serial.print("VFD CYCLE30 start: ");
        Serial.print(minHz, 2);
        Serial.print(" -> ");
        Serial.print(maxHz, 2);
        Serial.println(" -> STOP");

        MbResult r = writeVfdHzSetpoint(minHz, freqScale, rawMinU16);
        if (r == MbResult::ArgError) {
            Serial.println("VFD CYCLE30: frequency out of range.");
            return;
        }
        if (r != MbResult::Ok) {
            Serial.println("VFD CYCLE30 aborted at pre-set.");
            return;
        }

        r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_FWD, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r != MbResult::Ok) {
            Serial.println("VFD CYCLE30 aborted at start command.");
            return;
        }

        const uint32_t halfMs = VFD_CYCLE30_TOTAL_MS / 2U;
        const uint32_t steps = VFD_CYCLE30_TOTAL_MS / VFD_CYCLE30_STEP_MS;
        const uint32_t cycleStartMs = millis();

        for (uint32_t step = 0; step <= steps; step++) {
            const uint32_t elapsed = step * VFD_CYCLE30_STEP_MS;
            while (static_cast<uint32_t>(millis() - cycleStartMs) < elapsed) {
                delay(1);
            }

            float phase = 0.0F;
            float targetHz = minHz;

            if (elapsed <= halfMs) {
                phase = static_cast<float>(elapsed) / static_cast<float>(halfMs);
                targetHz = minHz + (maxHz - minHz) * phase;
            } else {
                phase = static_cast<float>(elapsed - halfMs) / static_cast<float>(halfMs);
                targetHz = maxHz - (maxHz - minHz) * phase;
            }

            uint16_t targetRawU16 = 0;
            if (!hzToPanelRawU16(targetHz, freqScale, targetRawU16)) {
                Serial.println("VFD CYCLE30: internal target out of range, stopping.");
                break;
            }

            r = writeVfdHzSetpoint(targetHz, freqScale, targetRawU16);
            if (r != MbResult::Ok) {
                Serial.println("VFD CYCLE30 write failed, stopping.");
                break;
            }

            uint16_t runRawU16 = 0;
            const MbResult runR = modbusReadHoldingRetryFromSlave(g_modbusSlaveId, VFD_REG_FREQ_RUN, 1, &runRawU16, MODBUS_RETRY_COUNT, true);

            Serial.print("CYCLE30 t=");
            Serial.print(elapsed / 1000U);
            Serial.print("s target=");
            Serial.print(targetHz, 2);
            Serial.print("Hz actual=");

            if (runR == MbResult::Ok) {
                g_mbOkCount++;
                const int16_t runRaw = static_cast<int16_t>(runRawU16);
                Serial.print(static_cast<float>(runRaw) / VFD_RUN_FREQ_SCALE, 2);
                Serial.println("Hz");
            } else {
                Serial.println("n/a");
                printMbResult(runR);
            }
        }

        r = modbusWriteSingleRetryToSlave(g_modbusSlaveId, VFD_REG_CMD, VFD_CMD_STOP_DEC, MODBUS_RETRY_COUNT);
        printMbResult(r);
        Serial.println("VFD CYCLE30 finished.");
        return;
    }

    Serial.println("Unknown VFD subcommand. Use: VFD HELP");
}

bool handleConsoleLine(String line)
{
    line.trim();
    if (line.isEmpty()) {
        g_lastCommandResult = "РїСѓСЃС‚Р°СЏ РєРѕРјР°РЅРґР°";
        return false;
    }

    g_lastCommandResult = "РІС‹РїРѕР»РЅРµРЅРѕ";

    String cmd = line;
    String args = "";
    const int sp = cmd.indexOf(' ');
    if (sp > 0) {
        args = cmd.substring(sp + 1);
        cmd = cmd.substring(0, sp);
    } else if (sp == 0) {
        cmd = "";
    }

    cmd.trim();
    cmd.toUpperCase();
    args.trim();

    if (cmd == "H" || cmd == "HELP") {
        printHelp();
        return true;
    }

    if (cmd == "STATE") {
        printState();
        return true;
    }

    if (cmd == "MBID") {
        handleCommandMbId(args);
        return true;
    }

    if (cmd == "MBUART") {
        handleCommandMbUart(args);
        return true;
    }

    if (cmd == "MBR") {
        handleCommandMbRead(args);
        return true;
    }

    if (cmd == "MBRAW") {
        handleCommandMbRaw(args);
        return true;
    }

    if (cmd == "MBW") {
        handleCommandMbWrite(args);
        return true;
    }

    if (cmd == "MBSCAN") {
        handleCommandMbScan(args);
        return true;
    }

    if (cmd == "VFD") {
        handleCommandVfd(args);
        return true;
    }

    if (cmd == "SEAL") {
        handleCommandSeal(args);
        return true;
    }

    if (cmd == "COMMON" || cmd == "AUTOCYCLE") {
        handleCommandCommonCycle(args);
        return true;
    }

    if (cmd == "OTCYCLE" || cmd == "OTVODCYCLE") {
        handleCommandOtCycle(args);
        return true;
    }

    if (cmd == "WIFI") {
        handleCommandWifi(args);
        return true;
    }

    if (cmd == "MQTT") {
        handleCommandMqtt(args);
        return true;
    }

    if (cmd == "CONV" || cmd == "CONVEYOR") {
        return handleManagedDeviceConsoleCommand(CONVEYOR_ID, "РєРѕРЅРІРµР№РµСЂ", args);
    }

    if (cmd == "MAN" || cmd == "MANIP" || cmd == "MANIPULATOR") {
        return handleManagedDeviceConsoleCommand(MANIPULATOR_ID, "РјР°РЅРёРїСѓР»СЏС‚РѕСЂ", args);
    }

    if (cmd == "RS") {
        rs485SendTextLine(args);
        return true;
    }

    if (cmd == "RSPING") {
        handleCommandRsPing(args);
        return true;
    }

    String mapped;
    if (mapToFlagCommand(cmd + (args.isEmpty() ? "" : String(" ") + args), mapped)) {
        return handleManagedDeviceConsoleCommand(CONVEYOR_ID, "conveyor", mapped);
    }

    g_lastCommandResult = "РЅРµРёР·РІРµСЃС‚РЅР°СЏ РєРѕРјР°РЅРґР°";
    Serial.println("Unknown command. Use H.");
    return false;
}

void processConsole()
{
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_serialCmdBuffer.isEmpty()) {
                (void)handleConsoleLine(g_serialCmdBuffer);
                g_serialCmdBuffer = "";
            }
            continue;
        }

        g_serialCmdBuffer += ch;
    }
}

void processRs485RxText()
{
    while (Serial2.available() > 0) {
        const char ch = static_cast<char>(Serial2.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_rs485RxBuffer.isEmpty()) {
                g_rsRxCount++;
                Serial.print("RS485 RX: ");
                Serial.println(g_rs485RxBuffer);
                g_rs485RxBuffer = "";
            }
            continue;
        }

        g_rs485RxBuffer += ch;
        if (g_rs485RxBuffer.length() > RS485_BUFFER_MAX_LEN) {
            g_rs485RxBuffer = "";
            Serial.println("RS485 RX overflow, buffer cleared.");
        }
    }
}

void processRs485DiagPassive()
{
    if (Serial2.available() < static_cast<int>(RS485_DIAG_FRAME_LEN)) {
        return;
    }
    if (Serial2.peek() != RS485_DIAG_MAGIC_0) {
        return;
    }

    uint8_t frame[RS485_DIAG_FRAME_LEN] = {};
    if (!rs485ReadExact(frame, sizeof(frame), 10)) {
        return;
    }

    if (rs485HandleDiagFrame(frame, true)) {
        g_rsRxCount++;
    }
}

void printBanner()
{
    Serial.println();
    Serial.print("=== ");
    Serial.print(DEVICE_NAME);
    Serial.println(" ===");
    Serial.print("FW: ");
    Serial.println(FW_VERSION);
    Serial.println("Role: VFD over RS485, conveyor/manipulator over I2C.");
    Serial.print("RS485: RX=");
    Serial.print(PIN_RS485_RX);
    Serial.print(", TX=");
    Serial.print(PIN_RS485_TX);
    Serial.print(", DE/RE=");
    Serial.println(PIN_RS485_DE_RE);
    printRs485Uart();
    Serial.print("I2C: SDA=");
    Serial.print(PIN_I2C_SDA);
    Serial.print(", SCL=");
    Serial.println(PIN_I2C_SCL);
    Serial.print("SEAL: START=");
    Serial.print(PIN_SEAL_START);
    Serial.print(", DONE=");
    Serial.println(PIN_SEAL_DONE);
    Serial.print("MODBUS default slave id: ");
    Serial.println(g_modbusSlaveId);
    Serial.print("MBSCAN: ");
    Serial.print(g_rs485ScanEnabled ? "on" : "off");
    Serial.print(", range=");
    Serial.print(g_rs485ScanMinId);
    Serial.print("..");
    Serial.println(g_rs485ScanMaxId);
    wifiPrintStatus();
    mqttPrintStatus();
    printHelp();
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    WiFi.mode(WIFI_STA);
    if (!wifiTryAutoConnect(true)) {
        Serial.println("WIFI: startup connect skipped/failed. Use WIFI HELP.");
    }

    (void)mqttLoadSettings();
    if (WiFi.status() == WL_CONNECTED && !g_mqttHost.isEmpty()) {
        (void)mqttConnectNow();
    } else if (g_mqttHost.isEmpty()) {
        Serial.println("MQTT: host is not configured. Use MQTT SET <host> <port>.");
    }

    pinMode(PIN_RS485_DE_RE, OUTPUT);
    rs485SetReceiveMode();
    rs485ApplyUart(RS485_BAUD, SERIAL_8E1);
    pinMode(PIN_SEAL_START, OUTPUT);
    sealWriteStartOutput(false);
    pinMode(PIN_SEAL_DONE, INPUT_PULLUP);
    g_sealDoneLastActive = sealIsDoneActive();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    rs485ScanResetAll();

    printBanner();
}

void loop()
{
    processConsole();
    rs485ScanLoop();
    mqttLoop();
    processSealIo();
    processOtvodWorkCycleStrict();
    processCommonCycle();
    processRs485DiagPassive();
    if (RS485_TEXT_SNIFFER_ENABLED) {
        processRs485RxText();
    }
}
