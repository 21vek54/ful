// Production legacy master COM12: orchestrates COMMON, pause-contract and field diagnostics for the line.
// Здесь живет актуальный runtime мастера, включая новый request -> wait ready -> finalize pause-контракт.

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
// COM12 I2C hardening: timeout mitigates silent Wire hangs but is not a proof
// that the underlying root cause is eliminated; recovery remains protocol-level.
// Watchdog resets are intentionally not used as a replacement for root-cause fixes.
constexpr uint32_t I2C_TIMEOUT_MS = 40;
constexpr uint32_t I2C_WARN_LATENCY_MS = 30;
constexpr uint32_t I2C_RECOVER_COOLDOWN_MS = 500;
constexpr uint32_t I2C_START_TRACE_THROTTLE_MS = 5000;
constexpr uint32_t I2C_WIRE_TRACE_SAMPLE_EVERY = 32;
constexpr uint32_t I2C_ACTIVE_GUARD_MS = 350;
constexpr uint32_t LOOP_HEARTBEAT_INTERVAL_MS = 5000;
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
constexpr uint16_t DEVICE_STATUS_BUSY_BIT = 1U << 1;
constexpr uint16_t DEVICE_STATUS_ALARM_BIT = 1U << 3;
constexpr uint16_t CONVEYOR_STATUS_PROGRAM1_ACTIVE_BIT = 1U << 5;
constexpr uint16_t CONVEYOR_STATUS_BATCH_READY_BIT = 1U << 6;
constexpr uint16_t CONVEYOR_STATUS_STEP2_ACTIVE_BIT = 1U << 7;
constexpr uint16_t CONVEYOR_STATUS_OUTFEED_READY_FOR_BATCH_BIT = 1U << 15;
constexpr uint16_t CONVEYOR_ERROR_POST7_MANUAL_RECOVERY_REQUIRED_BIT = 1U << 2;
constexpr uint16_t CONVEYOR_EXTRA0_FLAG_MASK = 0x0003U;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_SEQ_LOW_SHIFT = 2U;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_SEQ_LOW_MASK = 0x003FU;
constexpr uint16_t CONVEYOR_EXTRA0_MOTION_SHIFT = 8U;
constexpr uint16_t CONVEYOR_EXTRA0_MOTION_MASK = 0x0003U;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_BUSY_BIT = 1U << 10;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_START_PULSE_BIT = 1U << 11;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_DONE_ACTIVE_BIT = 1U << 12;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_SEQ_HIGH_SHIFT = 13U;
constexpr uint16_t CONVEYOR_EXTRA0_SEALER_SEQ_HIGH_MASK = 0x0003U;
constexpr uint16_t CONVEYOR_EXTRA0_VFD_TIMED_RUN_ACTIVE_BIT = 1U << 15;
constexpr uint16_t CONVEYOR_EXTRA2_FEED_SIDE_EMPTY_STRICT_BIT = 1U << 14;
constexpr uint16_t CONVEYOR_EXTRA2_FEED_SIDE_EMPTY_VALID_BIT = 1U << 15;
constexpr uint8_t CONVEYOR_EXTRA2_PROGRAM_PASS_MASK = 0x3FU;
constexpr uint16_t MANIPULATOR_STATUS_STEP7_READY_BIT = 1U << 5;
constexpr uint16_t MANIPULATOR_STATUS_STEP3_READY_BIT = 1U << 6;
constexpr uint16_t MANIPULATOR_STATUS_WORK_START_READY_BIT = 1U << 7;
constexpr uint16_t MANIPULATOR_STATUS_NEEDS_GRIP_OPEN_ONLY_BIT = 1U << 8;
constexpr uint16_t MANIPULATOR_SENSOR_RIGHT_LIMIT_BIT = 1U << 1;
constexpr uint16_t MANIPULATOR_SENSOR_Z_UP_BIT = 1U << 2;
constexpr uint16_t MANIPULATOR_SENSOR_GRIP_OPEN_BIT = 1U << 4;
constexpr uint16_t MANIPULATOR_SENSOR_GRIP_CLOSED_BIT = 1U << 5;

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
constexpr uint8_t OTVOD_WORK_VFD_START_RETRY_MAX = 1;
constexpr uint32_t OTVOD_WORK_VFD_TIMEOUT_MARGIN_MS = 4000;
constexpr uint32_t OTVOD_WORK_VFD_TIMEOUT_FALLBACK_MS = 15000;
constexpr uint32_t COMMON_CYCLE_SEAL_SETTLE_MS = 1000;
constexpr uint32_t COMMON_DIAG_WAIT_LOG_MS = 15000;
constexpr uint32_t COMMON_WAIT_MANIP_READY_TIMEOUT_MS = 12000;
constexpr uint32_t COMMON_WAIT_OUTFEED_READY_TIMEOUT_MS = 30000;
constexpr uint32_t COMMON_ABORT_DRAIN_TIMEOUT_MS = 45000;
constexpr uint32_t COMMON_ABORT_DRAIN_NO_PROGRESS_MS = 12000;
constexpr uint32_t COMMON_ABORT_DRAIN_WAIT_LOG_MS = 5000;
constexpr uint32_t COMMON_ABORT_DRAIN_SETTLE_MS = 500;
constexpr uint8_t COMMON_ABORT_DRAIN_SETTLE_MIN_CHECKS = 2;
constexpr uint32_t COMMON_REFILL_RETRY_GUARD_MS = RS485_SCAN_STEP_INTERVAL_MS + 250;
constexpr uint32_t COMMON_PAUSE_STRICT_SNAPSHOT_SYNC_RETRY_MS = 1000;
constexpr uint32_t COMMON_PAUSE_STRICT_SNAPSHOT_FORCE_DRAIN_MS = 8000;
constexpr uint32_t COMMON_PAUSE_ARMING_TIMEOUT_MS = 45000;
constexpr uint32_t COMMON_PAUSE_ARMING_NO_PROGRESS_MS = 12000;
constexpr uint32_t COMMON_PAUSE_ARMING_WAIT_LOG_MS = 5000;
constexpr uint32_t COMMON_PAUSE_MANDATORY_UNLOAD_TIMEOUT_MS = 90000;
constexpr uint32_t COMMON_PAUSE_STATUS_POLL_MS = 1000;
constexpr int16_t COMMON_PAUSE_SEALER_RESIDUAL_COUNT = 6;
constexpr uint32_t COMMON_PAUSE_NODE_FRAME_STALE_MS = 900;
constexpr uint32_t COMMON_PAUSE_MANDATORY_UNLOAD_BUSY_HEARTBEAT_MS = 3000;
constexpr uint32_t COMMON_PAUSE_NODE_PREPARING_HEARTBEAT_MS = 3000;
constexpr uint32_t COMMON_PAUSE_FINALIZE_ACTIVITY_HEARTBEAT_MS = 3000;
constexpr uint32_t COMMON_PAUSE_FILL_LAST_BLOCK_DECISION_MS = 3000;
constexpr uint16_t CONVEYOR_ERROR_PAUSE_STATUS_ACK_VALID_BIT = 1U << 3;
constexpr uint32_t CONVEYOR_HEARTBEAT_PAUSE_STATE_SHIFT = 30U;
constexpr uint32_t CONVEYOR_HEARTBEAT_PAUSE_EXT_VALID_BIT = 1U << 29U;
constexpr uint32_t CONVEYOR_HEARTBEAT_RESIDUAL_KNOWN_BIT = 1U << 28U;
constexpr uint8_t CONVEYOR_HEARTBEAT_PAUSE_COUNT_UNKNOWN = 0x0FU;
constexpr uint16_t MANIPULATOR_ERROR_PAUSE_STATUS_ACK_VALID_BIT = 1U << 5;
#ifndef MASTER_COMMON_TRACE_DEBUG
#define MASTER_COMMON_TRACE_DEBUG 0
#endif
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
    uint32_t managedHeartbeatMs = 0;
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
    uint8_t vfdStartRetryCount = 0;
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

enum class CommonPauseState : uint8_t {
    None = 0,
    Requested = 1,
    FillLastBlock = 2,
    StableWait = 3,
    WaitStrictFeedSnapshot = 4
};

enum class CommonRefillLaunchResult : uint8_t {
    Failed = 0,
    AlreadyReady = 1,
    AlreadyProgram1Active = 2,
    Pending = 3,
    SkippedBusy = 4,
    Requested = 5
};

enum class CommonAbortPost7Policy : uint8_t {
    ForceStop = 0,
    AllowDrainToSafe = 1
};

enum class CommonAbortPost7Decision : uint8_t {
    ForceStop = 0,
    SkipAlreadySettled = 1,
    SkipDrainToSafe = 2
};

enum class CommonRuntimeState : uint8_t {
    Idle = 0,
    Active = 1,
    CommonAborted = 2,
    ManualRecoveryRequired = 3
};

enum class CommonPauseRuntimeState : uint8_t {
    None = 0,
    PauseArming = 1,
    PauseHold = 2,
    PauseEmpty = 3,
    ManualRecoveryRequired = 4
};

enum class CommonPauseNodeState : uint8_t {
    None = 0,
    Requested = 1,
    Preparing = 2,
    Ready = 3,
    BlockedFault = 4
};

enum class PauseCountSource : uint8_t {
    None = 0,
    TrustedWire = 1,
    TrustedWireUnknown = 2,
    SurrogateFeedFlags = 3,
    SurrogateSealerSignals = 4
};

enum class CommonAbortDrainOutcome : uint8_t {
    None = 0,
    InProgress = 1,
    ForcedStop = 2,
    DrainedToSafe = 3,
    LocalFault = 4,
    Timeout = 5,
    NoProgress = 6,
    LostSupervision = 7
};

struct CommonCycleState {
    bool active = false;
    bool pauseRequested = false;
    bool step3Seen = false;
    bool step3LaunchDone = false;
    bool parallelLaunchDone = false;
    bool refillStartPending = false;
    bool currentCycleLoadsSealer = false;
    bool outfeedStartIssued = false;
    bool outfeedNotReadyObserved = false;
    uint32_t cycleId = 0;
    uint8_t outfeedStep2SeqBase = 0;
    bool sealerDonePendingUnload = false;
    CommonPauseState pauseState = CommonPauseState::None;
    CommonCycleStage stage = CommonCycleStage::Idle;
    uint16_t lastConsumedBatchSeq = 0;
    uint32_t manipStarts = 0;
    uint32_t parallelStarts = 0;
    uint32_t refillStartRequestedMs = 0;
    uint32_t outfeedStartedMs = 0;
    uint32_t pauseStrictSnapshotSinceMs = 0;
    uint32_t pauseStrictSnapshotRetryMs = 0;
    uint8_t post7SealerCompletionSeqBase = 0;
    bool pauseStrictSnapshotForceDrain = false;
    bool pauseForceDrainApplied = false;
    bool pauseStrictSnapshotRawReadyIgnoredLogged = false;
    bool pauseStrictGateLatched = false;
    bool pauseStrictRollbackBlockedLogged = false;
    CommonRuntimeState runtimeState = CommonRuntimeState::Idle;
    CommonAbortDrainOutcome abortDrainOutcome = CommonAbortDrainOutcome::None;
    bool abortDrainActive = false;
    bool abortFsinvPending = false;
    bool abortRequiresManualRecovery = false;
    bool abortDrainLastOutfeedReady = false;
    bool abortDrainLastStep2Active = false;
    bool abortDrainLastVfdActive = false;
    uint32_t abortDrainStartedMs = 0;
    uint32_t abortDrainLastProgressMs = 0;
    uint32_t abortDrainLastWaitLogMs = 0;
    uint8_t abortDrainLastStep2Seq = 0;
    uint8_t abortDrainLastSealerSeq = 0;
    bool abortDrainSettleActive = false;
    uint32_t abortDrainSettleSinceMs = 0;
    uint8_t abortDrainSettleChecks = 0;
    String abortReason;
    String manipStartReason;
    String lastEvent = "idle";
};

struct CommonPauseSnapshot {
    uint32_t pauseEpoch = 0;
    uint32_t pauseCycleId = 0;
    CommonPauseRuntimeState pauseState = CommonPauseRuntimeState::None;
    CommonPauseNodeState nodeState = CommonPauseNodeState::None;
    bool nodeStateFresh = false;
    int16_t bufferCount = 0;
    bool bufferCountValid = false;
    int16_t sealerCount = 0;
    bool sealerCountValid = false;
    bool sealerResidualKnown = false;
    bool bufferCountFresh = false;
    bool sealerCountFresh = false;
    PauseCountSource bufferCountSource = PauseCountSource::None;
    PauseCountSource sealerCountSource = PauseCountSource::None;
    bool nodePauseEpochConfirmed = false;
    bool contractSnapshotTrusted = false;
    bool manualRecoveryRequired = false;
    bool progressObserved = false;
    uint32_t updatedMs = 0;
};

struct CommonPauseRuntime {
    CommonPauseRuntimeState state = CommonPauseRuntimeState::None;
    uint32_t currentEpoch = 0;
    uint32_t pauseCycleId = 0;
    uint32_t armingSinceMs = 0;
    uint32_t lastProgressMs = 0;
    uint32_t lastStatusPollMs = 0;
    bool progressObserved = false;
    CommonPauseState lastLegacyPauseState = CommonPauseState::None;
    CommonPauseNodeState lastConveyorNodeState = CommonPauseNodeState::None;
    int16_t lastBufferCount = 0;
    bool lastBufferCountValid = false;
    int16_t lastSealerCount = 0;
    bool lastSealerCountValid = false;
    PauseCountSource lastBufferCountSource = PauseCountSource::None;
    PauseCountSource lastSealerCountSource = PauseCountSource::None;
    bool armSentConveyor = false;
    bool armSentManipulator = false;
    bool releaseSentConveyor = false;
    bool releaseSentManipulator = false;
    bool lastNodePauseEpochConfirmed = false;
    bool nodeReadyConfirmed = false;
    bool finalizeStarted = false;
    bool mandatoryUnloadActive = false;
    uint32_t mandatoryUnloadSinceMs = 0;
    uint32_t mandatoryUnloadLastProgressMs = 0;
    uint32_t lastWatchdogWaitLogMs = 0;
    bool lastConveyorBusy = false;
    bool lastManipulatorBusy = false;
    uint8_t lastManipulatorWorkStep = 0;
    bool lastOutfeedReady = false;
    bool lastOutfeedStep2Active = false;
    bool lastOutfeedVfdActive = false;
    uint8_t lastOutfeedStep2Seq = 0;
    uint8_t lastOutfeedSealerSeq = 0;
    bool lastFillLastBlockCompleted = false;
    bool fillLastBlockCompletionLatched = false;
    uint32_t fillLastBlockBlockedSinceMs = 0;
    bool fillLastBlockBlockedLogged = false;
    bool watchdogPolicyKnown = false;
    bool lastWatchdogPolicyMandatoryUnload = false;
    bool lastWatchdogPolicyMandatoryInFlight = false;
    bool lastWatchdogPolicyForceDrainStableWait = false;
    CommonPauseSnapshot conveyorSnapshot = {};
    CommonPauseSnapshot manipulatorSnapshot = {};
};

struct I2cDiagState {
    bool active = false;
    uint32_t opSeq = 0;
    uint32_t activeSeq = 0;
    uint32_t activeStartedMs = 0;
    uint8_t activeId = 0;
    char activeOp[16] = "idle";
    char activeOrigin[24] = "-";
    char activeDetail[40] = "-";
    uint32_t lastCompletedMs = 0;
    uint32_t lastDurationMs = 0;
    uint32_t maxDurationMs = 0;
    uint32_t slowCount = 0;
    uint32_t failCount = 0;
    uint8_t lastId = 0;
    char lastOp[16] = "none";
    char lastOrigin[24] = "-";
    char lastDetail[40] = "-";
    char lastOutcome[24] = "none";
    uint32_t lastStartTraceMs = 0;
    uint32_t guardTripCount = 0;
    uint32_t lastGuardSeq = 0;
    uint32_t lastGuardMs = 0;
};

struct LoopDiagState {
    uint32_t tickSeq = 0;
    uint32_t lastTickStartedMs = 0;
    uint32_t lastTickCompletedMs = 0;
    uint32_t maxTickDurationMs = 0;
    const char *currentStage = "boot";
    uint32_t currentStageSinceMs = 0;
};

struct ScanDiagState {
    bool active = false;
    uint8_t activeId = 0;
    char activeOrigin[24] = "-";
    uint32_t activeStartedMs = 0;
    uint8_t lastId = 0;
    char lastOrigin[24] = "-";
    char lastResult[24] = "none";
    uint8_t lastException = 0;
    uint32_t lastStartedMs = 0;
    uint32_t lastFinishedMs = 0;
    uint32_t lastDurationMs = 0;
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
bool g_sealRemoteObserved = false;
uint32_t g_sealDoneLastRiseMs = 0;
uint32_t g_sealStartPulseDurationMs = SEAL_START_PULSE_MS_DEFAULT;
uint8_t g_sealCompletionSeqLast = 0;
OtvodWorkCycleState g_otvodWorkCycle;
CommonCycleState g_commonCycle;
CommonPauseRuntime g_pauseRuntime;
uint32_t g_commonCycleIdCounter = 0;
CommonCycleStage g_commonDiagLastStage = CommonCycleStage::Idle;
uint32_t g_commonDiagStageSinceMs = 0;
uint32_t g_commonDiagLastWaitLogMs = 0;
I2cDiagState g_i2cDiag;
LoopDiagState g_loopDiag;
ScanDiagState g_scanDiag;
uint32_t g_i2cLastRecoverMs = 0;
uint32_t g_loopHeartbeatLastMs = 0;

bool mqttPublishStatus(bool retained);
bool i2cSendManagedDeviceCommand(uint8_t id, const String &command, const char *origin = "unspecified");
void i2cDiagFinish(bool ok, const char *outcome);
void i2cRecoverBus(const char *reason);
bool startOtvodWorkCycle(uint8_t totalCycles = OTVOD_WORK_TOTAL_CYCLES,
                         uint32_t stepSteps = OTVOD_WORK_STEP_STEPS,
                         float legacyVfdDistanceCm = 0.0F);
bool startCommonConveyorProgram1(const String &reason);
bool commonPauseRuntimeCanRelease();
bool commonPauseFillLastBlockCompleted(const Rs485DeviceState &conveyor,
                                       const Rs485DeviceState &manipulator);
bool commonPauseContractDrainIdle(const Rs485DeviceState &conveyor,
                                  const Rs485DeviceState &manipulator,
                                  uint32_t nowMs);
const char *commonPauseRuntimeSubstateName(const Rs485DeviceState &conveyor,
                                           const Rs485DeviceState &manipulator);
bool commonPauseRuntimeMandatoryUnloadInFlight(const Rs485DeviceState &conveyor,
                                               const Rs485DeviceState &manipulator);
bool commonPauseRuntimeMandatoryUnloadPolicyActive();
bool commonPauseRuntimeForceDrainStableWaitPolicyActive();
bool commonPauseRuntimeWatchdogActive();
uint32_t commonPauseRuntimeMandatoryUnloadElapsedMs(uint32_t nowMs);
const char *commonPauseRuntimeWatchdogProfileName(bool mandatoryUnloadPolicyActive);
const char *commonPauseRuntimeWatchdogPolicyReason(bool mandatoryUnloadPolicyActive,
                                                   bool mandatoryUnloadInFlight);
void commonSetEventIfChanged(const String &eventText);
void commonPauseRuntimeMarkMandatoryUnloadStarted(uint32_t nowMs);
void commonPauseRuntimeClearMandatoryUnloadTracking();
void commonPauseRuntimeRestartArmingWindow(uint32_t nowMs, const char *reason);
uint32_t commonPauseRuntimeArmingElapsedMs(uint32_t nowMs);
uint32_t commonPauseRuntimeNoProgressMs(uint32_t nowMs);

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

const char *commonPauseStateName(CommonPauseState state)
{
    switch (state) {
        case CommonPauseState::Requested:
            return "pause_requested";
        case CommonPauseState::FillLastBlock:
            return "pause_fill_last_block";
        case CommonPauseState::StableWait:
            return "pause_stable_wait";
        case CommonPauseState::WaitStrictFeedSnapshot:
            return "pause_wait_strict_snapshot";
        case CommonPauseState::None:
        default:
            return "none";
    }
}

const char *commonRuntimeStateName(CommonRuntimeState state)
{
    switch (state) {
        case CommonRuntimeState::Active:
            return "active";
        case CommonRuntimeState::CommonAborted:
            return "common_aborted";
        case CommonRuntimeState::ManualRecoveryRequired:
            return "manual_recovery_required";
        case CommonRuntimeState::Idle:
        default:
            return "idle";
    }
}

const char *commonPauseRuntimeStateName(CommonPauseRuntimeState state)
{
    switch (state) {
        case CommonPauseRuntimeState::PauseArming:
            return "pause_arming";
        case CommonPauseRuntimeState::PauseHold:
            return "pause_hold";
        case CommonPauseRuntimeState::PauseEmpty:
            return "pause_empty";
        case CommonPauseRuntimeState::ManualRecoveryRequired:
            return "manual_recovery_required";
        case CommonPauseRuntimeState::None:
        default:
            return "none";
    }
}

const char *commonPauseNodeStateName(CommonPauseNodeState state)
{
    switch (state) {
        case CommonPauseNodeState::Requested:
            return "requested";
        case CommonPauseNodeState::Preparing:
            return "preparing";
        case CommonPauseNodeState::Ready:
            return "ready";
        case CommonPauseNodeState::BlockedFault:
            return "blocked";
        case CommonPauseNodeState::None:
        default:
            return "none";
    }
}

const char *pauseCountSourceName(PauseCountSource source)
{
    switch (source) {
        case PauseCountSource::TrustedWire:
            return "wire_trusted";
        case PauseCountSource::TrustedWireUnknown:
            return "wire_unknown";
        case PauseCountSource::SurrogateFeedFlags:
            return "surrogate_feed_flags";
        case PauseCountSource::SurrogateSealerSignals:
            return "surrogate_sealer_signals";
        case PauseCountSource::None:
        default:
            return "none";
    }
}

const char *commonAbortDrainOutcomeName(CommonAbortDrainOutcome outcome)
{
    switch (outcome) {
        case CommonAbortDrainOutcome::InProgress:
            return "in_progress";
        case CommonAbortDrainOutcome::ForcedStop:
            return "forced_stop";
        case CommonAbortDrainOutcome::DrainedToSafe:
            return "drained_to_safe";
        case CommonAbortDrainOutcome::LocalFault:
            return "local_fault";
        case CommonAbortDrainOutcome::Timeout:
            return "timeout";
        case CommonAbortDrainOutcome::NoProgress:
            return "no_progress";
        case CommonAbortDrainOutcome::LostSupervision:
            return "lost_supervision";
        case CommonAbortDrainOutcome::None:
        default:
            return "none";
    }
}

const char *commonCoarseStateName()
{
    if (g_commonCycle.abortDrainActive) {
        return "post7_draining_to_safe";
    }
    switch (g_commonCycle.runtimeState) {
        case CommonRuntimeState::Active:
            return "common_active";
        case CommonRuntimeState::CommonAborted:
            return "common_aborted";
        case CommonRuntimeState::ManualRecoveryRequired:
            return "manual_recovery_required";
        case CommonRuntimeState::Idle:
        default:
            return "idle";
    }
}

bool commonManualRecoveryRequired()
{
    return g_commonCycle.runtimeState == CommonRuntimeState::ManualRecoveryRequired ||
        g_commonCycle.abortRequiresManualRecovery;
}

void copyDiagText(char *dst, size_t dstSize, const char *src)
{
    if (dst == nullptr || dstSize == 0) {
        return;
    }
    const char *safe = (src == nullptr || src[0] == '\0') ? "-" : src;
    strncpy(dst, safe, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

void loopMarkStage(const char *stage)
{
    const char *safe = (stage == nullptr || stage[0] == '\0') ? "unknown" : stage;
    if (strcmp(g_loopDiag.currentStage, safe) == 0) {
        return;
    }

    g_loopDiag.currentStage = safe;
    g_loopDiag.currentStageSinceMs = millis();
}

bool i2cWireTraceShouldPrint(bool force)
{
    if (force || I2C_WIRE_TRACE_SAMPLE_EVERY <= 1U) {
        return true;
    }
    return (g_i2cDiag.activeSeq % I2C_WIRE_TRACE_SAMPLE_EVERY) == 0U;
}

void i2cTraceWireStep(uint8_t id,
                      const char *origin,
                      const char *phase,
                      int32_t value = -1,
                      bool force = false)
{
    if (!i2cWireTraceShouldPrint(force)) {
        return;
    }

    const char *safeOrigin = (origin == nullptr || origin[0] == '\0') ? "-" : origin;
    const char *safePhase = (phase == nullptr || phase[0] == '\0') ? "unknown" : phase;

    Serial.print("I2C TRACE: wire seq=");
    Serial.print(g_i2cDiag.activeSeq);
    Serial.print(", id=");
    Serial.print(id);
    Serial.print(", origin=");
    Serial.print(safeOrigin);
    Serial.print(", phase=");
    Serial.print(safePhase);
    if (value >= 0) {
        Serial.print(", value=");
        Serial.print(value);
    }
    Serial.println();
}

void scanDiagStart(uint8_t id, const char *origin)
{
    ScanDiagState &diag = g_scanDiag;
    const uint32_t now = millis();
    diag.active = true;
    diag.activeId = id;
    diag.activeStartedMs = now;
    diag.lastStartedMs = now;
    copyDiagText(diag.activeOrigin, sizeof(diag.activeOrigin), origin);
}

void scanDiagFinish(uint8_t id, MbResult result, uint8_t exceptionCode)
{
    ScanDiagState &diag = g_scanDiag;
    const uint32_t now = millis();
    const uint32_t elapsedMs = diag.active
        ? static_cast<uint32_t>(now - diag.activeStartedMs)
        : 0U;

    diag.active = false;
    diag.lastId = id;
    diag.lastException = exceptionCode;
    diag.lastFinishedMs = now;
    diag.lastDurationMs = elapsedMs;
    copyDiagText(diag.lastOrigin, sizeof(diag.lastOrigin), diag.activeOrigin);
    copyDiagText(diag.lastResult, sizeof(diag.lastResult), mbResultCode(result));
    if (result == MbResult::Exception) {
        snprintf(diag.lastResult, sizeof(diag.lastResult), "exception_0x%02X", exceptionCode);
    }
}

void printScanDiagStatusLine()
{
    const uint32_t now = millis();
    Serial.print("MBSCAN diag: enabled=");
    Serial.print(g_rs485ScanEnabled ? "yes" : "no");
    Serial.print(", last_id=");
    Serial.print(g_scanDiag.lastId);
    Serial.print(", last_origin=");
    Serial.print(g_scanDiag.lastOrigin);
    Serial.print(", last_result=");
    Serial.print(g_scanDiag.lastResult);
    Serial.print(", last_ms=");
    Serial.print(g_scanDiag.lastDurationMs);
    Serial.print(", active=");
    Serial.print(g_scanDiag.active ? "yes" : "no");
    if (g_scanDiag.active) {
        Serial.print(", active_id=");
        Serial.print(g_scanDiag.activeId);
        Serial.print(", active_origin=");
        Serial.print(g_scanDiag.activeOrigin);
        Serial.print(", active_ms=");
        Serial.print(static_cast<uint32_t>(now - g_scanDiag.activeStartedMs));
    }
    Serial.print(", i2c_unfinished=");
    Serial.print(g_i2cDiag.active ? "yes" : "no");
    Serial.print(", i2c_op=");
    Serial.print(g_i2cDiag.active ? g_i2cDiag.activeOp : g_i2cDiag.lastOp);
    Serial.print("@");
    Serial.print(g_i2cDiag.active ? g_i2cDiag.activeOrigin : g_i2cDiag.lastOrigin);
    Serial.println();
}

void i2cCheckActiveGuard()
{
    if (!g_i2cDiag.active) {
        return;
    }

    const uint32_t now = millis();
    const uint32_t activeMs = static_cast<uint32_t>(now - g_i2cDiag.activeStartedMs);
    if (activeMs < I2C_ACTIVE_GUARD_MS) {
        return;
    }

    if (g_i2cDiag.lastGuardSeq == g_i2cDiag.activeSeq) {
        return;
    }

    g_i2cDiag.lastGuardSeq = g_i2cDiag.activeSeq;
    g_i2cDiag.lastGuardMs = now;
    g_i2cDiag.guardTripCount++;

    Serial.print("I2C GUARD: active op timeout, seq=");
    Serial.print(g_i2cDiag.activeSeq);
    Serial.print(", op=");
    Serial.print(g_i2cDiag.activeOp);
    Serial.print(", id=");
    Serial.print(g_i2cDiag.activeId);
    Serial.print(", origin=");
    Serial.print(g_i2cDiag.activeOrigin);
    Serial.print(", ms=");
    Serial.println(activeMs);

    i2cDiagFinish(false, "guard_timeout");
    i2cRecoverBus("active op guard timeout");
}

void loopPrintHeartbeat()
{
    const uint32_t now = millis();
    if ((uint32_t)(now - g_loopHeartbeatLastMs) < LOOP_HEARTBEAT_INTERVAL_MS) {
        return;
    }
    g_loopHeartbeatLastMs = now;

    Serial.print("HEARTBEAT: stage=");
    Serial.print(g_loopDiag.currentStage);
    Serial.print(", stage_ms=");
    Serial.print(static_cast<uint32_t>(now - g_loopDiag.currentStageSinceMs));
    Serial.print(", common=");
    Serial.print(commonCycleStageName(g_commonCycle.stage));
    Serial.print(", common_coarse=");
    Serial.print(commonCoarseStateName());
    Serial.print(", i2c_seq=");
    Serial.print(g_i2cDiag.opSeq);
    Serial.print(", i2c_active=");
    Serial.print(g_i2cDiag.active ? "yes" : "no");
    if (g_i2cDiag.active) {
        Serial.print(", i2c_active_ms=");
        Serial.print(static_cast<uint32_t>(now - g_i2cDiag.activeStartedMs));
        Serial.print(", i2c_active_op=");
        Serial.print(g_i2cDiag.activeOp);
        Serial.print("@");
        Serial.print(g_i2cDiag.activeOrigin);
    } else {
        Serial.print(", i2c_last=");
        Serial.print(g_i2cDiag.lastOp);
        Serial.print("@");
        Serial.print(g_i2cDiag.lastOrigin);
        Serial.print("/");
        Serial.print(g_i2cDiag.lastOutcome);
    }
    Serial.println();
}

void i2cDiagStart(uint8_t id, const char *op, const char *origin, const char *detail)
{
    I2cDiagState &diag = g_i2cDiag;
    const uint32_t now = millis();

    diag.active = true;
    diag.activeSeq = ++diag.opSeq;
    diag.activeStartedMs = now;
    diag.activeId = id;
    copyDiagText(diag.activeOp, sizeof(diag.activeOp), op);
    copyDiagText(diag.activeOrigin, sizeof(diag.activeOrigin), origin);
    copyDiagText(diag.activeDetail, sizeof(diag.activeDetail), detail);

    const bool sameContext =
        strcmp(diag.lastOp, diag.activeOp) == 0 &&
        strcmp(diag.lastOrigin, diag.activeOrigin) == 0 &&
        strcmp(diag.lastDetail, diag.activeDetail) == 0;
    if (sameContext && (uint32_t)(now - diag.lastStartTraceMs) < I2C_START_TRACE_THROTTLE_MS) {
        return;
    }

    diag.lastStartTraceMs = now;
    Serial.print("I2C TRACE: start seq=");
    Serial.print(diag.activeSeq);
    Serial.print(", op=");
    Serial.print(diag.activeOp);
    Serial.print(", id=");
    Serial.print(id);
    Serial.print(", origin=");
    Serial.print(diag.activeOrigin);
    Serial.print(", detail=");
    Serial.println(diag.activeDetail);
}

void i2cDiagFinish(bool ok, const char *outcome)
{
    I2cDiagState &diag = g_i2cDiag;
    if (!diag.active) {
        return;
    }

    const uint32_t now = millis();
    const uint32_t elapsedMs = static_cast<uint32_t>(now - diag.activeStartedMs);
    diag.active = false;
    diag.lastCompletedMs = now;
    diag.lastDurationMs = elapsedMs;
    if (elapsedMs > diag.maxDurationMs) {
        diag.maxDurationMs = elapsedMs;
    }
    if (!ok) {
        diag.failCount++;
    }
    if (elapsedMs >= I2C_WARN_LATENCY_MS) {
        diag.slowCount++;
    }

    diag.lastId = diag.activeId;
    copyDiagText(diag.lastOp, sizeof(diag.lastOp), diag.activeOp);
    copyDiagText(diag.lastOrigin, sizeof(diag.lastOrigin), diag.activeOrigin);
    copyDiagText(diag.lastDetail, sizeof(diag.lastDetail), diag.activeDetail);
    copyDiagText(diag.lastOutcome, sizeof(diag.lastOutcome), outcome);

    if (ok && elapsedMs < I2C_WARN_LATENCY_MS) {
        return;
    }

    Serial.print("I2C TRACE: done seq=");
    Serial.print(diag.activeSeq);
    Serial.print(", op=");
    Serial.print(diag.lastOp);
    Serial.print(", id=");
    Serial.print(diag.lastId);
    Serial.print(", result=");
    Serial.print(diag.lastOutcome);
    Serial.print(", ms=");
    Serial.print(elapsedMs);
    Serial.print(", loop_stage=");
    Serial.print(g_loopDiag.currentStage);
    Serial.print(", common_stage=");
    Serial.print(commonCycleStageName(g_commonCycle.stage));
    Serial.print(", pause=");
    Serial.println(commonPauseStateName(g_commonCycle.pauseState));
}

void i2cRecoverBus(const char *reason)
{
    const uint32_t now = millis();
    if ((uint32_t)(now - g_i2cLastRecoverMs) < I2C_RECOVER_COOLDOWN_MS) {
        return;
    }
    g_i2cLastRecoverMs = now;

    Serial.print("I2C RECOVER: ");
    Serial.print((reason == nullptr || reason[0] == '\0') ? "reason=unknown" : reason);
    Serial.print(", last=");
    Serial.print(g_i2cDiag.lastOp);
    Serial.print("@");
    Serial.print(g_i2cDiag.lastOrigin);
    Serial.println(".");

    Wire.end();
    delay(2);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
}

bool deviceStatusBusy(const Rs485DeviceState &st)
{
    return (st.statusWord & DEVICE_STATUS_BUSY_BIT) != 0;
}

bool deviceStatusAlarm(const Rs485DeviceState &st)
{
    return (st.statusWord & DEVICE_STATUS_ALARM_BIT) != 0;
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
    return static_cast<uint8_t>(st.extra0 & CONVEYOR_EXTRA0_FLAG_MASK);
}

uint8_t conveyorMotionState(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.extra0 >> CONVEYOR_EXTRA0_MOTION_SHIFT) &
                                CONVEYOR_EXTRA0_MOTION_MASK);
}

bool conveyorVfdTimedRunActive(const Rs485DeviceState &st)
{
    return (st.extra0 & CONVEYOR_EXTRA0_VFD_TIMED_RUN_ACTIVE_BIT) != 0U;
}

uint32_t conveyorVfdTickDurationMs(const Rs485DeviceState &st)
{
    return static_cast<uint32_t>(st.extra1) * 10U;
}

bool conveyorSealerBusy(const Rs485DeviceState &st)
{
    return (st.extra0 & CONVEYOR_EXTRA0_SEALER_BUSY_BIT) != 0U;
}

bool conveyorSealerStartPulseActive(const Rs485DeviceState &st)
{
    return (st.extra0 & CONVEYOR_EXTRA0_SEALER_START_PULSE_BIT) != 0U;
}

bool conveyorSealerDoneActive(const Rs485DeviceState &st)
{
    return (st.extra0 & CONVEYOR_EXTRA0_SEALER_DONE_ACTIVE_BIT) != 0U;
}

uint8_t conveyorSealerCompletionSeq(const Rs485DeviceState &st)
{
    const uint8_t low = static_cast<uint8_t>((st.extra0 >> CONVEYOR_EXTRA0_SEALER_SEQ_LOW_SHIFT) &
                                             CONVEYOR_EXTRA0_SEALER_SEQ_LOW_MASK);
    const uint8_t high = static_cast<uint8_t>((st.extra0 >> CONVEYOR_EXTRA0_SEALER_SEQ_HIGH_SHIFT) &
                                              CONVEYOR_EXTRA0_SEALER_SEQ_HIGH_MASK);
    return static_cast<uint8_t>(low | (high << 6));
}

bool conveyorStep2Active(const Rs485DeviceState &st)
{
    return (st.statusWord & CONVEYOR_STATUS_STEP2_ACTIVE_BIT) != 0;
}

bool conveyorOutfeedReadyForBatch(const Rs485DeviceState &st)
{
    // COM10 production contract: true только когда outfeed полностью settled для COMMON.
    return (st.statusWord & CONVEYOR_STATUS_OUTFEED_READY_FOR_BATCH_BIT) != 0;
}

bool conveyorPost7ManualRecoveryRequired(const Rs485DeviceState &st)
{
    return (st.errorWord & CONVEYOR_ERROR_POST7_MANUAL_RECOVERY_REQUIRED_BIT) != 0U;
}

uint8_t conveyorStep2CompletionSeq(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.statusWord >> 8) & 0x007FU);
}

uint16_t conveyorBatchSeq(const Rs485DeviceState &st)
{
    return static_cast<uint16_t>(st.extra3 & 0x7FFFU);
}

bool conveyorNextBatchReadyForCommon(const Rs485DeviceState &st, uint16_t lastConsumedBatchSeq)
{
    const uint16_t seq = conveyorBatchSeq(st);
    return conveyorBatchReady(st) &&
           seq != 0U &&
           seq != lastConsumedBatchSeq;
}

bool conveyorReadyBatchIsStaleForCommon(const Rs485DeviceState &st, uint16_t lastConsumedBatchSeq)
{
    const uint16_t seq = conveyorBatchSeq(st);
    return conveyorBatchReady(st) &&
           seq != 0U &&
           seq == lastConsumedBatchSeq;
}

uint8_t conveyorProgramStateCode(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>(st.extra2 & 0x00FFU);
}

uint8_t conveyorProgramPass(const Rs485DeviceState &st)
{
    return static_cast<uint8_t>((st.extra2 >> 8) & CONVEYOR_EXTRA2_PROGRAM_PASS_MASK);
}

bool conveyorFeedSideEmptyStrict(const Rs485DeviceState &st)
{
    return (st.extra2 & CONVEYOR_EXTRA2_FEED_SIDE_EMPTY_STRICT_BIT) != 0U;
}

bool conveyorFeedSideEmptyValid(const Rs485DeviceState &st)
{
    return (st.extra2 & CONVEYOR_EXTRA2_FEED_SIDE_EMPTY_VALID_BIT) != 0U;
}

bool conveyorPauseHeartbeatExtValid(uint32_t hb)
{
    return (hb & CONVEYOR_HEARTBEAT_PAUSE_EXT_VALID_BIT) != 0U;
}

bool conveyorPauseStatusAckValid(const Rs485DeviceState &st)
{
    return (st.errorWord & CONVEYOR_ERROR_PAUSE_STATUS_ACK_VALID_BIT) != 0U;
}

uint16_t conveyorPauseStatusAckEpoch(const Rs485DeviceState &st)
{
    return static_cast<uint16_t>((st.errorWord >> 4) & 0x0FFFU);
}

bool manipulatorPauseStatusAckValid(const Rs485DeviceState &st)
{
    return (st.errorWord & MANIPULATOR_ERROR_PAUSE_STATUS_ACK_VALID_BIT) != 0U;
}

uint16_t manipulatorPauseStatusAckEpoch(const Rs485DeviceState &st)
{
    return static_cast<uint16_t>((st.errorWord >> 6) & 0x03FFU);
}

void conveyorUnpackPauseHeartbeat(uint32_t hb,
                                  int16_t &bufferCount,
                                  bool &bufferCountValid,
                                  int16_t &sealerCount,
                                  bool &sealerCountValid,
                                  bool &residualKnown)
{
    const uint8_t wireBuffer = static_cast<uint8_t>((hb >> 20) & 0x0FU);
    const uint8_t wireSealer = static_cast<uint8_t>((hb >> 24) & 0x0FU);
    bufferCountValid = wireBuffer != CONVEYOR_HEARTBEAT_PAUSE_COUNT_UNKNOWN;
    sealerCountValid = wireSealer != CONVEYOR_HEARTBEAT_PAUSE_COUNT_UNKNOWN;
    bufferCount = bufferCountValid ? static_cast<int16_t>(wireBuffer) : -1;
    sealerCount = sealerCountValid ? static_cast<int16_t>(wireSealer) : -1;
    residualKnown = (hb & CONVEYOR_HEARTBEAT_RESIDUAL_KNOWN_BIT) != 0U;
}

CommonPauseNodeState conveyorPauseHeartbeatNodeState(uint32_t hb)
{
    if (!conveyorPauseHeartbeatExtValid(hb)) {
        return CommonPauseNodeState::None;
    }

    switch ((hb >> CONVEYOR_HEARTBEAT_PAUSE_STATE_SHIFT) & 0x03U) {
        case 0U:
            return CommonPauseNodeState::Requested;
        case 1U:
            return CommonPauseNodeState::Preparing;
        case 2U:
            return CommonPauseNodeState::Ready;
        case 3U:
            return CommonPauseNodeState::BlockedFault;
        default:
            return CommonPauseNodeState::None;
    }
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

bool manipulatorStep3Ready(const Rs485DeviceState &st)
{
    return (st.statusWord & MANIPULATOR_STATUS_STEP3_READY_BIT) != 0;
}

bool manipulatorInWorkStartPose(const Rs485DeviceState &st)
{
    return (st.statusWord & MANIPULATOR_STATUS_WORK_START_READY_BIT) != 0;
}

bool manipulatorNeedsOnlyGripOpenForWorkStart(const Rs485DeviceState &st)
{
    return (st.statusWord & MANIPULATOR_STATUS_NEEDS_GRIP_OPEN_ONLY_BIT) != 0;
}

bool manipulatorWorkStartGripUnknown(const Rs485DeviceState &st)
{
    const uint16_t sensorBits = manipulatorSensorBits(st);
    const bool rightAndZUp =
        (sensorBits & (MANIPULATOR_SENSOR_RIGHT_LIMIT_BIT | MANIPULATOR_SENSOR_Z_UP_BIT)) ==
        (MANIPULATOR_SENSOR_RIGHT_LIMIT_BIT | MANIPULATOR_SENSOR_Z_UP_BIT);
    const bool gripKnown =
        (sensorBits & (MANIPULATOR_SENSOR_GRIP_OPEN_BIT | MANIPULATOR_SENSOR_GRIP_CLOSED_BIT)) != 0U;

    return rightAndZUp &&
        !gripKnown &&
        !manipulatorInWorkStartPose(st) &&
        !manipulatorNeedsOnlyGripOpenForWorkStart(st);
}

bool conveyorSealerOnline()
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    return conveyor.online && conveyor.protocolOk;
}

void printSealStatus()
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    Serial.print("SEAL: start_out=");
    Serial.print(conveyorSealerStartPulseActive(conveyor) ? "on" : "off");
    Serial.print(", done=");
    Serial.print(conveyorSealerDoneActive(conveyor) ? "active" : "inactive");
    Serial.print(", busy=");
    Serial.print(conveyorSealerBusy(conveyor) ? "yes" : "no");
    Serial.print(", owner=conveyor(12)");
    Serial.print(", online=");
    Serial.print(conveyorSealerOnline() ? "yes" : "no");
    Serial.print(", pulse_ms=");
    Serial.println(g_sealStartPulseDurationMs);
}

void processSealIo()
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    if (!conveyor.online || !conveyor.protocolOk) {
        g_sealStartPulseActive = false;
        g_sealStartOutputActive = false;
        g_sealDoneLastActive = false;
        g_sealRemoteObserved = false;
        return;
    }

    const bool doneActive = conveyorSealerDoneActive(conveyor);
    const bool startPulseActive = conveyorSealerStartPulseActive(conveyor);
    const uint8_t completionSeq = conveyorSealerCompletionSeq(conveyor);
    g_sealStartPulseActive = startPulseActive;
    g_sealStartOutputActive = startPulseActive;

    if (!g_sealRemoteObserved) {
        g_sealRemoteObserved = true;
        g_sealDoneLastActive = doneActive;
        g_sealCompletionSeqLast = completionSeq;
        return;
    }

    bool changed = false;
    if (completionSeq != g_sealCompletionSeqLast) {
        g_sealCompletionSeqLast = completionSeq;
        g_sealDoneLastRiseMs = millis();
        g_commonCycle.sealerDonePendingUnload = true;
        Serial.println("SEAL: conveyor cycle complete latched.");
        g_lastCommandResult = "SEAL cycle completed";
        changed = true;
    }

    if (doneActive != g_sealDoneLastActive) {
        g_sealDoneLastActive = doneActive;
        if (doneActive) {
            Serial.println("SEAL: cycle complete input became active.");
        } else {
            Serial.println("SEAL: cycle complete input released.");
        }
        changed = true;
    }

    if (changed) {
        (void)mqttPublishStatus(false);
    }
}

void handleCommandSeal(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        Serial.println("SEAL commands:");
        Serial.println("  SEAL START [ms]   - send START pulse to conveyor sealer (default 300 ms)");
        Serial.println("  SEAL STATUS       - print conveyor START/DONE state");
        Serial.println("  SEAL OUT ON       - force conveyor START output ON for wiring test");
        Serial.println("  SEAL OUT OFF      - force conveyor START output OFF");
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
            if (!conveyorSealerOnline()) {
                g_lastCommandResult = "SEAL OUT failed: conveyor offline";
                Serial.println(g_lastCommandResult);
                return;
            }
            if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "SEAL OUT ON", "seal_cmd_out_on")) {
                g_lastCommandResult = "SEAL OUT failed: send error";
                Serial.println(g_lastCommandResult);
                return;
            }
            g_lastCommandResult = "SEAL OUT ON";
            Serial.println(g_lastCommandResult);
            return;
        }
        if (mode == "OFF") {
            if (!conveyorSealerOnline()) {
                g_lastCommandResult = "SEAL OUT failed: conveyor offline";
                Serial.println(g_lastCommandResult);
                return;
            }
            if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "SEAL OUT OFF", "seal_cmd_out_off")) {
                g_lastCommandResult = "SEAL OUT failed: send error";
                Serial.println(g_lastCommandResult);
                return;
            }
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

        const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
        if (!conveyor.online || !conveyor.protocolOk) {
            g_lastCommandResult = "SEAL START failed: conveyor offline";
            Serial.println(g_lastCommandResult);
            return;
        }
        if (conveyorSealerBusy(conveyor)) {
            g_lastCommandResult = "SEAL START ignored: pulse already active";
            Serial.println(g_lastCommandResult);
            return;
        }

        g_sealStartPulseDurationMs = pulseMs;
        if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "SEAL START " + String(pulseMs), "seal_cmd_start")) {
            g_lastCommandResult = "SEAL START failed: send error";
            Serial.println(g_lastCommandResult);
            return;
        }
        Serial.print("SEAL START: pulse on conveyor for ");
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
    const bool conveyorHasReadyBatch = conveyorBatchReady(conveyor);
    const uint16_t conveyorReadySeq = conveyorBatchSeq(conveyor);
    const bool conveyorNextBatchReady = conveyorNextBatchReadyForCommon(
        conveyor,
        g_commonCycle.lastConsumedBatchSeq);
    const bool conveyorReadyBatchStale = conveyorReadyBatchIsStaleForCommon(
        conveyor,
        g_commonCycle.lastConsumedBatchSeq);
    const uint32_t nowMs = millis();
    const uint32_t sealAgeMs = (g_sealDoneLastRiseMs == 0)
        ? 0
        : static_cast<uint32_t>(nowMs - g_sealDoneLastRiseMs);
    const uint32_t drainElapsedMs = g_commonCycle.abortDrainActive
        ? static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainStartedMs)
        : 0U;
    const uint32_t drainNoProgressMs = g_commonCycle.abortDrainActive
        ? static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainLastProgressMs)
        : 0U;
    const bool pauseMandatoryUnloadInFlight =
        commonPauseRuntimeMandatoryUnloadInFlight(conveyor, manipulator);
    const bool pauseMandatoryUnloadPolicyActive =
        commonPauseRuntimeMandatoryUnloadPolicyActive();
    const bool pauseForceDrainStableWaitPolicyActive =
        commonPauseRuntimeForceDrainStableWaitPolicyActive();
    const uint32_t pauseMandatoryUnloadElapsedMs =
        commonPauseRuntimeMandatoryUnloadElapsedMs(nowMs);
    const char *pauseWatchdogProfile = commonPauseRuntimeWatchdogProfileName(
        pauseMandatoryUnloadPolicyActive);
    const char *pauseWatchdogPolicyReason = commonPauseRuntimeWatchdogPolicyReason(
        pauseMandatoryUnloadPolicyActive,
        pauseMandatoryUnloadInFlight);

    Serial.print("COMMON: active=");
    Serial.print(g_commonCycle.active ? "yes" : "no");
    Serial.print(", coarse_state=");
    Serial.print(commonCoarseStateName());
    Serial.print(", runtime_state=");
    Serial.print(commonRuntimeStateName(g_commonCycle.runtimeState));
    Serial.print(", manual_recovery_required=");
    Serial.print(commonManualRecoveryRequired() ? "yes" : "no");
    Serial.print(", pause=");
    Serial.print(g_commonCycle.pauseRequested ? "yes" : "no");
    Serial.print(", pause_state=");
    Serial.print(commonPauseRuntimeStateName(g_pauseRuntime.state));
    Serial.print(", pause_legacy_state=");
    Serial.print(commonPauseStateName(g_commonCycle.pauseState));
    Serial.print(", pause_runtime=");
    Serial.print(commonPauseRuntimeStateName(g_pauseRuntime.state));
    Serial.print(", pause_epoch=");
    Serial.print(g_pauseRuntime.currentEpoch);
    Serial.print(", pause_cycle_id=");
    Serial.print(g_pauseRuntime.pauseCycleId);
    Serial.print(", pause_can_release=");
    Serial.print(commonPauseRuntimeCanRelease() ? "yes" : "no");
    Serial.print(", pause_arming_elapsed_ms=");
    Serial.print(commonPauseRuntimeArmingElapsedMs(nowMs));
    Serial.print(", pause_no_progress_ms=");
    Serial.print(commonPauseRuntimeNoProgressMs(nowMs));
    Serial.print(", pause_arming_timeout_limit_ms=");
    Serial.print(pauseMandatoryUnloadPolicyActive
        ? COMMON_PAUSE_MANDATORY_UNLOAD_TIMEOUT_MS
        : COMMON_PAUSE_ARMING_TIMEOUT_MS);
    Serial.print(", pause_watchdog_active=");
    Serial.print(commonPauseRuntimeWatchdogActive() ? "yes" : "no");
    Serial.print(", pause_watchdog_profile=");
    Serial.print(pauseWatchdogProfile);
    Serial.print(", pause_substate=");
    Serial.print(commonPauseRuntimeSubstateName(conveyor, manipulator));
    Serial.print(", pause_watchdog_policy_reason=");
    Serial.print(pauseWatchdogPolicyReason);
    Serial.print(", pause_force_drain_stable_wait_policy=");
    Serial.print(pauseForceDrainStableWaitPolicyActive ? "yes" : "no");
    Serial.print(", pause_no_progress_guard=");
    Serial.print(pauseMandatoryUnloadPolicyActive ? "hold" : "normal");
    Serial.print(", pause_mandatory_unload_active=");
    Serial.print(g_pauseRuntime.mandatoryUnloadActive ? "yes" : "no");
    Serial.print(", pause_mandatory_unload_in_flight=");
    Serial.print(pauseMandatoryUnloadInFlight ? "yes" : "no");
    Serial.print(", pause_mandatory_unload_elapsed_ms=");
    Serial.print(pauseMandatoryUnloadElapsedMs);
    Serial.print(", buffer_count=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCount);
    Serial.print(", buffer_count_valid=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "yes" : "no");
    Serial.print(", buffer_count_source=");
    Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
    Serial.print(", sealer_count=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCount);
    Serial.print(", sealer_count_valid=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "yes" : "no");
    Serial.print(", sealer_count_source=");
    Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
    Serial.print(", sealer_residual_known=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.sealerResidualKnown ? "yes" : "no");
    Serial.print(", buffer_count_fresh=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountFresh ? "yes" : "no");
    Serial.print(", sealer_count_fresh=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountFresh ? "yes" : "no");
    Serial.print(", pause_node_epoch_confirmed=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "yes" : "no");
    Serial.print(", pause_contract_trusted=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.contractSnapshotTrusted ? "yes" : "no");
    Serial.print(", pause_node_state=");
    Serial.print(commonPauseNodeStateName(g_pauseRuntime.conveyorSnapshot.nodeState));
    Serial.print(", pause_node_state_fresh=");
    Serial.print(g_pauseRuntime.conveyorSnapshot.nodeStateFresh ? "yes" : "no");
    Serial.print(", pause_node_ready_confirmed=");
    Serial.print(g_pauseRuntime.nodeReadyConfirmed ? "yes" : "no");
    Serial.print(", pause_finalize_started=");
    Serial.print(g_pauseRuntime.finalizeStarted ? "yes" : "no");
    const bool pauseFillLastBlockBlocked =
        g_pauseRuntime.fillLastBlockBlockedSinceMs != 0;
    Serial.print(", pause_fill_last_block_blocked=");
    Serial.print(pauseFillLastBlockBlocked ? "yes" : "no");
    Serial.print(", pause_fill_last_block_blocked_ms=");
    Serial.print(
        pauseFillLastBlockBlocked
            ? static_cast<uint32_t>(nowMs - g_pauseRuntime.fillLastBlockBlockedSinceMs)
            : 0U);
    Serial.print(", pause_progress_observed=");
    Serial.print(g_pauseRuntime.progressObserved ? "yes" : "no");
    Serial.print(", stage=");
    Serial.print(commonCycleStageName(g_commonCycle.stage));
    Serial.print(", cycle_id=");
    Serial.print(g_commonCycle.cycleId);
    Serial.print(", manip_starts=");
    Serial.print(g_commonCycle.manipStarts);
    Serial.print(", parallel_starts=");
    Serial.print(g_commonCycle.parallelStarts);
    Serial.print(", batch_seq=");
    Serial.print(g_commonCycle.lastConsumedBatchSeq);
    Serial.print(", conveyor_ready=");
    Serial.print(conveyorHasReadyBatch ? "yes" : "no");
    Serial.print(", conveyor_seq=");
    Serial.print(conveyorReadySeq);
    Serial.print(", conveyor_next_ready=");
    Serial.print(conveyorNextBatchReady ? "yes" : "no");
    Serial.print(", conveyor_ready_stale=");
    Serial.print(conveyorReadyBatchStale ? "yes" : "no");
    Serial.print(", feed_empty_strict=");
    Serial.print(conveyorFeedSideEmptyStrict(conveyor) ? "yes" : "no");
    Serial.print(", feed_empty_valid=");
    Serial.print(conveyorFeedSideEmptyValid(conveyor) ? "yes" : "no");
    Serial.print(", conveyor_p1_state=");
    Serial.print(conveyorProgramStateCode(conveyor));
    Serial.print(", conveyor_p1_pass=");
    Serial.print(conveyorProgramPass(conveyor));
    Serial.print(", step3_seen=");
    Serial.print(g_commonCycle.step3Seen ? "yes" : "no");
    Serial.print(", step3_refill_done=");
    Serial.print(g_commonCycle.step3LaunchDone ? "yes" : "no");
    Serial.print(", refill_pending=");
    Serial.print(g_commonCycle.refillStartPending ? "yes" : "no");
    Serial.print(", refill_pending_ms=");
    if (g_commonCycle.refillStartPending && g_commonCycle.refillStartRequestedMs != 0) {
        Serial.print(static_cast<uint32_t>(nowMs - g_commonCycle.refillStartRequestedMs));
    } else {
        Serial.print(0);
    }
    Serial.print(", manip_busy=");
    Serial.print(deviceStatusBusy(manipulator) ? "yes" : "no");
    Serial.print(", manip_alarm=");
    Serial.print(deviceStatusAlarm(manipulator) ? "yes" : "no");
    Serial.print(", manip_start_ready=");
    Serial.print(manipulatorInWorkStartPose(manipulator) ? "yes" : "no");
    Serial.print(", manip_q_only=");
    Serial.print(manipulatorNeedsOnlyGripOpenForWorkStart(manipulator) ? "yes" : "no");
    Serial.print(", manip_step3=");
    Serial.print(manipulatorStep3Ready(manipulator) ? "yes" : "no");
    Serial.print(", manip_step7=");
    Serial.print(manipulatorStep7Ready(manipulator) ? "yes" : "no");
    Serial.print(", manip_step=");
    Serial.print(manipulatorWorkStep(manipulator));
    Serial.print(", otvod_ready=");
    Serial.print(conveyor.online && conveyor.protocolOk && conveyorOutfeedReadyForBatch(conveyor) ? "yes" : "no");
    Serial.print(", otvod_seen_not_ready=");
    Serial.print(g_commonCycle.outfeedNotReadyObserved ? "yes" : "no");
    Serial.print(", otvod_seq=");
    Serial.print(conveyorStep2CompletionSeq(conveyor));
    Serial.print(", otvod_seq_base=");
    Serial.print(g_commonCycle.outfeedStep2SeqBase);
    const uint8_t sealerSeq = conveyorSealerCompletionSeq(conveyor);
    const bool sealerSeqAdvancedForPost7 =
        sealerSeq != g_commonCycle.post7SealerCompletionSeqBase;
    Serial.print(", sealer_seq=");
    Serial.print(sealerSeq);
    Serial.print(", sealer_seq_base=");
    Serial.print(g_commonCycle.post7SealerCompletionSeqBase);
    Serial.print(", sealer_seq_advanced=");
    Serial.print(sealerSeqAdvancedForPost7 ? "yes" : "no");
    Serial.print(", sealer_done_pending_unload=");
    Serial.print(g_commonCycle.sealerDonePendingUnload ? "yes" : "no");
    Serial.print(", drain_supervision_active=");
    Serial.print(g_commonCycle.abortDrainActive ? "yes" : "no");
    Serial.print(", drain_tail_outcome=");
    Serial.print(commonAbortDrainOutcomeName(g_commonCycle.abortDrainOutcome));
    Serial.print(", drain_elapsed_ms=");
    Serial.print(drainElapsedMs);
    Serial.print(", drain_no_progress_ms=");
    Serial.print(drainNoProgressMs);
    Serial.print(", abort_reason=");
    Serial.print(g_commonCycle.abortReason);
    Serial.print(", seal_age_ms=");
    Serial.print(sealAgeMs);
    Serial.print(", i2c_last=");
    Serial.print(g_i2cDiag.lastOp);
    Serial.print("@");
    Serial.print(g_i2cDiag.lastOrigin);
    Serial.print("/");
    Serial.print(g_i2cDiag.lastOutcome);
    Serial.print("/");
    Serial.print(g_i2cDiag.lastDurationMs);
    Serial.print("ms");
    if (g_i2cDiag.active) {
        Serial.print(", i2c_active=");
        Serial.print(g_i2cDiag.activeOp);
        Serial.print("@");
        Serial.print(g_i2cDiag.activeOrigin);
        Serial.print("/");
        Serial.print(static_cast<uint32_t>(nowMs - g_i2cDiag.activeStartedMs));
        Serial.print("ms");
    }
    Serial.print(", last=");
    Serial.println(g_commonCycle.lastEvent);
}

bool commonFeedSideCollectsLastBlock(const Rs485DeviceState &conveyor)
{
    if (!conveyorFeedSideEmptyValid(conveyor)) {
        return false;
    }
    return !conveyorFeedSideEmptyStrict(conveyor);
}

bool commonFeedSideEmpty(const Rs485DeviceState &conveyor)
{
    return conveyorFeedSideEmptyValid(conveyor) && conveyorFeedSideEmptyStrict(conveyor);
}

bool commonFeedSideSnapshotKnown(const Rs485DeviceState &conveyor)
{
    return conveyorFeedSideEmptyValid(conveyor);
}

bool commonPauseStrictGateForwardOnly()
{
    return g_commonCycle.pauseRequested &&
        g_commonCycle.pauseStrictGateLatched &&
        g_commonCycle.pauseState != CommonPauseState::StableWait;
}

void commonLogPauseStrictRollbackBlocked()
{
    if (g_commonCycle.pauseStrictRollbackBlockedLogged) {
        return;
    }
    g_commonCycle.pauseStrictRollbackBlockedLogged = true;
    Serial.println("COMMON: pause strict gate rollback blocked; keep forward-only path.");
}

void commonResetPauseStrictSnapshotTracking()
{
    g_commonCycle.pauseStrictSnapshotSinceMs = 0;
    g_commonCycle.pauseStrictSnapshotRetryMs = 0;
    g_commonCycle.pauseStrictSnapshotForceDrain = false;
    g_commonCycle.pauseStrictSnapshotRawReadyIgnoredLogged = false;
    if (g_commonCycle.pauseState == CommonPauseState::WaitStrictFeedSnapshot) {
        g_commonCycle.pauseState = CommonPauseState::Requested;
    }
}

void commonSetPauseWaitsStrictFeedSnapshot(uint32_t nowMs)
{
    const bool enteringStrictGate =
        g_commonCycle.pauseState != CommonPauseState::WaitStrictFeedSnapshot;
    g_commonCycle.pauseState = CommonPauseState::WaitStrictFeedSnapshot;
    g_commonCycle.pauseStrictGateLatched = true;
    if (enteringStrictGate) {
        g_commonCycle.pauseStrictRollbackBlockedLogged = false;
    }
    if (g_commonCycle.pauseStrictSnapshotSinceMs == 0) {
        g_commonCycle.pauseStrictSnapshotSinceMs = nowMs;
    }
    if (g_commonCycle.lastEvent != "COMMON: pause waits strict feed snapshot") {
        g_commonCycle.lastEvent = "COMMON: pause waits strict feed snapshot";
        g_lastCommandResult = g_commonCycle.lastEvent;
        Serial.println(g_lastCommandResult);
        (void)mqttPublishStatus(false);
    }
}

bool commonTryBypassPauseStrictSnapshotUnknown(const Rs485DeviceState &conveyor,
                                               bool nextBatchReadyForCommon,
                                               uint32_t nowMs)
{
    if (!g_commonCycle.pauseStrictSnapshotForceDrain) {
        commonSetPauseWaitsStrictFeedSnapshot(nowMs);
    }

    const bool rawBatchReady = conveyorBatchReady(conveyor);
    const bool rawReadyButNoSeqAwareNextBatch = rawBatchReady && !nextBatchReadyForCommon;
    if (rawReadyButNoSeqAwareNextBatch && !g_commonCycle.pauseStrictSnapshotRawReadyIgnoredLogged) {
        g_commonCycle.pauseStrictSnapshotRawReadyIgnoredLogged = true;
        Serial.println(
            "COMMON: pause strict snapshot ignores raw batchReady without seq-aware next batch.");
    } else if (!rawReadyButNoSeqAwareNextBatch) {
        g_commonCycle.pauseStrictSnapshotRawReadyIgnoredLogged = false;
    }

    if (!commonPauseStrictGateForwardOnly() &&
        (conveyorProgram1Active(conveyor) || nextBatchReadyForCommon)) {
        return false;
    }

    if (!g_commonCycle.pauseStrictSnapshotForceDrain) {
        if (g_commonCycle.pauseStrictSnapshotRetryMs == 0 ||
            static_cast<uint32_t>(nowMs - g_commonCycle.pauseStrictSnapshotRetryMs) >=
                COMMON_PAUSE_STRICT_SNAPSHOT_SYNC_RETRY_MS) {
            g_commonCycle.pauseStrictSnapshotRetryMs = nowMs;
            if (i2cSendManagedDeviceCommand(CONVEYOR_ID, "FSC", "common_pause_fs_sync")) {
                Serial.println("COMMON: pause strict snapshot sync requested (FSC).");
            } else {
                Serial.println("COMMON warning: pause strict snapshot FSC send error.");
            }
        }
    }

    if (!g_commonCycle.pauseStrictSnapshotForceDrain &&
        g_commonCycle.pauseStrictSnapshotSinceMs != 0 &&
        static_cast<uint32_t>(nowMs - g_commonCycle.pauseStrictSnapshotSinceMs) >=
            COMMON_PAUSE_STRICT_SNAPSHOT_FORCE_DRAIN_MS) {
        g_commonCycle.pauseStrictSnapshotForceDrain = true;
        g_commonCycle.pauseForceDrainApplied = true;
        g_commonCycle.pauseState = CommonPauseState::Requested;
        g_commonCycle.lastEvent = "COMMON: pause strict snapshot timeout, force drain path";
        g_lastCommandResult = g_commonCycle.lastEvent;
        Serial.println(g_lastCommandResult);
        (void)mqttPublishStatus(false);
    }

    return g_commonCycle.pauseStrictSnapshotForceDrain;
}

bool commonSealerDoneStable(uint32_t nowMs)
{
    if (!g_commonCycle.sealerDonePendingUnload || g_sealDoneLastRiseMs == 0) {
        return false;
    }
    return static_cast<uint32_t>(nowMs - g_sealDoneLastRiseMs) >= COMMON_CYCLE_SEAL_SETTLE_MS;
}

bool commonSealerDoneForCurrentCycle(const Rs485DeviceState &conveyor)
{
    if (!g_commonCycle.currentCycleLoadsSealer) {
        return true;
    }
    return conveyorSealerCompletionSeq(conveyor) != g_commonCycle.post7SealerCompletionSeqBase;
}

bool commonSealerEmptyConfirmed(const Rs485DeviceState &conveyor)
{
    if (g_commonCycle.sealerDonePendingUnload) {
        return false;
    }
    if (conveyorSealerBusy(conveyor) || conveyorSealerDoneActive(conveyor)) {
        return false;
    }
    if (g_commonCycle.currentCycleLoadsSealer && !commonSealerDoneForCurrentCycle(conveyor)) {
        return false;
    }
    return true;
}

bool commonTryEnterPauseStableWait(const Rs485DeviceState &conveyor)
{
    if (!commonSealerEmptyConfirmed(conveyor)) {
        g_commonCycle.pauseState = CommonPauseState::Requested;
        if (g_commonCycle.lastEvent != "COMMON: pause waits SEALER empty") {
            g_commonCycle.lastEvent = "COMMON: pause waits SEALER empty";
            g_lastCommandResult = g_commonCycle.lastEvent;
            Serial.println(g_lastCommandResult);
            (void)mqttPublishStatus(false);
        }
        return false;
    }

    const bool finalizedAfterForceDrain =
        g_commonCycle.pauseStrictSnapshotForceDrain || g_commonCycle.pauseForceDrainApplied;
    const uint32_t nowMs = millis();
    if (g_pauseRuntime.mandatoryUnloadActive) {
        Serial.print("COMMON: pause mandatory unload settlement complete; ");
        Serial.print("gate=sealer_empty_confirmed");
        Serial.print(", sealer_pending_unload=");
        Serial.print(g_commonCycle.sealerDonePendingUnload ? "yes" : "no");
        Serial.print(", sealer_busy=");
        Serial.print(conveyorSealerBusy(conveyor) ? "yes" : "no");
        Serial.print(", sealer_done_active=");
        Serial.print(conveyorSealerDoneActive(conveyor) ? "yes" : "no");
        Serial.print(", sealer_done_for_cycle=");
        Serial.print(commonSealerDoneForCurrentCycle(conveyor) ? "yes" : "no");
        Serial.print(", outfeed_ready=");
        Serial.print(conveyorOutfeedReadyForBatch(conveyor) ? "yes" : "no");
        Serial.print(", mandatory_unload_elapsed_ms=");
        Serial.println(commonPauseRuntimeMandatoryUnloadElapsedMs(nowMs));
    }
    commonPauseRuntimeClearMandatoryUnloadTracking();
    commonResetPauseStrictSnapshotTracking();
    g_commonCycle.pauseState = CommonPauseState::StableWait;
    const String stableEvent = finalizedAfterForceDrain
        ? String("COMMON: pause finalized after force drain, strict snapshot gate released")
        : String("COMMON: pause_stable_wait");
    if (g_commonCycle.lastEvent != stableEvent) {
        g_commonCycle.lastEvent = stableEvent;
        g_lastCommandResult = g_commonCycle.lastEvent;
        Serial.println(g_lastCommandResult);
        (void)mqttPublishStatus(false);
    }
    if (finalizedAfterForceDrain) {
        commonPauseRuntimeRestartArmingWindow(
            nowMs,
            "force_drain_finalized_waits_terminal_outcome");
    }
    return true;
}

void commonSetEventIfChanged(const String &eventText)
{
    if (g_commonCycle.lastEvent == eventText) {
        return;
    }

    g_commonCycle.lastEvent = eventText;
    g_lastCommandResult = eventText;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
}

bool commonPauseRuntimeNodeReadyForFinalize()
{
    return g_pauseRuntime.nodeReadyConfirmed;
}

void commonPauseRuntimeMarkFinalizeStarted()
{
    if (g_pauseRuntime.finalizeStarted) {
        return;
    }

    g_pauseRuntime.finalizeStarted = true;
    commonSetEventIfChanged("COMMON: pause finalize started after node ready");
}

bool commonPauseStableContractReached()
{
    if (!g_commonCycle.pauseRequested ||
        g_commonCycle.pauseState != CommonPauseState::StableWait) {
        return false;
    }

    const bool terminalPauseOutcomeReached =
        g_pauseRuntime.state == CommonPauseRuntimeState::PauseHold ||
        g_pauseRuntime.state == CommonPauseRuntimeState::PauseEmpty ||
        g_pauseRuntime.state == CommonPauseRuntimeState::ManualRecoveryRequired;
    if (terminalPauseOutcomeReached) {
        return true;
    }

    if (g_commonCycle.pauseForceDrainApplied) {
        commonSetEventIfChanged("COMMON: pause finalized after force drain, strict snapshot gate released");
    }
    return true;
}

void commonClearRefillPending()
{
    g_commonCycle.refillStartPending = false;
    g_commonCycle.refillStartRequestedMs = 0;
}

bool commonRefillPendingFresh(uint32_t nowMs)
{
    if (!g_commonCycle.refillStartPending) {
        return false;
    }
    return static_cast<uint32_t>(nowMs - g_commonCycle.refillStartRequestedMs) < COMMON_REFILL_RETRY_GUARD_MS;
}

void commonPauseRuntimeApplyState(CommonPauseRuntimeState state)
{
    g_pauseRuntime.state = state;
    g_pauseRuntime.conveyorSnapshot.pauseState = state;
    g_pauseRuntime.manipulatorSnapshot.pauseState = state;
}

bool commonPauseRuntimeMandatoryUnloadInFlight(const Rs485DeviceState &conveyor,
                                               const Rs485DeviceState &manipulator)
{
    if (!g_pauseRuntime.mandatoryUnloadActive) {
        return false;
    }
    return deviceStatusBusy(manipulator) ||
        conveyorStep2Active(conveyor) ||
        conveyorVfdTimedRunActive(conveyor);
}

bool commonPauseRuntimeMandatoryUnloadPolicyActive()
{
    // Mandatory unload keeps pause runtime under extended watchdog contract
    // until settlement is explicitly completed.
    return g_pauseRuntime.mandatoryUnloadActive;
}

bool commonPauseRuntimeForceDrainStableWaitPolicyActive()
{
    return g_commonCycle.pauseRequested &&
        g_commonCycle.pauseState == CommonPauseState::StableWait &&
        g_commonCycle.pauseForceDrainApplied &&
        !g_pauseRuntime.mandatoryUnloadActive;
}

bool commonPauseRuntimeWatchdogActive()
{
    return g_commonCycle.pauseRequested &&
        g_pauseRuntime.state != CommonPauseRuntimeState::PauseHold &&
        g_pauseRuntime.state != CommonPauseRuntimeState::PauseEmpty &&
        g_pauseRuntime.state != CommonPauseRuntimeState::ManualRecoveryRequired;
}

uint32_t commonPauseRuntimeMandatoryUnloadElapsedMs(uint32_t nowMs)
{
    if (!g_pauseRuntime.mandatoryUnloadActive || g_pauseRuntime.mandatoryUnloadSinceMs == 0) {
        return 0;
    }
    return static_cast<uint32_t>(nowMs - g_pauseRuntime.mandatoryUnloadSinceMs);
}

const char *commonPauseRuntimeWatchdogProfileName(bool mandatoryUnloadPolicyActive)
{
    if (mandatoryUnloadPolicyActive) {
        return "mandatory_unload";
    }
    if (commonPauseRuntimeForceDrainStableWaitPolicyActive()) {
        return "post_force_drain_stable_wait";
    }
    return "normal_pause";
}

const char *commonPauseRuntimeWatchdogPolicyReason(bool mandatoryUnloadPolicyActive,
                                                   bool mandatoryUnloadInFlight)
{
    if (mandatoryUnloadPolicyActive && mandatoryUnloadInFlight) {
        return "mandatory_unload_in_flight";
    }
    if (mandatoryUnloadPolicyActive) {
        return "mandatory_unload_settle_wait";
    }
    if (commonPauseRuntimeForceDrainStableWaitPolicyActive()) {
        return "force_drain_finalized_waits_terminal_outcome";
    }
    return "normal_pause_contract";
}

void commonPauseRuntimeRestartArmingWindow(uint32_t nowMs, const char *reason)
{
    const uint32_t previousElapsedMs = commonPauseRuntimeArmingElapsedMs(nowMs);
    const uint32_t previousNoProgressMs = commonPauseRuntimeNoProgressMs(nowMs);
    g_pauseRuntime.armingSinceMs = nowMs;
    g_pauseRuntime.lastProgressMs = nowMs;
    g_pauseRuntime.lastWatchdogWaitLogMs = 0;
    g_pauseRuntime.watchdogPolicyKnown = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryUnload = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryInFlight = false;
    g_pauseRuntime.lastWatchdogPolicyForceDrainStableWait = false;

    Serial.print("COMMON: pause arming window restarted; reason=");
    Serial.print(reason);
    Serial.print(", previous_elapsed_ms=");
    Serial.print(previousElapsedMs);
    Serial.print(", previous_no_progress_ms=");
    Serial.print(previousNoProgressMs);
    Serial.print(", pause_state=");
    Serial.println(commonPauseStateName(g_commonCycle.pauseState));
}

const char *commonPauseRuntimeSubstateName(const Rs485DeviceState &conveyor,
                                           const Rs485DeviceState &manipulator)
{
    if (!g_commonCycle.pauseRequested) {
        return "none";
    }
    if (!g_pauseRuntime.nodeReadyConfirmed) {
        if (g_pauseRuntime.conveyorSnapshot.nodeStateFresh) {
            return "wait_node_ready";
        }
        return "pause_request";
    }
    if (g_pauseRuntime.mandatoryUnloadActive) {
        return "mandatory_unload";
    }
    if (g_commonCycle.pauseState == CommonPauseState::WaitStrictFeedSnapshot) {
        return "pause_wait_strict_snapshot";
    }
    if (g_commonCycle.pauseState == CommonPauseState::StableWait) {
        return "pause_stable_wait";
    }
    if (g_commonCycle.pauseState == CommonPauseState::FillLastBlock) {
        if (commonPauseFillLastBlockCompleted(conveyor, manipulator)) {
            return "pause_fill_last_block_completed";
        }
        if (g_pauseRuntime.fillLastBlockBlockedSinceMs != 0) {
            return "pause_fill_last_block_terminal_decision";
        }
        return "pause_fill_last_block";
    }
    return "pause_finalize";
}

void commonPauseRuntimeClearMandatoryUnloadTracking()
{
    g_pauseRuntime.mandatoryUnloadActive = false;
    g_pauseRuntime.mandatoryUnloadSinceMs = 0;
    g_pauseRuntime.mandatoryUnloadLastProgressMs = 0;
    g_pauseRuntime.lastConveyorBusy = false;
    g_pauseRuntime.lastManipulatorBusy = false;
    g_pauseRuntime.lastManipulatorWorkStep = 0;
    g_pauseRuntime.lastOutfeedReady = false;
    g_pauseRuntime.lastOutfeedStep2Active = false;
    g_pauseRuntime.lastOutfeedVfdActive = false;
    g_pauseRuntime.lastOutfeedStep2Seq = 0;
    g_pauseRuntime.lastOutfeedSealerSeq = 0;
    g_pauseRuntime.watchdogPolicyKnown = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryUnload = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryInFlight = false;
    g_pauseRuntime.lastWatchdogPolicyForceDrainStableWait = false;
}

void commonPauseRuntimeMarkMandatoryUnloadStarted(uint32_t nowMs)
{
    g_pauseRuntime.mandatoryUnloadActive = true;
    g_pauseRuntime.mandatoryUnloadSinceMs = nowMs;
    g_pauseRuntime.mandatoryUnloadLastProgressMs = nowMs;
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    g_pauseRuntime.lastConveyorBusy = deviceStatusBusy(conveyor);
    g_pauseRuntime.lastManipulatorBusy = deviceStatusBusy(manipulator);
    g_pauseRuntime.lastManipulatorWorkStep = manipulatorWorkStep(manipulator);
    g_pauseRuntime.lastOutfeedReady = conveyorOutfeedReadyForBatch(conveyor);
    g_pauseRuntime.lastOutfeedStep2Active = conveyorStep2Active(conveyor);
    g_pauseRuntime.lastOutfeedVfdActive = conveyorVfdTimedRunActive(conveyor);
    g_pauseRuntime.lastOutfeedStep2Seq = conveyorStep2CompletionSeq(conveyor);
    g_pauseRuntime.lastOutfeedSealerSeq = conveyorSealerCompletionSeq(conveyor);
}

bool commonPauseRuntimeCanRelease()
{
    return g_pauseRuntime.state == CommonPauseRuntimeState::PauseHold ||
        g_pauseRuntime.state == CommonPauseRuntimeState::PauseEmpty;
}

uint32_t commonPauseRuntimeNextEpoch()
{
    uint32_t next = g_pauseRuntime.currentEpoch + 1U;
    if (next == 0U) {
        next = 1U;
    }
    g_pauseRuntime.currentEpoch = next;
    return next;
}

void commonPauseRuntimeDeactivate()
{
    commonPauseRuntimeApplyState(CommonPauseRuntimeState::None);
    g_pauseRuntime.pauseCycleId = 0;
    g_pauseRuntime.armingSinceMs = 0;
    g_pauseRuntime.lastProgressMs = 0;
    g_pauseRuntime.lastStatusPollMs = 0;
    g_pauseRuntime.progressObserved = false;
    g_pauseRuntime.lastLegacyPauseState = CommonPauseState::None;
    g_pauseRuntime.lastConveyorNodeState = CommonPauseNodeState::None;
    g_pauseRuntime.lastBufferCount = 0;
    g_pauseRuntime.lastBufferCountValid = false;
    g_pauseRuntime.lastSealerCount = 0;
    g_pauseRuntime.lastSealerCountValid = false;
    g_pauseRuntime.lastBufferCountSource = PauseCountSource::None;
    g_pauseRuntime.lastSealerCountSource = PauseCountSource::None;
    g_pauseRuntime.armSentConveyor = false;
    g_pauseRuntime.armSentManipulator = false;
    g_pauseRuntime.releaseSentConveyor = false;
    g_pauseRuntime.releaseSentManipulator = false;
    g_pauseRuntime.lastNodePauseEpochConfirmed = false;
    g_pauseRuntime.nodeReadyConfirmed = false;
    g_pauseRuntime.finalizeStarted = false;
    commonPauseRuntimeClearMandatoryUnloadTracking();
    g_pauseRuntime.lastWatchdogWaitLogMs = 0;
    g_pauseRuntime.lastConveyorBusy = false;
    g_pauseRuntime.lastManipulatorBusy = false;
    g_pauseRuntime.lastManipulatorWorkStep = 0;
    g_pauseRuntime.lastOutfeedReady = false;
    g_pauseRuntime.lastOutfeedStep2Active = false;
    g_pauseRuntime.lastOutfeedVfdActive = false;
    g_pauseRuntime.lastOutfeedStep2Seq = 0;
    g_pauseRuntime.lastOutfeedSealerSeq = 0;
    g_pauseRuntime.lastFillLastBlockCompleted = false;
    g_pauseRuntime.fillLastBlockCompletionLatched = false;
    g_pauseRuntime.fillLastBlockBlockedSinceMs = 0;
    g_pauseRuntime.fillLastBlockBlockedLogged = false;
    g_pauseRuntime.watchdogPolicyKnown = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryUnload = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryInFlight = false;
    g_pauseRuntime.lastWatchdogPolicyForceDrainStableWait = false;
    g_pauseRuntime.conveyorSnapshot = CommonPauseSnapshot{};
    g_pauseRuntime.manipulatorSnapshot = CommonPauseSnapshot{};
}

void commonPauseRuntimeBeginEpoch(uint32_t nowMs)
{
    const uint32_t epoch = commonPauseRuntimeNextEpoch();
    g_pauseRuntime.pauseCycleId = g_commonCycle.cycleId;
    g_pauseRuntime.armingSinceMs = nowMs;
    g_pauseRuntime.lastProgressMs = nowMs;
    g_pauseRuntime.lastStatusPollMs = 0;
    g_pauseRuntime.progressObserved = false;
    g_pauseRuntime.lastLegacyPauseState = g_commonCycle.pauseState;
    g_pauseRuntime.lastConveyorNodeState = CommonPauseNodeState::None;
    g_pauseRuntime.lastBufferCount = 0;
    g_pauseRuntime.lastBufferCountValid = false;
    g_pauseRuntime.lastSealerCount = 0;
    g_pauseRuntime.lastSealerCountValid = false;
    g_pauseRuntime.armSentConveyor = false;
    g_pauseRuntime.armSentManipulator = false;
    g_pauseRuntime.releaseSentConveyor = false;
    g_pauseRuntime.releaseSentManipulator = false;
    g_pauseRuntime.lastNodePauseEpochConfirmed = false;
    g_pauseRuntime.nodeReadyConfirmed = false;
    g_pauseRuntime.finalizeStarted = false;
    g_pauseRuntime.watchdogPolicyKnown = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryUnload = false;
    g_pauseRuntime.lastWatchdogPolicyMandatoryInFlight = false;
    g_pauseRuntime.lastWatchdogPolicyForceDrainStableWait = false;
    commonPauseRuntimeClearMandatoryUnloadTracking();
    g_pauseRuntime.lastWatchdogWaitLogMs = 0;
    g_pauseRuntime.lastConveyorBusy = false;
    g_pauseRuntime.lastManipulatorBusy = false;
    g_pauseRuntime.lastManipulatorWorkStep = 0;
    g_pauseRuntime.lastOutfeedReady = false;
    g_pauseRuntime.lastOutfeedStep2Active = false;
    g_pauseRuntime.lastOutfeedVfdActive = false;
    g_pauseRuntime.lastOutfeedStep2Seq = 0;
    g_pauseRuntime.lastOutfeedSealerSeq = 0;
    g_pauseRuntime.lastFillLastBlockCompleted = false;
    g_pauseRuntime.fillLastBlockCompletionLatched = false;
    g_pauseRuntime.fillLastBlockBlockedSinceMs = 0;
    g_pauseRuntime.fillLastBlockBlockedLogged = false;
    g_pauseRuntime.conveyorSnapshot = CommonPauseSnapshot{};
    g_pauseRuntime.manipulatorSnapshot = CommonPauseSnapshot{};
    g_pauseRuntime.conveyorSnapshot.pauseEpoch = epoch;
    g_pauseRuntime.conveyorSnapshot.pauseCycleId = g_pauseRuntime.pauseCycleId;
    g_pauseRuntime.manipulatorSnapshot.pauseEpoch = epoch;
    g_pauseRuntime.manipulatorSnapshot.pauseCycleId = g_pauseRuntime.pauseCycleId;
    commonPauseRuntimeApplyState(CommonPauseRuntimeState::PauseArming);
}

bool commonPauseFillLastBlockCompleted(const Rs485DeviceState &conveyor,
                                       const Rs485DeviceState &manipulator)
{
    const CommonPauseSnapshot &snap = g_pauseRuntime.conveyorSnapshot;
    const bool nodeReadySnapshot =
        g_pauseRuntime.nodeReadyConfirmed &&
        snap.nodePauseEpochConfirmed &&
        snap.nodeStateFresh &&
        snap.nodeState == CommonPauseNodeState::Ready;
    const bool trustedBufferEmpty =
        snap.nodePauseEpochConfirmed &&
        snap.bufferCountValid &&
        snap.bufferCountFresh &&
        snap.bufferCountSource == PauseCountSource::TrustedWire &&
        snap.bufferCount == 0;
    const bool manipIdle = !deviceStatusBusy(manipulator);
    const bool conveyorTailIdle =
        !deviceStatusBusy(conveyor) &&
        !conveyorStep2Active(conveyor) &&
        !conveyorVfdTimedRunActive(conveyor);
    const bool outfeedReady = conveyorOutfeedReadyForBatch(conveyor);
    return nodeReadySnapshot &&
        trustedBufferEmpty &&
        manipIdle &&
        conveyorTailIdle &&
        outfeedReady;
}

bool commonPauseContractDrainIdle(const Rs485DeviceState &conveyor,
                                  const Rs485DeviceState &manipulator,
                                  uint32_t nowMs)
{
    (void)nowMs;
    if (g_commonCycle.pauseState == CommonPauseState::StableWait) {
        return true;
    }
    const bool legacyDrainIdle =
        !conveyorProgram1Active(conveyor) &&
        !deviceStatusBusy(manipulator) &&
        conveyorFeedSideEmptyValid(conveyor) &&
        conveyorFeedSideEmptyStrict(conveyor);
    if (legacyDrainIdle) {
        return true;
    }
    return commonPauseFillLastBlockCompleted(conveyor, manipulator);
}

bool commonPauseFinalizeFeedSideKnownEmpty(bool feedSideKnown, bool feedSideEmpty)
{
    if (feedSideKnown && feedSideEmpty) {
        return true;
    }

    const CommonPauseSnapshot &snap = g_pauseRuntime.conveyorSnapshot;
    return g_pauseRuntime.nodeReadyConfirmed &&
        snap.nodePauseEpochConfirmed &&
        snap.nodeStateFresh &&
        snap.nodeState == CommonPauseNodeState::Ready &&
        snap.bufferCountValid &&
        snap.bufferCountFresh &&
        snap.bufferCountSource == PauseCountSource::TrustedWire &&
        snap.bufferCount == 0;
}

void commonPauseRuntimeUpdateSnapshots(const Rs485DeviceState &conveyor, uint32_t nowMs)
{
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    CommonPauseSnapshot conveyorSnap = {};
    conveyorSnap.pauseEpoch = g_pauseRuntime.currentEpoch;
    conveyorSnap.pauseCycleId = g_pauseRuntime.pauseCycleId;
    conveyorSnap.pauseState = g_pauseRuntime.state;
    conveyorSnap.nodeState = CommonPauseNodeState::None;
    conveyorSnap.nodeStateFresh = false;
    conveyorSnap.manualRecoveryRequired = commonManualRecoveryRequired();
    conveyorSnap.progressObserved = g_pauseRuntime.progressObserved;
    conveyorSnap.updatedMs = nowMs;

    const bool conveyorOnline = conveyor.online && conveyor.protocolOk;
    const bool manipulatorOnline = manipulator.online && manipulator.protocolOk;
    const bool convStale =
        !conveyorOnline ||
        (nowMs - conveyor.lastSeenMs) > COMMON_PAUSE_NODE_FRAME_STALE_MS;
    const bool manipStale =
        !manipulatorOnline ||
        (nowMs - manipulator.lastSeenMs) > COMMON_PAUSE_NODE_FRAME_STALE_MS;

    const uint32_t convHb = conveyor.managedHeartbeatMs;
    const bool extValid = conveyorOnline && conveyorPauseHeartbeatExtValid(convHb);
    const uint16_t epoch12 = static_cast<uint16_t>(g_pauseRuntime.currentEpoch & 0x0FFFU);
    const uint16_t epoch10 = static_cast<uint16_t>(g_pauseRuntime.currentEpoch & 0x03FFU);

    const bool convAck =
        conveyorOnline &&
        conveyorPauseStatusAckValid(conveyor) &&
        conveyorPauseStatusAckEpoch(conveyor) == epoch12;
    const bool manipAck =
        manipulatorOnline &&
        manipulatorPauseStatusAckValid(manipulator) &&
        manipulatorPauseStatusAckEpoch(manipulator) == epoch10;

    const bool nodeEpochConfirmed =
        convAck && manipAck && !convStale && !manipStale && extValid;

    int16_t wireBuf = 0;
    bool wireBufValid = false;
    int16_t wireSeal = 0;
    bool wireSealValid = false;
    bool wireResidualKnown = false;
    if (extValid) {
        conveyorUnpackPauseHeartbeat(
            convHb,
            wireBuf,
            wireBufValid,
            wireSeal,
            wireSealValid,
            wireResidualKnown);
    }
    const bool conveyorNodeStateVisible =
        convAck && extValid && !convStale;
    if (conveyorNodeStateVisible) {
        conveyorSnap.nodeState = conveyorPauseHeartbeatNodeState(convHb);
        conveyorSnap.nodeStateFresh = true;
    }

    conveyorSnap.nodePauseEpochConfirmed = nodeEpochConfirmed;

    if (nodeEpochConfirmed && extValid) {
        conveyorSnap.bufferCount = wireBuf;
        conveyorSnap.bufferCountValid = wireBufValid;
        conveyorSnap.bufferCountFresh = wireBufValid && !convStale;
        conveyorSnap.bufferCountSource = wireBufValid
            ? PauseCountSource::TrustedWire
            : PauseCountSource::TrustedWireUnknown;
        conveyorSnap.sealerCount = wireSeal;
        conveyorSnap.sealerCountValid = wireSealValid;
        conveyorSnap.sealerCountFresh = wireSealValid && !convStale;
        conveyorSnap.sealerCountSource = wireSealValid
            ? PauseCountSource::TrustedWire
            : PauseCountSource::TrustedWireUnknown;
        if (conveyorSnap.sealerCount > 0) {
            conveyorSnap.sealerResidualKnown = wireResidualKnown;
        } else {
            conveyorSnap.sealerResidualKnown = wireSealValid;
        }
        conveyorSnap.contractSnapshotTrusted =
            conveyorSnap.bufferCountValid &&
            conveyorSnap.bufferCountFresh &&
            conveyorSnap.sealerCountValid &&
            conveyorSnap.sealerCountFresh &&
            (conveyorSnap.sealerCount == 0 ||
             (conveyorSnap.sealerCount > 0 && conveyorSnap.sealerResidualKnown));
    } else {
        conveyorSnap.bufferCountSource = PauseCountSource::SurrogateFeedFlags;
        conveyorSnap.bufferCountValid = conveyorOnline && conveyorFeedSideEmptyValid(conveyor);
        if (conveyorSnap.bufferCountValid) {
            conveyorSnap.bufferCount = conveyorFeedSideEmptyStrict(conveyor) ? 0 : 1;
        } else {
            conveyorSnap.bufferCount = -1;
        }
        conveyorSnap.bufferCountFresh = false;

        conveyorSnap.sealerCountSource = PauseCountSource::SurrogateSealerSignals;
        const bool sealerResidualSeen =
            conveyorOnline &&
            (conveyorSealerBusy(conveyor) ||
             conveyorSealerDoneActive(conveyor) ||
             g_commonCycle.sealerDonePendingUnload);
        conveyorSnap.sealerCountValid = sealerResidualSeen;
        if (conveyorSnap.sealerCountValid) {
            conveyorSnap.sealerCount = COMMON_PAUSE_SEALER_RESIDUAL_COUNT;
        } else {
            conveyorSnap.sealerCount = -1;
        }
        conveyorSnap.sealerCountFresh = false;
        conveyorSnap.sealerResidualKnown = false;
        conveyorSnap.contractSnapshotTrusted = false;
    }

    g_pauseRuntime.conveyorSnapshot = conveyorSnap;

    CommonPauseSnapshot manipulatorSnap = conveyorSnap;
    manipulatorSnap.bufferCount = 0;
    manipulatorSnap.bufferCountValid = false;
    manipulatorSnap.sealerCount = 0;
    manipulatorSnap.sealerCountValid = false;
    manipulatorSnap.sealerResidualKnown = false;
    manipulatorSnap.bufferCountFresh = false;
    manipulatorSnap.sealerCountFresh = false;
    manipulatorSnap.bufferCountSource = PauseCountSource::None;
    manipulatorSnap.sealerCountSource = PauseCountSource::None;
    manipulatorSnap.nodePauseEpochConfirmed = nodeEpochConfirmed;
    manipulatorSnap.contractSnapshotTrusted = conveyorSnap.contractSnapshotTrusted;
    manipulatorSnap.nodeState = CommonPauseNodeState::None;
    manipulatorSnap.nodeStateFresh = false;
    g_pauseRuntime.manipulatorSnapshot = manipulatorSnap;
}

bool commonPauseRuntimeRecordProgress(uint32_t nowMs,
                                      const Rs485DeviceState &conveyor,
                                      const Rs485DeviceState &manipulator)
{
    const CommonPauseSnapshot &snap = g_pauseRuntime.conveyorSnapshot;
    bool progress = false;
    String progressDiag;
    auto markProgress = [&](const String &marker) {
        progress = true;
        if (!progressDiag.isEmpty()) {
            progressDiag += ", ";
        }
        progressDiag += marker;
    };

    if (snap.nodePauseEpochConfirmed && !g_pauseRuntime.lastNodePauseEpochConfirmed) {
        markProgress("node_epoch_confirmed");
    }
    g_pauseRuntime.lastNodePauseEpochConfirmed = snap.nodePauseEpochConfirmed;
    if (snap.nodeStateFresh && snap.nodeState != g_pauseRuntime.lastConveyorNodeState) {
        Serial.print("COMMON: pause node state: ");
        Serial.println(commonPauseNodeStateName(snap.nodeState));
        markProgress("node_state=" + String(commonPauseNodeStateName(snap.nodeState)));
    }
    if (g_commonCycle.pauseState != g_pauseRuntime.lastLegacyPauseState) {
        markProgress("pause_state=" + String(commonPauseStateName(g_commonCycle.pauseState)));
    }
    if (snap.bufferCountValid && !g_pauseRuntime.lastBufferCountValid) {
        markProgress("buffer_count_valid");
    }
    if (snap.sealerCountValid && !g_pauseRuntime.lastSealerCountValid) {
        markProgress("sealer_count_valid");
    }
    if (snap.bufferCountSource != g_pauseRuntime.lastBufferCountSource) {
        markProgress("buffer_source=" + String(pauseCountSourceName(snap.bufferCountSource)));
    }
    if (snap.sealerCountSource != g_pauseRuntime.lastSealerCountSource) {
        markProgress("sealer_source=" + String(pauseCountSourceName(snap.sealerCountSource)));
    }
    if (snap.bufferCountValid &&
        g_pauseRuntime.lastBufferCountValid &&
        snap.bufferCount < g_pauseRuntime.lastBufferCount) {
        markProgress("buffer_count_drop=" + String(snap.bufferCount));
    }
    if (snap.sealerCountValid &&
        g_pauseRuntime.lastSealerCountValid &&
        snap.sealerCount < g_pauseRuntime.lastSealerCount) {
        markProgress("sealer_count_drop=" + String(snap.sealerCount));
    }
    if (!progress &&
        snap.nodeStateFresh &&
        snap.nodeState == CommonPauseNodeState::Preparing &&
        g_pauseRuntime.lastProgressMs != 0 &&
        static_cast<uint32_t>(nowMs - g_pauseRuntime.lastProgressMs) >=
            COMMON_PAUSE_NODE_PREPARING_HEARTBEAT_MS) {
        markProgress("node_preparing_heartbeat");
    }

    const bool conveyorBusy = deviceStatusBusy(conveyor);
    const bool manipBusy = deviceStatusBusy(manipulator);
    const uint8_t manipStep = manipulatorWorkStep(manipulator);
    const bool outfeedReady = conveyorOutfeedReadyForBatch(conveyor);
    const bool outfeedStep2Active = conveyorStep2Active(conveyor);
    const bool outfeedVfdActive = conveyorVfdTimedRunActive(conveyor);
    const uint8_t outfeedStep2Seq = conveyorStep2CompletionSeq(conveyor);
    const uint8_t outfeedSealerSeq = conveyorSealerCompletionSeq(conveyor);
    const bool fillLastBlockCompleted =
        commonPauseFillLastBlockCompleted(conveyor, manipulator);

    if (g_pauseRuntime.finalizeStarted && !g_pauseRuntime.mandatoryUnloadActive) {
        const bool activityInFlight =
            conveyorBusy || manipBusy || outfeedStep2Active || outfeedVfdActive;

        if (conveyorBusy != g_pauseRuntime.lastConveyorBusy) {
            markProgress("pause_finalize.conv_busy=" + String(conveyorBusy ? "yes" : "no"));
        }
        if (manipBusy != g_pauseRuntime.lastManipulatorBusy) {
            markProgress("pause_finalize.manip_busy=" + String(manipBusy ? "yes" : "no"));
        }
        if (manipStep != g_pauseRuntime.lastManipulatorWorkStep) {
            markProgress("pause_finalize.manip_step=" + String(manipStep));
        }
        if (outfeedReady != g_pauseRuntime.lastOutfeedReady) {
            markProgress("pause_finalize.outfeed_ready=" + String(outfeedReady ? "yes" : "no"));
        }
        if (outfeedStep2Active && !g_pauseRuntime.lastOutfeedStep2Active) {
            markProgress("pause_finalize.step2_active_rise");
        }
        if (outfeedVfdActive && !g_pauseRuntime.lastOutfeedVfdActive) {
            markProgress("pause_finalize.vfd_active_rise");
        }
        if (outfeedStep2Seq != g_pauseRuntime.lastOutfeedStep2Seq) {
            markProgress("pause_finalize.step2_seq=" + String(outfeedStep2Seq));
        }
        if (outfeedSealerSeq != g_pauseRuntime.lastOutfeedSealerSeq) {
            markProgress("pause_finalize.sealer_seq=" + String(outfeedSealerSeq));
        }
        if (fillLastBlockCompleted != g_pauseRuntime.lastFillLastBlockCompleted) {
            markProgress("pause_finalize.fill_last_block_completed=" +
                         String(fillLastBlockCompleted ? "yes" : "no"));
        }
        if (!progress &&
            activityInFlight &&
            g_pauseRuntime.lastProgressMs != 0 &&
            static_cast<uint32_t>(nowMs - g_pauseRuntime.lastProgressMs) >=
                COMMON_PAUSE_FINALIZE_ACTIVITY_HEARTBEAT_MS) {
            markProgress("pause_finalize.activity_heartbeat");
        }
    }

    if (g_pauseRuntime.mandatoryUnloadActive) {
        const bool outfeedReady = conveyorOutfeedReadyForBatch(conveyor);
        const bool outfeedStep2Active = conveyorStep2Active(conveyor);
        const bool outfeedVfdActive = conveyorVfdTimedRunActive(conveyor);
        const uint8_t outfeedStep2Seq = conveyorStep2CompletionSeq(conveyor);
        const uint8_t outfeedSealerSeq = conveyorSealerCompletionSeq(conveyor);
        const bool mandatoryUnloadInFlightNow =
            manipBusy || outfeedStep2Active || outfeedVfdActive;
        const bool mandatoryUnloadInFlightPrev =
            g_pauseRuntime.lastManipulatorBusy ||
            g_pauseRuntime.lastOutfeedStep2Active ||
            g_pauseRuntime.lastOutfeedVfdActive;

        if (mandatoryUnloadInFlightPrev && !mandatoryUnloadInFlightNow) {
            Serial.print("COMMON: pause mandatory unload enters settle wait; ");
            Serial.print("reason=all_in_flight_signals_cleared");
            Serial.print(", manip_busy=");
            Serial.print(manipBusy ? "yes" : "no");
            Serial.print(", outfeed_step2_active=");
            Serial.print(outfeedStep2Active ? "yes" : "no");
            Serial.print(", outfeed_vfd_active=");
            Serial.print(outfeedVfdActive ? "yes" : "no");
            Serial.print(", outfeed_ready=");
            Serial.print(outfeedReady ? "yes" : "no");
            Serial.print(", arming_elapsed_ms=");
            Serial.print(commonPauseRuntimeArmingElapsedMs(nowMs));
            Serial.print(", mandatory_unload_elapsed_ms=");
            Serial.println(commonPauseRuntimeMandatoryUnloadElapsedMs(nowMs));
        }

        if (manipBusy != g_pauseRuntime.lastManipulatorBusy) {
            markProgress("mandatory_unload.manip_busy=" + String(manipBusy ? "yes" : "no"));
        }
        if (manipStep != g_pauseRuntime.lastManipulatorWorkStep) {
            markProgress("mandatory_unload.manip_step=" + String(manipStep));
        }
        if (outfeedReady != g_pauseRuntime.lastOutfeedReady) {
            markProgress("mandatory_unload.outfeed_ready=" + String(outfeedReady ? "yes" : "no"));
        }
        if (outfeedStep2Active && !g_pauseRuntime.lastOutfeedStep2Active) {
            markProgress("mandatory_unload.step2_active_rise");
        }
        if (outfeedVfdActive && !g_pauseRuntime.lastOutfeedVfdActive) {
            markProgress("mandatory_unload.vfd_active_rise");
        }
        if (outfeedStep2Seq != g_pauseRuntime.lastOutfeedStep2Seq) {
            markProgress("mandatory_unload.step2_seq=" + String(outfeedStep2Seq));
        }
        if (outfeedSealerSeq != g_pauseRuntime.lastOutfeedSealerSeq) {
            markProgress("mandatory_unload.sealer_seq=" + String(outfeedSealerSeq));
        }
        if (!progress &&
            manipBusy &&
            g_pauseRuntime.lastProgressMs != 0 &&
            static_cast<uint32_t>(nowMs - g_pauseRuntime.lastProgressMs) >=
                COMMON_PAUSE_MANDATORY_UNLOAD_BUSY_HEARTBEAT_MS) {
            markProgress("mandatory_unload.busy_heartbeat");
        }
    }

    g_pauseRuntime.lastConveyorBusy = conveyorBusy;
    g_pauseRuntime.lastManipulatorBusy = manipBusy;
    g_pauseRuntime.lastManipulatorWorkStep = manipStep;
    g_pauseRuntime.lastOutfeedReady = outfeedReady;
    g_pauseRuntime.lastOutfeedStep2Active = outfeedStep2Active;
    g_pauseRuntime.lastOutfeedVfdActive = outfeedVfdActive;
    g_pauseRuntime.lastOutfeedStep2Seq = outfeedStep2Seq;
    g_pauseRuntime.lastOutfeedSealerSeq = outfeedSealerSeq;
    g_pauseRuntime.lastFillLastBlockCompleted = fillLastBlockCompleted;

    g_pauseRuntime.lastLegacyPauseState = g_commonCycle.pauseState;
    if (snap.nodeStateFresh) {
        g_pauseRuntime.lastConveyorNodeState = snap.nodeState;
    }
    if (snap.bufferCountValid) {
        g_pauseRuntime.lastBufferCount = snap.bufferCount;
        g_pauseRuntime.lastBufferCountValid = true;
    } else {
        g_pauseRuntime.lastBufferCountValid = false;
    }
    if (snap.sealerCountValid) {
        g_pauseRuntime.lastSealerCount = snap.sealerCount;
        g_pauseRuntime.lastSealerCountValid = true;
    } else {
        g_pauseRuntime.lastSealerCountValid = false;
    }
    g_pauseRuntime.lastBufferCountSource = snap.bufferCountSource;
    g_pauseRuntime.lastSealerCountSource = snap.sealerCountSource;

    if (progress) {
        g_pauseRuntime.lastProgressMs = nowMs;
        if (g_pauseRuntime.mandatoryUnloadActive) {
            g_pauseRuntime.mandatoryUnloadLastProgressMs = nowMs;
        }
        g_pauseRuntime.progressObserved = true;
        Serial.print("COMMON: pause progress ");
        Serial.print(progressDiag);
        Serial.print(", substate=");
        Serial.println(commonPauseRuntimeSubstateName(conveyor, manipulator));
    }

    g_pauseRuntime.conveyorSnapshot.progressObserved = g_pauseRuntime.progressObserved;
    g_pauseRuntime.manipulatorSnapshot.progressObserved = g_pauseRuntime.progressObserved;
    return progress;
}

uint32_t commonPauseRuntimeArmingElapsedMs(uint32_t nowMs)
{
    if (g_pauseRuntime.armingSinceMs == 0) {
        return 0;
    }
    return static_cast<uint32_t>(nowMs - g_pauseRuntime.armingSinceMs);
}

uint32_t commonPauseRuntimeNoProgressMs(uint32_t nowMs)
{
    if (g_pauseRuntime.lastProgressMs == 0) {
        return 0;
    }
    return static_cast<uint32_t>(nowMs - g_pauseRuntime.lastProgressMs);
}

void commonPauseSendNodeCommand(uint8_t id, const String &command, const char *origin)
{
    if (!i2cSendManagedDeviceCommand(id, command, origin)) {
        Serial.print("COMMON warning: failed to send ");
        Serial.print(command);
        Serial.print(" to node ");
        Serial.println(id);
    }
}

void commonPauseSendArmCommands()
{
    if (g_pauseRuntime.currentEpoch == 0) {
        return;
    }
    const String command = "PAUSE ARM " + String(g_pauseRuntime.currentEpoch);
    if (!g_pauseRuntime.armSentConveyor) {
        commonPauseSendNodeCommand(CONVEYOR_ID, command, "common_pause_arm");
        g_pauseRuntime.armSentConveyor = true;
    }
    if (!g_pauseRuntime.armSentManipulator) {
        commonPauseSendNodeCommand(MANIPULATOR_ID, command, "common_pause_arm");
        g_pauseRuntime.armSentManipulator = true;
    }
}

void commonPausePollStatusCommands(uint32_t nowMs)
{
    if (g_pauseRuntime.currentEpoch == 0) {
        return;
    }
    if (g_pauseRuntime.lastStatusPollMs != 0 &&
        static_cast<uint32_t>(nowMs - g_pauseRuntime.lastStatusPollMs) < COMMON_PAUSE_STATUS_POLL_MS) {
        return;
    }

    g_pauseRuntime.lastStatusPollMs = nowMs;
    const String command = "PAUSE STATUS " + String(g_pauseRuntime.currentEpoch);
    commonPauseSendNodeCommand(CONVEYOR_ID, command, "common_pause_status");
    commonPauseSendNodeCommand(MANIPULATOR_ID, command, "common_pause_status");
}

void commonPauseSendReleaseCommands()
{
    if (g_pauseRuntime.currentEpoch == 0) {
        return;
    }
    const String command = "PAUSE RELEASE " + String(g_pauseRuntime.currentEpoch);
    if (!g_pauseRuntime.releaseSentConveyor) {
        commonPauseSendNodeCommand(CONVEYOR_ID, command, "common_pause_release");
        g_pauseRuntime.releaseSentConveyor = true;
    }
    if (!g_pauseRuntime.releaseSentManipulator) {
        commonPauseSendNodeCommand(MANIPULATOR_ID, command, "common_pause_release");
        g_pauseRuntime.releaseSentManipulator = true;
    }
}

void commonPauseEscalateToManualRecovery(const String &reason, const char *trigger)
{
    commonResetPauseStrictSnapshotTracking();
    commonPauseRuntimeClearMandatoryUnloadTracking();
    commonClearRefillPending();
    g_commonCycle.active = false;
    g_commonCycle.pauseRequested = false;
    g_commonCycle.pauseState = CommonPauseState::None;
    g_commonCycle.step3Seen = false;
    g_commonCycle.step3LaunchDone = false;
    g_commonCycle.parallelLaunchDone = false;
    g_commonCycle.currentCycleLoadsSealer = false;
    g_commonCycle.outfeedStartIssued = false;
    g_commonCycle.outfeedNotReadyObserved = false;
    g_commonCycle.runtimeState = CommonRuntimeState::ManualRecoveryRequired;
    g_commonCycle.abortDrainOutcome = CommonAbortDrainOutcome::NoProgress;
    g_commonCycle.abortRequiresManualRecovery = true;
    g_commonCycle.abortReason = reason;
    g_commonCycle.stage = CommonCycleStage::Idle;
    g_commonCycle.cycleId = 0;
    commonPauseRuntimeApplyState(CommonPauseRuntimeState::ManualRecoveryRequired);
    g_pauseRuntime.pauseCycleId = 0;
    g_pauseRuntime.armingSinceMs = 0;
    g_pauseRuntime.nodeReadyConfirmed = false;
    g_pauseRuntime.finalizeStarted = false;

    const bool watchdogTrigger =
        trigger == nullptr || trigger[0] == '\0' || strcmp(trigger, "watchdog") == 0;
    if (watchdogTrigger) {
        g_commonCycle.lastEvent = "COMMON: pause watchdog escalated to manual recovery (" + reason + ")";
    } else {
        g_commonCycle.lastEvent = "COMMON: pause manual recovery required (trigger=" +
            String(trigger) + ", " + reason + ")";
    }
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
}

void commonPauseRuntimeTick(uint32_t nowMs, const Rs485DeviceState &conveyor)
{
    if (commonManualRecoveryRequired()) {
        commonPauseRuntimeApplyState(CommonPauseRuntimeState::ManualRecoveryRequired);
        commonPauseRuntimeUpdateSnapshots(conveyor, nowMs);
        return;
    }

    if (!g_commonCycle.pauseRequested) {
        if (g_pauseRuntime.state != CommonPauseRuntimeState::None) {
            commonPauseRuntimeDeactivate();
        }
        return;
    }

    if (g_pauseRuntime.currentEpoch == 0 || g_pauseRuntime.pauseCycleId == 0) {
        commonPauseRuntimeBeginEpoch(nowMs);
    }

    commonPauseRuntimeApplyState(CommonPauseRuntimeState::PauseArming);
    commonPauseSendArmCommands();
    commonPausePollStatusCommands(nowMs);
    commonPauseRuntimeUpdateSnapshots(conveyor, nowMs);
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    (void)commonPauseRuntimeRecordProgress(nowMs, conveyor, manipulator);
    if (g_pauseRuntime.conveyorSnapshot.nodeStateFresh &&
        g_pauseRuntime.conveyorSnapshot.nodeState == CommonPauseNodeState::BlockedFault) {
        String faultText = "COMMON: pause blocked by node fault";
        faultText += " (node_state=blocked";
        faultText += ", conveyor_alarm=" + String(deviceStatusAlarm(conveyor) ? "yes" : "no");
        faultText += ", post7_manual_recovery=" +
            String(conveyorPost7ManualRecoveryRequired(conveyor) ? "yes" : "no");
        faultText += ")";
        Serial.println(faultText);
        commonPauseEscalateToManualRecovery(
            "pause_node_blocked state=blocked, conveyor_alarm=" +
            String(deviceStatusAlarm(conveyor) ? "yes" : "no") +
            ", post7_manual_recovery=" +
            String(conveyorPost7ManualRecoveryRequired(conveyor) ? "yes" : "no"),
            "node_fault");
        return;
    }
    if (!g_pauseRuntime.nodeReadyConfirmed &&
        g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed &&
        g_pauseRuntime.conveyorSnapshot.nodeState == CommonPauseNodeState::Ready) {
        g_pauseRuntime.nodeReadyConfirmed = true;
        commonSetEventIfChanged("COMMON: pause node ready confirmed");
    }
    const bool trusted = g_pauseRuntime.conveyorSnapshot.contractSnapshotTrusted;
    const bool bufOk =
        trusted &&
        g_pauseRuntime.conveyorSnapshot.bufferCountValid &&
        g_pauseRuntime.conveyorSnapshot.bufferCountFresh &&
        g_pauseRuntime.conveyorSnapshot.bufferCount == 0;
    const bool sealerOkForHold =
        g_pauseRuntime.conveyorSnapshot.sealerCountValid &&
        g_pauseRuntime.conveyorSnapshot.sealerCountFresh &&
        (g_pauseRuntime.conveyorSnapshot.sealerCount == 0 ||
         (g_pauseRuntime.conveyorSnapshot.sealerCount > 0 &&
          g_pauseRuntime.conveyorSnapshot.sealerResidualKnown));
    const bool fillLastBlockCompleted =
        commonPauseFillLastBlockCompleted(conveyor, manipulator);
    const bool drainIdle =
        commonPauseContractDrainIdle(conveyor, manipulator, nowMs);
    const bool canConfirmHold =
        trusted &&
        bufOk &&
        sealerOkForHold &&
        drainIdle &&
        g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed;

    if (fillLastBlockCompleted && !g_pauseRuntime.fillLastBlockCompletionLatched) {
        g_pauseRuntime.fillLastBlockCompletionLatched = true;
        Serial.print("COMMON: pause fill_last_block completed; ");
        Serial.print("gate=node_ready+trusted_buffer_empty+idle_tail+outfeed_ready");
        Serial.print(", pause_state=");
        Serial.print(commonPauseStateName(g_commonCycle.pauseState));
        Serial.print(", node_state=");
        Serial.print(commonPauseNodeStateName(g_pauseRuntime.conveyorSnapshot.nodeState));
        Serial.print(", buffer_count=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCount);
        Serial.print(", buffer_valid=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "yes" : "no");
        Serial.print(", buffer_fresh=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountFresh ? "yes" : "no");
        Serial.print(", buffer_source=");
        Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
        Serial.print(", sealer_count=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCount);
        Serial.print(", sealer_valid=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "yes" : "no");
        Serial.print(", sealer_fresh=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountFresh ? "yes" : "no");
        Serial.print(", sealer_source=");
        Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
        Serial.print(", drain_idle=");
        Serial.print(drainIdle ? "yes" : "no");
        Serial.print(", can_confirm_hold=");
        Serial.println(canConfirmHold ? "yes" : "no");
        if (!canConfirmHold) {
            commonPauseRuntimeRestartArmingWindow(
                nowMs,
                "fill_last_block_completed_waits_terminal_outcome");
        }
    }

    if (canConfirmHold) {
        const bool outcomePauseEmpty = g_pauseRuntime.conveyorSnapshot.sealerCount == 0;
        Serial.print("COMMON: pause terminal gate satisfied; ");
        Serial.print("trusted=");
        Serial.print(trusted ? "yes" : "no");
        Serial.print(", buffer_empty=");
        Serial.print(bufOk ? "yes" : "no");
        Serial.print(", sealer_ok=");
        Serial.print(sealerOkForHold ? "yes" : "no");
        Serial.print(", drain_idle=");
        Serial.print(drainIdle ? "yes" : "no");
        Serial.print(", node_epoch_confirmed=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "yes" : "no");
        Serial.print(", fill_last_block_completed=");
        Serial.print(fillLastBlockCompleted ? "yes" : "no");
        Serial.print(", buffer_count=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCount);
        Serial.print(", buffer_valid=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "yes" : "no");
        Serial.print(", buffer_fresh=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountFresh ? "yes" : "no");
        Serial.print(", buffer_source=");
        Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
        Serial.print(", sealer_count=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCount);
        Serial.print(", sealer_valid=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "yes" : "no");
        Serial.print(", sealer_fresh=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountFresh ? "yes" : "no");
        Serial.print(", sealer_source=");
        Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
        Serial.print(", sealer_residual_known=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.sealerResidualKnown ? "yes" : "no");
        Serial.print(", selected_outcome=");
        Serial.println(outcomePauseEmpty ? "pause_empty" : "pause_hold");
        if (g_pauseRuntime.conveyorSnapshot.sealerCount == 0) {
            commonPauseRuntimeApplyState(CommonPauseRuntimeState::PauseEmpty);
            commonSetEventIfChanged("COMMON: pause outcome=pause_empty");
        } else {
            commonPauseRuntimeApplyState(CommonPauseRuntimeState::PauseHold);
            commonSetEventIfChanged("COMMON: pause outcome=pause_hold");
        }
        commonPauseRuntimeClearMandatoryUnloadTracking();
        g_pauseRuntime.armingSinceMs = 0;
        commonPauseRuntimeUpdateSnapshots(conveyor, nowMs);
        return;
    }

    const bool fillLastBlockBlockedByResidual =
        g_pauseRuntime.finalizeStarted &&
        g_commonCycle.pauseState == CommonPauseState::FillLastBlock &&
        trusted &&
        g_pauseRuntime.conveyorSnapshot.bufferCountValid &&
        g_pauseRuntime.conveyorSnapshot.bufferCountFresh &&
        g_pauseRuntime.conveyorSnapshot.bufferCount > 0 &&
        !conveyorNextBatchReadyForCommon(conveyor, g_commonCycle.lastConsumedBatchSeq) &&
        !conveyorProgram1Active(conveyor) &&
        !deviceStatusBusy(conveyor) &&
        !deviceStatusBusy(manipulator) &&
        !conveyorStep2Active(conveyor) &&
        !conveyorVfdTimedRunActive(conveyor);

    if (fillLastBlockBlockedByResidual) {
        if (g_pauseRuntime.fillLastBlockBlockedSinceMs == 0) {
            g_pauseRuntime.fillLastBlockBlockedSinceMs = nowMs;
            g_pauseRuntime.fillLastBlockBlockedLogged = false;
        }
        const uint32_t blockedElapsedMs =
            static_cast<uint32_t>(nowMs - g_pauseRuntime.fillLastBlockBlockedSinceMs);
        if (!g_pauseRuntime.fillLastBlockBlockedLogged) {
            g_pauseRuntime.fillLastBlockBlockedLogged = true;
            Serial.print("COMMON: pause fill_last_block blocked; ");
            Serial.print("reason=trusted_non_empty_buffer_without_refill_path");
            Serial.print(", buffer_count=");
            Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCount);
            Serial.print(", buffer_valid=");
            Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "yes" : "no");
            Serial.print(", buffer_fresh=");
            Serial.print(g_pauseRuntime.conveyorSnapshot.bufferCountFresh ? "yes" : "no");
            Serial.print(", buffer_source=");
            Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
            Serial.print(", sealer_count=");
            Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCount);
            Serial.print(", sealer_valid=");
            Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "yes" : "no");
            Serial.print(", sealer_fresh=");
            Serial.print(g_pauseRuntime.conveyorSnapshot.sealerCountFresh ? "yes" : "no");
            Serial.print(", sealer_source=");
            Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
            Serial.print(", next_batch_ready=no");
            Serial.print(", p1_active=no");
            Serial.print(", drain_idle=");
            Serial.print(drainIdle ? "yes" : "no");
            Serial.print(", fill_last_block_completed=");
            Serial.print(fillLastBlockCompleted ? "yes" : "no");
            Serial.print(", decision_timeout_ms=");
            Serial.println(COMMON_PAUSE_FILL_LAST_BLOCK_DECISION_MS);
            commonSetEventIfChanged(
                "COMMON: pause_fill_last_block blocked, terminal decision pending");
        }
        if (blockedElapsedMs >= COMMON_PAUSE_FILL_LAST_BLOCK_DECISION_MS) {
            String reason =
                "pause_finalize_gate_unsatisfied cause=fill_last_block_non_empty_buffer";
            reason += ", decision_elapsed_ms=" + String(blockedElapsedMs);
            reason += ", decision_timeout_ms=" +
                String(COMMON_PAUSE_FILL_LAST_BLOCK_DECISION_MS);
            reason += ", substate=" + String(commonPauseRuntimeSubstateName(conveyor, manipulator));
            reason += ", hold_gate_trusted=" + String(trusted ? "yes" : "no");
            reason += ", hold_gate_buffer_empty=" + String(bufOk ? "yes" : "no");
            reason += ", hold_gate_sealer_ok=" + String(sealerOkForHold ? "yes" : "no");
            reason += ", hold_gate_drain_idle=" + String(drainIdle ? "yes" : "no");
            reason += ", hold_gate_node_epoch=" +
                String(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "yes" : "no");
            reason += ", hold_gate_fill_last_block_completed=" +
                String(fillLastBlockCompleted ? "yes" : "no");
            reason += ", buffer_count=" + String(g_pauseRuntime.conveyorSnapshot.bufferCount);
            reason += ", buffer_valid=" +
                String(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "yes" : "no");
            reason += ", buffer_fresh=" +
                String(g_pauseRuntime.conveyorSnapshot.bufferCountFresh ? "yes" : "no");
            reason += ", buffer_source=" +
                String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
            reason += ", sealer_count=" + String(g_pauseRuntime.conveyorSnapshot.sealerCount);
            reason += ", sealer_valid=" +
                String(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "yes" : "no");
            reason += ", sealer_fresh=" +
                String(g_pauseRuntime.conveyorSnapshot.sealerCountFresh ? "yes" : "no");
            reason += ", sealer_source=" +
                String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
            reason += ", sealer_residual_known=" +
                String(g_pauseRuntime.conveyorSnapshot.sealerResidualKnown ? "yes" : "no");
            commonPauseEscalateToManualRecovery(reason, "finalize_gate");
            return;
        }
    } else {
        g_pauseRuntime.fillLastBlockBlockedSinceMs = 0;
        g_pauseRuntime.fillLastBlockBlockedLogged = false;
    }

    if (g_pauseRuntime.armingSinceMs == 0) {
        g_pauseRuntime.armingSinceMs = nowMs;
    }

    const uint32_t armingElapsedMs = commonPauseRuntimeArmingElapsedMs(nowMs);
    const uint32_t noProgressMs = commonPauseRuntimeNoProgressMs(nowMs);
    const bool mandatoryUnloadInFlight =
        commonPauseRuntimeMandatoryUnloadInFlight(conveyor, manipulator);
    const bool mandatoryUnloadPolicyActive =
        commonPauseRuntimeMandatoryUnloadPolicyActive();
    const bool forceDrainStableWaitPolicyActive =
        commonPauseRuntimeForceDrainStableWaitPolicyActive();
    const uint32_t armingTimeoutLimitMs = mandatoryUnloadPolicyActive
        ? COMMON_PAUSE_MANDATORY_UNLOAD_TIMEOUT_MS
        : COMMON_PAUSE_ARMING_TIMEOUT_MS;
    const bool noProgressGuardActive = mandatoryUnloadPolicyActive;
    const char *watchdogProfile = commonPauseRuntimeWatchdogProfileName(
        mandatoryUnloadPolicyActive);
    const char *watchdogPolicyReason = commonPauseRuntimeWatchdogPolicyReason(
        mandatoryUnloadPolicyActive,
        mandatoryUnloadInFlight);
    if (!g_pauseRuntime.watchdogPolicyKnown ||
        g_pauseRuntime.lastWatchdogPolicyMandatoryUnload != mandatoryUnloadPolicyActive ||
        g_pauseRuntime.lastWatchdogPolicyMandatoryInFlight != mandatoryUnloadInFlight ||
        g_pauseRuntime.lastWatchdogPolicyForceDrainStableWait != forceDrainStableWaitPolicyActive) {
        g_pauseRuntime.watchdogPolicyKnown = true;
        g_pauseRuntime.lastWatchdogPolicyMandatoryUnload = mandatoryUnloadPolicyActive;
        g_pauseRuntime.lastWatchdogPolicyMandatoryInFlight = mandatoryUnloadInFlight;
        g_pauseRuntime.lastWatchdogPolicyForceDrainStableWait = forceDrainStableWaitPolicyActive;
        Serial.print("COMMON: pause watchdog policy selected, profile=");
        Serial.print(watchdogProfile);
        Serial.print(", reason=");
        Serial.print(watchdogPolicyReason);
        Serial.print(", arming_timeout_limit_ms=");
        Serial.print(armingTimeoutLimitMs);
        Serial.print(", watchdog_active=");
        Serial.print(commonPauseRuntimeWatchdogActive() ? "yes" : "no");
        Serial.print(", force_drain_stable_wait_policy=");
        Serial.print(forceDrainStableWaitPolicyActive ? "yes" : "no");
        Serial.print(", no_progress_guard=");
        Serial.print(noProgressGuardActive ? "hold" : "normal");
        Serial.print(", hold_gate_trusted=");
        Serial.print(trusted ? "yes" : "no");
        Serial.print(", hold_gate_buffer_empty=");
        Serial.print(bufOk ? "yes" : "no");
        Serial.print(", hold_gate_sealer_ok=");
        Serial.print(sealerOkForHold ? "yes" : "no");
        Serial.print(", hold_gate_drain_idle=");
        Serial.print(drainIdle ? "yes" : "no");
        Serial.print(", hold_gate_node_epoch=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "yes" : "no");
        Serial.print(", hold_gate_fill_last_block_completed=");
        Serial.print(fillLastBlockCompleted ? "yes" : "no");
        Serial.print(", buffer_source=");
        Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
        Serial.print(", sealer_source=");
        Serial.println(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
    }
    if (g_pauseRuntime.lastWatchdogWaitLogMs == 0 ||
        static_cast<uint32_t>(nowMs - g_pauseRuntime.lastWatchdogWaitLogMs) >=
            COMMON_PAUSE_ARMING_WAIT_LOG_MS) {
        g_pauseRuntime.lastWatchdogWaitLogMs = nowMs;
        Serial.print("COMMON: pause watchdog wait, elapsed_ms=");
        Serial.print(armingElapsedMs);
        Serial.print(", no_progress_ms=");
        Serial.print(noProgressMs);
        Serial.print(", arming_timeout_limit_ms=");
        Serial.print(armingTimeoutLimitMs);
        Serial.print(", substate=");
        Serial.print(commonPauseRuntimeSubstateName(conveyor, manipulator));
        Serial.print(", mandatory_unload_active=");
        Serial.print(g_pauseRuntime.mandatoryUnloadActive ? "yes" : "no");
        Serial.print(", mandatory_unload_in_flight=");
        Serial.print(mandatoryUnloadInFlight ? "yes" : "no");
        Serial.print(", watchdog_active=");
        Serial.print(commonPauseRuntimeWatchdogActive() ? "yes" : "no");
        Serial.print(", watchdog_profile=");
        Serial.print(watchdogProfile);
        Serial.print(", watchdog_policy_reason=");
        Serial.print(watchdogPolicyReason);
        Serial.print(", force_drain_stable_wait_policy=");
        Serial.print(forceDrainStableWaitPolicyActive ? "yes" : "no");
        Serial.print(", no_progress_guard=");
        Serial.print(noProgressGuardActive ? "hold" : "normal");
        Serial.print(", hold_gate_trusted=");
        Serial.print(trusted ? "yes" : "no");
        Serial.print(", hold_gate_buffer_empty=");
        Serial.print(bufOk ? "yes" : "no");
        Serial.print(", hold_gate_sealer_ok=");
        Serial.print(sealerOkForHold ? "yes" : "no");
        Serial.print(", hold_gate_drain_idle=");
        Serial.print(drainIdle ? "yes" : "no");
        Serial.print(", hold_gate_node_epoch=");
        Serial.print(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "yes" : "no");
        Serial.print(", hold_gate_fill_last_block_completed=");
        Serial.print(fillLastBlockCompleted ? "yes" : "no");
        Serial.print(", buffer_source=");
        Serial.print(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
        Serial.print(", sealer_source=");
        Serial.println(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
    }

    const bool armingTimeoutExceeded = armingElapsedMs >= armingTimeoutLimitMs;
    const bool noProgressTimeoutExceeded =
        !noProgressGuardActive && noProgressMs >= COMMON_PAUSE_ARMING_NO_PROGRESS_MS;
    if (armingTimeoutExceeded || noProgressTimeoutExceeded) {
        String reason = "pause_arming_timeout elapsed_ms=" + String(armingElapsedMs) +
            ", no_progress_ms=" + String(noProgressMs);
        reason += ", arming_timeout_limit_ms=" + String(armingTimeoutLimitMs);
        reason += ", substate=" + String(commonPauseRuntimeSubstateName(conveyor, manipulator));
        reason += ", mandatory_unload_active=" + String(g_pauseRuntime.mandatoryUnloadActive ? "yes" : "no");
        reason += ", mandatory_unload_in_flight=" + String(mandatoryUnloadInFlight ? "yes" : "no");
        reason += ", watchdog_policy_reason=" + String(watchdogPolicyReason);
        reason += ", no_progress_guard=" + String(noProgressGuardActive ? "hold" : "normal");
        reason += ", hold_gate_trusted=" + String(trusted ? "yes" : "no");
        reason += ", hold_gate_buffer_empty=" + String(bufOk ? "yes" : "no");
        reason += ", hold_gate_sealer_ok=" + String(sealerOkForHold ? "yes" : "no");
        reason += ", hold_gate_drain_idle=" + String(drainIdle ? "yes" : "no");
        reason += ", hold_gate_node_epoch=" +
            String(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "yes" : "no");
        reason += ", hold_gate_fill_last_block_completed=" +
            String(fillLastBlockCompleted ? "yes" : "no");
        reason += ", buffer_source=" +
            String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource));
        reason += ", sealer_source=" +
            String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource));
        commonPauseEscalateToManualRecovery(reason, "watchdog");
    }
}

bool conveyorBusyLikelyBlocksProgram1Start(const Rs485DeviceState &conveyor)
{
    if (!deviceStatusBusy(conveyor)) {
        return false;
    }

    const bool post7RelatedBusy =
        conveyorStep2Active(conveyor) ||
        conveyorVfdTimedRunActive(conveyor) ||
        conveyorSealerBusy(conveyor);
    return !post7RelatedBusy;
}

CommonRefillLaunchResult commonTryStartConveyorRefill(const String &reason, bool allowBusySkip)
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    if (!conveyor.online || !conveyor.protocolOk) {
        g_lastCommandResult = "COMMON failed: conveyor offline";
        Serial.println(g_lastCommandResult);
        commonClearRefillPending();
        return CommonRefillLaunchResult::Failed;
    }
    if (deviceStatusAlarm(conveyor) || conveyorPost7ManualRecoveryRequired(conveyor)) {
        commonClearRefillPending();
        String faultText = "COMMON: refill blocked by conveyor fault (" + reason + ")";
        faultText += ", alarm=" + String(deviceStatusAlarm(conveyor) ? "yes" : "no");
        faultText += ", manual_recovery=" + String(conveyorPost7ManualRecoveryRequired(conveyor) ? "yes" : "no");
        commonSetEventIfChanged(faultText);
        return CommonRefillLaunchResult::Failed;
    }

    const uint16_t batchSeq = conveyorBatchSeq(conveyor);
    const bool hasReadyBatch = conveyorBatchReady(conveyor);
    const bool nextBatchReady = conveyorNextBatchReadyForCommon(
        conveyor,
        g_commonCycle.lastConsumedBatchSeq);
    const bool staleReadyBatch = conveyorReadyBatchIsStaleForCommon(
        conveyor,
        g_commonCycle.lastConsumedBatchSeq);

    if (nextBatchReady) {
        commonClearRefillPending();
        commonSetEventIfChanged("COMMON: refill skipped because already ready (new batch, " +
                                reason +
                                ", seq=" + String(batchSeq) +
                                ", consumed=" + String(g_commonCycle.lastConsumedBatchSeq) + ")");
        return CommonRefillLaunchResult::AlreadyReady;
    }

    if (staleReadyBatch) {
        commonSetEventIfChanged("COMMON: ready but stale batch (" + reason +
                                ", seq=" + String(batchSeq) +
                                ", consumed=" + String(g_commonCycle.lastConsumedBatchSeq) + ")");
    } else if (hasReadyBatch && batchSeq == 0U) {
        commonSetEventIfChanged("COMMON: batch ready without sequence (" + reason +
                                "), treating as not-next");
    }

    if (conveyorProgram1Active(conveyor)) {
        commonClearRefillPending();
        commonSetEventIfChanged("COMMON: refill skipped because program1 already active (" + reason +
                                ", ready=" + String(hasReadyBatch ? "yes" : "no") +
                                ", seq=" + String(batchSeq) + ")");
        return CommonRefillLaunchResult::AlreadyProgram1Active;
    }

    const uint32_t nowMs = millis();
    if (g_commonCycle.refillStartPending) {
        if (commonRefillPendingFresh(nowMs)) {
            return CommonRefillLaunchResult::Pending;
        }
        commonClearRefillPending();
    }

    if (allowBusySkip && conveyorBusyLikelyBlocksProgram1Start(conveyor)) {
        commonSetEventIfChanged("COMMON: refill skipped because conveyor busy (" + reason + ")");
        return CommonRefillLaunchResult::SkippedBusy;
    }

    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "1", "common_conv_p1")) {
        g_lastCommandResult = "COMMON failed: CONV 1 send error";
        Serial.println(g_lastCommandResult);
        commonClearRefillPending();
        return CommonRefillLaunchResult::Failed;
    }

    g_commonCycle.refillStartPending = true;
    g_commonCycle.refillStartRequestedMs = nowMs;
    if (staleReadyBatch) {
        commonSetEventIfChanged("COMMON: need refill because last ready batch already consumed (" +
                                reason +
                                ", stale_seq=" + String(batchSeq) +
                                ", consumed=" + String(g_commonCycle.lastConsumedBatchSeq) +
                                "); refill start requested");
    } else {
        commonSetEventIfChanged("COMMON: refill start requested (" + reason + ")");
    }
    return CommonRefillLaunchResult::Requested;
}

bool commonAbortReasonSuggestsManualRecovery(const String &reason)
{
    String upper = reason;
    upper.toUpperCase();
    return upper.indexOf("ALARM") >= 0 ||
        upper.indexOf("MANUAL RECOVERY") >= 0 ||
        upper.indexOf("TIMEOUT") >= 0;
}

void commonResetAfterAbortFinalization()
{
    g_commonCycle.active = false;
    g_commonCycle.pauseRequested = false;
    g_commonCycle.pauseState = CommonPauseState::None;
    g_commonCycle.step3Seen = false;
    g_commonCycle.step3LaunchDone = false;
    g_commonCycle.parallelLaunchDone = false;
    g_commonCycle.refillStartPending = false;
    g_commonCycle.currentCycleLoadsSealer = false;
    g_commonCycle.outfeedStartIssued = false;
    g_commonCycle.outfeedNotReadyObserved = false;
    g_commonCycle.refillStartRequestedMs = 0;
    g_commonCycle.outfeedStartedMs = 0;
    g_commonCycle.pauseStrictSnapshotSinceMs = 0;
    g_commonCycle.pauseStrictSnapshotRetryMs = 0;
    g_commonCycle.post7SealerCompletionSeqBase = 0;
    g_commonCycle.pauseStrictSnapshotForceDrain = false;
    g_commonCycle.pauseForceDrainApplied = false;
    g_commonCycle.pauseStrictGateLatched = false;
    g_commonCycle.pauseStrictRollbackBlockedLogged = false;
    g_commonCycle.sealerDonePendingUnload = false;
    g_commonCycle.abortDrainActive = false;
    g_commonCycle.abortFsinvPending = false;
    g_commonCycle.abortRequiresManualRecovery = false;
    g_commonCycle.abortDrainLastOutfeedReady = false;
    g_commonCycle.abortDrainLastStep2Active = false;
    g_commonCycle.abortDrainLastVfdActive = false;
    g_commonCycle.abortDrainStartedMs = 0;
    g_commonCycle.abortDrainLastProgressMs = 0;
    g_commonCycle.abortDrainLastWaitLogMs = 0;
    g_commonCycle.abortDrainLastStep2Seq = 0;
    g_commonCycle.abortDrainLastSealerSeq = 0;
    g_commonCycle.abortDrainSettleActive = false;
    g_commonCycle.abortDrainSettleSinceMs = 0;
    g_commonCycle.abortDrainSettleChecks = 0;
    g_commonCycle.abortReason = "";
    g_commonCycle.stage = CommonCycleStage::Idle;
    g_commonCycle.cycleId = 0;
    commonPauseRuntimeDeactivate();
}

void commonFinalizeAbortState(const String &reason,
                              CommonAbortDrainOutcome outcome,
                              bool manualRecoveryRequired,
                              bool sendFsinvNow,
                              const char *fsinvOrigin,
                              const String &details = "")
{
    const bool fsinvDeferred = g_commonCycle.abortFsinvPending;
    if (sendFsinvNow || g_commonCycle.abortFsinvPending) {
        if (fsinvDeferred) {
            Serial.println("COMMON: sending deferred FSINV after abort supervision outcome.");
        }
        if (!i2cSendManagedDeviceCommand(
                CONVEYOR_ID,
                "FSINV",
                (fsinvOrigin == nullptr || fsinvOrigin[0] == '\0')
                    ? "common_abort_fsinv"
                    : fsinvOrigin)) {
            Serial.println("COMMON warning: CONV FSINV send error");
        } else {
            Serial.println("COMMON: FSINV sent.");
        }
    }

    commonResetAfterAbortFinalization();
    g_commonCycle.runtimeState = manualRecoveryRequired
        ? CommonRuntimeState::ManualRecoveryRequired
        : CommonRuntimeState::CommonAborted;
    g_commonCycle.abortDrainOutcome = outcome;
    g_commonCycle.abortRequiresManualRecovery = manualRecoveryRequired;
    g_commonCycle.abortReason = reason;
    if (manualRecoveryRequired) {
        commonPauseRuntimeApplyState(CommonPauseRuntimeState::ManualRecoveryRequired);
    } else {
        commonPauseRuntimeDeactivate();
    }

    String finalEvent = "COMMON: abort outcome=" + String(commonAbortDrainOutcomeName(outcome));
    finalEvent += manualRecoveryRequired ? ", manual recovery required" : ", common aborted";
    if (!details.isEmpty()) {
        finalEvent += " (" + details + ")";
    }

    g_commonCycle.lastEvent = finalEvent;
    g_lastCommandResult = finalEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
}

CommonAbortPost7Decision commonAbortDecidePost7Action(const Rs485DeviceState &conveyor,
                                                      CommonAbortPost7Policy policy,
                                                      String &diag)
{
    diag = "";
    if (policy != CommonAbortPost7Policy::AllowDrainToSafe || !g_commonCycle.outfeedStartIssued) {
        diag = "policy=force_stop_or_post7_not_started";
        return CommonAbortPost7Decision::ForceStop;
    }

    const bool conveyorOnline = conveyor.online && conveyor.protocolOk;
    if (!conveyorOnline) {
        diag = "conveyor_offline";
        return CommonAbortPost7Decision::ForceStop;
    }

    const bool outfeedReady = conveyorOutfeedReadyForBatch(conveyor);
    const bool outfeedStep2Active = conveyorStep2Active(conveyor);
    const bool outfeedVfdActive = conveyorVfdTimedRunActive(conveyor);
    const uint8_t outfeedStep2Seq = conveyorStep2CompletionSeq(conveyor);
    const bool outfeedStep2SeqAdvanced = outfeedStep2Seq != g_commonCycle.outfeedStep2SeqBase;
    const uint8_t sealerSeq = conveyorSealerCompletionSeq(conveyor);
    const bool sealerSeqAdvanced = sealerSeq != g_commonCycle.post7SealerCompletionSeqBase;
    const bool sealerDoneForJob = !g_commonCycle.currentCycleLoadsSealer || sealerSeqAdvanced;
    const bool tailInactive = !outfeedStep2Active && !outfeedVfdActive;

    const bool explicitDrainMarkers =
        outfeedStep2Active ||
        outfeedVfdActive ||
        outfeedStep2SeqAdvanced ||
        sealerSeqAdvanced;
    const bool alreadySettled = outfeedReady &&
        tailInactive &&
        sealerDoneForJob &&
        (g_commonCycle.outfeedNotReadyObserved || outfeedStep2SeqAdvanced || sealerSeqAdvanced);
    const bool drainWindow = explicitDrainMarkers && !alreadySettled;

    diag = "outfeed_ready=" + String(outfeedReady ? "yes" : "no");
    diag += ", outfeed_seen_not_ready=" + String(g_commonCycle.outfeedNotReadyObserved ? "yes" : "no");
    diag += ", outfeed_step2=" + String(outfeedStep2Active ? "yes" : "no");
    diag += ", outfeed_vfd=" + String(outfeedVfdActive ? "yes" : "no");
    diag += ", tail_inactive=" + String(tailInactive ? "yes" : "no");
    diag += ", outfeed_seq=" + String(outfeedStep2Seq);
    diag += ", outfeed_seq_base=" + String(g_commonCycle.outfeedStep2SeqBase);
    diag += ", sealer_seq=" + String(sealerSeq);
    diag += ", sealer_seq_base=" + String(g_commonCycle.post7SealerCompletionSeqBase);
    diag += ", sealer_seq_advanced=" + String(sealerSeqAdvanced ? "yes" : "no");
    diag += ", sealer_done_for_job=" + String(sealerDoneForJob ? "yes" : "no");

    if (alreadySettled) {
        return CommonAbortPost7Decision::SkipAlreadySettled;
    }
    if (drainWindow) {
        return CommonAbortPost7Decision::SkipDrainToSafe;
    }
    return CommonAbortPost7Decision::ForceStop;
}

void commonBeginAbortDrainSupervision(const String &reason,
                                      bool manualRecoveryRequired,
                                      const Rs485DeviceState &conveyor,
                                      const String &diag)
{
    const uint32_t nowMs = millis();
    const bool conveyorOnline = conveyor.online && conveyor.protocolOk;
    const bool outfeedReady = conveyorOnline && conveyorOutfeedReadyForBatch(conveyor);

    g_commonCycle.active = false;
    g_commonCycle.pauseRequested = false;
    g_commonCycle.pauseState = CommonPauseState::None;
    g_commonCycle.step3Seen = false;
    g_commonCycle.step3LaunchDone = false;
    g_commonCycle.parallelLaunchDone = false;
    g_commonCycle.refillStartPending = false;
    g_commonCycle.refillStartRequestedMs = 0;
    g_commonCycle.pauseStrictSnapshotSinceMs = 0;
    g_commonCycle.pauseStrictSnapshotRetryMs = 0;
    g_commonCycle.pauseStrictSnapshotForceDrain = false;
    g_commonCycle.pauseForceDrainApplied = false;
    g_commonCycle.pauseStrictGateLatched = false;
    g_commonCycle.pauseStrictRollbackBlockedLogged = false;
    g_commonCycle.stage = CommonCycleStage::Idle;
    g_commonCycle.cycleId = 0;
    g_commonCycle.runtimeState = CommonRuntimeState::CommonAborted;
    g_commonCycle.abortDrainOutcome = CommonAbortDrainOutcome::InProgress;
    g_commonCycle.abortDrainActive = true;
    g_commonCycle.abortFsinvPending = true;
    g_commonCycle.abortRequiresManualRecovery = manualRecoveryRequired;
    g_commonCycle.abortReason = reason;
    g_commonCycle.abortDrainStartedMs = nowMs;
    g_commonCycle.abortDrainLastProgressMs = nowMs;
    g_commonCycle.abortDrainLastWaitLogMs = nowMs;
    g_commonCycle.abortDrainLastOutfeedReady = outfeedReady;
    g_commonCycle.abortDrainLastStep2Active = conveyorOnline && conveyorStep2Active(conveyor);
    g_commonCycle.abortDrainLastVfdActive = conveyorOnline && conveyorVfdTimedRunActive(conveyor);
    g_commonCycle.abortDrainLastStep2Seq = conveyorOnline ? conveyorStep2CompletionSeq(conveyor) : 0U;
    g_commonCycle.abortDrainLastSealerSeq = conveyorOnline ? conveyorSealerCompletionSeq(conveyor) : 0U;
    g_commonCycle.abortDrainSettleActive = false;
    g_commonCycle.abortDrainSettleSinceMs = 0;
    g_commonCycle.abortDrainSettleChecks = 0;
    if (!outfeedReady) {
        g_commonCycle.outfeedNotReadyObserved = true;
    }
    commonPauseRuntimeDeactivate();

    String event = "COMMON: abort keeps POST7 drain-to-safe under supervision, FSINV deferred";
    if (!diag.isEmpty()) {
        event += " (" + diag + ")";
    }
    g_commonCycle.lastEvent = event;
    g_lastCommandResult = event;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
}

void processCommonAbortDrainSupervision(uint32_t nowMs, const Rs485DeviceState &conveyor)
{
    if (!g_commonCycle.abortDrainActive) {
        return;
    }

    const bool conveyorOnline = conveyor.online && conveyor.protocolOk;
    if (!conveyorOnline) {
        const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainStartedMs);
        commonFinalizeAbortState(
            g_commonCycle.abortReason,
            CommonAbortDrainOutcome::LostSupervision,
            true,
            true,
            "common_abort_drain_fsinv",
            "elapsed_ms=" + String(elapsedMs));
        return;
    }

    const bool outfeedReady = conveyorOutfeedReadyForBatch(conveyor);
    const bool outfeedStep2Active = conveyorStep2Active(conveyor);
    const bool outfeedVfdActive = conveyorVfdTimedRunActive(conveyor);
    const uint8_t outfeedStep2Seq = conveyorStep2CompletionSeq(conveyor);
    const uint8_t sealerSeq = conveyorSealerCompletionSeq(conveyor);
    const bool sealerSeqAdvanced = sealerSeq != g_commonCycle.post7SealerCompletionSeqBase;
    const bool sealerDoneForJob = !g_commonCycle.currentCycleLoadsSealer || sealerSeqAdvanced;
    if (!g_commonCycle.outfeedNotReadyObserved && !outfeedReady) {
        g_commonCycle.outfeedNotReadyObserved = true;
    }

    bool progress = false;
    String progressDiag;
    if (outfeedReady != g_commonCycle.abortDrainLastOutfeedReady) {
        progress = true;
        progressDiag += "outfeed_ready=" + String(outfeedReady ? "yes" : "no");
    }
    if (outfeedStep2Active && !g_commonCycle.abortDrainLastStep2Active) {
        if (!progressDiag.isEmpty()) {
            progressDiag += ", ";
        }
        progress = true;
        progressDiag += "step2_active_rise";
    }
    if (outfeedVfdActive && !g_commonCycle.abortDrainLastVfdActive) {
        if (!progressDiag.isEmpty()) {
            progressDiag += ", ";
        }
        progress = true;
        progressDiag += "vfd_active_rise";
    }
    if (outfeedStep2Seq != g_commonCycle.abortDrainLastStep2Seq) {
        if (!progressDiag.isEmpty()) {
            progressDiag += ", ";
        }
        progress = true;
        progressDiag += "step2_seq=" + String(outfeedStep2Seq);
    }
    if (sealerSeq != g_commonCycle.abortDrainLastSealerSeq) {
        if (!progressDiag.isEmpty()) {
            progressDiag += ", ";
        }
        progress = true;
        progressDiag += "sealer_seq=" + String(sealerSeq);
    }

    g_commonCycle.abortDrainLastOutfeedReady = outfeedReady;
    g_commonCycle.abortDrainLastStep2Active = outfeedStep2Active;
    g_commonCycle.abortDrainLastVfdActive = outfeedVfdActive;
    g_commonCycle.abortDrainLastStep2Seq = outfeedStep2Seq;
    g_commonCycle.abortDrainLastSealerSeq = sealerSeq;

    if (progress) {
        g_commonCycle.abortDrainLastProgressMs = nowMs;
        Serial.print("COMMON: drain progress ");
        Serial.println(progressDiag);
    }

    if (deviceStatusAlarm(conveyor) || conveyorPost7ManualRecoveryRequired(conveyor)) {
        String details = "conveyor_alarm=" + String(deviceStatusAlarm(conveyor) ? "yes" : "no");
        details += ", manual_recovery_latch=" +
            String(conveyorPost7ManualRecoveryRequired(conveyor) ? "yes" : "no");
        commonFinalizeAbortState(
            g_commonCycle.abortReason,
            CommonAbortDrainOutcome::LocalFault,
            true,
            true,
            "common_abort_drain_fsinv",
            details);
        return;
    }

    const bool safeFinalCandidate = outfeedReady &&
        sealerDoneForJob &&
        !outfeedStep2Active &&
        !outfeedVfdActive;
    if (safeFinalCandidate) {
        if (!g_commonCycle.abortDrainSettleActive) {
            g_commonCycle.abortDrainSettleActive = true;
            g_commonCycle.abortDrainSettleSinceMs = nowMs;
            g_commonCycle.abortDrainSettleChecks = 1;
            Serial.print("COMMON: drain settle started");
            Serial.print(", outfeed_ready=");
            Serial.print(outfeedReady ? "yes" : "no");
            Serial.print(", sealer_done_for_job=");
            Serial.print(sealerDoneForJob ? "yes" : "no");
            Serial.print(", step2_active=");
            Serial.print(outfeedStep2Active ? "yes" : "no");
            Serial.print(", vfd_active=");
            Serial.println(outfeedVfdActive ? "yes" : "no");
        } else if (g_commonCycle.abortDrainSettleChecks < 0xFFU) {
            g_commonCycle.abortDrainSettleChecks++;
        }

        const uint32_t settleElapsedMs = static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainSettleSinceMs);
        const bool settleConfirmedByTime = settleElapsedMs >= COMMON_ABORT_DRAIN_SETTLE_MS;
        const bool settleConfirmedByChecks = g_commonCycle.abortDrainSettleChecks >= COMMON_ABORT_DRAIN_SETTLE_MIN_CHECKS;
        if (settleConfirmedByTime || settleConfirmedByChecks) {
            commonFinalizeAbortState(
                g_commonCycle.abortReason,
                CommonAbortDrainOutcome::DrainedToSafe,
                g_commonCycle.abortRequiresManualRecovery,
                true,
                "common_abort_drain_fsinv",
                "outfeed_ready=yes, sealer_done_for_job=yes, step2_active=no, vfd_active=no, settle_elapsed_ms=" +
                    String(settleElapsedMs) +
                    ", settle_checks=" + String(g_commonCycle.abortDrainSettleChecks));
            return;
        }
    } else if (g_commonCycle.abortDrainSettleActive) {
        g_commonCycle.abortDrainSettleActive = false;
        g_commonCycle.abortDrainSettleSinceMs = 0;
        g_commonCycle.abortDrainSettleChecks = 0;
        Serial.println("COMMON: drain settle reset because activity returned");
    }

    const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainStartedMs);
    const uint32_t noProgressMs = static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainLastProgressMs);
    if (noProgressMs >= COMMON_ABORT_DRAIN_NO_PROGRESS_MS) {
        commonFinalizeAbortState(
            g_commonCycle.abortReason,
            CommonAbortDrainOutcome::NoProgress,
            true,
            true,
            "common_abort_drain_fsinv",
            "elapsed_ms=" + String(elapsedMs) + ", no_progress_ms=" + String(noProgressMs));
        return;
    }
    if (elapsedMs >= COMMON_ABORT_DRAIN_TIMEOUT_MS) {
        commonFinalizeAbortState(
            g_commonCycle.abortReason,
            CommonAbortDrainOutcome::Timeout,
            true,
            true,
            "common_abort_drain_fsinv",
            "elapsed_ms=" + String(elapsedMs) + ", outfeed_ready=" + String(outfeedReady ? "yes" : "no"));
        return;
    }

    if (static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainLastWaitLogMs) >=
        COMMON_ABORT_DRAIN_WAIT_LOG_MS) {
        g_commonCycle.abortDrainLastWaitLogMs = nowMs;
        Serial.print("COMMON: drain supervision wait, elapsed_ms=");
        Serial.print(elapsedMs);
        Serial.print(", no_progress_ms=");
        Serial.print(noProgressMs);
        Serial.print(", outfeed_ready=");
        Serial.print(outfeedReady ? "yes" : "no");
        Serial.print(", step2_active=");
        Serial.print(outfeedStep2Active ? "yes" : "no");
        Serial.print(", vfd_active=");
        Serial.print(outfeedVfdActive ? "yes" : "no");
        Serial.print(", step2_seq=");
        Serial.print(outfeedStep2Seq);
        Serial.print(", step2_seq_base=");
        Serial.print(g_commonCycle.outfeedStep2SeqBase);
        Serial.print(", sealer_seq=");
        Serial.print(sealerSeq);
        Serial.print(", sealer_seq_base=");
        Serial.print(g_commonCycle.post7SealerCompletionSeqBase);
        Serial.print(", sealer_done_for_job=");
        Serial.print(sealerDoneForJob ? "yes" : "no");
        Serial.print(", settle_active=");
        Serial.print(g_commonCycle.abortDrainSettleActive ? "yes" : "no");
        Serial.print(", settle_checks=");
        Serial.print(g_commonCycle.abortDrainSettleChecks);
        Serial.print(", settle_elapsed_ms=");
        Serial.println(g_commonCycle.abortDrainSettleActive
            ? static_cast<uint32_t>(nowMs - g_commonCycle.abortDrainSettleSinceMs)
            : 0U);
    }
}

void abortCommonCycle(const String &reason,
                      CommonAbortPost7Policy post7Policy = CommonAbortPost7Policy::ForceStop)
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    Serial.print("COMMON: abort requested, reason=");
    Serial.println(reason);
    const bool manualRecoveryRequired =
        commonAbortReasonSuggestsManualRecovery(reason) ||
        (manipulator.online && manipulator.protocolOk && deviceStatusAlarm(manipulator)) ||
        (conveyor.online && conveyor.protocolOk &&
         (deviceStatusAlarm(conveyor) || conveyorPost7ManualRecoveryRequired(conveyor)));

    String post7Diag;
    const CommonAbortPost7Decision post7Decision = commonAbortDecidePost7Action(
        conveyor, post7Policy, post7Diag);

    if (post7Decision == CommonAbortPost7Decision::ForceStop && g_commonCycle.outfeedStartIssued) {
        if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "POST7 STOP", "common_abort_post7_stop")) {
            Serial.println("COMMON warning: CONV POST7 STOP send error");
        } else {
            Serial.println("COMMON: abort forced POST7 STOP.");
        }
    }

    if (post7Decision == CommonAbortPost7Decision::SkipAlreadySettled) {
        String settledEvent = "COMMON: abort sees POST7 already settled, skip STOP";
        if (!post7Diag.isEmpty()) {
            settledEvent += " (" + post7Diag + ")";
        }
        Serial.println(settledEvent);
        commonBeginAbortDrainSupervision(reason, manualRecoveryRequired, conveyor, post7Diag);
        return;
    }

    if (post7Decision == CommonAbortPost7Decision::SkipDrainToSafe) {
        commonBeginAbortDrainSupervision(reason, manualRecoveryRequired, conveyor, post7Diag);
        return;
    }

    commonFinalizeAbortState(
        reason,
        CommonAbortDrainOutcome::ForcedStop,
        manualRecoveryRequired,
        true,
        "common_abort_fsinv",
        post7Diag);
}

bool sendCommonManipulatorWorkCycleCommand(const String &reason, bool cycleLoadsSealer)
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
    if (!i2cSendManagedDeviceCommand(MANIPULATOR_ID, "R", "common_man_start")) {
        g_lastCommandResult = "COMMON failed: MAN R send error";
        Serial.println(g_lastCommandResult);
        return false;
    }

    if (cycleLoadsSealer && !i2cSendManagedDeviceCommand(CONVEYOR_ID, "FSC", "common_conv_fsc")) {
        Serial.println("COMMON warning: CONV FSC sync send error");
    }

    if (g_commonCycle.sealerDonePendingUnload) {
        g_commonCycle.sealerDonePendingUnload = false;
    }

    g_commonCycle.stage = CommonCycleStage::WaitManipStep7;
    g_commonCycle.currentCycleLoadsSealer = cycleLoadsSealer;
    g_commonCycle.step3Seen = false;
    g_commonCycle.step3LaunchDone = g_commonCycle.pauseRequested || !cycleLoadsSealer;
    g_commonCycle.parallelLaunchDone = false;
    commonResetPauseStrictSnapshotTracking();
    commonClearRefillPending();
    g_commonCycle.manipStarts++;
    g_commonCycle.manipStartReason = reason;
    g_commonCycle.lastEvent = "COMMON: manipulator start (" + reason + ")";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    if (g_commonCycle.pauseRequested &&
        !cycleLoadsSealer &&
        reason == "pause mandatory unload") {
        const uint32_t nowMs = millis();
        commonPauseRuntimeMarkMandatoryUnloadStarted(nowMs);
        Serial.print("COMMON: pause mandatory unload supervision armed, elapsed_ms=0, substate=");
        Serial.println(commonPauseRuntimeSubstateName(
            g_rs485Devices[CONVEYOR_ID],
            g_rs485Devices[MANIPULATOR_ID]));
    }
    (void)mqttPublishStatus(false);
    return true;
}

bool startCommonManipulatorWorkCycle(const String &reason, bool cycleLoadsSealer)
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
    if (deviceStatusAlarm(manipulator)) {
        g_lastCommandResult = "COMMON failed: manipulator alarm";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_commonCycle.currentCycleLoadsSealer = cycleLoadsSealer;
    g_commonCycle.step3Seen = false;
    g_commonCycle.step3LaunchDone = g_commonCycle.pauseRequested || !cycleLoadsSealer;
    g_commonCycle.parallelLaunchDone = false;
    commonClearRefillPending();

    if (manipulatorInWorkStartPose(manipulator)) {
        return sendCommonManipulatorWorkCycleCommand(reason, cycleLoadsSealer);
    }
    if (manipulatorNeedsOnlyGripOpenForWorkStart(manipulator)) {
        if (!i2cSendManagedDeviceCommand(MANIPULATOR_ID, "Q", "common_man_prepare_q")) {
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
        "COMMON failed: manipulator start not ready";
    Serial.println(g_lastCommandResult);
    return false;
}

bool startCommonInfeedFillAfterStep3()
{
    const CommonRefillLaunchResult refillResult =
        commonTryStartConveyorRefill("parallel after step3", true);
    if (refillResult == CommonRefillLaunchResult::Failed) {
        return false;
    }
    if (refillResult != CommonRefillLaunchResult::SkippedBusy) {
        g_commonCycle.step3LaunchDone = true;
    }
    return true;
}

bool startCommonConveyorProgram1(const String &reason)
{
    return commonTryStartConveyorRefill(reason, true) != CommonRefillLaunchResult::Failed;
}

bool startCommonPost7Job(const Rs485DeviceState &conveyor, uint32_t nowMs)
{
    if (!conveyor.online || !conveyor.protocolOk) {
        g_lastCommandResult = "COMMON failed: conveyor offline at POST7 start";
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (deviceStatusAlarm(conveyor)) {
        if (conveyorPost7ManualRecoveryRequired(conveyor)) {
            g_lastCommandResult = "COMMON failed: conveyor manual recovery required at POST7 start";
        } else {
            g_lastCommandResult = "COMMON failed: conveyor alarm at POST7 start";
        }
        Serial.println(g_lastCommandResult);
        return false;
    }

    const String post7Mode = g_commonCycle.currentCycleLoadsSealer ? "LOAD" : "UNLOAD_ONLY";
    const String command = "POST7 START " + post7Mode;
    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, command, "common_step7_post7_start")) {
        g_lastCommandResult = "COMMON failed: CONV POST7 START send error";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_commonCycle.outfeedStartIssued = true;
    g_commonCycle.outfeedNotReadyObserved = false;
    g_commonCycle.outfeedStep2SeqBase = conveyorStep2CompletionSeq(conveyor);
    g_commonCycle.post7SealerCompletionSeqBase = conveyorSealerCompletionSeq(conveyor);
    g_commonCycle.outfeedStartedMs = nowMs;
    g_commonCycle.sealerDonePendingUnload = false;

    g_commonCycle.step3LaunchDone = true;
    g_commonCycle.parallelLaunchDone = true;
    g_commonCycle.parallelStarts++;
    g_commonCycle.stage = CommonCycleStage::WaitNextBatch;
    if (g_commonCycle.currentCycleLoadsSealer) {
        g_commonCycle.lastEvent = "COMMON: step7 reached, started POST7 LOAD";
    } else {
        g_commonCycle.lastEvent = "COMMON: step7 reached, started POST7 UNLOAD_ONLY";
    }
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
    return true;
}

bool startCommonCycle()
{
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];

    if (g_commonCycle.abortDrainActive) {
        g_lastCommandResult = "COMMON aborted: POST7 drain supervision in progress";
        Serial.println(g_lastCommandResult);
        return false;
    }

    if (commonManualRecoveryRequired()) {
        commonPauseRuntimeApplyState(CommonPauseRuntimeState::ManualRecoveryRequired);
        g_lastCommandResult = "COMMON blocked: manual recovery required";
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        (void)mqttPublishStatus(false);
        return false;
    }

    if (g_commonCycle.active) {
        g_commonCycle.runtimeState = CommonRuntimeState::Active;
        if (g_commonCycle.pauseRequested) {
            g_lastCommandResult =
                "COMMON blocked: pause active, run COMMON PAUSE RELEASE, then COMMON START";
            g_commonCycle.lastEvent = g_lastCommandResult;
            Serial.println(g_lastCommandResult);
            (void)mqttPublishStatus(false);
            return false;
        }
        g_lastCommandResult = "COMMON already active";
        Serial.println(g_lastCommandResult);
        return false;
    }

    g_commonCycle = CommonCycleState{};
    g_commonCycleIdCounter++;
    if (g_commonCycleIdCounter == 0U) {
        g_commonCycleIdCounter = 1U;
    }
    g_commonCycle.cycleId = g_commonCycleIdCounter;
    g_commonCycle.active = true;
    g_commonCycle.runtimeState = CommonRuntimeState::Active;
    g_commonCycle.abortDrainOutcome = CommonAbortDrainOutcome::None;
    g_commonCycle.abortReason = "";
    g_commonCycle.abortRequiresManualRecovery = false;
    g_commonCycle.stage = CommonCycleStage::WaitInitialBatch;
    g_commonCycle.pauseState = CommonPauseState::None;
    g_commonCycle.sealerDonePendingUnload = conveyor.online &&
        conveyor.protocolOk &&
        conveyorSealerDoneActive(conveyor);
    g_commonCycle.post7SealerCompletionSeqBase = conveyorSealerCompletionSeq(conveyor);
    g_commonCycle.lastEvent = "COMMON: waiting initial batch";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    commonPauseRuntimeDeactivate();

    if (conveyorBatchReady(conveyor)) {
        g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
        if (!startCommonManipulatorWorkCycle("initial batch ready", true)) {
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

bool clearCommonRecoveryState()
{
    if (g_commonCycle.abortDrainActive) {
        g_lastCommandResult = "COMMON RECOVERY CLEAR rejected: drain supervision active";
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        return false;
    }

    if (!commonManualRecoveryRequired()) {
        g_lastCommandResult = "COMMON RECOVERY CLEAR rejected: no manual recovery state";
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        return false;
    }

    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];
    const bool conveyorOnline = conveyor.online && conveyor.protocolOk;
    const bool manipulatorOnline = manipulator.online && manipulator.protocolOk;
    const bool conveyorAlarm = conveyorOnline && deviceStatusAlarm(conveyor);
    const bool conveyorManualRecovery = conveyorOnline && conveyorPost7ManualRecoveryRequired(conveyor);
    const bool manipulatorAlarm = manipulatorOnline && deviceStatusAlarm(manipulator);

    if (conveyorAlarm || conveyorManualRecovery || manipulatorAlarm) {
        String reason = "COMMON RECOVERY CLEAR rejected: local fault/recovery still active";
        reason += ", conveyor_alarm=" + String(conveyorAlarm ? "yes" : "no");
        reason += ", conveyor_manual_recovery=" + String(conveyorManualRecovery ? "yes" : "no");
        reason += ", manipulator_alarm=" + String(manipulatorAlarm ? "yes" : "no");
        g_lastCommandResult = reason;
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        return false;
    }

    commonResetAfterAbortFinalization();
    g_commonCycle.runtimeState = CommonRuntimeState::Idle;
    g_commonCycle.abortDrainOutcome = CommonAbortDrainOutcome::None;
    g_commonCycle.abortRequiresManualRecovery = false;
    g_commonCycle.abortReason = "";
    g_commonCycle.lastEvent = "COMMON RECOVERY CLEAR accepted";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    (void)mqttPublishStatus(false);
    return true;
}

void requestCommonCyclePause()
{
    if (!g_commonCycle.active) {
        g_lastCommandResult = "COMMON is not active";
        Serial.println(g_lastCommandResult);
        return;
    }

    const bool pauseWasRequested = g_commonCycle.pauseRequested;
    const uint32_t nowMs = millis();
    g_commonCycle.pauseRequested = true;
    g_commonCycle.pauseForceDrainApplied = false;
    if (!pauseWasRequested || g_pauseRuntime.currentEpoch == 0) {
        commonPauseRuntimeBeginEpoch(nowMs);
        g_commonCycle.pauseStrictGateLatched = false;
        g_commonCycle.pauseStrictRollbackBlockedLogged = false;
    }
    commonPauseRuntimeApplyState(CommonPauseRuntimeState::PauseArming);
    commonPauseSendArmCommands();
    commonPausePollStatusCommands(nowMs);
    if (g_commonCycle.pauseState == CommonPauseState::None) {
        g_commonCycle.pauseState = CommonPauseState::Requested;
    }
    g_commonCycle.lastEvent = "COMMON: pause requested";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    Serial.print("COMMON: pause request sent, epoch=");
    Serial.println(g_pauseRuntime.currentEpoch);
    (void)mqttPublishStatus(false);
}

bool releaseCommonCyclePause()
{
    if (g_commonCycle.abortDrainActive) {
        g_lastCommandResult = "COMMON PAUSE RELEASE rejected: drain supervision active";
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (!g_commonCycle.active || !g_commonCycle.pauseRequested) {
        g_lastCommandResult = "COMMON PAUSE RELEASE rejected: pause is not active";
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        return false;
    }
    if (!commonPauseRuntimeCanRelease()) {
        g_lastCommandResult = "COMMON PAUSE RELEASE rejected: pause outcome is not hold/empty (" +
            String(commonPauseRuntimeStateName(g_pauseRuntime.state)) + ")";
        g_commonCycle.lastEvent = g_lastCommandResult;
        Serial.println(g_lastCommandResult);
        (void)mqttPublishStatus(false);
        return false;
    }

    commonPauseSendReleaseCommands();

    g_commonCycle = CommonCycleState{};
    g_commonCycle.runtimeState = CommonRuntimeState::Idle;
    g_commonCycle.abortDrainOutcome = CommonAbortDrainOutcome::None;
    g_commonCycle.lastEvent = "COMMON: pause released; waiting new COMMON START";
    g_lastCommandResult = g_commonCycle.lastEvent;
    Serial.println(g_lastCommandResult);
    commonPauseRuntimeDeactivate();
    (void)mqttPublishStatus(false);
    return true;
}

void processCommonCycle()
{
    const uint32_t nowMs = millis();
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    const Rs485DeviceState &manipulator = g_rs485Devices[MANIPULATOR_ID];

    if (g_commonCycle.abortDrainActive) {
        processCommonAbortDrainSupervision(nowMs, conveyor);
        return;
    }

    if (!g_commonCycle.active) {
        commonPauseRuntimeTick(nowMs, conveyor);
        return;
    }

    commonPauseRuntimeTick(nowMs, conveyor);
    if (!g_commonCycle.active) {
        return;
    }

    const bool sealerDoneStable = commonSealerDoneStable(nowMs);
    const bool conveyorOnline = conveyor.online && conveyor.protocolOk;
    const bool manipulatorOnline = manipulator.online && manipulator.protocolOk;
    const bool feedSideKnown = commonFeedSideSnapshotKnown(conveyor);
    const bool feedSideEmpty = commonFeedSideEmpty(conveyor);
    const bool outfeedReady = conveyorOnline && conveyorOutfeedReadyForBatch(conveyor);

    if (g_commonCycle.stage != g_commonDiagLastStage) {
        g_commonDiagLastStage = g_commonCycle.stage;
        g_commonDiagStageSinceMs = nowMs;
        g_commonDiagLastWaitLogMs = nowMs;
#if MASTER_COMMON_TRACE_DEBUG
        Serial.print("COMMON TRACE: stage=");
        Serial.print(commonCycleStageName(g_commonCycle.stage));
        Serial.print(", pause=");
        Serial.print(commonPauseStateName(g_commonCycle.pauseState));
        Serial.print(", event=");
        Serial.println(g_commonCycle.lastEvent);
#endif
    }

    if ((uint32_t)(nowMs - g_commonDiagStageSinceMs) >= COMMON_DIAG_WAIT_LOG_MS &&
        (uint32_t)(nowMs - g_commonDiagLastWaitLogMs) >= COMMON_DIAG_WAIT_LOG_MS) {
        g_commonDiagLastWaitLogMs = nowMs;
#if MASTER_COMMON_TRACE_DEBUG
        Serial.print("COMMON TRACE: waiting stage=");
        Serial.print(commonCycleStageName(g_commonCycle.stage));
        Serial.print(", stage_ms=");
        Serial.print(static_cast<uint32_t>(nowMs - g_commonDiagStageSinceMs));
        Serial.print(", pause=");
        Serial.print(commonPauseStateName(g_commonCycle.pauseState));
        Serial.print(", conveyor_online=");
        Serial.print(conveyorOnline ? "yes" : "no");
        Serial.print(", manip_online=");
        Serial.print(manipulatorOnline ? "yes" : "no");
        Serial.print(", manip_ready=");
        Serial.print(manipulatorInWorkStartPose(manipulator) ? "yes" : "no");
        Serial.print(", manip_q_only=");
        Serial.print(manipulatorNeedsOnlyGripOpenForWorkStart(manipulator) ? "yes" : "no");
        Serial.print(", manip_grip_unknown=");
        Serial.print(manipulatorWorkStartGripUnknown(manipulator) ? "yes" : "no");
        Serial.print(", otvod_ready=");
        Serial.print(outfeedReady ? "yes" : "no");
        Serial.print(", otvod_seen_not_ready=");
        Serial.print(g_commonCycle.outfeedNotReadyObserved ? "yes" : "no");
        Serial.print(", otvod_seq=");
        Serial.print(conveyorOnline ? conveyorStep2CompletionSeq(conveyor) : 0);
        Serial.print(", otvod_seq_base=");
        Serial.print(g_commonCycle.outfeedStep2SeqBase);
        Serial.print(", post7_mode=");
        Serial.print(g_commonCycle.currentCycleLoadsSealer ? "LOAD" : "UNLOAD_ONLY");
        Serial.print(", sealer_seq=");
        Serial.print(conveyorOnline ? conveyorSealerCompletionSeq(conveyor) : 0);
        Serial.print(", sealer_seq_base=");
        Serial.print(g_commonCycle.post7SealerCompletionSeqBase);
        Serial.print(", sealer_seq_advanced=");
        Serial.print(conveyorOnline &&
            conveyorSealerCompletionSeq(conveyor) != g_commonCycle.post7SealerCompletionSeqBase
                ? "yes"
                : "no");
        Serial.print(", sealer_pending_unload=");
        Serial.print(g_commonCycle.sealerDonePendingUnload ? "yes" : "no");
        Serial.print(", seal_stable=");
        Serial.print(sealerDoneStable ? "yes" : "no");
        Serial.print(", feed_known=");
        Serial.print(feedSideKnown ? "yes" : "no");
        Serial.print(", feed_empty=");
        Serial.println(feedSideEmpty ? "yes" : "no");
#endif
    }

    switch (g_commonCycle.stage) {
        case CommonCycleStage::WaitInitialBatch:
            if (!conveyorOnline || !manipulatorOnline) {
                return;
            }
            if (!conveyorBatchReady(conveyor)) {
                if (g_commonCycle.pauseRequested) {
                    if (!commonPauseRuntimeNodeReadyForFinalize()) {
                        return;
                    }
                    commonPauseRuntimeMarkFinalizeStarted();
                    const bool pauseFeedKnownEmpty =
                        commonPauseFinalizeFeedSideKnownEmpty(feedSideKnown, feedSideEmpty);
                    const bool pauseFeedKnown = feedSideKnown || pauseFeedKnownEmpty;
                    const bool pauseFeedCollecting =
                        !pauseFeedKnownEmpty && feedSideKnown && !feedSideEmpty;
                    if (commonPauseStableContractReached()) {
                        return;
                    }
                    const bool pauseStrictForwardOnly = commonPauseStrictGateForwardOnly();
                    if (pauseStrictForwardOnly) {
                        if (conveyorProgram1Active(conveyor) || pauseFeedCollecting) {
                            commonLogPauseStrictRollbackBlocked();
                        }
                        const bool strictSnapshotBypassed =
                            commonTryBypassPauseStrictSnapshotUnknown(conveyor, false, nowMs);
                        if (!strictSnapshotBypassed) {
                            return;
                        }
                        if (g_commonCycle.sealerDonePendingUnload && sealerDoneStable) {
                            if (!outfeedReady) {
                                if (g_commonCycle.lastEvent !=
                                    "COMMON: pause waits OUT2 for mandatory unload") {
                                    g_commonCycle.lastEvent =
                                        "COMMON: pause waits OUT2 for mandatory unload";
                                    g_lastCommandResult = g_commonCycle.lastEvent;
                                    Serial.println(g_lastCommandResult);
                                    (void)mqttPublishStatus(false);
                                }
                                return;
                            }
                            if (!startCommonManipulatorWorkCycle("pause mandatory unload", false)) {
                                abortCommonCycle("COMMON failed: mandatory unload start");
                            }
                            return;
                        }
                        (void)commonTryEnterPauseStableWait(conveyor);
                        return;
                    }
                    if (conveyorProgram1Active(conveyor)) {
                        commonResetPauseStrictSnapshotTracking();
                        g_commonCycle.pauseState = CommonPauseState::FillLastBlock;
                        return;
                    }
                    bool strictSnapshotBypassed = false;
                    if (!pauseFeedKnown) {
                        strictSnapshotBypassed =
                            commonTryBypassPauseStrictSnapshotUnknown(conveyor, false, nowMs);
                        if (!strictSnapshotBypassed) {
                            return;
                        }
                    } else {
                        commonResetPauseStrictSnapshotTracking();
                    }
                    if (pauseFeedCollecting && !strictSnapshotBypassed) {
                        commonResetPauseStrictSnapshotTracking();
                        g_commonCycle.pauseState = CommonPauseState::FillLastBlock;
                        return;
                    }
                    if (g_commonCycle.sealerDonePendingUnload && sealerDoneStable) {
                        if (!outfeedReady) {
                            if (g_commonCycle.lastEvent !=
                                "COMMON: pause waits OUT2 for mandatory unload") {
                                g_commonCycle.lastEvent =
                                    "COMMON: pause waits OUT2 for mandatory unload";
                                g_lastCommandResult = g_commonCycle.lastEvent;
                                Serial.println(g_lastCommandResult);
                                (void)mqttPublishStatus(false);
                            }
                            return;
                        }
                        if (!startCommonManipulatorWorkCycle("pause mandatory unload", false)) {
                            abortCommonCycle("COMMON failed: mandatory unload start");
                        }
                        return;
                    }
                    (void)commonTryEnterPauseStableWait(conveyor);
                    return;
                }
                if (!g_commonCycle.pauseRequested &&
                    !deviceStatusBusy(conveyor) &&
                    !conveyorProgram1Active(conveyor) &&
                    !startCommonConveyorProgram1("retry initial fill")) {
                    abortCommonCycle("COMMON failed: retry P1 start");
                }
                return;
            }
            if (g_commonCycle.pauseRequested) {
                if (!commonPauseRuntimeNodeReadyForFinalize()) {
                    return;
                }
                commonPauseRuntimeMarkFinalizeStarted();
                commonResetPauseStrictSnapshotTracking();
                g_commonCycle.pauseState = CommonPauseState::FillLastBlock;
            }
            g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
            if (!startCommonManipulatorWorkCycle(
                    g_commonCycle.pauseRequested ? String("pause final block") : String("initial batch ready"),
                    true)) {
                abortCommonCycle("COMMON failed: manipulator start");
            }
            return;

        case CommonCycleStage::WaitManipReady: {
            const bool manipulatorOnline = manipulator.online && manipulator.protocolOk;
            const bool workStartReady = manipulatorInWorkStartPose(manipulator);
            const bool needsQOnly = manipulatorNeedsOnlyGripOpenForWorkStart(manipulator);
            const bool gripUnknown = manipulatorWorkStartGripUnknown(manipulator);
            const uint32_t waitMs = static_cast<uint32_t>(nowMs - g_commonDiagStageSinceMs);

            if (manipulatorOnline && deviceStatusAlarm(manipulator)) {
                abortCommonCycle("COMMON failed: manipulator alarm");
                return;
            }
            if (waitMs >= COMMON_WAIT_MANIP_READY_TIMEOUT_MS) {
                String reason = "COMMON failed: WaitManipReady timeout";
                reason += " (" + String(waitMs) + " ms)";
                reason += ", online=" + String(manipulatorOnline ? "yes" : "no");
                reason += ", busy=" + String(deviceStatusBusy(manipulator) ? "yes" : "no");
                reason += ", ready=" + String(workStartReady ? "yes" : "no");
                reason += ", q_only=" + String(needsQOnly ? "yes" : "no");
                reason += ", grip_unknown=" + String(gripUnknown ? "yes" : "no");
                reason += ", sensor_bits=" + String(manipulatorSensorBits(manipulator));
                abortCommonCycle(reason);
                return;
            }
            if (!manipulatorOnline) {
                return;
            }
            if (deviceStatusBusy(manipulator)) {
                return;
            }
            if (!workStartReady) {
                if (gripUnknown &&
                    g_commonCycle.lastEvent !=
                        "COMMON: wait manip ready, grip unknown (right+zUp)") {
                    g_commonCycle.lastEvent =
                        "COMMON: wait manip ready, grip unknown (right+zUp)";
                    g_lastCommandResult = g_commonCycle.lastEvent;
                    Serial.println(g_lastCommandResult);
                    (void)mqttPublishStatus(false);
                }
                return;
            }
            if (!sendCommonManipulatorWorkCycleCommand(
                    g_commonCycle.manipStartReason.isEmpty() ? String("prepared") : g_commonCycle.manipStartReason,
                    g_commonCycle.currentCycleLoadsSealer)) {
                abortCommonCycle("COMMON failed: manipulator start after prepare");
            }
            return;
        }

        case CommonCycleStage::WaitManipStep7:
            if (!manipulator.online || !manipulator.protocolOk) {
                return;
            }
            if (deviceStatusAlarm(manipulator)) {
                abortCommonCycle("COMMON failed: manipulator alarm");
                return;
            }
            if (!g_commonCycle.step3Seen &&
                g_commonCycle.currentCycleLoadsSealer &&
                !g_commonCycle.pauseRequested &&
                deviceStatusBusy(manipulator) &&
                manipulatorStep3Ready(manipulator)) {
                g_commonCycle.step3Seen = true;
                commonSetEventIfChanged("COMMON: step3 seen");
            }
            if (!g_commonCycle.step3LaunchDone &&
                g_commonCycle.currentCycleLoadsSealer &&
                !g_commonCycle.pauseRequested &&
                deviceStatusBusy(manipulator) &&
                manipulatorStep3Ready(manipulator)) {
                if (!startCommonInfeedFillAfterStep3()) {
                    abortCommonCycle("COMMON failed: parallel start after step3");
                }
                return;
            }
            if (!g_commonCycle.parallelLaunchDone &&
                deviceStatusBusy(manipulator) &&
                manipulatorStep7Ready(manipulator)) {
                if (!g_commonCycle.step3LaunchDone &&
                    g_commonCycle.currentCycleLoadsSealer &&
                    !g_commonCycle.pauseRequested) {
                    if (!startCommonInfeedFillAfterStep3()) {
                        abortCommonCycle("COMMON failed: fallback start before step7");
                        return;
                    }
                }
                if (!startCommonPost7Job(conveyor, nowMs)) {
                    abortCommonCycle("COMMON failed: POST7 start after step7");
                }
            }
            return;

        case CommonCycleStage::WaitNextBatch: {
            if (manipulatorOnline && deviceStatusAlarm(manipulator)) {
                abortCommonCycle("COMMON failed: manipulator alarm",
                                 CommonAbortPost7Policy::AllowDrainToSafe);
                return;
            }
            if (conveyorOnline &&
                (deviceStatusAlarm(conveyor) || conveyorPost7ManualRecoveryRequired(conveyor))) {
                if (conveyorPost7ManualRecoveryRequired(conveyor)) {
                    abortCommonCycle("COMMON failed: conveyor manual recovery required");
                } else {
                    abortCommonCycle("COMMON failed: conveyor alarm");
                }
                return;
            }

            const bool outfeedStep2Active = conveyorOnline && conveyorStep2Active(conveyor);
            const bool outfeedVfdActive = conveyorOnline && conveyorVfdTimedRunActive(conveyor);
            const uint8_t outfeedStep2Seq = conveyorOnline ? conveyorStep2CompletionSeq(conveyor) : 0U;
            const bool outfeedStep2SeqAdvanced = conveyorOnline &&
                outfeedStep2Seq != g_commonCycle.outfeedStep2SeqBase;
            const uint8_t sealerSeq = conveyorOnline ? conveyorSealerCompletionSeq(conveyor) : 0U;
            const bool sealerSeqAdvancedForPost7 = conveyorOnline &&
                sealerSeq != g_commonCycle.post7SealerCompletionSeqBase;
            const bool outfeedActivityObserved = !outfeedReady ||
                outfeedStep2Active ||
                outfeedVfdActive ||
                outfeedStep2SeqAdvanced;

            if (g_commonCycle.outfeedStartIssued &&
                conveyorOnline &&
                !g_commonCycle.outfeedNotReadyObserved &&
                outfeedActivityObserved) {
                g_commonCycle.outfeedNotReadyObserved = true;
            }

            const bool manipulatorIdle = manipulatorOnline && !deviceStatusBusy(manipulator);
            const bool otvodReady = outfeedReady && g_commonCycle.outfeedNotReadyObserved;
            const bool sealerDoneForJob =
                !g_commonCycle.currentCycleLoadsSealer || sealerSeqAdvancedForPost7;
            const bool cycleSettled = g_commonCycle.parallelLaunchDone &&
                manipulatorIdle &&
                otvodReady &&
                sealerDoneForJob;

            if (!cycleSettled) {
                if (g_commonCycle.parallelLaunchDone && g_commonCycle.outfeedStartIssued) {
                    const uint32_t outfeedWaitMs =
                        static_cast<uint32_t>(nowMs - g_commonCycle.outfeedStartedMs);
                    if (outfeedWaitMs >= COMMON_WAIT_OUTFEED_READY_TIMEOUT_MS) {
                        String reason = "COMMON failed: POST7 wait timeout";
                        reason += " (" + String(outfeedWaitMs) + " ms)";
                        reason += ", online=" + String(conveyorOnline ? "yes" : "no");
                        reason += ", post7_mode=" +
                            String(g_commonCycle.currentCycleLoadsSealer ? "LOAD" : "UNLOAD_ONLY");
                        reason += ", outfeed_ready=" + String(outfeedReady ? "yes" : "no");
                        reason += ", outfeed_seen_not_ready=" +
                            String(g_commonCycle.outfeedNotReadyObserved ? "yes" : "no");
                        reason += ", outfeed_step2=" + String(outfeedStep2Active ? "yes" : "no");
                        reason += ", outfeed_vfd=" + String(outfeedVfdActive ? "yes" : "no");
                        reason += ", outfeed_seq=" + String(outfeedStep2Seq);
                        reason += ", outfeed_seq_base=" + String(g_commonCycle.outfeedStep2SeqBase);
                        reason += ", sealer_expected=" +
                            String(g_commonCycle.currentCycleLoadsSealer ? "yes" : "no");
                        reason += ", sealer_seq=" + String(sealerSeq);
                        reason += ", sealer_seq_base=" + String(g_commonCycle.post7SealerCompletionSeqBase);
                        reason += ", sealer_seq_advanced=" + String(sealerSeqAdvancedForPost7 ? "yes" : "no");
                        if (!otvodReady) {
                            reason += ", blocker=outfeed";
                        } else if (!sealerDoneForJob) {
                            reason += ", blocker=sealer_seq";
                        } else {
                            reason += ", blocker=unknown";
                        }
                        abortCommonCycle(reason);
                        return;
                    }
                }

                if (g_commonCycle.pauseRequested &&
                    feedSideKnown &&
                    feedSideEmpty &&
                    g_commonCycle.sealerDonePendingUnload &&
                    sealerDoneStable &&
                    !otvodReady &&
                    g_commonCycle.lastEvent != "COMMON: pause waits OUT2 for mandatory unload") {
                    g_commonCycle.lastEvent = "COMMON: pause waits OUT2 for mandatory unload";
                    g_lastCommandResult = g_commonCycle.lastEvent;
                    Serial.println(g_lastCommandResult);
                    (void)mqttPublishStatus(false);
                }
                if (g_commonCycle.pauseRequested &&
                    !feedSideKnown &&
                    g_commonCycle.pauseState == CommonPauseState::WaitStrictFeedSnapshot) {
                    const bool nextBatchReadyForPause =
                        conveyorNextBatchReadyForCommon(conveyor, g_commonCycle.lastConsumedBatchSeq);
                    (void)commonTryBypassPauseStrictSnapshotUnknown(
                        conveyor,
                        nextBatchReadyForPause,
                        nowMs);
                }
                return;
            }

            const bool feedHasBatch = conveyorNextBatchReadyForCommon(
                conveyor,
                g_commonCycle.lastConsumedBatchSeq);
            const bool feedCollecting = feedSideKnown && !feedSideEmpty;
            const bool pauseFeedKnownEmpty =
                commonPauseFinalizeFeedSideKnownEmpty(feedSideKnown, feedSideEmpty);
            const bool pauseFeedKnown = feedSideKnown || pauseFeedKnownEmpty;
            const bool pauseFeedCollecting = !pauseFeedKnownEmpty && feedCollecting;
            const bool nextBatchReady = feedHasBatch;

            if (g_commonCycle.pauseRequested) {
                if (!commonPauseRuntimeNodeReadyForFinalize()) {
                    return;
                }
                commonPauseRuntimeMarkFinalizeStarted();
                if (commonPauseStableContractReached()) {
                    return;
                }
                const bool pauseStrictForwardOnly = commonPauseStrictGateForwardOnly();
                if (pauseStrictForwardOnly) {
                    if (feedHasBatch ||
                        conveyorProgram1Active(conveyor) ||
                        pauseFeedCollecting) {
                        commonLogPauseStrictRollbackBlocked();
                    }
                    const bool strictSnapshotBypassed =
                        commonTryBypassPauseStrictSnapshotUnknown(conveyor, feedHasBatch, nowMs);
                    if (!strictSnapshotBypassed) {
                        return;
                    }
                    if (g_commonCycle.sealerDonePendingUnload && sealerDoneStable) {
                        if (!startCommonManipulatorWorkCycle("pause mandatory unload", false)) {
                            abortCommonCycle("COMMON failed: mandatory unload start");
                        }
                        return;
                    }

                    (void)commonTryEnterPauseStableWait(conveyor);
                    return;
                }
                if (feedHasBatch) {
                    g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
                    if (!startCommonManipulatorWorkCycle("pause final block", true)) {
                        abortCommonCycle("COMMON failed: pause final block start");
                    }
                    return;
                }

                if (conveyorProgram1Active(conveyor)) {
                    commonResetPauseStrictSnapshotTracking();
                    g_commonCycle.pauseState = CommonPauseState::FillLastBlock;
                    if (g_commonCycle.lastEvent != "COMMON: pause_fill_last_block waiting batch") {
                        g_commonCycle.lastEvent = "COMMON: pause_fill_last_block waiting batch";
                        g_lastCommandResult = g_commonCycle.lastEvent;
                        Serial.println(g_lastCommandResult);
                        (void)mqttPublishStatus(false);
                    }
                    return;
                }

                bool strictSnapshotBypassed = false;
                if (!pauseFeedKnown) {
                    strictSnapshotBypassed =
                        commonTryBypassPauseStrictSnapshotUnknown(conveyor, feedHasBatch, nowMs);
                    if (!strictSnapshotBypassed) {
                        return;
                    }
                } else {
                    commonResetPauseStrictSnapshotTracking();
                }

                if (pauseFeedCollecting && !strictSnapshotBypassed) {
                    commonResetPauseStrictSnapshotTracking();
                    g_commonCycle.pauseState = CommonPauseState::FillLastBlock;
                    return;
                }

                if (g_commonCycle.sealerDonePendingUnload && sealerDoneStable) {
                    if (!startCommonManipulatorWorkCycle("pause mandatory unload", false)) {
                        abortCommonCycle("COMMON failed: mandatory unload start");
                    }
                    return;
                }

                (void)commonTryEnterPauseStableWait(conveyor);
                return;
            }

            if (!nextBatchReady) {
                if (!startCommonConveyorProgram1("retry after step7")) {
                    abortCommonCycle("COMMON failed: retry P1 after step7");
                    return;
                }
                return;
            }

            g_commonCycle.lastConsumedBatchSeq = conveyorBatchSeq(conveyor);
            if (!startCommonManipulatorWorkCycle("next batch ready", true)) {
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

    if (!i2cSendManagedDeviceCommand(id, command, "cli_managed_cmd")) {
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

bool i2cSendManagedDeviceCommand(uint8_t id, const String &command, const char *origin)
{
    String text = command;
    text.trim();
    if (text.isEmpty() || text.length() > I2C_MANAGED_TEXT_MAX_LEN) {
        return false;
    }

    char detail[40] = {};
    snprintf(detail, sizeof(detail), "cmd:%s", text.c_str());
    i2cDiagStart(id, "cmd", origin, detail);

    Wire.beginTransmission(id);
    Wire.write(I2C_MANAGED_TEXT_CMD);
    Wire.write(reinterpret_cast<const uint8_t *>(text.c_str()), text.length());
    const uint8_t txErr = Wire.endTransmission(true);
    if (txErr == 0) {
        i2cDiagFinish(true, "ok");
        return true;
    }

    char outcome[24] = {};
    snprintf(outcome, sizeof(outcome), "tx_err_%u", txErr);
    i2cDiagFinish(false, outcome);
    i2cRecoverBus("managed command tx error");
    return false;
}

MbResult i2cReadManagedDeviceFrame(uint8_t id, ManagedDeviceI2cFrame &frame, const char *origin)
{
    memset(&frame, 0, sizeof(frame));

    i2cDiagStart(id, "read", origin, "managed frame");

    i2cTraceWireStep(id, origin, "pre_begin_tx");
    Wire.beginTransmission(id);
    Wire.write(I2C_MANAGED_POKE_CMD);
    const uint8_t pokeErr = Wire.endTransmission(true);
    i2cTraceWireStep(id, origin, "post_end_tx", pokeErr, pokeErr != 0);
    if (pokeErr != 0) {
        char outcome[24] = {};
        snprintf(outcome, sizeof(outcome), "poke_err_%u", pokeErr);
        i2cDiagFinish(false, outcome);
        i2cRecoverBus("managed poke tx error");
        return MbResult::Timeout;
    }

    const size_t want = sizeof(frame);
    i2cTraceWireStep(id, origin, "pre_request", static_cast<int32_t>(want));
    const size_t got = Wire.requestFrom(static_cast<int>(id), static_cast<int>(want), true);
    i2cTraceWireStep(id, origin, "post_request", static_cast<int32_t>(got), got != want);
    if (got != want) {
        while (Wire.available() > 0) {
            (void)Wire.read();
        }
        if (got == 0) {
            i2cDiagFinish(false, "timeout");
            i2cRecoverBus("managed frame timeout");
            return MbResult::Timeout;
        }
        i2cDiagFinish(false, "short_frame");
        i2cRecoverBus("managed frame short read");
        return MbResult::ProtocolError;
    }

    uint8_t *dst = reinterpret_cast<uint8_t *>(&frame);
    i2cTraceWireStep(id, origin, "read_begin", static_cast<int32_t>(want));
    for (size_t i = 0; i < want; i++) {
        if (Wire.available() <= 0) {
            i2cTraceWireStep(id, origin, "read_underflow", static_cast<int32_t>(i), true);
            i2cDiagFinish(false, "underflow");
            i2cRecoverBus("managed frame underflow");
            return MbResult::ProtocolError;
        }
        dst[i] = static_cast<uint8_t>(Wire.read());
    }
    i2cTraceWireStep(id, origin, "read_done", static_cast<int32_t>(want));

    i2cDiagFinish(true, "ok");
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
    st.managedHeartbeatMs = 0;
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

void rs485ProbeManagedDevice(uint8_t id, uint16_t expectedKind, const char *origin)
{
    Rs485DeviceState &st = g_rs485Devices[id];
    const uint32_t now = millis();
    st.lastProbeMs = now;

    ManagedDeviceI2cFrame frame = {};
    const MbResult r = i2cReadManagedDeviceFrame(id, frame, origin);
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
    st.managedHeartbeatMs = frame.heartbeatMs;
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

void rs485ScanProbeId(uint8_t id, const char *scanOrigin)
{
    if (id == 0 || id > RS485_SCAN_TABLE_MAX_ID) {
        return;
    }

    Rs485DeviceState &st = g_rs485Devices[id];
    scanDiagStart(id, scanOrigin);

    if (id == VFD_ID) {
        rs485ProbeVfd(id);
        scanDiagFinish(id, st.lastResult, st.lastException);
        return;
    }
    if (id == CONVEYOR_ID) {
        rs485ProbeManagedDevice(id, DEVICE_KIND_CONVEYOR, "scan");
        scanDiagFinish(id, st.lastResult, st.lastException);
        return;
    }
    if (id == MANIPULATOR_ID) {
        rs485ProbeManagedDevice(id, DEVICE_KIND_MANIPULATOR, "scan");
        scanDiagFinish(id, st.lastResult, st.lastException);
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

    const uint32_t now = millis();
    st.lastProbeMs = now;
    st.lastResult = r;
    st.lastException = exceptionCode;

    if (isMbAliveResult(r)) {
        rs485MarkDeviceAlive(st, r, exceptionCode);
        scanDiagFinish(id, st.lastResult, st.lastException);
        return;
    }

    st.errCount++;
    if (!st.everSeen || (uint32_t)(now - st.lastSeenMs) > RS485_SCAN_STALE_MS) {
        rs485MarkDeviceOffline(st);
    }

    scanDiagFinish(id, st.lastResult, st.lastException);
}

void rs485ScanLoop()
{
    const uint32_t now = millis();

    if (!g_rs485ScanEnabled) {
        if ((uint32_t)(now - g_rs485ScanLastStepMs) < RS485_SCAN_STEP_INTERVAL_MS) {
            return;
        }
        g_rs485ScanLastStepMs = now;

        rs485ScanProbeId(CONVEYOR_ID, "core_poll");
        rs485ScanProbeId(MANIPULATOR_ID, "core_poll");
        return;
    }

    if (g_rs485ScanMinId == 0 ||
        g_rs485ScanMaxId > RS485_SCAN_TABLE_MAX_ID ||
        g_rs485ScanMinId > g_rs485ScanMaxId) {
        return;
    }

    if ((uint32_t)(now - g_rs485ScanLastStepMs) < RS485_SCAN_STEP_INTERVAL_MS) {
        return;
    }
    g_rs485ScanLastStepMs = now;

    for (uint8_t id = g_rs485ScanMinId; id <= g_rs485ScanMaxId; id++) {
        rs485ScanProbeId(id, "loop");
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
    if (g_rs485ScanEnabled) {
        Serial.println("MBSCAN scope: periodic background sweep over configured range.");
    } else {
        Serial.print("MBSCAN scope: background sweep OFF; core poll keeps 12/13 refreshed every ");
        Serial.print(RS485_SCAN_STEP_INTERVAL_MS);
        Serial.println(" ms.");
    }
    Serial.println("MBSCAN note: direct I2C commands to 12/13 are always enabled.");
    printScanDiagStatusLine();

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
    Serial.print(", vfd_timed_run=");
    Serial.print(conveyorVfdTimedRunActive(conveyor) ? "yes" : "no");
    Serial.print(", feed_strict=");
    Serial.print(conveyorFeedSideEmptyStrict(conveyor) ? "yes" : "no");
    Serial.print(", feed_valid=");
    Serial.print(conveyorFeedSideEmptyValid(conveyor) ? "yes" : "no");
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
    Serial.print(", busy=");
    Serial.print(deviceStatusBusy(manipulator) ? "yes" : "no");
    Serial.print(", alarm=");
    Serial.print(deviceStatusAlarm(manipulator) ? "yes" : "no");
    Serial.print(", mode=");
    Serial.print(manipulator.extra0);
    Serial.print(", job=");
    Serial.print(manipulator.extra1);
    Serial.print(", work_start_ready=");
    Serial.print(manipulatorInWorkStartPose(manipulator) ? "yes" : "no");
    Serial.print(", needs_q_only=");
    Serial.print(manipulatorNeedsOnlyGripOpenForWorkStart(manipulator) ? "yes" : "no");
    Serial.print(", work_start_ambiguous=");
    Serial.print(manipulatorWorkStartGripUnknown(manipulator) ? "yes" : "no");
    Serial.print(", sensor_bits=");
    Serial.println(manipulatorSensorBits(manipulator));
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
    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    payload += ",\"seal\":{";
    payload += "\"owner\":\"conveyor\"";
    payload += ",\"online\":" + String(conveyor.online && conveyor.protocolOk ? "true" : "false");
    payload += ",\"start_out\":" + String(conveyorSealerStartPulseActive(conveyor) ? "true" : "false");
    payload += ",\"done\":" + String(conveyorSealerDoneActive(conveyor) ? "true" : "false");
    payload += ",\"busy\":" + String(conveyorSealerBusy(conveyor) ? "true" : "false");
    payload += ",\"completion_seq\":" + String(conveyorSealerCompletionSeq(conveyor));
    payload += ",\"pulse_ms\":" + String(g_sealStartPulseDurationMs);
    payload += "}";
    const bool outfeedOnline = conveyor.online && conveyor.protocolOk;
    const bool outfeedReady = outfeedOnline && conveyorOutfeedReadyForBatch(conveyor);
    const bool outfeedStep2Active = outfeedOnline && conveyorStep2Active(conveyor);
    const bool outfeedVfdTimedRunActive = outfeedOnline && conveyorVfdTimedRunActive(conveyor);
    const uint8_t outfeedStep2Seq = outfeedOnline ? conveyorStep2CompletionSeq(conveyor) : 0U;
    const bool outfeedActive = outfeedOnline &&
        (outfeedStep2Active || outfeedVfdTimedRunActive || !outfeedReady);
    payload += ",\"otvod_cycle\":{";
    payload += "\"source\":\"conveyor_status\"";
    payload += ",\"online\":" + String(outfeedOnline ? "true" : "false");
    payload += ",\"active\":" + String(outfeedActive ? "true" : "false");
    payload += ",\"ready\":" + String(outfeedReady ? "true" : "false");
    payload += ",\"step_started\":" + String(outfeedStep2Active ? 1 : 0);
    payload += ",\"step_done\":" + String(outfeedStep2Seq);
    payload += ",\"vfd_started\":" + String(outfeedVfdTimedRunActive ? 1 : 0);
    payload += ",\"vfd_done\":" + String(outfeedReady ? 1 : 0);
    payload += ",\"step2_active\":" + String(outfeedStep2Active ? "true" : "false");
    payload += ",\"vfd_timed_run_active\":" + String(outfeedVfdTimedRunActive ? "true" : "false");
    payload += ",\"step2_seq\":" + String(outfeedStep2Seq);
    payload += ",\"last\":\"" + String(outfeedOnline
        ? (outfeedReady ? "settled" : "running_or_waiting")
        : "offline") + "\"";
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
    payload += ",\"vfd_timed_run_active\":" + String(conveyorVfdTimedRunActive(conveyor) ? "true" : "false");
    payload += ",\"program_state\":" + String(conveyorProgramStateCode(conveyor));
    payload += ",\"program_pass\":" + String(conveyorProgramPass(conveyor));
    payload += ",\"batch_ready\":" + String(conveyorBatchReady(conveyor) ? "true" : "false");
    payload += ",\"batch_ready_next_for_common\":" +
        String(conveyorNextBatchReadyForCommon(conveyor, g_commonCycle.lastConsumedBatchSeq) ? "true" : "false");
    payload += ",\"batch_ready_stale_for_common\":" +
        String(conveyorReadyBatchIsStaleForCommon(conveyor, g_commonCycle.lastConsumedBatchSeq) ? "true" : "false");
    payload += ",\"outfeed_ready_for_batch\":" +
        String((conveyor.online && conveyor.protocolOk && conveyorOutfeedReadyForBatch(conveyor)) ? "true" : "false");
    payload += ",\"batch_seq\":" + String(conveyorBatchSeq(conveyor));
    payload += ",\"feed_side_empty_strict\":" + String(conveyorFeedSideEmptyStrict(conveyor) ? "true" : "false");
    payload += ",\"feed_side_empty_valid\":" + String(conveyorFeedSideEmptyValid(conveyor) ? "true" : "false");
    payload += ",\"vfd_tick_ms\":" + String(conveyorVfdTickDurationMs(conveyor));
    payload += "},";
    payload += "\"manipulator\":{";
    payload += "\"id\":" + String(MANIPULATOR_ID);
    payload += ",\"online\":" + String(manipulator.online ? "true" : "false");
    payload += ",\"kind_ok\":" + String(manipulator.protocolOk ? "true" : "false");
    payload += ",\"status_word\":" + String(manipulator.statusWord);
    payload += ",\"error_word\":" + String(manipulator.errorWord);
    payload += ",\"busy\":" + String(deviceStatusBusy(manipulator) ? "true" : "false");
    payload += ",\"alarm\":" + String(deviceStatusAlarm(manipulator) ? "true" : "false");
    payload += ",\"mode\":" + String(manipulator.extra0);
    payload += ",\"job\":" + String(manipulator.extra1);
    payload += ",\"work_start_ready\":" + String(manipulatorInWorkStartPose(manipulator) ? "true" : "false");
    payload += ",\"needs_grip_open_only\":" +
        String(manipulatorNeedsOnlyGripOpenForWorkStart(manipulator) ? "true" : "false");
    payload += ",\"work_start_ambiguous\":" +
        String(manipulatorWorkStartGripUnknown(manipulator) ? "true" : "false");
    payload += ",\"sensor_bits\":" + String(manipulatorSensorBits(manipulator));
    payload += ",\"conflict_bits\":" + String(manipulatorConflictBits(manipulator));
    payload += ",\"work_step\":" + String(manipulatorWorkStep(manipulator));
    payload += ",\"step3_ready\":" + String(manipulatorStep3Ready(manipulator) ? "true" : "false");
    payload += ",\"step7_ready\":" + String(manipulatorStep7Ready(manipulator) ? "true" : "false");
    payload += "}";
    payload += "}";
    payload += ",\"common_cycle\":{";
    payload += "\"active\":" + String(g_commonCycle.active ? "true" : "false");
    payload += ",\"coarse_state\":\"" + String(commonCoarseStateName()) + "\"";
    payload += ",\"runtime_state\":\"" + String(commonRuntimeStateName(g_commonCycle.runtimeState)) + "\"";
    payload += ",\"manual_recovery_required\":" +
        String(commonManualRecoveryRequired() ? "true" : "false");
    payload += ",\"drain_supervision_active\":" + String(g_commonCycle.abortDrainActive ? "true" : "false");
    payload += ",\"drain_tail_outcome\":\"" + String(commonAbortDrainOutcomeName(g_commonCycle.abortDrainOutcome)) + "\"";
    payload += ",\"drain_elapsed_ms\":" +
        String(g_commonCycle.abortDrainActive
            ? static_cast<uint32_t>(millis() - g_commonCycle.abortDrainStartedMs)
            : 0U);
    payload += ",\"drain_no_progress_ms\":" +
        String(g_commonCycle.abortDrainActive
            ? static_cast<uint32_t>(millis() - g_commonCycle.abortDrainLastProgressMs)
            : 0U);
    payload += ",\"abort_reason\":\"" + escapeJsonString(g_commonCycle.abortReason) + "\"";
    payload += ",\"pause\":" + String(g_commonCycle.pauseRequested ? "true" : "false");
    payload += ",\"pause_state\":\"" + String(commonPauseRuntimeStateName(g_pauseRuntime.state)) + "\"";
    payload += ",\"pause_legacy_state\":\"" + String(commonPauseStateName(g_commonCycle.pauseState)) + "\"";
    payload += ",\"cycle_id\":" + String(g_commonCycle.cycleId);
    payload += ",\"pause_runtime_state\":\"" + String(commonPauseRuntimeStateName(g_pauseRuntime.state)) + "\"";
    payload += ",\"pause_epoch\":" + String(g_pauseRuntime.currentEpoch);
    payload += ",\"pause_cycle_id\":" + String(g_pauseRuntime.pauseCycleId);
    const bool pauseMandatoryUnloadInFlight =
        commonPauseRuntimeMandatoryUnloadInFlight(conveyor, manipulator);
    const bool pauseMandatoryUnloadPolicyActive =
        commonPauseRuntimeMandatoryUnloadPolicyActive();
    const bool pauseForceDrainStableWaitPolicyActive =
        commonPauseRuntimeForceDrainStableWaitPolicyActive();
    const uint32_t pauseMandatoryUnloadElapsedMs =
        commonPauseRuntimeMandatoryUnloadElapsedMs(millis());
    const char *pauseWatchdogProfile = commonPauseRuntimeWatchdogProfileName(
        pauseMandatoryUnloadPolicyActive);
    const char *pauseWatchdogPolicyReason = commonPauseRuntimeWatchdogPolicyReason(
        pauseMandatoryUnloadPolicyActive,
        pauseMandatoryUnloadInFlight);
    payload += ",\"pause_can_release\":" + String(commonPauseRuntimeCanRelease() ? "true" : "false");
    payload += ",\"pause_arming_elapsed_ms\":" + String(commonPauseRuntimeArmingElapsedMs(millis()));
    payload += ",\"pause_no_progress_ms\":" + String(commonPauseRuntimeNoProgressMs(millis()));
    payload += ",\"pause_arming_timeout_limit_ms\":" + String(
        pauseMandatoryUnloadPolicyActive
            ? COMMON_PAUSE_MANDATORY_UNLOAD_TIMEOUT_MS
            : COMMON_PAUSE_ARMING_TIMEOUT_MS);
    payload += ",\"pause_watchdog_active\":" +
        String(commonPauseRuntimeWatchdogActive() ? "true" : "false");
    payload += ",\"pause_watchdog_profile\":\"" +
        String(pauseWatchdogProfile) + "\"";
    payload += ",\"pause_substate\":\"" +
        String(commonPauseRuntimeSubstateName(conveyor, manipulator)) + "\"";
    payload += ",\"pause_watchdog_policy_reason\":\"" +
        String(pauseWatchdogPolicyReason) + "\"";
    payload += ",\"pause_force_drain_stable_wait_policy\":" +
        String(pauseForceDrainStableWaitPolicyActive ? "true" : "false");
    payload += ",\"pause_no_progress_guard\":\"" +
        String(pauseMandatoryUnloadPolicyActive ? "hold" : "normal") + "\"";
    payload += ",\"pause_mandatory_unload_active\":" +
        String(g_pauseRuntime.mandatoryUnloadActive ? "true" : "false");
    payload += ",\"pause_mandatory_unload_in_flight\":" +
        String(pauseMandatoryUnloadInFlight ? "true" : "false");
    payload += ",\"pause_mandatory_unload_elapsed_ms\":" +
        String(pauseMandatoryUnloadElapsedMs);
    payload += ",\"pause_progress_observed\":" + String(g_pauseRuntime.progressObserved ? "true" : "false");
    payload += ",\"buffer_count\":" + String(g_pauseRuntime.conveyorSnapshot.bufferCount);
    payload += ",\"buffer_count_valid\":" + String(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "true" : "false");
    payload += ",\"buffer_count_source\":\"" +
        String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource)) + "\"";
    payload += ",\"sealer_count\":" + String(g_pauseRuntime.conveyorSnapshot.sealerCount);
    payload += ",\"sealer_count_valid\":" + String(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "true" : "false");
    payload += ",\"sealer_count_source\":\"" +
        String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource)) + "\"";
    payload += ",\"sealer_residual_known\":" + String(g_pauseRuntime.conveyorSnapshot.sealerResidualKnown ? "true" : "false");
    payload += ",\"buffer_count_fresh\":" + String(g_pauseRuntime.conveyorSnapshot.bufferCountFresh ? "true" : "false");
    payload += ",\"sealer_count_fresh\":" + String(g_pauseRuntime.conveyorSnapshot.sealerCountFresh ? "true" : "false");
    payload += ",\"pause_node_epoch_confirmed\":" +
        String(g_pauseRuntime.conveyorSnapshot.nodePauseEpochConfirmed ? "true" : "false");
    payload += ",\"pause_contract_trusted\":" +
        String(g_pauseRuntime.conveyorSnapshot.contractSnapshotTrusted ? "true" : "false");
    payload += ",\"pause_node_state\":\"" +
        String(commonPauseNodeStateName(g_pauseRuntime.conveyorSnapshot.nodeState)) + "\"";
    payload += ",\"pause_node_state_fresh\":" +
        String(g_pauseRuntime.conveyorSnapshot.nodeStateFresh ? "true" : "false");
    payload += ",\"pause_node_ready_confirmed\":" +
        String(g_pauseRuntime.nodeReadyConfirmed ? "true" : "false");
    payload += ",\"pause_finalize_started\":" +
        String(g_pauseRuntime.finalizeStarted ? "true" : "false");
    const bool pauseFillLastBlockBlocked =
        g_pauseRuntime.fillLastBlockBlockedSinceMs != 0;
    payload += ",\"pause_fill_last_block_blocked\":" +
        String(pauseFillLastBlockBlocked ? "true" : "false");
    payload += ",\"pause_fill_last_block_blocked_ms\":" +
        String(
            pauseFillLastBlockBlocked
                ? static_cast<uint32_t>(millis() - g_pauseRuntime.fillLastBlockBlockedSinceMs)
                : 0U);
    payload += ",\"stage\":\"" + String(commonCycleStageName(g_commonCycle.stage)) + "\"";
    payload += ",\"manip_starts\":" + String(g_commonCycle.manipStarts);
    payload += ",\"parallel_starts\":" + String(g_commonCycle.parallelStarts);
    payload += ",\"step3_seen\":" + String(g_commonCycle.step3Seen ? "true" : "false");
    payload += ",\"step3_refill_done\":" + String(g_commonCycle.step3LaunchDone ? "true" : "false");
    payload += ",\"refill_pending\":" + String(g_commonCycle.refillStartPending ? "true" : "false");
    payload += ",\"refill_pending_ms\":" +
        String((g_commonCycle.refillStartPending && g_commonCycle.refillStartRequestedMs != 0)
            ? static_cast<uint32_t>(millis() - g_commonCycle.refillStartRequestedMs)
            : 0U);
    payload += ",\"batch_seq\":" + String(g_commonCycle.lastConsumedBatchSeq);
    payload += ",\"next_batch_ready\":" +
        String(conveyorNextBatchReadyForCommon(conveyor, g_commonCycle.lastConsumedBatchSeq) ? "true" : "false");
    payload += ",\"ready_batch_stale\":" +
        String(conveyorReadyBatchIsStaleForCommon(conveyor, g_commonCycle.lastConsumedBatchSeq) ? "true" : "false");
    payload += ",\"outfeed_ready\":" +
        String((conveyor.online && conveyor.protocolOk && conveyorOutfeedReadyForBatch(conveyor)) ? "true" : "false");
    payload += ",\"outfeed_seen_not_ready\":" +
        String(g_commonCycle.outfeedNotReadyObserved ? "true" : "false");
    payload += ",\"sealer_done_pending_unload\":" + String(g_commonCycle.sealerDonePendingUnload ? "true" : "false");
    payload += ",\"pause_nodes\":{";
    payload += "\"conveyor\":{";
    payload += "\"pause_state\":\"" + String(commonPauseRuntimeStateName(g_pauseRuntime.conveyorSnapshot.pauseState)) + "\"";
    payload += ",\"node_state\":\"" +
        String(commonPauseNodeStateName(g_pauseRuntime.conveyorSnapshot.nodeState)) + "\"";
    payload += ",\"pause_epoch\":" + String(g_pauseRuntime.conveyorSnapshot.pauseEpoch);
    payload += ",\"buffer_count\":" + String(g_pauseRuntime.conveyorSnapshot.bufferCount);
    payload += ",\"buffer_count_valid\":" + String(g_pauseRuntime.conveyorSnapshot.bufferCountValid ? "true" : "false");
    payload += ",\"buffer_count_source\":\"" +
        String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.bufferCountSource)) + "\"";
    payload += ",\"sealer_count\":" + String(g_pauseRuntime.conveyorSnapshot.sealerCount);
    payload += ",\"sealer_count_valid\":" + String(g_pauseRuntime.conveyorSnapshot.sealerCountValid ? "true" : "false");
    payload += ",\"sealer_count_source\":\"" +
        String(pauseCountSourceName(g_pauseRuntime.conveyorSnapshot.sealerCountSource)) + "\"";
    payload += ",\"node_state_fresh\":" +
        String(g_pauseRuntime.conveyorSnapshot.nodeStateFresh ? "true" : "false");
    payload += ",\"manual_recovery_required\":" +
        String(g_pauseRuntime.conveyorSnapshot.manualRecoveryRequired ? "true" : "false");
    payload += "},";
    payload += "\"manipulator\":{";
    payload += "\"pause_state\":\"" + String(commonPauseRuntimeStateName(g_pauseRuntime.manipulatorSnapshot.pauseState)) + "\"";
    payload += ",\"pause_epoch\":" + String(g_pauseRuntime.manipulatorSnapshot.pauseEpoch);
    payload += ",\"buffer_count\":" + String(g_pauseRuntime.manipulatorSnapshot.bufferCount);
    payload += ",\"buffer_count_valid\":" + String(g_pauseRuntime.manipulatorSnapshot.bufferCountValid ? "true" : "false");
    payload += ",\"buffer_count_source\":\"" +
        String(pauseCountSourceName(g_pauseRuntime.manipulatorSnapshot.bufferCountSource)) + "\"";
    payload += ",\"sealer_count\":" + String(g_pauseRuntime.manipulatorSnapshot.sealerCount);
    payload += ",\"sealer_count_valid\":" + String(g_pauseRuntime.manipulatorSnapshot.sealerCountValid ? "true" : "false");
    payload += ",\"sealer_count_source\":\"" +
        String(pauseCountSourceName(g_pauseRuntime.manipulatorSnapshot.sealerCountSource)) + "\"";
    payload += ",\"manual_recovery_required\":" +
        String(g_pauseRuntime.manipulatorSnapshot.manualRecoveryRequired ? "true" : "false");
    payload += "}";
    payload += "}";
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
    Serial.println("  MBSCAN <...>      - background sweep; 12/13 mandatory poll stays for runtime");
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
    printScanDiagStatusLine();

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
    Serial.print(", vfd_timed_run=");
    Serial.print(conveyorVfdTimedRunActive(conveyor) ? "yes" : "no");
    Serial.print(", outfeed_ready=");
    Serial.print(conveyor.protocolOk && conveyorOutfeedReadyForBatch(conveyor) ? "yes" : "no");
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
    Serial.print(", busy=");
    Serial.print(deviceStatusBusy(manipulator) ? "yes" : "no");
    Serial.print(", alarm=");
    Serial.print(deviceStatusAlarm(manipulator) ? "yes" : "no");
    Serial.print(", mode=");
    Serial.print(manipulator.extra0);
    Serial.print(", job=");
    Serial.print(manipulator.extra1);
    Serial.print(", work_start_ready=");
    Serial.print(manipulatorInWorkStartPose(manipulator) ? "yes" : "no");
    Serial.print(", needs_q_only=");
    Serial.print(manipulatorNeedsOnlyGripOpenForWorkStart(manipulator) ? "yes" : "no");
    Serial.print(", work_start_ambiguous=");
    Serial.print(manipulatorWorkStartGripUnknown(manipulator) ? "yes" : "no");
    Serial.print(", sensor_bits=");
    Serial.println(manipulatorSensorBits(manipulator));

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

    const uint32_t now = millis();
    Serial.print("Loop diag: stage=");
    Serial.print(g_loopDiag.currentStage);
    Serial.print(", stage_ms=");
    Serial.print(static_cast<uint32_t>(now - g_loopDiag.currentStageSinceMs));
    Serial.print(", tick_seq=");
    Serial.print(g_loopDiag.tickSeq);
    Serial.print(", tick_max_ms=");
    Serial.println(g_loopDiag.maxTickDurationMs);

    Serial.print("I2C diag: active=");
    Serial.print(g_i2cDiag.active ? "yes" : "no");
    Serial.print(", active_op=");
    Serial.print(g_i2cDiag.activeOp);
    Serial.print(", active_origin=");
    Serial.print(g_i2cDiag.activeOrigin);
    Serial.print(", active_id=");
    Serial.print(g_i2cDiag.activeId);
    Serial.print(", active_ms=");
    Serial.print(g_i2cDiag.active ? static_cast<uint32_t>(now - g_i2cDiag.activeStartedMs) : 0U);
    Serial.print(", last_op=");
    Serial.print(g_i2cDiag.lastOp);
    Serial.print(", last_origin=");
    Serial.print(g_i2cDiag.lastOrigin);
    Serial.print(", last_outcome=");
    Serial.print(g_i2cDiag.lastOutcome);
    Serial.print(", last_ms=");
    Serial.print(g_i2cDiag.lastDurationMs);
    Serial.print(", max_ms=");
    Serial.print(g_i2cDiag.maxDurationMs);
    Serial.print(", fails=");
    Serial.print(g_i2cDiag.failCount);
    Serial.print(", slow=");
    Serial.print(g_i2cDiag.slowCount);
    Serial.print(", guard=");
    Serial.print(g_i2cDiag.guardTripCount);
    Serial.print(", unfinished=");
    Serial.print(g_i2cDiag.active ? "yes" : "no");
    Serial.println();
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
        Serial.println("  OFF disables only background sweep; mandatory poll for 12/13 stays active.");
        return;
    }

    if (sub == "STATUS") {
        rs485ScanPrintStatus();
        return;
    }

    if (sub == "ON") {
        g_rs485ScanEnabled = true;
        g_rs485ScanLastStepMs = 0;
        Serial.println("MBSCAN: enabled (background periodic sweep over selected range).");
        rs485ScanPrintStatus();
        return;
    }

    if (sub == "OFF") {
        g_rs485ScanEnabled = false;
        g_rs485ScanLastStepMs = 0;
        rs485ScanProbeId(CONVEYOR_ID, "core_poll");
        rs485ScanProbeId(MANIPULATOR_ID, "core_poll");
        Serial.println("MBSCAN: background sweep disabled; mandatory poll for 12/13 remains active.");
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

        rs485ScanProbeId(static_cast<uint8_t>(id), "manual_now");
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
    rs485ProbeManagedDevice(CONVEYOR_ID, DEVICE_KIND_CONVEYOR, "otvod_poll");
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

bool pollOtvodVfdCycleState(bool &vfdTimedRunActiveOut, uint32_t &tickDurationMsOut)
{
    if (!pollOtvodConveyorState()) {
        return false;
    }

    const Rs485DeviceState &conveyor = g_rs485Devices[CONVEYOR_ID];
    vfdTimedRunActiveOut = conveyorVfdTimedRunActive(conveyor);
    tickDurationMsOut = conveyorVfdTickDurationMs(conveyor);
    return true;
}

void printOtvodWorkCycleStatus()
{
    uint32_t tickDurationMs = g_otvodWorkCycle.vfdDoneDelayMs;
    if (!g_otvodWorkCycle.active && tickDurationMs == 0) {
        bool vfdTimedRunActive = false;
        uint32_t polledTickMs = 0;
        if (pollOtvodVfdCycleState(vfdTimedRunActive, polledTickMs) && polledTickMs > 0) {
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
    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, command, "otvod_step2_start")) {
        Serial.println("OTCYCLE: failed to send OTVOD command to conveyor.");
        return false;
    }

    g_otvodWorkCycle.stepRunning = true;
    g_otvodWorkCycle.stepObservedActive = false;
    g_otvodWorkCycle.stepRunsStarted = cycleIndex;
    g_otvodWorkCycle.stepCompletionSeqBase = conveyorStep2CompletionSeq(g_rs485Devices[CONVEYOR_ID]);
    g_otvodWorkCycle.vfdStartRetryCount = 0;
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
        bool vfdTimedRunActive = false;
        if (!pollOtvodVfdCycleState(vfdTimedRunActive, tickDurationMs)) {
            Serial.println("OTCYCLE: failed to read conveyor VFDTICK state.");
            return false;
        }
    }

    if (tickDurationMs == 0) {
        Serial.println("OTCYCLE: conveyor VFDTICK duration is not configured.");
        return false;
    }

    if (!i2cSendManagedDeviceCommand(CONVEYOR_ID, "VFDTICK RUN", "otvod_vfdtick_run")) {
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
        (void)i2cSendManagedDeviceCommand(CONVEYOR_ID, "VFDSTOP", "otvod_stop_vfd");
    }
    (void)i2cSendManagedDeviceCommand(CONVEYOR_ID, "STEP2STOP", "otvod_stop_step2");
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
        bool vfdTimedRunActive = false;
        if (!pollOtvodVfdCycleState(vfdTimedRunActive, vfdDoneDelayMs)) {
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

            bool vfdTimedRunActive = false;
            uint32_t tickDurationMs = 0;
            if (pollOtvodVfdCycleState(vfdTimedRunActive, tickDurationMs)) {
                if (tickDurationMs > 0) {
                    g_otvodWorkCycle.vfdDoneDelayMs = tickDurationMs;
                }
                if (vfdTimedRunActive) {
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
            if (g_otvodWorkCycle.vfdStartRetryCount < OTVOD_WORK_VFD_START_RETRY_MAX) {
                g_otvodWorkCycle.vfdStartRetryCount++;
                Serial.print("OTCYCLE: VFDTICK start retry ");
                Serial.print(g_otvodWorkCycle.vfdStartRetryCount);
                Serial.print("/");
                Serial.println(OTVOD_WORK_VFD_START_RETRY_MAX);
                if (!startOtvodWorkVfdRun(g_otvodWorkCycle.stepRunsStarted)) {
                    abortOtvodWorkCycle("OTCYCLE failed: VFDTICK retry start", true);
                    return;
                }
                return;
            }
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
        Serial.println("  COMMON START            - start new common production cycle");
        Serial.println("  COMMON PAUSE            - enter pause arming");
        Serial.println("  COMMON PAUSE STATUS     - print pause runtime state");
        Serial.println("  COMMON PAUSE RELEASE    - release pause hold/empty; old cycle is not resumed");
        Serial.println("  COMMON RECOVERY CLEAR - clear master manual recovery latch");
        Serial.println("  COMMON STATUS           - print common cycle state");
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
        String action = nextToken(args);
        action.toUpperCase();
        if (action.isEmpty()) {
            requestCommonCyclePause();
            return;
        }
        if (action == "STATUS" || action == "STATE") {
            printCommonCycleStatus();
            g_lastCommandResult = "COMMON PAUSE status printed";
            return;
        }
        if (action == "RELEASE") {
            (void)releaseCommonCyclePause();
            return;
        }
        Serial.println("Usage: COMMON PAUSE [STATUS|RELEASE]");
        g_lastCommandResult = "COMMON PAUSE failed: unknown subcommand";
        return;
    }

    if (sub == "RECOVERY") {
        String action = nextToken(args);
        action.toUpperCase();
        if (action == "CLEAR") {
            (void)clearCommonRecoveryState();
            return;
        }
        Serial.println("Usage: COMMON RECOVERY CLEAR");
        g_lastCommandResult = "COMMON RECOVERY failed: unknown subcommand";
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
                rs485ScanProbeId(VFD_ID, "vfd_find");
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
    Serial.print("I2C timeout ms: ");
    Serial.println(I2C_TIMEOUT_MS);
    Serial.print("I2C wire trace sample: every ");
    Serial.print(I2C_WIRE_TRACE_SAMPLE_EVERY);
    Serial.println(" ops (+forced on anomalies).");
    Serial.println("SEAL: owner=conveyor(12) via I2C managed commands/status.");
    Serial.print("MODBUS default slave id: ");
    Serial.println(g_modbusSlaveId);
    Serial.print("MBSCAN: ");
    Serial.print(g_rs485ScanEnabled ? "on" : "off");
    Serial.print(", range=");
    Serial.print(g_rs485ScanMinId);
    Serial.print("..");
    Serial.println(g_rs485ScanMaxId);
    Serial.print("MBSCAN OFF keeps mandatory poll for 12/13 every ");
    Serial.print(RS485_SCAN_STEP_INTERVAL_MS);
    Serial.println(" ms.");
    wifiPrintStatus();
    mqttPrintStatus();
    printHelp();
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);
    g_loopDiag.currentStage = "setup";
    g_loopDiag.currentStageSinceMs = millis();

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
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    rs485ScanResetAll();

    printBanner();
    loopMarkStage("idle");
}

void loop()
{
    const uint32_t tickStartedMs = millis();
    g_loopDiag.tickSeq++;
    g_loopDiag.lastTickStartedMs = tickStartedMs;

    loopMarkStage("console");
    loopPrintHeartbeat();
    processConsole();
    loopMarkStage("scan");
    loopPrintHeartbeat();
    rs485ScanLoop();
    loopMarkStage("mqtt");
    loopPrintHeartbeat();
    mqttLoop();
    loopMarkStage("seal");
    loopPrintHeartbeat();
    processSealIo();
    loopMarkStage("otvod");
    loopPrintHeartbeat();
    processOtvodWorkCycleStrict();
    loopMarkStage("common");
    loopPrintHeartbeat();
    processCommonCycle();
    loopMarkStage("rs485_diag");
    loopPrintHeartbeat();
    processRs485DiagPassive();
    if (RS485_TEXT_SNIFFER_ENABLED) {
        loopMarkStage("rs485_sniffer");
        loopPrintHeartbeat();
        processRs485RxText();
    }
    i2cCheckActiveGuard();
    loopPrintHeartbeat();
    loopMarkStage("idle");

    const uint32_t tickDurationMs = static_cast<uint32_t>(millis() - tickStartedMs);
    g_loopDiag.lastTickCompletedMs = millis();
    if (tickDurationMs > g_loopDiag.maxTickDurationMs) {
        g_loopDiag.maxTickDurationMs = tickDurationMs;
    }
}
