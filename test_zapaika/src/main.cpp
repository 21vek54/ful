#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

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
#define MQTT_LOCAL_PORT 1884
#define MQTT_LOCAL_USER ""
#define MQTT_LOCAL_PASSWORD ""
#define MQTT_LOCAL_BASE_TOPIC "fyl/test_zapaika"
#endif

namespace {

constexpr char FW_VERSION[] = "v0.1-test-zapaika";
constexpr char DEVICE_NAME[] = "Test zapaika controller";
constexpr char HOSTNAME[] = "test-zapaika";

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint8_t PIN_START_RELAY = 23;
constexpr uint8_t PIN_DONE_INPUT = 25;
constexpr bool START_ACTIVE_LEVEL = HIGH;
constexpr bool DONE_ACTIVE_LEVEL = LOW; // INPUT_PULLUP + relay contact to GND
constexpr uint32_t START_PULSE_MS_DEFAULT = 300;
constexpr uint32_t START_PULSE_MS_MIN = 50;
constexpr uint32_t START_PULSE_MS_MAX = 5000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
constexpr uint32_t MQTT_STATUS_INTERVAL_MS = 2000;

WiFiClient g_wifiClient;
PubSubClient g_mqttClient(g_wifiClient);

bool g_startPulseActive = false;
bool g_startOutputActive = false;
bool g_doneLastActive = false;
bool g_cycleRunning = false;
bool g_otaStarted = false;
uint32_t g_startPulseStartedMs = 0;
uint32_t g_startPulseDurationMs = START_PULSE_MS_DEFAULT;
uint32_t g_doneCount = 0;
uint32_t g_lastWifiAttemptMs = 0;
uint32_t g_lastMqttAttemptMs = 0;
uint32_t g_lastStatusPublishMs = 0;
String g_lastResult = "ожидание";
String g_mqttHost = MQTT_LOCAL_HOST;
uint16_t g_mqttPort = MQTT_LOCAL_PORT;
String g_mqttUser = MQTT_LOCAL_USER;
String g_mqttPassword = MQTT_LOCAL_PASSWORD;
String g_mqttBaseTopic = MQTT_LOCAL_BASE_TOPIC;

String escapeJsonString(const String &value)
{
    String out = value;
    out.replace("\\", "\\\\");
    out.replace("\"", "\\\"");
    out.replace("\r", " ");
    out.replace("\n", " ");
    return out;
}

String topicCmd()
{
    return g_mqttBaseTopic + "/cmd";
}

String topicStatus()
{
    return g_mqttBaseTopic + "/status";
}

String topicResp()
{
    return g_mqttBaseTopic + "/resp";
}

void writeStartOutput(bool active)
{
    digitalWrite(PIN_START_RELAY, active ? START_ACTIVE_LEVEL : !START_ACTIVE_LEVEL);
    g_startOutputActive = active;
}

bool isDoneActive()
{
    return digitalRead(PIN_DONE_INPUT) == DONE_ACTIVE_LEVEL;
}

String buildStatusJson()
{
    String payload = "{";
    payload += "\"fw\":\"" + String(FW_VERSION) + "\"";
    payload += ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
    payload += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
    payload += ",\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
    payload += ",\"start_pin\":" + String(PIN_START_RELAY);
    payload += ",\"done_pin\":" + String(PIN_DONE_INPUT);
    payload += ",\"start_out\":" + String(g_startOutputActive ? "true" : "false");
    payload += ",\"done_active\":" + String(isDoneActive() ? "true" : "false");
    payload += ",\"cycle_running\":" + String(g_cycleRunning ? "true" : "false");
    payload += ",\"done_count\":" + String(g_doneCount);
    payload += ",\"pulse_ms\":" + String(g_startPulseDurationMs);
    payload += ",\"last_result\":\"" + escapeJsonString(g_lastResult) + "\"";
    payload += "}";
    return payload;
}

bool mqttPublish(const String &topic, const String &payload, bool retained = false)
{
    if (!g_mqttClient.connected()) {
        return false;
    }
    return g_mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

void publishStatus(bool retained = false)
{
    const String payload = buildStatusJson();
    (void)mqttPublish(topicStatus(), payload, retained);
}

void publishResponse(const String &result)
{
    String payload = "{";
    payload += "\"ok\":true";
    payload += ",\"result\":\"" + escapeJsonString(result) + "\"";
    payload += "}";
    (void)mqttPublish(topicResp(), payload, false);
}

bool parseU32(String token, uint32_t &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    uint32_t acc = 0;
    for (size_t i = 0; i < token.length(); i++) {
        const char c = token[i];
        if (c < '0' || c > '9') {
            return false;
        }
        const uint32_t digit = static_cast<uint32_t>(c - '0');
        if (acc > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        acc = (acc * 10U) + digit;
    }
    value = acc;
    return true;
}

String nextToken(String &s)
{
    s.trim();
    if (s.isEmpty()) {
        return String();
    }
    const int splitPos = s.indexOf(' ');
    if (splitPos < 0) {
        String token = s;
        s = "";
        token.trim();
        return token;
    }
    String token = s.substring(0, splitPos);
    s = s.substring(splitPos + 1);
    token.trim();
    s.trim();
    return token;
}

void printStatus()
{
    Serial.print("WiFi: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("MQTT: ");
    Serial.println(g_mqttClient.connected() ? "connected" : "disconnected");
    Serial.print("START: pin=");
    Serial.print(PIN_START_RELAY);
    Serial.print(", out=");
    Serial.println(g_startOutputActive ? "on" : "off");
    Serial.print("DONE: pin=");
    Serial.print(PIN_DONE_INPUT);
    Serial.print(", active=");
    Serial.println(isDoneActive() ? "yes" : "no");
    Serial.print("Cycle running: ");
    Serial.println(g_cycleRunning ? "yes" : "no");
    Serial.print("Done count: ");
    Serial.println(g_doneCount);
    Serial.print("Last result: ");
    Serial.println(g_lastResult);
}

void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  START [ms]  - pulse start relay, default 300 ms");
    Serial.println("  OUT ON      - force start relay ON");
    Serial.println("  OUT OFF     - force start relay OFF");
    Serial.println("  STATUS      - print current state");
    Serial.println("  H / HELP    - help");
}

void startPulse(uint32_t pulseMs)
{
    g_startPulseDurationMs = pulseMs;
    g_startPulseStartedMs = millis();
    g_startPulseActive = true;
    g_cycleRunning = true;
    writeStartOutput(true);
    g_lastResult = "START pulse sent";

    Serial.print("START pulse: ");
    Serial.print(pulseMs);
    Serial.println(" ms");
    publishStatus(false);
}

void handleCommand(String command, bool publishRespEnabled)
{
    command.trim();
    if (command.isEmpty()) {
        return;
    }

    String args = command;
    String cmd = nextToken(args);
    cmd.toUpperCase();

    if (cmd == "H" || cmd == "HELP") {
        printHelp();
        g_lastResult = "help printed";
    } else if (cmd == "STATUS") {
        printStatus();
        g_lastResult = "status printed";
        publishStatus(false);
    } else if (cmd == "OUT") {
        String mode = nextToken(args);
        mode.toUpperCase();
        if (mode == "ON") {
            g_startPulseActive = false;
            writeStartOutput(true);
            g_lastResult = "OUT ON";
            Serial.println(g_lastResult);
        } else if (mode == "OFF") {
            g_startPulseActive = false;
            writeStartOutput(false);
            g_lastResult = "OUT OFF";
            Serial.println(g_lastResult);
        } else {
            g_lastResult = "OUT failed";
            Serial.println("Usage: OUT <ON|OFF>");
        }
    } else if (cmd == "START" || cmd == "PULSE") {
        String pulseTok = nextToken(args);
        uint32_t pulseMs = START_PULSE_MS_DEFAULT;
        if (!pulseTok.isEmpty()) {
            if (!parseU32(pulseTok, pulseMs) ||
                pulseMs < START_PULSE_MS_MIN || pulseMs > START_PULSE_MS_MAX) {
                g_lastResult = "START failed";
                Serial.println("Usage: START [50..5000]");
                if (publishRespEnabled) {
                    publishResponse(g_lastResult);
                }
                return;
            }
        }
        if (g_startPulseActive) {
            g_lastResult = "START ignored: pulse already active";
            Serial.println(g_lastResult);
        } else {
            startPulse(pulseMs);
        }
    } else {
        g_lastResult = "unknown command";
        Serial.println("Unknown command. Use H.");
    }

    if (publishRespEnabled) {
        publishResponse(g_lastResult);
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    String topicStr = topic ? String(topic) : String();
    String text;
    text.reserve(length);
    for (unsigned int i = 0; i < length; i++) {
        text += static_cast<char>(payload[i]);
    }
    text.trim();

    Serial.print("MQTT RX [");
    Serial.print(topicStr);
    Serial.print("]: ");
    Serial.println(text);

    if (topicStr == topicCmd()) {
        handleCommand(text, true);
    }
}

void wifiEnsureConnected()
{
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    if (!WIFI_LOCAL_SECRETS_AVAILABLE) {
        return;
    }

    if ((millis() - g_lastWifiAttemptMs) < WIFI_RECONNECT_INTERVAL_MS) {
        return;
    }
    g_lastWifiAttemptMs = millis();

    Serial.print("WIFI: connecting to ");
    Serial.println(WIFI_LOCAL_SSID);
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(WIFI_LOCAL_SSID, WIFI_LOCAL_PASSWORD);
}

void mqttEnsureConnected()
{
    if (WiFi.status() != WL_CONNECTED || g_mqttHost.isEmpty()) {
        return;
    }
    if (g_mqttClient.connected()) {
        return;
    }
    if ((millis() - g_lastMqttAttemptMs) < MQTT_RECONNECT_INTERVAL_MS) {
        return;
    }
    g_lastMqttAttemptMs = millis();

    g_mqttClient.setServer(g_mqttHost.c_str(), g_mqttPort);
    g_mqttClient.setCallback(mqttCallback);

    Serial.print("MQTT: connecting to ");
    Serial.print(g_mqttHost);
    Serial.print(":");
    Serial.println(g_mqttPort);

    const bool ok = g_mqttUser.isEmpty()
        ? g_mqttClient.connect(HOSTNAME)
        : g_mqttClient.connect(HOSTNAME, g_mqttUser.c_str(), g_mqttPassword.c_str());

    if (!ok) {
        Serial.print("MQTT: connect failed, rc=");
        Serial.println(g_mqttClient.state());
        return;
    }

    (void)g_mqttClient.subscribe(topicCmd().c_str(), 1);
    publishStatus(true);
    Serial.print("MQTT: subscribed to ");
    Serial.println(topicCmd());
}

void otaEnsureStarted()
{
    if (g_otaStarted || WiFi.status() != WL_CONNECTED) {
        return;
    }

    ArduinoOTA.begin();
    g_otaStarted = true;
    Serial.print("OTA: ready at ");
    Serial.println(WiFi.localIP());
}

void processStartPulse()
{
    if (!g_startPulseActive) {
        return;
    }

    if ((millis() - g_startPulseStartedMs) < g_startPulseDurationMs) {
        return;
    }

    g_startPulseActive = false;
    writeStartOutput(false);
    g_lastResult = "START pulse finished";
    Serial.println(g_lastResult);
    publishStatus(false);
}

void processDoneInput()
{
    const bool doneActive = isDoneActive();
    if (doneActive == g_doneLastActive) {
        return;
    }

    g_doneLastActive = doneActive;
    if (doneActive) {
        g_doneCount++;
        g_cycleRunning = false;
        g_lastResult = "cycle completed";
        Serial.println("DONE input became active.");
    } else {
        Serial.println("DONE input released.");
    }
    publishStatus(false);
}

void processSerial()
{
    static String buffer;
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            buffer.trim();
            if (!buffer.isEmpty()) {
                handleCommand(buffer, false);
            }
            buffer = "";
            continue;
        }
        buffer += c;
        if (buffer.length() > 160) {
            buffer = "";
        }
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
    Serial.print("Pins: START=");
    Serial.print(PIN_START_RELAY);
    Serial.print(", DONE=");
    Serial.println(PIN_DONE_INPUT);
    Serial.print("MQTT base topic: ");
    Serial.println(g_mqttBaseTopic);
    printHelp();
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    pinMode(PIN_START_RELAY, OUTPUT);
    writeStartOutput(false);
    pinMode(PIN_DONE_INPUT, INPUT_PULLUP);
    g_doneLastActive = isDoneActive();

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    g_lastWifiAttemptMs = millis() - WIFI_RECONNECT_INTERVAL_MS;
    g_lastMqttAttemptMs = millis() - MQTT_RECONNECT_INTERVAL_MS;

    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.onStart([]() {
        Serial.println("OTA: start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA: done");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.print("OTA error: ");
        Serial.println(static_cast<int>(error));
    });

    printBanner();
    wifiEnsureConnected();
}

void loop()
{
    processSerial();
    wifiEnsureConnected();
    otaEnsureStarted();
    mqttEnsureConnected();

    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }
    if (g_mqttClient.connected()) {
        g_mqttClient.loop();
        if ((millis() - g_lastStatusPublishMs) >= MQTT_STATUS_INTERVAL_MS) {
            g_lastStatusPublishMs = millis();
            publishStatus(false);
        }
    }

    processStartPulse();
    processDoneInput();
    delay(2);
}
