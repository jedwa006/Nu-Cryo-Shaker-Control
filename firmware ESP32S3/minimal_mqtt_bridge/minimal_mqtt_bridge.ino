/*
 * minimal_mqtt_bridge.ino
 *
 * Nu-Cryo Shaker – ESP32-S3-POE-ETH-8DI-8RO bridge
 *
 * Version history
 *  v0.1 – First minimal MQTT bridge over WiFi (prototype).
 *  v0.2 – Added basic JSON status + Node-RED integration.
 *  v0.3 – Mapped DIN interlocks and status topic mill/status/state.
 *  v0.4 – Switched to Waveshare DIN / GPIO APIs, cleaned JSON structure.
 *  v0.5 – Use Waveshare WS_ETH (W5500 Ethernet) instead of WiFi,
 *         static IP support, disable auto DIN→relay mapping,
 *         door interlock temporarily mirrored to lid for bring-up.
 */

#include <Arduino.h>
#include <PubSubClient.h>

#include "WS_GPIO.h"
#include "WS_DIN.h"
#include "WS_ETH.h"

// --- Externals from Waveshare libs ----------------------------------

// From WS_DIN.cpp
extern bool DIN_Read_CH1(void);
extern bool DIN_Read_CH2(void);
extern bool DIN_Read_CH3(void);
extern bool Relay_Immediate_Enable;   // auto DIN→relay mapping flag

// --- Debug options --------------------------------------------------
const bool STATUS_SERIAL_DEBUG = false;   // set to true to see STATUS JSON on Serial

// --- Network / MQTT configuration -----------------------------------

static const IPAddress ETH_LOCAL_IP(192, 168, 50, 10);
static const IPAddress ETH_GATEWAY(192, 168, 50, 1);
static const IPAddress ETH_SUBNET(255, 255, 255, 0);
static const IPAddress ETH_DNS(192, 168, 50, 2);  // Pi as DNS (mostly irrelevant on this link)

static const char *MQTT_HOST          = "192.168.50.2";
static const uint16_t MQTT_PORT       = 1883;
static const char *MQTT_CLIENT_ID     = "nu-cryo-esp32-s3";
static const char *MQTT_STATUS_TOPIC  = "mill/status/state";
static const char *MQTT_CMD_SUB_TOPIC = "mill/cmd/#";

// Use the generic Network client provided by the ESP32 core
NetworkClient netClient;
PubSubClient mqttClient(netClient);

// --- Mill state machine ---------------------------------------------

enum MillState {
  MILL_IDLE = 0,
  MILL_RUN,
  MILL_HOLD,
  MILL_FAULT
};

MillState millState = MILL_IDLE;

const char *stateToString(MillState s) {
  switch (s) {
    case MILL_RUN:   return "RUN";
    case MILL_HOLD:  return "HOLD";
    case MILL_FAULT: return "FAULT";
    case MILL_IDLE:
    default:         return "IDLE";
  }
}

// --- Interlocks & process variables ---------------------------------

// Hardware mapping (DIN pins):
//   CH1 → E-Stop OK
//   CH2 → Lid locked
//   CH3 → Door closed (not yet wired, currently mirrored to CH2)
bool estop_ok    = false;
bool lid_locked  = false;
bool door_closed = false;

// Temporary hack: mirror "door_closed" from lid until DI3 is wired.
bool mirror_door_to_lid = true;

// Small Helper and Tracking variable for interlock status
bool allInterlocksOk() {
  return estop_ok && lid_locked && door_closed;
}

// Interlock global status tracking
bool lastInterlocksOk = false;


// Cycle / timing placeholders (can be wired to real logic later)
uint32_t cycle_current    = 0;
uint32_t cycle_target     = 0;
uint32_t time_remaining_s = 0;

// LN2 PV temperature (°C) – placeholder until a real sensor is wired in
float ln2_pv_c = 0.0f;

// --- Timing ---------------------------------------------------------

unsigned long lastStatusPublishMs      = 0;
const unsigned long STATUS_PUBLISH_MS  = 500;   // 2 Hz to Node-RED

unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_MS  = 2000;  // 2 s

// --- Forward declarations -------------------------------------------

void mqttCallback(char *topic, byte *payload, unsigned int length);
void mqttReconnect();
void publishStatus();
void checkInterlocks();
void handleCommand(const String &cmd);

// -------------------------------------------------------------------
// Interlocks
// -------------------------------------------------------------------

void checkInterlocks() {
  // Raw DIN reads: HIGH = 1, LOW = 0 (because of INPUT_PULLUP)
  bool din_estop   = DIN_Read_CH1();
  bool din_lid     = DIN_Read_CH2();
  bool din_door    = mirror_door_to_lid ? din_lid : DIN_Read_CH3();

  // Invert semantics so:
  //   LOW  (pressed / closed to GND) = OK
  //   HIGH (released / open / broken) = FAULT
  estop_ok    = !din_estop;
  lid_locked  = !din_lid;
  door_closed = !din_door;
}

// -------------------------------------------------------------------
// MQTT command handling
// -------------------------------------------------------------------

void handleCommand(const String &cmd) {
  // Always evaluate commands against *fresh* interlock state
  checkInterlocks();

  if (cmd == "START") {
    // Only allow RUN if all interlocks are good and we are not faulted
    if (estop_ok && lid_locked && door_closed && millState != MILL_FAULT) {
      millState = MILL_RUN;
      Serial.println("[CMD] START → RUN");
    } else {
      // Trying to start with bad interlocks => fault
      millState = MILL_FAULT;
      Serial.println("[CMD] START blocked → FAULT (interlocks not OK)");
    }
  }
  else if (cmd == "STOP") {
    millState = MILL_IDLE;
    Serial.println("[CMD] STOP → IDLE");
  }
  else if (cmd == "HOLD") {
    if (millState == MILL_RUN) {
      millState = MILL_HOLD;
      Serial.println("[CMD] HOLD → HOLD");
    } else {
      Serial.println("[CMD] HOLD ignored (not in RUN)");
    }
  }
  else if (cmd == "RESET_FAULT") {
    if (millState == MILL_FAULT && estop_ok && lid_locked && door_closed) {
      millState = MILL_IDLE;
      Serial.println("[CMD] RESET_FAULT → IDLE");
    } else {
      Serial.println("[CMD] RESET_FAULT ignored (not in FAULT or interlocks bad)");
    }
  }
}

// MQTT callback from PubSubClient
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String t(topic);
  String body;
  body.reserve(length + 1);
  for (unsigned int i = 0; i < length; ++i) {
    body += static_cast<char>(payload[i]);
  }

  Serial.print("[MQTT] RX ");
  Serial.print(t);
  Serial.print(" : ");
  Serial.println(body);

  if (!t.startsWith("mill/cmd")) {
    return;
  }

  // Extremely simple parse: just look for the command token
  String cmd;
  if      (body.indexOf("RESET_FAULT") >= 0) cmd = "RESET_FAULT";
  else if (body.indexOf("START")       >= 0) cmd = "START";
  else if (body.indexOf("STOP")        >= 0) cmd = "STOP";
  else if (body.indexOf("HOLD")        >= 0) cmd = "HOLD";

  if (cmd.length() > 0) {
    handleCommand(cmd);
  } else {
    Serial.println("[MQTT] No recognized cmd in payload");
  }
}

// -------------------------------------------------------------------
// MQTT connect/reconnect
// -------------------------------------------------------------------

void mqttReconnect() {
  // Don't hammer the broker if the interface has no IP yet
  if (ETH.localIP() == IPAddress(0, 0, 0, 0)) {
    return;
  }

  if (mqttClient.connected()) {
    return;
  }

  Serial.print("[MQTT] Connecting to broker ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println("[MQTT] Connected");
    mqttClient.subscribe(MQTT_CMD_SUB_TOPIC);
    Serial.print("[MQTT] Subscribed to ");
    Serial.println(MQTT_CMD_SUB_TOPIC);
  } else {
    Serial.print("[MQTT] Connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// -------------------------------------------------------------------
// Status JSON publish
// -------------------------------------------------------------------

void publishStatus() {
  checkInterlocks();

  // Simple manual JSON build compatible with Node-RED "Prepare UI state"
  String json = "{";

  json += "\"state\":\"";
  json += stateToString(millState);
  json += "\",";

  json += "\"cycle_current\":";
  json += String(cycle_current);
  json += ",";

  json += "\"cycle_target\":";
  json += String(cycle_target);
  json += ",";

  json += "\"time_remaining_s\":";
  json += String(time_remaining_s);
  json += ",";

  // pid: { pv_c }
  json += "\"pid\":{";
  json += "\"pv_c\":";
  json += String(ln2_pv_c, 1);  // one decimal place
  json += "},";

  // interlocks: { door_closed, estop_ok, lid_locked }
  json += "\"interlocks\":{";
  json += "\"door_closed\":";
  json += door_closed ? "true" : "false";
  json += ",";
  json += "\"estop_ok\":";
  json += estop_ok ? "true" : "false";
  json += ",";
  json += "\"lid_locked\":";
  json += lid_locked ? "true" : "false";
  json += "}";

  json += "}";

  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_STATUS_TOPIC, json.c_str());
  }

  // Optional debug
  if (STATUS_SERIAL_DEBUG) {
    Serial.print("[STATUS] ");
    Serial.println(json);
  }
}

// -------------------------------------------------------------------
// setup() / loop()
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("Nu-Cryo minimal_mqtt_bridge v0.5 (Ethernet + Node-RED)");

  // Waveshare RGB + Buzzer
  GPIO_Init();

  // Disable auto "DIN→Relay" behavior for now; we are not driving relays yet.
  Relay_Immediate_Enable = false;

  // Initialize digital inputs + background task
  DIN_Init();

  // Bring up Ethernet via Waveshare helper
  ETH_Init();

  // Apply static IP configuration for your 192.168.50.x lab network
  ETH.config(ETH_LOCAL_IP, ETH_GATEWAY, ETH_SUBNET, ETH_DNS);

  // MQTT client setup
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  Serial.print("ETH local IP (after init): ");
  Serial.println(ETH.localIP());
}

void loop() {
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt >= MQTT_RECONNECT_MS) {
      lastMqttReconnectAttempt = now;
      mqttReconnect();
    }
  } else {
    mqttClient.loop();
  }

  // Periodic status publish
  unsigned long now = millis();
  if (mqttClient.connected() &&
      (now - lastStatusPublishMs >= STATUS_PUBLISH_MS)) {
    lastStatusPublishMs = now;
    publishStatus();
  }

  // Interlock monitoring: drop to FAULT if any interlock goes bad
  checkInterlocks();
  bool currentOk = allInterlocksOk();

  if (!currentOk && lastInterlocksOk && millState != MILL_FAULT) {
    millState = MILL_FAULT;
    Serial.println("[SAFETY] Interlock opened → FAULT");
  }

  lastInterlocksOk = currentOk;

  // Diesel idle – let FreeRTOS tasks (DIN, RGB, Buzzer, ETH) breathe
  delay(10);
}
