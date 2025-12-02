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
 *  v0.6 – Add simple cycle timer, CH1 relay control, and SET_CONFIG
 *         command to set cycle_target_s from Node-RED (ESP is source of truth).
 *  v0.7 - multi-cycle support and START lockout when no cycle config.
 */

#include <Arduino.h>
#include <PubSubClient.h>

#include "WS_GPIO.h"
#include "WS_DIN.h"
#include "WS_ETH.h"
#include "I2C_Driver.h"   // for I2C_Init and relay expander

// --- Externals from Waveshare libs ----------------------------------

// From WS_DIN.cpp
extern bool DIN_Read_CH1(void);
extern bool DIN_Read_CH2(void);
extern bool DIN_Read_CH3(void);
extern bool Relay_Immediate_Enable;   // auto DIN→relay mapping flag

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

// --- Debug options --------------------------------------------------

static const bool STATUS_SERIAL_DEBUG = false; // set true to see STATUS JSON on Serial

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

// LN2 PV temperature (°C) – placeholder until a real sensor is wired in
float ln2_pv_c = 0.0f;

// --- Cycle timing ---------------------------------------------------
static const uint32_t DEFAULT_CYCLE_TARGET_S = 300;  // 5 min default

uint32_t cycle_current    = 0;   // seconds into current cycle
uint32_t cycle_target     = 0;   // seconds per cycle
uint32_t time_remaining_s = 0;   // seconds left in current cycle

unsigned long lastCycleTickMs = 0;

// Multi-cycle
uint32_t cycle_total  = 0;   // total cycles requested (from HMI)
uint32_t cycle_index  = 0;   // 0 when idle, 1..cycle_total when running

// --- Relay control (mill motor on CH1) ------------------------------

bool millRelayState = false;  // our view of CH1 state

// --- Interlock tracking ---------------------------------------------

bool allInterlocksOk() {
  return estop_ok && lid_locked && door_closed;
}

bool lastInterlocksOk = false;

// --- MQTT timing ----------------------------------------------------

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
void handleConfig(const String &body);
void updateCycleTimer();
void updateRelayFromState();

// -------------------------------------------------------------------
// Interlocks
// -------------------------------------------------------------------

void checkInterlocks() {
  // Raw DIN reads: HIGH = 1, LOW = 0 (because of INPUT_PULLUP)
  bool din_estop = DIN_Read_CH1();
  bool din_lid   = DIN_Read_CH2();
  bool din_door  = mirror_door_to_lid ? din_lid : DIN_Read_CH3();

  // Invert semantics so:
  //   LOW  (pressed / closed to GND) = OK
  //   HIGH (released / open / broken) = FAULT
  estop_ok    = !din_estop;
  lid_locked  = !din_lid;
  door_closed = !din_door;
}

// -------------------------------------------------------------------
// Cycle timer
// -------------------------------------------------------------------

void updateCycleTimer() {
  unsigned long now = millis();

  if (millState == MILL_RUN && cycle_target > 0 && cycle_total > 0 && cycle_index > 0) {
    if (now - lastCycleTickMs >= 1000) {  // 1 Hz tick
      lastCycleTickMs = now;

      if (cycle_current < cycle_target) {
        cycle_current++;
      }

      if (cycle_current >= cycle_target) {
        // End of this cycle
        cycle_current    = cycle_target;
        time_remaining_s = 0;

        if (cycle_index < cycle_total) {
          // Start next cycle immediately
          cycle_index++;
          cycle_current    = 0;
          time_remaining_s = cycle_target;
          Serial.print("[CYCLE] Starting next cycle ");
          Serial.print(cycle_index);
          Serial.print(" / ");
          Serial.println(cycle_total);
        } else {
          // All cycles complete
          millState = MILL_IDLE;
          Serial.println("[CYCLE] All cycles complete → IDLE");
        }
      } else {
        time_remaining_s = cycle_target - cycle_current;
      }
    }
  } else {
    // Not running: keep tick anchor fresh so we don't "jump" later
    lastCycleTickMs = now;
  }
}


// -------------------------------------------------------------------
// Relay control (CH1 follows RUN state)
// -------------------------------------------------------------------

void updateRelayFromState() {
  // For now: CH1 = mill motor enable
  bool want = (millState == MILL_RUN);

  if (want != millRelayState) {
    bool ok = Relay_CHx(1, want);  // CH1

    if (ok) {
      millRelayState = want;
      Serial.print("[RELAY] CH1 → ");
      Serial.println(want ? "ON" : "OFF");
    } else {
      Serial.println("[RELAY] Failed to set CH1");
    }
  }
}

// -------------------------------------------------------------------
// Command handling
// -------------------------------------------------------------------

void handleCommand(const String &cmd) {
  // Always evaluate commands against *fresh* interlock state
  checkInterlocks();

    if (cmd == "START") {
    checkInterlocks();

    // Require both a time per cycle and a cycle count
    if (cycle_target == 0 || cycle_total == 0) {
      Serial.println("[CMD] START ignored → no cycle config (cycle_target or total_cycles is 0)");
      return;
    }

    if (estop_ok && lid_locked && door_closed && millState != MILL_FAULT) {
      // Start first cycle
      cycle_index      = 1;
      cycle_current    = 0;
      time_remaining_s = cycle_target;

      millState = MILL_RUN;
      Serial.print("[CMD] START → RUN (cycle ");
      Serial.print(cycle_index);
      Serial.print(" / ");
      Serial.print(cycle_total);
      Serial.println(")");
    } else {
      millState = MILL_FAULT;
      Serial.println("[CMD] START blocked → FAULT (interlocks not OK)");
    }
  }

    else if (cmd == "STOP") {
    millState        = MILL_IDLE;
    cycle_current    = 0;
    time_remaining_s = cycle_target;
    cycle_index      = 0;
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
      millState        = MILL_IDLE;
      cycle_current    = 0;
      time_remaining_s = cycle_target;
      cycle_index      = 0;
      Serial.println("[CMD] RESET_FAULT → IDLE");
    } else {
      Serial.println("[CMD] RESET_FAULT ignored (not in FAULT or interlocks bad)");
    }
  }
}

// Very simple config handler: looks for "cycle_target_s": <integer>
void handleConfig(const String &body) {
  // --- cycle_target_s ------------------------------------------------
  int keyPos = body.indexOf("cycle_target_s");
  if (keyPos >= 0) {
    int colonPos = body.indexOf(':', keyPos);
    if (colonPos >= 0) {
      int idx = colonPos + 1;
      while (idx < (int)body.length() && (body[idx] == ' ' || body[idx] == '\"')) {
        idx++;
      }
      int start = idx;
      while (idx < (int)body.length() && isDigit(body[idx])) {
        idx++;
      }
      if (idx > start) {
        uint32_t val = body.substring(start, idx).toInt();

        if (val == 0) {
          // Explicitly disable timing
          cycle_target     = 0;
          time_remaining_s = 0;
          Serial.println("[CFG] cycle_target_s DISABLED (0)");
        } else if (val <= 24UL * 3600UL) {
          cycle_target     = val;
          time_remaining_s = val;
          Serial.print("[CFG] cycle_target_s set to ");
          Serial.print(val);
          Serial.println(" s");
        } else {
          Serial.print("[CFG] cycle_target_s out of range: ");
          Serial.println(val);
        }
      }
    }
  }


  // --- total_cycles --------------------------------------------------
  keyPos = body.indexOf("total_cycles");
  if (keyPos >= 0) {
    int colonPos = body.indexOf(':', keyPos);
    if (colonPos >= 0) {
      int idx = colonPos + 1;
      while (idx < (int)body.length() && (body[idx] == ' ' || body[idx] == '\"')) {
        idx++;
      }
      int start = idx;
      while (idx < (int)body.length() && isDigit(body[idx])) {
        idx++;
      }
      if (idx > start) {
        uint32_t val = body.substring(start, idx).toInt();

        if (val == 0) {
          // Explicitly disable multi-cycle
          cycle_total = 0;
          cycle_index = 0;
          Serial.println("[CFG] total_cycles DISABLED (0)");
        } else if (val <= 9999UL) {
          cycle_total = val;
          Serial.print("[CFG] total_cycles set to ");
          Serial.println(val);
        } else {
          Serial.print("[CFG] total_cycles out of range: ");
          Serial.println(val);
        }
      }
    }
  }
}

// -------------------------------------------------------------------
// MQTT callback
// -------------------------------------------------------------------

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

  // Determine what kind of command this is
  String cmd;
  if      (body.indexOf("SET_CONFIG")   >= 0) cmd = "SET_CONFIG";
  else if (body.indexOf("RESET_FAULT") >= 0) cmd = "RESET_FAULT";
  else if (body.indexOf("START")       >= 0) cmd = "START";
  else if (body.indexOf("STOP")        >= 0) cmd = "STOP";
  else if (body.indexOf("HOLD")        >= 0) cmd = "HOLD";

  if (cmd == "SET_CONFIG") {
    handleConfig(body);
  }
  else if (cmd.length() > 0) {
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

  // New: multi-cycle info
  json += "\"cycle_index\":";
  json += String(cycle_index);
  json += ",";
  json += "\"cycle_total\":";
  json += String(cycle_total);
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
  Serial.println("Nu-Cryo minimal_mqtt_bridge v0.6 (Ethernet + timer + relay + config)");

  // RGB/Buzzer
  GPIO_Init();

  // I2C + relay expander
  I2C_Init();
  Relay_Init();

  // We do NOT want DIN to auto-drive relays
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

  lastCycleTickMs = millis();

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

  // Cycle timer
  updateCycleTimer();

  // Interlock monitoring → FAULT on open
  checkInterlocks();
  bool currentOk = allInterlocksOk();

  if (!currentOk && lastInterlocksOk && millState != MILL_FAULT) {
    millState = MILL_FAULT;
    Serial.println("[SAFETY] Interlock opened → FAULT");
  }

  lastInterlocksOk = currentOk;

  // Drive relay CH1 based on millState
  updateRelayFromState();

  // Let FreeRTOS tasks (DIN, RGB, Buzzer, ETH) breathe
  delay(10);
}
