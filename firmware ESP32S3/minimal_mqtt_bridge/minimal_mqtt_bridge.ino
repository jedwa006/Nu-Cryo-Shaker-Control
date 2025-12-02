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
 *  v0.8 - map CH2 fault relay to MILL_FAULT state and centralize relay mapping helpers.
 */

#include <Arduino.h>
#include <PubSubClient.h>

#include "WS_GPIO.h"
#include "WS_DIN.h"
#include "WS_ETH.h"

// -------------------------------------------------------------------
// MQTT configuration
// -------------------------------------------------------------------

// Broker IP (your Pi's static IP on the mill network)
static const char *MQTT_BROKER_IP   = "192.168.50.2";
static const uint16_t MQTT_BROKER_PORT = 1883;

// MQTT topic names
static const char *TOPIC_STATUS  = "mill/status/state";   // JSON state → HMI
static const char *TOPIC_CONTROL = "mill/cmd/control";    // HMI → commands/SET_CONFIG

// Reconnect intervals
static const unsigned long MQTT_RECONNECT_MS = 3000;      // 3 seconds
static const unsigned long STATUS_PUBLISH_MS  = 500;      // 0.5 seconds for now

// -------------------------------------------------------------------
// Mill state machine
// -------------------------------------------------------------------

enum MillState {
  MILL_IDLE = 0,
  MILL_RUN,
  MILL_HOLD,
  MILL_FAULT
};

MillState millState = MILL_IDLE;

// Interlock status (logical)
bool estop_ok    = false;
bool lid_locked  = false;
bool door_closed = false;

// For detecting transitions (e.g. interlock edge → FAULT)
bool lastInterlocksOk = false;

// Mirror door to lid while DI3 switch not wired, for bring-up.
// Set to false when real door switch is installed.
bool mirror_door_to_lid = false;

// LN2 PV placeholder (°C)
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

// --- Relay control (logical mapping) ----------------------------------

// Physical channels 1..8 on the Waveshare relay board
#define RELAY_MOTOR_ENABLE_CH     1   // shaker motor contactor
#define RELAY_FAULT_INDICATOR_CH  2   // fault lamp / buzzer (optional)
// #define RELAY_LN2_VALVE_CH     3   // reserved for LN2 valve
// #define RELAY_CABINET_FAN_CH   4   // reserved for cabinet fan

bool millRelayState = false;  // our view of CH1 (motor) state

// Helper: set a relay channel with basic sanity/error logging
bool setRelayChannel(uint8_t ch, bool on) {
  if (ch < 1 || ch > 8) {
    Serial.print("[RELAY] invalid channel: ");
    Serial.println(ch);
    return false;
  }
  bool ok = Relay_CHx(ch, on);
  if (!ok) {
    Serial.print("[RELAY] Relay_CHx failed for CH");
    Serial.println(ch);
  }
  return ok;
}

// --- Interlock tracking ---------------------------------------------

bool allInterlocksOk() {
  return estop_ok && lid_locked && door_closed;
}

// -------------------------------------------------------------------
// Ethernet + MQTT
// -------------------------------------------------------------------

EthernetClient ethClient;
PubSubClient   mqttClient(ethClient);

unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastStatusPublishMs      = 0;

// -------------------------------------------------------------------
// Forward declarations
// -------------------------------------------------------------------

void mqttCallback(char *topic, byte *payload, unsigned int length);
void mqttReconnect();
void publishStatus();
void checkInterlocks();
void handleCommand(const String &cmd);
void handleConfig(const String &body);
void updateCycleTimer();
void updateRelayFromState();
void updateFaultRelayFromState();

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
  //   HIGH (open / floating)         = NOT OK
  estop_ok    = (din_estop == LOW);
  lid_locked  = (din_lid   == LOW);
  door_closed = (din_door  == LOW);
}

// -------------------------------------------------------------------
// JSON status publishing
// -------------------------------------------------------------------

void publishStatus() {
  // Build JSON string manually:
  // {
  //   "state": "IDLE",
  //   "cycle_current": 0,
  //   "cycle_target": 0,
  //   "time_remaining_s": 0,
  //   "cycle_total": 0,
  //   "cycle_index": 0,
  //   "pid": { "pv_c": 0.0 },
  //   "interlocks": {
  //     "door_closed": true,
  //     "estop_ok": true,
  //     "lid_locked": true
  //   }
  // }

  String json;
  json.reserve(256);

  json += "{";
  // state
  json += "\"state\":\"";
  switch (millState) {
    case MILL_IDLE:  json += "IDLE";  break;
    case MILL_RUN:   json += "RUN";   break;
    case MILL_HOLD:  json += "HOLD";  break;
    case MILL_FAULT: json += "FAULT"; break;
  }
  json += "\",";

  // cycle_current
  json += "\"cycle_current\":";
  json += String(cycle_current);
  json += ",";

  // cycle_target
  json += "\"cycle_target\":";
  json += String(cycle_target);
  json += ",";

  // time_remaining_s
  json += "\"time_remaining_s\":";
  json += String(time_remaining_s);
  json += ",";

  // cycle_total
  json += "\"cycle_total\":";
  json += String(cycle_total);
  json += ",";

  // cycle_index
  json += "\"cycle_index\":";
  json += String(cycle_index);
  json += ",";

  // pid
  json += "\"pid\":{";
  json += "\"pv_c\":";
  json += String(ln2_pv_c, 1);
  json += "},";

  // interlocks
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

  // Debug helper — can be toggled off if too chatty
  // Serial.print("[STATUS] ");
  // Serial.println(json);

  mqttClient.publish(TOPIC_STATUS, json.c_str());
}

// -------------------------------------------------------------------
// Cycle timer update (called from loop())
// -------------------------------------------------------------------

void updateCycleTimer() {
  unsigned long now = millis();

  // Only tick in RUN state and if we have a valid cycle_target
  if (millState == MILL_RUN && cycle_target > 0 && cycle_total > 0) {
    if (lastCycleTickMs == 0) {
      lastCycleTickMs = now;
      return;
    }

    unsigned long dt = now - lastCycleTickMs;
    if (dt >= 1000) {
      // Advance by whole seconds according to elapsed ms
      uint32_t inc = dt / 1000;
      lastCycleTickMs += inc * 1000;

      cycle_current += inc;
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
          // All cycles complete → go to IDLE
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
  // Motor enable follows RUN state
  bool wantMotor = (millState == MILL_RUN);

  if (wantMotor != millRelayState) {
    if (setRelayChannel(RELAY_MOTOR_ENABLE_CH, wantMotor)) {
      millRelayState = wantMotor;
      Serial.print("[RELAY] MOTOR CH");
      Serial.print(RELAY_MOTOR_ENABLE_CH);
      Serial.println(wantMotor ? " → ON" : " → OFF");
    } else {
      Serial.println("[RELAY] Failed to set motor relay");
    }
  }
}

// -------------------------------------------------------------------
// Fault relay (CH2) follows FAULT state
// -------------------------------------------------------------------

void updateFaultRelayFromState() {
  static bool lastFaultRelay = false;

  bool wantFault = (millState == MILL_FAULT);

  if (wantFault != lastFaultRelay) {
    if (setRelayChannel(RELAY_FAULT_INDICATOR_CH, wantFault)) {
      lastFaultRelay = wantFault;
      Serial.print("[RELAY] FAULT CH");
      Serial.print(RELAY_FAULT_INDICATOR_CH);
      Serial.println(wantFault ? " → ON" : " → OFF");
    } else {
      Serial.println("[RELAY] Failed to set fault relay");
    }
  }
}

// -------------------------------------------------------------------
// Command handling
// -------------------------------------------------------------------

void handleCommand(const String &cmd) {
  // Always evaluate commands against *fresh* interlock state
  checkInterlocks();
  bool currentOk = allInterlocksOk();

  if (cmd == "RESET_FAULT") {
    if (millState == MILL_FAULT && currentOk) {
      millState       = MILL_IDLE;
      cycle_current   = 0;
      time_remaining_s = 0;
      cycle_index     = 0;  // reset multi-cycle index
      Serial.println("[CMD] RESET_FAULT → IDLE");
    } else {
      Serial.println("[CMD] RESET_FAULT ignored (not in FAULT or interlocks not OK)");
    }
    return;
  }

  if (cmd == "START") {
    Serial.println("[CMD] START received");
    if (!currentOk) {
      Serial.println("[CMD] START ignored → interlock not OK");
      return;
    }

    if (cycle_target == 0 || cycle_total == 0) {
      Serial.println("[CMD] START ignored → no cycle config (cycle_target or total_cycles is 0)");
      return;
    }

    millState        = MILL_RUN;
    cycle_current    = 0;
    time_remaining_s = cycle_target;

    if (cycle_index == 0) {
      cycle_index = 1;
    }
    lastCycleTickMs  = millis();
    Serial.print("[CMD] START → RUN: cycle_target_s=");
    Serial.print(cycle_target);
    Serial.print(" total_cycles=");
    Serial.println(cycle_total);
    return;
  }

  if (cmd == "HOLD") {
    if (millState == MILL_RUN) {
      millState = MILL_HOLD;
      Serial.println("[CMD] HOLD → H
OLD");
    } else {
      Serial.println("[CMD] HOLD ignored (not in RUN)");
    }
    return;
  }

  if (cmd == "STOP") {
    if (millState == MILL_RUN || millState == MILL_HOLD) {
      millState        = MILL_IDLE;
      cycle_current    = 0;
      time_remaining_s = 0;
      cycle_index      = 0;  // reset multi-cycle on STOP
      Serial.println("[CMD] STOP → IDLE");
    } else {
      Serial.println("[CMD] STOP ignored (not in RUN/HOLD)");
    }
    return;
  }

  Serial.print("[CMD] Unknown command: ");
  Serial.println(cmd);
}

// -------------------------------------------------------------------
// SET_CONFIG handler
//
// body is a JSON-like string, e.g.:
// {
//   "cmd": "SET_CONFIG",
//   "cycle_target_s": 300,
//   "total_cycles": 10
// }
//
// We parse the integers manually, ignoring everything else.
//
// cycle_target_s:
//   0       → disable timing (no recipe)
//   1..86400 (24h max) → valid
//
// total_cycles:
//   0       → disable multi-cycle
//   1..9999 → valid
// -------------------------------------------------------------------

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

  Serial.print("[MQTT] RX topic=");
  Serial.print(t);
  Serial.print(" payload=");
  Serial.println(body);

  if (t == TOPIC_CONTROL) {
    // Expect either:
    //  { "cmd": "START" }
    //  { "cmd": "STOP" }
    //  { "cmd": "HOLD" }
    //  { "cmd": "RESET_FAULT" }
    //  { "cmd": "SET_CONFIG", "cycle_target_s":123, "total_cycles": 10 }
    int cmdPos = body.indexOf("\"cmd\"");
    if (cmdPos >= 0) {
      int colonPos = body.indexOf(':', cmdPos);
      if (colonPos >= 0) {
        int quote1 = body.indexOf('\"', colonPos);
        int quote2 = body.indexOf('\"', quote1 + 1);
        if (quote1 >= 0 && quote2 > quote1) {
          String cmd = body.substring(quote1 + 1, quote2);
          if (cmd == "SET_CONFIG") {
            handleConfig(body);
          } else {
            handleCommand(cmd);
          }
        }
      }
    }
  } else {
    Serial.println("[MQTT] Unknown topic; ignoring");
  }
}

// -------------------------------------------------------------------
// MQTT reconnect logic
// -------------------------------------------------------------------

void mqttReconnect() {
  if (mqttClient.connected()) {
    return;
  }

  Serial.print("[MQTT] Connecting to broker ");
  Serial.print(MQTT_BROKER_IP);
  Serial.print(":");
  Serial.println(MQTT_BROKER_PORT);

  String clientId = "NuCryoBridge-";
  clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("[MQTT] Connected!");
    mqttClient.subscribe(TOPIC_CONTROL);
  } else {
    Serial.print("[MQTT] Connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// -------------------------------------------------------------------
// Arduino setup()
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Nu-Cryo minimal_mqtt_bridge v0.8 (Ethernet + Node-RED)");

  // Initialize Waveshare board subsystems
  WS_Init();
  GPIO_Init();
  DIN_Init();
  ETH_Init();

  Serial.println("Ethernet Start");

  // Bring up Ethernet (W5500 via WS_ETH)
  ETH_Start();
  Serial.println("ETH Started");

  IPAddress ip = ETH_LocalIP();
  Serial.print("ETH local IP (after init): ");
  Serial.println(ip);

  mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  mqttClient.setCallback(mqttCallback);

  mqttReconnect();

  // Initial interlock read
  checkInterlocks();
  lastInterlocksOk = allInterlocksOk();

  // Ensure relay is in a known state
  millRelayState = false;
  setRelayChannel(RELAY_MOTOR_ENABLE_CH, false);

  // Initialize cycle config as "not configured"
  cycle_target = 0;
  cycle_total  = 0;
  cycle_index  = 0;
  cycle_current = 0;
  time_remaining_s = 0;
}

// -------------------------------------------------------------------
// Arduino loop()
// -------------------------------------------------------------------

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

  // Drive relays based on millState
  updateRelayFromState();       // CH1 motor
  updateFaultRelayFromState();  // CH2 fault indicator

  // Let FreeRTOS tasks (DIN, RGB, Buzzer, ETH) breathe
  delay(10);
}
