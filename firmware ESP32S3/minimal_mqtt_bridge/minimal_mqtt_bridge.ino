/*
 * minimal_mqtt_bridge.ino
 *
 * Nu-Cryo Shaker – ESP32-S3-POE-ETH-8DI-8RO bridge
 *
 * Version history
 *  v0.1  – First minimal MQTT bridge over WiFi (prototype).
 *  v0.2  – Added basic JSON status + Node-RED integration.
 *  v0.3  – Mapped DIN interlocks and status topic mill/status/state.
 *  v0.4  – Switched to Waveshare DIN / GPIO APIs, cleaned JSON structure.
 *  v0.5  – Use Waveshare WS_ETH (W5500 Ethernet) instead of WiFi,
 *          static IP support, disable auto DIN→relay mapping,
 *          door interlock temporarily mirrored to lid for bring-up.
 *  v0.6  – Add simple cycle timer, CH1 relay control, and SET_CONFIG
 *          command to set cycle_target_s from Node-RED (ESP is source of truth).
 *  v0.7  – Multi-cycle support and START lockout when no cycle config.
 *  v0.8  – Centralize relay mapping, add CH2 fault relay following MILL_FAULT,
 *          clean up Ethernet init and keep ESP as timing source of truth.
 *  v0.9  – Map CH3 LN2 valve + CH4 cabinet fan from millState,
 *          add fault_code + fault_reason to status JSON for Node-RED.
 *  v0.10 – Add LN2 PID snapshot stub (pid_ln2) and richer status JSON
 *          for future RS-485 / Modbus integration, keep legacy pid.pv_c.
 *  v0.11 – Housekeeping: JSON schema doc, default status debug off,
 *          keep MQTT buffer override and LN2 stub stable for Node-RED.
 *  v0.12 – Switch LN2 polling to real LC108 Modbus RTU over Serial1 RS-485,
 *          no WS_RS485 task, pid_ln2.comm_ok reflects actual comm status.
 *
 * Status JSON schema (mill/status/state):
 *  {
 *    "state": "IDLE" | "RUN" | "HOLD" | "FAULT",
 *    "cycle_current":    <uint>,      // seconds elapsed in current cycle
 *    "cycle_target":     <uint>,      // seconds per cycle
 *    "time_remaining_s": <uint>,      // seconds remaining in current cycle
 *    "cycle_total":      <uint>,      // requested total cycles in recipe
 *    "cycle_index":      <uint>,      // 0 when idle, 1..cycle_total when running/completed
 *    "fault_code":       <uint>,      // 0 = none; 1=ESTOP, 2=LID, 3=DOOR, 10=INTERLOCK
 *    "fault_reason":     "<string>",  // e.g. "LID_OPEN"
 *    "pid": {
 *      "pv_c": <float>               // legacy LN2 PV for existing UI (°C)
 *    },
 *    "pid_ln2": {
 *      "pv_c":   <float>,            // LN2 PV (°C), from LC108 Modbus
 *      "sv_c":   <float>,            // LN2 setpoint (°C), from LC108 Modbus
 *      "comm_ok": true|false         // RS-485 comm health for last poll
 *    },
 *    "interlocks": {
 *      "door_closed": true|false,
 *      "estop_ok":    true|false,
 *      "lid_locked":  true|false
 *    }
 *  }
 */

#include <Arduino.h>
#include <PubSubClient.h>

#include "WS_GPIO.h"
#include "WS_DIN.h"
#include "WS_Relay.h"
#include "I2C_Driver.h"
#include "WS_ETH.h"

// -------------------------------------------------------------------
// RS-485 / Serial1 for LC108 controllers
// -------------------------------------------------------------------

// Use the RXD1/TXD1 macros from WS_GPIO.h (ESP32-S3 pins for Serial1)
static const int RS485_RX_PIN   = RXD1;
static const int RS485_TX_PIN   = TXD1;
// If your RS-485 transceiver has DE/RE, you can add a pin here later.
// For now we assume auto-direction control on the transceiver.
HardwareSerial &rs485 = Serial1;

// -------------------------------------------------------------------
// Externals from Waveshare libs
// -------------------------------------------------------------------

// From WS_DIN.cpp (not declared in .h)
extern bool DIN_Read_CH1(void);
extern bool DIN_Read_CH2(void);
extern bool DIN_Read_CH3(void);

// Auto DIN→relay mapping flag we want OFF
extern bool Relay_Immediate_Enable;

// -------------------------------------------------------------------
// Network / MQTT configuration
// -------------------------------------------------------------------

static const IPAddress ETH_LOCAL_IP(192, 168, 50, 10);
static const IPAddress ETH_GATEWAY(192, 168, 50, 1);
static const IPAddress ETH_SUBNET(255, 255, 255, 0);
static const IPAddress ETH_DNS(192, 168, 50, 2);  // Pi as DNS (not critical)

static const char *MQTT_HOST          = "192.168.50.2";
static const uint16_t MQTT_PORT       = 1883;
static const char *MQTT_CLIENT_ID     = "nu-cryo-esp32-s3";
static const char *MQTT_STATUS_TOPIC  = "mill/status/state";
static const char *MQTT_CMD_SUB_TOPIC = "mill/cmd/control";

// Generic network client from ESP32 Ethernet stack (via ETH.h)
NetworkClient netClient;
PubSubClient  mqttClient(netClient);

// Debug option: echo JSON STATUS to Serial (length + JSON payload)
// Set to true while debugging, false for normal operation.
static const bool STATUS_SERIAL_DEBUG = true;

// -------------------------------------------------------------------
// Mill state machine
// -------------------------------------------------------------------

enum MillState {
  MILL_IDLE = 0,
  MILL_RUN,
  MILL_HOLD,
  MILL_FAULT
};

MillState millState            = MILL_IDLE;
MillState lastStateBeforeFault = MILL_IDLE;  // used for soft-fault logic

// -------------------------------------------------------------------
// Interlocks & process variables
// -------------------------------------------------------------------

// Hardware mapping (DIN pins):
//   CH1 → E-Stop OK
//   CH2 → Lid locked
//   CH3 → Door closed (physically wired later; may be mirrored to CH2)
bool estop_ok    = false;
bool lid_locked  = false;
bool door_closed = false;

// Mirror door to lid while DI3 switch not wired, for bring-up.
// Set to false when real door switch is installed.
bool mirror_door_to_lid = true;

// -------------------------------------------------------------------
// PID snapshots (LN2 – via LC108 Modbus)
// -------------------------------------------------------------------

struct PidSnapshot {
  bool  comm_ok;      // true if last poll was successful
  float pv_c;         // process variable (°C)
  float sv_c;         // setpoint (°C)
  float output_pct;   // controller output (%), if/when we wire it
};

PidSnapshot pid_ln2 = { false, 0.0f, 0.0f, 0.0f };

// Legacy scalar LN2 PV (kept for backwards compatibility with Node-RED)
float ln2_pv_c = 0.0f;

// -------------------------------------------------------------------
// LC108 (LN2 Controller) Modbus configuration (LN2 channel)
// -------------------------------------------------------------------

// LN2 controller is Modbus ID 3 on the shared RS-485 bus
static const uint8_t  LC108_LN2_ADDR    = 3;

// LC108 manual uses 1-based register numbering; Modbus FC03 uses 0-based
// addresses. If the PV is register 1, SV is register 6 in the manual,
// their FC03 addresses are 0 and 5 respectively.
static const uint16_t LC108_REG_PV_ADDR = 0;   // PV  (°C × 10), register 1 → address 0
static const uint16_t LC108_REG_SV_ADDR = 5;   // SV  (°C × 10), register 6 → address 5

static const uint32_t LC108_TIMEOUT_MS  = 50;  // per-request timeout

// -------------------------------------------------------------------
// Cycle timing
// -------------------------------------------------------------------

// cycle_target: seconds per cycle
// cycle_total:  number of cycles requested
// cycle_index:  0 when idle, 1..cycle_total when running/completed
uint32_t cycle_current    = 0;
uint32_t cycle_target     = 0;
uint32_t time_remaining_s = 0;

uint32_t cycle_total      = 0;
uint32_t cycle_index      = 0;

unsigned long lastCycleTickMs = 0;

// -------------------------------------------------------------------
// Relay control (logical mapping)
// -------------------------------------------------------------------

// Physical channels 1..8 on the Waveshare relay board
#define RELAY_MOTOR_ENABLE_CH     1   // shaker motor contactor
#define RELAY_FAULT_INDICATOR_CH  2   // fault lamp / buzzer
#define RELAY_LN2_VALVE_CH        3   // LN2 solenoid (simple state-based for now)
#define RELAY_CABINET_FAN_CH      4   // enclosure / cabinet fan

bool motorRelayState = false;   // our view of CH1

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

// -------------------------------------------------------------------
// Interlock tracking & fault info
// -------------------------------------------------------------------

bool allInterlocksOk() {
  return estop_ok && lid_locked && door_closed;
}

bool lastInterlocksOk = false;

// Fault metadata for Node-RED
uint8_t fault_code   = 0;       // 0 = none; 1=ESTOP, 2=LID, 3=DOOR, 10=INTERLOCK
String  fault_reason = "";      // e.g. "ESTOP_OPEN", "LID_OPEN", etc.

// -------------------------------------------------------------------
// MQTT timing
// -------------------------------------------------------------------

unsigned long lastStatusPublishMs      = 0;
const unsigned long STATUS_PUBLISH_MS  = 1000;   // 1 Hz to Node-RED

unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_MS  = 2000;  // 2 s

bool lastMqttConnected = false;

// -------------------------------------------------------------------
// PID polling timing (LN2 via Modbus)
// -------------------------------------------------------------------

unsigned long lastPidPollMs = 0;
const unsigned long PID_POLL_MS = 1000;  // 1 s poll for LN2 PID

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
void updateLn2RelayFromState();
void updateFanRelayFromState();
void pollPidLn2();

// Modbus helpers
uint16_t modbus_crc16(const uint8_t *data, uint16_t len);
bool lc108_read_holding(uint8_t slave,
                        uint16_t reg,
                        uint16_t count,
                        uint16_t *out);
bool lc108_read_u16(uint8_t addr, uint16_t reg, uint16_t *out);

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
  estop_ok    = (din_estop == LOW);
  lid_locked  = (din_lid   == LOW);
  door_closed = (din_door  == LOW);
}

// -------------------------------------------------------------------
// publishStatus Debugging wrapper
// -------------------------------------------------------------------

bool publishStatusWithDebug(const char *topic, const char *payload) {
  bool ok = mqttClient.publish(topic, payload);
  if (!ok) {
    Serial.print("[MQTT] publishStatus() FAILED for topic ");
    Serial.println(topic);
  }
  return ok;
}

// -------------------------------------------------------------------
// Cycle timer
// -------------------------------------------------------------------

void updateCycleTimer() {
  unsigned long now = millis();

  if (millState == MILL_RUN && cycle_target > 0 && cycle_total > 0 && cycle_index > 0) {
    if (lastCycleTickMs == 0) {
      lastCycleTickMs = now;
      return;
    }

    unsigned long dt = now - lastCycleTickMs;
    if (dt >= 1000) {
      uint32_t inc = dt / 1000;
      lastCycleTickMs += inc * 1000;

      cycle_current += inc;
      if (cycle_current >= cycle_target) {
        // End of this cycle
        cycle_current    = cycle_target;
        time_remaining_s = 0;

        if (cycle_index < cycle_total) {
          // Start next cycle
          cycle_index++;
          cycle_current    = 0;
          time_remaining_s = cycle_target;
          Serial.print("[CYCLE] Starting next cycle ");
          Serial.print(cycle_index);
          Serial.print(" / ");
          Serial.println(cycle_total);
        } else {
          // All cycles complete → go to IDLE
          millState        = MILL_IDLE;
          cycle_index      = 0;
          cycle_current    = 0;
          time_remaining_s = 0;
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
  bool wantMotor = (millState == MILL_RUN);

  if (wantMotor != motorRelayState) {
    if (setRelayChannel(RELAY_MOTOR_ENABLE_CH, wantMotor)) {
      motorRelayState = wantMotor;
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
// LN2 valve relay (CH3) – simple state-based
// For now: ON in RUN or HOLD; OFF otherwise.
// Later: may add PV-based control or hysteresis here.
// -------------------------------------------------------------------

void updateLn2RelayFromState() {
  static bool lastLn2 = false;

  bool wantLn2 = (millState == MILL_RUN || millState == MILL_HOLD);

  if (wantLn2 != lastLn2) {
    if (setRelayChannel(RELAY_LN2_VALVE_CH, wantLn2)) {
      lastLn2 = wantLn2;
      Serial.print("[RELAY] LN2 CH");
      Serial.print(RELAY_LN2_VALVE_CH);
      Serial.println(wantLn2 ? " → ON" : " → OFF");
    } else {
      Serial.println("[RELAY] Failed to set LN2 relay");
    }
  }
}

// -------------------------------------------------------------------
// Cabinet fan relay (CH4)
// For now: ON in RUN, HOLD, or FAULT; OFF in IDLE.
// -------------------------------------------------------------------

void updateFanRelayFromState() {
  static bool lastFan = false;

  bool wantFan = (millState == MILL_RUN ||
                  millState == MILL_HOLD ||
                  millState == MILL_FAULT);

  if (wantFan != lastFan) {
    if (setRelayChannel(RELAY_CABINET_FAN_CH, wantFan)) {
      lastFan = wantFan;
      Serial.print("[RELAY] FAN CH");
      Serial.print(RELAY_CABINET_FAN_CH);
      Serial.println(wantFan ? " → ON" : " → OFF");
    } else {
      Serial.println("[RELAY] Failed to set fan relay");
    }
  }
}

// -------------------------------------------------------------------
// LC108: Read holding registers via RS-485 (function 0x03)
//
// slave   = Modbus device address (1..247)
// reg     = start register *address* (0-based)
// count   = number of 16-bit registers
// out[]   = caller-provided array of length >= count
//
// Returns true on success, false on any error (timeout/CRC/etc).
// -------------------------------------------------------------------
bool lc108_read_holding(uint8_t slave,
                        uint16_t reg,
                        uint16_t count,
                        uint16_t *out) {
  if (count == 0 || count > 16) {
    return false;  // sanity limit
  }

  // Build request: [slave][0x03][reg hi][reg lo][cnt hi][cnt lo][CRClo][CRChi]
  uint8_t req[8];
  req[0] = slave;
  req[1] = 0x03;               // Read Holding Registers
  req[2] = (reg >> 8) & 0xFF;
  req[3] = (reg     ) & 0xFF;
  req[4] = (count >> 8) & 0xFF;
  req[5] = (count     ) & 0xFF;

  uint16_t crc = modbus_crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  // Clear any stale bytes in RX buffer
  while (rs485.available()) {
    (void)rs485.read();
  }

  // Send request
  rs485.write(req, sizeof(req));
  rs485.flush();

  // Expected response:
  // [slave][0x03][byteCount][data ...][CRClo][CRChi]
  // byteCount = 2 * count
  const uint8_t expectedByteCount = 2 * count;
  const uint8_t expectedLen       = 5 + expectedByteCount;
  if (expectedLen > 64) {
    return false;  // safety
  }

  uint8_t resp[64];
  size_t  got = rs485.readBytes(resp, expectedLen);

  if (got != expectedLen) {
    // Timeout / short frame
    Serial.print("[LC108] read_holding: short read got=");
    Serial.print(got);
    Serial.print(" expected=");
    Serial.println(expectedLen);
    return false;
  }

  // Check header
  if (resp[0] != slave || resp[1] != 0x03 || resp[2] != expectedByteCount) {
    Serial.print("[LC108] read_holding: bad header id=");
    Serial.print(resp[0]);
    Serial.print(" func=");
    Serial.print(resp[1]);
    Serial.print(" bc=");
    Serial.println(resp[2]);
    return false;
  }

  // Check CRC
  uint16_t crcRx   = resp[expectedLen - 2] | (uint16_t(resp[expectedLen - 1]) << 8);
  uint16_t crcCalc = modbus_crc16(resp, expectedLen - 2);
  if (crcRx != crcCalc) {
    Serial.print("[LC108] read_holding: CRC mismatch resp=0x");
    Serial.print(crcRx, HEX);
    Serial.print(" calc=0x");
    Serial.println(crcCalc, HEX);
    return false;
  }

  // Extract registers
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t hi = resp[3 + 2 * i];
    uint8_t lo = resp[4 + 2 * i];
    out[i] = (uint16_t(hi) << 8) | lo;
  }

  return true;
}

// -------------------------------------------------------------------
// Modbus CRC-16 (standard polynomial 0xA001, initial 0xFFFF)
// -------------------------------------------------------------------
uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; ++pos) {
    crc ^= data[pos];
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// -------------------------------------------------------------------
// Low-level: read one holding register (function 0x03) as uint16_t
//  addr  = slave address
//  reg   = starting register *address* (0-based)
//  out   = filled with raw 16-bit value on success
// Returns true on success, false on timeout/CRC/protocol error.
// -------------------------------------------------------------------
bool lc108_read_u16(uint8_t addr, uint16_t reg, uint16_t *out) {
  if (!out) return false;

  uint16_t tmp = 0;
  if (!lc108_read_holding(addr, reg, 1, &tmp)) {
    return false;
  }

  *out = tmp;
  return true;
}

// -------------------------------------------------------------------
// LN2 PID polling (real LC108 over RS-485 / Modbus RTU)
//
// Reads PV and SV from LC108 and updates pid_ln2 + ln2_pv_c.
// On any error, comm_ok is set false and previous PV/SV are kept.
// -------------------------------------------------------------------
void pollPidLn2() {
  uint16_t pvRaw = 0;
  uint16_t svRaw = 0;

  bool okPv = lc108_read_u16(LC108_LN2_ADDR, LC108_REG_PV_ADDR, &pvRaw);
  bool okSv = lc108_read_u16(LC108_LN2_ADDR, LC108_REG_SV_ADDR, &svRaw);

  if (!(okPv && okSv)) {
    pid_ln2.comm_ok = false;
    // Keep last known pv_c / sv_c, but flag comm fault
    Serial.println("[LC108] pollPidLn2: comm error (timeout, bad header, or CRC)");
    return;
  }

  pid_ln2.comm_ok = true;

  // These are signed 16-bit values representing °C × 10
  int16_t pvInt = (int16_t)pvRaw;
  int16_t svInt = (int16_t)svRaw;

  pid_ln2.pv_c = pvInt / 10.0f;
  pid_ln2.sv_c = svInt / 10.0f;

  // Keep legacy scalar in sync for any old wiring
  ln2_pv_c = pid_ln2.pv_c;

  Serial.print("[LC108] PV=");
  Serial.print(pid_ln2.pv_c);
  Serial.print("°C  SV=");
  Serial.print(pid_ln2.sv_c);
  Serial.println("°C  (ID=3)");
}

// -------------------------------------------------------------------
// Status JSON publish
// -------------------------------------------------------------------

void publishStatus() {
  String json;
  json.reserve(320);

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

  // fault_code
  json += "\"fault_code\":";
  json += String(fault_code);
  json += ",";

  // fault_reason (string)
  json += "\"fault_reason\":\"";
  json += fault_reason;
  json += "\",";

  // legacy pid block (for existing UI) – uses ln2_pv_c
  json += "\"pid\":{";
  json += "\"pv_c\":";
  json += String(ln2_pv_c, 1);
  json += "},";

  // richer LN2 PID snapshot (new)
  json += "\"pid_ln2\":{";
  json += "\"pv_c\":";
  json += String(pid_ln2.pv_c, 1);
  json += ",\"sv_c\":";
  json += String(pid_ln2.sv_c, 1);
  json += ",\"comm_ok\":";
  json += pid_ln2.comm_ok ? "true" : "false";
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

  if (STATUS_SERIAL_DEBUG) {
    Serial.print("[STATUS] len=");
    Serial.println(json.length());
    Serial.print("[STATUS] ");
    Serial.println(json);
  }

  // Use debug wrapper so we can see if MQTT actually sends
  publishStatusWithDebug(MQTT_STATUS_TOPIC, json.c_str());
}

// -------------------------------------------------------------------
// Command handling
// -------------------------------------------------------------------

void handleCommand(const String &cmd) {
  // Always evaluate commands against *fresh* interlock state
  checkInterlocks();
  bool currentOk = allInterlocksOk();

  // ---------------------------------------------------------------
  // RESET_FAULT
  // ---------------------------------------------------------------
  if (cmd == "RESET_FAULT") {
    if (millState == MILL_FAULT && currentOk) {

      // "Soft" access fault:
      //  - LID_OPEN (code 2) or DOOR_OPEN (code 3)
      //  - occurred while we were in HOLD
      //  - and we actually had a recipe defined
      bool softAccessHoldFault =
        ((fault_code == 2 /* LID_OPEN */ ||
          fault_code == 3 /* DOOR_OPEN */) &&
         lastStateBeforeFault == MILL_HOLD &&
         cycle_total > 0);

      if (softAccessHoldFault) {
        // Restore HOLD and keep timing + cycle position
        millState = MILL_HOLD;
        Serial.println("[CMD] RESET_FAULT → HOLD (access fault cleared, timing preserved)");
      } else {
        // All other faults: fall back to a "hard" reset to IDLE
        millState        = MILL_IDLE;
        cycle_current    = 0;
        time_remaining_s = 0;
        cycle_index      = 0;  // reset multi-cycle index, but keep recipe config
        Serial.println("[CMD] RESET_FAULT → IDLE, fault cleared");
      }

      // Clear fault metadata either way
      fault_code   = 0;
      fault_reason = "";
      lastStateBeforeFault = millState;
    } else {
      Serial.println("[CMD] RESET_FAULT ignored (not in FAULT or interlocks bad)");
    }
    return;
  }

  // ---------------------------------------------------------------
  // START (fresh start) or RESUME from HOLD
  // ---------------------------------------------------------------
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

    // Decide if this should be a RESUME (from HOLD) or a fresh START
    bool resumeFromHold =
      (millState == MILL_HOLD &&
       cycle_total > 0 &&
       cycle_target > 0 &&
       cycle_index > 0 &&
       cycle_current < cycle_target);

    if (resumeFromHold) {
      // RESUME: keep cycle_current & time_remaining_s as frozen in HOLD
      millState       = MILL_RUN;
      lastCycleTickMs = millis();  // restart timing from "now"
      Serial.print("[CMD] RESUME → RUN at t=");
      Serial.print(cycle_current);
      Serial.print(" / ");
      Serial.print(cycle_target);
      Serial.print(" s, cycle ");
      Serial.print(cycle_index);
      Serial.print(" / ");
      Serial.println(cycle_total);
    } else {
      // Fresh START: reset timing
      millState        = MILL_RUN;
      cycle_current    = 0;
      time_remaining_s = cycle_target;

      if (cycle_index == 0) {
        cycle_index = 1;
      }
      lastCycleTickMs = millis();

      Serial.print("[CMD] START → RUN: cycle_target_s=");
      Serial.print(cycle_target);
      Serial.print(" total_cycles=");
      Serial.println(cycle_total);
    }

    lastStateBeforeFault = millState;
    return;
  }

  // ---------------------------------------------------------------
  // HOLD
  // ---------------------------------------------------------------
  if (cmd == "HOLD") {
    if (millState == MILL_RUN) {
      millState = MILL_HOLD;
      Serial.println("[CMD] HOLD → HOLD");
      lastStateBeforeFault = millState;
    } else {
      Serial.println("[CMD] HOLD ignored (not in RUN)");
    }
    return;
  }

  // ---------------------------------------------------------------
  // STOP
  // ---------------------------------------------------------------
  if (cmd == "STOP") {
    if (millState == MILL_RUN || millState == MILL_HOLD) {
      millState        = MILL_IDLE;
      cycle_current    = 0;
      time_remaining_s = 0;
      cycle_index      = 0;  // reset multi-cycle on STOP
      Serial.println("[CMD] STOP → IDLE");
      lastStateBeforeFault = millState;
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

  if (t == MQTT_CMD_SUB_TOPIC) {
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
// Arduino setup()
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("Nu-Cryo minimal_mqtt_bridge v0.12 (Ethernet + cycles + relays + LN2 Modbus PV/SV)");

  // RGB/Buzzer and local GPIO
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

  // Bring up RS-485 serial (Serial1) for LC108 Modbus
  rs485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  rs485.setTimeout(LC108_TIMEOUT_MS);

  // MQTT client setup
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  // Larger MQTT packet size for richer JSON payloads
  mqttClient.setBufferSize(512);   // bump to 768/1024 later if JSON grows

  // Initial interlock read
  checkInterlocks();
  lastInterlocksOk = allInterlocksOk();

  // Ensure relays are in a known state
  motorRelayState = false;
  setRelayChannel(RELAY_MOTOR_ENABLE_CH, false);
  setRelayChannel(RELAY_FAULT_INDICATOR_CH, false);
  setRelayChannel(RELAY_LN2_VALVE_CH, false);
  setRelayChannel(RELAY_CABINET_FAN_CH, false);

  // Initialize cycle config as "not configured"
  cycle_target      = 0;
  cycle_total       = 0;
  cycle_index       = 0;
  cycle_current     = 0;
  time_remaining_s  = 0;

  fault_code        = 0;
  fault_reason      = "";
  lastStateBeforeFault = millState;

  lastCycleTickMs   = millis();
  lastPidPollMs     = millis();
}

// -------------------------------------------------------------------
// Arduino loop()
// -------------------------------------------------------------------

void loop() {
  unsigned long now = millis();

  // Track connection edges for debugging
  bool nowConnected = mqttClient.connected();
  if (nowConnected != lastMqttConnected) {
    if (nowConnected) {
      Serial.println("[MQTT] Connection state: CONNECTED");
    } else {
      Serial.println("[MQTT] Connection state: DISCONNECTED");
    }
    lastMqttConnected = nowConnected;
  }

  // --------------------------------------------------------------------
  // 1) Maintain MQTT connection
  // --------------------------------------------------------------------
  if (!mqttClient.connected()) {
    if (now - lastMqttReconnectAttempt >= MQTT_RECONNECT_MS) {
      lastMqttReconnectAttempt = now;
      mqttReconnect();
    }
  } else {
    mqttClient.loop();
  }

  // --------------------------------------------------------------------
  // 2) Cycle timer (advance RUN timing before we potentially enter FAULT)
  // --------------------------------------------------------------------
  updateCycleTimer();

  // --------------------------------------------------------------------
  // 3) Interlock monitoring → FAULT on open
  // --------------------------------------------------------------------
  checkInterlocks();
  bool currentOk = allInterlocksOk();

  if (!currentOk) {
    MillState prevState = millState;

    // Decide which input caused the fault
    if (!estop_ok) {
      fault_code   = 1;
      fault_reason = "ESTOP_OPEN";
    } else if (!lid_locked) {
      fault_code   = 2;
      fault_reason = "LID_OPEN";
    } else if (!door_closed) {
      fault_code   = 3;
      fault_reason = "DOOR_OPEN";
    } else {
      fault_code   = 10;
      fault_reason = "INTERLOCK_OPEN";
    }

    // Only log + force publish on transition into FAULT
    if (millState != MILL_FAULT) {
      // If we were RUN or HOLD, keep whatever timing snapshot we had,
      // but make sure cycle_index is at least 1 so UI can show the
      // cycle on which the fault occurred.
      if ((prevState == MILL_RUN || prevState == MILL_HOLD) &&
          cycle_total > 0 && cycle_index == 0) {
        cycle_index = 1;
      }

      millState            = MILL_FAULT;
      lastStateBeforeFault = prevState;

      Serial.print("[SAFETY] Interlock opened → FAULT (");
      Serial.print(fault_reason);
      Serial.println(")");

      // Immediately push a FAULT status frame
      publishStatus();
    }
  }

  // we only clear the fault via RESET_FAULT when interlocks are OK
  lastInterlocksOk = currentOk;

  // --------------------------------------------------------------------
  // 4) LN2 PID polling (real LC108 Modbus, once per PID_POLL_MS)
  // --------------------------------------------------------------------
  if (now - lastPidPollMs >= PID_POLL_MS) {
    lastPidPollMs = now;
    pollPidLn2();
  }

  // --------------------------------------------------------------------
  // 5) Drive relays based on millState
  // --------------------------------------------------------------------
  updateRelayFromState();       // CH1 motor
  updateFaultRelayFromState();  // CH2 fault indicator
  updateLn2RelayFromState();    // CH3 LN2 valve
  updateFanRelayFromState();    // CH4 cabinet fan

  // --------------------------------------------------------------------
  // 6) Periodic status publish (runs in ALL states, including FAULT)
  // --------------------------------------------------------------------
  if (mqttClient.connected() &&
      (now - lastStatusPublishMs >= STATUS_PUBLISH_MS)) {
    lastStatusPublishMs = now;
    publishStatus();
  }

  // --------------------------------------------------------------------
  // 7) Let FreeRTOS tasks (DIN, RGB, Buzzer, ETH) breathe
  // --------------------------------------------------------------------
  delay(10);
}
