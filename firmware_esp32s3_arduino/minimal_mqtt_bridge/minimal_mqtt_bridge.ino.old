/*
  minimal_mqtt_bridge.ino
  ESP32-S3-ETH-8DI-8RO  ←→  W5500  ←→  Mosquitto on Pi

  v0.3:
    - v0.2 RGB state indication via WS_GPIO (IDLE/RUN/HOLD/FAULT colors)
    - NEW: map DIN1..DIN3 into interlocks (door/estop/lid)
    - NEW: auto-fault if any interlock drops while RUN or HOLD
    - NEW: reject START if interlocks not OK
    - Still publishing mill/status/state once per second
    - Still consuming mill/cmd/control from HMI

  Interlock mapping (for LV harness):
    DIN1 (GPIO 4) → door_closed  (HIGH = door closed / OK)
    DIN2 (GPIO 5) → estop_ok     (HIGH = estop circuit OK)
    DIN3 (GPIO 6) → lid_locked   (HIGH = lid lock confirmed)
*/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <math.h>          // for sinf

// Waveshare board helpers (pins, RGB/buzzer tasks)
#include "WS_GPIO.h"       // from Waveshare examples

// ----------- BOARD / HARDWARE CONFIG (Waveshare ESP32-S3-ETH-8DI-8RO) -----

// W5500 / Ethernet pins (from WS_ETH.h)
#define ETH_CS_PIN    16
#define ETH_IRQ_PIN   12
#define ETH_RST_PIN   39

#define SPI_SCK_PIN   15
#define SPI_MISO_PIN  14
#define SPI_MOSI_PIN  13

// We now use RGB via WS_GPIO helpers (RGB_Light / RGB_Open_Time)
#define TEST_OUTPUT_PIN  GPIO_PIN_RGB   // kept for possible future direct use

// DINs as interlock inputs (active HIGH = OK)
// Map DIN1..DIN3 → door, estop, lid
const int PIN_DIN_DOOR  = 4;   // DIN CH1
const int PIN_DIN_ESTOP = 5;   // DIN CH2
const int PIN_DIN_LID   = 6;   // DIN CH3

// ----------- NETWORK CONFIG ---------------------------------------------

// MAC address for W5500 (arbitrary but unique on your LAN)
byte mac[] = { 0x02, 0x34, 0x56, 0x78, 0x9A, 0xBC };

// Static IPs on private Pi ↔ ESP link
IPAddress localIp(192, 168, 50, 10);   // ESP
IPAddress gateway(192, 168, 50, 1);    // optional; not critical for point-to-point
IPAddress subnet(255, 255, 255, 0);

// MQTT broker (Pi) IP on that same link
IPAddress mqttServer(192, 168, 50, 2);
const uint16_t mqttPort = 1883;

const char* mqttClientId = "esp32s3-mill-bridge";

// ----------- MQTT TOPICS -------------------------------------------------

const char* TOPIC_CMD_CONTROL  = "mill/cmd/control";
const char* TOPIC_STATUS_STATE = "mill/status/state";

// ----------- MILL STATE MACHINE TYPES -----------------------------------

enum MillState {
  STATE_IDLE,
  STATE_RUN,
  STATE_HOLD,
  STATE_FAULT
};

enum MillSubstate {
  SUB_IDLE_READY,
  SUB_RUN_ACTIVE,
  SUB_RUN_COOLING,
  SUB_HOLD_USER,
  SUB_HOLD_INTERLOCK,
  SUB_FAULT_INTERLOCK,
  SUB_FAULT_DEVICE,
  SUB_FAULT_INTERNAL
};

struct MillStatus {
  MillState    state;
  MillSubstate substate;

  uint16_t cycleCurrent;
  uint16_t cycleTarget;

  uint32_t runTime_s;
  uint32_t coolTime_s;

  uint32_t timeInState_s;
  int32_t  timeRemaining_s;

  bool doorClosed;
  bool estopOk;
  bool lidLocked;

  float pidPv_c;
  float pidSv_c;
  bool  pidAlarm;

  uint32_t heartbeat;
};

MillStatus gStatus;

// timing
unsigned long lastStatusPublishMs = 0;
const unsigned long STATUS_INTERVAL_MS = 1000;

// Ethernet & MQTT clients
EthernetClient ethClient;
PubSubClient mqtt(ethClient);

// ----------- FORWARD DECLARATIONS ---------------------------------------

void initMillStatus();
void updateMillStatusFromTick();
void updateInterlocksFromHardware();
void handleControlCommand(const char* payload, size_t len);
void publishStatus();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void ensureMqttConnected();

void showStateColor();   // helper to keep RGB and gStatus in sync

// ========================================================================
// SETUP
// ========================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println(F("minimal_mqtt_bridge v0.3 starting..."));

  // Initialize GPIO / RGB / buzzer tasks from Waveshare library
  GPIO_Init();

  // Brief blue pulse on boot so we know RGB + tasks are alive
  RGB_Open_Time(0, 0, 32, 300, 0);   // (R,G,B, duration_ms, flicker_ms=0) – solid blue for 300ms

  // Interlock inputs
  pinMode(PIN_DIN_DOOR,  INPUT);  // opto output → logic level
  pinMode(PIN_DIN_ESTOP, INPUT);
  pinMode(PIN_DIN_LID,   INPUT);

  // Reset W5500
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);
  delay(50);
  digitalWrite(ETH_RST_PIN, HIGH);
  delay(200);

  // Set up SPI for W5500
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Ethernet.init(ETH_CS_PIN);

  Serial.println(F("Bringing up Ethernet (W5500)..."));
  Ethernet.begin(mac, localIp, gateway, gateway, subnet);
  delay(1000);

  Serial.print(F("Local IP: "));
  Serial.println(Ethernet.localIP());

  mqtt.setServer(mqttServer, mqttPort);
  mqtt.setCallback(mqttCallback);

  // Increase MQTT buffer size to handle our JSON payload
  mqtt.setBufferSize(512);

  initMillStatus();
  // Make sure interlock booleans reflect actual pins before first publish
  updateInterlocksFromHardware();
  showStateColor();   // set initial RGB based on IDLE state
}

// ========================================================================
// LOOP
// ========================================================================

void loop() {
  ensureMqttConnected();
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastStatusPublishMs >= STATUS_INTERVAL_MS) {
    lastStatusPublishMs = now;
    updateMillStatusFromTick();
    publishStatus();
  }
}

// ========================================================================
// MILL STATE MACHINE IMPLEMENTATION (v0 SIM + interlocks)
// ========================================================================

void initMillStatus() {
  gStatus.state        = STATE_IDLE;
  gStatus.substate     = SUB_IDLE_READY;
  gStatus.cycleCurrent = 0;
  gStatus.cycleTarget  = 5;

  gStatus.runTime_s    = 300;   // 5 min
  gStatus.coolTime_s   = 120;   // 2 min

  gStatus.timeInState_s    = 0;
  gStatus.timeRemaining_s  = 0;

  // These will be updated from hardware on first tick
  gStatus.doorClosed  = true;
  gStatus.estopOk     = true;
  gStatus.lidLocked   = true;

  gStatus.pidPv_c     = -85.0f;
  gStatus.pidSv_c     = -90.0f;
  gStatus.pidAlarm    = false;

  gStatus.heartbeat   = 0;
}

// Read interlock inputs from DIN hardware
void updateInterlocksFromHardware() {
  // Active HIGH = OK (HIGH = true, LOW = fault)
  gStatus.doorClosed = (digitalRead(PIN_DIN_DOOR)  == HIGH);
  gStatus.estopOk    = (digitalRead(PIN_DIN_ESTOP) == HIGH);
  gStatus.lidLocked  = (digitalRead(PIN_DIN_LID)   == HIGH);
}

// called once per STATUS_INTERVAL_MS
void updateMillStatusFromTick() {
  gStatus.heartbeat++;

  // Always refresh interlocks from hardware first
  updateInterlocksFromHardware();

  // If any interlock goes bad while RUN/HOLD, trip to FAULT
  bool interlockOk = gStatus.doorClosed && gStatus.estopOk && gStatus.lidLocked;
  if (!interlockOk && (gStatus.state == STATE_RUN || gStatus.state == STATE_HOLD)) {
    gStatus.state          = STATE_FAULT;
    gStatus.substate       = SUB_FAULT_INTERLOCK;
    gStatus.timeInState_s  = 0;
    gStatus.timeRemaining_s = 0;
    showStateColor();  // will go red
    return;            // don't advance timers when faulted
  }

  if (gStatus.state == STATE_RUN || gStatus.state == STATE_HOLD) {
    gStatus.timeInState_s++;
    if (gStatus.timeRemaining_s > 0) {
      gStatus.timeRemaining_s--;
    } else if (gStatus.state == STATE_RUN) {
      // end-of-phase transitions while RUN
      if (gStatus.substate == SUB_RUN_ACTIVE) {
        gStatus.substate        = SUB_RUN_COOLING;
        gStatus.timeInState_s   = 0;
        gStatus.timeRemaining_s = gStatus.coolTime_s;
      } else if (gStatus.substate == SUB_RUN_COOLING) {
        gStatus.substate        = SUB_RUN_ACTIVE;
        gStatus.timeInState_s   = 0;
        gStatus.timeRemaining_s = gStatus.runTime_s;
        gStatus.cycleCurrent++;
        if (gStatus.cycleCurrent > gStatus.cycleTarget) {
          gStatus.state       = STATE_IDLE;
          gStatus.substate    = SUB_IDLE_READY;
          gStatus.timeInState_s   = 0;
          gStatus.timeRemaining_s = 0;
        }
      }
    }
  }

  // wiggle LN2 PV so the gauge moves
  float t = (float)gStatus.heartbeat;
  gStatus.pidPv_c = gStatus.pidSv_c + 5.0f * sinf(t / 30.0f);
}

// enum → string helpers
const char* stateToString(MillState s) {
  switch (s) {
    case STATE_IDLE:  return "IDLE";
    case STATE_RUN:   return "RUN";
    case STATE_HOLD:  return "HOLD";
    case STATE_FAULT: return "FAULT";
    default:          return "UNKNOWN";
  }
}

const char* substateToString(MillSubstate s) {
  switch (s) {
    case SUB_IDLE_READY:       return "IDLE_READY";
    case SUB_RUN_ACTIVE:       return "RUN_ACTIVE";
    case SUB_RUN_COOLING:      return "RUN_COOLING";
    case SUB_HOLD_USER:        return "HOLD_USER";
    case SUB_HOLD_INTERLOCK:   return "HOLD_INTERLOCK";
    case SUB_FAULT_INTERLOCK:  return "FAULT_INTERLOCK";
    case SUB_FAULT_DEVICE:     return "FAULT_DEVICE";
    case SUB_FAULT_INTERNAL:   return "FAULT_INTERNAL";
    default:                   return "UNKNOWN";
  }
}

// Keep RGB color consistent with current state
void showStateColor() {
  switch (gStatus.state) {
    case STATE_IDLE:
      // IDLE: LED off
      RGB_Light(0, 0, 0);
      break;

    case STATE_RUN:
      // RUN: solid green
      RGB_Light(0, 32, 0);
      break;

    case STATE_HOLD:
      // HOLD: yellow
      RGB_Light(32, 32, 0);
      break;

    case STATE_FAULT:
      // FAULT: red (later we can blink)
      RGB_Light(32, 0, 0);
      break;

    default:
      RGB_Light(0, 0, 0);
      break;
  }
}

// ========================================================================
// MQTT HANDLING
// ========================================================================

void ensureMqttConnected() {
  if (mqtt.connected()) return;

  Serial.print(F("Connecting to MQTT... "));

  while (!mqtt.connected()) {
    if (mqtt.connect(mqttClientId)) {
      Serial.println(F("connected"));
      mqtt.subscribe(TOPIC_CMD_CONTROL);
      Serial.print(F("Subscribed to "));
      Serial.println(TOPIC_CMD_CONTROL);
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqtt.state());
      Serial.println(F(" retrying in 2 seconds"));
      delay(2000);
    }
  }
}

void publishStatus() {
  char json[512];

  unsigned long ts = millis() / 1000;

  int n = snprintf(
    json, sizeof(json),
    "{"
      "\"ts\":%lu,"
      "\"state\":\"%s\","
      "\"substate\":\"%s\","
      "\"cycle_current\":%u,"
      "\"cycle_target\":%u,"
      "\"run_time_s\":%lu,"
      "\"cool_time_s\":%lu,"
      "\"time_in_state_s\":%lu,"
      "\"time_remaining_s\":%ld,"
      "\"interlocks\":{"
        "\"door_closed\":%s,"
        "\"estop_ok\":%s,"
        "\"lid_locked\":%s"
      "},"
      "\"pid\":{"
        "\"pv_c\":%.1f,"
        "\"sv_c\":%.1f,"
        "\"alarm\":%s"
      "},"
      "\"heartbeat\":%lu"
    "}",
    ts,
    stateToString(gStatus.state),
    substateToString(gStatus.substate),
    gStatus.cycleCurrent,
    gStatus.cycleTarget,
    (unsigned long)gStatus.runTime_s,
    (unsigned long)gStatus.coolTime_s,
    (unsigned long)gStatus.timeInState_s,
    (long)gStatus.timeRemaining_s,
    gStatus.doorClosed ? "true" : "false",
    gStatus.estopOk    ? "true" : "false",
    gStatus.lidLocked  ? "true" : "false",
    gStatus.pidPv_c,
    gStatus.pidSv_c,
    gStatus.pidAlarm   ? "true" : "false",
    (unsigned long)gStatus.heartbeat
  );

  if (n <= 0 || n >= (int)sizeof(json)) {
    Serial.println(F("Status JSON truncated / format error"));
    return;
  }

  bool ok = mqtt.publish(TOPIC_STATUS_STATE, json);
  if (!ok) {
    Serial.println(F("MQTT publish failed"));
  }
}

// payload is JSON like {"cmd":"START","source":"HMI","ts":...}
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("MQTT message on topic "));
  Serial.print(topic);
  Serial.print(F(": "));

  static char buf[256];
  length = (length >= sizeof(buf)) ? sizeof(buf) - 1 : length;
  memcpy(buf, payload, length);
  buf[length] = '\0';

  Serial.println(buf);

  if (strcmp(topic, TOPIC_CMD_CONTROL) == 0) {
    handleControlCommand(buf, length);
  }
}

// naive "parser": look for "cmd":"..."
void handleControlCommand(const char* payload, size_t len) {
  const char* cmdKey = "\"cmd\"";
  const char* p = strstr(payload, cmdKey);
  if (!p) {
    Serial.println(F("No cmd field in payload"));
    return;
  }

  p = strchr(p + strlen(cmdKey), ':');
  if (!p) return;
  p++;

  while (*p && *p == ' ') p++;
  if (*p != '\"') return;
  p++;

  char cmd[16];
  size_t i = 0;
  while (*p && *p != '\"' && i < sizeof(cmd) - 1) {
    cmd[i++] = *p++;
  }
  cmd[i] = '\0';

  Serial.print(F("Parsed cmd = "));
  Serial.println(cmd);

  // Map commands → state; keep RGB in sync with showStateColor()

  if (strcmp(cmd, "START") == 0) {
    // Only allow START if all interlocks are OK
    bool interlockOk = gStatus.doorClosed && gStatus.estopOk && gStatus.lidLocked;
    if (!interlockOk) {
      Serial.println(F("START rejected: interlocks not OK"));
      gStatus.state    = STATE_FAULT;
      gStatus.substate = SUB_FAULT_INTERLOCK;
      gStatus.timeInState_s   = 0;
      gStatus.timeRemaining_s = 0;
      showStateColor();  // red
      return;
    }

    gStatus.state       = STATE_RUN;
    gStatus.substate    = SUB_RUN_ACTIVE;
    if (gStatus.cycleCurrent == 0 || gStatus.cycleCurrent > gStatus.cycleTarget) {
      gStatus.cycleCurrent = 1;
    }
    gStatus.timeInState_s   = 0;
    gStatus.timeRemaining_s = gStatus.runTime_s;

    showStateColor();  // solid green

  } else if (strcmp(cmd, "STOP") == 0) {
    gStatus.state       = STATE_IDLE;
    gStatus.substate    = SUB_IDLE_READY;
    gStatus.timeInState_s   = 0;
    gStatus.timeRemaining_s = 0;

    showStateColor();  // off

  } else if (strcmp(cmd, "HOLD") == 0) {
    if (gStatus.state == STATE_RUN) {
      gStatus.state    = STATE_HOLD;
      gStatus.substate = SUB_HOLD_USER;

      showStateColor(); // yellow
    }

  } else if (strcmp(cmd, "RESUME") == 0) {
    if (gStatus.state == STATE_HOLD) {
      gStatus.state = STATE_RUN;
      // substate unchanged (RUN_ACTIVE or RUN_COOLING)
      showStateColor(); // green again
    }

  } else if (strcmp(cmd, "RESET_FAULT") == 0) {
    if (gStatus.state == STATE_FAULT) {
      // Only clear FAULT if interlocks are now OK
      bool interlockOk = gStatus.doorClosed && gStatus.estopOk && gStatus.lidLocked;
      if (interlockOk) {
        gStatus.state       = STATE_IDLE;
        gStatus.substate    = SUB_IDLE_READY;
        gStatus.timeInState_s   = 0;
        gStatus.timeRemaining_s = 0;
        showStateColor(); // off
      } else {
        Serial.println(F("RESET_FAULT rejected: interlocks still not OK"));
      }
    }

  } else {
    Serial.println(F("Unknown command, ignoring"));
  }
}
