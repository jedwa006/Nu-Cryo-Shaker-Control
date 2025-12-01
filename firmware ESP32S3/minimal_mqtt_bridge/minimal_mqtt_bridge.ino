/*
  minimal_mqtt_bridge.ino
  ESP32-S3-ETH-8DI-8RO  ←→  W5500  ←→  Mosquitto on Pi

  v0: 
    - Bring up Ethernet (W5500, Waveshare pinout)
    - Static IP: 192.168.50.10
    - Connect to MQTT broker on Pi at 192.168.50.2:1883
    - Subscribe to mill/cmd/control
    - Publish mill/status/state once per second
    - Toggle RGB LED based on START/STOP/HOLD/RESUME

  Requires:
    - Ethernet library
    - PubSubClient library
    - Waveshare WS_GPIO.h (and its .cpp) in the sketch folder or as a library
*/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// Waveshare board helpers (pins, etc.)
#include "WS_GPIO.h"   // from Waveshare examples

// ----------- BOARD / HARDWARE CONFIG (Waveshare ESP32-S3-ETH-8DI-8RO) -----

// W5500 / Ethernet pins (from WS_ETH.h)
#define ETH_CS_PIN    16
#define ETH_IRQ_PIN   12
#define ETH_RST_PIN   39

#define SPI_SCK_PIN   15
#define SPI_MISO_PIN  14
#define SPI_MOSI_PIN  13

// Use on-board RGB LED as "test output"
#define TEST_OUTPUT_PIN  GPIO_PIN_RGB   // defined in WS_GPIO.h

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
void handleControlCommand(const char* payload, size_t len);
void publishStatus();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void ensureMqttConnected();

// ========================================================================
// SETUP
// ========================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println(F("minimal_mqtt_bridge starting..."));

  // Initialize GPIO / RGB / buzzer tasks from Waveshare library
  GPIO_Init();

  // Test output (RGB LED)
  // pinMode(TEST_OUTPUT_PIN, OUTPUT);
  // digitalWrite(TEST_OUTPUT_PIN, LOW);

  // Optional: a power-on color so you know the RGB works
  RGB_Light(0, 0, 16);  // dim blue on boot

  // Optional: reset W5500
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

  // 🔧 Increase MQTT buffer size to handle our JSON payload
  mqtt.setBufferSize(512);

  initMillStatus();
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
// MILL STATE MACHINE IMPLEMENTATION (v0 SIM)
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

  gStatus.doorClosed  = true;
  gStatus.estopOk     = true;
  gStatus.lidLocked   = true;

  gStatus.pidPv_c     = -85.0f;
  gStatus.pidSv_c     = -90.0f;
  gStatus.pidAlarm    = false;

  gStatus.heartbeat   = 0;
}

// called once per STATUS_INTERVAL_MS
void updateMillStatusFromTick() {
  gStatus.heartbeat++;

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

  // Map commands → state + test output

  if (strcmp(cmd, "START") == 0) {
    gStatus.state       = STATE_RUN;
    gStatus.substate    = SUB_RUN_ACTIVE;
    if (gStatus.cycleCurrent == 0 || gStatus.cycleCurrent > gStatus.cycleTarget) {
      gStatus.cycleCurrent = 1;
    }
    gStatus.timeInState_s   = 0;
    gStatus.timeRemaining_s = gStatus.runTime_s;
    digitalWrite(TEST_OUTPUT_PIN, HIGH);  // RGB on

  } else if (strcmp(cmd, "STOP") == 0) {
    gStatus.state       = STATE_IDLE;
    gStatus.substate    = SUB_IDLE_READY;
    gStatus.timeInState_s   = 0;
    gStatus.timeRemaining_s = 0;
    digitalWrite(TEST_OUTPUT_PIN, LOW);   // RGB off

  } else if (strcmp(cmd, "HOLD") == 0) {
    if (gStatus.state == STATE_RUN) {
      gStatus.state    = STATE_HOLD;
      gStatus.substate = SUB_HOLD_USER;
      // keep remaining time; timers still tick in update() but
      // you can change that behaviour later if you prefer
    }

  } else if (strcmp(cmd, "RESUME") == 0) {
    if (gStatus.state == STATE_HOLD) {
      gStatus.state = STATE_RUN;
      // substate unchanged (RUN_ACTIVE or RUN_COOLING)
    }

  } else if (strcmp(cmd, "RESET_FAULT") == 0) {
    if (gStatus.state == STATE_FAULT) {
      if (gStatus.doorClosed && gStatus.estopOk && gStatus.lidLocked) {
        gStatus.state       = STATE_IDLE;
        gStatus.substate    = SUB_IDLE_READY;
        gStatus.timeInState_s   = 0;
        gStatus.timeRemaining_s = 0;
      }
    }

  } else {
    Serial.println(F("Unknown command, ignoring"));
  }
}
