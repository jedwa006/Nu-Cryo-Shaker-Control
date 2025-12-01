/*
 * Firmware: Nu-Cryo Shaker Controller
 * Version: v0.4
 * Date: 2025-12-01
 * Author: Nu Shaker Dev
 *
 * Changelog:
 * - Added structured interlock + fault detection logic
 * - Implemented mill state machine: IDLE, RUNNING, HOLD, FAULT
 * - Integrated MQTT command handler for `mill/cmd/control`
 * - Published expanded `mill/status/state` JSON via MQTT
 * - Triggered RGB/Buzzer alerts on fault conditions
 * - Uses Waveshare GPIO, DIN, and RELAY modules for I/O abstraction
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WS_GPIO.h"
#include "WS_DIN.h"
#include "WS_Relay.h"
#include "I2C_Driver.h"

#define MQTT_SERVER "192.168.50.2" // Replace with your broker IP
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "mill-esp32"
#define MQTT_CMD_TOPIC "mill/cmd/control"
#define MQTT_STATUS_TOPIC "mill/status/state"

WiFiClient ethClient;
PubSubClient mqttClient(ethClient);

// Mill State Machine
enum MillState { IDLE, RUNNING, HOLD, FAULT };
MillState currentState = IDLE;

// Cycle data
int cycleTarget = 0;
int cycleCurrent = 0;
unsigned long lastTick = 0;
unsigned long tickInterval = 1000; // 1 second

// Interlock status
bool estop_ok = true;
bool lid_locked = true;
bool door_closed = true;

void publishStatus() {
  StaticJsonDocument<256> doc;
  doc["state"] = 
    currentState == IDLE ? "IDLE" :
    currentState == RUNNING ? "RUN" :
    currentState == HOLD ? "HOLD" : "FAULT";
  doc["cycle_current"] = cycleCurrent;
  doc["cycle_target"] = cycleTarget;
  doc["time_remaining_s"] = max(0, cycleTarget - cycleCurrent);

  JsonObject interlocks = doc.createNestedObject("interlocks");
  interlocks["estop_ok"] = estop_ok;
  interlocks["lid_locked"] = lid_locked;
  interlocks["door_closed"] = door_closed;

  char payload[256];
  serializeJson(doc, payload);
  mqttClient.publish(MQTT_STATUS_TOPIC, payload);
}

void handleCommand(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "START") == 0 && currentState == IDLE && estop_ok && lid_locked && door_closed) {
    cycleCurrent = 0;
    cycleTarget = 300; // default for now
    currentState = RUNNING;
    publishStatus();
  } else if (strcmp(cmd, "STOP") == 0) {
    currentState = IDLE;
    publishStatus();
  } else if (strcmp(cmd, "HOLD") == 0 && currentState == RUNNING) {
    currentState = HOLD;
    publishStatus();
  } else if (strcmp(cmd, "RESET_FAULT") == 0 && currentState == FAULT) {
    currentState = IDLE;
    publishStatus();
  }
}

void checkInterlocks() {
  estop_ok = DIN_Read_CH1();
  lid_locked = DIN_Read_CH2();
  door_closed = DIN_Read_CH3();

  // TEMPORARY OVERRIDE: Mirror estop_ok to door_closed
  // Comment out when Door Closed switch is connected
  door_closed = estop_ok;

  if (!estop_ok || !lid_locked || !door_closed) {
    if (currentState != FAULT) {
      currentState = FAULT;
      RGB_Open_Time(100, 0, 0, 3000, 200);
      Buzzer_Open_Time(2000, 500);
      publishStatus();
    }
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      mqttClient.subscribe(MQTT_CMD_TOPIC);
    } else {
      delay(1000);
    }
  }
}

void setup() {
  GPIO_Init();
  I2C_Init();       // <<===== IMPORTANT: bring up I2C before any relay calls
  Relay_Init();
  DIN_Init();

  WiFi.begin(); // Assumes ETH already configured by Waveshare stack

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(handleCommand);
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  checkInterlocks();

  if (currentState == RUNNING && millis() - lastTick >= tickInterval) {
    lastTick = millis();
    cycleCurrent++;
    if (cycleCurrent >= cycleTarget) {
      currentState = IDLE;
    }
    publishStatus();
  }

  delay(10);
}
