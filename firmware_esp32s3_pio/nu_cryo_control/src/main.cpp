#include <Arduino.h>

#include <SPI.h>
#include <ETH.h>
#include <WiFi.h>          // Provides WiFiClient; ETH uses the same lwIP sockets on ESP32
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "app/app_config.h"
#include "core/mqtt_bus.h"
#include "core/health_manager.h"

#if NUCRYO_USE_MODBUS_RTU
  #include <ModbusRTU.h>
  #include "components/pid_modbus.h"
#endif

// ---------------------------
// Safe defaults (override in app_config.h)
// ---------------------------
#ifndef MACHINE_ID
  #define MACHINE_ID "nu-cryo"
#endif

#ifndef NODE_ID
  #define NODE_ID "esp32s3"
#endif

#ifndef MQTT_BROKER_HOST
  // Prefer setting this in app_config.h (hostname or IP).
  #define MQTT_BROKER_HOST "192.168.1.2"
#endif

#ifndef MQTT_BROKER_PORT
  #define MQTT_BROKER_PORT 1883
#endif

#ifndef MQTT_TOPIC_BOOT
  // These are SUBTOPICS under the MqttBus root.
  #define MQTT_TOPIC_BOOT "status/boot"
#endif

#ifndef MQTT_TOPIC_LWT
  // Included for completeness; LWT handling is typically part of the connect() call.
  #define MQTT_TOPIC_LWT "status/lwt"
#endif

// ---------------------------
// W5500 (SPI Ethernet) pin defaults (override in app_config.h)
// ---------------------------
#if NUCRYO_USE_ETH_W5500

  #ifndef W5500_CS_PIN
    #define W5500_CS_PIN 10
  #endif
  #ifndef W5500_INT_PIN
    #define W5500_INT_PIN -1
  #endif
  #ifndef W5500_RST_PIN
    #define W5500_RST_PIN -1
  #endif
  #ifndef W5500_SCK_PIN
    #define W5500_SCK_PIN SCK
  #endif
  #ifndef W5500_MISO_PIN
    #define W5500_MISO_PIN MISO
  #endif
  #ifndef W5500_MOSI_PIN
    #define W5500_MOSI_PIN MOSI
  #endif

#endif // NUCRYO_USE_ETH_W5500

#if NUCRYO_USE_MODBUS_RTU
  // RS-485 defaults (override in app_config.h)
  #ifndef RS485_BAUD
    #define RS485_BAUD 9600
  #endif
  #ifndef RS485_RX_PIN
    #define RS485_RX_PIN -1
  #endif
  #ifndef RS485_TX_PIN
    #define RS485_TX_PIN -1
  #endif
  #ifndef RS485_DE_RE_PIN
    #define RS485_DE_RE_PIN -1
  #endif
#endif

// ---------------------------
// Globals
// ---------------------------

// Socket client used by PubSubClient. On ESP32, ETH uses lwIP sockets,
// so WiFiClient works for both Wi-Fi and Ethernet transports.
static WiFiClient g_net;
static PubSubClient g_mqtt(g_net);
static MqttBus g_bus;

static HealthManager g_health;

#if NUCRYO_USE_MODBUS_RTU
  static ModbusRTU g_mb;

  // IMPORTANT: Update the slave IDs to match your actual PID controller IDs.
  static PidModbusComponent pid_heat1("heat1", /*slave_id*/ 1, g_mb);
  static PidModbusComponent pid_heat2("heat2", /*slave_id*/ 2, g_mb);
  static PidModbusComponent pid_cool1("cool1", /*slave_id*/ 3, g_mb);
#endif

static bool g_eth_has_ip = false;

// ---------------------------
// Helpers
// ---------------------------

static void eth_begin_w5500() {
#if NUCRYO_USE_ETH_W5500
  // Ensure SPI pins are explicitly configured (important on some custom boards).
  SPI.begin(W5500_SCK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN, W5500_CS_PIN);

  // Bring up SPI Ethernet (W5500) using Arduino-ESP32 ETH API.
  // NOTE: Some cores ignore phy_addr for W5500; it's kept for signature compatibility.
  ETH.begin(ETH_PHY_W5500, /*phy_addr*/ 1, W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN, SPI);

  // Optional: set hostname if supported by the core (safe to call even if ignored).
  ETH.setHostname(NODE_ID);
#endif
}

static void wait_for_ip(uint32_t timeout_ms = 8000) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    // ETH.localIP() becomes non-zero when an address is assigned.
    if (ETH.localIP()) {
      g_eth_has_ip = true;
      return;
    }
    delay(50);
  }
  g_eth_has_ip = false;
}

static void publish_boot() {
  // ArduinoJson v7: prefer JsonDocument over StaticJsonDocument (avoids deprecation warnings).
  JsonDocument doc;
  doc["node"] = NODE_ID;
  doc["machine"] = MACHINE_ID;
  doc["ms"] = millis();

  // Publish under the bus root (subtopic).
  g_bus.publish_json(MQTT_TOPIC_BOOT, doc, /*retain*/ true);
}

// ---------------------------
// Arduino entry points
// ---------------------------

void setup() {
  Serial.begin(115200);
  delay(150);

#if NUCRYO_USE_ETH_W5500
  eth_begin_w5500();
  wait_for_ip();
#endif

  // MQTT configuration
  g_mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);

#if defined(MQTT_MAX_PACKET_SIZE)
  // PubSubClient: configure payload buffer size at runtime as well.
  g_mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
#endif

  // Initialize our MQTT bus wrapper (sets root prefix, callback shim, etc.)
  g_bus.begin(g_mqtt, MACHINE_ID, NODE_ID);

  // Configure health components
#if NUCRYO_USE_MODBUS_RTU
  // Modbus RTU setup (RS-485 master)
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  g_mb.begin(&Serial2, RS485_DE_RE_PIN);
  g_mb.master();

  // Register PID components with health manager
  pid_heat1.configure(/*expected*/ true, /*required*/ true);
  pid_heat2.configure(/*expected*/ true, /*required*/ true);
  pid_cool1.configure(/*expected*/ true, /*required*/ true);

  g_health.add(&pid_heat1);
  g_health.add(&pid_heat2);
  g_health.add(&pid_cool1);
#endif

  publish_boot();
}

void loop() {
  const uint32_t now_ms = millis();

#if NUCRYO_USE_MODBUS_RTU
  // The modbus-esp8266 library requires periodic servicing.
  g_mb.task();
#endif

  // Keep MQTT session healthy.
  // ensure_connected() should be lightweight; call it at a modest cadence.
  static uint32_t last_conn_try = 0;
  if (now_ms - last_conn_try >= 1000) {
    last_conn_try = now_ms;
    (void)g_bus.ensure_connected();
  }
  g_bus.loop();

  // Tick components on a fixed cadence
  static uint32_t last_tick = 0;
  if (now_ms - last_tick >= 250) {
    last_tick = now_ms;

#if NUCRYO_USE_MODBUS_RTU
    pid_heat1.tick(now_ms);
    pid_heat2.tick(now_ms);
    pid_cool1.tick(now_ms);
#endif
  }

  // Aggregate system health (does not call tick() for you)
  g_health.evaluate(now_ms);

  delay(5);
}
