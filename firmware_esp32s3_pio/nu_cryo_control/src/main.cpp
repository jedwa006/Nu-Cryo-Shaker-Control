#include <Arduino.h>

#include <SPI.h>
#include <ETH.h>
#include <Network.h>         // Network.onEvent()
#include <NetworkClient.h>   // NetworkClient

#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "app/app_config.h"
#include "core/health_manager.h"
#include "core/mqtt_bus.h"

#if NUCRYO_USE_MODBUS_RTU
  #include <HardwareSerial.h>
  #include <ModbusRTU.h>
  #include "components/pid_modbus.h"
#endif

// -------------------------------------------------------------------------------------------------
// Pin fallbacks (Waveshare ESP32-S3-ETH-8DI-8RO / W5500)
// These match Waveshare's vendor code + wiki.
// If you already define these in app_config.h, those values win.
// -------------------------------------------------------------------------------------------------
#ifndef W5500_INT_PIN
  #define W5500_INT_PIN 12
#endif
#ifndef W5500_MOSI_PIN
  #define W5500_MOSI_PIN 13
#endif
#ifndef W5500_MISO_PIN
  #define W5500_MISO_PIN 14
#endif
#ifndef W5500_SCK_PIN
  #define W5500_SCK_PIN 15
#endif
#ifndef W5500_CS_PIN
  #define W5500_CS_PIN 16
#endif
#ifndef W5500_RST_PIN
  #define W5500_RST_PIN 39
#endif

// MQTT subtopics (under MqttBus root())
#ifndef MQTT_SUBTOPIC_LWT
  #define MQTT_SUBTOPIC_LWT    "status/lwt"
#endif
#ifndef MQTT_SUBTOPIC_BOOT
  #define MQTT_SUBTOPIC_BOOT   "status/boot"
#endif
#ifndef MQTT_SUBTOPIC_HEALTH
  #define MQTT_SUBTOPIC_HEALTH "status/health"
#endif

// -------------------------------------------------------------------------------------------------
// Networking state & MQTT plumbing
// -------------------------------------------------------------------------------------------------
static bool g_eth_connected = false;
static bool g_eth_static = false;
static NetworkClient g_net_client;
static PubSubClient g_mqtt(g_net_client);

// -------------------------------------------------------------------------------------------------
// Core objects
// -------------------------------------------------------------------------------------------------
static HealthManager g_health;
static MqttBus g_bus;

// -------------------------------------------------------------------------------------------------
// Health component: Ethernet link
// -------------------------------------------------------------------------------------------------
class EthHealthComponent : public IHealthComponent {
public:
  const char* name() const override { return "eth"; }

  void configure(bool expected, bool required) override {
    rep_.expected = expected;
    rep_.required = required;
    rep_.status = expected ? HealthStatus::MISSING : HealthStatus::OK;
    rep_.severity = expected ? Severity::WARN : Severity::INFO;
    rep_.reason = expected ? "init" : "n/a";
    rep_.since_ms = millis();
    rep_.last_ok_ms = 0;
  }

  bool probe(uint32_t now_ms) override {
    // For ETH, "probe" just reflects current link state.
    return tick(now_ms);
  }

  bool tick(uint32_t now_ms) override {
    if (!rep_.expected) {
      rep_.status = HealthStatus::OK;
      rep_.severity = Severity::INFO;
      rep_.reason = "disabled";
      rep_.since_ms = now_ms;
      return true;
    }

    if (g_eth_connected) {
      if (rep_.status != HealthStatus::OK) rep_.since_ms = now_ms;
      rep_.status = HealthStatus::OK;
      rep_.severity = Severity::INFO;
      rep_.reason = "up";
      rep_.last_ok_ms = now_ms;
      return true;
    }

    // Link down (or no IP yet)
    if (rep_.status == HealthStatus::OK) rep_.since_ms = now_ms;
    rep_.status = HealthStatus::MISSING;
    rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
    rep_.reason = "down";
    return false;
  }

  uint32_t stale_timeout_ms() const override { return 0; }
  HealthReport report() const override { return rep_; }

private:
  HealthReport rep_ {};
};

static EthHealthComponent eth_health;

// -------------------------------------------------------------------------------------------------
// Optional: Modbus RTU PID components
// -------------------------------------------------------------------------------------------------
#if NUCRYO_USE_MODBUS_RTU

// Waveshare vendor header uses UART1 pins: TX=17, RX=18 (RS485/CAN block)
#ifndef RS485_TX_PIN
  #define RS485_TX_PIN 17
#endif
#ifndef RS485_RX_PIN
  #define RS485_RX_PIN 18
#endif
#ifndef RS485_BAUD
  #define RS485_BAUD 9600
#endif

#ifndef PID_HEAT1_SLAVE_ID
  #define PID_HEAT1_SLAVE_ID 1
#endif
#ifndef PID_HEAT2_SLAVE_ID
  #define PID_HEAT2_SLAVE_ID 2
#endif
#ifndef PID_COOL1_SLAVE_ID
  #define PID_COOL1_SLAVE_ID 3
#endif

static ModbusRTU g_mb;

static PidModbusComponent pid_heat1("pid_heat1", PID_HEAT1_SLAVE_ID, g_mb);
static PidModbusComponent pid_heat2("pid_heat2", PID_HEAT2_SLAVE_ID, g_mb);
static PidModbusComponent pid_cool1("pid_cool1", PID_COOL1_SLAVE_ID, g_mb);

#endif

// -------------------------------------------------------------------------------------------------
// Ethernet event handling (Arduino-ESP32 v3.x style)
// -------------------------------------------------------------------------------------------------
static void on_eth_event(arduino_event_id_t event, arduino_event_info_t /*info*/)
{
  // NOTE: called from another FreeRTOS task.
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[eth] start");
      ETH.setHostname("nu-cryo-esp32s3");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[eth] link up");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[eth] got ip: ");
      Serial.println(ETH.localIP());
      g_eth_connected = true;
      break;

#ifdef ARDUINO_EVENT_ETH_LOST_IP
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("[eth] lost ip");
      g_eth_connected = false;
      break;
#endif

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[eth] link down");
      g_eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[eth] stop");
      g_eth_connected = false;
      break;

    default:
      break;
  }
}

static bool eth_begin()
{
#if NUCRYO_USE_ETH_W5500
  Network.onEvent(on_eth_event);

  // Waveshare W5500 is on dedicated SPI pins.
  SPI.begin(W5500_SCK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN);

  // Extra hard reset pulse (helps with "w5500_reset(): reset timeout" cases)
#if (W5500_RST_PIN >= 0)
  pinMode(W5500_RST_PIN, OUTPUT);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(50);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(150);
#endif

  // phy_addr is typically 1 in Arduino-ESP32 W5500 examples.
  bool ok = ETH.begin(ETH_PHY_W5500, /*phy_addr*/ 1,
                      W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN,
                      SPI);

  if (!ok) {
    Serial.println("[eth] ETH.begin() failed (W5500 init) â€” continuing without network");
    g_eth_connected = false;
    return false;
  }

  const bool static_requested = (ETH_STATIC_IP.a | ETH_STATIC_IP.b | ETH_STATIC_IP.c | ETH_STATIC_IP.d) != 0;
  if (static_requested) {
    IPAddress local(ETH_STATIC_IP.a, ETH_STATIC_IP.b, ETH_STATIC_IP.c, ETH_STATIC_IP.d);
    IPAddress gateway(ETH_STATIC_GW.a, ETH_STATIC_GW.b, ETH_STATIC_GW.c, ETH_STATIC_GW.d);
    IPAddress subnet(ETH_STATIC_MASK.a, ETH_STATIC_MASK.b, ETH_STATIC_MASK.c, ETH_STATIC_MASK.d);
    const bool configured = ETH.config(local, gateway, subnet, gateway);
    g_eth_static = configured;

    if (configured) {
      Serial.print("[eth] static addressing ");
      Serial.println(local);
    } else {
      Serial.println("[eth] ETH.config() failed (static) â€” using DHCP");
    }
  } else {
    Serial.println("[eth] DHCP (no static IP configured)");
    g_eth_static = false;
  }
#endif
  return true;
}

// -------------------------------------------------------------------------------------------------
// MQTT connect + boot/LWT publishing
// -------------------------------------------------------------------------------------------------
static String mqtt_full_topic(const char* subtopic)
{
  String t = g_bus.root();
  t += "/";
  t += subtopic;
  return t;
}

static bool mqtt_connect(uint32_t now_ms)
{
  g_mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);

#if defined(MQTT_MAX_PACKET_SIZE)
  g_mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
#endif

  // Last Will: retained offline message at <root>/status/lwt
  JsonDocument lwt;
  lwt["v"] = NUCRYO_SCHEMA_V;
  lwt["ts_ms"] = now_ms;
  lwt["state"] = "offline";

  char lwt_buf[160];
  size_t n = serializeJson(lwt, lwt_buf, sizeof(lwt_buf));
  if (n >= sizeof(lwt_buf)) n = sizeof(lwt_buf) - 1;
  lwt_buf[n] = ' ';

  const String willTopic = mqtt_full_topic(MQTT_SUBTOPIC_LWT);
  return g_mqtt.connect(NODE_ID, willTopic.c_str(),
                        /*willQos*/ 1, /*willRetain*/ true,
                        /*willMessage*/ lwt_buf);
}

static void publish_lwt_online(uint32_t now_ms)
{
  JsonDocument doc;
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["state"] = "online";
  g_bus.publish_json(MQTT_SUBTOPIC_LWT, doc, /*retained*/ true, /*qos*/ 1);
}

static void publish_boot(uint32_t now_ms)
{
  JsonDocument doc;
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["node"] = NODE_ID;
  doc["fw"] = "nu_cryo_control_pio";
  doc["eth"] = g_eth_connected;
  doc["eth_static"] = g_eth_static;

  if (g_eth_connected) {
    doc["ip"] = ETH.localIP().toString();
  }

  g_bus.publish_json(MQTT_SUBTOPIC_BOOT, doc, /*retained*/ true);
}

// -------------------------------------------------------------------------------------------------
// Setup / Loop
// -------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println("\n[nu-cryo] boot");

  // Bus can be initialized before MQTT connects; it just stores root + callback shims.
  g_bus.begin(g_mqtt, MACHINE_ID, NODE_ID);

  // Health policy: register and configure components
  eth_health.configure(/*expected*/ true, /*required*/ true);
  g_health.add(&eth_health);

#if NUCRYO_USE_MODBUS_RTU
  // Bring up RS485 UART and Modbus stack.
  Serial1.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  g_mb.begin(&Serial1);
  g_mb.master();

  pid_heat1.configure(true, true);
  pid_heat2.configure(true, true);
  pid_cool1.configure(true, true);

  g_health.add(&pid_heat1);
  g_health.add(&pid_heat2);
  g_health.add(&pid_cool1);
#endif

  // Bring up Ethernet (W5500)
  eth_begin();
}

void loop()
{
  const uint32_t now_ms = millis();

  // Tick components (HealthManager only evaluates; it doesn't call tick())
  static uint32_t last_eth_tick = 0;
  if (now_ms - last_eth_tick >= 250) {
    last_eth_tick = now_ms;
    eth_health.tick(now_ms);
  }

#if NUCRYO_USE_MODBUS_RTU
  static uint32_t last_pid_tick = 0;
  if (now_ms - last_pid_tick >= 200) {
    last_pid_tick = now_ms;
    // Service modbus stack + refresh data
    g_mb.task();

    pid_heat1.tick(now_ms);
    pid_heat2.tick(now_ms);
    pid_cool1.tick(now_ms);
  }
#endif

  // MQTT state machine (only attempt connect when ETH is up)
  if (g_eth_connected) {
    if (!g_mqtt.connected()) {
      static uint32_t last_try = 0;
      if (now_ms - last_try > 2000) {
        last_try = now_ms;
        Serial.println("[mqtt] connecting...");
        if (mqtt_connect(now_ms)) {
          Serial.println("[mqtt] connected");
          publish_boot(now_ms);
          publish_lwt_online(now_ms);
        } else {
          Serial.print("[mqtt] failed rc=");
          Serial.println(g_mqtt.state());
        }
      }
    }
  }

  // Let the bus pump MQTT callbacks when connected
  g_bus.loop();

  // Evaluate system health (stale logic + aggregation)
  g_health.evaluate(now_ms);

  // Publish health summary (1 Hz) when MQTT is up
  static uint32_t last_health_pub = 0;
  if (g_mqtt.connected() && (now_ms - last_health_pub > 1000)) {
    last_health_pub = now_ms;

    const SystemHealth& sh = g_health.system_health();

    JsonDocument doc;
    doc["v"] = NUCRYO_SCHEMA_V;
    doc["ts_ms"] = now_ms;
    doc["degraded"] = sh.degraded;
    doc["run_allowed"] = sh.run_allowed;
    doc["outputs_allowed"] = sh.outputs_allowed;
    doc["warn"] = sh.warn_count;
    doc["crit"] = sh.crit_count;

    g_bus.publish_json(MQTT_SUBTOPIC_HEALTH, doc, /*retained*/ false);
  }

  delay(5);
}
