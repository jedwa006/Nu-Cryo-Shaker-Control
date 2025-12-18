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

static ModbusRTU g_mb;

static PidModbusComponent pid_heat1("pid_heat1", MODBUS_CONFIG.pid_heat1_id, g_mb);
static PidModbusComponent pid_heat2("pid_heat2", MODBUS_CONFIG.pid_heat2_id, g_mb);
static PidModbusComponent pid_cool1("pid_cool1", MODBUS_CONFIG.pid_cool1_id, g_mb);

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
  SPI.begin(BOARD_PINS.w5500_sck, BOARD_PINS.w5500_miso, BOARD_PINS.w5500_mosi);

  // Extra hard reset pulse (helps with "w5500_reset(): reset timeout" cases)
#if (BOARD_PINS.w5500_rst >= 0)
  pinMode(BOARD_PINS.w5500_rst, OUTPUT);
  digitalWrite(BOARD_PINS.w5500_rst, LOW);
  delay(50);
  digitalWrite(BOARD_PINS.w5500_rst, HIGH);
  delay(150);
#endif

  // phy_addr is typically 1 in Arduino-ESP32 W5500 examples.
  bool ok = ETH.begin(ETH_PHY_W5500, /*phy_addr*/ 1,
                      BOARD_PINS.w5500_cs, BOARD_PINS.w5500_int, BOARD_PINS.w5500_rst,
                      SPI);

  if (!ok) {
    Serial.println("[eth] ETH.begin() failed (W5500 init) â€” continuing without network");
    g_eth_connected = false;
    return false;
  }

  const bool static_requested =
    (NET_DEFAULTS.static_ip.a | NET_DEFAULTS.static_ip.b | NET_DEFAULTS.static_ip.c | NET_DEFAULTS.static_ip.d) != 0;
  if (static_requested) {
    IPAddress local(NET_DEFAULTS.static_ip.a, NET_DEFAULTS.static_ip.b, NET_DEFAULTS.static_ip.c, NET_DEFAULTS.static_ip.d);
    IPAddress gateway(NET_DEFAULTS.static_gw.a, NET_DEFAULTS.static_gw.b, NET_DEFAULTS.static_gw.c, NET_DEFAULTS.static_gw.d);
    IPAddress subnet(NET_DEFAULTS.static_mask.a, NET_DEFAULTS.static_mask.b, NET_DEFAULTS.static_mask.c, NET_DEFAULTS.static_mask.d);
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
  g_mqtt.setServer(NET_DEFAULTS.mqtt_broker_host, NET_DEFAULTS.mqtt_broker_port);

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

  const String willTopic = mqtt_full_topic(NET_TOPICS.lwt);
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
  g_bus.publish_json(NET_TOPICS.lwt, doc, /*retained*/ true, /*qos*/ 1);
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

  g_bus.publish_json(NET_TOPICS.boot, doc, /*retained*/ true);
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
  Serial1.begin(MODBUS_CONFIG.baud, SERIAL_8N1, MODBUS_CONFIG.rx_pin, MODBUS_CONFIG.tx_pin);
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

    g_bus.publish_json(NET_TOPICS.health, doc, /*retained*/ false);
  }

  delay(5);
}
