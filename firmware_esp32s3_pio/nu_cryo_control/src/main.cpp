#include <Arduino.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "app/app_config.h"
#include "components/eth_health.h"
#include "core/fieldbus_service.h"
#include "core/health_manager.h"
#include "core/health_registry.h"
#include "core/mqtt_bus.h"
#include "core/network_manager.h"

// -------------------------------------------------------------------------------------------------
// Core objects
// -------------------------------------------------------------------------------------------------
static NetworkManager g_network;
static PubSubClient g_mqtt(g_network.client());
static MqttBus g_bus;
static HealthManager g_health;
static HealthRegistry g_health_registry(g_health);
static EthHealthComponent g_eth_health(g_network);
static FieldbusService g_fieldbus;

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
  doc["eth"] = g_network.connected();
  doc["eth_static"] = g_network.using_static_ip();

  if (g_network.connected()) {
    doc["ip"] = g_network.local_ip().toString();
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
  g_health_registry.register_component(g_eth_health, /*expected*/ true, /*required*/ true);

  g_fieldbus.begin();
  g_fieldbus.register_components(g_health_registry);

  // Bring up Ethernet (W5500)
  g_network.begin();
}

void loop()
{
  const uint32_t now_ms = millis();

  // Tick components (HealthManager only evaluates; it doesn't call tick())
  static uint32_t last_eth_tick = 0;
  if (now_ms - last_eth_tick >= 250) {
    last_eth_tick = now_ms;
    g_eth_health.tick(now_ms);
  }

  g_fieldbus.tick(now_ms);

  // MQTT state machine (only attempt connect when ETH is up)
  if (g_network.connected()) {
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
