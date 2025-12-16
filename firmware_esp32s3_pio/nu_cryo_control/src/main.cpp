\
    #include <Arduino.h>

    #include <SPI.h>
    #include <ETH.h>
    #include <WiFi.h>  // provides WiFiClient (lwIP sockets used by ETH as well)

    #include <PubSubClient.h>
    #include <ArduinoJson.h>

    #include "app/app_config.h"
    #include "core/health_manager.h"
    #include "core/mqtt_bus.h"

    #if NUCRYO_USE_MODBUS_RTU
    #include "components/pid_modbus.h"
    #endif

    #if NUCRYO_ENABLE_ADXL345
    #include "components/adxl345.h"
    #endif

    // --- Networking state ---
    static bool g_eth_connected = false;
    static WiFiClient g_net_client;
    static PubSubClient g_mqtt(g_net_client);

    // --- Core objects ---
    static HealthManager g_health;
    static MqttBus g_bus;

    // --- Components (example set) ---
    #if NUCRYO_USE_MODBUS_RTU
    static PidModbusComponent pid_heat1("heat1");
    static PidModbusComponent pid_heat2("heat2");
    static PidModbusComponent pid_cool1("cool1");
    #endif

    static void on_eth_event(arduino_event_id_t event)
    {
      // WARNING: Network.onEvent is called from another FreeRTOS task.
      switch (event) {
        case ARDUINO_EVENT_ETH_START:
          Serial.println("[eth] start");
          // Must be set after interface is started but before DHCP.
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

        case ARDUINO_EVENT_ETH_LOST_IP:
          Serial.println("[eth] lost ip");
          g_eth_connected = false;
          break;

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

    static void eth_begin()
    {
    #if NUCRYO_USE_ETH_W5500
      Network.onEvent(on_eth_event);

      // W5500 lives on a dedicated SPI pin set on this board.
      SPI.begin(W5500_SCK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN);

      // Bring up W5500 using Arduino-ESP32 lwIP Ethernet
      // phy_addr is typically 1 for W5500 in Arduino-ESP32 examples for SPI PHY.
      ETH.begin(ETH_PHY_W5500, /*phy_addr*/ 1, W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN, SPI);
    #endif
    }

    static bool mqtt_connect(uint32_t now_ms)
    {
      g_mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);

    #if defined(MQTT_MAX_PACKET_SIZE)
      // PubSubClient also supports setBufferSize() at runtime; keep both safe.
      g_mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
    #endif

      // LWT helps the UI detect link loss / resets
      StaticJsonDocument<128> lwt;
      lwt["v"] = NUCRYO_SCHEMA_V;
      lwt["ts_ms"] = now_ms;
      lwt["state"] = "offline";
      char lwt_buf[128];
      size_t n = serializeJson(lwt, lwt_buf, sizeof(lwt_buf));

      return g_mqtt.connect(NODE_ID, MQTT_TOPIC_LWT, /*qos*/ 1, /*retain*/ true, lwt_buf, n);
    }

    static void publish_boot(uint32_t now_ms)
    {
      StaticJsonDocument<192> doc;
      doc["v"] = NUCRYO_SCHEMA_V;
      doc["ts_ms"] = now_ms;
      doc["node"] = NODE_ID;
      doc["fw"] = "nu_cryo_control_pio";
      doc["eth"] = g_eth_connected;
      doc["ip"] = ETH.localIP().toString();

      g_bus.publish_json(MQTT_TOPIC_BOOT, doc, /*retain*/ true);
    }

    void setup()
    {
      Serial.begin(115200);
      delay(50);
      Serial.println("\n[nu-cryo] boot");

      // Configure health policy
      g_health.configure_component("eth", /*expected*/ true, /*required*/ true);

    #if NUCRYO_USE_MODBUS_RTU
      g_health.configure_component("pid_heat1", true, true);
      g_health.configure_component("pid_heat2", true, true);
      g_health.configure_component("pid_cool1", true, true);
    #endif

    #if NUCRYO_ENABLE_ADXL345
      g_health.configure_component("adxl345", true, false); // expected, but optional
    #else
      g_health.configure_component("adxl345", false, false);
    #endif

      eth_begin();

      // Initialize MQTT routing (commands, etc.)
      g_bus.begin(g_mqtt, MACHINE_ID, NODE_ID);
    }

    void loop()
    {
      const uint32_t now_ms = millis();

      // Service MQTT I/O
      if (g_eth_connected) {
        if (!g_mqtt.connected()) {
          static uint32_t last_try = 0;
          if (now_ms - last_try > 2000) {
            last_try = now_ms;
            Serial.println("[mqtt] connecting...");
            if (mqtt_connect(now_ms)) {
              Serial.println("[mqtt] connected");
              publish_boot(now_ms);
              g_bus.publish_online(now_ms);
            } else {
              Serial.print("[mqtt] failed rc=");
              Serial.println(g_mqtt.state());
            }
          }
        } else {
          g_mqtt.loop();
        }
      }

      // Poll / update components (example cadence)
    #if NUCRYO_USE_MODBUS_RTU
      pid_heat1.loop(now_ms, g_health);
      pid_heat2.loop(now_ms, g_health);
      pid_cool1.loop(now_ms, g_health);
    #endif

      // Compute and publish health / system state periodically
      g_health.loop(now_ms);
      g_bus.loop(now_ms, g_health);

      delay(5);
    }
