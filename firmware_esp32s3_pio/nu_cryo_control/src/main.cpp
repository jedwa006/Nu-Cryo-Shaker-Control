\
    #include <Arduino.h>

    #include <SPI.h>
    #if NUCRYO_USE_ETH_W5500
      #include <Ethernet.h>
    #endif

    #include <PubSubClient.h>
    #include <ArduinoJson.h>

    #if NUCRYO_USE_MODBUS_RTU
      #include <HardwareSerial.h>
      #include <ModbusRTU.h>
    #endif

    #include "app/app_config.h"
    #include "core/health_manager.h"
    #include "core/mqtt_bus.h"
    #include "core/publishers.h"

    #include "components/pid_modbus.h"
    #if NUCRYO_ENABLE_ADXL345
      #include "components/adxl345.h"
    #endif

    // ------------ Networking ------------
    #if NUCRYO_USE_ETH_W5500
    static byte mac_addr[6];
    EthernetClient eth_client;
    #endif

    PubSubClient mqtt_client;
    MqttBus bus;

    // ------------ Modbus ------------
    #if NUCRYO_USE_MODBUS_RTU
    HardwareSerial RS485(1);
    ModbusRTU mb;
    #endif

    // ------------ Components ------------
    HealthManager hm;

    #if NUCRYO_USE_MODBUS_RTU
    PidModbusComponent pid_heat1("heat1", PID_HEAT1_ID, mb);
    PidModbusComponent pid_heat2("heat2", PID_HEAT2_ID, mb);
    PidModbusComponent pid_cool1("cool1", PID_COOL1_ID, mb);
    #endif

    #if NUCRYO_ENABLE_ADXL345
    Adxl345Component adxl(Wire, 0x53);
    #endif

    // ------------ Timers ------------
    static uint32_t t_heartbeat=0, t_syshealth=0, t_comphealth=0, t_pid=0;

    static void make_mac_from_efuse(byte out[6]) {
      uint64_t id = ESP.getEfuseMac();
      out[0] = (id >> 40) & 0xFF;
      out[1] = (id >> 32) & 0xFF;
      out[2] = (id >> 24) & 0xFF;
      out[3] = (id >> 16) & 0xFF;
      out[4] = (id >>  8) & 0xFF;
      out[5] = (id >>  0) & 0xFF;
      out[0] |= 0x02; // locally administered
      out[0] &= 0xFE; // unicast
    }

    static void eth_begin() {
    #if NUCRYO_USE_ETH_W5500
      make_mac_from_efuse(mac_addr);
      Ethernet.init(W5500_CS_PIN);

      if (ETH_STATIC_IP.a != 0) {
        IPAddress ip(ETH_STATIC_IP.a, ETH_STATIC_IP.b, ETH_STATIC_IP.c, ETH_STATIC_IP.d);
        IPAddress gw(ETH_STATIC_GW.a, ETH_STATIC_GW.b, ETH_STATIC_GW.c, ETH_STATIC_GW.d);
        IPAddress mask(ETH_STATIC_MASK.a, ETH_STATIC_MASK.b, ETH_STATIC_MASK.c, ETH_STATIC_MASK.d);
        Ethernet.begin(mac_addr, ip, gw, gw, mask);
      } else {
        Ethernet.begin(mac_addr); // DHCP
      }

      delay(200);
    #endif
    }

    static bool mqtt_connect() {
      mqtt_client.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);

      char client_id[48];
      snprintf(client_id, sizeof(client_id), "nucryo-%s-%lu", NODE_ID, (unsigned long)ESP.getEfuseMac());

      bool ok;
      if (strlen(MQTT_USERNAME) > 0) {
        ok = mqtt_client.connect(client_id, MQTT_USERNAME, MQTT_PASSWORD,
                                 (String(MACHINE_ID)+"/"+NODE_ID+"/sys/status").c_str(),
                                 1, true, "offline");
      } else {
        ok = mqtt_client.connect(client_id,
                                 (String(MACHINE_ID)+"/"+NODE_ID+"/sys/status").c_str(),
                                 1, true, "offline");
      }

      if (ok) {
        StaticJsonDocument<128> birth;
        birth["v"] = NUCRYO_SCHEMA_V;
        birth["ts_ms"] = millis();
        birth["src"] = NODE_ID;
        birth["status"] = "online";
        bus.publish_json("sys/birth", birth, true, 1);

        // retained "online" status
        mqtt_client.publish((String(MACHINE_ID)+"/"+NODE_ID+"/sys/status").c_str(),
                            (const uint8_t*)"online", 6, true);

        // Subscribe to PID SV commands (extend later)
        bus.subscribe("pid/+/cmd/#", 1);
      }
      return ok;
    }

    // Minimal command handler: pid/<name>/cmd/sv with {"cmd_id":..., "value":...}
    static void on_mqtt_message(const char* topic, const uint8_t* payload, size_t len) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, payload, len);
      if (err) return;

      const uint32_t cmd_id = doc["cmd_id"] | 0;
      const float value = doc["value"] | 0.0f;

      const uint32_t now_ms = millis();

      // Parse which PID by topic string
      // Expected: "<machine>/<node>/pid/<pid_name>/cmd/sv"
      const String t(topic);
      const String prefix = String(MACHINE_ID) + "/" + NODE_ID + "/pid/";
      if (!t.startsWith(prefix)) return;

      const int p0 = prefix.length();
      const int p1 = t.indexOf('/', p0);
      if (p1 < 0) return;
      const String pid_name = t.substring(p0, p1);

    #if NUCRYO_USE_MODBUS_RTU
      bool ok = false;
      if (pid_name == "heat1") ok = pid_heat1.set_sv(value, now_ms);
      else if (pid_name == "heat2") ok = pid_heat2.set_sv(value, now_ms);
      else if (pid_name == "cool1") ok = pid_cool1.set_sv(value, now_ms);

      StaticJsonDocument<256> ack;
      ack["v"] = NUCRYO_SCHEMA_V;
      ack["ts_ms"] = now_ms;
      ack["src"] = NODE_ID;
      ack["cmd_id"] = cmd_id;
      ack["ok"] = ok;
      ack["err"] = ok ? "" : "write_failed";
      if (ok) ack["applied"]["sv"] = value;

      char subtopic[64];
      snprintf(subtopic, sizeof(subtopic), "pid/%s/ack", pid_name.c_str());
      bus.publish_json(subtopic, ack, false, 1);
    #endif
    }

    void setup() {
      Serial.begin(115200);
      delay(200);

      // --- I2C ---
      Wire.begin();

      // --- Ethernet ---
      eth_begin();

      // --- MQTT ---
    #if NUCRYO_USE_ETH_W5500
      mqtt_client.setClient(eth_client);
    #endif
      bus.begin(mqtt_client, MACHINE_ID, NODE_ID);
      bus.set_handler(on_mqtt_message);

      // --- Modbus RTU ---
    #if NUCRYO_USE_MODBUS_RTU
      RS485.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
      mb.begin(&RS485, MODBUS_DE_RE_PIN);
      mb.master();
    #endif

      // --- Register components with HealthManager ---
    #if NUCRYO_USE_MODBUS_RTU
      pid_heat1.configure(true, true);
      pid_heat2.configure(true, true);
      pid_cool1.configure(true, true);
      hm.add(&pid_heat1);
      hm.add(&pid_heat2);
      hm.add(&pid_cool1);
    #endif

    #if NUCRYO_ENABLE_ADXL345
      adxl.configure(true, false);   // expected optional
      hm.add(&adxl);
    #endif

      // initial probe so UI has immediate visibility
      const uint32_t now_ms = millis();
      for (size_t i=0; i<hm.count(); ++i) {
        auto* c = hm.component(i);
        if (c) c->probe(now_ms);
      }

      // connect MQTT (retry in loop if down)
      mqtt_connect();

      t_heartbeat = t_syshealth = t_comphealth = t_pid = now_ms;
    }

    void loop() {
      const uint32_t now_ms = millis();

      // MQTT reconnect policy (simple). Add exponential backoff later if desired.
      if (!mqtt_client.connected()) {
        mqtt_connect();
      }
      bus.loop();

      // tick components
    #if NUCRYO_USE_MODBUS_RTU
      mb.task();
      pid_heat1.tick(now_ms);
      pid_heat2.tick(now_ms);
      pid_cool1.tick(now_ms);
    #endif

    #if NUCRYO_ENABLE_ADXL345
      adxl.tick(now_ms);
    #endif

      // evaluate aggregate health
      hm.evaluate(now_ms);

      // publish heartbeat
      if ((now_ms - t_heartbeat) >= HEARTBEAT_PERIOD_MS) {
        publishers::publish_heartbeat(bus, now_ms, now_ms/1000);
        t_heartbeat = now_ms;
      }

      // publish system health
      if ((now_ms - t_syshealth) >= SYS_HEALTH_PERIOD_MS) {
        publishers::publish_system_health(bus, hm.system_health(), now_ms);
        t_syshealth = now_ms;
      }

      // publish component health
      if ((now_ms - t_comphealth) >= COMPONENT_HEALTH_PERIOD_MS) {
        for (size_t i=0; i<hm.count(); ++i) {
          auto* c = hm.component(i);
          if (c) publishers::publish_component_health(bus, *c, now_ms);
        }
        t_comphealth = now_ms;
      }

      // publish PID state
      if ((now_ms - t_pid) >= PID_STATE_PERIOD_MS) {
      #if NUCRYO_USE_MODBUS_RTU
        publishers::publish_pid_state(bus, "heat1", pid_heat1.state(), now_ms);
        publishers::publish_pid_state(bus, "heat2", pid_heat2.state(), now_ms);
        publishers::publish_pid_state(bus, "cool1", pid_cool1.state(), now_ms);
      #endif
        t_pid = now_ms;
      }
    }
