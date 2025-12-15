\
    #include "publishers.h"

    using namespace publishers;

    void publishers::publish_heartbeat(MqttBus& bus, uint32_t now_ms, uint32_t uptime_s) {
      StaticJsonDocument<384> doc;
      doc["v"] = NUCRYO_SCHEMA_V;
      doc["ts_ms"] = now_ms;
      doc["src"] = NODE_ID;
      doc["uptime_s"] = uptime_s;
      bus.publish_json("sys/heartbeat", doc, false, 0);
    }

    void publishers::publish_component_health(MqttBus& bus, const IHealthComponent& c, uint32_t now_ms) {
      const HealthReport r = c.report();

      StaticJsonDocument<384> doc;
      doc["v"] = NUCRYO_SCHEMA_V;
      doc["ts_ms"] = now_ms;
      doc["src"] = NODE_ID;

      doc["component"] = c.name();
      doc["status"] = to_str(r.status);
      doc["severity"] = to_str(r.severity);
      doc["expected"] = r.expected;
      doc["required"] = r.required;
      doc["reason"] = r.reason;
      doc["since_ms"] = r.since_ms;
      doc["last_ok_ms"] = r.last_ok_ms;

      char subtopic[128];
      snprintf(subtopic, sizeof(subtopic), "health/%s/state", c.name());
      bus.publish_json(subtopic, doc, false, 0);
    }

    void publishers::publish_system_health(MqttBus& bus, const SystemHealth& sh, uint32_t now_ms) {
      StaticJsonDocument<384> doc;
      doc["v"] = NUCRYO_SCHEMA_V;
      doc["ts_ms"] = now_ms;
      doc["src"] = NODE_ID;

      // Map enum to a higher-level name; you can refine this later.
      const char* system_state =
        (sh.system_state == HealthStatus::OK) ? "OK" :
        (sh.system_state == HealthStatus::DEGRADED) ? "DEGRADED" :
        "FAULT";

      doc["system_state"] = system_state;
      doc["degraded"] = sh.degraded;

      JsonObject inhibit = doc["inhibit"].to<JsonObject>();
      inhibit["run_allowed"] = sh.run_allowed;
      inhibit["outputs_allowed"] = sh.outputs_allowed;

      JsonObject summary = doc["summary"].to<JsonObject>();
      summary["warn_count"] = sh.warn_count;
      summary["crit_count"] = sh.crit_count;

      bus.publish_json("sys/health", doc, false, 0);
    }

    void publishers::publish_pid_state(MqttBus& bus, const char* pid_name, const PidState& st, uint32_t now_ms) {
      StaticJsonDocument<384> doc;
      doc["v"] = NUCRYO_SCHEMA_V;
      doc["ts_ms"] = now_ms;
      doc["src"] = NODE_ID;

      doc["pv"] = st.pv;
      doc["sv"] = st.sv;
      doc["out_pct"] = st.out_pct;
      doc["valid"] = st.valid;

      char subtopic[128];
      snprintf(subtopic, sizeof(subtopic), "pid/%s/state", pid_name);
      bus.publish_json(subtopic, doc, false, 0);
    }
