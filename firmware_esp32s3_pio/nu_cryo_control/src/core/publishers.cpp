#include "core/publishers.h"

using namespace publishers;

void publishers::publish_heartbeat(MqttBus& bus, uint32_t now_ms, uint32_t uptime_s) {
  DynamicJsonDocument doc(384);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;
  doc["uptime_s"] = uptime_s;
  bus.publish_json("sys/heartbeat", doc, false, 0);
}

void publishers::publish_component_health(MqttBus& bus, const IHealthComponent& c, uint32_t now_ms) {
  const HealthReport r = c.report();

  DynamicJsonDocument doc(384);
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

void publishers::publish_system_health(MqttBus& bus, const SystemHealth& sh, const RunStatus& run,
                                       uint32_t now_ms, const char* subtopic) {
  DynamicJsonDocument doc(384);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;

  const char* system_state =
    (sh.system_state == HealthStatus::OK) ? "OK" :
    (sh.system_state == HealthStatus::DEGRADED) ? "DEGRADED" :
    "FAULT";

  doc["system_state"] = system_state;
  doc["degraded"] = sh.degraded;

  doc["run_state"] = ::to_str(run.state);
  doc["run_reason"] = run.reason;

  JsonObject inhibit = doc["inhibit"].to<JsonObject>();
  inhibit["run_allowed"] = run.run_allowed;
  inhibit["outputs_allowed"] = run.outputs_allowed;

  JsonObject summary = doc["summary"].to<JsonObject>();
  summary["warn_count"] = sh.warn_count;
  summary["crit_count"] = sh.crit_count;

  bus.publish_json(subtopic, doc, false, 0);
}

void publishers::publish_pid_state(MqttBus& bus, const char* pid_name, const PidState& st, uint32_t now_ms) {
  DynamicJsonDocument doc(384);
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

void publishers::publish_din_state(MqttBus& bus, const DinSnapshot& din, uint32_t now_ms) {
  DynamicJsonDocument doc(256);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;
  doc["mask"] = din.mask;
  bus.publish_json("io/din/state", doc, false, 0);
}

void publishers::publish_din_event(MqttBus& bus, const DinSnapshot& din, uint8_t prev_mask, uint32_t now_ms) {
  DynamicJsonDocument doc(256);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;
  doc["mask"] = din.mask;
  doc["prev_mask"] = prev_mask;
  doc["rising"] = din.rising;
  doc["falling"] = din.falling;
  bus.publish_json("io/din/event", doc, false, 0);
}

void publishers::publish_dout_state(MqttBus& bus, uint8_t mask, bool outputs_allowed, uint32_t now_ms) {
  DynamicJsonDocument doc(256);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;
  doc["mask"] = mask;
  doc["outputs_allowed"] = outputs_allowed;
  bus.publish_json("io/dout/state", doc, false, 0);
}

void publishers::publish_dout_ack(MqttBus& bus, uint32_t now_ms, uint32_t cmd_id, bool ok, const char* err, uint8_t mask, bool outputs_allowed) {
  DynamicJsonDocument doc(256);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;
  doc["cmd_id"] = cmd_id;
  doc["ok"] = ok;
  doc["mask"] = mask;
  doc["outputs_allowed"] = outputs_allowed;
  if (!ok && err) {
    doc["err"] = err;
  }
  bus.publish_json("io/dout/ack", doc, false, 0);
}

void publishers::publish_run_ack(MqttBus& bus, uint32_t now_ms, uint32_t cmd_id, bool ok, const RunStatus& run, const char* err) {
  DynamicJsonDocument doc(256);
  doc["v"] = NUCRYO_SCHEMA_V;
  doc["ts_ms"] = now_ms;
  doc["src"] = NODE_ID;
  doc["cmd_id"] = cmd_id;
  doc["ok"] = ok;
  doc["state"] = ::to_str(run.state);
  doc["reason"] = run.reason;
  doc["run_allowed"] = run.run_allowed;
  doc["outputs_allowed"] = run.outputs_allowed;
  if (!ok && err) {
    doc["err"] = err;
  }
  bus.publish_json("run/ack", doc, false, 0);
}
