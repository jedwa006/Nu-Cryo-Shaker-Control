#include "core/io_service.h"

#include <Arduino.h>
#include <cstring>

#include "app/app_config.h"
#include "core/publishers.h"

IoService::IoService(MqttBus& bus, HealthManager& health, RunControl& run_control)
  : bus_(bus), health_(health), run_control_(run_control), din_(din_hal_), relay_(relay_hal_) {}

void IoService::begin() {
}

void IoService::on_mqtt_connected() {
  bus_.subscribe("io/cmd/event");
  bus_.subscribe("run/cmd");
}

void IoService::register_components(HealthRegistry& registry) {
  registry.register_component(din_, /*expected*/ true, /*required*/ true);
  registry.register_component(relay_, /*expected*/ true, /*required*/ true);
}

const char* IoService::subtopic_from_root(const char* topic) const {
  const char* root = bus_.root();
  const size_t root_len = strlen(root);
  if (strncmp(topic, root, root_len) != 0) return nullptr;

  const char* sub = topic + root_len;
  if (*sub == '/') ++sub;
  return sub;
}

void IoService::handle_message(const char* topic, const uint8_t* payload, size_t len) {
  const char* subtopic = subtopic_from_root(topic);
  if (!subtopic) return;

  DynamicJsonDocument doc(256);
  const DeserializationError err = deserializeJson(doc, payload, len);
  if (err) return;

  const uint32_t cmd_id = doc["cmd_id"] | 0;
  const uint32_t now_ms = millis();
  if (strcmp(subtopic, "io/cmd/event") == 0) {
    handle_command(doc, now_ms, cmd_id);
  } else if (strcmp(subtopic, "run/cmd") == 0) {
    handle_run_command(doc, now_ms, cmd_id);
  }
}

bool IoService::outputs_allowed() const {
  return run_control_.outputs_allowed();
}

bool IoService::handle_command(const JsonDocument& doc, uint32_t now_ms, uint32_t cmd_id) {
  const bool outputs_ok = outputs_allowed();
  if (!outputs_ok) {
    publishers::publish_dout_ack(bus_, now_ms, cmd_id, false, "outputs_inhibited", relay_.mask(), outputs_ok);
    return false;
  }

  if (doc["mask"].is<uint8_t>()) {
    const uint8_t mask = doc["mask"];
    const bool ok = apply_mask(mask, now_ms);
    publishers::publish_dout_ack(bus_, now_ms, cmd_id, ok, ok ? nullptr : "write_fail", relay_.mask(), outputs_ok);
    return ok;
  }

  if (doc["channel"].is<uint8_t>()) {
    const uint8_t ch = doc["channel"];
    const bool state = doc["state"] | true;
    const bool ok = apply_channel(ch, state, now_ms);
    const char* err = ok ? nullptr : "invalid_channel_or_write_fail";
    publishers::publish_dout_ack(bus_, now_ms, cmd_id, ok, err, relay_.mask(), outputs_ok);
    return ok;
  }

  publishers::publish_dout_ack(bus_, now_ms, cmd_id, false, "invalid_payload", relay_.mask(), outputs_ok);
  return false;
}

bool IoService::handle_run_command(const JsonDocument& doc, uint32_t now_ms, uint32_t cmd_id) {
  const char* cmd_str = doc["cmd"] | "";
  RunCommand cmd;
  if (strcmp(cmd_str, "start") == 0) {
    cmd = RunCommand::START;
  } else if (strcmp(cmd_str, "stop") == 0) {
    cmd = RunCommand::STOP;
  } else if (strcmp(cmd_str, "hold") == 0) {
    cmd = RunCommand::HOLD;
  } else if (strcmp(cmd_str, "reset") == 0) {
    cmd = RunCommand::RESET;
  } else {
    publishers::publish_run_ack(bus_, now_ms, cmd_id, false, run_control_.status(), "invalid_cmd");
    return false;
  }

  const char* err = nullptr;
  const bool ok = run_control_.handle_command(cmd, health_, din_, now_ms, &err);
  publishers::publish_run_ack(bus_, now_ms, cmd_id, ok, run_control_.status(), err);
  return ok;
}

bool IoService::apply_mask(uint8_t mask, uint32_t now_ms) {
  return relay_.set_mask(mask, now_ms);
}

bool IoService::apply_channel(uint8_t channel, bool state, uint32_t now_ms) {
  if (channel == 0 || channel > 8) return false;
  uint8_t mask = relay_.mask();
  if (state) {
    mask |= (1u << (channel - 1));
  } else {
    mask &= ~(1u << (channel - 1));
  }
  return apply_mask(mask, now_ms);
}

void IoService::tick(uint32_t now_ms) {
  if (now_ms - last_io_tick_ms_ < IO_STATE_PERIOD_MS) return;
  last_io_tick_ms_ = now_ms;

  const uint8_t prev_din_mask = din_.snapshot().mask;
  din_.tick(now_ms);
  const DinSnapshot& snap = din_.snapshot();

  if (snap.mask != prev_din_mask || snap.rising || snap.falling) {
    publishers::publish_din_event(bus_, snap, prev_din_mask, now_ms);
  }
  publishers::publish_din_state(bus_, snap, now_ms);

  relay_.tick(now_ms);
  publishers::publish_dout_state(bus_, relay_.mask(), outputs_allowed(), now_ms);
}
