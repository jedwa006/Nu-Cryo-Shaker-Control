#include "core/io_service.h"

#include <Arduino.h>
#include <cstring>

#include "app/app_config.h"
#include "core/publishers.h"

IoService::IoService(MqttBus& bus, HealthManager& health)
  : bus_(bus), health_(health), din_(din_hal_), relay_(relay_hal_) {}

void IoService::begin() {
}

void IoService::on_mqtt_connected() {
  bus_.subscribe("io/cmd/event");
}

void IoService::register_components(HealthRegistry& registry) {
  registry.register_component(din_, /*expected*/ true, /*required*/ true);
  registry.register_component(relay_, /*expected*/ true, /*required*/ true);
}

bool IoService::topic_matches(const char* topic) const {
  const char* root = bus_.root();
  const size_t root_len = strlen(root);
  if (strncmp(topic, root, root_len) != 0) return false;

  const char* sub = topic + root_len;
  if (*sub == '/') ++sub;
  return strcmp(sub, "io/cmd/event") == 0;
}

void IoService::handle_message(const char* topic, const uint8_t* payload, size_t len) {
  if (!topic_matches(topic)) return;

  DynamicJsonDocument doc(256);
  const DeserializationError err = deserializeJson(doc, payload, len);
  if (err) return;

  const uint32_t cmd_id = doc["cmd_id"] | 0;
  const uint32_t now_ms = millis();
  handle_command(doc, now_ms, cmd_id);
}

bool IoService::outputs_allowed() const {
  const SystemHealth& sh = health_.system_health();
  return sh.outputs_allowed && din_.interlocks_ok();
}

bool IoService::handle_command(const JsonDocument& doc, uint32_t now_ms, uint32_t cmd_id) {
  const bool outputs_ok = outputs_allowed();
  if (!outputs_ok) {
    publishers::publish_dout_ack(bus_, now_ms, cmd_id, false, "outputs_inhibited", relay_.mask(), outputs_ok);
    return false;
  }

  if (doc.containsKey("mask")) {
    const uint8_t mask = doc["mask"];
    const bool ok = apply_mask(mask, now_ms);
    publishers::publish_dout_ack(bus_, now_ms, cmd_id, ok, ok ? nullptr : "write_fail", relay_.mask(), outputs_ok);
    return ok;
  }

  if (doc.containsKey("channel")) {
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
