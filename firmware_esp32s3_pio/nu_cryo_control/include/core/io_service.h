#pragma once

#include <ArduinoJson.h>

#include "core/health_manager.h"
#include "core/health_registry.h"
#include "core/mqtt_bus.h"
#include "core/run_control.h"
#include "components/din.h"
#include "components/relay_component.h"

class IoService {
public:
  IoService(MqttBus& bus, HealthManager& health, RunControl& run_control);

  void begin();
  void on_mqtt_connected();
  void register_components(HealthRegistry& registry);
  void handle_message(const char* topic, const uint8_t* payload, size_t len);
  void tick(uint32_t now_ms);

  DinComponent& din() { return din_; }
  RelayComponent& relay() { return relay_; }

private:
  bool handle_command(const JsonDocument& doc, uint32_t now_ms, uint32_t cmd_id);
  bool handle_run_command(const JsonDocument& doc, uint32_t now_ms, uint32_t cmd_id);
  bool apply_mask(uint8_t mask, uint32_t now_ms);
  bool apply_channel(uint8_t channel, bool state, uint32_t now_ms);
  bool outputs_allowed() const;
  const char* subtopic_from_root(const char* topic) const;

  MqttBus& bus_;
  HealthManager& health_;
  RunControl& run_control_;
  DinHal din_hal_;
  RelayHal relay_hal_;
  DinComponent din_;
  RelayComponent relay_;
  uint32_t last_io_tick_ms_ {0};
};
