#pragma once

#include <stdint.h>

#include "core/health_manager.h"
#include "waveshare_hal/relay.h"

class RelayComponent : public IHealthComponent {
public:
  explicit RelayComponent(RelayHal& hal);

  const char* name() const override { return "relay"; }
  void configure(bool expected, bool required) override;
  bool probe(uint32_t now_ms) override;
  bool tick(uint32_t now_ms) override;

  uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
  HealthReport report() const override { return rep_; }

  uint8_t mask() const { return mask_; }
  bool set_mask(uint8_t mask, uint32_t now_ms);
  bool refresh_state(uint32_t now_ms);

private:
  void set_error(uint32_t now_ms, const char* reason, HealthStatus status = HealthStatus::ERROR);

  RelayHal& hal_;
  HealthReport rep_ {};
  uint8_t mask_ {0};
  uint32_t stale_timeout_ms_ {1000};
  bool initialized_ {false};
};
