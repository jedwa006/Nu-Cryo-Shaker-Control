#pragma once

#include <stdint.h>

#include "core/health_manager.h"
#include "waveshare_hal/din.h"

struct DinSnapshot {
  uint8_t mask {0};
  uint8_t rising {0};
  uint8_t falling {0};
};

// Digital input health + state component. Maps DIN1..DIN3 to estop/lid/door
// interlocks and exposes an 8-bit mask for other consumers.
class DinComponent : public IHealthComponent {
public:
  explicit DinComponent(DinHal& hal);

  const char* name() const override { return "din"; }
  void configure(bool expected, bool required) override;
  bool probe(uint32_t now_ms) override;
  bool tick(uint32_t now_ms) override;

  uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
  HealthReport report() const override { return rep_; }

  const DinSnapshot& snapshot() const { return snapshot_; }

  bool interlocks_ok() const;

  static constexpr uint8_t BIT_ESTOP_OK = 0;
  static constexpr uint8_t BIT_LID_LOCKED = 1;
  static constexpr uint8_t BIT_DOOR_CLOSED = 2;

private:
  void update_health(uint32_t now_ms);

  DinHal& hal_;
  HealthReport rep_ {};
  DinSnapshot snapshot_ {};
  uint32_t stale_timeout_ms_ {1000};
  bool initialized_ {false};
};
