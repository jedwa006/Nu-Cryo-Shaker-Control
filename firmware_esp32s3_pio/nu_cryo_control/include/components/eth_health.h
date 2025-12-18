#pragma once

#include "core/health_manager.h"
#include "core/network_manager.h"

class EthHealthComponent : public IHealthComponent {
public:
  explicit EthHealthComponent(NetworkManager& network) : network_(network) {}

  const char* name() const override { return "eth"; }

  void configure(bool expected, bool required) override;
  bool probe(uint32_t now_ms) override;
  bool tick(uint32_t now_ms) override;

  uint32_t stale_timeout_ms() const override { return 0; }
  HealthReport report() const override { return rep_; }

private:
  NetworkManager& network_;
  HealthReport rep_ {};
};

