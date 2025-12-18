#pragma once

#include "core/health_manager.h"

class HealthRegistry {
public:
  explicit HealthRegistry(HealthManager& manager) : manager_(manager) {}

  void register_component(IHealthComponent& component, bool expected, bool required) {
    component.configure(expected, required);
    manager_.add(&component);
  }

private:
  HealthManager& manager_;
};

