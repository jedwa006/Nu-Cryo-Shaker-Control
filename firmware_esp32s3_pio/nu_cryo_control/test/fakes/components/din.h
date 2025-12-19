#pragma once

#include <stdbool.h>

#include "core/health_manager.h"

class DinComponent {
public:
  DinComponent() = default;

  bool interlocks_ok() const { return interlocks_ok_; }
  void set_interlocks_ok(bool ok) { interlocks_ok_ = ok; }

  const HealthReport& report() const { return report_; }
  void set_reason(const char* reason) { report_.reason = reason; }

private:
  bool interlocks_ok_ {true};
  HealthReport report_ {};
};
