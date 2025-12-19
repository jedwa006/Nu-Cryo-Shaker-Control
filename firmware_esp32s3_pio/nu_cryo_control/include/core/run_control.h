#pragma once

#include <stdint.h>

#include "components/din.h"
#include "core/health_manager.h"

enum class RunState : uint8_t {
  STOPPED = 0,
  RUNNING = 1,
  HOLDING = 2,
  ESTOP = 3
};

enum class RunCommand : uint8_t {
  START = 0,
  STOP = 1,
  HOLD = 2,
  RESET = 3
};

struct RunStatus {
  RunState state {RunState::STOPPED};
  const char* reason {"operator_stop"};
  bool run_allowed {false};
  bool outputs_allowed {false};
};

inline const char* to_str(RunState state) {
  switch (state) {
    case RunState::STOPPED: return "STOPPED";
    case RunState::RUNNING: return "RUNNING";
    case RunState::HOLDING: return "HOLDING";
    case RunState::ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

class RunControl {
public:
  RunControl() = default;

  void update(const HealthManager& health, const DinComponent& din, uint32_t now_ms);
  bool handle_command(RunCommand cmd, const HealthManager& health, const DinComponent& din,
                      uint32_t now_ms, const char** err);

  const RunStatus& status() const { return status_; }
  bool run_allowed() const { return status_.run_allowed; }
  bool outputs_allowed() const { return status_.outputs_allowed; }

private:
  void recompute_status(const HealthManager& health, const DinComponent& din);

  RunState desired_state_ {RunState::STOPPED};
  bool estop_latched_ {false};
  bool health_fault_latched_ {false};
  const char* last_estop_reason_ {nullptr};
  RunStatus status_ {};
};
