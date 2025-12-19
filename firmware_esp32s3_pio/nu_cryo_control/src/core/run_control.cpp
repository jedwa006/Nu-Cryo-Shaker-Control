#include "core/run_control.h"

namespace {
const char* operator_reason(RunState state) {
  switch (state) {
    case RunState::RUNNING: return "operator_start";
    case RunState::HOLDING: return "operator_hold";
    case RunState::STOPPED: return "operator_stop";
    case RunState::ESTOP: return "operator_estop";
    default: return "operator_stop";
  }
}
} // namespace

void RunControl::update(const HealthManager& health, const DinComponent& din, uint32_t now_ms) {
  (void)now_ms;
  if (!din.interlocks_ok()) {
    estop_latched_ = true;
    last_estop_reason_ = din.report().reason;
  }

  if (!health.system_health().run_allowed) {
    health_fault_latched_ = true;
  }

  recompute_status(health, din);
}

bool RunControl::handle_command(RunCommand cmd, const HealthManager& health, const DinComponent& din,
                                uint32_t now_ms, const char** err) {
  update(health, din, now_ms);
  if (err) {
    *err = nullptr;
  }

  if (cmd == RunCommand::RESET) {
    if (estop_latched_ || health_fault_latched_) {
      if (din.interlocks_ok() && health.system_health().run_allowed) {
        estop_latched_ = false;
        health_fault_latched_ = false;
        desired_state_ = RunState::STOPPED;
        recompute_status(health, din);
        return true;
      }
      if (err) {
        *err = "reset_inhibited";
      }
      recompute_status(health, din);
      return false;
    }
    return true;
  }

  if (estop_latched_ || health_fault_latched_) {
    if (err) {
      *err = "inhibited";
    }
    return false;
  }

  switch (cmd) {
    case RunCommand::START:
      if (!din.interlocks_ok() || !health.system_health().run_allowed) {
        if (err) {
          *err = "inhibited";
        }
        return false;
      }
      desired_state_ = RunState::RUNNING;
      break;
    case RunCommand::HOLD:
      desired_state_ = RunState::HOLDING;
      break;
    case RunCommand::STOP:
      desired_state_ = RunState::STOPPED;
      break;
    case RunCommand::RESET:
      break;
  }

  recompute_status(health, din);
  return true;
}

void RunControl::recompute_status(const HealthManager& health, const DinComponent& din) {
  RunState effective_state = desired_state_;
  const char* reason = operator_reason(desired_state_);

  if (estop_latched_) {
    effective_state = RunState::ESTOP;
    reason = din.interlocks_ok() ? "estop_latched" : (last_estop_reason_ ? last_estop_reason_ : "estop");
  } else if (health_fault_latched_ || !health.system_health().run_allowed) {
    effective_state = RunState::STOPPED;
    reason = "health_fault";
  }

  status_.state = effective_state;
  status_.reason = reason;
  status_.run_allowed = (effective_state == RunState::RUNNING);
  status_.outputs_allowed = (effective_state == RunState::RUNNING || effective_state == RunState::HOLDING);
}
