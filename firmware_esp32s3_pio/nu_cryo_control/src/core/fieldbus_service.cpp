#include "core/fieldbus_service.h"

#include <HardwareSerial.h>

FieldbusService::FieldbusService()
#if NUCRYO_USE_MODBUS_RTU
  : pid_heat1_("pid_heat1", MODBUS_CONFIG.pid_heat1_id, mb_),
    pid_heat2_("pid_heat2", MODBUS_CONFIG.pid_heat2_id, mb_),
    pid_cool1_("pid_cool1", MODBUS_CONFIG.pid_cool1_id, mb_),
    pids_ {{&pid_heat1_, &pid_heat2_, &pid_cool1_}}
#endif
{
}

bool FieldbusService::begin() {
#if NUCRYO_USE_MODBUS_RTU
  Serial1.begin(MODBUS_CONFIG.baud, SERIAL_8N1, MODBUS_CONFIG.rx_pin, MODBUS_CONFIG.tx_pin);
  mb_.begin(&Serial1, MODBUS_CONFIG.de_re_pin);
  mb_.master();
  enabled_ = true;
  const uint32_t now_ms = millis();
  for (PidModbusComponent* pid : pids_) {
    pid->probe(now_ms);
  }
  return true;
#else
  return false;
#endif
}

void FieldbusService::register_components(HealthRegistry& registry) {
#if NUCRYO_USE_MODBUS_RTU
  const bool expected[] = {MODBUS_CONFIG.pid_heat1_expected, MODBUS_CONFIG.pid_heat2_expected,
                           MODBUS_CONFIG.pid_cool1_expected};
  const bool required[] = {MODBUS_CONFIG.pid_heat1_required, MODBUS_CONFIG.pid_heat2_required,
                           MODBUS_CONFIG.pid_cool1_required};
  for (size_t i = 0; i < pids_.size(); ++i) {
    registry.register_component(*pids_[i], expected[i], required[i]);
  }
#else
  (void)registry;
#endif
}

void FieldbusService::tick(uint32_t now_ms) {
#if NUCRYO_USE_MODBUS_RTU
  if (!enabled_) return;

  if (now_ms - last_pid_tick_ms_ < PID_STATE_PERIOD_MS) return;
  last_pid_tick_ms_ = now_ms;

  mb_.task();
  const size_t kPidCount = pids_.size();
  const size_t scheduled_index = next_pid_index_ % kPidCount;
  const size_t scheduled_params_index = next_pid_params_index_ % kPidCount;
  for (PidModbusComponent* pid : pids_) {
    pid->tick(now_ms);
  }

  if (now_ms - last_pid_params_ms_ >= PID_PARAMS_PERIOD_MS) {
    if (pids_[scheduled_params_index]->start_read_params(now_ms)) {
      last_pid_params_ms_ = now_ms;
      next_pid_params_index_ = static_cast<uint8_t>((next_pid_params_index_ + 1) % kPidCount);
      return;
    }
  }

  if (pids_[scheduled_index]->start_read(now_ms)) {
    next_pid_index_ = static_cast<uint8_t>((next_pid_index_ + 1) % kPidCount);
  }
#else
  (void)now_ms;
#endif
}

bool FieldbusService::enabled() const {
#if NUCRYO_USE_MODBUS_RTU
  return enabled_;
#else
  return false;
#endif
}
