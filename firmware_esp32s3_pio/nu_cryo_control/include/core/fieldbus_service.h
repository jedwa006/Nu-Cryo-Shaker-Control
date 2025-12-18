#pragma once

#include <Arduino.h>

#include "app/app_config.h"
#include "components/pid_modbus.h"
#include "core/health_registry.h"

class FieldbusService {
public:
  FieldbusService();

  bool begin();
  void register_components(HealthRegistry& registry);
  void tick(uint32_t now_ms);

  bool enabled() const;

#if NUCRYO_USE_MODBUS_RTU
  ModbusRTU& modbus() { return mb_; }
  PidModbusComponent& pid_heat1() { return pid_heat1_; }
  PidModbusComponent& pid_heat2() { return pid_heat2_; }
  PidModbusComponent& pid_cool1() { return pid_cool1_; }
#endif

private:
#if NUCRYO_USE_MODBUS_RTU
  ModbusRTU mb_;
  PidModbusComponent pid_heat1_;
  PidModbusComponent pid_heat2_;
  PidModbusComponent pid_cool1_;
  bool enabled_ {false};
  uint32_t last_pid_tick_ms_ {0};
#endif
};

