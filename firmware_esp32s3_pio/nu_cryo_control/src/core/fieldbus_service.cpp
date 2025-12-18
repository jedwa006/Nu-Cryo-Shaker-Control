#include "core/fieldbus_service.h"

#include <HardwareSerial.h>

FieldbusService::FieldbusService()
#if NUCRYO_USE_MODBUS_RTU
  : pid_heat1_("pid_heat1", MODBUS_CONFIG.pid_heat1_id, mb_),
    pid_heat2_("pid_heat2", MODBUS_CONFIG.pid_heat2_id, mb_),
    pid_cool1_("pid_cool1", MODBUS_CONFIG.pid_cool1_id, mb_)
#endif
{
}

bool FieldbusService::begin() {
#if NUCRYO_USE_MODBUS_RTU
  Serial1.begin(MODBUS_CONFIG.baud, SERIAL_8N1, MODBUS_CONFIG.rx_pin, MODBUS_CONFIG.tx_pin);
  mb_.begin(&Serial1);
  mb_.master();
  enabled_ = true;
  return true;
#else
  return false;
#endif
}

void FieldbusService::register_components(HealthRegistry& registry) {
#if NUCRYO_USE_MODBUS_RTU
  registry.register_component(pid_heat1_, /*expected*/ true, /*required*/ true);
  registry.register_component(pid_heat2_, /*expected*/ true, /*required*/ true);
  registry.register_component(pid_cool1_, /*expected*/ true, /*required*/ true);
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
  pid_heat1_.tick(now_ms);
  pid_heat2_.tick(now_ms);
  pid_cool1_.tick(now_ms);
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
