\
    #pragma once
    #include "core/mqtt_bus.h"
    #include "core/health_manager.h"
    #include "components/pid_modbus.h"
    #include "app/app_config.h"

    namespace publishers {

    inline const char* to_str(HealthStatus s) {
      switch (s) {
        case HealthStatus::UNCONFIGURED: return "UNCONFIGURED";
        case HealthStatus::MISSING:      return "MISSING";
        case HealthStatus::OK:           return "OK";
        case HealthStatus::DEGRADED:     return "DEGRADED";
        case HealthStatus::STALE:        return "STALE";
        case HealthStatus::ERROR:        return "ERROR";
        default: return "UNKNOWN";
      }
    }

    inline const char* to_str(Severity s) {
      switch (s) {
        case Severity::INFO: return "INFO";
        case Severity::WARN: return "WARN";
        case Severity::CRIT: return "CRIT";
        default: return "INFO";
      }
    }

    void publish_heartbeat(MqttBus& bus, uint32_t now_ms, uint32_t uptime_s);
    void publish_component_health(MqttBus& bus, const IHealthComponent& c, uint32_t now_ms);
    void publish_system_health(MqttBus& bus, const SystemHealth& sh, uint32_t now_ms);

    void publish_pid_state(MqttBus& bus, const char* pid_name, const PidState& st, uint32_t now_ms);

    } // namespace publishers
