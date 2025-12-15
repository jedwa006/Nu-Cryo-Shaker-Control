\
    #pragma once
    #include "core/health_manager.h"
    #include <ModbusRTU.h>

    struct PidState {
      float pv {0};
      float sv {0};
      float out_pct {0};
      bool valid {false};
    };

    class PidModbusComponent : public IHealthComponent {
    public:
      PidModbusComponent(const char* name, uint8_t slave_id, ModbusRTU& mb);

      const char* name() const override { return name_; }
      void configure(bool expected, bool required) override;

      bool probe(uint32_t now_ms) override;
      bool tick(uint32_t now_ms) override;

      uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
      HealthReport report() const override { return rep_; }

      const PidState& state() const { return state_; }
      bool set_sv(float sv, uint32_t now_ms);

    private:
      const char* name_;
      uint8_t slave_id_;
      ModbusRTU& mb_;
      PidState state_ {};
      HealthReport rep_ {};

      uint32_t stale_timeout_ms_ {1500};

      // TODO: adapt to your controller scaling.
      bool read_live(float& pv, float& sv, float& out);
    };
