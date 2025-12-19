\
    #pragma once
    #include "app/app_config.h"
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
      bool tick(uint32_t now_ms) override { return tick(now_ms, true); }
      bool tick(uint32_t now_ms, bool scheduled);

      uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
      HealthReport report() const override { return rep_; }

      const PidState& state() const { return state_; }
      bool set_sv(float sv, uint32_t now_ms);

    private:
      static constexpr uint16_t kRegBase = REG_PV;
      static constexpr uint16_t kRegCount = (REG_SV - REG_PV + 1);

      const char* name_;
      uint8_t slave_id_;
      ModbusRTU& mb_;
      PidState state_ {};
      HealthReport rep_ {};

      uint32_t stale_timeout_ms_ {1500};

      uint16_t regs_[kRegCount] {};
      bool request_in_flight_ {false};
      uint32_t last_request_ms_ {0};

      enum class PendingResult : uint8_t { None, Ok, Fail };
      PendingResult pending_result_ {PendingResult::None};

      static PidModbusComponent* active_request_;
      static bool on_transaction(Modbus::ResultCode result, uint16_t, void*);

      bool start_read(uint32_t now_ms);
      void handle_transaction(Modbus::ResultCode result);
    };
