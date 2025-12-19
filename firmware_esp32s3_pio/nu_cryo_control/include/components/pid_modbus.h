    #pragma once
    #include "app/app_config.h"
    #include "core/health_manager.h"
    #include <ModbusRTU.h>

    struct PidState {
      float pv {0};
      float sv {0};
      float out_pct {0};
      uint16_t status {0};
      uint16_t alarm1 {0};
      uint16_t alarm2 {0};
      bool alarm_active {false};
      bool valid {false};
    };

    struct PidParams {
      float p {0};
      float i {0};
      float d {0};
      float output_min {0};
      float output_max {0};
      float sv_min {0};
      float sv_max {0};
      bool valid {false};
    };

    class PidModbusComponent : public IHealthComponent {
    public:
      PidModbusComponent(const char* name, uint8_t slave_id, ModbusRTU& mb);

      const char* name() const override { return name_; }
      void configure(bool expected, bool required) override;

      bool probe(uint32_t now_ms) override;
      bool tick(uint32_t now_ms) override;
      bool start_read(uint32_t now_ms);
      bool start_read_params(uint32_t now_ms);

      uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
      HealthReport report() const override { return rep_; }

      const PidState& state() const { return state_; }
      const PidParams& params() const { return params_; }
      bool set_sv(float sv, uint32_t now_ms);

    private:
      static constexpr bool kHasAlarmRegs =
        (REG_ALARM1 > 0 && REG_ALARM2 >= REG_ALARM1 && REG_ALARM1 >= REG_PV);
      static constexpr uint16_t kRegBase = REG_PV;
      static constexpr uint16_t kRegCount =
        kHasAlarmRegs ? static_cast<uint16_t>(REG_ALARM2 - REG_PV + 1)
                      : static_cast<uint16_t>(REG_SV - REG_PV + 1);
      static constexpr uint16_t kParamsRegBase = REG_P1;
      static constexpr uint16_t kParamsRegCount =
        static_cast<uint16_t>(REG_USPL - REG_P1 + 1);

      const char* name_;
      uint8_t slave_id_;
      ModbusRTU& mb_;
      PidState state_ {};
      PidParams params_ {};
      HealthReport rep_ {};

      uint32_t stale_timeout_ms_ {1500};

      uint16_t regs_[kRegCount] {};
      uint16_t params_regs_[kParamsRegCount] {};
      bool request_in_flight_ {false};
      uint32_t last_request_ms_ {0};

      enum class RequestType : uint8_t { None, State, Params };
      struct PendingResult {
        RequestType type {RequestType::None};
        bool ok {false};
      };
      PendingResult pending_result_ {};
      RequestType request_type_ {RequestType::None};

      static PidModbusComponent* active_request_;
      static bool on_transaction(Modbus::ResultCode result, uint16_t, void*);
      void handle_transaction(Modbus::ResultCode result);
    };
