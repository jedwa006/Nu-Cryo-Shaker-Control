#include "components/pid_modbus.h"
#include "app/app_config.h"

    PidModbusComponent::PidModbusComponent(const char* name, uint8_t slave_id, ModbusRTU& mb)
      : name_(name), slave_id_(slave_id), mb_(mb) {}

    void PidModbusComponent::configure(bool expected, bool required) {
      rep_.expected = expected;
      rep_.required = required;
      rep_.status = expected ? HealthStatus::MISSING : HealthStatus::UNCONFIGURED;
      rep_.severity = required ? Severity::CRIT : Severity::INFO;
      rep_.reason = expected ? "not_probed" : "unconfigured";
      rep_.since_ms = 0;
      rep_.last_ok_ms = 0;
    }

    PidModbusComponent* PidModbusComponent::active_request_ = nullptr;

    bool PidModbusComponent::probe(uint32_t now_ms) {
      if (!rep_.expected) return false;
      if (start_read(now_ms)) {
        rep_.status = HealthStatus::MISSING;
        rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
        rep_.reason = "modbus_request_pending";
        if (rep_.since_ms == 0) rep_.since_ms = now_ms;
      }
      return false;
    }

    namespace {
    uint16_t reg_index(uint16_t reg, uint16_t base) {
      return static_cast<uint16_t>(reg - base);
    }

    uint16_t modbus_addr(uint16_t one_based) {
      return (one_based > 0) ? static_cast<uint16_t>(one_based - 1) : 0;
    }
    } // namespace

    bool PidModbusComponent::tick(uint32_t now_ms) {
      if (!rep_.expected) return false;

      bool refreshed = false;
      if (pending_result_.type != RequestType::None) {
        const RequestType handled_type = pending_result_.type;
        const bool ok = pending_result_.ok;
        pending_result_ = {};

        if (handled_type == RequestType::State) {
          if (ok) {
            rep_.status = HealthStatus::OK;
            rep_.severity = Severity::INFO;
            rep_.reason = "ok";
            rep_.last_ok_ms = now_ms;
            state_.valid = true;
            state_.pv = LC108::decode_temp(static_cast<int16_t>(regs_[reg_index(REG_PV, kRegBase)]));
            state_.out_pct =
              LC108::decode_percent(static_cast<int16_t>(regs_[reg_index(REG_OUT_PCT, kRegBase)]));
            state_.sv = LC108::decode_temp(static_cast<int16_t>(regs_[reg_index(REG_SV, kRegBase)]));
            if (REG_STATUS >= kRegBase && REG_STATUS < (kRegBase + kRegCount)) {
              state_.status = regs_[reg_index(REG_STATUS, kRegBase)];
            }
            if (kHasAlarmRegs) {
              state_.alarm1 = regs_[reg_index(REG_ALARM1, kRegBase)];
              state_.alarm2 = regs_[reg_index(REG_ALARM2, kRegBase)];
              state_.alarm_active = (state_.alarm1 != 0u || state_.alarm2 != 0u);
            }
            refreshed = true;
          } else {
            if (rep_.status == HealthStatus::OK) {
              rep_.status = HealthStatus::DEGRADED;
              rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
              rep_.reason = "modbus_read_fail";
              rep_.since_ms = now_ms;
            } else {
              rep_.status = HealthStatus::MISSING;
              rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
              rep_.reason = "modbus_no_response";
              if (rep_.since_ms == 0) rep_.since_ms = now_ms;
            }
            state_.valid = false;
          }
        } else if (handled_type == RequestType::Params) {
          if (ok) {
            params_.valid = true;
            params_.p = static_cast<float>(static_cast<int16_t>(params_regs_[reg_index(REG_P1, kParamsRegBase)]));
            params_.i = static_cast<float>(static_cast<int16_t>(params_regs_[reg_index(REG_I1, kParamsRegBase)]));
            params_.d = static_cast<float>(static_cast<int16_t>(params_regs_[reg_index(REG_D1, kParamsRegBase)]));
            params_.output_min = LC108::decode_percent(
              static_cast<int16_t>(params_regs_[reg_index(REG_OPL1, kParamsRegBase)]));
            params_.output_max = LC108::decode_percent(
              static_cast<int16_t>(params_regs_[reg_index(REG_OPH1, kParamsRegBase)]));
            params_.sv_min = LC108::decode_temp(
              static_cast<int16_t>(params_regs_[reg_index(REG_LSPL, kParamsRegBase)]));
            params_.sv_max = LC108::decode_temp(
              static_cast<int16_t>(params_regs_[reg_index(REG_USPL, kParamsRegBase)]));
          } else {
            params_.valid = false;
          }
        }
      }
      return refreshed;
    }

    bool PidModbusComponent::on_transaction(Modbus::ResultCode result, uint16_t, void*) {
      if (!active_request_) return true;
      active_request_->handle_transaction(result);
      return true;
    }

    bool PidModbusComponent::start_read(uint32_t now_ms) {
      if (request_in_flight_ || active_request_) return false;
      active_request_ = this;
      request_in_flight_ = true;
      last_request_ms_ = now_ms;
      request_type_ = RequestType::State;
      const uint16_t tx_id = mb_.readHreg(
        slave_id_, modbus_addr(kRegBase), regs_, kRegCount, &PidModbusComponent::on_transaction);
      if (tx_id == 0) {
        request_in_flight_ = false;
        active_request_ = nullptr;
        request_type_ = RequestType::None;
        return false;
      }
      return true;
    }

    bool PidModbusComponent::start_read_params(uint32_t now_ms) {
      if (request_in_flight_ || active_request_) return false;
      active_request_ = this;
      request_in_flight_ = true;
      last_request_ms_ = now_ms;
      request_type_ = RequestType::Params;
      const uint16_t tx_id = mb_.readHreg(
        slave_id_, modbus_addr(kParamsRegBase), params_regs_, kParamsRegCount,
        &PidModbusComponent::on_transaction);
      if (tx_id == 0) {
        request_in_flight_ = false;
        active_request_ = nullptr;
        request_type_ = RequestType::None;
        return false;
      }
      return true;
    }

    void PidModbusComponent::handle_transaction(Modbus::ResultCode result) {
      request_in_flight_ = false;
      active_request_ = nullptr;
      pending_result_.type = request_type_;
      pending_result_.ok = (result == Modbus::EX_SUCCESS);
      request_type_ = RequestType::None;
    }

    bool PidModbusComponent::set_sv(float sv, uint32_t now_ms) {
      if (!rep_.expected) return false;
      const uint16_t sv_raw = static_cast<uint16_t>(LC108::encode_temp(sv));

      const bool ok = mb_.writeHreg(slave_id_, modbus_addr(REG_SV), sv_raw);

      if (ok) {
        rep_.last_ok_ms = now_ms;
        rep_.status = HealthStatus::OK;
        rep_.reason = "ok";
        state_.sv = sv;
      } else {
        rep_.status = HealthStatus::DEGRADED;
        rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
        rep_.reason = "sv_write_fail";
      }
      return ok;
    }
