\
#include "components/pid_modbus.h"
    #include "app/app_config.h"
    #include <cstring>

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

    bool PidModbusComponent::tick(uint32_t now_ms, bool scheduled) {
      if (!rep_.expected) return false;

      bool refreshed = false;
      if (pending_result_ == PendingResult::Ok) {
        pending_result_ = PendingResult::None;
        rep_.status = HealthStatus::OK;
        rep_.severity = Severity::INFO;
        rep_.reason = "ok";
        rep_.last_ok_ms = now_ms;
        state_.valid = true;
        state_.pv = LC108::decode_temp(static_cast<int16_t>(regs_[0]));
        state_.out_pct = LC108::decode_percent(static_cast<int16_t>(regs_[1]));
        state_.sv = LC108::decode_temp(static_cast<int16_t>(regs_[kRegCount - 1]));
        refreshed = true;
      }

      if (pending_result_ == PendingResult::Fail) {
        pending_result_ = PendingResult::None;
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

      if (!scheduled) return refreshed;

      if (!request_in_flight_) {
        if (rep_.reason && strcmp(rep_.reason, "not_probed") == 0) {
          probe(now_ms);
        } else {
          start_read(now_ms);
        }
      }
      return refreshed;
    }

    namespace {
    uint16_t modbus_addr(uint16_t one_based) {
      return (one_based > 0) ? static_cast<uint16_t>(one_based - 1) : 0;
    }
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
      const uint16_t tx_id = mb_.readHreg(
        slave_id_, modbus_addr(kRegBase), regs_, kRegCount, &PidModbusComponent::on_transaction);
      if (tx_id == 0) {
        request_in_flight_ = false;
        active_request_ = nullptr;
        return false;
      }
      return true;
    }

    void PidModbusComponent::handle_transaction(Modbus::ResultCode result) {
      request_in_flight_ = false;
      active_request_ = nullptr;
      if (result == Modbus::EX_SUCCESS) {
        pending_result_ = PendingResult::Ok;
      } else {
        pending_result_ = PendingResult::Fail;
      }
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
