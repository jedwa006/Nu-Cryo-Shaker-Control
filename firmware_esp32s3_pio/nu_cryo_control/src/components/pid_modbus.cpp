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

    bool PidModbusComponent::probe(uint32_t now_ms) {
      if (!rep_.expected) return false;

      // Simple probe: attempt a single live read.
      float pv=0, sv=0, out=0;
      const bool ok = read_live(pv, sv, out);

      if (ok) {
        rep_.status = HealthStatus::OK;
        rep_.severity = Severity::INFO;
        rep_.reason = "ok";
        rep_.since_ms = now_ms;
        rep_.last_ok_ms = now_ms;
        state_.valid = true;
        state_.pv = pv; state_.sv = sv; state_.out_pct = out;
      } else {
        rep_.status = HealthStatus::MISSING;
        rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
        rep_.reason = "modbus_no_response";
        if (rep_.since_ms == 0) rep_.since_ms = now_ms;
        state_.valid = false;
      }
      return ok;
    }

    bool PidModbusComponent::tick(uint32_t now_ms) {
      if (!rep_.expected) return false;

      if (rep_.reason && strcmp(rep_.reason, "not_probed") == 0) {
        return probe(now_ms);
      }

      float pv=0, sv=0, out=0;
      const bool ok = read_live(pv, sv, out);

      if (ok) {
        rep_.status = HealthStatus::OK;
        rep_.severity = Severity::INFO;
        rep_.reason = "ok";
        rep_.last_ok_ms = now_ms;
        state_.valid = true;
        state_.pv = pv; state_.sv = sv; state_.out_pct = out;
      } else {
        // Do not immediately jump to MISSING; treat as DEGRADED until stale timeout crosses.
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
      return ok;
    }

    namespace {
    uint16_t modbus_addr(uint16_t one_based) {
      return (one_based > 0) ? static_cast<uint16_t>(one_based - 1) : 0;
    }
    }

    bool PidModbusComponent::read_live(float& pv, float& sv, float& out) {
      uint16_t pv_raw=0, sv_raw=0, out_raw=0;

      bool ok = true;
      ok &= mb_.readHreg(slave_id_, modbus_addr(REG_PV), &pv_raw, 1);
      ok &= mb_.readHreg(slave_id_, modbus_addr(REG_SV), &sv_raw, 1);
      ok &= mb_.readHreg(slave_id_, modbus_addr(REG_OUT_PCT), &out_raw, 1);
      mb_.task(); // allow library to progress

      if (!ok) return false;

      pv = LC108::decode_temp(static_cast<int16_t>(pv_raw));
      sv = LC108::decode_temp(static_cast<int16_t>(sv_raw));
      out = LC108::decode_percent(static_cast<int16_t>(out_raw));
      return true;
    }

    bool PidModbusComponent::set_sv(float sv, uint32_t now_ms) {
      if (!rep_.expected) return false;
      const uint16_t sv_raw = static_cast<uint16_t>(LC108::encode_temp(sv));

      const bool ok = mb_.writeHreg(slave_id_, modbus_addr(REG_SV), sv_raw);
      mb_.task();

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
