\
    #include "pid_modbus.h"
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
        }
        state_.valid = false;
      }
      return ok;
    }

    bool PidModbusComponent::read_live(float& pv, float& sv, float& out) {
      // Placeholder implementation:
      // Many PID controllers return scaled ints (e.g. 10x). Update this to your register map.
      uint16_t pv_raw=0, sv_raw=0, out_raw=0;

      bool ok = true;
      ok &= mb_.readHreg(slave_id_, REG_PV, &pv_raw, 1);
      ok &= mb_.readHreg(slave_id_, REG_SV, &sv_raw, 1);
      ok &= mb_.readHreg(slave_id_, REG_OUT_PCT, &out_raw, 1);
      mb_.task(); // allow library to progress

      if (!ok) return false;

      pv = (float)pv_raw * 0.1f;
      sv = (float)sv_raw * 0.1f;
      out = (float)out_raw * 0.1f;
      return true;
    }

    bool PidModbusComponent::set_sv(float sv, uint32_t now_ms) {
      if (!rep_.expected) return false;
      const uint16_t sv_raw = (uint16_t)(sv * 10.0f);

      const bool ok = mb_.writeHreg(slave_id_, REG_SV, sv_raw);
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
