\
#include "components/adxl345.h"
    static constexpr uint8_t REG_DEVID = 0x00;
    static constexpr uint8_t ADXL345_DEVID = 0xE5;

    Adxl345Component::Adxl345Component(TwoWire& wire, uint8_t addr)
      : wire_(wire), addr_(addr) {}

    void Adxl345Component::configure(bool expected, bool required) {
      rep_.expected = expected;
      rep_.required = required;
      rep_.status = expected ? HealthStatus::MISSING : HealthStatus::UNCONFIGURED;
      rep_.severity = required ? Severity::CRIT : Severity::INFO;
      rep_.reason = expected ? "not_probed" : "unconfigured";
      rep_.since_ms = 0;
      rep_.last_ok_ms = 0;
    }

    bool Adxl345Component::read_devid(uint8_t& devid) {
      wire_.beginTransmission(addr_);
      wire_.write(REG_DEVID);
      if (wire_.endTransmission(false) != 0) return false;
      if (wire_.requestFrom((int)addr_, 1) != 1) return false;
      devid = wire_.read();
      return true;
    }

    bool Adxl345Component::probe(uint32_t now_ms) {
      if (!rep_.expected) return false;

      uint8_t devid = 0;
      const bool ok = read_devid(devid) && (devid == ADXL345_DEVID);

      if (ok) {
        rep_.status = HealthStatus::OK;
        rep_.severity = Severity::INFO;
        rep_.reason = "ok";
        rep_.since_ms = now_ms;
        rep_.last_ok_ms = now_ms;
      } else {
        rep_.status = HealthStatus::MISSING;
        rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
        rep_.reason = "not_detected_on_i2c";
        if (rep_.since_ms == 0) rep_.since_ms = now_ms;
      }
      return ok;
    }

    bool Adxl345Component::tick(uint32_t now_ms) {
      if (!rep_.expected) return false;

      // For now, we only refresh presence. Replace with actual reads + RMS features later.
      uint8_t devid = 0;
      const bool ok = read_devid(devid) && (devid == ADXL345_DEVID);

      if (ok) {
        rep_.status = HealthStatus::OK;
        rep_.severity = Severity::INFO;
        rep_.reason = "ok";
        rep_.last_ok_ms = now_ms;

        // placeholder feature
        rms_g_ = 0.0f;
      } else {
        if (rep_.status == HealthStatus::OK) {
          rep_.status = HealthStatus::DEGRADED;
          rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
          rep_.reason = "i2c_read_fail";
          rep_.since_ms = now_ms;
        }
      }
      return ok;
    }
