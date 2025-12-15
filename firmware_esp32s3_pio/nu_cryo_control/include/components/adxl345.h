\
    #pragma once
    #include "core/health_manager.h"
    #include <Wire.h>

    class Adxl345Component : public IHealthComponent {
    public:
      explicit Adxl345Component(TwoWire& wire, uint8_t addr=0x53);

      const char* name() const override { return "adxl345"; }
      void configure(bool expected, bool required) override;

      bool probe(uint32_t now_ms) override;
      bool tick(uint32_t now_ms) override;

      uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
      HealthReport report() const override { return rep_; }

      // Minimal feature output (extend later)
      float rms_g() const { return rms_g_; }

    private:
      TwoWire& wire_;
      uint8_t addr_;
      HealthReport rep_ {};
      uint32_t stale_timeout_ms_ {1500};

      float rms_g_ {0};

      bool read_devid(uint8_t& devid);
    };
