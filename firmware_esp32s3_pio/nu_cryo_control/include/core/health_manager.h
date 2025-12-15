\
    #pragma once
    #include <stdint.h>
    #include <stddef.h>

    enum class HealthStatus : uint8_t {
      UNCONFIGURED = 0,
      MISSING      = 1,
      OK           = 2,
      DEGRADED     = 3,
      STALE        = 4,
      ERROR        = 5
    };

    enum class Severity : uint8_t { INFO=0, WARN=1, CRIT=2 };

    struct HealthReport {
      HealthStatus status {HealthStatus::UNCONFIGURED};
      Severity severity {Severity::INFO};
      bool expected {false};   // do we currently care if it exists?
      bool required {false};   // must be OK to run safely?
      const char* reason {""}; // short machine-readable string
      uint32_t since_ms {0};   // when this status started
      uint32_t last_ok_ms {0}; // last time component was OK/fresh
    };

    class IHealthComponent {
    public:
      virtual ~IHealthComponent() = default;
      virtual const char* name() const = 0;

      // Called once at setup
      virtual void configure(bool expected, bool required) = 0;

      // Called on boot and occasionally if expected-but-missing.
      virtual bool probe(uint32_t now_ms) = 0;

      // Called periodically when expected=true. Should attempt to refresh data.
      // Return true if data refreshed successfully ("fresh").
      virtual bool tick(uint32_t now_ms) = 0;

      // How long without success before considered STALE (0 disables stale logic)
      virtual uint32_t stale_timeout_ms() const = 0;

      // Current health snapshot (must be internally consistent)
      virtual HealthReport report() const = 0;
    };

    struct SystemHealth {
      HealthStatus system_state {HealthStatus::OK};
      bool degraded {false};
      bool run_allowed {true};
      bool outputs_allowed {true};
      uint16_t warn_count {0};
      uint16_t crit_count {0};
    };

    class HealthManager {
    public:
      static constexpr size_t MAX_COMPONENTS = 16;

      bool add(IHealthComponent* c);

      // Call often (e.g., in loop). This does not call tick() for components;
      // it only evaluates reports and applies stale logic/aggregation.
      void evaluate(uint32_t now_ms);

      const SystemHealth& system_health() const { return sys_; }

      size_t count() const { return n_; }
      IHealthComponent* component(size_t i) const { return (i < n_) ? comps_[i] : nullptr; }

    private:
      IHealthComponent* comps_[MAX_COMPONENTS] {};
      size_t n_ {0};
      SystemHealth sys_ {};

      static bool is_bad(HealthStatus s);
    };
