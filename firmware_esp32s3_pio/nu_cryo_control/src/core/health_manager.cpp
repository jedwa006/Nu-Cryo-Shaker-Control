\
    #include "health_manager.h"

    bool HealthManager::add(IHealthComponent* c) {
      if (!c) return false;
      if (n_ >= MAX_COMPONENTS) return false;
      comps_[n_++] = c;
      return true;
    }

    bool HealthManager::is_bad(HealthStatus s) {
      return (s == HealthStatus::MISSING) || (s == HealthStatus::ERROR) || (s == HealthStatus::STALE);
    }

    void HealthManager::evaluate(uint32_t now_ms) {
      // Aggregate across component reports. Components own their own transitions;
      // we only compute system-level view and optionally apply stale rules.
      uint16_t warn = 0, crit = 0;
      bool any_required_bad = false;
      bool any_optional_bad = false;

      for (size_t i = 0; i < n_; ++i) {
        IHealthComponent* c = comps_[i];
        if (!c) continue;

        HealthReport r = c->report();

        // Apply stale rule centrally (without mutating component) by interpreting report.
        // If a component is expected and has a stale timeout, and hasn't been OK recently,
        // we treat it as STALE for aggregation severity purposes.
        if (r.expected && r.status == HealthStatus::OK) {
          // OK is OK
        } else if (r.expected && c->stale_timeout_ms() > 0) {
          const uint32_t dt = now_ms - r.last_ok_ms;
          if (r.last_ok_ms > 0 && dt > c->stale_timeout_ms()) {
            r.status = HealthStatus::STALE;
          }
        }

        const bool bad = r.expected && is_bad(r.status);

        if (bad) {
          if (r.required) {
            any_required_bad = true;
            crit++;
          } else {
            any_optional_bad = true;
            warn++;
          }
        }
      }

      sys_.warn_count = warn;
      sys_.crit_count = crit;

      if (any_required_bad) {
        sys_.system_state = HealthStatus::ERROR; // interpreted as FAULT at publish layer
        sys_.degraded = true;
        sys_.run_allowed = false;
        sys_.outputs_allowed = false;
      } else if (any_optional_bad) {
        sys_.system_state = HealthStatus::DEGRADED;
        sys_.degraded = true;
        sys_.run_allowed = true;
        sys_.outputs_allowed = true;
      } else {
        sys_.system_state = HealthStatus::OK;
        sys_.degraded = false;
        sys_.run_allowed = true;
        sys_.outputs_allowed = true;
      }
    }
