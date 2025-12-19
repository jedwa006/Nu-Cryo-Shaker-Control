#include <unity.h>

#include "core/health_manager.h"

namespace {
class FakeComponent : public IHealthComponent {
public:
  explicit FakeComponent(const char* name) : name_(name) {}

  const char* name() const override { return name_; }
  void configure(bool expected, bool required) override {
    report_.expected = expected;
    report_.required = required;
  }
  bool probe(uint32_t) override { return true; }
  bool tick(uint32_t) override { return true; }
  uint32_t stale_timeout_ms() const override { return stale_timeout_ms_; }
  HealthReport report() const override { return report_; }

  void set_report(const HealthReport& report) { report_ = report; }
  void set_stale_timeout_ms(uint32_t timeout_ms) { stale_timeout_ms_ = timeout_ms; }

private:
  const char* name_;
  HealthReport report_ {};
  uint32_t stale_timeout_ms_ {0};
};

void test_required_component_fault_sets_inhibit() {
  HealthManager health;
  FakeComponent pid("pid_heat1");
  pid.configure(true, true);

  HealthReport report {};
  report.expected = true;
  report.required = true;
  report.status = HealthStatus::ERROR;
  report.last_ok_ms = 100;
  pid.set_report(report);

  TEST_ASSERT_TRUE(health.add(&pid));
  health.evaluate(200);

  const SystemHealth& sys = health.system_health();
  TEST_ASSERT_EQUAL(HealthStatus::ERROR, sys.system_state);
  TEST_ASSERT_FALSE(sys.run_allowed);
  TEST_ASSERT_FALSE(sys.outputs_allowed);
  TEST_ASSERT_EQUAL_UINT16(1, sys.crit_count);
}

void test_optional_component_fault_degrades_only() {
  HealthManager health;
  FakeComponent aux("aux_sensor");
  aux.configure(true, false);

  HealthReport report {};
  report.expected = true;
  report.required = false;
  report.status = HealthStatus::ERROR;
  report.last_ok_ms = 100;
  aux.set_report(report);

  TEST_ASSERT_TRUE(health.add(&aux));
  health.evaluate(200);

  const SystemHealth& sys = health.system_health();
  TEST_ASSERT_EQUAL(HealthStatus::DEGRADED, sys.system_state);
  TEST_ASSERT_TRUE(sys.run_allowed);
  TEST_ASSERT_TRUE(sys.outputs_allowed);
  TEST_ASSERT_EQUAL_UINT16(1, sys.warn_count);
}

void test_stale_required_component_inhibits_run() {
  HealthManager health;
  FakeComponent pid("pid_cool1");
  pid.configure(true, true);
  pid.set_stale_timeout_ms(50);

  HealthReport report {};
  report.expected = true;
  report.required = true;
  report.status = HealthStatus::OK;
  report.last_ok_ms = 100;
  pid.set_report(report);

  TEST_ASSERT_TRUE(health.add(&pid));
  health.evaluate(200);

  const SystemHealth& sys = health.system_health();
  TEST_ASSERT_EQUAL(HealthStatus::ERROR, sys.system_state);
  TEST_ASSERT_FALSE(sys.run_allowed);
  TEST_ASSERT_FALSE(sys.outputs_allowed);
}
} // namespace

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_required_component_fault_sets_inhibit);
  RUN_TEST(test_optional_component_fault_degrades_only);
  RUN_TEST(test_stale_required_component_inhibits_run);
  UNITY_END();
}

void loop() {}
