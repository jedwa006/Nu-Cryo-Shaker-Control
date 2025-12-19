#include <unity.h>

#include "core/health_manager.h"
#include "core/run_control.h"

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

private:
  const char* name_;
  HealthReport report_ {};
  uint32_t stale_timeout_ms_ {0};
};

HealthManager make_health_ok(uint32_t now_ms) {
  HealthManager health;
  auto* component = new FakeComponent("pid_heat1");
  component->configure(true, true);

  HealthReport report {};
  report.expected = true;
  report.required = true;
  report.status = HealthStatus::OK;
  report.last_ok_ms = now_ms;
  component->set_report(report);

  health.add(component);
  health.evaluate(now_ms);
  return health;
}

HealthManager make_health_fault(uint32_t now_ms) {
  HealthManager health;
  auto* component = new FakeComponent("pid_heat1");
  component->configure(true, true);

  HealthReport report {};
  report.expected = true;
  report.required = true;
  report.status = HealthStatus::ERROR;
  report.last_ok_ms = now_ms;
  component->set_report(report);

  health.add(component);
  health.evaluate(now_ms);
  return health;
}

void test_start_sets_running_state() {
  RunControl run;
  DinComponent din;
  din.set_interlocks_ok(true);

  const uint32_t now_ms = 100;
  HealthManager health = make_health_ok(now_ms);

  const char* err = nullptr;
  TEST_ASSERT_TRUE(run.handle_command(RunCommand::START, health, din, now_ms, &err));
  TEST_ASSERT_NULL(err);
  TEST_ASSERT_EQUAL(RunState::RUNNING, run.status().state);
  TEST_ASSERT_TRUE(run.status().run_allowed);
  TEST_ASSERT_TRUE(run.status().outputs_allowed);
}

void test_estop_latch_requires_reset() {
  RunControl run;
  DinComponent din;
  din.set_reason("estop_tripped");

  uint32_t now_ms = 100;
  HealthManager health = make_health_ok(now_ms);

  din.set_interlocks_ok(false);
  run.update(health, din, now_ms);
  TEST_ASSERT_EQUAL(RunState::ESTOP, run.status().state);

  din.set_interlocks_ok(true);
  const char* err = nullptr;
  TEST_ASSERT_FALSE(run.handle_command(RunCommand::START, health, din, now_ms + 10, &err));
  TEST_ASSERT_EQUAL_STRING("inhibited", err);

  err = nullptr;
  TEST_ASSERT_TRUE(run.handle_command(RunCommand::RESET, health, din, now_ms + 20, &err));
  TEST_ASSERT_NULL(err);
  TEST_ASSERT_EQUAL(RunState::STOPPED, run.status().state);
}

void test_health_fault_blocks_start_until_reset() {
  RunControl run;
  DinComponent din;
  din.set_interlocks_ok(true);

  uint32_t now_ms = 200;
  HealthManager health_fault = make_health_fault(now_ms);

  run.update(health_fault, din, now_ms);
  TEST_ASSERT_EQUAL(RunState::STOPPED, run.status().state);

  const char* err = nullptr;
  TEST_ASSERT_FALSE(run.handle_command(RunCommand::START, health_fault, din, now_ms + 10, &err));
  TEST_ASSERT_EQUAL_STRING("inhibited", err);

  HealthManager health_ok = make_health_ok(now_ms + 20);
  err = nullptr;
  TEST_ASSERT_TRUE(run.handle_command(RunCommand::RESET, health_ok, din, now_ms + 20, &err));
  TEST_ASSERT_NULL(err);
}
} // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_start_sets_running_state);
  RUN_TEST(test_estop_latch_requires_reset);
  RUN_TEST(test_health_fault_blocks_start_until_reset);
  return UNITY_END();
}
