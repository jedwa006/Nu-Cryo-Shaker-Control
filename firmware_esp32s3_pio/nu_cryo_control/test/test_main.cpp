#include <unity.h>

void test_required_component_fault_sets_inhibit();
void test_optional_component_fault_degrades_only();
void test_stale_required_component_inhibits_run();
void test_start_sets_running_state();
void test_estop_latch_requires_reset();
void test_health_fault_blocks_start_until_reset();

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_required_component_fault_sets_inhibit);
  RUN_TEST(test_optional_component_fault_degrades_only);
  RUN_TEST(test_stale_required_component_inhibits_run);
  RUN_TEST(test_start_sets_running_state);
  RUN_TEST(test_estop_latch_requires_reset);
  RUN_TEST(test_health_fault_blocks_start_until_reset);
  return UNITY_END();
}
