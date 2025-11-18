#include <unity.h>

#include "domain/AckLogic.h"

void test_rate_limit_and_duplicate() {
  domain::AckLogic logic(5000, 20000, 70000);
  auto decision = logic.evaluate(10, "bodyA", 0);
  TEST_ASSERT_TRUE(decision.shouldPost);

  logic.markSuccess(10, "bodyA", 100);

  decision = logic.evaluate(10, "bodyA", 1000);
  TEST_ASSERT_FALSE(decision.shouldPost);
  TEST_ASSERT_TRUE(decision.sendAck);
  TEST_ASSERT_EQUAL_STRING("DUP", decision.status.c_str());

  decision = logic.evaluate(11, "bodyB", 1000);
  TEST_ASSERT_FALSE(decision.shouldPost);
  TEST_ASSERT_TRUE(decision.sendAck);
  TEST_ASSERT_EQUAL_STRING("RATE", decision.status.c_str());

  decision = logic.evaluate(11, "bodyB", 21000);
  TEST_ASSERT_TRUE(decision.shouldPost);
}

void test_inflight_guard() {
  domain::AckLogic logic(5000, 0, 70000);
  logic.markInflight(42, 1000);
  auto decision = logic.evaluate(42, "body", 2000);
  TEST_ASSERT_FALSE(decision.shouldPost);
  TEST_ASSERT_TRUE(decision.sendAck);
  TEST_ASSERT_EQUAL_STRING("DUP", decision.status.c_str());

  logic.expireInflight(7000);
  decision = logic.evaluate(42, "body", 7000);
  TEST_ASSERT_TRUE(decision.shouldPost);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_rate_limit_and_duplicate);
  RUN_TEST(test_inflight_guard);
  return UNITY_END();
}
