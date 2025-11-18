#include <cmath>

#include <unity.h>

#include "domain/WindAverager.h"

void test_empty_block() {
  domain::WindAverager averager;
  core::WindSummary summary;
  TEST_ASSERT_FALSE(averager.finalize(summary));
  TEST_ASSERT_EQUAL_UINT16(0, summary.sampleCount);
}

void test_single_sample() {
  domain::WindAverager averager;
  domain::WindSample sample{10.0f, 10.0f, 180.0f, true};
  averager.addSample(sample);

  core::WindSummary summary;
  TEST_ASSERT_TRUE(averager.finalize(summary));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, summary.averageSpeedMs);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, summary.maxGustMs);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 180.0f, summary.averageDirectionDeg);
  TEST_ASSERT_EQUAL_UINT16(1, summary.sampleCount);
}

void test_multiple_samples() {
  domain::WindAverager averager;
  averager.addSample(domain::WindSample{8.0f, 8.0f, 90.0f, true});
  averager.addSample(domain::WindSample{12.0f, 12.0f, 120.0f, true});

  core::WindSummary summary;
  TEST_ASSERT_TRUE(averager.finalize(summary));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, summary.averageSpeedMs);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, summary.maxGustMs);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 105.0f, summary.averageDirectionDeg);
  TEST_ASSERT_EQUAL_UINT16(2, summary.sampleCount);
}

void test_direction_wraparound() {
  domain::WindAverager averager;
  averager.addSample(domain::WindSample{5.0f, 5.0f, 350.0f, true});
  averager.addSample(domain::WindSample{5.0f, 5.0f, 10.0f, true});

  core::WindSummary summary;
  TEST_ASSERT_TRUE(averager.finalize(summary));
  float dir = summary.averageDirectionDeg;
  bool nearZero = std::fabs(dir - 0.0f) <= 1.0f;
  bool nearCircle = std::fabs(dir - 360.0f) <= 1.0f;
  TEST_ASSERT_TRUE(nearZero || nearCircle);
}

void test_gust_tracking() {
  domain::WindAverager averager;
  averager.addSample(domain::WindSample{5.0f, 12.0f, 45.0f, true});
  averager.addSample(domain::WindSample{7.0f, 7.0f, 50.0f, true});

  core::WindSummary summary;
  TEST_ASSERT_TRUE(averager.finalize(summary));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, summary.maxGustMs);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_empty_block);
  RUN_TEST(test_single_sample);
  RUN_TEST(test_multiple_samples);
  RUN_TEST(test_direction_wraparound);
  RUN_TEST(test_gust_tracking);
  return UNITY_END();
}
