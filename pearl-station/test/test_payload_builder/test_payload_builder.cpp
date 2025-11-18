#include <unity.h>

#include <cstdio>
#include <string>

#include "domain/PayloadBuilder.h"

namespace {
uint16_t crc16(const std::string& data) {
  uint16_t crc = 0xFFFF;
  for (char c : data) {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(c)) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
}

void test_payload_contents() {
  domain::PayloadBuilder builder;
  core::WindSummary summary;
  summary.averageSpeedMs = 5.0f;
  summary.maxGustMs = 8.0f;
  summary.averageDirectionDeg = 200.0f;

  std::string payload = builder.build(summary, 3.30f, 42);
  auto star = payload.find('*');
  TEST_ASSERT_NOT_EQUAL(std::string::npos, star);

  std::string json = payload.substr(0, star);
  std::string crc = payload.substr(star + 1);
  TEST_ASSERT_EQUAL_STRING(
      "{\"wind_avg\":9.7,\"wind_max\":15.6,\"wind_dir\":200,\"cnt\":42,\"batt\":3.30}",
      json.c_str());

  uint16_t expectedCrc = crc16(json);
  char crcBuf[5];
  std::snprintf(crcBuf, sizeof(crcBuf), "%04X", static_cast<unsigned>(expectedCrc));
  TEST_ASSERT_EQUAL_STRING(crcBuf, crc.c_str());
}

void test_payload_negative_battery() {
  domain::PayloadBuilder builder;
  core::WindSummary summary;
  summary.averageSpeedMs = 0.0f;
  summary.maxGustMs = 0.0f;
  summary.averageDirectionDeg = 90.0f;

  std::string payload = builder.build(summary, -1.0f, 7);
  auto star = payload.find('*');
  TEST_ASSERT_NOT_EQUAL(std::string::npos, star);
  std::string json = payload.substr(0, star);
  TEST_ASSERT_EQUAL_STRING(
      "{\"wind_avg\":0.0,\"wind_max\":0.0,\"wind_dir\":90,\"cnt\":7,\"batt\":-1.00}",
      json.c_str());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_payload_contents);
  RUN_TEST(test_payload_negative_battery);
  return UNITY_END();
}
