#include <unity.h>

#include <cstdio>
#include <string>

#include "domain/LoRaPacket.h"

namespace {
std::string buildFrame(const std::string& json) {
  uint16_t crc = domain::LoRaPacket::ComputeCrc(json);
  char trailer[6];
  std::snprintf(trailer, sizeof(trailer), "*%04X", static_cast<unsigned>(crc));
  return json + trailer;
}
}

void test_valid_packet() {
  domain::LoRaPacket parser;
  std::string json = "{\"wind_avg\":12.3,\"wind_max\":18.5,\"wind_dir\":200,\"cnt\":42,\"batt\":3.7}";
  auto frame = buildFrame(json);
  auto result = parser.parse(frame);
  TEST_ASSERT_TRUE(result.ok());
  TEST_ASSERT_EQUAL_FLOAT(12.3f, result.packet.windAvg);
  TEST_ASSERT_EQUAL_FLOAT(18.5f, result.packet.windMax);
  TEST_ASSERT_EQUAL_INT(200, result.packet.windDir);
  TEST_ASSERT_EQUAL_INT32(42, result.packet.counter);
  TEST_ASSERT_EQUAL_FLOAT(3.7f, result.packet.batteryVolts);
}

void test_bad_crc() {
  domain::LoRaPacket parser;
  std::string frame = "{\"wind_avg\":1}*FFFF";
  auto result = parser.parse(frame);
  TEST_ASSERT_FALSE(result.ok());
  TEST_ASSERT_EQUAL(domain::ParseStatus::kCrcMismatch, result.status);
}

void test_missing_trailer() {
  domain::LoRaPacket parser;
  auto result = parser.parse("{\"wind_avg\":1}");
  TEST_ASSERT_FALSE(result.ok());
  TEST_ASSERT_EQUAL(domain::ParseStatus::kMissingTrailer, result.status);
}

void test_zero_payload() {
  domain::LoRaPacket parser;
  std::string json = "{\"wind_avg\":0,\"wind_max\":0,\"wind_dir\":0,\"cnt\":10,\"batt\":3.2}";
  auto frame = buildFrame(json);
  auto result = parser.parse(frame);
  TEST_ASSERT_FALSE(result.ok());
  TEST_ASSERT_EQUAL(domain::ParseStatus::kZeroPayload, result.status);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_valid_packet);
  RUN_TEST(test_bad_crc);
  RUN_TEST(test_missing_trailer);
  RUN_TEST(test_zero_payload);
  return UNITY_END();
}
