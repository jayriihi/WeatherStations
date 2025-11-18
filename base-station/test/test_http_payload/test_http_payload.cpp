#include <unity.h>

#include <string>

#include "domain/HttpPayloadBuilder.h"

void test_payload_format() {
  domain::HttpPayloadBuilder builder;
  core::HttpPostRequest req;
  req.packet.windAvg = 12.5f;
  req.packet.windMax = 20.0f;
  req.packet.windDir = 180;
  req.packet.counter = 55;
  req.packet.batteryVolts = 3.65f;
  req.health.crcOk = 10;
  req.health.crcFail = 1;
  req.health.rfDuplicate = 2;
  req.health.uptimeSeconds = 3600;
  req.health.bootId = 12345;
  req.health.dropGap = 1;
  req.health.dropTotal = 3;
  req.health.dropRate = 0.1234f;
  req.rssi = -110.5f;
  req.snr = 7.2f;

  std::string body = builder.build(req);
  TEST_ASSERT_EQUAL_STRING(
      "wind_avg=12.5&wind_max=20.0&wind_dir=180&cnt=55&batt=3.65&"
      "rssi=-110.5&snr=7.2&crc_ok=10&crc_fail=1&rf_dup=2&"
      "uptime_s=3600&boot_id=12345&fw=base_1&drop_gap=1&drop_total=3&drop_rate=0.1234",
      body.c_str());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_payload_format);
  return UNITY_END();
}
