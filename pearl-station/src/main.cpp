#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>

// === CRC16 APP-LAYER ===
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}


// ====== CONFIG ======
// 0 = read real WindSonic (to be added in getSample())
// 1 = generate simulated wind samples
#define SIM_MODE   1

// length of each measurement block in seconds
// use 10 for bench testing; set to 120 (2 min) in production
static const uint16_t BLOCK_SECONDS = 120;

// --- Heltec WiFi LoRa 32 V3 (SX1262) pins ---
static const int PIN_NSS  = 8;   // CS
static const int PIN_DIO1 = 14;
static const int PIN_RST  = 12;  // NRST
static const int PIN_BUSY = 13;
static const int PIN_SCK  = 9;
static const int PIN_MISO = 11;
static const int PIN_MOSI = 10;

Module* modPtr = nullptr;
SX1262* lora   = nullptr;

// Battery sensing not wired yet.
// #define PIN_BATTERY_SENSE  1   // <- will become an ADC1 pin once installed

static const float ADC_REF_V       = 3.3f;
static const float ADC_MAX_COUNTS  = 4095.0f;
static const float DIVIDER_GAIN    = 4.6f;   // 360k / 100k divider ratio
static const int   BATT_SAMPLES    = 8;

float readBatteryVolts() {
  // Battery sense not hooked up yet.
  // Return sentinel so we can still log a batt field.
  return -1.0f;
}

// ---- helpers ----

// random float in [a,b]
static inline float frand(float a, float b) {
  return a + (b - a) * (float)esp_random() / (float)UINT32_MAX;
}

// minutes since boot; swap to epoch/60 if you add RTC/NTP later
static inline uint32_t bootMinutes() { return millis() / 60000UL; }

// RadioLib wants a non-const String&
static inline void txOnce(String& body) {
  int st = lora->transmit(body);
  Serial.printf("PEARL TX state: %d\n", st);
}

// ========== ACCUMULATORS FOR THE ACTIVE BLOCK (2 min in production) ==========
static uint16_t sample_count = 0;
static float    sum_speed    = 0.0f;   // m/s
static float    max_gust_1s  = 0.0f;   // m/s
static float    sum_dir_x    = 0.0f;   // unitless
static float    sum_dir_y    = 0.0f;   // unitless

static void accumulateSample(float spd_ms, float dir_deg) {
  // avg speed math
  sum_speed += spd_ms;

  // gust math (1-second instantaneous peak)
  if (spd_ms > max_gust_1s) {
    max_gust_1s = spd_ms;
  }

  // circular mean for direction
  float rad = dir_deg * DEG_TO_RAD;
  sum_dir_x += cosf(rad);
  sum_dir_y += sinf(rad);

  sample_count++;
}

// finalize the block → produce wind_avg, wind_max, wind_dir, then reset accumulators
static void finalizeBlock(float &wind_avg_ms,
                          float &wind_max_ms,
                          float &wind_dir_deg)
{
  if (sample_count > 0) {
    wind_avg_ms = sum_speed / sample_count;
  } else {
    wind_avg_ms = 0.0f;
  }

  wind_max_ms = max_gust_1s;

  float avg_rad = atan2f(sum_dir_y, sum_dir_x);
  float avg_deg = avg_rad * RAD_TO_DEG;
  if (avg_deg < 0.0f) {
    avg_deg += 360.0f;
  }
  wind_dir_deg = avg_deg;

  // reset for next block
  sample_count = 0;
  sum_speed    = 0.0f;
  max_gust_1s  = 0.0f;
  sum_dir_x    = 0.0f;
  sum_dir_y    = 0.0f;
}

// ========== SAMPLE SOURCE ==========
// getSample() is the ONLY place that knows if we're sim or real sensor.
// Later we drop WindSonic parsing into the #else path.
static void getSample(float &spd_ms, float &dir_deg) {
#if SIM_MODE
  // --- SIMULATED DATA PATH ---
  // baseline wind 8–12 m/s with rare gust spikes to ~17 m/s
  spd_ms = frand(8.0f, 12.0f);
  if (frand(0.0f, 1.0f) > 0.97f) {   // ~3% gust pop
    spd_ms = frand(13.0f, 17.0f);
  }

  // direction hovering ~070° ± 8°
  dir_deg = 70.0f + frand(-8.0f, 8.0f);
  if (dir_deg < 0.0f)    dir_deg += 360.0f;
  if (dir_deg >= 360.0f) dir_deg -= 360.0f;

#else
  // --- REAL SENSOR PATH (WindSonic) ---
  // TODO:
  // 1. Read one line from the WindSonic serial (NMEA-style sentence)
  // 2. Parse speed (m/s) and direction (deg)
  // 3. Assign to spd_ms and dir_deg
  //
  // For now, just dummy fallback so code compiles:
  spd_ms  = 0.0f;
  dir_deg = 0.0f;
#endif
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(300);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(915.0);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa init failed (%d)\n", st);
    while (true) delay(1000);
  }

  // MUST match Base
  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setOutputPower(-10);   // keep low if boards are close
  lora->setCRC(true);

  Serial.println("PEARL: LoRa OK");
  Serial.printf("SIM_MODE=%d BLOCK_SECONDS=%u\n", (int)SIM_MODE, (unsigned)BLOCK_SECONDS);
}

// timing state
static unsigned long last_sample_ms   = 0;
static uint16_t      seconds_in_block = 0;

// ========== LOOP ==========
// We do 1 Hz sampling into a block. When the block ends, we:
// - compute avg/dir/gust
// - build JSON
// - LoRa transmit once
// - reset for next block
void loop() {
  unsigned long now = millis();

  // 1 Hz sampler
  if (now - last_sample_ms >= 1000) {
    last_sample_ms = now;
    seconds_in_block++;

    float spd_ms, dir_deg;
    getSample(spd_ms, dir_deg);
    accumulateSample(spd_ms, dir_deg);

    Serial.printf("sample %3u: spd=%.2f m/s dir=%.1f deg\n",
                  seconds_in_block, spd_ms, dir_deg);
  }

  // end of block?
  if (seconds_in_block >= BLOCK_SECONDS) {
    seconds_in_block = 0;

    // finalize the rolling 2-min-style block
    float wind_avg_ms, wind_max_ms, wind_dir_deg;
    finalizeBlock(wind_avg_ms, wind_max_ms, wind_dir_deg);

    // convert avg + gust to knots for debug sanity
    float wind_avg_kt = wind_avg_ms * 1.94384f;
    float wind_max_kt = wind_max_ms * 1.94384f;

    // placeholder battery reading
    float vbatt = readBatteryVolts();   // -1.00 until wired

    // message counter for dedupe / ordering
    uint32_t cnt = bootMinutes();       // minutes since boot

    // Build payload consistent with Base expectations:
    //   wind_avg  (m/s)
    //   wind_max  (m/s gust)
    //   wind_dir  (deg 0-360)
    //   cnt       (monotonic counter)
    //   batt      (V or -1.00 sentinel)
    char buf[224];
    snprintf(buf, sizeof(buf),
      "{\"wind_avg\":%.1f,\"wind_max\":%.1f,\"wind_dir\":%.0f,\"cnt\":%lu,\"batt\":%.2f}",
      wind_avg_ms,
      wind_max_ms,
      wind_dir_deg,
      (unsigned long)cnt,
      vbatt
    );

    String payload(buf);

    // === CRC16 APP-LAYER === compute over JSON bytes, append "*%04X"
    uint16_t crc = crc16_ccitt((const uint8_t*)payload.c_str(), payload.length());
    char trailer[6];                                   // "*" + 4 hex + '\0'
    snprintf(trailer, sizeof(trailer), "*%04X", (unsigned)crc);
    payload += trailer;                                // "{...}*9A5B"

    // Debug local printout
    Serial.println("----- BLOCK RESULT -----");
    Serial.printf("avg   = %.2f m/s (%.1f kt)\n", wind_avg_ms, wind_avg_kt);
    Serial.printf("gust  = %.2f m/s (%.1f kt)\n", wind_max_ms, wind_max_kt);
    Serial.printf("dir   = %.1f deg\n",          wind_dir_deg);
    Serial.printf("cnt   = %lu\n",               (unsigned long)cnt);
    Serial.printf("batt  = %.2f V\n",            vbatt);
    Serial.println("[TX] " + payload);
    Serial.println("------------------------\n");

    // LoRa TX
    txOnce(payload);

    // jittered pause before starting next block sampling loop
    // (keeps us from slamming exactly on the minute every time)
    uint32_t ms     = millis();
    uint32_t jitter = 200 + (esp_random() % 400);  // 200..599 ms
    delay(jitter);

    // NOTE:
    // We immediately resume sampling next loop(), no deep sleep yet.
    // Later for power savings on the island we can:
    // - sleep between samples
    // - or only wake for BLOCK_SECONDS every 5 minutes
  }

  // tiny idle delay so we aren't burning 100% CPU in the spin
  delay(5);
}

// handy:
// pio run -t upload --upload-port /dev/cu.usbserial-0001
// pio device monitor -p /dev/cu.usbserial-0001 -b 115200
