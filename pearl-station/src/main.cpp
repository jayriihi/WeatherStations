/*
  Build-time configuration:

  - Select env in PlatformIO:
      base_field / base_lab
      pearl_field / pearl_lab

  - Macros:
      ENV_LAB / ENV_FIELD   : which environment is built
      LAB_MODE              : base-station only, selects test vs prod Apps Script
      LORA_FREQ             : LoRa center frequency (Hz)
      LORA_TX_POWER         : LoRa TX power (dBm)
      POST_BASE             : Google Apps Script endpoint (base-station only)
      API_KEY               : shared API key for Apps Script (base-station only)

  LAB builds:
    - Define ENV_LAB (both Pearl + Base)
    - Define LAB_MODE (Base only)
    - 914 MHz, low TX power (2 dBm)
    - Base posts to TEST sheet script URL

  FIELD builds:
    - Define ENV_FIELD
    - Do NOT define LAB_MODE
    - 915 MHz, normal TX power (14 dBm)
    - Base posts to PROD 'Pearl' sheet script URL
*/

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>
#include <string.h>
#include "Config.h"

static void getSample(float& spd_ms, float& dir_deg);     // you already have this
bool readWindsonicLine(float& spd_ms, float& gust_ms, float& dir_deg);


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
// --- WindSonic configuration ---
#define USE_WINDSONIC     1          // 1 = live sensor, 0 = test generator
#define WINDSONIC_BAUD    4800
#define WIND_RX_PIN       19        // converter TXD -> this pin
#define WIND_TX_PIN       -1         // not sending to sensor

// length of each measurement block in seconds
// use 10 for bench testing; set to 120 (2 min) in production
static const uint16_t BLOCK_SECONDS = 30;

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

//static void getSample(float& spd_ms, float& dir_deg);

// ---- helpers ----

// random float in [a,b]
static inline float frand(float a, float b) {
  return a + (b - a) * (float)esp_random() / (float)UINT32_MAX;
}

static inline void simulateSample(float& spd_ms, float& dir_deg) {
  spd_ms = frand(8.0f, 12.0f);
  if (frand(0.0f, 1.0f) > 0.97f) spd_ms = frand(13.0f, 17.0f);
  dir_deg = 70.0f + frand(-8.0f, 8.0f);
  if (dir_deg < 0) dir_deg += 360.0f;
  if (dir_deg >= 360.0f) dir_deg -= 360.0f;
}


// RadioLib wants a non-const String&
static inline void txOnce(String& body) {
  int st = lora->transmit(body);
  Serial.printf("PEARL TX state: %d\n", st);
}

// === ACK SUPPORT ===

static bool parseAckLine(const String& line, uint32_t& out_cnt, String& out_status) {
  int star = line.lastIndexOf('*');
  if (star < 0 || (line.length() - star - 1) < 4) return false;

  // verify CRC over the part before '*'
  String head = line.substring(0, star);
  char hex4[5];
  hex4[0] = line[star + 1];
  hex4[1] = line[star + 2];
  hex4[2] = line[star + 3];
  hex4[3] = line[star + 4];
  hex4[4] = '\0';

  unsigned rx_u = 0;
  sscanf(hex4, "%04X", &rx_u);
  uint16_t calc = crc16_ccitt((const uint8_t*)head.c_str(), head.length());
  if ((uint16_t)rx_u != calc) return false;

  // expected form: "ACK <cnt> <STATUS>"
  if (!head.startsWith("ACK ")) return false;
  int sp1 = head.indexOf(' ', 4);
  if (sp1 < 0) return false;

  String cntStr = head.substring(4, sp1);
  String stStr  = head.substring(sp1 + 1);
  out_cnt    = (uint32_t)cntStr.toInt();
  out_status = stStr;
  return true;
}

static bool waitForAck(uint32_t expect_cnt, uint32_t timeout_ms, String* out_status = nullptr) {
  unsigned long t0 = millis();

  // brief purge to flush any stale/loopback bytes
  lora->startReceive();
  unsigned long purge_end = t0 + 120;     // ~120 ms
  while (millis() < purge_end) {
    if (lora->getPacketLength() != 0) {
      String junk; (void)lora->readData(junk);
      // don't log junk here; it's expected on bench
    }
    delay(5);
  }

  // main wait
  lora->startReceive();
  delay(20);  // small settle after RX re-enable
  unsigned nonAckPrinted = 0;

  while ((millis() - t0) < timeout_ms) {
    if (lora->getPacketLength() != 0) {
      String line;
      int st = lora->readData(line);
      if (st == RADIOLIB_ERR_NONE) {
        line.trim();

        // optional debug — show first few non-ACK frames only
        if (line.startsWith("ACK ")) {
          Serial.printf("PEARL: ACK RX raw: %s\n", line.c_str());
        } else if (nonAckPrinted < 2) {
          Serial.printf("PEARL: non-ACK sample: %s\n", line.c_str());
          nonAckPrinted++;
        }

        uint32_t cnt_rx = 0;
        String status;
        if (parseAckLine(line, cnt_rx, status)) {
          if (cnt_rx == expect_cnt) {
            if (out_status) *out_status = status;
            Serial.printf("PEARL: got ACK cnt=%lu status=%s\n",
                          (unsigned long)cnt_rx, status.c_str());
            return true;
          }
        }
      }
    }
    delay(10);
  }
  return false;
}


// ========== ACCUMULATORS FOR THE ACTIVE BLOCK (2 min in production) ==========
static uint16_t sample_count = 0;
static float    sum_speed    = 0.0f;   // m/s
static float    max_gust_1s  = 0.0f;   // m/s
static float    sum_dir_x    = 0.0f;   // unitless
static float    sum_dir_y    = 0.0f;   // unitless

struct WindSample {
  float    wind_avg_ms;
  float    wind_max_ms;
  float    wind_dir_deg;
  float    batt_v;
  uint32_t cnt;
};

// Simple per-packet sequence counter (resets on reboot).
// Later we can persist this in NVS to survive resets.
static uint32_t g_cnt = 0;
static inline uint32_t nextCnt() { return g_cnt++; }

static void accumulateSample(float spd_ms, float dir_deg) {
  if (isnan(spd_ms) || isnan(dir_deg)) return;
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

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(300);

#ifdef ENV_LAB
  Serial.println("[ENV] LAB mode");
#elif defined(ENV_FIELD)
  Serial.println("[ENV] FIELD mode");
#else
  Serial.println("[ENV] UNKNOWN mode - no ENV_* macro set");
#endif
  Serial.print("LORA_FREQ = ");
  Serial.println((unsigned long)LORA_FREQ);
  Serial.print("LORA_TX_POWER = ");
  Serial.println(LORA_TX_POWER);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(loraFreqMHz());
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa init failed (%d)\n", st);
    while (true) delay(1000);
  }

  #if USE_WINDSONIC
    Serial1.begin(WINDSONIC_BAUD, SERIAL_8N1, WIND_RX_PIN, WIND_TX_PIN);
    Serial.println("WindSonic UART ready");
  #endif

  // MUST match Base
  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setOutputPower(LORA_TX_POWER);
  lora->setCRC(true);

  Serial.println("PEARL: LoRa OK");
  Serial.printf("USE_WINDSONIC=%d BLOCK_SECONDS=%u\n",
              (int)USE_WINDSONIC, (unsigned)BLOCK_SECONDS);
}

// timing state
static unsigned long last_sample_ms   = 0;
static uint16_t      seconds_in_block = 0;

// ========== LOOP ==========
// We do 1 Hz sampling into a block. When the block ends, we:
// - compute avg/dir/gust
// - build JSON
// - LoRa transmit once (with ACK/retry)
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

    // placeholder battery reading
    float vbatt = readBatteryVolts();   // -1.00 until wired

    WindSample sample{
      wind_avg_ms,
      wind_max_ms,
      wind_dir_deg,
      vbatt,
      nextCnt()
    };

    // convert avg + gust to knots for debug sanity
    float wind_avg_kt = sample.wind_avg_ms * 1.94384f;
    float wind_max_kt = sample.wind_max_ms * 1.94384f;

    // Build payload consistent with Base expectations:
    //   wind_avg  (knots)
    //   wind_max  (knots gust)
    //   wind_dir  (deg 0-360)
    //   cnt       (monotonic counter)
    //   batt      (V or -1.00 sentinel)
    char buf[224];
    snprintf(buf, sizeof(buf),
      "{\"wind_avg\":%.1f,\"wind_max\":%.1f,\"wind_dir\":%.0f,\"cnt\":%lu,\"batt\":%.2f}",
      wind_avg_kt,
      wind_max_kt,
      sample.wind_dir_deg,
      (unsigned long)sample.cnt,
      sample.batt_v
    );

    String payload(buf);

    // === CRC16 APP-LAYER === compute over JSON bytes, append "*%04X"
    uint16_t crc = crc16_ccitt((const uint8_t*)payload.c_str(), payload.length());
    char trailer[6];                                   // "*" + 4 hex + '\0'
    snprintf(trailer, sizeof(trailer), "*%04X", (unsigned)crc);
    payload += trailer;                                // "{...}*9A5B"

    // Debug local printout
    Serial.println("----- BLOCK RESULT -----");
    Serial.printf("avg   = %.2f m/s (%.1f kt)\n", sample.wind_avg_ms, wind_avg_kt);
    Serial.printf("gust  = %.2f m/s (%.1f kt)\n", sample.wind_max_ms, wind_max_kt);
    Serial.printf("dir   = %.1f deg\n",          sample.wind_dir_deg);
    Serial.printf("cnt   = %lu\n",               (unsigned long)sample.cnt);
    Serial.printf("batt  = %.2f V\n",            sample.batt_v);
    Serial.println("[TX] " + payload);
    Serial.println("------------------------\n");

    // LoRa TX with ACK/Retry
    const int MAX_TX_ATTEMPTS = 3;
    int attempt   = 0;
    bool acked    = false;
    String ackStatus;

    while (attempt < MAX_TX_ATTEMPTS && !acked) {
      attempt++;
      Serial.printf("PEARL: TX attempt %d\n", attempt);
      txOnce(payload);

      // wait a bit longer for ACK to help bench/short-range timing
      if (waitForAck((uint32_t)sample.cnt, 3000, &ackStatus)) {
        acked = true;
        Serial.printf("PEARL: ACKed with status %s\n", ackStatus.c_str());
        break;
      }

      // small randomized backoff before retry
      uint32_t backoff = 200 + (esp_random() % 300);  // 200..499 ms
      delay(backoff);
    }

    if (!acked) {
      Serial.println("PEARL: no ACK received after retries");
    }

    // jittered pause before starting next block sampling loop
    // (keeps us from slamming exactly on the minute every time)
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

// === NMEA CHECKSUM HELPERS =====================================

    // sentence should look like: "$...*HH" (HH = 2 hex chars, case-insensitive)
    static int hexDigitToInt(char c) {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
      if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
      return -1;
    }

    static bool parseHexByte(char hi, char lo, uint8_t &out) {
      int hiVal = hexDigitToInt(hi);
      int loVal = hexDigitToInt(lo);
      if (hiVal < 0 || loVal < 0) return false;
      out = static_cast<uint8_t>((hiVal << 4) | loVal);
      return true;
    }

    static bool nmeaChecksumValid(const char* sentence) {
      if (!sentence) return false;

      // Find '$'
      const char* p = strchr(sentence, '$');
      if (!p) return false;
      p++;  // move past '$'

      // Find '*'
      const char* star = strchr(p, '*');
      if (!star || !star[1] || !star[2]) {
        return false;  // need 2 hex chars after '*'
      }

      // Compute XOR from after '$' up to char before '*'
      uint8_t cs = 0;
      const char* q = p;
      while (q < star) {
        cs ^= static_cast<uint8_t>(*q);
        q++;
      }

      uint8_t msgCs = 0;
      if (!parseHexByte(star[1], star[2], msgCs)) {
        return false;
      }

      return cs == msgCs;
    }

// ==========================================================
// ========== WindSonic Sample Reader Integration ============
// ==========================================================

// Replaces simulator in getSample()

static void getSample(float& spd_ms, float& dir_deg) {
#if USE_WINDSONIC
  float gust_dummy = NAN;
  if (readWindsonicLine(spd_ms, gust_dummy, dir_deg)) {
    return;
  }
  // In LAB (or when sensor data is missing), fall back to the simulator to keep
  // the 1 Hz loop producing values instead of NaNs.
  #ifdef ENV_LAB
    simulateSample(spd_ms, dir_deg);
  #else
    spd_ms = NAN;
    dir_deg = NAN;
  #endif
#else
  // existing simulator
  simulateSample(spd_ms, dir_deg);
#endif
}

// ==========================================================
// ========== WindSonic UART Line Reader + Parser ============
// ==========================================================
static bool readLineFrom(Stream& ser, char* buf, size_t cap, uint32_t timeout_ms = 300) {
  uint32_t start = millis();
  size_t n = 0;
  while (millis() - start < timeout_ms) {
    while (ser.available()) {
      char c = (char)ser.read();
      if (c == '\r') continue;
      if (c == '\n') { buf[n] = '\0'; return n > 0; }
      if (n + 1 < cap) buf[n++] = c;
    }
  }
  return false;
}

bool readWindsonicLine(float& spd_ms, float& gust_ms, float& dir_deg) {
  char line[96];
  if (!readLineFrom(Serial1, line, sizeof(line))) return false;

  // --- If this is an NMEA sentence, enforce checksum ---
  if (line[0] == '$') {
    if (!nmeaChecksumValid(line)) {
      // Optional: bump a counter or print for debug
      // Serial.printf("Bad NMEA checksum: %s\n", line);
      return false;
    }

    // Strip "*HH" so it doesn't confuse strtok()
    char* star = strchr(line, '*');
    if (star) {
      *star = '\0';
    }
  }

  // --- Gill "Q" format: Q,dir_deg,speed_ms[,gust_ms] ---
  if ((line[0] == 'Q' || line[0] == 'q') && line[1] == ',') {
    char* p = line + 2;
    char* t1 = strtok(p, ",");
    char* t2 = strtok(nullptr, ",");
    char* t3 = strtok(nullptr, ",");
    if (t1 && t2) {
      dir_deg = atof(t1);
      spd_ms  = atof(t2);
      gust_ms = t3 ? atof(t3) : spd_ms;
      return (dir_deg >= 0 && dir_deg <= 360 && spd_ms >= 0);
    }
  }

  // --- NMEA MWV format: $--MWV,angle,R,speed,unit*CS (checksum already stripped) ---
  if (strstr(line, "MWV")) {
    char* p = (line[0] == '$') ? line + 1 : line;
    char* comma = strchr(p, ',');
    if (!comma) return false;
    p = comma + 1;

    char* angle = strtok(p, ",");
    strtok(nullptr, ",");                 // R/T
    char* spd   = strtok(nullptr, ",");
    char* unit  = strtok(nullptr, ",");   // N/M/K

    if (angle && spd && unit) {
      dir_deg = atof(angle);
      float v = atof(spd);
      spd_ms  = (*unit == 'N') ? v * 0.5144444f :
                (*unit == 'K') ? v / 3.6f : v;    // 'M' = m/s
      gust_ms = spd_ms;
      return (dir_deg >= 0 && dir_deg <= 360 && spd_ms >= 0);
    }
  }

  return false;
}

// handy:
// pio run -t upload --upload-port /dev/cu.usbserial-0001
// pio device monitor -p /dev/cu.usbserial-0001 -b 115200
