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

// Logging helpers: quiet in FIELD, verbose in LAB
#ifdef ENV_FIELD
#define LOGV(...) do {} while(0)
#define LOGI(...) do { Serial.printf(__VA_ARGS__); } while(0)
#else
#define LOGV(...) do { Serial.printf(__VA_ARGS__); } while(0)
#define LOGI(...) do { Serial.printf(__VA_ARGS__); } while(0)
#endif
#define LOGE(...) do { Serial.printf(__VA_ARGS__); } while(0)

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
#define WIND_RX_PIN       19       // converter TXD -> this pin
#define WIND_TX_PIN       -1         // not sending to sensor

// Timing knobs (Field vs Lab)
#ifdef ENV_FIELD
static const uint32_t ACK_WAIT_MS         = 2500;   // wait for ACK per attempt
static const uint32_t RETRY_BACKOFF_MIN   = 100;    // ms
static const uint32_t RETRY_BACKOFF_RANGE = 100;    // adds 0..99 ms
#else
static const uint32_t ACK_WAIT_MS         = 3000;
static const uint32_t RETRY_BACKOFF_MIN   = 200;
static const uint32_t RETRY_BACKOFF_RANGE = 300;    // adds 0..299 ms
#endif

// length of each measurement block in seconds
// LAB stays fast for bench; FIELD slows to 300s (5 min) between posts
#ifdef ENV_FIELD
static const uint16_t BLOCK_SECONDS = 300;
#else
static const uint16_t BLOCK_SECONDS = 30;
#endif

// Sampling + averaging windows (seconds)
static const uint16_t SAMPLE_PERIOD_SEC = 1;                 // 1 Hz
static const uint16_t POST_PERIOD_SEC   = BLOCK_SECONDS;     // e.g. 300s in FIELD
static const uint16_t MEAN_WINDOW_SEC   = 120;               // 2-minute mean
static const uint8_t  GUST_WINDOW_SEC   = 3;                 // 3-second gust definition
static const uint16_t GUST_LOOKBACK_SEC = BLOCK_SECONDS;     // 5-minute gust lookback
static_assert(GUST_LOOKBACK_SEC == POST_PERIOD_SEC, "gust lookback must match posting interval");

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

// -------- Battery sensing config --------

// ADC pin used for the divider: GPIO2 on the Heltec
#define PIN_BATTERY_SENSE   2

// ADC reference and resolution
static const float ADC_REF_V      = 3.3f;
static const float ADC_MAX_COUNTS = 4095.0f;

// Divider resistor values (ohms)
static const float R1_BAT = 220000.0f;  // battery -> node
static const float R2_BAT = 47000.0f;   // node -> GND
static const float R3_BAT = 100.0f;     // node -> GND (small load / RC tail)

// How many samples to average per reading
static const int   BATT_SAMPLES   = 16;

// Precompute gain from node voltage to battery voltage:
// Vbat = Vnode * (R1 + R2 + R3) / (R2 + R3)
static const float BAT_DIVIDER_GAIN =
    (R1_BAT + R2_BAT + R3_BAT) / (R2_BAT + R3_BAT);

// Simple LiFePO4 status thresholds (12.8V nominal 4S)
static const float BATT_OK_MIN   = 13.1f;  // >= OK
static const float BATT_LOW_MIN  = 12.7f;  // LOW if [12.7, 13.1)
static const float BATT_CRIT_MIN = 12.5f;  // CRIT if [12.5, 12.7)

// Map voltage to status string
static const char* batteryStatusFromVolts(float v) {
  if (v < 0.0f) {
    return "UNKNOWN";  // e.g. sensor not wired
  } else if (v < BATT_CRIT_MIN) {
    return "FAIL";     // essentially “this shouldn’t happen”
  } else if (v < BATT_LOW_MIN) {
    return "CRIT";
  } else if (v < BATT_OK_MIN) {
    return "LOW";
  } else {
    return "OK";
  }
}

// Read and scale battery voltage (in volts)
float readBatteryVolts() {
  // If the pin isn't wired yet, this will just read noise/0-ish.
  // That's fine until the divider is physically connected.
  uint32_t acc = 0;
  for (int i = 0; i < BATT_SAMPLES; ++i) {
    acc += (uint32_t)analogRead(PIN_BATTERY_SENSE);
    delay(2);  // tiny settle between samples
  }

  float raw    = (float)acc / (float)BATT_SAMPLES;         // 0..4095
  float v_node = (raw / ADC_MAX_COUNTS) * ADC_REF_V;       // node voltage
  float v_batt = v_node * BAT_DIVIDER_GAIN;                // scaled to battery

  return v_batt;
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
  LOGI("PEARL TX state: %d\n", st);
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
          LOGV("PEARL: ACK RX raw: %s\n", line.c_str());
        } else if (nonAckPrinted < 2) {
          LOGV("PEARL: non-ACK sample: %s\n", line.c_str());
          nonAckPrinted++;
        }

        uint32_t cnt_rx = 0;
        String status;
        if (parseAckLine(line, cnt_rx, status)) {
          if (cnt_rx == expect_cnt) {
            if (out_status) *out_status = status;
            LOGI("PEARL: got ACK cnt=%lu status=%s\n",
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


// ========== WIND BUFFER (2-min mean inside a 5-min posting block) ==========
// Mean is a 2-minute vector average; gust is a 3-second running mean over the full block.
// We keep up to POST_PERIOD_SEC samples (max 300 in FIELD)
static const uint16_t MAX_SAMPLES = POST_PERIOD_SEC;  // 300

static float speed_buf[MAX_SAMPLES];
static float dirx_buf[MAX_SAMPLES];
static float diry_buf[MAX_SAMPLES];

static uint16_t buf_count = 0;   // how many valid samples we have (<= MAX_SAMPLES)
static uint16_t buf_head  = 0;   // index where the next sample will be written

// For the 3-second running-mean gust over the whole block
static float max_gust_3s_ms = 0.0f;

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

  float rad = dir_deg * DEG_TO_RAD;
  float x = cosf(rad);
  float y = sinf(rad);

  // write at head
  speed_buf[buf_head] = spd_ms;
  dirx_buf[buf_head]  = x;
  diry_buf[buf_head]  = y;

  buf_head = (buf_head + 1) % MAX_SAMPLES;
  if (buf_count < MAX_SAMPLES) {
    buf_count++;
  }

  // 3-second running mean gust (only if we have at least 3 samples)
  if (buf_count >= GUST_WINDOW_SEC) {
    // indices of last 3 samples in the ring
    int i2 = (int(buf_head) + MAX_SAMPLES - 1) % MAX_SAMPLES;
    int i1 = (i2 + MAX_SAMPLES - 1) % MAX_SAMPLES;
    int i0 = (i1 + MAX_SAMPLES - 1) % MAX_SAMPLES;

    float mean3 = (speed_buf[i0] + speed_buf[i1] + speed_buf[i2]) / 3.0f;
    if (mean3 > max_gust_3s_ms) {
      max_gust_3s_ms = mean3;
    }
  }
}

// finalize the block → produce wind_avg, wind_max, wind_dir, then reset accumulators
static void finalizeBlock(float &wind_avg_ms,
                          float &wind_max_ms,
                          float &wind_dir_deg)
{
  if (buf_count == 0) {
    wind_avg_ms   = 0.0f;
    wind_max_ms   = 0.0f;
    wind_dir_deg  = 0.0f;
  } else {
    uint16_t n_mean = MEAN_WINDOW_SEC;
    if (n_mean > buf_count) {
      n_mean = buf_count;
    }

    float sum_speed = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;

    int idx = (int(buf_head) + MAX_SAMPLES - 1) % MAX_SAMPLES;
    for (uint16_t i = 0; i < n_mean; ++i) {
      sum_speed += speed_buf[idx];
      sum_x     += dirx_buf[idx];
      sum_y     += diry_buf[idx];

      idx = (idx + MAX_SAMPLES - 1) % MAX_SAMPLES;
    }

    wind_avg_ms = sum_speed / (float)n_mean;

    float avg_rad = atan2f(sum_y, sum_x);
    float avg_deg = avg_rad * RAD_TO_DEG;
    if (avg_deg < 0.0f) avg_deg += 360.0f;
    wind_dir_deg = avg_deg;

    wind_max_ms = max_gust_3s_ms;
  }

  // Reset gust + buffer state for the next 5-minute block
  buf_count       = 0;
  buf_head        = 0;
  max_gust_3s_ms  = 0.0f;
}

// timing state
static unsigned long next_sample_ms   = 0;
static unsigned long next_block_ms    = 0;
static uint16_t      seconds_in_block = 0;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(300);

#ifdef ENV_LAB
  LOGI("[ENV] LAB mode\n");
#elif defined(ENV_FIELD)
  LOGI("[ENV] FIELD mode\n");
#else
  LOGI("[ENV] UNKNOWN mode - no ENV_* macro set\n");
#endif
  LOGI("LORA_FREQ = %lu\n", (unsigned long)LORA_FREQ);
  LOGI("LORA_TX_POWER = %d\n", LORA_TX_POWER);

  // Battery ADC setup
  pinMode(PIN_BATTERY_SENSE, INPUT);
  analogReadResolution(12);        // 0..4095
  analogSetAttenuation(ADC_11db);  // allow ~3.3V at the node

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(loraFreqMHz());
  if (st != RADIOLIB_ERR_NONE) {
    LOGE("LoRa init failed (%d)\n", st);
    while (true) delay(1000);
  }

  #if USE_WINDSONIC
    Serial1.begin(WINDSONIC_BAUD, SERIAL_8N1, WIND_RX_PIN, WIND_TX_PIN);
    LOGV("WindSonic UART ready\n");
  #endif

  // MUST match Base
  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setOutputPower(LORA_TX_POWER);
  lora->setCRC(true);

  LOGI("PEARL: LoRa OK\n");
  LOGI("USE_WINDSONIC=%d BLOCK_SECONDS=%u\n",
       (int)USE_WINDSONIC, (unsigned)BLOCK_SECONDS);

  unsigned long t0 = millis();
  next_sample_ms = t0 + (uint32_t)SAMPLE_PERIOD_SEC * 1000UL;
  next_block_ms  = t0 + (uint32_t)POST_PERIOD_SEC * 1000UL;
}

// ========== LOOP ==========
// We do 1 Hz sampling into a block. When the block ends, we:
// - compute avg/dir/gust
// - build JSON
// - LoRa transmit once (with ACK/retry)
// - reset for next block
void loop() {
  unsigned long now = millis();

  // 1 Hz sampler (absolute schedule to avoid drift)
  while ((int32_t)(now - next_sample_ms) >= 0) {
    next_sample_ms += (uint32_t)SAMPLE_PERIOD_SEC * 1000UL;
    seconds_in_block++;

    float spd_ms, dir_deg;
    getSample(spd_ms, dir_deg);
    accumulateSample(spd_ms, dir_deg);

    LOGV("sample %3u: spd=%.2f m/s dir=%.1f deg\n",
         seconds_in_block, spd_ms, dir_deg);
  }

  // end of block?
  if ((int32_t)(now - next_block_ms) >= 0) {
    seconds_in_block = 0;
    next_block_ms += (uint32_t)POST_PERIOD_SEC * 1000UL;
    while ((int32_t)(now - next_block_ms) >= 0) {
      next_block_ms += (uint32_t)POST_PERIOD_SEC * 1000UL;
    }

    // finalize the rolling 2-min-style block
    float wind_avg_ms, wind_max_ms, wind_dir_deg;
    finalizeBlock(wind_avg_ms, wind_max_ms, wind_dir_deg);

    float vbatt = readBatteryVolts();

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

    const char* batt_status = batteryStatusFromVolts(sample.batt_v);

    // Build payload consistent with Base expectations:
    //   wind_avg  (knots)
    //   wind_max  (knots gust)
    //   wind_dir  (deg 0-360)
    //   cnt       (monotonic counter)
    //   batt      (V or -1.00 sentinel)
    //   batt_status ("OK"/"LOW"/"CRIT"/"FAIL"/"UNKNOWN")
    char buf[224];
    snprintf(buf, sizeof(buf),
      "{\"wind_avg\":%.1f,\"wind_max\":%.1f,"
      "\"wind_dir\":%.0f,\"cnt\":%lu,"
      "\"batt\":%.2f,\"batt_status\":\"%s\"}",
      wind_avg_kt,
      wind_max_kt,
      sample.wind_dir_deg,
      (unsigned long)sample.cnt,
      sample.batt_v,
      batt_status
    );

    String payload(buf);

    // === CRC16 APP-LAYER === compute over JSON bytes, append "*%04X"
    uint16_t crc = crc16_ccitt((const uint8_t*)payload.c_str(), payload.length());
    char trailer[6];                                   // "*" + 4 hex + '\0'
    snprintf(trailer, sizeof(trailer), "*%04X", (unsigned)crc);
    payload += trailer;                                // "{...}*9A5B"

    // Debug local printout
    LOGV("----- BLOCK RESULT -----\n");
    LOGV("avg   = %.2f m/s (%.1f kt)\n", sample.wind_avg_ms, wind_avg_kt);
    LOGV("gust  = %.2f m/s (%.1f kt)\n", sample.wind_max_ms, wind_max_kt);
    LOGV("dir   = %.1f deg\n",          sample.wind_dir_deg);
    LOGV("cnt   = %lu\n",               (unsigned long)sample.cnt);
    LOGV("batt  = %.2f V\n",            sample.batt_v);
    LOGV("[TX] %s\n", payload.c_str());
    LOGV("------------------------\n");

    // LoRa TX with ACK/Retry
    const int MAX_TX_ATTEMPTS = 3;
    int attempt   = 0;
    bool acked    = false;
    String ackStatus;

    while (attempt < MAX_TX_ATTEMPTS && !acked) {
      attempt++;
      LOGI("PEARL: TX attempt %d\n", attempt);
      txOnce(payload);

      // wait a bit longer for ACK to help bench/short-range timing
      if (waitForAck((uint32_t)sample.cnt, ACK_WAIT_MS, &ackStatus)) {
        acked = true;
        LOGI("PEARL: ACKed with status %s\n", ackStatus.c_str());
        break;
      }

      // small randomized backoff before retry
      uint32_t backoff = RETRY_BACKOFF_MIN + (esp_random() % RETRY_BACKOFF_RANGE);
      delay(backoff);
    }

    if (!acked) {
      LOGE("PEARL: no ACK received after retries\n");
    }

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
