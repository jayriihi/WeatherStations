#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

static void getSample(float& spd_ms, float& dir_deg);
bool readWindsonicLine(float& spd_ms, float& gust_ms, float& dir_deg);

#ifdef LAB_MODE
static const bool ENABLE_BF_ACK = true;
static bool parseBackfillReq(const String& line,
                             uint32_t& from_cnt,
                             uint32_t& to_cnt);
static void handleBackfillReq(uint32_t from_cnt, uint32_t to_cnt);
static const bool MUTE_TX_LOGS = false;  // show TX/backfill logs for lab debug
#endif


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
#define USE_WINDSONIC     0          // 1 = live sensor, 0 = test generator
#define WINDSONIC_BAUD    4800
#define WIND_RX_PIN       19        // converter TXD -> this pin
#define WIND_TX_PIN       -1         // not sending to sensor

// length of each measurement block in seconds
// LAB bench testing: 30 seconds
#ifdef LAB_MODE
static const uint16_t BLOCK_SECONDS = 30;    // 30-second block in lab
#else
static const uint16_t BLOCK_SECONDS = 120;  // 2-min block in production
#endif

// --- Heltec WiFi LoRa 32 V3 (SX1262) pins ---
static const int PIN_NSS  = 8;   // CS
static const int PIN_DIO1 = 14;
static const int PIN_RST  = 12;  // NRST
static const int PIN_BUSY = 13;
static const int PIN_SCK  = 9;
static const int PIN_MISO = 11;
static const int PIN_MOSI = 10;

#ifdef LAB_MODE
static const float LORA_FREQ_MHZ = 914.0f;
#else
static const float LORA_FREQ_MHZ = 915.0f;
#endif

Module* modPtr = nullptr;
SX1262* lora   = nullptr;

// Battery sensing not wired yet.
// #define PIN_BATTERY_SENSE  1   // <- will become an ADC1 pin once installed

static const float ADC_REF_V       = 3.3f;
static const float ADC_MAX_COUNTS  = 4095.0f;
static const float DIVIDER_GAIN    = 4.6f;   // 360k / 100k divider ratio
static const int   BATT_SAMPLES    = 8;

struct Sample {
  uint32_t ts;        // epoch seconds
  uint32_t cnt;       // packet counter
  float    wind_avg;  // m/s
  float    wind_max;  // m/s
  uint16_t wind_dir;  // 0–359 degrees
  float    batt_v;    // volts
};

static const uint16_t SAMPLE_BUFFER_SIZE = 3600;  // 5 days @ 2-minute cadence
static Sample sampleBuffer[SAMPLE_BUFFER_SIZE];

static inline uint32_t currentTimestampSeconds() {
  time_t now = time(nullptr);
  if (now <= 0) {
    return (uint32_t)(millis() / 1000);
  }
  return (uint32_t)now;
}

// Simple per-packet sequence counter (resets on reboot).
static uint32_t g_cnt = 0;
static inline uint32_t nextCnt() { return g_cnt++; }


static void storeSample(const Sample& s) {
  size_t idx = s.cnt % SAMPLE_BUFFER_SIZE;
  sampleBuffer[idx] = s;
}

static Sample* findSampleByCnt(uint32_t cnt) {
  size_t idx = cnt % SAMPLE_BUFFER_SIZE;
  Sample* s = &sampleBuffer[idx];

  if (s->cnt != cnt) {
#ifdef LAB_MODE
    Serial.printf(
      "PEARL BF_SAMPLE miss cnt=%lu (idx=%u stored_cnt=%lu)\n",
      (unsigned long)cnt,
      (unsigned)idx,
      (unsigned long)s->cnt
    );
#endif
    return nullptr;
  }
  return s;
}

#ifdef LAB_MODE
static bool parseBackfillAck(const String& line, uint32_t& out_cnt) {
  int star = line.lastIndexOf('*');
  if (star < 0 || (line.length() - star - 1) < 4) return false;
  String head = line.substring(0, star);
  char hex4[5];
  for (int i = 0; i < 4; ++i) {
    char c = line[star + 1 + i];
    if (!isxdigit((unsigned char)c)) return false;
    hex4[i] = c;
  }
  hex4[4] = '\0';

  unsigned rx_u = 0;
  if (sscanf(hex4, "%04X", &rx_u) != 1) return false;
  uint16_t rx_crc = (uint16_t)rx_u;

  if (!head.startsWith("ACK_BF ")) return false;
  String cntStr = head.substring(7);
  cntStr.trim();
  if (cntStr.length() == 0) return false;
  out_cnt = (uint32_t)cntStr.toInt();

  uint16_t calc = crc16_ccitt((const uint8_t*)head.c_str(), head.length());
  return calc == rx_crc;
}

static bool waitForBackfillAck(uint32_t expect_cnt, uint32_t timeout_ms) {
  unsigned long t0 = millis();
  lora->startReceive();
  while ((millis() - t0) < timeout_ms) {
    if (lora->getPacketLength() != 0) {
      String line;
      int st = lora->readData(line);
      if (st == RADIOLIB_ERR_NONE) {
        line.trim();
        uint32_t ack_cnt = 0;
        if (parseBackfillAck(line, ack_cnt)) {
          if (ack_cnt == expect_cnt) {
            if (!MUTE_TX_LOGS) {
              Serial.printf("PEARL BF_ACK rx for cnt=%lu\n", (unsigned long)ack_cnt);
            }
            return true;
          }
        }
      }
    }
    delay(20);
  }
  return false;
}
#endif

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

#if !USE_WINDSONIC
// simulator sample (only compiled when USE_WINDSONIC == 0)
static inline void simulateSample(float& spd_ms, float& dir_deg) {
  spd_ms = frand(8.0f, 12.0f);
  if (frand(0.0f, 1.0f) > 0.97f) spd_ms = frand(13.0f, 17.0f);
  dir_deg = 70.0f + frand(-8.0f, 8.0f);
  if (dir_deg < 0) dir_deg += 360.0f;
  if (dir_deg >= 360.0f) dir_deg -= 360.0f;
}
#endif


// RadioLib wants a non-const String&
static inline void txOnce(String& body) {
  lora->standby();
  int st = lora->transmit(body);
#ifdef LAB_MODE
  if (!MUTE_TX_LOGS) {
    Serial.printf("PEARL TX state: %d\n", st);
  }
#else
  Serial.printf("PEARL TX state: %d\n", st);
#endif
  lora->startReceive();
}
// === ACK SUPPORT ===

static bool parseAckLine(const String& line, uint32_t& out_cnt, String& out_status) {
  String ackPart;

  // 1) Normal case: we actually see "ACK "
  int pos = line.indexOf("ACK ");
  if (pos >= 0) {
    ackPart = line.substring(pos);   // "ACK 0 OK*6A27Z"
  } else if (line.startsWith("CK ")) {
    // 2) LAB quirk: leading 'A' is missing; treat "CK 0 OK*..." as "ACK 0 OK*..."
    ackPart = "ACK " + line.substring(3);   // "ACK 0 OK*6A27Z"
  } else {
    return false;
  }

  // Strip optional CRC trailer "*HHHH" if present
  int star = ackPart.lastIndexOf('*');
  String head = (star >= 0) ? ackPart.substring(0, star) : ackPart;

  // Shape: "ACK <cnt> <STATUS>"
  if (!head.startsWith("ACK ")) return false;
  int sp1 = head.indexOf(' ', 4);
  if (sp1 < 0) return false;

  String cntStr = head.substring(4, sp1);
  String stStr  = head.substring(sp1 + 1);

  cntStr.trim();
  stStr.trim();

  if (cntStr.length() == 0) return false;

  out_cnt    = (uint32_t)cntStr.toInt();
  out_status = stStr;
  return true;
}

static bool waitForAck(uint32_t expect_cnt,
                       uint32_t timeout_ms,
                       String* out_status = nullptr) {
  unsigned long t0 = millis();

  // Purge any old packets
  lora->startReceive();
  unsigned long purge_end = t0 + 120;
  while (millis() < purge_end) {
    if (lora->getPacketLength() != 0) {
      String junk;
      (void)lora->readData(junk);
    }
    delay(5);
  }

  lora->startReceive();

  while ((millis() - t0) < timeout_ms) {
    if (lora->getPacketLength() != 0) {
      String line;
      int st = lora->readData(line);
      if (st == RADIOLIB_ERR_NONE) {
        line.trim();

#ifdef LAB_MODE
        // First: see if this is a BF_REQ that should trigger backfill
        uint32_t from_cnt = 0, to_cnt = 0;
        if (parseBackfillReq(line, from_cnt, to_cnt)) {
          if (!MUTE_TX_LOGS) {
            Serial.printf("PEARL ACK-window BF_REQ parsed: %lu->%lu\n",
                          (unsigned long)from_cnt,
                          (unsigned long)to_cnt);
          }
          handleBackfillReq(from_cnt, to_cnt);
          // Go back to RX and keep waiting for the ACK
          lora->startReceive();
          continue;
        }
#endif

        // Only log frames that look like ACK-ish things
        if (line.indexOf("ACK ") >= 0 || line.startsWith("CK ")) {
          Serial.printf("PEARL ACK-WINDOW candidate: '%s'\n", line.c_str());
        }

        uint32_t cnt_rx = 0;
        String   status;
        if (parseAckLine(line, cnt_rx, status)) {
          Serial.printf("PEARL: parsed ACK cnt=%lu status=%s\n",
                        (unsigned long)cnt_rx, status.c_str());
          if (cnt_rx == expect_cnt) {
            if (out_status) {
              *out_status = status;
            }
            Serial.printf("PEARL: got matching ACK for cnt=%lu\n",
                          (unsigned long)expect_cnt);
            return true;
          } else {
            Serial.printf("PEARL: ACK for different cnt=%lu (expected %lu)\n",
                          (unsigned long)cnt_rx,
                          (unsigned long)expect_cnt);
          }
        }
        // else: ignore junk silently
      } else {
        Serial.printf("PEARL ACK-WINDOW RX ERR: %d\n", st);
      }
    }

    delay(10);
  }

  Serial.println("PEARL: ACK wait timeout (no matching ACK)");
  return false;
}


#ifdef LAB_MODE

// -------- BF_REQ parsing --------
static bool parseBackfillReq(const String& line,
                             uint32_t& from_cnt,
                             uint32_t& to_cnt) {
  // We EXPECT something that was originally:
  //   "BF_REQ <from> <to>*HHHH"
  // but the leading "BF_REQ ..." may be mangled when received.

  bool bfHint = (line.indexOf("BF") >= 0 || line.indexOf("bf") >= 0 ||
                 line.indexOf("REQ") >= 0 || line.indexOf("req") >= 0);

#ifdef LAB_MODE
  auto logReject = [&](const char* reason) {
    if (bfHint) {
      Serial.printf("PEARL BF_REQ reject: %s: '%s'\n", reason, line.c_str());
    }
  };
#else
  auto logReject = [&](const char*) {};
#endif

  // Ignore clearly unrelated noise before spending cycles/logs
  if (line.startsWith("BF_SAMPLE")) {
    return false;
  }
  if (!bfHint) {
    return false;
  }

  // 1) Find trailer "*HHHH"
  int star = line.lastIndexOf('*');
  if (star < 0 || (line.length() - star - 1) < 4) {
    logReject("bad trailer");
    return false;
  }

  // 2) Parse the 4 hex digits after '*'
  char hex4[5];
  for (int i = 0; i < 4; ++i) {
    char c = line[star + 1 + i];
    if (!isxdigit((unsigned char)c)) {
      logReject("bad trailer");
      return false;
    }
    hex4[i] = c;
  }
  hex4[4] = '\0';

  unsigned rx_u = 0;
  if (sscanf(hex4, "%04X", &rx_u) != 1) {
    logReject("bad trailer");
    return false;
  }
  uint16_t rx_crc = (uint16_t)rx_u;

  // 3) Head = everything before '*'
  String head = line.substring(0, star);
  head.trim();   // drop leading/trailing spaces

  // We now expect head to end with: "<from> <to>"
  // e.g. "00F360 5 6" OR "BF_REQ 5 6"
  int lastSpace = head.lastIndexOf(' ');
  if (lastSpace < 0) {
    logReject("missing to-cnt");
    return false;
  }
  String toStr = head.substring(lastSpace + 1);
  toStr.trim();

  String beforeTo = head.substring(0, lastSpace);
  beforeTo.trim();

  int prevSpace = beforeTo.lastIndexOf(' ');
  if (prevSpace < 0) {
    logReject("missing from-cnt");
    return false;
  }
  String fromStr = beforeTo.substring(prevSpace + 1);
  fromStr.trim();

  if (fromStr.length() == 0 || toStr.length() == 0) {
    logReject("empty from/to");
    return false;
  }

  auto digitsOnly = [](const String& s) {
    for (size_t i = 0; i < s.length(); ++i) {
      if (!isdigit((unsigned char)s[i])) return false;
    }
    return s.length() > 0;
  };
  if (!digitsOnly(fromStr) || !digitsOnly(toStr)) {
    logReject("non-numeric from/to");
    return false;
  }

  from_cnt = (uint32_t)fromStr.toInt();
  to_cnt   = (uint32_t)toStr.toInt();

  // 4) Rebuild canonical string used on Base for CRC:
  //    "BF_REQ <from> <to>"
  String canonical = "BF_REQ ";
  canonical += String(from_cnt);
  canonical += " ";
  canonical += String(to_cnt);

  uint16_t calc = crc16_ccitt(
      (const uint8_t*)canonical.c_str(),
      canonical.length()
  );

  if (calc != rx_crc) {
    // Not one of our BF_REQ frames
    char reason[80];
    snprintf(reason, sizeof(reason), "CRC mismatch calc=%04X rx=%04X",
             (unsigned)calc, (unsigned)rx_crc);
    logReject(reason);
    return false;
  }

  if (!MUTE_TX_LOGS) {
    Serial.printf("PEARL: parsed BF_REQ %lu->%lu (crc OK)\n",
                  (unsigned long)from_cnt,
                  (unsigned long)to_cnt);
  }
  return true;
}

// -------- BF_SAMPLE sender --------
static void sendBackfillSample(const Sample& s) {
  float wind_avg_kn = s.wind_avg * 1.94384f;
  float wind_max_kn = s.wind_max * 1.94384f;

  String head = "BF_SAMPLE ";
  head += String(s.cnt);
  head += " ";
  head += String(wind_avg_kn, 1);
  head += " ";
  head += String(wind_max_kn, 1);
  head += " ";
  head += String((int)s.wind_dir);
  head += " ";
  head += String(s.batt_v, 2);

  uint16_t crc = crc16_ccitt((const uint8_t*)head.c_str(), head.length());
  char trailer[6];
  snprintf(trailer, sizeof(trailer), "*%04X", (unsigned)crc);
  String frame = head + trailer;

  lora->standby();
  int st = lora->transmit(frame);
  if (!MUTE_TX_LOGS) {
    Serial.printf("PEARL BF_SAMPLE cnt=%lu state=%d\n",
                  (unsigned long)s.cnt, st);
  }
#ifdef LAB_MODE
  if (ENABLE_BF_ACK) {
    bool ok = waitForBackfillAck(s.cnt, 1800);
    if (!ok) {
      delay(150);
      st = lora->transmit(frame);
      if (!MUTE_TX_LOGS) {
        Serial.printf("PEARL BF_SAMPLE retry cnt=%lu state=%d\n",
                      (unsigned long)s.cnt, st);
      }
      (void)waitForBackfillAck(s.cnt, 1200);
    }
  }
#endif
  lora->startReceive();
}

// -------- BF_REQ handler --------
static void handleBackfillReq(uint32_t from_cnt, uint32_t to_cnt) {
  // small pause after BF_REQ RX so Base can flip back to RX
  delay(150);
  for (uint32_t cnt = from_cnt; cnt <= to_cnt; ++cnt) {
    Sample* s = findSampleByCnt(cnt);
    if (!s) {
#ifdef LAB_MODE
      Serial.printf("PEARL BF_SAMPLE miss cnt=%lu (not in buffer)\n",
                    (unsigned long)cnt);
#endif
      continue;
    }
    sendBackfillSample(*s);
#ifdef LAB_MODE
    Serial.printf("PEARL BF_SAMPLE send cnt=%lu\n", (unsigned long)s->cnt);
#endif
    delay(500);  // spacing so Base can RX sequential backfill frames
  }
}

#endif  // LAB_MODE



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

  #ifdef LAB_MODE
  Serial.println("PEARL: LAB_MODE is ON");
#else
  Serial.println("PEARL: LAB_MODE is OFF");
#endif

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(LORA_FREQ_MHZ);
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
  lora->setRxBoostedGainMode(true);

#ifdef LAB_MODE
  // Lab / porch: keep TX power very low, we don't want it blasting through the house
  lora->setOutputPower(0);     // 0 dBm
#else
  // Production / island: strong, but not absolute max
  lora->setOutputPower(20);    // ~20 dBm
#endif

  lora->setCRC(true);
  lora->startReceive();

  Serial.println("PEARL: LoRa OK");
  Serial.printf("USE_WINDSONIC=%d BLOCK_SECONDS=%u\n",
              (int)USE_WINDSONIC, (unsigned)BLOCK_SECONDS);
}

// timing state
static unsigned long last_sample_ms   = 0;
static uint16_t      seconds_in_block = 0;
static unsigned long dl_debug_until   = 0;   // window to log all RX frames
static uint16_t      dl_debug_cap     = 0;   // cap lines during debug window
static uint16_t      dl_debug_seen    = 0;   // frames seen in debug window
static uint16_t      dl_debug_bf      = 0;   // BF-ish frames seen in debug window
static String        idle_last_line;
static uint16_t      idle_last_repeat = 0;

// ========== LOOP ==========
// We do 1 Hz sampling into a block. When the block ends, we:
// - compute avg/dir/gust
// - build JSON
// - LoRa transmit once (with ACK/retry)
// - reset for next block

void loop() {
  unsigned long now = millis();

#ifdef LAB_MODE
  // Idle downlink listener: first priority is BF_REQ, everything else is optional
  if (lora->getPacketLength() != 0) {
    String line;
    int st = lora->readData(line);
    if (st == RADIOLIB_ERR_NONE) {
      line.trim();

      bool inDebugWindow = (dl_debug_until != 0 && millis() < dl_debug_until);
      bool bfLike = (line.indexOf("BF_REQ") >= 0 ||
                     line.indexOf("BF") >= 0 || line.indexOf("bf") >= 0 ||
                     line.indexOf("REQ") >= 0 || line.indexOf("req") >= 0);
      uint32_t from_cnt = 0, to_cnt = 0;
      if (parseBackfillReq(line, from_cnt, to_cnt)) {
        // Always show when we actually parse a BF_REQ
        Serial.printf("PEARL idle BF_REQ parsed: %lu->%lu\n",
                      (unsigned long)from_cnt,
                      (unsigned long)to_cnt);
        handleBackfillReq(from_cnt, to_cnt);
      } else {
        bool allowLog = !inDebugWindow || (dl_debug_cap > 0);
        bool suppressNoise = line.startsWith("BF_SAMPLE") ||
                             line.indexOf("wind_avg") >= 0 ||
                             line.indexOf("{\"wind_avg\"") >= 0 ||
                             line.startsWith("68400");
        if (allowLog && !suppressNoise) {
          if (line == idle_last_line) {
            idle_last_repeat++;
            if (idle_last_repeat % 20 == 0) {
              Serial.printf("PEARL idle RX RAW (x%u): '%s'\n",
                            (unsigned)idle_last_repeat, line.c_str());
            }
          } else {
            if (idle_last_repeat > 1) {
              Serial.printf("PEARL idle RX RAW (x%u): '%s'\n",
                            (unsigned)idle_last_repeat, idle_last_line.c_str());
            } else if (idle_last_repeat == 1) {
              Serial.printf("PEARL idle RX RAW: '%s'\n", idle_last_line.c_str());
            }
            idle_last_line   = line;
            idle_last_repeat = 1;
            Serial.printf("PEARL idle RX RAW: '%s'\n", line.c_str());
          }
        }
        if (inDebugWindow) {
          dl_debug_seen++;
          if (bfLike) dl_debug_bf++;
          if (dl_debug_cap > 0) {
            dl_debug_cap--;
            if (dl_debug_cap == 0) {
              Serial.println("PEARL idle RX debug window exhausted");
            }
          }
        }
      }
    } else if (!MUTE_TX_LOGS) {
      Serial.printf("PEARL idle RX err: %d\n", st);
    }

    lora->startReceive();
  }
#endif

// 1 Hz sampler
if (now - last_sample_ms >= 1000) {
  last_sample_ms = now;
  seconds_in_block++;

  float spd_ms, dir_deg;
  getSample(spd_ms, dir_deg);
  accumulateSample(spd_ms, dir_deg);

#ifdef LAB_MODE
  if (!MUTE_TX_LOGS) {
    Serial.printf("sample %3u: spd=%.2f m/s dir=%.1f deg\n",
                  seconds_in_block, spd_ms, dir_deg);
  }
#else
  Serial.printf("sample %3u: spd=%.2f m/s dir=%.1f deg\n",
                seconds_in_block, spd_ms, dir_deg);
#endif
}

  // end of block?
  if (seconds_in_block >= BLOCK_SECONDS) {
    seconds_in_block = 0;

  // finalize the rolling 2-min-style block
  float wind_avg_ms, wind_max_ms, wind_dir_deg;
  finalizeBlock(wind_avg_ms, wind_max_ms, wind_dir_deg);

  // placeholder battery reading
  float vbatt = readBatteryVolts();   // -1.00 until wired

// decide what this packet's counter will be
uint32_t thisCnt = nextCnt();


  // build the in-RAM sample & store it, regardless of whether we TX
  WindSample sample{
    wind_avg_ms,
    wind_max_ms,
    wind_dir_deg,
    vbatt,
    thisCnt
  };

  Sample currentSample;
  currentSample.ts       = currentTimestampSeconds();
  currentSample.cnt      = thisCnt;
  currentSample.wind_avg = sample.wind_avg_ms;
  currentSample.wind_max = sample.wind_max_ms;
  currentSample.wind_dir = (uint16_t)sample.wind_dir_deg;
  currentSample.batt_v   = sample.batt_v;
  storeSample(currentSample);

#ifdef LAB_MODE
if (currentSample.cnt == 2 || currentSample.cnt == 3) {
  Serial.printf(
    "PEARL storeSample cnt=%lu idx=%u avg=%.1f max=%.1f dir=%u\n",
    (unsigned long)currentSample.cnt,
    (unsigned)(currentSample.cnt % SAMPLE_BUFFER_SIZE),
    currentSample.wind_avg,
    currentSample.wind_max,
    (unsigned)currentSample.wind_dir
  );
}
#endif

  // No forced gaps; allow natural gap detection from Base

    // convert avg + gust to knots for debug sanity
    float wind_avg_kt = sample.wind_avg_ms * 1.94384f;
    float wind_max_kt = sample.wind_max_ms * 1.94384f;

    // Build payload consistent with Base expectations:
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

#if defined(LAB_MODE)
    // LAB: keep it simple for now – TX once, no ACK wait
    if (!MUTE_TX_LOGS) {
      Serial.println("PEARL: TX (LAB, no ACK wait)");
    }
    txOnce(payload);
#else
    // PROD: still fire-and-forget for now (we can add retries later)
    txOnce(payload);
#endif


    // jittered pause before starting next block sampling loop
    uint32_t jitter = 200 + (esp_random() % 400);  // 200..599 ms
    delay(jitter);
  }

  // tiny idle delay so we aren't burning 100% CPU in the spin
  delay(5);

#ifdef LAB_MODE
  if (dl_debug_until != 0 && millis() >= dl_debug_until) {
    Serial.printf("PEARL idle RX debug window summary: %u frames seen, %u BF-ish\n",
                  (unsigned)dl_debug_seen, (unsigned)dl_debug_bf);
    dl_debug_until = 0;
    dl_debug_cap   = 0;
    dl_debug_seen  = 0;
    dl_debug_bf    = 0;
  }
#endif
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
  spd_ms = NAN;
  dir_deg = NAN;
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
// pio device list
// pio run -t upload --upload-port /dev/cu.usbserial-0001
// pio device monitor -p /dev/cu.usbserial-0001 -b 115200

// LAB_MODE summary:
// - BLOCK_SECONDS = 30 (one packet every ~30 s)
// - BF_REQ frames from Base are logged as "PEARL idle RX RAW: '...'"
//   and, when valid, "PEARL idle BF_REQ parsed: from->to"
// - For each valid BF_REQ, Pearl sends BF_SAMPLE frames and logs
//   "PEARL BF_SAMPLE send cnt=..."
