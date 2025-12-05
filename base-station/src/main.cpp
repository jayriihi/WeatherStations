#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_attr.h"
#include <time.h>
#include <ctype.h>
#include <string.h>

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



// === LEDS ===
// Heltec V3 usually exposes an onboard LED via LED_BUILTIN.
// If not defined by the core, fall back to GPIO 35.

// === LED ===
// Onboard white LED on GPIO 35
static const int LED_HEARTBEAT = 35;

static uint32_t g_lastHeartbeatMs = 0;
static bool     g_hbState         = false;

static uint32_t g_rxHoldUntilMs = 0;


// ---------- Wi-Fi ----------
static void connectWiFi(uint32_t maxWaitMs = 15000) {
  // List of networks to try, in order of preference
  const char* SSIDS[] = {
    "Hapenny",
    "Spanky’s House",
    "Jaysphone",
  };
  const char* PASSES[] = {
    "hapennyhouse",
    "131Glenmont",
    "Riihiluoma",
  };
  const int WIFI_COUNT = sizeof(SSIDS) / sizeof(SSIDS[0]);

  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);

  // Split the total wait time across all networks
  uint32_t perNetWait = (WIFI_COUNT > 0) ? maxWaitMs / WIFI_COUNT : maxWaitMs;
  if (perNetWait < 2000) perNetWait = 3000;  // don't bother with super tiny timeouts

  for (int i = 0; i < WIFI_COUNT; ++i) {
    Serial.printf("WiFi: trying SSID '%s'\n", SSIDS[i]);
    WiFi.begin(SSIDS[i], PASSES[i]);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < perNetWait) {
      delay(250);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWiFi OK ssid=%s ip=%s\n",
                    SSIDS[i],
                    WiFi.localIP().toString().c_str());
      return;
    }

    Serial.println("\nWiFi: failed on this SSID, trying next");
    WiFi.disconnect(true, true);
    delay(200);
  }

  Serial.println("WiFi: not connected to any configured network, continuing offline");
}



// Health counters
static uint32_t g_crc_ok   = 0;
static uint32_t g_crc_fail = 0;
static uint32_t g_rf_dup   = 0;
static uint32_t g_boot_id  = (uint32_t)esp_random();
static uint32_t g_start_ms = 0;

// ---------- Google Apps Script endpoint + API key ----------
#ifdef LAB_MODE
  // LAB: writes to 'test' tab
  #define POST_BASE "https://script.google.com/macros/s/AKfycbw04W7Gro8RZLqBTO1T64v_6ii_u_Sa5rm2CY-NmL3s-tl4hnEIZXStDCfbS3oVdJ5kTg/exec"
#else
  // PROD: writes to 'Pearl' tab
  #define POST_BASE "https://script.google.com/macros/s/AKfycbxkcVc6BP2oJtQcA8BcAvWwrIU9eDIGanyI5yWvj7GwHgISrCKnozDGZMXJ0b0xHGFu/exec"
#endif

#define API_KEY "jI6nrJ2KTsgK0SDu"



// Heltec WiFi LoRa 32 V3 (SX1262) pins
static const int PIN_NSS  = 8;
static const int PIN_DIO1 = 14;
static const int PIN_RST  = 12;
static const int PIN_BUSY = 13;
static const int PIN_SCK  = 9;
static const int PIN_MISO = 11;
static const int PIN_MOSI = 10;

// RADIO & GLOBALS
Module* modPtr = nullptr;
SX1262* lora   = nullptr;

static int32_t  g_lastCntPosted = -1;
static String   g_lastRx;
static uint32_t g_lastRxMs = 0;
static String   g_lastBody;
static uint32_t g_lastPostMs = 0;

static uint16_t g_dupCount   = 0;
static uint32_t g_dupLastLog = 0;

static const uint32_t MIN_POST_MS = 10000UL;
static int32_t  g_inflightCnt = -1;
static uint32_t g_inflightMs  = 0;
static const uint32_t INFLIGHT_GUARD_MS = 5000UL;

// Drop/miss tracking (per-packet seq cnt)
static uint32_t g_last_cnt      = 0;
static bool     g_have_last_cnt = false;
static uint32_t g_drop_total    = 0;
static uint32_t g_total_packets = 0;
static uint32_t g_drop_gap      = 0;
static double   g_drop_rate     = 0.0;
static Preferences g_prefs;

static uint32_t g_delivered_total  = 0;

#ifdef LAB_MODE
static bool g_sentDlTest = false;
static unsigned long g_lastDlTestMs = 0;
static uint8_t g_dlTestAttempts = 0;
#endif
static void resetRadioForRx();

#ifdef LAB_MODE
struct BackfillState {
  bool     active      = false;
  uint32_t from_cnt    = 0;
  uint32_t to_cnt      = 0;
  uint32_t next_req_ms = 0;
  uint8_t  attempts    = 0;
  uint32_t filled_high = 0;
  uint32_t rx_count    = 0;
  static const uint8_t MAX_WIN = 32;
  bool     received[MAX_WIN] = {0};
  uint8_t  win_len    = 0;
};
static BackfillState g_backfill;

static bool tryHandleBackfillFrame(const String& line);
static bool sendBackfillReq(uint32_t from_cnt, uint32_t to_cnt);
static void requestBackfillRange(uint32_t from_cnt, uint32_t to_cnt);
static void maybeSendPendingBackfill();
static void sendBackfillAck(uint32_t cnt);
#endif


// TIME
static void initTimeUTC() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("NTP: syncing");
  for (int i = 0; i < 40; ++i) {
    if (time(nullptr) > 1600000000) { Serial.println("\nNTP OK"); return; }
    Serial.print(".");
    delay(250);
  }
  Serial.println("\nNTP: failed (will still try).");
}
static uint32_t minuteEpoch(time_t now) { return (uint32_t)((now / 60) * 60); }

// --- LoRa frequency selection ---
#ifdef LAB_MODE
static const float LORA_FREQ_MHZ = 914.0f;   // LAB: Meshtastic test pair
static const int   DOWNLINK_TX_DBM = 0;      // ACK/BF_REQ/DL_TEST power for LAB
static const bool  ENABLE_ACK = false;       // disable ACK to simplify BF_REQ testing
static const bool  LOG_POST_BODIES = false;  // mute verbose POST bodies in lab
static const bool  ENABLE_BF_ACK = true;
#else
static const float LORA_FREQ_MHZ = 915.0f;   // Porch / production
static const int   DOWNLINK_TX_DBM = 0;    // keep downlink low in production
static const bool  ENABLE_ACK = true;
static const bool  LOG_POST_BODIES = true;
#endif

// HARD RX RESET
static void resetRadioForRx() {
  lora->standby();
  delay(5);

  lora->begin(LORA_FREQ_MHZ);
  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setRxBoostedGainMode(true);
  lora->setCRC(true);
  lora->setOutputPower(DOWNLINK_TX_DBM);


  lora->startReceive();
}

// HTTP POST
static int postOnce(const String& body) {
  if (WiFi.status() != WL_CONNECTED) return -1;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.useHTTP10(true);
  http.setReuse(false);
  http.setTimeout(15000);

  String url = String(POST_BASE) + "?api_key=" + API_KEY;
  if (!http.begin(client, url)) return -1;

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Accept", "*/*");
  http.addHeader("User-Agent", "curl/8.0.1");
  http.addHeader("Accept-Encoding", "identity");
  http.addHeader("Connection", "close");

  Serial.printf("POST %s\nBODY: %s\n", url.c_str(), body.c_str());

  int code = http.POST(body);
  String resp = http.getString();
  http.end();

  // Only log real problems: network/<0>, timeout 408, 5xx, and non-400 client errors
  if (code < 0 || code == 408 || (code >= 500 && code < 600) || (code >= 400 && code < 500 && code != 400)) {
    Serial.printf("HTTP %d\n", code);
    if (code >= 500) {
      Serial.println(resp);
    }
  }

  return code;
}

// Build the form body for posting a Pearl sample (shared by live and backfill).
static String buildPostBody(float wind_avg_kn,
                            float wind_max_kn,
                            float wind_dir_deg,
                            long  cnt,
                            float batt_v,
                            float rssi_f,
                            float snr_f) {
  char buf[420];
  snprintf(buf, sizeof(buf),
           "wind_avg=%.1f&wind_max=%.1f&wind_dir=%.0f"
           "&cnt=%ld&batt=%.2f&rssi=%.1f&snr=%.1f"
           "&crc_ok=%lu&crc_fail=%lu&rf_dup=%lu&uptime_s=%lu&boot_id=%lu&fw=base_1"
           "&drop_gap=%ld&drop_total=%lu&drop_rate=%.4f",
           wind_avg_kn, wind_max_kn, wind_dir_deg,
           cnt, batt_v, rssi_f, snr_f,
           (unsigned long)g_crc_ok, (unsigned long)g_crc_fail, (unsigned long)g_rf_dup,
           (unsigned long)((millis() - g_start_ms) / 1000UL), (unsigned long)g_boot_id,
           (long)g_drop_gap, (unsigned long)g_drop_total, g_drop_rate);
  return String(buf);
}

#ifdef LAB_MODE
static bool tryHandleBackfillFrame(const String& line) {
  int star = line.lastIndexOf('*');
  if (star < 0 || (line.length() - star - 1) < 4) return false;

  String head = line.substring(0, star);
  char hex4[5];
  for (int i = 0; i < 4; ++i) {
    hex4[i] = line[star + 1 + i];
    if (!isxdigit((unsigned char)hex4[i])) return false;
  }
  hex4[4] = '\0';

  unsigned rx_u = 0;
  sscanf(hex4, "%04X", &rx_u);
  uint16_t calc = crc16_ccitt((const uint8_t*)head.c_str(), head.length());
  if ((uint16_t)rx_u != calc) {
#ifdef LAB_MODE
    Serial.printf("BASE BF_SAMPLE CRC fail (calc=%04X rx=%04X) line='%s'\n",
                  (unsigned)calc, (unsigned)rx_u, line.c_str());
#endif
    return false;
  }

  if (!head.startsWith("BF_SAMPLE ")) return false;

  const char* cstr = head.c_str();
  char tag[16];
  unsigned long cnt_ul = 0;
  float wind_avg_kn = 0.0f, wind_max_kn = 0.0f, wind_dir_deg = 0.0f, batt_v = 0.0f;
  int parsed = sscanf(cstr, "%15s %lu %f %f %f %f",
                      tag, &cnt_ul, &wind_avg_kn, &wind_max_kn, &wind_dir_deg, &batt_v);
  if (parsed != 6) return false;

  String body = buildPostBody(wind_avg_kn, wind_max_kn, wind_dir_deg,
                              (long)cnt_ul, batt_v, -1.0f, -1.0f);

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(2000);
  }

  if (LOG_POST_BODIES) {
    Serial.printf("BASE BF_SAMPLE POST cnt=%lu body=%s\n", (unsigned long)cnt_ul, body.c_str());
  }
  int code = postOnce(body);
  if (code == 200 || code == 201 || code == 400) {
    g_lastBody   = body;
    g_lastPostMs = millis();
    if (g_backfill.active &&
        cnt_ul >= g_backfill.from_cnt && cnt_ul <= g_backfill.to_cnt) {
      uint32_t idx = cnt_ul - g_backfill.from_cnt;
      if (idx < BackfillState::MAX_WIN && !g_backfill.received[idx]) {
        g_backfill.received[idx] = true;
        g_backfill.rx_count++;
      }
      if (cnt_ul > g_backfill.filled_high) g_backfill.filled_high = (uint32_t)cnt_ul;
      uint32_t expected = g_backfill.to_cnt - g_backfill.from_cnt + 1;
      if (g_backfill.rx_count >= expected) {
        Serial.printf("BASE BF_SAMPLE range complete %lu->%lu (rx=%lu)\n",
                      (unsigned long)g_backfill.from_cnt,
                      (unsigned long)g_backfill.to_cnt,
                      (unsigned long)g_backfill.rx_count);
        g_backfill.active   = false;
        g_backfill.attempts = 0;
        g_backfill.rx_count = 0;
      }
    }
    if (!LOG_POST_BODIES) {
      Serial.printf("BASE BF_SAMPLE POST cnt=%lu code=%d\n",
                    (unsigned long)cnt_ul, code);
    }
    #ifdef LAB_MODE
    sendBackfillAck((uint32_t)cnt_ul);
    #endif
  } else {
    Serial.printf("BASE BF_SAMPLE post failed (code=%d)\n", code);
  }
  return true;
}

static bool sendBackfillReq(uint32_t from_cnt, uint32_t to_cnt) {
  String head = "BF_REQ " + String(from_cnt) + " " + String(to_cnt);
  uint16_t crc = crc16_ccitt((const uint8_t*)head.c_str(), head.length());
  char trailer[6];
  snprintf(trailer, sizeof(trailer), "*%04X", (unsigned)crc);
  String frame = head + trailer;

  Serial.printf("BASE BF_REQ TX RAW: '%s'\n", frame.c_str());

  lora->standby();
  int st = lora->transmit(frame);
  Serial.printf("BASE BF_REQ %lu->%lu state=%d\n",
                (unsigned long)from_cnt, (unsigned long)to_cnt, st);
  resetRadioForRx();
  return st == RADIOLIB_ERR_NONE;
}

static void requestBackfillRange(uint32_t from_cnt, uint32_t to_cnt) {
  if (from_cnt > to_cnt) return;
  if (!g_backfill.active) {
    g_backfill.active      = true;
    g_backfill.from_cnt    = from_cnt;
    g_backfill.to_cnt      = to_cnt;
    g_backfill.filled_high = 0;
    g_backfill.attempts    = 0;
    g_backfill.rx_count    = 0;
    uint32_t span = to_cnt - from_cnt + 1;
    g_backfill.win_len = (span <= BackfillState::MAX_WIN) ? (uint8_t)span : BackfillState::MAX_WIN;
    for (uint8_t i = 0; i < g_backfill.win_len; ++i) g_backfill.received[i] = false;
    g_backfill.next_req_ms = 0;  // send ASAP
    Serial.printf("BASE BF_REQ scheduled %lu->%lu\n",
                  (unsigned long)from_cnt, (unsigned long)to_cnt);
  } else {
    Serial.printf("BASE BF_REQ already active %lu->%lu, skipping new range %lu->%lu\n",
                  (unsigned long)g_backfill.from_cnt,
                  (unsigned long)g_backfill.to_cnt,
                  (unsigned long)from_cnt,
                  (unsigned long)to_cnt);
  }
}

static void maybeSendPendingBackfill() {
  if (!g_backfill.active) return;
  unsigned long now = millis();
  if (now < g_backfill.next_req_ms) return;

  // stop after a few tries to avoid flooding
  const uint8_t MAX_REQ_ATTEMPTS = 6;
  uint32_t expected = g_backfill.to_cnt - g_backfill.from_cnt + 1;
  if (g_backfill.attempts >= MAX_REQ_ATTEMPTS) {
    if (g_backfill.rx_count < expected) {
      Serial.printf("BASE BF_REQ incomplete %lu->%lu rx=%lu/%lu, retrying\n",
                    (unsigned long)g_backfill.from_cnt,
                    (unsigned long)g_backfill.to_cnt,
                    (unsigned long)g_backfill.rx_count,
                    (unsigned long)expected);
      g_backfill.attempts = 0;
      g_backfill.next_req_ms = now + 8000UL;  // backoff before next round
      return;
    } else {
      Serial.printf("BASE BF_REQ attempts exhausted for %lu->%lu (rx=%lu)\n",
                    (unsigned long)g_backfill.from_cnt,
                    (unsigned long)g_backfill.to_cnt,
                    (unsigned long)g_backfill.rx_count);
      g_backfill.active = false;
      return;
    }
  }

  if (sendBackfillReq(g_backfill.from_cnt, g_backfill.to_cnt)) {
    g_backfill.attempts++;
    Serial.printf("BASE BF_REQ attempt %u for %lu->%lu\n",
                  (unsigned)g_backfill.attempts,
                  (unsigned long)g_backfill.from_cnt,
                  (unsigned long)g_backfill.to_cnt);
    g_backfill.next_req_ms = now + 5000UL;  // retry window

    // brief listen window for immediate responses
    unsigned long t0 = millis();
    while ((millis() - t0) < 5000UL) {
      if (lora->getPacketLength() != 0) {
        String line;
        int st = lora->readData(line);
        if (st == RADIOLIB_ERR_NONE) {
          line.trim();
          (void)tryHandleBackfillFrame(line);
        }
      }
      delay(20);
    }
    resetRadioForRx();
  }
}
#endif

// OPTIONAL: TEST POSTER
static const bool ENABLE_TEST_POSTS = false;
static uint32_t    g_lastTestMinute = 0;
static uint32_t    g_testCnt        = 0;

static void maybeSendTestPacket() {
  if (!ENABLE_TEST_POSTS) return;
  time_t now = time(nullptr);
  if (now < 1600000000) return;

  uint32_t mt = minuteEpoch(now);
  if (mt == g_lastTestMinute) return;
  g_lastTestMinute = mt;

  static int k = 0; k = (k + 1) % 12;
  float wind_avg = 10.0f + 0.5f * k;
  float wind_max = wind_avg * 1.5f;
  int   wind_dir = 200 + 5 * k;
  uint32_t cnt = ++g_testCnt;

  char buf[128];
  snprintf(buf, sizeof(buf),
           "wind_avg=%.1f&wind_max=%.1f&wind_dir=%d&cnt=%lu",
           wind_avg, wind_max, wind_dir, (unsigned long)cnt);
  String body(buf);

  Serial.printf("TEST POSTING: %s\n", body.c_str());
  int code = postOnce(body);

  bool transient = (code < 0) || code == 408 || (code >= 500 && code < 600);
  if (transient) {
    Serial.printf("TEST transient (code=%d), retrying once...\n", code);
    delay(1200);
    code = postOnce(body);
  } else if (code >= 400 && code < 500) {
    Serial.printf("TEST client error (%d), skipping retry\n", code);
  }

  if (code == 200 || code == 201) {
    Serial.println("TEST POST accepted (200/201)");
  } else if (code == 400) {
    Serial.println("TEST POST accepted (benign 400)");
  } else {
    Serial.printf("TEST POST not successful (code=%d)\n", code);
  }
}

// === ACK SUPPORT ===
static void sendAck(uint32_t cnt, const char* status) {
  if (!ENABLE_ACK) return;
  char head[64];
  snprintf(head, sizeof(head), "ACK %lu %s", (unsigned long)cnt, status);
  uint16_t crc = crc16_ccitt((const uint8_t*)head, strnlen(head, sizeof(head)));
  char frame[80];
  snprintf(frame, sizeof(frame), "%s*%04X", head, (unsigned)crc);
  String s(frame);
  lora->standby();
  int st = lora->transmit(s);
  Serial.printf("BASE: ACK TX (%s) state=%d\n", frame, st);
  resetRadioForRx();
}

#ifdef LAB_MODE
static void sendBackfillAck(uint32_t cnt) {
  if (!ENABLE_BF_ACK) return;
  char head[32];
  snprintf(head, sizeof(head), "ACK_BF %lu", (unsigned long)cnt);
  uint16_t crc = crc16_ccitt((const uint8_t*)head, strnlen(head, sizeof(head)));
  char frame[48];
  snprintf(frame, sizeof(frame), "%s*%04X", head, (unsigned)crc);
  lora->standby();
  int st = lora->transmit(frame);
  Serial.printf("BASE: ACK_BF TX (%s) state=%d\n", frame, st);
  resetRadioForRx();
}
#endif

// SETUP
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_HEARTBEAT, OUTPUT);
  digitalWrite(LED_HEARTBEAT, LOW);

  // Optional: boot test – three quick blinks so we know pin 35 is really the LED
  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_HEARTBEAT, HIGH);
    delay(150);
    digitalWrite(LED_HEARTBEAT, LOW);
    delay(150);
  }

  connectWiFi(15000);
  initTimeUTC();
  g_start_ms = millis();
  g_prefs.begin("base_state", false);
  uint32_t stored = g_prefs.getUInt("last_cnt", UINT32_MAX);
  if (stored != UINT32_MAX) {
    g_last_cnt      = stored;
    g_have_last_cnt = true;
#ifdef LAB_MODE
    Serial.printf("BASE: restored last cnt=%lu from NVS\n", (unsigned long)g_last_cnt);
#endif
  }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(LORA_FREQ_MHZ);
  Serial.print("BASE begin()-> "); Serial.println(st);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init failed");
    while (true) delay(1000);
  }

  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setRxBoostedGainMode(true);
  lora->setCRC(true);

  lora->startReceive();
  Serial.println("BASE: ready (listening)");
}

void loop() {
  uint32_t now = millis();

  // ---------------------------
  // LED heartbeat / range test
  // ---------------------------
  if (now < g_rxHoldUntilMs) {
    // Solid ON when we've heard a packet recently
    digitalWrite(LED_HEARTBEAT, HIGH);
  } else {
    // Gentle heartbeat wink every 2 seconds
    static bool     hbOn    = false;
    static uint32_t hbOffMs = 0;

    if (!hbOn && (now - g_lastHeartbeatMs >= 2000)) {
      g_lastHeartbeatMs = now;
      hbOn    = true;
      hbOffMs = now + 50;           // LED ON for ~50 ms
      digitalWrite(LED_HEARTBEAT, HIGH);
    }

    if (hbOn && now >= hbOffMs) {
      hbOn = false;
      digitalWrite(LED_HEARTBEAT, LOW);
    }
  }

#ifdef LAB_MODE
  // Retry pending backfill requests even when no live packets are arriving
  maybeSendPendingBackfill();
#endif

  // ---------------------------
  // LoRa RX path
  // ---------------------------
  if (lora->getPacketLength() != 0) {
    String payload;
    int st = lora->readData(payload);

    if (st == RADIOLIB_ERR_NONE) {
      payload.trim();
      if (payload.length() == 0) {
        Serial.println("BASE: empty payload, skip");
        resetRadioForRx();
        delay(20);
        return;
      }

      // Backfill frames do NOT start with '{'
      if (payload[0] != '{') {
#ifdef LAB_MODE
        if (tryHandleBackfillFrame(payload)) {
          resetRadioForRx();
          delay(20);
          return;
        }
#endif
        Serial.println("Packet does not start with '{' - skipping");
        resetRadioForRx();
        delay(20);
        return;
      }

      // ---------------------------
      // RF duplicate burst guard
      // ---------------------------
      const uint32_t RF_DUP_MS = 5000UL;
      if (payload == g_lastRx && (millis() - g_lastRxMs) < RF_DUP_MS) {
        g_dupCount++;
        g_rf_dup++;   // count cumulative RF dups
        if (millis() - g_dupLastLog > 20000UL) {
          Serial.printf("BASE: duplicate RF payload burst (+%u skipped)\n",
                        g_dupCount);
          g_dupLastLog = millis();
          g_dupCount   = 0;
        }
        resetRadioForRx();
        delay(20);
        return;
      }
      if (g_dupCount > 0) {
        Serial.printf("BASE: duplicate burst ended (total skipped=%u)\n",
                      g_dupCount);
        g_dupCount = 0;
      }
      g_lastRx   = payload;
      g_lastRxMs = millis();

      // ---------------------------
      // CRC16 verify
      // ---------------------------
      if (LOG_POST_BODIES) {
        Serial.printf("RAW PAYLOAD: %s\n", payload.c_str());
      }
      int starIdx = payload.lastIndexOf('*');
      if (starIdx < 0 || (payload.length() - starIdx - 1) < 4) {
        Serial.println("CRC trailer missing/short; dropping packet");
        resetRadioForRx();
        delay(20);
        return;
      }

      char hex4[5];
      hex4[0] = payload[starIdx + 1];
      hex4[1] = payload[starIdx + 2];
      hex4[2] = payload[starIdx + 3];
      hex4[3] = payload[starIdx + 4];
      hex4[4] = '\0';

      for (int i = 0; i < 4; ++i) {
        if (!isxdigit((unsigned char)hex4[i])) {
          Serial.println("CRC trailer not hex; dropping packet");
          resetRadioForRx();
          delay(20);
          return;
        }
      }

      unsigned rx_crc_u = 0;
      sscanf(hex4, "%04X", &rx_crc_u);
      uint16_t rx_crc = (uint16_t)rx_crc_u;

      String jsonPart = payload.substring(0, starIdx);
      uint16_t calc = crc16_ccitt(
          (const uint8_t *)jsonPart.c_str(), jsonPart.length());
      if (calc != rx_crc) {
        g_crc_fail++;
        Serial.printf("CRC mismatch: calc=%04X rx=%04X -> drop\n",
                      (unsigned)calc, (unsigned)rx_crc);
        resetRadioForRx();
        delay(20);
        return;
      }

      payload = jsonPart;
      g_crc_ok++;

      // ---------------------------
      // JSON parse
      // ---------------------------
      JsonDocument d;
      DeserializationError err = deserializeJson(d, payload);
      if (err) {
        Serial.println("Parse fail: bad JSON, skipping this packet");
        resetRadioForRx();
        delay(20);
        return;
      }

      float wind_avg = d["wind_avg"] | 0.0f;
      float wind_max = d["wind_max"] | 0.0f;
      int   wind_dir = d["wind_dir"] | 0;
      long  cntJ     = d["cnt"]      | -1;
      float batt_v   = d["batt"]     | -1.0f;

      Serial.printf("PARSED: wind_avg=%.1f wind_max=%.1f wind_dir=%d cnt=%ld batt=%.2f\n",
                    wind_avg, wind_max, wind_dir, cntJ, batt_v);

      if (wind_avg == 0.0f && wind_max == 0.0f && wind_dir == 0) {
        Serial.println("Packet looked valid but values are all zero; skipping as corrupted");
        resetRadioForRx();
        delay(20);
        return;
      }

      // Range-test indicator
      g_rxHoldUntilMs = millis() + 10000UL;
      Serial.printf("RX: hold LED until %lu\n",
                    (unsigned long)g_rxHoldUntilMs);

      float   rssi_f = lora->getRSSI();
      float   snr_f  = lora->getSNR();
      int32_t cnt    = (int32_t)cntJ;

      // ---------------------------
      // EARLY ACK
      // ---------------------------
      if (cnt >= 0) {
        sendAck((uint32_t)cnt, "OK");
      }

      // ---------------------------
      // GAP / DROP TRACKING
      // ---------------------------
      if (cnt >= 0) {
        uint32_t rx_cnt = (uint32_t)cnt;
        g_total_packets++;

        if (!g_have_last_cnt) {
          g_have_last_cnt = true;
          g_last_cnt      = rx_cnt;
          g_drop_gap      = 0;
          g_drop_total    = 0;
#ifdef LAB_MODE
          Serial.printf("BASE: first RX this boot, cnt=%lu – starting fresh\n",
                        (unsigned long)rx_cnt);
#endif
        } else {
          uint32_t gap = 0;

          if (rx_cnt > g_last_cnt) {
            gap = rx_cnt - g_last_cnt - 1;
          } else {
#ifdef LAB_MODE
            Serial.printf("BASE: counter reset detected (old=%lu new=%lu), "
                          "clearing drop stats\n",
                          (unsigned long)g_last_cnt,
                          (unsigned long)rx_cnt);
#endif
            g_drop_gap   = 0;
            g_drop_total = 0;
            gap          = 0;
          }

          if (gap > 0) {
            g_drop_gap   = gap;
            g_drop_total += gap;
#ifdef LAB_MODE
            Serial.printf("BASE: gap detected last=%lu new=%lu gap=%lu => "
                          "schedule BF_REQ %lu->%lu\n",
                          (unsigned long)g_last_cnt,
                          (unsigned long)rx_cnt,
                          (unsigned long)gap,
                          (unsigned long)(g_last_cnt + 1),
                          (unsigned long)(rx_cnt - 1));
            requestBackfillRange(g_last_cnt + 1, rx_cnt - 1);
#endif
          } else {
            g_drop_gap = 0;
          }

          g_last_cnt = rx_cnt;
        }
        g_prefs.putUInt("last_cnt", g_last_cnt);

        if ((g_drop_total + g_total_packets) > 0) {
          g_drop_rate =
              (double)g_drop_total /
              (double)(g_drop_total + g_total_packets);
        } else {
          g_drop_rate = 0.0;
        }
      }

      // ---------------------------
      // Build body & HTTP post
      // ---------------------------
      String body = buildPostBody(wind_avg, wind_max, wind_dir,
                                  (long)cnt, batt_v, rssi_f, snr_f);
      if (LOG_POST_BODIES) {
        Serial.printf("BODY (clean): %s\n", body.c_str());
      }

      // in-flight / dedupe / rate limit
      if (cnt >= 0 && g_inflightCnt == cnt &&
          (millis() - g_inflightMs) < INFLIGHT_GUARD_MS) {
        Serial.println("BASE: in-flight guard (same cnt), skip");
        sendAck((uint32_t)cnt, "DUP");
        resetRadioForRx();
        delay(20);
        return;
      }
      if (cnt >= 0 && cnt == g_lastCntPosted) {
        Serial.println("BASE: duplicate by cnt, skip");
        sendAck((uint32_t)cnt, "DUP");
        resetRadioForRx();
        delay(20);
        return;
      }
      if ((millis() - g_lastPostMs) < MIN_POST_MS) {
        Serial.println("BASE: rate-limit active, skip post");
        sendAck((uint32_t)cnt, "RATE");
        resetRadioForRx();
        delay(20);
        return;
      }
      const uint32_t HTTP_DUP_MS = 70000UL;
      if (body == g_lastBody &&
          (millis() - g_lastPostMs) < HTTP_DUP_MS) {
        Serial.println("BASE: duplicate body within window, skip");
        sendAck((uint32_t)cnt, "DUP");
        resetRadioForRx();
        delay(20);
        return;
      }

      if (WiFi.status() != WL_CONNECTED) {
        connectWiFi(2000);
      }

      lora->standby();
      g_inflightCnt = cnt;
      g_inflightMs  = millis();

      if (LOG_POST_BODIES) {
        Serial.printf("BASE POSTING (rssi=%.1f, snr=%.1f): %s\n",
                      rssi_f, snr_f, body.c_str());
      }

      int code = postOnce(body);

      bool transient =
          (code < 0) || code == 408 || (code >= 500 && code < 600);
      if (transient) {
        Serial.printf("POST transient (code=%d), retrying once...\n",
                      code);
        delay(1200);
        code = postOnce(body);
      } else if (code >= 400 && code < 500 && code != 400) {
        Serial.printf("Client error (%d), skipping retry\n", code);
      }

      if (code == 200 || code == 201 || code == 400) {
        g_lastBody   = body;
        g_lastPostMs = millis();
        if (cnt >= 0) g_lastCntPosted = cnt;
        g_delivered_total++;
        if (cnt >= 0) sendAck((uint32_t)cnt, "OK");
        if (!LOG_POST_BODIES) {
          Serial.printf("BASE POST ok cnt=%ld code=%d\n", cnt, code);
        }
      } else {
        Serial.printf("POST not successful (code=%d)\n", code);
        // no ACK here → Pearl may retry
      }

      resetRadioForRx();
      if ((millis() - g_inflightMs) >= INFLIGHT_GUARD_MS) {
        g_inflightCnt = -1;
      }
    } else {
      Serial.printf("BASE RX error: %d\n", st);
      resetRadioForRx();
    }
  }

  maybeSendTestPacket();
  delay(20);
}




// handy:
// pio device list
// pio run -t upload --upload-port /dev/cu.usbserial-3
// pio device monitor -p /dev/cu.usbserial-3 -b 115200
