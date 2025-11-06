#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ctype.h>

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

// ---------- Wi-Fi ----------
static void connectWiFi() {
  const char* SSID = "Hapenny";
  const char* PASS = "hapennyhouse";
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.printf("WiFi: connecting to %s", SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\nWiFi: timeout, retrying");
      t0 = millis();
      WiFi.disconnect(true, true);
      WiFi.begin(SSID, PASS);
    }
  }
  Serial.printf("\nWiFi OK ip=%s\n", WiFi.localIP().toString().c_str());
}

// Health counters
static uint32_t g_crc_ok   = 0;
static uint32_t g_crc_fail = 0;
static uint32_t g_rf_dup   = 0;
static uint32_t g_boot_id  = (uint32_t)esp_random();
static uint32_t g_start_ms = 0;

// ---------- Google Apps Script endpoint + API key ----------
#define POST_BASE "https://script.google.com/macros/s/AKfycbxkcVc6BP2oJtQcAB8cAvWWrIU9eDIGanyI5yWVj7GwHgISrCKnozDGZMXJobOxHGFu/exec"
#define API_KEY   "jI6nrJ2KTsgK0SDu"

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

static const uint32_t MIN_POST_MS = 20000UL;
static int32_t  g_inflightCnt = -1;
static uint32_t g_inflightMs  = 0;
static const uint32_t INFLIGHT_GUARD_MS = 5000UL;

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

// HARD RX RESET
static void resetRadioForRx() {
  lora->standby();
  delay(5);

  lora->begin(915.0);
  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setRxBoostedGainMode(true);
  lora->setCRC(true);

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

  Serial.printf("HTTP %d\n", code);
  if (code >= 500) Serial.println(resp);
  return code;
}

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

// SETUP
void setup() {
  Serial.begin(115200);
  delay(300);
  connectWiFi();
  initTimeUTC();
  g_start_ms = millis();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(915.0);
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

// LOOP
void loop() {
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

      if (payload[0] != '{') {
        Serial.println("Packet does not start with '{' - skipping");
        resetRadioForRx();
        delay(20);
        return;
      }

      // RF duplicate burst guard
      const uint32_t RF_DUP_MS = 5000UL;
      if (payload == g_lastRx && (millis() - g_lastRxMs) < RF_DUP_MS) {
        g_dupCount++;
        g_rf_dup++;   // count cumulative RF dups
        if (millis() - g_dupLastLog > 20000UL) {
          Serial.printf("BASE: duplicate RF payload burst (+%u skipped)\n", g_dupCount);
          g_dupLastLog = millis();
          g_dupCount   = 0;
        }
        resetRadioForRx();
        delay(20);
        return;
      }
      if (g_dupCount > 0) {
        Serial.printf("BASE: duplicate burst ended (total skipped=%u)\n", g_dupCount);
        g_dupCount = 0;
      }
      g_lastRx   = payload;
      g_lastRxMs = millis();

      // CRC16 verify
      Serial.printf("RAW PAYLOAD: %s\n", payload.c_str());
      int starIdx = payload.lastIndexOf('*');
      if (starIdx < 0 || (payload.length() - starIdx - 1) < 4) {
        Serial.println("CRC trailer missing/short; dropping packet");
        resetRadioForRx(); delay(20); return;
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
          resetRadioForRx(); delay(20); return;
        }
      }

      unsigned rx_crc_u = 0;
      sscanf(hex4, "%04X", &rx_crc_u);
      uint16_t rx_crc = (uint16_t)rx_crc_u;

      String jsonPart = payload.substring(0, starIdx);
      uint16_t calc = crc16_ccitt((const uint8_t*)jsonPart.c_str(), jsonPart.length());
      if (calc != rx_crc) {
        g_crc_fail++;   // count failures
        Serial.printf("CRC mismatch: calc=%04X rx=%04X -> drop\n", (unsigned)calc, (unsigned)rx_crc);
        resetRadioForRx(); delay(20); return;
      }

      payload = jsonPart;
      g_crc_ok++;       // count passes

      // Parse JSON
      StaticJsonDocument<256> d;
      DeserializationError err = deserializeJson(d, payload);
      if (err) {
        Serial.println("Parse fail: bad JSON, skipping this packet");
        resetRadioForRx(); delay(20); return;
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
        resetRadioForRx(); delay(20); return;
      }

      float rssi_f = lora->getRSSI();
      float snr_f  = lora->getSNR();
      int32_t cnt  = (int32_t)cntJ;

      // Build body (with health)
      uint32_t uptime_s = (millis() - g_start_ms) / 1000UL;
      char buf[360];
      snprintf(buf, sizeof(buf),
               "wind_avg=%.1f&wind_max=%.1f&wind_dir=%d"
               "&cnt=%ld&batt=%.2f&rssi=%.1f&snr=%.1f"
               "&crc_ok=%lu&crc_fail=%lu&rf_dup=%lu&uptime_s=%lu&boot_id=%lu&fw=base_1",
               wind_avg, wind_max, wind_dir, (long)cnt, batt_v, rssi_f, snr_f,
               (unsigned long)g_crc_ok, (unsigned long)g_crc_fail, (unsigned long)g_rf_dup,
               (unsigned long)uptime_s, (unsigned long)g_boot_id);

      String body(buf);
      Serial.printf("BODY (clean): %s\n", body.c_str());

      // in-flight / dedupe / rate limit
      if (cnt >= 0 && g_inflightCnt == cnt && (millis() - g_inflightMs) < INFLIGHT_GUARD_MS) {
        Serial.println("BASE: in-flight guard (same cnt), skip");
        resetRadioForRx(); delay(20); return;
      }
      if (cnt >= 0 && cnt == g_lastCntPosted) {
        Serial.println("BASE: duplicate by cnt, skip");
        resetRadioForRx(); delay(20); return;
      }
      if ((millis() - g_lastPostMs) < MIN_POST_MS) {
        Serial.println("BASE: rate-limit active, skip post");
        resetRadioForRx(); delay(20); return;
      }
      const uint32_t HTTP_DUP_MS = 70000UL;
      if (body == g_lastBody && (millis() - g_lastPostMs) < HTTP_DUP_MS) {
        Serial.println("BASE: duplicate body within window, skip");
        resetRadioForRx(); delay(20); return;
      }

      if (WiFi.status() != WL_CONNECTED) connectWiFi();

      lora->standby();
      g_inflightCnt = cnt;
      g_inflightMs  = millis();

      Serial.printf("BASE POSTING (rssi=%.1f, snr=%.1f): %s\n", rssi_f, snr_f, body.c_str());
      int code = postOnce(body);

      bool transient = (code < 0) || code == 408 || (code >= 500 && code < 600);
      if (transient) {
        Serial.printf("POST transient (code=%d), retrying once...\n", code);
        delay(1200);
        code = postOnce(body);
      } else if (code >= 400 && code < 500) {
        Serial.printf("Client error (%d), skipping retry\n", code);
      }

      if (code == 200 || code == 201 || code == 400) {
        g_lastBody   = body;
        g_lastPostMs = millis();
        if (cnt >= 0) g_lastCntPosted = cnt;
      } else {
        Serial.printf("POST not successful (code=%d)\n", code);
      }

      resetRadioForRx();
      if ((millis() - g_inflightMs) >= INFLIGHT_GUARD_MS) g_inflightCnt = -1;

    } else {
      Serial.printf("BASE RX error: %d\n", st);
      resetRadioForRx();
    }
  }

  maybeSendTestPacket();
  delay(20);
}

// handy:
// pio run -t upload --upload-port /dev/cu.usbserial-3
// pio device monitor -p /dev/cu.usbserial-3 -b 115200