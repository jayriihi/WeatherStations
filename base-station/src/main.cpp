#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// ================== USER CONFIG ==================
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

// naive extract "key":value from a JSON object string like {"key":12.3,...}
static bool extractFloat(const String& src, const char* key, float& outVal) {
  // look for "\"key\":"
  String pattern = String("\"") + key + "\":";
  int i = src.indexOf(pattern);
  if (i < 0) return false;
  i += pattern.length();

  // read until comma or end brace
  int j = i;
  while (j < (int)src.length() && src[j] != ',' && src[j] != '}') {
    j++;
  }
  if (j <= i) return false;

  String num = src.substring(i, j);
  outVal = num.toFloat();
  return true;
}

static bool extractInt(const String& src, const char* key, int& outVal) {
  float tmp;
  if (!extractFloat(src, key, tmp)) return false;
  outVal = (int)tmp;
  return true;
}

static bool extractULong(const String& src, const char* key, unsigned long& outVal) {
  // similar logic, but we want unsigned long
  String pattern = String("\"") + key + "\":";
  int i = src.indexOf(pattern);
  if (i < 0) return false;
  i += pattern.length();

  int j = i;
  while (j < (int)src.length() && src[j] != ',' && src[j] != '}') {
    j++;
  }
  if (j <= i) return false;

  String num = src.substring(i, j);
  outVal = (unsigned long) strtoul(num.c_str(), nullptr, 10);
  return true;
}


// ---------- Google Apps Script endpoint + API key ----------
#define POST_BASE "https://script.google.com/macros/s/AKfycbxkcVc6BP2oJtQcAB8cAvWWrIU9eDIGanyI5yWVj7GwHgISrCKnozDGZMXJobOxHGFu/exec"
#define API_KEY   "jI6nrJ2KTsgK0SDu"

// ---------- Heltec WiFi LoRa 32 V3 (SX1262) pins ----------
static const int PIN_NSS  = 8;
static const int PIN_DIO1 = 14;
static const int PIN_RST  = 12;
static const int PIN_BUSY = 13;
static const int PIN_SCK  = 9;
static const int PIN_MISO = 11;
static const int PIN_MOSI = 10;

// ================== RADIO & GLOBALS ==================
Module* modPtr = nullptr;
SX1262* lora   = nullptr;

// last-post / dedupe state (RF)
static int32_t  g_lastCntPosted = -1;
static String   g_lastRx;
static uint32_t g_lastRxMs = 0;
static String   g_lastBody;
static uint32_t g_lastPostMs = 0;

static uint16_t g_dupCount   = 0;
static uint32_t g_dupLastLog = 0;

static const uint32_t MIN_POST_MS = 20000UL;  // accept at most 1 post/min

// in-flight guard (prevents re-entrant posts for same cnt)
static int32_t  g_inflightCnt = -1;
static uint32_t g_inflightMs  = 0;
static const uint32_t INFLIGHT_GUARD_MS = 5000UL;  // drop repeats for 5s

// ================== TIME (for test poster) ==================
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

// ================== HELPERS ==================
static String getFormVal(const String& body, const String& key) {
  int pos = body.indexOf(key + "=");
  if (pos < 0) return "";
  int i = pos + key.length() + 1;
  int j = body.indexOf('&', i);
  if (j < 0) j = body.length();
  return body.substring(i, j);
}

// ---------- HARD RX RESET HELPER ----------
// Re-arm the radio as if we just rebooted, so we don't get stuck
static void resetRadioForRx() {
  // idle first
  lora->standby();
  delay(5);

  // full LoRa config (must match Pearl settings)
  lora->begin(915.0);
  lora->setDio2AsRfSwitch(true);
  lora->setSyncWord(0x34);
  lora->setBandwidth(125.0);
  lora->setSpreadingFactor(9);
  lora->setCodingRate(5);
  lora->setRxBoostedGainMode(true);
  lora->setCRC(true);

  // listen again
  lora->startReceive();
}

// ---------- HTTP POST (form-encoded, matches your curl/script) ----------
static int postOnce(const String& body) {
  if (WiFi.status() != WL_CONNECTED) return -1;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;

  // redirects + simple response framing
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.useHTTP10(true);                     // force HTTP/1.0 (no chunked)
  http.setReuse(false);
  http.setTimeout(15000);

  String url = String(POST_BASE) + "?api_key=" + API_KEY;
  if (!http.begin(client, url)) return -1;

  // headers
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Accept", "*/*");
  http.addHeader("User-Agent", "curl/8.0.1");
  http.addHeader("Accept-Encoding", "identity");  // no gzip/deflate
  http.addHeader("Connection", "close");          // close cleanly

  Serial.printf("POST %s\nBODY: %s\n", url.c_str(), body.c_str());

  int code = http.POST(body);              // adds Content-Length
  String resp = http.getString();
  http.end();

  Serial.printf("HTTP %d\n", code);
  if (code >= 500) {
    Serial.println(resp);                  // only print real server errors
  }
  return code;
}

// ================== OPTIONAL: TEST POSTER (1/min) ==================
static const bool ENABLE_TEST_POSTS = false;  // Pearl is active
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

  // retry only on transient errors
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

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(300);
  connectWiFi();
  initTimeUTC();

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

// ================== LOOP ==================
// ================== LOOP ==================
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

      // reject obvious tail fragments that don't start a JSON object
      if (payload[0] != '{') {
        Serial.println("Packet does not start with '{' - skipping");
        resetRadioForRx();
        delay(20);
        return;
      }

      // burst dedupe at RF level
      const uint32_t RF_DUP_MS = 5000UL;
      if (payload == g_lastRx && (millis() - g_lastRxMs) < RF_DUP_MS) {
        g_dupCount++;
        if (millis() - g_dupLastLog > 20000UL) {
          Serial.printf("BASE: duplicate RF payload burst (+%u skipped)\n", g_dupCount);
          g_dupLastLog = millis();
          g_dupCount = 0;
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

      // ---------- PARSE JSON FROM PEARL ----------
      Serial.printf("RAW PAYLOAD: %s\n", payload.c_str());

      StaticJsonDocument<256> d;
      DeserializationError err = deserializeJson(d, payload);
      if (err) {
        Serial.println("Parse fail: bad JSON, skipping this packet");
        resetRadioForRx();
        delay(20);
        return;
      }

      // Pearl now sends:
      // {"wind_avg":..,"wind_max":..,"wind_dir":..,"cnt":..,"batt":..}

      float wind_avg = d["wind_avg"] | 0.0f;
      float wind_max = d["wind_max"] | 0.0f;
      int   wind_dir = d["wind_dir"] | 0;
      long  cntJ     = d["cnt"]      | -1;
      float batt_v   = d["batt"]     | -1.0f;   // -1.0 when Pearl battery sense isn't wired yet

      Serial.printf("PARSED: wind_avg=%.1f wind_max=%.1f wind_dir=%d cnt=%ld batt=%.2f\n",
                    wind_avg, wind_max, wind_dir, cntJ, batt_v);

      // sanity filter: all-zero wind is almost always corruption, drop it
      if (wind_avg == 0.0f && wind_max == 0.0f && wind_dir == 0) {
        Serial.println("Packet looked valid but values are all zero; skipping as corrupted");
        resetRadioForRx();
        delay(20);
        return;
      }

      // grab link quality from this RX
      float rssi_f = lora->getRSSI();
      float snr_f  = lora->getSNR();

      // use cnt from Pearl for dedupe/rate-limit logic
      int32_t cnt = (int32_t)cntJ;

      // Build body for Google Apps Script.
      // We KEEP the original wind_* names so the sheet/website don't break.
      // We ADD:
      //   cnt   (first time it'll be logged)
      //   batt  (Pearl battery volts or -1.00 for now)
      //   rssi  (Base RX dBm)
      //   snr   (Base RX SNR dB)
      //
      // Note: if your Apps Script currently expects a 'ts=' field for timestamp,
      // and postOnce() adds it internally, we're fine. If not, we'll append &ts= later.
      char buf[256];
      snprintf(buf, sizeof(buf),
               "wind_avg=%.1f&wind_max=%.1f&wind_dir=%d"
               "&cnt=%ld&batt=%.2f&rssi=%.1f&snr=%.1f",
               wind_avg,
               wind_max,
               wind_dir,
               (long)cnt,
               batt_v,
               rssi_f,
               snr_f);

      String body(buf);

      Serial.printf("BODY (clean): %s\n", body.c_str());

      // in-flight guard (same cnt within short window)
      if (cnt >= 0 && g_inflightCnt == cnt && (millis() - g_inflightMs) < INFLIGHT_GUARD_MS) {
        Serial.println("BASE: in-flight guard (same cnt), skip");
        resetRadioForRx();
        delay(20);
        return;
      }

      // Counter de-dupe (don't post same cnt twice)
      if (cnt >= 0 && cnt == g_lastCntPosted) {
        Serial.println("BASE: duplicate by cnt, skip");
        resetRadioForRx();
        delay(20);
        return;
      }

      // One accepted post per minute
      if ((millis() - g_lastPostMs) < MIN_POST_MS) {
        Serial.println("BASE: rate-limit active, skip post");
        resetRadioForRx();
        delay(20);
        return;
      }

      // Body duplicate guard within window
      const uint32_t HTTP_DUP_MS = 70000UL;
      if (body == g_lastBody && (millis() - g_lastPostMs) < HTTP_DUP_MS) {
        Serial.println("BASE: duplicate body within window, skip");
        resetRadioForRx();
        delay(20);
        return;
      }

      if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
      }

      // ---- pause RF and mark in-flight before HTTP ----
      lora->standby();                 // keep TLS calm (no RX IRQs while HTTPS runs)
      g_inflightCnt = cnt;             // may be -1; fine
      g_inflightMs  = millis();

      Serial.printf("BASE POSTING (rssi=%.1f, snr=%.1f): %s\n",
                    rssi_f, snr_f, body.c_str());

      int code = postOnce(body);

      // retry only on transient errors (no retry on 4xx)
      bool transient = (code < 0) || code == 408 || (code >= 500 && code < 600);
      if (transient) {
        Serial.printf("POST transient (code=%d), retrying once...\n", code);
        delay(1200);
        code = postOnce(body);
      } else if (code >= 400 && code < 500) {
        Serial.printf("Client error (%d), skipping retry\n", code);
      }

      // consider 400 from Google benign for latching
      if (code == 200 || code == 201 || code == 400) {
        g_lastBody   = body;
        g_lastPostMs = millis();
        if (cnt >= 0) {
          g_lastCntPosted = cnt;
        }
      } else {
        Serial.printf("POST not successful (code=%d)\n", code);
      }

      // ---- HARD RESET RADIO FOR NEXT MINUTE ----
      resetRadioForRx();

      if ((millis() - g_inflightMs) >= INFLIGHT_GUARD_MS) {
        g_inflightCnt = -1;
      }

    } else {
      // RX error case
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
