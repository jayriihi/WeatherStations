#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

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

// ---- random helpers for sim data ----
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
  lora->setOutputPower(0);   // keep low if boards are close

  Serial.println("PEARL: LoRa OK, TX mode (2 repeats/min with jitter)");
}

void loop() {
  static uint32_t lastMinute = 0;

  uint32_t mNow = bootMinutes();
  if (mNow == lastMinute) {
    delay(20);
    return;
  }
  lastMinute = mNow;

  // --- simulate wind values (replace with real sensor reads) ---
  float ws_avg  = 12.0f + frand(-3.0f, 3.5f);
  float ws_gust = ws_avg + frand(2.0f, 8.0f);
  int   wd_deg  = (230 + (int)roundf(frand(-40.f, 40.f)) + 360) % 360;

  // Authoritative counter: one new value per minute
  uint32_t cnt = mNow;   // base de-dupes on this

  // Build form body (base forwards this as-is)
  char buf[128];
  snprintf(buf, sizeof(buf),
           "wind_avg=%.1f&wind_max=%.1f&wind_dir=%d&cnt=%lu",
           ws_avg, ws_gust, wd_deg, (unsigned long)cnt);
  String body(buf);

  Serial.printf("PEARL cnt=%lu\n", (unsigned long)cnt);
  Serial.println("PEARL TX body: " + body);

  // --- Send 2 quick repeats then STOP until next minute ---
  txOnce(body);
  delay(120 + (esp_random() % 160));  // 120..279 ms random gap
  txOnce(body);

  // --- Sleep until next minute boundary with small jitter ---
  uint32_t ms     = millis();
  uint32_t next   = ((ms / 60000UL) + 1) * 60000UL; // next minute edge
  uint32_t jitter = 100 + (esp_random() % 400);     // 100..499 ms
  uint32_t wait   = (next + jitter > ms) ? (next + jitter - ms) : 200;
  delay(wait);
}

// handy:
// pio run -t upload --upload-port /dev/cu.usbserial-0001
// pio device monitor -p /dev/cu.usbserial-0001 -b 115200
