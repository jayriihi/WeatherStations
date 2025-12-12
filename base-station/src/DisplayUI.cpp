#ifdef HAS_DISPLAY

#include "DisplayUI.h"
#include <SSD1306Wire.h>

namespace {

constexpr uint32_t DISPLAY_ACTIVE_MS  = 20000UL;
constexpr uint32_t RENDER_INTERVAL_MS = 100UL;
static uint8_t spinnerX              = 0;
static const uint8_t SPINNER_Y       = 24;
static const uint8_t SPINNER_STEP    = 4;
static const uint8_t SPINNER_WIDTH   = 128;
static uint32_t lastSpinnerMs        = 0;
static const uint32_t SPINNER_INTERVAL_MS = 150UL;

struct UiState {
  uint32_t bootMs         = 0;
  uint32_t lastActivityMs = 0;
  uint32_t lastRenderMs   = 0;
  uint32_t rxCount        = 0;
  uint32_t postOkCount    = 0;
  uint32_t postFailCount  = 0;
  int8_t   lastPostOk     = -1;  // -1 = none yet, 0 = fail, 1 = ok
  bool     blanked        = false;
  bool     pendingRender  = false;
  uint8_t  spinnerIdx     = 0;
};

UiState g_ui;
SSD1306Wire g_display(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64);

void render() {
  const uint32_t now = millis();
  g_ui.lastRenderMs   = now;
  g_ui.blanked        = false;

  g_display.clear();
  g_display.setFont(ArialMT_Plain_10);
  g_display.setTextAlignment(TEXT_ALIGN_LEFT);

  g_display.drawString(0, 0, "BASE STATION ONLINE");

  char line2[48];
  snprintf(line2, sizeof(line2),
           "RX:%lu OK:%lu ERR:%lu",
           (unsigned long)g_ui.rxCount,
           (unsigned long)g_ui.postOkCount,
           (unsigned long)g_ui.postFailCount);
  g_display.drawString(0, 12, line2);

  const char* lastPostStr = "LAST POST: NONE";
  if (g_ui.lastPostOk == 1) {
    lastPostStr = "LAST POST: OK";
  } else if (g_ui.lastPostOk == 0) {
    lastPostStr = "LAST POST: FAIL";
  }
  g_display.drawString(0, 24, lastPostStr);

  char line4[24];
  uint32_t upSec = (now - g_ui.bootMs) / 1000UL;
  snprintf(line4, sizeof(line4), "UP:%lus", (unsigned long)upSec);
  g_display.drawString(0, 36, line4);

  g_display.display();
}

}  // namespace

namespace DisplayUI {

void begin() {
  g_ui = UiState{};
  g_ui.bootMs = millis();
  g_ui.lastActivityMs = g_ui.bootMs;
  g_ui.pendingRender = true;
  spinnerX = 0;
  lastSpinnerMs = 0;
  g_ui.spinnerIdx = 0;

  // Power the OLED via Vext (Heltec V3 boards expose this on GPIO 36).
  const int VEXT_PIN = 36;
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);   // LOW turns Vext on for Heltec boards
  delay(10);

  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW);
  delay(10);
  digitalWrite(RST_OLED, HIGH);

  g_display.init();
  g_display.setContrast(255);
  g_display.clear();
  g_display.display();
}

void notifyRx() {
  g_ui.rxCount++;
  g_ui.lastActivityMs = millis();
  g_ui.pendingRender = true;
}

void notifyPost(bool success) {
  if (success) {
    g_ui.postOkCount++;
    g_ui.lastPostOk = 1;
  } else {
    g_ui.postFailCount++;
    g_ui.lastPostOk = 0;
  }
  g_ui.lastActivityMs = millis();
  g_ui.pendingRender = true;
}

void loop() {
  uint32_t now = millis();
  static const char SPINNER_CHARS[] = { '-', '\\', '|', '/' };
  bool isActive = (now - g_ui.lastActivityMs) <= DISPLAY_ACTIVE_MS;
  bool spinnerStepDue = (!isActive && (now - lastSpinnerMs) >= SPINNER_INTERVAL_MS);
  bool renderDue = g_ui.pendingRender || spinnerStepDue || (now - g_ui.lastRenderMs) >= RENDER_INTERVAL_MS;
  if (!renderDue) return;

  if (isActive) {
    g_ui.pendingRender = false;
    render();
    return;
  }

  if (spinnerStepDue) {
    lastSpinnerMs = now;
    spinnerX += SPINNER_STEP;
    if (spinnerX >= SPINNER_WIDTH) spinnerX = 0;
    g_ui.spinnerIdx = (g_ui.spinnerIdx + 1) % (sizeof(SPINNER_CHARS) / sizeof(SPINNER_CHARS[0]));
  }

  g_ui.lastRenderMs = now;
  g_display.clear();
  g_display.setFont(ArialMT_Plain_16);
  g_display.setTextAlignment(TEXT_ALIGN_LEFT);
  char sp[2] = { SPINNER_CHARS[g_ui.spinnerIdx], '\0' };
  g_display.drawString((int16_t)spinnerX, SPINNER_Y, sp);
  g_display.display();
}

}  // namespace DisplayUI

#endif  // HAS_DISPLAY
