#include <Arduino.h>
#include <RadioLib.h>

// Heltec WiFi LoRa 32 V3 pins
static const int PIN_NSS  = 8;
static const int PIN_DIO1 = 14;
static const int PIN_RST  = 12;
static const int PIN_BUSY = 13;
static const int PIN_SCK  = 9;
static const int PIN_MISO = 11;
static const int PIN_MOSI = 10;

// Downlink TX power (dBm) for this test
#ifndef TEST_DL_TX_DBM
#define TEST_DL_TX_DBM 2
#endif

#ifdef LAB_MODE
static const float LORA_FREQ_MHZ = 914.0f;
#else
static const float LORA_FREQ_MHZ = 915.0f;
#endif

Module* modPtr = nullptr;
SX1262* lora   = nullptr;

void setup() {
  Serial.begin(115200);
  delay(300);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  modPtr = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
  lora   = new SX1262(modPtr);

  int st = lora->begin(LORA_FREQ_MHZ);
  Serial.printf("BASE DLTEST begin()-> %d\n", st);
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
  lora->setOutputPower(TEST_DL_TX_DBM);
  lora->startReceive();
}

void loop() {
  static unsigned long lastTx = 0;
  unsigned long now = millis();

  // Periodic downlink TX of a simple marker
  if (now - lastTx > 5000) {
    String test = "DL_TEST";
    lora->standby();
    int st = lora->transmit(test);
    Serial.printf("BASE DL_TEST TX state=%d\n", st);
    lora->startReceive();
    lastTx = now;
  }

  // Log any received frame
  if (lora->getPacketLength() != 0) {
    String rx;
    int st = lora->readData(rx);
    if (st == RADIOLIB_ERR_NONE) {
      rx.trim();
      float rssi = lora->getRSSI();
      float snr  = lora->getSNR();
      Serial.printf("BASE RX: %s (rssi=%.1f snr=%.1f)\n", rx.c_str(), rssi, snr);
    } else {
      Serial.printf("BASE RX error: %d\n", st);
    }
    lora->startReceive();
  }

  delay(20);
}

