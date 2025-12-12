#pragma once

#ifdef HAS_DISPLAY

#include <Arduino.h>

namespace DisplayUI {
  void begin();                  // call once at startup (after hardware init)
  void notifyRx();               // called whenever a LoRa packet is received
  void notifyPost(bool success); // called after a POST attempt
  void loop();                   // called every main loop iteration
}

#endif  // HAS_DISPLAY

