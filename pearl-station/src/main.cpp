#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);  // Let Serial settle

  Serial.println("✅ Pearl Station booted up.");
  
  pinMode(LED_BUILTIN, OUTPUT);  // You may need to change this pin
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED ON");
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED OFF");
  delay(500);
}
