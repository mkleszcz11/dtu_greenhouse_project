#include "lora.h"

void setup() {
  Serial.begin(57600);
  lora_setup();
}

void loop() {
  lora_transmit("DEF"); // Message is hex encoded
  String message = lora_receive();
  Serial.println("Received message: " + message);
}
