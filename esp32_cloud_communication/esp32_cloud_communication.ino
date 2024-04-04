#include "lora.h"

JsonDocument sensor_info;
JsonDocument control_info;

void setup() {
  Serial.begin(57600);
  lora_setup();
}

void loop() {
  // Create a quick json rpc for testing
  control_info["fan_speed"] = 10;
  control_info["temperature"] = 37;

  // Transmit and recieve Lora signals
  lora_transmit(&control_info);
  lora_receive(&sensor_info);

  //Print the transmited and recieved signals
  Serial.println("Sent the control info:");
  serializeJson(control_info, Serial); Serial.println();

  Serial.println("Recieved the sensor info:");
  serializeJson(sensor_info, Serial); Serial.println();
}
