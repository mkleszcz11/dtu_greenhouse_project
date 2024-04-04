#include "lora.h"

JsonDocument sensor_info;
JsonDocument control_info;

void setup() {
  Serial.begin(57600);
  lora_setup();
}

void loop() {
  // Create a quick json rpc for testing
  sensor_info["humidity"] = 4;
  sensor_info["temperature"] = 5;

  // Transmit and recieve Lora signals
  lora_transmit(&sensor_info);
  lora_receive(&control_info);

  //Print the transmited and recieved signals
  Serial.println("Sent the sensor info:");
  serializeJson(sensor_info, Serial); Serial.println();

  Serial.println("Recieved the control info:");
  serializeJson(control_info, Serial); Serial.println();
}
