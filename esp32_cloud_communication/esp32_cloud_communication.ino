#include "lora.h"

// Is uint8_t enough, or should we make uint16_t?

// 1. temperature 2. air humidity 3. soil moisture
uint8_t sensor_info[2];
// 1. temperature target 2. air himidity target 3. soil moisture target
//uint8_t control_info[2];
uint8_t control_info[] = {4, 5, 6};

void setup() {
  Serial.begin(57600);
  lora_setup();
}

void loop() {
  // Transmit and recieve Lora signals
  lora_transmit(control_info, 2);
  lora_receive(sensor_info, 2); // control_info not being the correct size could cause segmentation fault

  // FOR TESTING - Print the transmited and recieved signals
  Serial.print("Sent the control info: ");
  char str_sensor_info[64]; sprintf(str_sensor_info, "[%02d, %02d, %02d]", control_info[0], control_info[1], control_info[2]);
  Serial.println(str_sensor_info);

  Serial.print("Got the sensor info: ");
  char str_control_info[64]; sprintf(str_control_info, "[%02d, %02d, %02d]", sensor_info[0], sensor_info[1], sensor_info[2]);
  Serial.println(str_control_info);
}
