#include "lora.h"

// Is uint8_t enough, or should we make uint16_t?

// 1. Humidity 2. Soil Moisture 3. Temperature 4. Water Level
uint8_t sensor_info[3];
// uint8_t sensor_info[] = {1, 2, 3};
// 1. Humidity L, 2. Humidity U, 3. Soil Moisture L, 4. Soil Moisture U, 5. Temperature L, 6. Temperature U, 7. Water Level L, 8. Water Level L
uint8_t control_info[7];

void setup() {
  Serial.begin(57600);
  lora_setup();
}

void loop() {
  // Transmit and recieve Lora signals
  lora_transmit(sensor_info, 3);
  lora_receive(control_info, 7); // control_info not being the correct size could cause segmentation fault

  // FOR TESTING - Print the transmited and recieved signals
  Serial.print("Sent the sensor info: ");
  char str_sensor_info[64]; sprintf(str_sensor_info, "[%02d, %02d, %02d]", sensor_info[0], sensor_info[1], sensor_info[2]);
  Serial.println(str_sensor_info);

  Serial.print("Got the control info: ");
  char str_control_info[64]; sprintf(str_control_info, "[%02d, %02d, %02d]", control_info[0], control_info[1], control_info[2]);
  Serial.println(str_control_info);
}
