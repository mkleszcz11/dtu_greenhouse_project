#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <ArduinoJson.h>

void lora_setup();
void lora_transmit(JsonDocument * sensor_info);
void lora_receive(JsonDocument * control_info);

#endif