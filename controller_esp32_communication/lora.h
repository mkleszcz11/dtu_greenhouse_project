#ifndef LORA_H
#define LORA_H

#include <Arduino.h>

void lora_setup();
void lora_transmit(String message);
String lora_receive();

#endif