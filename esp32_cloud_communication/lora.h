#ifndef LORA_H
#define LORA_H

#include <Arduino.h>

void lora_setup();
void lora_transmit(uint8_t * sending_info, int n);
void lora_receive(uint8_t * receiving_info, int n);

#endif