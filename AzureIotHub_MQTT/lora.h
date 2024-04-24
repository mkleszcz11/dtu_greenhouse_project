#ifndef LORA_H
#define LORA_H

#include <Arduino.h>

void lora_setup();


void lora_transmit(uint8_t * sending_info, int n, bool * lora_transmit_flag, bool * lora_receive_flag, bool * lora_receive_message_flag);

void check_for_incoming_message(uint8_t * receiving_info, int n, bool * new_message_flag, bool * lora_receive_message_flag);

void lora_process_receive_message(String response, uint8_t *receiving_info, int n, bool *receive_flag, bool *receive_complete_flag);

void lora_receive(uint8_t * receiving_info, int n, bool * new_message_flag, bool * lora_receive_message_flag);


#endif