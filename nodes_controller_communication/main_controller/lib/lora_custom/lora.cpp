#include "lora.h"
#include <HardwareSerial.h>
#include <string>

#define RXD2            16
#define TXD2            17
#define RST             23
#define SER_BUF_SIZE    1024

HardwareSerial loraSerial(2);
String str;

enum LoRaReceiveState {
  IDLE,
  WAITING_FOR_MESSAGE,
  PROCESSING_MESSAGE
};

LoRaReceiveState receiveState = IDLE;
String readBuffer = "";

void lora_autobaud() {
  String response = "";
  while (response=="")
  {
    delay(1000);
    loraSerial.write((byte)0x00);
    loraSerial.write(0x55);
    loraSerial.println();
    loraSerial.println("sys get ver");
    response = loraSerial.readStringUntil('\n');
  }
}

/*
 * This function blocks until the word "ok\n" is received on the UART,
 * or until a timeout of 3*5 seconds.
 */
int wait_for_ok() {
  str = loraSerial.readStringUntil('\n');
  if ( str.indexOf("ok") == 0 ) {
    return 1;
  }
  else return 0;
}

void lora_setup(){
  pinMode(RST, OUTPUT);

  // Resetting RN2483 by pulling RST pin low in 200 ms
  digitalWrite(RST, LOW);
  delay(200);
  digitalWrite(RST, HIGH);
  delay(200);

  // Open serial communications and wait for port to open:
  loraSerial.setRxBufferSize(SER_BUF_SIZE);
  loraSerial.begin(57600, SERIAL_8N1, RXD2, TXD2);
  loraSerial.setTimeout(1000);
  lora_autobaud();

  Serial.println("Initing LoRa");

  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  loraSerial.println("sys get ver");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("mac pause");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  //  loraSerial.println("radio set bt 0.5");
  //  wait_for_ok();

  loraSerial.println("radio set mod lora");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set freq 869106900");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set pwr 14");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set sf sf12");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set afcbw 41.7");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set rxbw 20.8");  // Receiver bandwidth can be adjusted here. Lower BW equals better link budget / SNR (less noise). 
  str = loraSerial.readStringUntil('\n');   // However, the system becomes more sensitive to frequency drift (due to temp) and PPM crystal inaccuracy. 
  Serial.println(str);

  //  loraSerial.println("radio set bitrate 50000");
  //  wait_for_ok();

  //  loraSerial.println("radio set fdev 25000");
  //  wait_for_ok();

  loraSerial.println("radio set prlen 8");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set crc on");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set iqi off");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set cr 4/5"); // Maximum reliability is 4/8 ~ overhead ratio of 2.0
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set wdt 0"); //disable for continuous reception
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set sync 12");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set bw 125");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
}

void lora_transmit(uint8_t *sending_info, int n, bool *transmit_flag, bool *receive_flag, bool *message_received_flag) {
    unsigned long start = millis();
    unsigned long delayTime = 1000;  // Initial delay time in ms
    const unsigned long maxDelayTime = 16000;  // Maximum delay time in ms
    unsigned long current_time = millis();
    bool busy_flag = true;
    String response;

    if (*transmit_flag) {
        char output[n * 2 + 1];
        int idx = 0;
        for (int i = 0; i < n; i++) {
            idx += sprintf(output + idx, "%02X", sending_info[i]);
        }
        output[idx] = '\0';  // Ensure null termination
        Serial.println("[LoRa - transmit] Transmitting: " + String(output));

        loraSerial.println("radio rxstop");
        // Wait to respond with ok, max 1 second.
        unsigned long start = millis();
        while (millis() - start < 1000) {
            if (loraSerial.available()) {
                String response_rx = loraSerial.readStringUntil('\n');
                //Serial.println("[LoRa - transmit] rxstop response -> " + response_rx);
                if (response_rx.startsWith("ok")) {
                    break;
                }
            }
        }

        loraSerial.println("mac pause");
        delay(100);
        while (loraSerial.available()) {
            String response_1 = loraSerial.readStringUntil('\n'); // Disregard any messages
            // Serial.println("[LoRa - transmit] mac pause response -> " + response_1);
        }

        while (busy_flag && delayTime <= maxDelayTime) {
            loraSerial.println("radio tx " + String(output));
            response = loraSerial.readStringUntil('\n');
            // Serial.println("[LoRa - transmit] response -> " + response);

            if (response.indexOf("busy") == -1) {
                // *transmit_flag = (response == "ok");
                busy_flag = false;
            } else {
                delay(delayTime);
                delayTime *= 2;  // Exponential back-off
                Serial.println("[LoRa - transmit] LoRa module is busy, retrying...");
            }
        }
        if (delayTime > maxDelayTime) {
            Serial.println("[LoRa - transmit] Failed to transmit: LoRa module too busy");
            *transmit_flag = false;
        }

        // wait 5 seconds for the radio_tx_ok response
        unsigned long start_wait_for_confirmation = millis();
        current_time = start_wait_for_confirmation;
        while (current_time - start_wait_for_confirmation < 5000) {
            if (loraSerial.available()) {
                String response = loraSerial.readStringUntil('\n');
                // Serial.println("[LoRa - transmit] Response: " + response);
                if (response.startsWith("radio_tx_ok")) {
                    Serial.println("[LoRa - transmit] Transmission successful");
                    *transmit_flag = false;
                    return;
                }
            }
            current_time = millis();
        }
        Serial.println("[LoRa - transmit] Transmission unsuccessful - no confirmation received.");
        *transmit_flag = false;
    }
}

void lora_process_receive_message(String response, uint8_t *receiving_info, int n, bool *receive_flag, bool *receive_complete_flag)
{
    if (response.startsWith("radio_rx")) {
        response.remove(0, 10);  // Adjust index based on actual prefix length "radio_rx "
        receiving_info[0] = strtol(response.substring(0, 2).c_str(), NULL, 16);
        receiving_info[1] = strtol(response.substring(2, 4).c_str(), NULL, 16);
        receiving_info[2] = strtol(response.substring(4, 6).c_str(), NULL, 16);

        *receive_complete_flag = true;
        *receive_flag = false;  // Data reception is complete, modukle is ready for new operation.
        Serial.println("[LoRa - receive] Data received successfully, data: " + String(receiving_info[0]) + " " + String(receiving_info[1]) + " " + String(receiving_info[2]));
        //break; // Exit after processing the message
    } else if (response.startsWith("radio_err")) {
        *receive_flag = false;                                             // Data reception was unsuccesful, module is ready for new operation.
    } else if (response.startsWith("busy")) {
        Serial.println("[LoRa - receive] Module is busy, retrying...");
    }
    else{
        Serial.println("[LoRa - receive] No valid data received or unexpected response");
    }
}

void check_for_incoming_message(uint8_t *receiving_info, int n, bool *receive_flag, bool *receive_complete_flag) {
    while (loraSerial.available()) {
        String response = loraSerial.readStringUntil('\n');
        // Serial.println("[LoRa - check_for_incoming_message] mac pause response -> " + response);
        if (response.startsWith("radio_rx")) {
            Serial.println("[LoRa - check_for_incoming_message] New incomming message available");
            lora_process_receive_message(response, receiving_info, n, receive_flag, receive_complete_flag);
        }
    }
}

void lora_receive(uint8_t *receiving_info, int n, bool *receive_flag, bool *receive_complete_flag) {
    if (*receive_flag) {
        loraSerial.println("mac pause");
        delay(100);
        check_for_incoming_message(receiving_info, n, receive_flag, receive_complete_flag);

        // Serial.println("[LoRa - receive] Entering continous receprion mode");
        loraSerial.println("radio rx 0"); // Put the radio into continous receive mode again.
    }
}
