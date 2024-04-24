#include "lora.h"
#include <HardwareSerial.h>

#define RXD2            16
#define TXD2            17
#define RST             23
#define SER_BUF_SIZE    1024

HardwareSerial loraSerial(2);
String str;

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

  loraSerial.println("radio set wdt 60000"); //disable for continuous reception
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set sync 12");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set bw 125");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
}

void lora_transmit(uint16_t * sending_info, int n) { // This function sends messages via Lora
  // Convert the list to hex numbers
  char output[n*4+4]; int i;
  for(i = 0; i <= n; i++){
    sprintf(output+i*4, "%04X", sending_info[i]);
  }

  loraSerial.println("radio tx " + String(output));
  str = loraSerial.readStringUntil('\n');
  str = loraSerial.readStringUntil('\n');
}

// Maybe this need to be a double pointer
void lora_receive(uint16_t * receiving_info, int n) { // This function reads messages via Lora
  Serial.println("waiting for a message");
  loraSerial.println("radio rx 0"); //wait for 60 seconds to receive
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  delay(20);

  if ( str.indexOf("ok") == 0 ) {
    str = String("");

    while(str=="") {
      str = loraSerial.readStringUntil('\n');
    }

    if ( str.indexOf("radio_rx") == 0 ) { //checking if data was reeived (equals radio_rx = <data>). indexOf returns position of "radio_rx"
      // remove "radio rx 0  "
      str.remove(0, 10);

      // convert hex to uint8_t list
      // Throw String to char[] to avoid error in following loop - FIXME
      int len = str.length(); char val[4];
      char str_copy[len]; int i;
      str.toCharArray(str_copy, len);

      for(i = 0; i <= n; i++){
        sscanf(str_copy+4*i, "%04X",val);
        receiving_info[i] = uint16_t(val[0]);
      }
    } else {
      Serial.println("Received nothing");
    }
  } else {
    Serial.println("radio not going into receive mode");
  }
}