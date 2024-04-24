#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud
  myservo.attach(22);  // attaches the servo on pin 22 to the servo object
}

void loop() {
  if (Serial.available() > 0) { // Check if data is available to read
    char receivedChar = Serial.read(); // Read the incoming byte
    
    if (receivedChar == '1') { // If '1' is received, open the pump
      for (pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 90 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
      delay(2000); // Wait for 2 seconds (adjust this value as needed)
      
      // Move back to position 0
      for (pos = 90; pos >= 0; pos -= 1) { // goes from 90 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    } else if (receivedChar == '0') { // If '0' is received, close the pump
      myservo.write(0); // Close the pump (move to 0 degrees)
      delay(15);        // waits 15ms for the servo to reach the position
    }
  }
}

