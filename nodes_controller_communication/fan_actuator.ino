int motorPin = 22;
unsigned long startTime = 0;
const unsigned long duration = 3000; // Duration in milliseconds (3 seconds)

void setup() 
{ 
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255");
  Serial.println("But the advice 50 to 255. Because the minimum voltage required to start the motor is 50.");
} 
 
void loop() 
{ 
  if (Serial.available())
  {
    int speed = Serial.parseInt();
    if (speed >= 50 && speed <= 255)
    {
      startTime = millis(); // Record the start time
      while (millis() - startTime <= duration) // Run the motor for 3 seconds
      {
        analogWrite(motorPin, speed);
      }
      analogWrite(motorPin, 0); // Stop the motor after 3 seconds, can be adjusted
    }
  }
}
