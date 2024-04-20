### Code to communicate multiple nodes with a main controller via BLE.
Code was developed using PlatformIO and Arduino framework.
There are to ways to run the code:
 - with PlatformIO - VS Code plugin is the easiest way to do it
 - with Arduino IDE - copy paste the code to the IDE, remove arduino header file (`#include <Arduino.h>`) and upload the code to the ESP32.

For now, use following files:
 - main_controller
 - soil_moisture_sensor 
 - humidity_sensor
 - solar_radiation_sensor

'heater' is disabled, as connections are limited to previously mentioned sensors and actuators.

Notes: # TODO delete bofore report submission
```bash
pkill -f /dev/ttyUSB0
platformio run --target upload --upload-port /dev/ttyUSB0
screen /dev/ttyUSB0 115200
```

cd ~/DTU/networking_technologies/dtu_greenhouse_project/nodes_controller_communication/
pkill -f /dev/ttyUSB0; platformio run --target upload --upload-port /dev/ttyUSB0; screen /dev/ttyUSB0 115200
pkill -f /dev/ttyUSB1; platformio run --target upload --upload-port /dev/ttyUSB1; screen /dev/ttyUSB1 115200
pkill -f /dev/ttyUSB2; platformio run --target upload --upload-port /dev/ttyUSB2; screen /dev/ttyUSB2 115200
pkill -f /dev/ttyUSB3; platformio run --target upload --upload-port /dev/ttyUSB3; screen /dev/ttyUSB3 115200
pkill -f /dev/ttyUSB4; platformio run --target upload --upload-port /dev/ttyUSB4; screen /dev/ttyUSB4 115200

To run without the sensors connected, comment out the line with the tag HARDCODE and comment the sensor reading lines
