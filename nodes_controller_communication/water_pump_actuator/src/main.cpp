/*
 * This is the main controller application that connects to the BLE devices (nodes) and recieves/sends messages to them.
 *
 * Code is part of the IoT project for the course "34346 Networking technologies and application development for Internet
 * of Things (IoT)" at the Technical University of Denmark.
 *
 * Code is based on the examples from the ArduinoBLE library, github link -> https://github.com/nkolban/ESP32_BLE_Arduino
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "9E61059E-B1CF-4953-A632-39CFD6A97255"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer = nullptr; // Pointer to BLE Server
BLEService *pService = nullptr; // Pointer to BLE Service
BLECharacteristic *pCharacteristic = nullptr; // Pointer to BLE Characteristic

std::vector<uint32_t> connectionIDs; // Vector to track client connection IDs

unsigned long previousMillis = 0; // Store last time a message was sent.
const long interval = 9000;       // Interval at which to send message (milliseconds).

/*
 * This class is used to define the callback function for the characteristic.
 * It is used to read the value of the characteristic.
*/
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string value = characteristic->getValue();

    if (value.length() > 0) {
      Serial.print("Received Value: ");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);

      Serial.println();
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
        Serial.println("Client Connected");
        uint32_t connId = param->connect.conn_id; // Get the connection ID
        connectionIDs.push_back(connId); // Add connection ID to the list
        Serial.print("Connection ID added: ");
        Serial.println(connId);
        // Add client to connections list
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("Client Disconnected");
        connectionIDs.clear(); 
        Serial.println("All connection IDs cleared (simple example)");
        Serial.println("Trying to reconnect...");
        BLEDevice::startAdvertising();
        // Remove client from connections list
    }
};

void setup() {
    Serial.begin(115200);
    BLEDevice::init("WaterPump");                     // Initialize the BLE device.
    pServer = BLEDevice::createServer();              // Create the BLE Server.
    pServer->setCallbacks(new ServerCallbacks());     // Set server callbacks
    pService = pServer->createService(SERVICE_UUID);  // Create the BLE Service.
    pCharacteristic = pService->createCharacteristic( // Create a BLE Characteristic, which will be used to read, write and notify.
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ | 
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Set the callback function for the characteristic.
    pService->start();                                              // Start the service.
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();     // Get the advertising object, to make the device will
    pAdvertising->addServiceUUID(SERVICE_UUID);                     // Add the service UUID to the advertising object.
    pAdvertising->setScanResponse(true);                            // Set the scan response to true, so the device will be visible to other devices.
    pAdvertising->setMinPreferred(0x12);                            // Set the minimum preferred interval value to 0x12 (shorter interval means faster connection, means more power consumption).
    BLEDevice::startAdvertising();                                  // Start advertising (now the device is visible to other devices).
    
    Serial.println("Device advertising, waiting for connections...");
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time in milliseconds.
  String message_to_sent = "Hey, I am a water pump actuator, my timestamp" + String(currentMillis); // Message to be sent to the main controller.

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;                       // Save the last time a message was sent.
    if (connectionIDs.size() > 0) {                       // If there are connected clients.
      pCharacteristic->setValue(message_to_sent.c_str()); // Set the characteristic value to some message.
      pCharacteristic->notify();                          // Notify the characteristic value to the client.
      Serial.println("Message sent to main controller.");
    }
    else {
      Serial.println("No clients connected.");
    }
  }

  // Do other stuff here.
}