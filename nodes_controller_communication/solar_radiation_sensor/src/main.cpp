/*
 * This is the main controller application that connects to the BLE devices (nodes) and recieves/sends messages to them.
 *
 * Code is part of the IoT project for the course "34346 Networking technologies and application development for Internet
 * of Things (IoT)" at the Technical University of Denmark.
 *
 * Code is based on the examples from the ArduinoBLE library, github link -> https://github.com/nkolban/ESP32_BLE_Arduino
 */
/*
  STATE MACHINE: -FIRST_CONNECTION
                 -LOW_CONSUMPTION
                 -ACTUATOR_MODE

  * FIRST_CONNECTION: State waiting for the first time the sensor conencts to device
  * LOW_CONSUMTPION: State when actuator is not needed. Send sensor value and sleeps for TIME_TO_SLEEP_LOW_CONSUMPTION .
  * ACTUATOR_MODE: State when actuator is needed. Send sensor values every interval value.
  *
  * Water Level sensor : ESP32 pin GPIO17 connected to sensor's VCC pin
  *                       ESP32 pin GPIO36 (ADC0) connected to sensor's signal pin
  * LED_BUILTIN is used to represent the state of the actuator
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <math.h>

#define SERVICE_UUID "9E61059E-B1CF-4953-A632-39CFD6A97255"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP_LOW_CONSUMPTION 60 /* Time ESP32 will go to sleep (in seconds) (5 minutes)*/
#define TIME_TO_SLEEP_ACTUATOR_MODE 30 /* Time ESP32 will go to sleep (in seconds)*/
#define CHARACTERISTIC_UUID_ACK "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define LED 2
#define POWER_PIN 17 // ESP32 pin GPIO17 connected to sensor's VCC pin
#define SIGNAL_PIN 36 // ESP32 pin GPIO36 (ADC0) connected to sensor's signal pin

BLEServer* pServer = nullptr; // Pointer to BLE Server
BLEService* pService = nullptr; // Pointer to BLE Service
BLECharacteristic* pCharacteristic = nullptr; // Pointer to BLE Characteristic
BLECharacteristic* pCharacteristicAck = nullptr;
std::vector<uint32_t> connectionIDs; // Vector to track client connection IDs

unsigned long previousMillis = 0; // Store last time a message was sent.
const long interval = 30000; // Interval at which to send message (milliseconds).

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool stateMachineStarted = false;
// RTC_DATA_ATTR float safeReadValue=0; /*HARDCODE*/

/*State Machine*/
enum States {
    FIRST_CONNECTION,
    LOW_CONSUMPTION,
    ACTUATOR_MODE
};

States currentState;
bool transition = false; // to know when a transition happens
bool messageACK = false; // ACK message. So this way, we are sure the value is received
bool sendMessage = false; // to know when message should be sent
float readValue;
// Function to transition to a new state
void changeState(States nextState)
{
    currentState = nextState;
    transition = true;
}

bool send_message_to_device(String message)
{

    if (message == nullptr || pCharacteristic == nullptr)
        return false;

    pCharacteristic->setValue(message.c_str()); // Set the characteristic value to some message.
    pCharacteristic->notify(); // Notify the characteristic value to the client.

    sendMessage = false;
    return true;
}
/*function to formatData that will be sent to the master*/
String formatData(float value)
{
    return String(value);
}

void print_current_state()
{
    Serial.print("[Solar Radiation Sensor] Current State: ");
    switch (currentState) {
    case FIRST_CONNECTION:
        Serial.println("FIRST_CONNECTION");
        break;
    case LOW_CONSUMPTION:
        Serial.println("LOW_CONSUMPTION");
        break;
    case ACTUATOR_MODE:
        Serial.println("ACTUATOR_MODE");
        break;
    }
}
/*
 * This class is used to define the callback function for the characteristic.
 * It is used to read the value of the characteristic.
 */
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic)
    {
        std::string value = characteristic->getValue();

        if (value.length() > 0) {
            Serial.print("[Solar Radiation Sensor] Received Value: ");
            for (int i = 0; i < value.length(); i++)
                Serial.print(value[i]);
            Serial.println();
        }

        /*if value is "LOW" the state changes for LOW_CONSUMPTION*/
        if (value == "LOW" && currentState != LOW_CONSUMPTION) { // confirm this way to compare works
            // Change the state to LOW_CONSUMPTION
            changeState(LOW_CONSUMPTION);
            print_current_state();
        } else if (value == "ACTUATOR" && currentState != ACTUATOR_MODE) {
            changeState(ACTUATOR_MODE);
            print_current_state();
            sendMessage = false;
        } else {
            Serial.println("[Solar Radiation Sensor] Keep the same State");
            print_current_state();
        }
        if (stateMachineStarted)
            messageACK = true;
        else {
            stateMachineStarted = true; // MainController sends signal to start the State Machine
            sendMessage = true;
        }
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param)
    {
        Serial.println("[Solar Radiation Sensor] Client Connected");
        uint32_t connId = param->connect.conn_id; // Get the connection ID
        connectionIDs.push_back(connId); // Add connection ID to the list
        Serial.print("[Solar Radiation Sensor] Connection ID added: ");
        Serial.println(connId);
        // Add client to connections list
    }

    void onDisconnect(BLEServer* pServer)
    {
        Serial.println("[Solar Radiation Sensor] Client Disconnected");
        connectionIDs.clear();
        Serial.println("[Solar Radiation Sensor] All connection IDs cleared (simple example)");
        Serial.println("[Solar Radiation Sensor] Trying to reconnect...");
        BLEDevice::startAdvertising();
        // Remove client from connections list
    }
};

void setup()
{

    Serial.begin(115200);
    pinMode(POWER_PIN, OUTPUT); // configure pin as an OUTPUT
    digitalWrite(POWER_PIN, LOW); // turn the sensor OFF

    BLEDevice::init("SensorSolarRadiation"); // Initialize the BLE device.
    pServer = BLEDevice::createServer(); // Create the BLE Server.
    pServer->setCallbacks(new ServerCallbacks()); // Set server callbacks
    pService = pServer->createService(SERVICE_UUID); // Create the BLE Service.
    pCharacteristic = pService->createCharacteristic( // Create a BLE Characteristic, which will be used to read, write and notify.
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicAck = pService->createCharacteristic( // Create a BLE Characteristic, which will be used to read, write and notify.
        CHARACTERISTIC_UUID_ACK,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pCharacteristicAck->setCallbacks(new MyCharacteristicCallbacks()); // Set the callback function for the characteristic.
    pService->start(); // Start the service.

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising(); // Get the advertising object, to make the device will
    pAdvertising->addServiceUUID(SERVICE_UUID); // Add the service UUID to the advertising object.
    pAdvertising->setScanResponse(true); // Set the scan response to true, so the device will be visible to other devices.
    pAdvertising->setMinPreferred(0x12); // Set the minimum preferred interval value to 0x12 (shorter interval means faster connection, means more power consumption).
    BLEDevice::startAdvertising(); // Start advertising (now the device is visible to other devices).

    Serial.println("[Solar Radiation Sensor] Device advertising, waiting for connections...");
    pinMode(LED, OUTPUT);

    /*If it's not the starting and setup is run, it means it comes from a deep sleep mode*/
    if (bootCount == 0)
        currentState = FIRST_CONNECTION;
    else if (bootCount == 1) {
        currentState = LOW_CONSUMPTION;
        sendMessage = true;
        delay(10000);
    } else if (bootCount = 2)
        currentState = ACTUATOR_MODE;
}

void loop()
{
    unsigned long currentMillis = millis(); // Get the current time in milliseconds.

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        print_current_state();
        Serial.print("The water sensor value: ");
        Serial.println(readValue);
        if (currentState == ACTUATOR_MODE) {
            sendMessage = true;
        }
    }

    /*
    if (connectionIDs.size() > 0 && currentState==FIRST_CONNECTION) {
      changeState(LOW_CONSUMPTION);
      sendMessage=true;
    } */

    /*Check if device is connected, if it's not Advertise. If it's perform State Machine actions*/
    if (connectionIDs.size() > 0) {
        switch (currentState) {
        case LOW_CONSUMPTION:
            if (transition) {
                /*
                Stop actuator
                */
                digitalWrite(LED, LOW);
                transition = false;
            }
            /* ESP goes to sleep when message is acknowledged*/
            if (messageACK) {
                bootCount = currentState; // to save state before sleep mode
                esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_LOW_CONSUMPTION * uS_TO_S_FACTOR);
                Serial.println("[Solar Radiation Sensor] Setup ESP32 to deep sleep for " + String(TIME_TO_SLEEP_LOW_CONSUMPTION) + " Seconds");
                Serial.println("[Solar Radiation Sensor] Entering Deep Sleep Mode...");
                Serial.flush();
                esp_deep_sleep_start();
            }
            break;
        case ACTUATOR_MODE:
            bootCount = currentState; // to keep the current state saved in RTC
            /*
            esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_ACTUATOR_MODE * uS_TO_S_FACTOR);
            Serial.println("[Solar Radiation Sensor] Setup ESP32 to deep sleep for " + String(TIME_TO_SLEEP_ACTUATOR_MODE) +" Seconds");
            Serial.println("[Solar Radiation Sensor] Entering Deep Sleep Mode...");
            Serial.flush();
            esp_deep_sleep_start();
            */
            if (transition) {
                /*
                Start actuator
                */
                digitalWrite(LED, HIGH);
                transition = false;
            }
            if (messageACK)
                messageACK = false;

            break;
        case FIRST_CONNECTION:
            break;
        default:
            Serial.println("[Solar Radiation Sensor] State not valid");
        }
    }

    if (sendMessage) {
        /*Hardcoded changes at the values to make sure State Machine is working correctly*/
        // safeReadValue=safeReadValue+1; /*HARDCODE*/
        // readValue=fmod(safeReadValue,4); /*HARDCODE*/
        digitalWrite(POWER_PIN, HIGH); // turn the sensor ON
        delay(10); // wait 10 milliseconds
        readValue = analogRead(SIGNAL_PIN); // read the analog value from sensor
        digitalWrite(POWER_PIN, LOW); // turn the sensor OFF

        String message_to_sent = formatData(readValue);
        if (send_message_to_device(message_to_sent)) {
            Serial.println("[Solar Radiation Sensor] Message sent to Main Controller " + String(readValue));
        } else
            Serial.println("[Solar Radiation Sensor] Message not sent to Main Controller ");
    }
    // Do other stuff here.
}
