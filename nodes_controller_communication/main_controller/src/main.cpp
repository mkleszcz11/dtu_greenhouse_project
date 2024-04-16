/*
 * This is the main controller application that connects to the BLE devices (nodes) and recieves/sends messages to them.
 *
 * Code is part of the IoT project for the course "34346 Networking technologies and application development for Internet
 * of Things (IoT)" at the Technical University of Denmark.
 *
 * Code is based on the examples from the ArduinoBLE library, github link -> https://github.com/nkolban/ESP32_BLE_Arduino
 */

extern "C" {
  #include "esp_bt.h"
}

#include <Arduino.h>
#include "BLEDevice.h"

#define NUMBER_OF_DEVICES 3  // Number of devices to connect to (number of nodes) - to have more than 3 with Arduino framework drivers alignments are needed.
#define MAX_MESSAGE_SIZE  32 // Maximum size of the message from server, if it is larger, it will be truncated.
                             // IMPORTANT - messages might be truncated also because of MTU size in BLE settings, it is better to
                             // make the message shorter than increase the MTU size.

#define NUMBER_OF_CONTROLLED_PARAMETERS 4 // Number of parameters controled - Temperature, Humidity, Solar Radiation and Soil Moisture

/*State Machine*/
enum States {
  FIRST_CONNECTION,
  LOW_CONSUMPTION,
  ACTUATOR_MODE
};

uint8_t connectedDevices = 0;       // Number of currently connected devices.
bool print_all_devices_flag = true; // Flag used to print all devices when all are connected.

unsigned long interval = 60000;     
unsigned long previous_millis = 0;

/* Enum to store the index of the devices, used later in the code to easlily reference to specific defice. */
enum DeviceIndex {
  SOIL_MOISTURE_SENSOR,
  SOLAR_RADIATION_SENSOR,
  HUMIDITY_TEMPERATURE_SENSOR,
};

bool startSystem;
/* Structure to manage connections */
struct DeviceConnection {
  BLEAdvertisedDevice* device;             // Pointer to the device.
  BLERemoteCharacteristic* characteristic; // Pointer to the characteristic of the device.
  BLERemoteCharacteristic* characteristicACK; // Pointer to the characteristic to ACK messages of the device.
  bool doConnect;                          // Flag indicating if the device should try to connect.
  bool connected;                          // Flag indicating if the device is connected.
  BLEUUID serviceUUID;                     // UUID for the service
  BLEUUID charUUID;                        // UUID for the characteristic
  BLEUUID charUUIDACK;                        // UUID for the characteristic for MainController to Ackowledge messages
  States currentState;                     // Current State in State Machine
  bool transition;                         // To keep track of transitions in State Machine
  bool messageACK;                         // To acknowledge every time a new value is received
  // Constructor, initialize the structure with the service and characteristic UUIDs.
  DeviceConnection(const char* serviceUUID, const char* charUUID, const char* charUUIDACK)
    : device(nullptr), characteristic(nullptr), doConnect(false), connected(false),
      serviceUUID(BLEUUID(serviceUUID)), charUUID(BLEUUID(charUUID)),charUUIDACK(BLEUUID(charUUIDACK)),
      currentState(FIRST_CONNECTION), transition(false), messageACK(false) {}
  // Method to change the actual state of the device
  void changeState(States newState){
    currentState=newState;
    transition=true;
  }
  void print_current_state(){
      Serial.print("Current State: ");
    switch(currentState){
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
};

/* Array to store all devices info, it should be mapped accordingly to nodes code. */
DeviceConnection devices[NUMBER_OF_DEVICES] = {
  DeviceConnection("2E01EA51-1E73-4DAC-886D-08C04C040282", "beb5483e-36e1-4688-b7f5-ea07361b26a8", "beb5483e-36e1-4688-b7f5-ea07361b26a9"), // Soil moisture sensor
  DeviceConnection("9E61059E-B1CF-4953-A632-39CFD6A97255", "beb5483e-36e1-4688-b7f5-ea07361b26a8", "beb5483e-36e1-4688-b7f5-ea07361b26a9"),  // Water pump actuator
  DeviceConnection("A077916E-2CBE-45E3-BB35-4514C322606F", "beb5483e-36e1-4688-b7f5-ea07361b26a8", "beb5483e-36e1-4688-b7f5-ea07361b26a9") // Humidity sensor
  //DeviceConnection("8FBB11A7-163B-43C8-9D4F-FCA02297A258", "beb5483e-36e1-4688-b7f5-ea07361b26a8")  // Heater
}; 

/*Conditions and relevant signals to represent Sensor-Actuator logic of work*/
struct SensorActuatorConfig {
    float highBoundary;
    float lowBoundary;
    bool actuatorOn;
    bool boundariesRespected;
    DeviceIndex deviceIndex;
    //Constructor, initialize the structure with high boundary, low boundary and device index
    SensorActuatorConfig(float lowLimit, float highLimit, DeviceIndex index)
    : highBoundary(highLimit), lowBoundary(lowLimit),
      actuatorOn(false), boundariesRespected(true), deviceIndex(index) {}
    // Method to check if the current value respects the limits
    bool respectsLimits(float currentValue) {
        boundariesRespected = (currentValue >= lowBoundary && currentValue <= highBoundary);
        return boundariesRespected;
    }
};

//Enum to store the index of each Sensor-Actuator control
enum SensorActuatorIndex {
  SOIL_MOISTURE_WATER_PUMP, // SensorActuator index matches with device index
  SOLAR_RADIATION_LIGHT,    // SensorActuator index matches with device index
  TEMPERATURE_WINDOW, // Temperature and Humidity share the same device and actuator
  HUMIDITY_WINDOW    // Temperature and Humidity share the same device and actuator
};

/* Array to store all controlled parameters information. */
SensorActuatorConfig controlPairs[NUMBER_OF_CONTROLLED_PARAMETERS] = { // LATER: Change limits
  SensorActuatorConfig(0,2,SOIL_MOISTURE_SENSOR), // Soil moisture 
  SensorActuatorConfig(0,2,HUMIDITY_TEMPERATURE_SENSOR), // Temperature 
  SensorActuatorConfig(0,2,HUMIDITY_TEMPERATURE_SENSOR), // Humidity
  SensorActuatorConfig(0,2,SOLAR_RADIATION_SENSOR) // Solar Radiation 

}; 

/*
 * Function to send a message to the device.
 * It sends a message to the device with the specified index.
 */
bool send_message_to_device(int deviceIndex, States state) {

  if (devices[deviceIndex].characteristic == nullptr) return false;
  if (!devices[deviceIndex].connected) return false;
  String message;
  
  if(devices[deviceIndex].transition){
    devices[deviceIndex].transition=false;
    switch(state){
      case LOW_CONSUMPTION: 
            message="LOW";
            break;
      case ACTUATOR_MODE:
            message="ACTUATOR";
            break;
      default:
            return false;
    }
  }else message="KEEP";

  devices[deviceIndex].messageACK=false;
  Serial.println("[Main Controller] Sending Message: "+ message);
  devices[deviceIndex].characteristicACK->writeValue(message.c_str(), message.length());

  return true;
}

/* Callback function for notify. This callback will be invoked only when the characteristic value is changed. */
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* p_data,
  size_t length,
  bool isNotify) {
  
  int deviceIndexNotification; // to have access to the device 

  Serial.print("[Main Controller] Notify callback for device: ");
  for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
    if (devices[i].device != nullptr && devices[i].connected) {
      if (devices[i].device->getAddress().equals(pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress())) {
        Serial.println(devices[i].device->getName().c_str());
        deviceIndexNotification=i;
        break;
      }
    }
  }

  // Print incomming data, handle the null character at the end of the string.
  Serial.print(" | Data: ");
  char buffer[MAX_MESSAGE_SIZE + 1] = {0};
  size_t cp_length = (length < MAX_MESSAGE_SIZE) ? length : MAX_MESSAGE_SIZE;
  memcpy(buffer, p_data, cp_length);
  Serial.println(buffer);
  
  /*LATER: IMPLEMENT READ THE STRING TO FLOAT*/

  float readValue1, readValue2=0.0; //readValue2 is used only for HUMIDITY_TEMPERATURE_SENSOR

  readValue1=atof(buffer);
  char* separator = strchr(buffer, ';'); // Find the position of the semicolon

  if(separator!=nullptr){ //HUMIDITY_TEMPERATURE_SENSOR
    readValue2=atof(separator +1);
  }

  Serial.println("[Main Controller] Received values: " + String(readValue1)+ " "+ String(readValue2));

  /*This part is important to define State Machine transitions */
  switch(deviceIndexNotification){
    case HUMIDITY_TEMPERATURE_SENSOR:
      //readValue1 is temperature
      //readValue2 is humidity

      if(controlPairs[HUMIDITY_WINDOW].respectsLimits(readValue2) && /* Both values respect the boundaries and it's in ACTUATOR MODE*/
      controlPairs[TEMPERATURE_WINDOW].respectsLimits(readValue1) && 
      devices[deviceIndexNotification].currentState==ACTUATOR_MODE)
      { 
        devices[deviceIndexNotification].changeState(LOW_CONSUMPTION);
        Serial.println(String("[Main Controller] Transition Device ") + devices[deviceIndexNotification].device->getName().c_str());
      }
      else if((!controlPairs[HUMIDITY_TEMPERATURE_SENSOR].respectsLimits(readValue2) || 
            !controlPairs[TEMPERATURE_WINDOW].respectsLimits(readValue1)) && 
            devices[deviceIndexNotification].currentState==LOW_CONSUMPTION ) /*One of the values does not respect the boundaries and it's in LOW_CONSUMPTION*/
      {
        devices[deviceIndexNotification].changeState(ACTUATOR_MODE);
        Serial.println(String("[Main Controller] Transition Device ") + devices[deviceIndexNotification].device->getName().c_str());
      }
      break; 
    default: //controlPair index is the same as device

      if(controlPairs[deviceIndexNotification].respectsLimits(readValue1) && /* value respect the boundaries and it's in ACTUATOR MODE*/
      devices[deviceIndexNotification].currentState==ACTUATOR_MODE)
      {
        devices[deviceIndexNotification].currentState=LOW_CONSUMPTION;
        devices[deviceIndexNotification].transition=true;
        Serial.println(String("[Main Controller] Transition Device ") + devices[deviceIndexNotification].device->getName().c_str());
      }
      else if(!controlPairs[deviceIndexNotification].respectsLimits(readValue1) && 
            devices[deviceIndexNotification].currentState==LOW_CONSUMPTION ) /* value does not respect the boundaries and it's in LOW_CONSUMPTION*/
      {
        devices[deviceIndexNotification].currentState=ACTUATOR_MODE;
        devices[deviceIndexNotification].transition=true;
        Serial.println( String("[Main Controller] Transition Device ") + devices[deviceIndexNotification].device->getName().c_str());
      }
  }
  devices[deviceIndexNotification].messageACK=true; //MainController will acknowledge the message
  devices[deviceIndexNotification].print_current_state();
}

/* Callback function for to handle client connect/diconnect operations. */
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    Serial.println("[Main Controller] - onConnect callback");
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      if (devices[i].device != nullptr && devices[i].device->getAddress() == pclient->getPeerAddress()) { // Check if the device is in the list.
        connectedDevices++;                                                                               // Increment the connected device count.
        devices[i].connected = true;                                                                      // Update the corresponding device status.
        Serial.println(String("[Main Controller] - Device connected: ") + devices[i].device->getName().c_str());
        break;
      }
    }
  }

  void onDisconnect(BLEClient* pclient) override {
    Serial.println("[Main Controller] - onDisconnect callback");
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      if (devices[i].device != nullptr && devices[i].device->getAddress() == pclient->getPeerAddress()) { // Check if the device is in the list.
        connectedDevices--;                                                                               // Decrement the connected device count.
        print_all_devices_flag = true;                                                                    // Set the flag to print all devices when the will be connected again.
        devices[i].connected = false;  
        devices[i].doConnect= false;                                                                   // Update the corresponding device status.
        Serial.println(String("[Main Controller] - Device disconnected: ") + devices[i].device->getName().c_str());
        devices[i].device != nullptr;                                                                     // Clear the device pointer, slot is not taken anymore.
        break;
      }
    }
  }
};

/* Function to connect to the server. */
bool connectToServer(BLEAdvertisedDevice* myDevice, int deviceIndex) {
  if (myDevice == nullptr) return false;

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println("[Main Controller] - Created client");
  pClient->setClientCallbacks(new MyClientCallback());

  // Attempt to connect
  if (!pClient->connect(myDevice)) {
    Serial.println("[Main Controller] Failed to connect");
    return false;
  }

  // Connected
  Serial.println("Connected to server");

  // Attempt to retrieve the service
  BLERemoteService* pRemoteService = pClient->getService(devices[deviceIndex].serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("[Main Controller] Failed to find service UUID");
    pClient->disconnect();
    return false;
  }

  // Attempt to retrieve the characteristic
  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(devices[deviceIndex].charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("[Main Controller] Failed to find characteristic UUID");
    pClient->disconnect();
    return false;
  }

  // Attempt to retrieve the characteristicACK
  BLERemoteCharacteristic* pRemoteCharacteristicACK = pRemoteService->getCharacteristic(devices[deviceIndex].charUUIDACK);
  if (pRemoteCharacteristicACK == nullptr) {
    Serial.println("[Main Controller] Failed to find characteristic UUID ACK");
    pClient->disconnect();
    return false;
  }

  if (pRemoteCharacteristic->canRead()) { // If the characteristic can be read, read it.
      std::string value = pRemoteCharacteristic->readValue();
    }

  if (pRemoteCharacteristic->canNotify()) // If the characteristic can notify, go to notify callback.
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  devices[deviceIndex].connected = true;
  devices[deviceIndex].characteristic = pRemoteCharacteristic;
  devices[deviceIndex].characteristicACK = pRemoteCharacteristicACK;

  return true;
}

/*
 * Callback function for the advertised devices.
 * If the device is found, it will stop the scan and try to connect to the device.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      if (advertisedDevice.haveServiceUUID() &&
          advertisedDevice.isAdvertisingService(devices[i].serviceUUID)) {
        if (devices[i].device == nullptr || !devices[i].connected) {
          Serial.println(String("[Main Controller] Found a device: ") + advertisedDevice.toString().c_str());
          BLEDevice::getScan()->stop();
          devices[i].device = new BLEAdvertisedDevice(advertisedDevice);
          devices[i].doConnect = true;
          return;
        }
      }
    }
  }
};

/* Function to print all connected devices. */
void print_all_connections() {
  Serial.println("#############################");
  Serial.println("### ALL SERVERS CONNECTED ###");
  Serial.println("#############################");
  for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
    Serial.print("Device: ");
    Serial.print(i);
    Serial.print(": ");
    if (devices[i].device != nullptr) {
      Serial.println(devices[i].device->toString().c_str());
    }
    Serial.print("Characteristic: ");
    Serial.println(devices[i].characteristic != nullptr);
    Serial.print("Connected: ");
    Serial.println(devices[i].connected);
    Serial.print("State Machine: ");
    Serial.println(devices[i].currentState);
    Serial.println("------");

  }
  Serial.println("#############################");
}

/*
 * Function to connect to devices.
 * It iterates over all devices and tries to connect to them.
 */
void connect_to_devices() {
  for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
    if (devices[i].doConnect && !devices[i].connected) {
      if (connectToServer(devices[i].device, i)) {
        Serial.println( String("[Main Controller] Connected to the device ") + devices[i].device->getName().c_str());
      }
      else {
        Serial.println("[Main Controller] Failed to connect. Retrying...");
        devices[i].doConnect = false;
      }
    }
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("[Main Controller] Starting Arduino BLE Client application...");

  // Try align low level BLE settings to connect more than 3 devices.
  // esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  // bt_cfg.ble_max_conn = 4;
  // esp_bt_controller_init(&bt_cfg);
  // esp_bt_controller_enable(ESP_BT_MODE_BLE);

  BLEDevice::init("main_controller");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  startSystem=true;
}

void loop() {
  unsigned long current_millis = millis(); // Get the current time, nodes should be updated even if not all devices are connected.

  /* Print all connection if we are successfully connected. */
  if (connectedDevices == NUMBER_OF_DEVICES && print_all_devices_flag && startSystem) {
    print_all_connections();
    print_all_devices_flag = false;
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      if (devices[i].currentState==FIRST_CONNECTION)
      {
        devices[i].changeState(LOW_CONSUMPTION);
        devices[i].messageACK=devices[i].connected;
      }
    }
    startSystem=false; //it will never be true again (if we decide to sleep MainController this must change)
  }

  /* Try to scan and connect to new devices as long as all slots are not taken. */
  if (connectedDevices < NUMBER_OF_DEVICES) {
    BLEDevice::getScan()->start(5, false); // Rescan for 5 seconds (it has blocking behavior)
    Serial.println("[Main Controller] Scanning for devices, connected devices: "
                   + String(connectedDevices) + " / "
                   + String(NUMBER_OF_DEVICES));
    connect_to_devices();
  }

  if (connectedDevices == NUMBER_OF_DEVICES && current_millis-previous_millis>interval) {
    Serial.println("[Main Controller] Connected devices: "
                   + String(connectedDevices) + " / "
                   + String(NUMBER_OF_DEVICES));
    previous_millis=current_millis;
  }
  /*Check State Machines transitions*/

  if(!startSystem){
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      /*send value to device which requires ACK to last message*/
      //Serial.println(String("[Main Controller] ACK FLAG : ") + devices[i].messageACK );
      if(devices[i].messageACK){
        if (send_message_to_device(i,devices[i].currentState))
        {
            Serial.println(String("[Main Controller] Message sent to Device ") + devices[i].device->getName().c_str() );
        }
        else  
            Serial.println(String("[Main Controller] Error: Message not sent to Device ") + devices[i].device->getName().c_str());
      }
    }
  }

}
