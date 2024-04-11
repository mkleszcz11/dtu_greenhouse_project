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

uint8_t connectedDevices = 0;       // Number of currently connected devices.
bool print_all_devices_flag = true; // Flag used to print all devices when all are connected.

unsigned long interval_humidity_sensor = 7000;     // Interval at which main controller sends a message to the humidity sensor.
unsigned long previous_millis_humidity_sensor = 0; // Store last time a message to humidity sensor was sent.

/* Enum to store the index of the devices, used later in the code to easlily reference to specific defice. */
enum DeviceIndex {
  SOIL_MOISTURE_SENSOR = 0,
  HUMIDITY_SENSOR = 1,
  WATER_PUMP_ACTUATOR = 2
  // HEATER = 3
};

/* Structure to manage connections */
struct DeviceConnection {
  BLEAdvertisedDevice* device;             // Pointer to the device.
  BLERemoteCharacteristic* characteristic; // Pointer to the characteristic of the device.
  bool doConnect;                          // Flag indicating if the device should try to connect.
  bool connected;                          // Flag indicating if the device is connected.
  BLEUUID serviceUUID;                     // UUID for the service
  BLEUUID charUUID;                        // UUID for the characteristic

  // Constructor, initialize the structure with the service and characteristic UUIDs.
  DeviceConnection(const char* serviceUUID, const char* charUUID)
    : device(nullptr), characteristic(nullptr), doConnect(false), connected(false),
      serviceUUID(BLEUUID(serviceUUID)), charUUID(BLEUUID(charUUID)) {}
};

/* Array to store all devices info, it should be mapped accordingly to nodes code. */
DeviceConnection devices[NUMBER_OF_DEVICES] = {
  DeviceConnection("2E01EA51-1E73-4DAC-886D-08C04C040282", "beb5483e-36e1-4688-b7f5-ea07361b26a8"), // Soil moisture sensor
  DeviceConnection("A077916E-2CBE-45E3-BB35-4514C322606F", "beb5483e-36e1-4688-b7f5-ea07361b26a8"), // Humidity sensor
  DeviceConnection("9E61059E-B1CF-4953-A632-39CFD6A97255", "beb5483e-36e1-4688-b7f5-ea07361b26a8")  // Water pump actuator
  //DeviceConnection("8FBB11A7-163B-43C8-9D4F-FCA02297A258", "beb5483e-36e1-4688-b7f5-ea07361b26a8")  // Heater
}; 

/* Callback function for notify. This callback will be invoked only when the characteristic value is changed. */
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* p_data,
  size_t length,
  bool isNotify) {
  Serial.print("Notify callback for device: ");
  for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
    if (devices[i].device != nullptr && devices[i].connected) {
      if (devices[i].device->getAddress().equals(pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress())) {
        Serial.print(devices[i].device->getName().c_str());
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
}

/* Callback function for to handle client connect/diconnect operations. */
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    Serial.println(" - onConnect callback");
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      if (devices[i].device != nullptr && devices[i].device->getAddress() == pclient->getPeerAddress()) { // Check if the device is in the list.
        connectedDevices++;                                                                               // Increment the connected device count.
        devices[i].connected = true;                                                                      // Update the corresponding device status.
        Serial.println(String(" - Device connected: ") + devices[i].device->getName().c_str());
        break;
      }
    }
  }

  void onDisconnect(BLEClient* pclient) override {
    Serial.println(" - onDisconnect callback");
    for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
      if (devices[i].device != nullptr && devices[i].device->getAddress() == pclient->getPeerAddress()) { // Check if the device is in the list.
        connectedDevices--;                                                                               // Decrement the connected device count.
        print_all_devices_flag = true;                                                                    // Set the flag to print all devices when the will be connected again.
        devices[i].connected = false;                                                                     // Update the corresponding device status.
        Serial.println(String(" - Device disconnected: ") + devices[i].device->getName().c_str());
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
  Serial.println(" - Created client");
  pClient->setClientCallbacks(new MyClientCallback());

  // Attempt to connect
  if (!pClient->connect(myDevice)) {
    Serial.println("Failed to connect");
    return false;
  }

  // Connected
  Serial.println("Connected to server");

  // Attempt to retrieve the service
  BLERemoteService* pRemoteService = pClient->getService(devices[deviceIndex].serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find service UUID");
    pClient->disconnect();
    return false;
  }

  // Attempt to retrieve the characteristic
  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(devices[deviceIndex].charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic UUID");
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
          Serial.println(String("Found a device: ") + advertisedDevice.toString().c_str());
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
    Serial.println("---");
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
        Serial.println("Connected to the device");
      }
      else {
        Serial.println("Failed to connect. Retrying...");
        devices[i].doConnect = false;
      }
    }
  }
}

/*
 * Function to send a message to the device.
 * It sends a message to the device with the specified index.
 */
bool send_message_to_device(int deviceIndex, String message) {
  if (devices[deviceIndex].characteristic == nullptr) return false;
  if (!devices[deviceIndex].connected) return false;

  devices[deviceIndex].characteristic->writeValue(message.c_str(), message.length());
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

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
}

void loop() {
  unsigned long current_millis = millis(); // Get the current time, nodes should be updated even if not all devices are connected.

  /* Print all connection if we are successfully connected. */
  if (connectedDevices == NUMBER_OF_DEVICES && print_all_devices_flag) {
    print_all_connections();
    print_all_devices_flag = false;
  }

  /* Try to scan and connect to new devices as long as all slots are not taken. */
  if (connectedDevices < NUMBER_OF_DEVICES) {
    BLEDevice::getScan()->start(5, false); // Rescan for 5 seconds (it has blocking behavior)
    Serial.println("Scanning for devices, connected devices: "
                   + String(connectedDevices) + " / "
                   + String(NUMBER_OF_DEVICES));
    connect_to_devices();
  }

  /* Send something to humidity sensor in specified time intervals (so far just for showcase). */
  if (current_millis - previous_millis_humidity_sensor >= interval_humidity_sensor) {
    previous_millis_humidity_sensor = current_millis;
    String message_to_sent = "Pls work, hum sensor ";
    if (send_message_to_device(HUMIDITY_SENSOR, message_to_sent)) {
      Serial.println("Humidity sensor successfully updated");
    }
    else {
      Serial.println("Humidity sensor update FAILED");
    }
  }

}
