#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_SENSOR "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_ACTUATOR "beb5483e-36e1-4688-b7f5-ea07361b26a9"

BLEServer *pServer = NULL;
BLECharacteristic *pSensorCharacteristic = NULL;
BLECharacteristic *pActuatorCharacteristic = NULL;

bool actuatorState= false;
bool actuatorStateOld=false;


class MySensorCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pSensorCharacteristic) {
    std::string value = pSensorCharacteristic->getValue();  // Read received string

    // Convert string to float
    float floatValue = std::stof(value);

    /* Conditions to verify if the value read follows the boundaries */
    
    // Example 
    if (floatValue >= 0.0 && floatValue <= 100.0) {
        // Value is within boundaries
        Serial.println("Boundaries for sensor followed!");
        if(actuatorState) actuatorState=false;
    } else {
        // Value is outside boundaries
        Serial.println("Boundaries for sensor not followed!");
        if(!actuatorState) actuatorState=true;

    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {  // Run upon client connection
    Serial.println("Device has connected!");
    // Device connected
  }
  void onDisconnect(BLEServer *pServer) {  // Run when client disconnects
    Serial.println("Device has disconnected...");
    // A device has disconnected
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  pinMode(LED_BUILTIN, OUTPUT);

  // Create BLE Device
  BLEDevice::init("Main Controller");

  // Create BLE Server
  pServer = BLEDevice::createServer();
  // Set server callback
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Sensor Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_SENSOR,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);
  // Set initial value
  //pSensorCharacteristic->setValue("0.0, 0.0");
  // Set characteristic callback
  pSensorCharacteristic->setCallbacks(new MySensorCallbacks());

  // Create BLE Actuator Characteristic
  pActuatorCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_ACTUATOR,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);
  // Set initial value
  //pSensorCharacteristic->setValue("0.0, 0.0");
  // Set characteristic callback
  // Start service
  pService->start();

  // Start advertising
  //pAdvertising->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristics defined! Server is now discoverable by client!");
}


void loop() {

  /*
  If the actuator action is required to fulfill the requirements, a activation signal
  is sento to the actuator and a signal is sent to the sensor, so the sensor will increase
  which the data is collected and sent to the Main Controller. 
  The Main Controller notifies both Chracateristics.
  When the System is in the Actuator State and finally the boundaries are followed again, the
  Main Controller sends signals to the sensor and the actuator to go back to Sleep Mode. 
  The Main Controller notifies both Chracateristics.
  */
  if(actuatorState && !actuatorStateOld){
    pActuatorCharacteristic->setValue("1"); // Signal for Actuator to leave Sleep mode
    pSensorCharacteristic->setValue("-1"); //Signal for Sensor to go leave Sleep mode ( now it will send data every 30 seconds)
    pSensorCharacteristic->notify(); // Notify connected BLE client
    pActuatorCharacteristic->notify(); // Notify connected BLE client
    actuatorStateOld=!actuatorStateOld;
  }else if(!actuatorState && actuatorStateOld){
    pActuatorCharacteristic->setValue("0"); // Signal for Actuator to go to Sleep mode (waiting for a Wake Up event when it is required)
    pSensorCharacteristic->setValue("-2"); //Signal for Sensor to go to Sleep mode (send data every 5 minutes)
    pSensorCharacteristic->notify(); // Notify connected BLE client
    pActuatorCharacteristic->notify(); // Notify connected BLE client
    actuatorStateOld=!actuatorStateOld;
  }
  delay(10); // Read every 10 milliseconds
}



