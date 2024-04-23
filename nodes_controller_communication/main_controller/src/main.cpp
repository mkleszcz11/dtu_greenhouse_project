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
#include "lora.h"

#define NUMBER_OF_DEVICES 3  // Number of devices to connect to (number of nodes) - to have more than 3 with Arduino framework drivers alignments are needed.
#define MAX_MESSAGE_SIZE  32 // Maximum size of the message from server, if it is larger, it will be truncated.
                             // IMPORTANT - messages might be truncated also because of MTU size in BLE settings, it is better to
                             // make the message shorter than increase the MTU size.

#define NUMBER_OF_CONTROLLED_PARAMETERS 4 // Number of parameters controled - Temperature, Humidity, Solar Radiation and Soil Moisture.

/*State Machine*/
enum States {
  FIRST_CONNECTION,
  LOW_CONSUMPTION,
  ACTUATOR_MODE
};

/* Enum to store the index of the devices, used later in the code to easlily reference to specific defice. */
enum DeviceIndex {
  SOIL_MOISTURE_SENSOR = 0,
  SOLAR_RADIATION_SENSOR = 1,
  HUMIDITY_TEMPERATURE_SENSOR = 2,
};

// Enumeration for control values.
enum control_values {
  VAL_SOIL_MOISTURE = 0,
  VAL_TEMPERATURE = 1,
  VAL_HUMIDITY = 2,
  VAL_SOLAR_RADIATION = 3,
  VAL_UNKNOWN = 4
};

uint8_t connectedDevices = 0;       // Number of currently connected devices.
bool print_all_devices_flag = true; // Flag used to print all devices when all are connected.

unsigned long interval = 60000;
unsigned long previous_millis = 0;

/* Flag to indicate if a new BLE message was received */
bool new_ble_message_flag = false;

uint8_t sensor_info[NUMBER_OF_CONTROLLED_PARAMETERS]; // {soil moisture, temperature, humidity, solar radiation}
uint8_t control_info[3]; // {value_type, desired_value_lower_bound, desired_value_upper_bound}


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
  SensorActuatorConfig(0,2,SOIL_MOISTURE_SENSOR),        // Soil moisture 
  SensorActuatorConfig(0,2,HUMIDITY_TEMPERATURE_SENSOR), // Temperature 
  SensorActuatorConfig(0,2,HUMIDITY_TEMPERATURE_SENSOR), // Humidity
  SensorActuatorConfig(0,2,SOLAR_RADIATION_SENSOR)       // Solar Radiation 

};

String map_value_index_to_value_name(int deviceIndex) {
  switch (deviceIndex) {
    case 0:
	  return "Soil moisture";
	case 1:
	  return "Temperature";
	case 2:
	  return "Humidity";
	case 3:
	  return "Solar radiation";
	default:
	  return "Unknown";
  }
}

// Map function used to map the code to parameter index, while receiving message about desired values
control_values map_code_to_parameter_idx(int recieved_code) {
  switch (recieved_code) {
    case 0:
      return VAL_SOIL_MOISTURE;
    case 1:
      return VAL_TEMPERATURE;
    case 2:
      return VAL_HUMIDITY;
    case 3:
      return VAL_SOLAR_RADIATION;
    default:
      return VAL_UNKNOWN;
  }
}

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

	//Serial.print("[Main Controller] Notify callback for device: ");
	for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
		if (devices[i].device != nullptr && devices[i].connected) {
		if (devices[i].device->getAddress().equals(pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress())) {
			//Serial.println(devices[i].device->getName().c_str());
			deviceIndexNotification=i;
			break;
		}
		}
	}

	// Print incomming data, handle the null character at the end of the string.
	// Serial.print(" | Data: ");
	char buffer[MAX_MESSAGE_SIZE + 1] = {0};
	size_t cp_length = (length < MAX_MESSAGE_SIZE) ? length : MAX_MESSAGE_SIZE;
	memcpy(buffer, p_data, cp_length);
	// Serial.println(buffer);

	/*LATER: IMPLEMENT READ THE STRING TO FLOAT*/

	float readValue1 = 0.0;
	float readValue2 = 0.0; //readValue2 is used only for HUMIDITY_TEMPERATURE_SENSOR

	readValue1=atof(buffer);
	char* separator = strchr(buffer, ';'); // Find the position of the semicolon

	if(separator!=nullptr){ //HUMIDITY_TEMPERATURE_SENSOR
		readValue2=atof(separator +1);
	}

	Serial.println("[Main Controller] BLE device index: " + String(deviceIndexNotification));
	Serial.println("[Main Controller] BLE Received values: " + String(readValue1)+ " "+ String(readValue2));

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

	// Store the value
	switch (deviceIndexNotification) {
		case SOIL_MOISTURE_SENSOR:
			sensor_info[VAL_SOIL_MOISTURE] = readValue1;
			break;
		case HUMIDITY_TEMPERATURE_SENSOR:
			sensor_info[VAL_TEMPERATURE] = readValue1;
			sensor_info[VAL_HUMIDITY] = readValue2;
			break;
		case SOLAR_RADIATION_SENSOR:
			sensor_info[VAL_SOLAR_RADIATION] = readValue1;
			break;
		default:
			break;
	}

	new_ble_message_flag = true;
}

/* Callback function for to handle client connect/diconnect operations. */
class MyClientCallback : public BLEClientCallbacks {
	void onConnect(BLEClient* pclient) override {
		//Serial.println("[Main Controller] - onConnect callback");
		for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
		if (devices[i].device != nullptr && devices[i].device->getAddress() == pclient->getPeerAddress()) { // Check if the device is in the list.
			connectedDevices++;                                                                               // Increment the connected device count.
			devices[i].connected = true;                                                                      // Update the corresponding device status.
			//Serial.println(String("[Main Controller] - Device connected: ") + devices[i].device->getName().c_str());
			break;
		}
		}
	}

	void onDisconnect(BLEClient* pclient) override {
		//Serial.println("[Main Controller] - onDisconnect callback");
		for (int i = 0; i < NUMBER_OF_DEVICES; i++) {
		if (devices[i].device != nullptr && devices[i].device->getAddress() == pclient->getPeerAddress()) { // Check if the device is in the list.
			connectedDevices--;                                                                               // Decrement the connected device count.
			print_all_devices_flag = true;                                                                    // Set the flag to print all devices when the will be connected again.
			devices[i].connected = false;
			devices[i].doConnect= false;                                                                   // Update the corresponding device status.
			//Serial.println(String("[Main Controller] - Device disconnected: ") + devices[i].device->getName().c_str());
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
	//Serial.println("[Main Controller] - Created client");
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
			//Serial.println(String("[Main Controller] Found a device: ") + advertisedDevice.toString().c_str());
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

void print_explicit_info(uint8_t* control_info) {
  Serial.print("[Main Controller] Request to set desired boundaries for ");
  control_values val_to_control = map_code_to_parameter_idx(control_info[0]);
  switch (val_to_control) {
    case VAL_SOIL_MOISTURE:
      Serial.print("Soil moisture: ");
      Serial.print(control_info[1]);
	  Serial.print(" - ");
	  Serial.println(control_info[2]);
      break;
    case VAL_SOLAR_RADIATION:
      Serial.print("Solar radiation: ");
      Serial.print(control_info[1]);
	  Serial.print(" - ");
	  Serial.println(control_info[2]);
      break;
    case VAL_TEMPERATURE:
      Serial.print("Temperature: ");
      Serial.print(control_info[1]);
	  Serial.print(" - ");
	  Serial.println(control_info[2]);
    case VAL_HUMIDITY:
      Serial.print("Humidity: ");
      Serial.print(control_info[1]);
	  Serial.print(" - ");
	  Serial.println(control_info[2]);
      break;
    default:
      Serial.println("Unknown device");
      break;
  }
}

void setup() {
	Serial.begin(115200);
	Serial.println("[Main Controller] Starting application...");
	BLEDevice::init("main_controller");

	BLEScan* pBLEScan = BLEDevice::getScan();
	pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
	pBLEScan->setActiveScan(true);
	pBLEScan->start(5, false);

	startSystem=true;

	lora_setup();
}

void loop() {
    unsigned long current_millis = millis();

    static unsigned long last_sent_time = millis();
    static unsigned long last_receive_time = millis();
    static bool lora_receive_mode_flag = false; // Flag indicating if module is in the receive mode
    static bool lora_transmit_mode_flag = false; // Flag indicating if module is in the transmit mode
    static bool lora_receive_message_flag = false;

    // /*********************************************************************************/
    // /*** LORA - RECEIVING DESIRED PARAMETERS BOUDARIES VALUES FROM THE REMOTE NODE ***/
    // /*********************************************************************************/

    // Handle receiving
    if (!lora_transmit_mode_flag && !lora_receive_mode_flag) {
        lora_receive_mode_flag = true;
        lora_receive(control_info, 3, &lora_receive_mode_flag, &lora_receive_message_flag);
    }

    // Check for new messages every 5 seconds
    if (lora_receive_mode_flag && current_millis - last_receive_time >= 5000) {
		check_for_incoming_message(control_info, 3, &lora_receive_mode_flag, &lora_receive_message_flag); // Check if there is a message that waits to be processed.
    	last_receive_time = current_millis;
    }

    // Print control info if received
    if (lora_receive_message_flag) {
        print_explicit_info(control_info);
		controlPairs[control_info[0]].lowBoundary = control_info[1];
		controlPairs[control_info[0]].highBoundary = control_info[2];
        lora_receive_message_flag = false; // Reset flag after processing
    }

	// /*******************************************************/
	// /*** LORA - SENDING SENSOR VALUES TO THE REMOTE NODE ***/
	// /*******************************************************/

	// Transmit sensor info every 10 seconds
    if (current_millis - last_sent_time >= 10000) { //current_millis - last_sent_time >= 10000 && !lora_receive_mode_flag) {
		lora_receive_mode_flag = false; // Put the lora module to receive mode again after sending the message.
		check_for_incoming_message(control_info, 3, &lora_receive_mode_flag, &lora_receive_message_flag); // Firstly, check if there is a message that waits to be processed.
        lora_transmit_mode_flag = true;
        lora_transmit(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_transmit_mode_flag, &lora_receive_mode_flag, &lora_receive_message_flag);
		
		last_sent_time = current_millis;
		new_ble_message_flag = false;
    }

	/*************************************/
	/********** BLE Communication ********/
	/*************************************/

	/* Print all connection if we are successfully connected. */
	if (connectedDevices == NUMBER_OF_DEVICES && print_all_devices_flag && startSystem) {
		// print_all_connections();
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
		// Serial.println("[Main Controller] Scanning for devices, connected devices: "
		// 			+ String(connectedDevices) + " / "
		// 			+ String(NUMBER_OF_DEVICES));
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
				if (send_message_to_device(i,devices[i].currentState)) {
					Serial.println(String("[Main Controller] Message sent to Device ") + devices[i].device->getName().c_str() );
				}
				else {
					Serial.println(String("[Main Controller] Error: Message not sent to Device ") + devices[i].device->getName().c_str());
				}
			}
		}
	}

	/**********************************/
	/********** DEBUGGING PART ********/
	/**********************************/

	// Print devices boundaries every 20 seconds
	static unsigned long dbg_last_sent_time = millis();
	if (current_millis - dbg_last_sent_time >= 3000) {
		Serial.println("#############################");
		Serial.println("### DEBUGGING INFORMATION ###");
		Serial.println("#############################");
		Serial.println("-----------");
		Serial.println("CONTROLLED PARAMETERS:");
		for (int i = 0; i < NUMBER_OF_CONTROLLED_PARAMETERS; i++) {
			Serial.print("Parameter: ");
			Serial.print(map_value_index_to_value_name(i));
			Serial.print(": ");
			Serial.print("Low boundary: ");
			Serial.print(controlPairs[i].lowBoundary);
			Serial.print(" - High boundary: ");
			Serial.println(controlPairs[i].highBoundary);
		}
		Serial.println("-----------");
		Serial.println("ACTUAL SENSOR VALUES:");
		for (int i = 0; i < 4; i++) {
			Serial.print("Sensor: ");
			Serial.print(map_value_index_to_value_name(i));
			Serial.print(": ");
			Serial.println(sensor_info[i]);
		}
		Serial.println("-----------");
		Serial.println("#############################");
		dbg_last_sent_time = current_millis;
	}

}
