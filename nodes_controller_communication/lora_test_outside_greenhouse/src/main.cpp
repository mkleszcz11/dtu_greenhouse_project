#include "lora.h"

#define NUMBER_OF_CONTROLLED_PARAMETERS 4 // Number of parameters controled - Temperature, Humidity, Solar Radiation and Soil Moisture.

uint8_t sensor_info[NUMBER_OF_CONTROLLED_PARAMETERS]; // {soil moisture, temperature, humidity, solar radiation}
uint8_t control_info[3]; // {value_type, desired_value_lower_bound, desired_value_upper_bound}

/* Flag to indicate if a new LoRa message was received */
bool new_message_flag = false;

// Enumeration for control parameters, it MUST be aligned to enum in the main controller
enum parameters {
	VAL_SOIL_MOISTURE = 0,
	VAL_TEMPERATURE = 1,
	VAL_HUMIDITY = 2,
	VAL_SOLAR_RADIATION = 3,
	VAL_UNKNOWN = 4
};

enum devices {
	SOIL_MOISTURE_IDX = 0,
	SOLAR_RADIATION_IDX = 1,
	TEMPERATURE_AND_HUMIDITY_IDX = 2,
	UNKNOWN_IDX
};

// uint8_t control_info[4][2] = {
//   {20, 22}, // Desired temperature bounds
//   {40, 60}, // Desired humidity bounds
//   {30, 50}, // Desired soil moisture bounds
//   {60, 70}  // Desired solar radiation bounds (#TODO is this correct?)
// };

devices map_device_idx_to_device_name(uint8_t device_idx) {
  switch (device_idx) {
	case 0:
      return SOIL_MOISTURE_IDX;
    case 1:
      return SOLAR_RADIATION_IDX;
    case 2:
      return TEMPERATURE_AND_HUMIDITY_IDX;
    default:
      return UNKNOWN_IDX;
  }
}

parameters map_control_id_to_control_val(uint8_t control_id) {
  switch (control_id) {
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

String map_control_enum_to_string(int deviceIndex) {
  switch (deviceIndex) {
    case VAL_TEMPERATURE:
      return "Temperature";
    case VAL_HUMIDITY:
      return "Humidity";
    case VAL_SOIL_MOISTURE:
      return "Soil moisture";
    case VAL_SOLAR_RADIATION:
      return "Solar radiation";
    default:
      return "Unknown";
  }
}

void print_explicit_info(uint8_t* sensor_info) {
  Serial.print("[Recieved sensor info:");
  for(int i = 0; i < NUMBER_OF_CONTROLLED_PARAMETERS; i++){
    Serial.print(" " + map_control_enum_to_string(i) + " ");
    Serial.print(sensor_info[i]);
  }
  Serial.println("]");
}

void setup() {
  Serial.begin(57600);
  lora_setup();
}


void loop() {
    unsigned long current_millis = millis();

    static unsigned long last_sent_time = millis();
    static unsigned long last_receive_time = millis();
    static bool lora_receive_mode_flag = false; // Flag indicating if module is in the receive mode
    static bool lora_transmit_mode_flag = false; // Flag indicating if module is in the transmit mode
    static bool lora_receive_message_flag = false;

    static unsigned int interval_incoming_message_check = 1000; // 1 second

    /***************************************************************/
    /*** LORA - RECEIVING SENSOR VALUES FROM THE MAIN CONTROLLER ***/
    /***************************************************************/

    // Handle receiving
    if (!lora_transmit_mode_flag && !lora_receive_mode_flag) {
        lora_receive_mode_flag = true;
        lora_receive(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag); // Put lora module in receive mode
    }

    // Check for new messages every 1 seconds
    if (lora_receive_mode_flag && current_millis - last_receive_time >= interval_incoming_message_check) {
		  check_for_incoming_message(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag); // Check if there is a message that waits to be processed.
    	last_receive_time = current_millis;	
    }

    // Print sensor info if received
    if (lora_receive_message_flag) {
        print_explicit_info(sensor_info);
        lora_receive_message_flag = false; // Reset flag after processing
    }

    /**************************************************************************/
    /*** LORA - SENDING DESIRED PARAMETERS BOUDARIES TO THE MAIN CONTROLLER ***/
    /**************************************************************************/

	// TODO change that -> for now we are generating some random parameters to send
    control_info[0] = VAL_SOLAR_RADIATION; 			  // Parameter ID, check "parameters" enum for details (e.g. VAL_TEMPERATURE)
    control_info[1] = random(0, 100);					  // Lower bound
    control_info[2] = random(control_info[1], 100); // Upper bound

    // Handle transmission every X seconds or as required
    // TODO change that -> we should send the message if some value should be changed
    // or we can send the message every X seconds to update the desired value for specified parameter
    if (current_millis - last_sent_time >= 10000) {
      Serial.print("[communication node] Transmitting new parameters for " + map_control_enum_to_string(map_control_id_to_control_val(control_info[0])));
      Serial.println(" (" + String(control_info[1]) + " - " + String(control_info[2]) + ")");
      last_sent_time = current_millis;
      lora_receive_mode_flag = false; // Put the lora module to receive mode again after sending the message.
      check_for_incoming_message(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag); // Firstly, check if there is a message that waits to be processed.
      lora_transmit_mode_flag = true;
      lora_transmit(control_info, 3, &lora_transmit_mode_flag, &lora_receive_mode_flag, &lora_receive_message_flag); // Pass the receive message flag as we could enter this function with message waiting to be processed.
    }


    /************************************/
    /*** WIFI AND CLOUD COMMUNICATION ***/
    /************************************/

    // TODO important -> implement in the non-blocking manner, otherwise lora communication will be blocked




	/**********************************/
	/********** DEBUGGING PART ********/
	/**********************************/

	// Print devices boundaries every 5 seconds
	static unsigned long dbg_last_sent_time = millis();
	if (current_millis - dbg_last_sent_time >= 3000) {
		Serial.println("#############################");
		Serial.println("### DEBUGGING INFORMATION ###");
		Serial.println("#############################");
		Serial.println("-----------");
		Serial.println("ACTUAL SENSOR VALUES:");
		for (int i = 0; i < 4; i++) {
			Serial.print("Sensor: ");
			Serial.print(map_control_enum_to_string(map_control_id_to_control_val(i)));
			Serial.print(": ");
			Serial.println(sensor_info[i]);
		}
		Serial.println("-----------");
		Serial.println("#############################");
		dbg_last_sent_time = current_millis;
	}

}
