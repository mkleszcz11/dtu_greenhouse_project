// C99 libraries
#include <cstdarg>
#include <cstdlib>
#include <string.h>
#include <time.h>

// For hmac SHA256 encryption
#include <mbedtls/base64.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "AzureIoT.h"
#include "Azure_IoT_PnP_Template.h"
#include "iot_configs.h"

// Library for sending JSON
#include <ArduinoJson.h>

// Library for LoRa communication
#include "lora.h"

#define NUMBER_OF_CONTROLLED_PARAMETERS 4  // Number of parameters controled - Temperature, Humidity, Solar Radiation and Soil Moisture.

uint8_t sensor_info[NUMBER_OF_CONTROLLED_PARAMETERS];  // {soil moisture, temperature, humidity, solar radiation}
uint8_t control_info[2];                               // {topic_type,value}

/* Flag to indicate if a new LoRa message was received */
bool new_message_flag = false;

// Enumeration for control parameters, it MUST be aligned to enum in the main controller
enum control_parameters {
  VAL_TEMPERATURE_MINIMUM = 0,
  VAL_TEMPERATURE_MAXIMUM = 1,
  VAL_HUMIDITY_MINIMUM = 2,
  VAL_HUMIDITY_MAXIMUM = 3,
  VAL_SOIL_MOISTURE_MINIMUM = 4,
  VAL_SOIL_MOISTURE_MAXIMUM = 5,
  VAL_WATER_LEVEL_MINIMUM = 6,
  VAL_WATER_LEVEL_MAXIMUM = 7,
  VAL_UNKNOWN_CONTROL = 8
};

enum sensor_parameters {
  VAL_TEMPERATURE = 0,
  VAL_HUMIDITY = 1,
  VAL_SOIL_MOISTURE = 2,
  VAL_WATER_LEVEL = 3,
  VAL_UNKNOWN_SENSOR = 4
};


enum devices {
  SOIL_MOISTURE_IDX = 0,
  WATER_LEVEL_IDX = 1,
  TEMPERATURE_AND_HUMIDITY_IDX = 2,
  UNKNOWN_IDX
};

devices map_device_idx_to_device_name(uint8_t device_idx) {
  switch (device_idx) {
    case 0:
      return SOIL_MOISTURE_IDX;
    case 1:
      return WATER_LEVEL_IDX;
    case 2:
      return TEMPERATURE_AND_HUMIDITY_IDX;
    default:
      return UNKNOWN_IDX;
  }
}

control_parameters map_control_id_to_control_val(uint8_t control_id) {
  switch (control_id) {
    case 0:
      return VAL_TEMPERATURE_MINIMUM;
    case 1:
      return VAL_TEMPERATURE_MAXIMUM;
    case 2:
      return VAL_HUMIDITY_MINIMUM;
    case 3:
      return VAL_HUMIDITY_MAXIMUM;
    case 4:
      return VAL_SOIL_MOISTURE_MINIMUM;
    case 5:
      return VAL_SOIL_MOISTURE_MAXIMUM;
    case 6:
      return VAL_WATER_LEVEL_MINIMUM;
    case 7:
      return VAL_WATER_LEVEL_MAXIMUM;
    default:
      return VAL_UNKNOWN_CONTROL;
  }
}

sensor_parameters map_control_id_to_sensor_val(uint8_t sensor_id) {
  switch (sensor_id) {
    case 0:
      return VAL_TEMPERATURE;
    case 1:
      return VAL_HUMIDITY;
    case 2:
      return VAL_SOIL_MOISTURE;
    case 3:
      return VAL_WATER_LEVEL;
    default:
      return VAL_UNKNOWN_SENSOR;
  }
}

// String map_control_enum_to_string(int deviceIndex) {
//   switch (deviceIndex) {
//     case VAL_TEMPERATURE_MINIMUM:
//       return "TemperatureMin";
//     case VAL_TEMPERATURE_MAXIMUM:
//       return "TemperatureMax";
//     case VAL_HUMIDITY_MINIMUM:
//       return "HumidityMin";
//     case VAL_HUMIDITY_MAXIMUM:
//       return "HumidityMax";
//     case VAL_SOIL_MOISTURE_MINIMUM:
//       return "SoilMoistureMin";
//     case VAL_SOIL_MOISTURE_MAXIMUM:
//       return "SoilMoistureMax";
//     case VAL_WATER_LEVEL_MINIMUM:
//       return "WaterLevelMin";
//     case VAL_WATER_LEVEL_MAXIMUM:
//       return "WaterLevelMax";
//     default:
//       return "Unknown";
//   }
// }

String map_control_enum_to_string(int deviceIndex) {
  switch (deviceIndex) {
    case VAL_TEMPERATURE:
      return "Temperature";
    case VAL_HUMIDITY:
      return "Humidity";
    case VAL_SOIL_MOISTURE:
      return "SoilMoisture";
    case VAL_WATER_LEVEL:
      return "WaterLevel";
    default:
      return "Unknown";
  }
}

void print_explicit_info(uint8_t* sensor_info) {
  Serial.print("[Received sensor info:");
  for (int i = 0; i < NUMBER_OF_CONTROLLED_PARAMETERS; i++) {
    Serial.print(" " + map_control_enum_to_string(i) + " ");
    Serial.print(sensor_info[i]);
  }
  Serial.println("]");
}

/* --- Sample-specific Settings --- */
#define SERIAL_LOGGER_BAUD_RATE 115200
#define MQTT_DO_NOT_RETAIN_MSG 0

/* --- Time and NTP Settings --- */
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"

#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1

#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

#define UNIX_TIME_NOV_13_2017 1510592825
#define UNIX_EPOCH_START_YEAR 1900

/* --- Function Returns --- */
#define RESULT_OK 0
#define RESULT_ERROR __LINE__

/* --- Handling iot_config.h Settings --- */
static const char* wifi_ssid = IOT_CONFIG_WIFI_SSID;
static const char* wifi_password = IOT_CONFIG_WIFI_PASSWORD;

/* --- Function Declarations --- */
static void sync_device_clock_with_ntp_server();
static void connect_to_wifi();
static esp_err_t esp_mqtt_event_handler(esp_mqtt_event_handle_t event);

// This is a logging function used by Azure IoT client.
static void logging_function(log_level_t log_level, char const* const format, ...);

/* --- Sample variables --- */
static azure_iot_config_t azure_iot_config;
static azure_iot_t azure_iot;
static esp_mqtt_client_handle_t mqtt_client;

static char mqtt_broker_uri[128];

#define AZ_IOT_DATA_BUFFER_SIZE 1500
static uint8_t az_iot_data_buffer[AZ_IOT_DATA_BUFFER_SIZE];

#define MQTT_PROTOCOL_PREFIX "mqtts://"

static uint32_t properties_request_id = 0;
static bool send_device_info = true;
static bool azure_initial_connect = false;  //Turns true when ESP32 successfully connects to Azure IoT Central for the first time

// Identifier to trigger LoRa transmission to main controller
bool fresh_data_transmit = false;

// Define a global uint8_t array to store the payload
#define MAX_PAYLOAD_SIZE 256  // Adjust the size according to your payload size
uint8_t received_payload_value = 0;
size_t received_payload_size = 0;
#define MAX_TOPIC_SIZE 128
char received_topic[MAX_TOPIC_SIZE];

/* --- MQTT Interface Functions --- */
/*
 * These functions are used by Azure IoT to interact with whatever MQTT client used by the sample
 * (in this case, Espressif's ESP MQTT). Please see the documentation in AzureIoT.h for more
 * details.
 */

/*
 * See the documentation of `mqtt_client_init_function_t` in AzureIoT.h for details.
 */
static int mqtt_client_init_function(
  mqtt_client_config_t* mqtt_client_config,
  mqtt_client_handle_t* mqtt_client_handle) {
  int result;
  esp_mqtt_client_config_t mqtt_config;
  memset(&mqtt_config, 0, sizeof(mqtt_config));

  az_span mqtt_broker_uri_span = AZ_SPAN_FROM_BUFFER(mqtt_broker_uri);
  mqtt_broker_uri_span = az_span_copy(mqtt_broker_uri_span, AZ_SPAN_FROM_STR(MQTT_PROTOCOL_PREFIX));
  mqtt_broker_uri_span = az_span_copy(mqtt_broker_uri_span, mqtt_client_config->address);
  az_span_copy_u8(mqtt_broker_uri_span, null_terminator);

  mqtt_config.uri = mqtt_broker_uri;
  mqtt_config.port = mqtt_client_config->port;
  mqtt_config.client_id = (const char*)az_span_ptr(mqtt_client_config->client_id);
  mqtt_config.username = (const char*)az_span_ptr(mqtt_client_config->username);

#ifdef IOT_CONFIG_USE_X509_CERT
  LogInfo("MQTT client using X509 Certificate authentication");
  mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
  mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
#else  // Using SAS key
  mqtt_config.password = (const char*)az_span_ptr(mqtt_client_config->password);
#endif

  mqtt_config.keepalive = 30;
  mqtt_config.disable_clean_session = 0;
  mqtt_config.disable_auto_reconnect = false;
  mqtt_config.event_handle = esp_mqtt_event_handler;
  mqtt_config.user_context = NULL;
  mqtt_config.cert_pem = (const char*)ca_pem;

  LogInfo("MQTT client target uri set to '%s'", mqtt_broker_uri);

  mqtt_client = esp_mqtt_client_init(&mqtt_config);

  if (mqtt_client == NULL) {
    LogError("esp_mqtt_client_init failed.");
    result = 1;
  } else {
    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

    if (start_result != ESP_OK) {
      LogError("esp_mqtt_client_start failed (error code: 0x%08x).", start_result);
      result = 1;
    } else {
      *mqtt_client_handle = mqtt_client;
      result = 0;
    }
  }

  return result;
}

/*
 * See the documentation of `mqtt_client_deinit_function_t` in AzureIoT.h for details.
 */
static int mqtt_client_deinit_function(mqtt_client_handle_t mqtt_client_handle) {
  int result = 0;
  esp_mqtt_client_handle_t esp_mqtt_client_handle = (esp_mqtt_client_handle_t)mqtt_client_handle;

  LogInfo("MQTT client being disconnected.");

  if (esp_mqtt_client_stop(esp_mqtt_client_handle) != ESP_OK) {
    LogError("Failed stopping MQTT client.");
  }

  if (esp_mqtt_client_destroy(esp_mqtt_client_handle) != ESP_OK) {
    LogError("Failed destroying MQTT client.");
  }

  if (azure_iot_mqtt_client_disconnected(&azure_iot) != 0) {
    LogError("Failed updating azure iot client of MQTT disconnection.");
  }

  return 0;
}

/*
 * See the documentation of `mqtt_client_subscribe_function_t` in AzureIoT.h for details.
 */
static int mqtt_client_subscribe_function(
  mqtt_client_handle_t mqtt_client_handle,
  az_span topic,
  mqtt_qos_t qos) {
  LogInfo("MQTT client subscribing to '%.*s'", az_span_size(topic), az_span_ptr(topic));

  // As per documentation, `topic` always ends with a null-terminator.
  // esp_mqtt_client_subscribe returns the packet id or negative on error already, so no conversion
  // is needed.
  int packet_id = esp_mqtt_client_subscribe(
    (esp_mqtt_client_handle_t)mqtt_client_handle, (const char*)az_span_ptr(topic), (int)qos);

  return packet_id;
}

/*
 * See the documentation of `mqtt_client_publish_function_t` in AzureIoT.h for details.
 */
static int mqtt_client_publish_function(
  mqtt_client_handle_t mqtt_client_handle,
  mqtt_message_t* mqtt_message) {
  LogInfo("MQTT client publishing to '%s'", az_span_ptr(mqtt_message->topic));

  int mqtt_result = esp_mqtt_client_publish(
    (esp_mqtt_client_handle_t)mqtt_client_handle,
    (const char*)az_span_ptr(mqtt_message->topic),  // topic is always null-terminated.
    (const char*)az_span_ptr(mqtt_message->payload),
    az_span_size(mqtt_message->payload),
    (int)mqtt_message->qos,
    MQTT_DO_NOT_RETAIN_MSG);

  if (mqtt_result == -1) {
    return RESULT_ERROR;
  } else {
    return RESULT_OK;
  }
}

/* --- Other Interface functions required by Azure IoT --- */

/*
 * See the documentation of `hmac_sha256_encryption_function_t` in AzureIoT.h for details.
 */
static int mbedtls_hmac_sha256(
  const uint8_t* key,
  size_t key_length,
  const uint8_t* payload,
  size_t payload_length,
  uint8_t* signed_payload,
  size_t signed_payload_size) {
  (void)signed_payload_size;
  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key, key_length);
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)payload, payload_length);
  mbedtls_md_hmac_finish(&ctx, (byte*)signed_payload);
  mbedtls_md_free(&ctx);

  return 0;
}

/*
 * See the documentation of `base64_decode_function_t` in AzureIoT.h for details.
 */
static int base64_decode(
  uint8_t* data,
  size_t data_length,
  uint8_t* decoded,
  size_t decoded_size,
  size_t* decoded_length) {
  return mbedtls_base64_decode(decoded, decoded_size, decoded_length, data, data_length);
}

/*
 * See the documentation of `base64_encode_function_t` in AzureIoT.h for details.
 */
static int base64_encode(
  uint8_t* data,
  size_t data_length,
  uint8_t* encoded,
  size_t encoded_size,
  size_t* encoded_length) {
  return mbedtls_base64_encode(encoded, encoded_size, encoded_length, data, data_length);
}

/*
 * See the documentation of `properties_update_completed_t` in AzureIoT.h for details.
 */
static void on_properties_update_completed(uint32_t request_id, az_iot_status status_code) {
  LogInfo("Properties update request completed (id=%d, status=%d)", request_id, status_code);
}

/*
 * See the documentation of `properties_received_t` in AzureIoT.h for details.
 */
void on_properties_received(az_span properties) {
  LogInfo("Properties update received: %.*s", az_span_size(properties), az_span_ptr(properties));

  // It is recommended not to perform work within callbacks.
  // The properties are being handled here to simplify the sample.
  if (azure_pnp_handle_properties_update(&azure_iot, properties, properties_request_id++) != 0) {
    LogError("Failed handling properties update.");
  }
}

/*
 * See the documentation of `command_request_received_t` in AzureIoT.h for details.
 */
static void on_command_request_received(command_request_t command) {
  az_span component_name = az_span_size(command.component_name) == 0 ? AZ_SPAN_FROM_STR("") : command.component_name;

  LogInfo(
    "Command request received (id=%.*s, component=%.*s, name=%.*s)",
    az_span_size(command.request_id),
    az_span_ptr(command.request_id),
    az_span_size(component_name),
    az_span_ptr(component_name),
    az_span_size(command.command_name),
    az_span_ptr(command.command_name));

  // Here the request is being processed within the callback that delivers the command request.
  // However, for production application the recommendation is to save `command` and process it
  // outside this callback, usually inside the main thread/task/loop.
  (void)azure_pnp_handle_command_request(&azure_iot, command);
}

static void configure_azure_iot() {
  /*
   * The configuration structure used by Azure IoT must remain unchanged (including data buffer)
   * throughout the lifetime of the sample. This variable must also not lose context so other
   * components do not overwrite any information within this structure.
   */
  azure_iot_config.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);
  azure_iot_config.model_id = azure_pnp_get_model_id();
  azure_iot_config.use_device_provisioning = true;  // Required for Azure IoT Central.
  azure_iot_config.iot_hub_fqdn = AZ_SPAN_EMPTY;
  azure_iot_config.device_id = AZ_SPAN_EMPTY;

#ifdef IOT_CONFIG_USE_X509_CERT
  azure_iot_config.device_certificate = AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_CERT);
  azure_iot_config.device_certificate_private_key = AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY);
  azure_iot_config.device_key = AZ_SPAN_EMPTY;
#else
  azure_iot_config.device_certificate = AZ_SPAN_EMPTY;
  azure_iot_config.device_certificate_private_key = AZ_SPAN_EMPTY;
  azure_iot_config.device_key = AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY);
#endif  // IOT_CONFIG_USE_X509_CERT

  azure_iot_config.dps_id_scope = AZ_SPAN_FROM_STR(DPS_ID_SCOPE);
  azure_iot_config.dps_registration_id = AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_ID);  // Use Device ID for Azure IoT Central.
  azure_iot_config.data_buffer = AZ_SPAN_FROM_BUFFER(az_iot_data_buffer);
  azure_iot_config.sas_token_lifetime_in_minutes = MQTT_PASSWORD_LIFETIME_IN_MINUTES;
  azure_iot_config.mqtt_client_interface.mqtt_client_init = mqtt_client_init_function;
  azure_iot_config.mqtt_client_interface.mqtt_client_deinit = mqtt_client_deinit_function;
  azure_iot_config.mqtt_client_interface.mqtt_client_subscribe = mqtt_client_subscribe_function;
  azure_iot_config.mqtt_client_interface.mqtt_client_publish = mqtt_client_publish_function;
  azure_iot_config.data_manipulation_functions.hmac_sha256_encrypt = mbedtls_hmac_sha256;
  azure_iot_config.data_manipulation_functions.base64_decode = base64_decode;
  azure_iot_config.data_manipulation_functions.base64_encode = base64_encode;
  azure_iot_config.on_properties_update_completed = on_properties_update_completed;
  azure_iot_config.on_properties_received = on_properties_received;
  azure_iot_config.on_command_request_received = on_command_request_received;

  azure_iot_init(&azure_iot, &azure_iot_config);
}

/* --- Arduino setup and loop Functions --- */
void setup() {
  Serial.begin(SERIAL_LOGGER_BAUD_RATE);
  set_logging_function(logging_function);

  connect_to_wifi();
  sync_device_clock_with_ntp_server();

  azure_pnp_init();

  configure_azure_iot();
  azure_iot_start(&azure_iot);

  LogInfo("Azure IoT client initialized (state=%d)", azure_iot.state);
  lora_setup();
}

void loop() {

  unsigned long current_millis = millis();

  //static unsigned long last_sent_time = millis();
  static unsigned long last_receive_time = millis();
  static bool lora_receive_mode_flag = false;   // Flag indicating if module is in the receive mode
  static bool lora_transmit_mode_flag = false;  // Flag indicating if module is in the transmit mode
  static bool lora_receive_message_flag = false;

  static unsigned int interval_incoming_message_check = 1000;  // 1 second

  /***************************************************************/
  /*** LORA - RECEIVING SENSOR VALUES FROM THE MAIN CONTROLLER ***/
  /***************************************************************/


  if (WiFi.status() != WL_CONNECTED) {
    azure_iot_stop(&azure_iot);

    connect_to_wifi();

    if (!azure_initial_connect) {
      configure_azure_iot();
    }

    azure_iot_start(&azure_iot);
  } else {
    switch (azure_iot_get_status(&azure_iot)) {
      case azure_iot_connected:
        azure_initial_connect = true;


        // Handle receiving
        if (!lora_transmit_mode_flag && !lora_receive_mode_flag) {
          lora_receive_mode_flag = true;
          lora_receive(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag);  // Put lora module in receive mode
        }

        // Check for new messages every 1 seconds
        if (lora_receive_mode_flag && current_millis - last_receive_time >= interval_incoming_message_check) {
          check_for_incoming_message(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag);  // Check if there is a message that waits to be processed.
          last_receive_time = current_millis;
        }

        // Print sensor info if received
        if (lora_receive_message_flag) {
          print_explicit_info(sensor_info);
        }


        /**************************************************************************/
        /*** LORA - SENDING DESIRED PARAMETERS BOUDARIES TO THE MAIN CONTROLLER ***/
        /**************************************************************************/

        // Handle transmission every X seconds or as required
        // TODO change that -> we should send the message if some value should be changed
        // or we can send the message every X seconds to update the desired value for specified parameter
        if (fresh_data_transmit) {
          Serial.print("[communication node] Transmitting new parameters for " + map_control_enum_to_string(map_control_id_to_control_val(control_info[0])));
          Serial.println(" (" + String(control_info[1]) + ")");
          //last_sent_time = current_millis;
          lora_receive_mode_flag = false;                                                                                                 // Put the lora module to receive mode again after sending the message.
          check_for_incoming_message(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag);  // Firstly, check if there is a message that waits to be processed.
          lora_transmit_mode_flag = true;
          lora_transmit(control_info, 2, &lora_transmit_mode_flag, &lora_receive_mode_flag, &lora_receive_message_flag);  // Pass the receive message flag as we could enter this function with message waiting to be processed.
          fresh_data_transmit = false;
        }


        if (send_device_info) {
          (void)azure_pnp_send_device_info(&azure_iot, properties_request_id++);
          send_device_info = false;  // Only need to send once.
        } else if (azure_pnp_send_telemetry(&azure_iot) != 0) {
          LogError("Failed sending telemetry.");
        }
        break;

      case azure_iot_error:
        LogError("Azure IoT client is in error state.");
        azure_iot_stop(&azure_iot);
        break;

      case azure_iot_disconnected:
        WiFi.disconnect();
        break;

      default:
        break;
    }

    azure_iot_do_work(&azure_iot);
  }
}

/* === Function Implementations === */

/*
 * These are support functions used by the sample itself to perform its basic tasks
 * of connecting to the internet, syncing the board clock, ESP MQTT client event handler
 * and logging.
 */

/* --- System and Platform Functions --- */
static void sync_device_clock_with_ntp_server() {
  LogInfo("Setting time using SNTP");

  configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
  time_t now = time(NULL);
  while (now < UNIX_TIME_NOV_13_2017) {
    delay(500);
    Serial.print(".");
    now = time(NULL);
  }
  Serial.println("");
  LogInfo("Time initialized!");
}

static void connect_to_wifi() {
  LogInfo("Connecting to WIFI wifi_ssid %s", wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");

  LogInfo("WiFi connected, IP address: %s", WiFi.localIP().toString().c_str());
}

static esp_err_t esp_mqtt_event_handler(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
    int i, r;

    case MQTT_EVENT_ERROR:
      LogError("MQTT client in ERROR state.");
      LogError(
        "esp_tls_stack_err=%d; "
        "esp_tls_cert_verify_flags=%d;esp_transport_sock_errno=%d;error_type=%d;connect_return_"
        "code=%d",
        event->error_handle->esp_tls_stack_err,
        event->error_handle->esp_tls_cert_verify_flags,
        event->error_handle->esp_transport_sock_errno,
        event->error_handle->error_type,
        event->error_handle->connect_return_code);

      switch (event->error_handle->connect_return_code) {
        case MQTT_CONNECTION_ACCEPTED:
          LogError("connect_return_code=MQTT_CONNECTION_ACCEPTED");
          break;
        case MQTT_CONNECTION_REFUSE_PROTOCOL:
          LogError("connect_return_code=MQTT_CONNECTION_REFUSE_PROTOCOL");
          break;
        case MQTT_CONNECTION_REFUSE_ID_REJECTED:
          LogError("connect_return_code=MQTT_CONNECTION_REFUSE_ID_REJECTED");
          break;
        case MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE:
          LogError("connect_return_code=MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE");
          break;
        case MQTT_CONNECTION_REFUSE_BAD_USERNAME:
          LogError("connect_return_code=MQTT_CONNECTION_REFUSE_BAD_USERNAME");
          break;
        case MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED:
          LogError("connect_return_code=MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED");
          break;
        default:
          LogError("connect_return_code=unknown (%d)", event->error_handle->connect_return_code);
          break;
      };

      break;
    case MQTT_EVENT_CONNECTED:
      LogInfo("MQTT client connected (session_present=%d).", event->session_present);

      if (azure_iot_mqtt_client_connected(&azure_iot) != 0) {
        LogError("azure_iot_mqtt_client_connected failed.");
      }

      break;
    case MQTT_EVENT_DISCONNECTED:
      LogInfo("MQTT client disconnected.");

      if (azure_iot_mqtt_client_disconnected(&azure_iot) != 0) {
        LogError("azure_iot_mqtt_client_disconnected failed.");
      }

      break;
    case MQTT_EVENT_SUBSCRIBED:
      LogInfo("MQTT topic subscribed (message id=%d).", event->msg_id);

      if (azure_iot_mqtt_client_subscribe_completed(&azure_iot, event->msg_id) != 0) {
        LogError("azure_iot_mqtt_client_subscribe_completed failed.");
      }

      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      LogInfo("MQTT topic unsubscribed.");
      break;
    case MQTT_EVENT_PUBLISHED:
      LogInfo("MQTT event MQTT_EVENT_PUBLISHED");

      if (azure_iot_mqtt_client_publish_completed(&azure_iot, event->msg_id) != 0) {
        LogError("azure_iot_mqtt_client_publish_completed failed (message id=%d).", event->msg_id);
      }

      break;
    case MQTT_EVENT_DATA:
      LogInfo("MQTT message received.");
      Serial.print("Topic: ");
      Serial.println(String(event->topic, event->topic_len));
      Serial.print("Payload: ");
      Serial.println(String(event->data, event->data_len));

      // Parse the payload data as an integer and store it
      received_payload_value = 0;
      for (size_t i = 0; i < event->data_len; i++) {
        if (event->data[i] >= '0' && event->data[i] <= '9') {
          received_payload_value = received_payload_value * 10 + (event->data[i] - '0');
        }
      }

      //size_t topic_len = event->topic_len;

      // Parse and store the topic
      if (event->topic_len < MAX_TOPIC_SIZE) {
        memcpy(received_topic, event->topic, event->topic_len);
        received_topic[event->topic_len] = '\0';  // Null-terminate the string
      } else {
        Serial.println("Topic exceeds maximum size. Data not saved.");
      }

      // Parse and store the topic, removing everything after "?"


      Serial.print("Topic: ");
      Serial.println(received_topic);
      // Remove everything after last slash
      int remove_last_slash;
      for (int i = event->topic_len; i >= 0; i--) {
        if (received_topic[i] == '/') {
          received_topic[i] = '\0';
          remove_last_slash = i;
          break;
        }
      }

      // Remove everything before (new) last slash
      int last_slash_index;
      last_slash_index = -1;
      for (int i = remove_last_slash; i >= 0; i--) {
        if (received_topic[i] == '/') {
          received_topic[i] = '\0';
          last_slash_index = i;
          break;
        }
      }
      // Remove everything before the (new) last slash
      if (last_slash_index != -1) {
        // Shift the string to start from the character after the last slash
        memmove(received_topic, received_topic + last_slash_index + 1, event->topic_len - last_slash_index);
      }


      // Print the received payload as a single integer value
      Serial.print("Received Payload Value: ");
      Serial.println(received_payload_value);

      Serial.print("Topic: ");
      Serial.println(received_topic);
      if (strcmp(received_topic, "SetTemperatureMinimum") == 0) {
        Serial.println("SUCCESS");
      }

      if (strcmp(received_topic, "SetTemperatureMinimum") == 0) {
        control_info[0] = VAL_TEMPERATURE_MINIMUM;
      } else if (strcmp(received_topic, "SetTemperatureMaximum") == 0) {
        control_info[0] = VAL_TEMPERATURE_MAXIMUM;
      } else if (strcmp(received_topic, "SetHumidityMinimum") == 0) {
        control_info[0] = VAL_HUMIDITY_MINIMUM;
      } else if (strcmp(received_topic, "SetHumidityMaximum") == 0) {
        control_info[0] = VAL_HUMIDITY_MAXIMUM;
      } else if (strcmp(received_topic, "SetWaterLevelMinimum") == 0) {
        control_info[0] = VAL_WATER_LEVEL_MINIMUM;
      } else if (strcmp(received_topic, "SetWaterLevelMaximum") == 0) {
        control_info[0] = VAL_WATER_LEVEL_MAXIMUM;
      } else if (strcmp(received_topic, "SetSoilMoistureMinimum") == 0) {
        control_info[0] = VAL_SOIL_MOISTURE_MINIMUM;
      } else if (strcmp(received_topic, "SetSoilMoistureMaximum") == 0) {
        control_info[0] = VAL_SOIL_MOISTURE_MAXIMUM;
      } else {
        control_info[0] = VAL_UNKNOWN_CONTROL;
        Serial.println("Invalid control value");
      }

      control_info[1] = received_payload_value;  // Lower bound


      mqtt_message_t mqtt_message;
      mqtt_message.topic = az_span_create((uint8_t*)event->topic, event->topic_len);
      mqtt_message.payload = az_span_create((uint8_t*)event->data, event->data_len);
      mqtt_message.qos = mqtt_qos_at_most_once;  // QoS is unused by azure_iot_mqtt_client_message_received.

      if (azure_iot_mqtt_client_message_received(&azure_iot, &mqtt_message) != 0) {
        LogError(
          "azure_iot_mqtt_client_message_received failed (topic=%.*s).",
          event->topic_len,
          event->topic);
      }
      if (control_info[0] != VAL_UNKNOWN_CONTROL) {
        fresh_data_transmit = true;
      }
      
      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      LogInfo("MQTT client connecting.");
      break;
    default:
      LogError("MQTT event UNKNOWN.");
      break;
  }

  return ESP_OK;
}

uint8_t sensor_temperature = 22;
uint8_t sensor_humidity = 24;
uint8_t sensor_soilMoisture = 70;
uint8_t sensor_waterLevel = 98;


static void logging_function(log_level_t log_level, char const* const format, ...) {
  struct tm* ptm;
  time_t now = time(NULL);

  ptm = gmtime(&now);

  Serial.print(ptm->tm_year + UNIX_EPOCH_START_YEAR);
  Serial.print("/");
  Serial.print(ptm->tm_mon + 1);
  Serial.print("/");
  Serial.print(ptm->tm_mday);
  Serial.print(" ");

  if (ptm->tm_hour < 10) {
    Serial.print(0);
  }

  Serial.print(ptm->tm_hour);
  Serial.print(":");

  if (ptm->tm_min < 10) {
    Serial.print(0);
  }

  Serial.print(ptm->tm_min);
  Serial.print(":");

  if (ptm->tm_sec < 10) {
    Serial.print(0);
  }

  Serial.print(ptm->tm_sec);

  Serial.print(log_level == log_level_info ? " [INFO] " : " [ERROR] ");

  char message[256];
  va_list ap;
  va_start(ap, format);
  int message_length = vsnprintf(message, 256, format, ap);
  va_end(ap);

  if (message_length < 0) {
    Serial.println("Failed encoding log message (!)");
  } else {
    Serial.println(message);
  }
}
