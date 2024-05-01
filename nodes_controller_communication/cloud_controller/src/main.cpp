// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs.h"

// Library for sending JSON
#include <ArduinoJson.h>

// Lora communication
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

devices map_device_idx_to_device_name(uint8_t device_idx)
{
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

parameters map_control_id_to_control_val(uint8_t control_id)
{
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

String map_control_enum_to_string(int deviceIndex)
{
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

void print_explicit_info(uint8_t* sensor_info)
{
    Serial.print("[Recieved sensor info:");
    for (int i = 0; i < NUMBER_OF_CONTROLLED_PARAMETERS; i++) {
        Serial.print(" " + map_control_enum_to_string(i) + " ");
        Serial.print(sensor_info[i]);
    }
    Serial.println("]");
}

#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)"

// Utility macros and defines
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define UNIX_TIME_NOV_13_2017 1510592825

#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1

#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

// Translate iot_configs.h defines into variables used by the sample
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;
static const char* host = IOT_CONFIG_IOTHUB_FQDN;
static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
static const char* device_id = IOT_CONFIG_DEVICE_ID;
static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

// Memory allocated for the sample's variables and structures.
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client client;

static char mqtt_client_id[128];
static char mqtt_username[128];
static char mqtt_password[200];
static uint8_t sas_signature_buffer[256];
static unsigned long next_telemetry_send_time_ms = 0;
static char telemetry_topic[128];
// static uint32_t telemetry_send_count = 0;
static String telemetry_payload = "{}";
bool freshData = false;
const char* delimiter = ",";

#define INCOMING_DATA_BUFFER_SIZE 128
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

// Auxiliary functions
#ifndef IOT_CONFIG_USE_X509_CERT
static AzIoTSasToken sasToken(
    &client,
    AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY),
    AZ_SPAN_FROM_BUFFER(sas_signature_buffer),
    AZ_SPAN_FROM_BUFFER(mqtt_password));
#endif // IOT_CONFIG_USE_X509_CERT

static void connectToWiFi()
{
    Logger.Info("Connecting to WIFI SSID " + String(ssid));

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");

    Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());
}

static void initializeTime()
{
    Logger.Info("Setting time using SNTP");

    configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
    time_t now = time(NULL);
    while (now < UNIX_TIME_NOV_13_2017) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("");
    Logger.Info("Time initialized!");
}

void receivedCallback(char* topic, byte* payload, unsigned int length)
{
    Logger.Info("Received [");
    Logger.Info(topic);
    Logger.Info("]: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println("");
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{

    char* token;

    switch (event->event_id) {
        int i, r;

    case MQTT_EVENT_ERROR:
        Logger.Info("MQTT event MQTT_EVENT_ERROR");
        break;
    case MQTT_EVENT_CONNECTED:
        Logger.Info("MQTT event MQTT_EVENT_CONNECTED");

        r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
        if (r == -1) {
            Logger.Error("Could not subscribe for cloud-to-device messages.");
        } else {
            Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
        }

        break;
    case MQTT_EVENT_DISCONNECTED:
        Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
        break;
    case MQTT_EVENT_DATA:
        Logger.Info("MQTT event MQTT_EVENT_DATA");

        for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->topic_len; i++) {
            incoming_data[i] = event->topic[i];
        }
        incoming_data[i] = '\0';
        Logger.Info("Topic: " + String(incoming_data));

        for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->data_len; i++) {
            incoming_data[i] = event->data[i];
        }
        incoming_data[i] = '\0';
        Logger.Info("Data: " + String(incoming_data));
        Serial.print("Delimiter is: ");
        Serial.println(delimiter);
        // Split the incoming data into three and map to the correct control info
        token = strtok(incoming_data, delimiter);

        if (strcasecmp(token, "soil moisture") == 0) {
            control_info[0] = VAL_SOIL_MOISTURE;
        } else if (strcasecmp(token, "temperature") == 0) {
            control_info[0] = VAL_TEMPERATURE;
        } else if (strcasecmp(token, "humidity") == 0) {
            control_info[0] = VAL_HUMIDITY;
        } else if (strcasecmp(token, "solar radiation") == 0) {
            control_info[0] = VAL_SOLAR_RADIATION;
        } else {
            control_info[0] = VAL_UNKNOWN;
            Serial.println("Invalid control value");
        }

        token = strtok(NULL, delimiter);
        control_info[1] = strtol(token, NULL, 10); // Lower bound
        token = strtok(NULL, delimiter);
        control_info[2] = strtol(token, NULL, 10); // Upper bound

        printf("Extracted numbers: ");
        for (size_t i = 0; i < 3; i++) {
            printf("%d ", control_info[i]);
        }
        printf("\n");
        freshData = true;
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
        break;
    default:
        Logger.Error("MQTT event UNKNOWN");
        break;
    }
    return ESP_OK;
}

static void initializeIoTHubClient()
{
    az_iot_hub_client_options options = az_iot_hub_client_options_default();
    options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

    if (az_result_failed(az_iot_hub_client_init(
            &client,
            az_span_create((uint8_t*)host, strlen(host)),
            az_span_create((uint8_t*)device_id, strlen(device_id)),
            &options))) {
        Logger.Error("Failed initializing Azure IoT Hub client");
        return;
    }

    size_t client_id_length;
    if (az_result_failed(az_iot_hub_client_get_client_id(
            &client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length))) {
        Logger.Error("Failed getting client id");
        return;
    }

    if (az_result_failed(az_iot_hub_client_get_user_name(
            &client, mqtt_username, sizeofarray(mqtt_username), NULL))) {
        Logger.Error("Failed to get MQTT clientId, return code");
        return;
    }

    Logger.Info("Client ID: " + String(mqtt_client_id));
    Logger.Info("Username: " + String(mqtt_username));
}

static int initializeMqttClient()
{
#ifndef IOT_CONFIG_USE_X509_CERT
    if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0) {
        Logger.Error("Failed generating SAS token");
        return 1;
    }
#endif

    esp_mqtt_client_config_t mqtt_config;
    memset(&mqtt_config, 0, sizeof(mqtt_config));
    mqtt_config.uri = mqtt_broker_uri;
    mqtt_config.port = mqtt_port;
    mqtt_config.client_id = mqtt_client_id;
    mqtt_config.username = mqtt_username;

#ifdef IOT_CONFIG_USE_X509_CERT
    Logger.Info("MQTT client using X509 Certificate authentication");
    mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
    mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
#else // Using SAS key
    mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
#endif

    mqtt_config.keepalive = 30;
    mqtt_config.disable_clean_session = 0;
    mqtt_config.disable_auto_reconnect = false;
    mqtt_config.event_handle = mqtt_event_handler;
    mqtt_config.user_context = NULL;
    mqtt_config.cert_pem = (const char*)ca_pem;

    mqtt_client = esp_mqtt_client_init(&mqtt_config);

    if (mqtt_client == NULL) {
        Logger.Error("Failed creating mqtt client");
        return 1;
    }

    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

    if (start_result != ESP_OK) {
        Logger.Error("Could not start mqtt client; error code:" + start_result);
        return 1;
    } else {
        Logger.Info("MQTT client started");
        return 0;
    }
}

/*
 * @brief           Gets the number of seconds since UNIX epoch until now.
 * @return uint32_t Number of seconds.
 */
// static uint32_t getEpochTimeInSecs() { return (uint32_t)time(NULL); }

static void establishConnection()
{
    connectToWiFi();
    initializeTime();
    initializeIoTHubClient();
    (void)initializeMqttClient();
}

static void generateTelemetryPayload()
{
    Logger.Info("Generating data for cloud upload");
    JsonDocument doc; // adjust size as needed
    doc.clear();

    // Create a nested JSON object with a custom name instead of "body"
    // JsonObject customNameObject = doc.createNestedObject("customName");  // Change "customName" to your desired name

    // Add data to the nested JSON object
    // doc["msgCount"] = telemetry_send_count++;
    for (int i = 0; i < NUMBER_OF_CONTROLLED_PARAMETERS; i++) {
        doc[map_control_enum_to_string(i)] = sensor_info[i];
    }
    // // Create a nested JSON object
    // JsonObject subtopicObject = customNameObject.createNestedObject("subtopic");
    // subtopicObject["testdata"] = 123;

    String jsonPayload;
    serializeJson(doc, jsonPayload);
    const char* payload = jsonPayload.c_str();
    // String jsonPayload;
    // serializeJson(sensor_info, jsonPayload);
    // const char* payload = jsonPayload.c_str();
    telemetry_payload = payload;
    // You can generate the JSON using any lib you want. Here we're showing how to do it manually, for simplicity.
    // This sample shows how to generate the payload using a syntax closer to regular delevelopment for Arduino, with
    // String type instead of az_span as it might be done in other samples. Using az_span has the advantage of reusing the
    // same char buffer instead of dynamically allocating memory each time, as it is done by using the String type below.
    // telemetry_payload = "{ \"msgCount\": " + String(telemetry_send_count++) + " }";
}

static void sendTelemetry()
{
    Logger.Info("Sending telemetry ...");

    // The topic could be obtained just once during setup,
    // however if properties are used the topic need to be generated again to reflect the
    // current values of the properties.
    if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(
            &client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL))) {
        Logger.Error("Failed az_iot_hub_client_telemetry_get_publish_topic");
        return;
    }

    generateTelemetryPayload();

    if (esp_mqtt_client_publish(
            mqtt_client,
            telemetry_topic,
            (const char*)telemetry_payload.c_str(),
            telemetry_payload.length(),
            MQTT_QOS1,
            DO_NOT_RETAIN_MSG)
        == 0) {
        Logger.Error("Failed publishing");
    } else {
        Logger.Info("Message published successfully");
    }
}

void setup()
{
    Serial.begin(57600);
    establishConnection();
    lora_setup();
}

void loop()
{
    unsigned long current_millis = millis();

    // static unsigned long last_sent_time = millis();
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
    }

    /**************************************************************************/
    /*** LORA - SENDING DESIRED PARAMETERS BOUDARIES TO THE MAIN CONTROLLER ***/
    /**************************************************************************/

    // Handle transmission every X seconds or as required
    // TODO change that -> we should send the message if some value should be changed
    // or we can send the message every X seconds to update the desired value for specified parameter
    if (freshData) {
        Serial.print("[communication node] Transmitting new parameters for " + map_control_enum_to_string(map_control_id_to_control_val(control_info[0])));
        Serial.println(" (" + String(control_info[1]) + " - " + String(control_info[2]) + ")");
        // last_sent_time = current_millis;
        lora_receive_mode_flag = false; // Put the lora module to receive mode again after sending the message.
        check_for_incoming_message(sensor_info, NUMBER_OF_CONTROLLED_PARAMETERS, &lora_receive_mode_flag, &lora_receive_message_flag); // Firstly, check if there is a message that waits to be processed.
        lora_transmit_mode_flag = true;
        lora_transmit(control_info, 3, &lora_transmit_mode_flag, &lora_receive_mode_flag, &lora_receive_message_flag); // Pass the receive message flag as we could enter this function with message waiting to be processed.
        freshData = false;
    }

    /************************************/
    /*** WIFI AND CLOUD COMMUNICATION ***/
    /************************************/

    // TODO important -> implement in the non-blocking manner, otherwise lora communication will be blocked

    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }

#ifndef IOT_CONFIG_USE_X509_CERT
    if (sasToken.IsExpired()) {
        Logger.Info("SAS token expired; reconnecting with a new one.");
        (void)esp_mqtt_client_destroy(mqtt_client);
        initializeMqttClient();
    }
#endif
    else if (lora_receive_message_flag) {
        sendTelemetry();
        next_telemetry_send_time_ms = millis() + TELEMETRY_FREQUENCY_MILLISECS;
        lora_receive_message_flag = false; // Reset flag after sending
    }

    /**********************************/
    /********** DEBUGGING PART ********/
    /**********************************/
    /*
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
  */
}
