/**************************************************************
* Raahi firmware header file
* Contains data types and defines specific to raahi fw, but 
* global to all files that make up the firmware 
**************************************************************/
#ifndef _RAAHI_FW_H_
#define _RAAHI_FW_H_

#define MAX_TOPIC_LEN 30
#define MAX_APN_LEN 20
#define MAX_MODBUS_SLAVES 2
#define MAX_CLIENT_ID_LEN 20
#define MAX_MODBUS_REGISTERS 3
#define MAX_ADC_CHANNELS 4
#define MAX_DEVICE_ID_LEN 20
#define ESP_RESTART_REASON_LEN 30
#define DATA_JSON_QUEUE_SIZE 10
#define DATA_JSON_STR_SIZE 250
#define EVENT_JSON_QUEUE_SIZE 10
#define EVENT_JSON_STR_SIZE 250
#define QUERY_JSON_QUEUE_SIZE 5
#define QUERY_JSON_STR_SIZE 300
#define MAX_INFO_JSON_ITEMS 15
#define INFO_JSON_LEN (uint16_t)(MAX_INFO_JSON_ITEMS * (MAX_KEY_LEN + MAX_VALUE_LEN))
#define MAX_AWS_FAILURE_COUNT (uint8_t)10
#define MAX_MODEM_FAILURE_COUNT (uint8_t)20
#define MAX_OTA_FAIL_COUNT (uint8_t) 20
#define MAX_MQTT_FAIL_TIME (time_t)180 // seconds
#define MAX_IDLING_TIME (time_t)600 // seconds
#define MODEM_MAX_OPERATOR_LENGTH (32) /*!< Max Operator Name Length */
#define MODEM_IMEI_LENGTH (15)         /*!< IMEI Number Length */
#define MAX_SUBSCRIBE_JSON_ITEMS 15
#define MAX_KEY_LEN 25
#define MAX_VALUE_LEN 25

#define OTA_RECORD_FILE_NAME (const char*)"/spiffs/otarecord.txt"

char raahi_log_str[EVENT_JSON_STR_SIZE];
#define RAAHI_LOGE( tag, format, ... ) do {\
	ESP_LOGE(tag, format, ##__VA_ARGS__);\
	sprintf(raahi_log_str, format, ##__VA_ARGS__);\
	strcat(raahi_log_str, " | ESP_LOGE");\
	compose_mqtt_event(tag, raahi_log_str);\
	}while(0)

#define RAAHI_LOGW( tag, format, ... ) do {\
	ESP_LOGW(tag, format, ##__VA_ARGS__);\
	sprintf(raahi_log_str, format, ##__VA_ARGS__);\
	strcat(raahi_log_str, " | ESP_LOGW");\
	compose_mqtt_event(tag, raahi_log_str);\
	}while(0)

#define RAAHI_LOGI(tag, format, ... ) do {\
	ESP_LOGI(tag, format, ##__VA_ARGS__);\
	sprintf(raahi_log_str, format, ##__VA_ARGS__);\
	strcat(raahi_log_str, " | ESP_LOGI");\
	compose_mqtt_event(tag, raahi_log_str);\
	}while(0)

#define RAAHI_LOGD( tag, format, ... ) do {\
	ESP_LOGD(tag, format, ##__VA_ARGS__);\
	sprintf(raahi_log_str, format, ##__VA_ARGS__);\
	strcat(raahi_log_str, " | ESP_LOGD");\
	compose_mqtt_event(tag, raahi_log_str);\
	}while(0)

#define RAAHI_LOGV( tag, format, ... ) do {\
	ESP_LOGV(tag, format, ##__VA_ARGS__);\
	sprintf(raahi_log_str, format, ##__VA_ARGS__);\
	strcat(raahi_log_str, " | ESP_LOGV");\
	compose_mqtt_event(tag, raahi_log_str);\
	}while(0)



// Function declarations
void compose_mqtt_event(const char *TAG, char *msg);

// Data type definitions
enum adc_port_type {NONE = 0, FOURTWENTY, RESISTIVE, DIRECT};

enum led_colour
{
	NOCOLOUR = 0,
	RED,
	BLUE,
	GREEN,
};

typedef struct{
	enum led_colour colour;
	bool blink;
	uint8_t blink_rate_in_ms;
} status_led_struct;
  
struct config_struct {
	uint8_t slave_id[MAX_MODBUS_SLAVES];
	uint16_t reg_address[MAX_MODBUS_REGISTERS];
	uint16_t sampling_period_in_sec;
	enum adc_port_type analog_sensor_type[MAX_ADC_CHANNELS];

	char client_id[MAX_CLIENT_ID_LEN + 1];	
	char topic[MAX_TOPIC_LEN + 1];
	char apn[MAX_APN_LEN + 1];
};

struct data_json_struct {
	char packet[DATA_JSON_QUEUE_SIZE][DATA_JSON_STR_SIZE];
	uint8_t read_ptr;
	uint8_t write_ptr;
};

struct event_json_struct {
	char packet[EVENT_JSON_QUEUE_SIZE][EVENT_JSON_STR_SIZE];
	uint8_t read_ptr;
	uint8_t write_ptr;
};

struct query_json_struct {
	char packet[QUERY_JSON_QUEUE_SIZE][QUERY_JSON_STR_SIZE];
	uint8_t read_ptr;
	uint8_t write_ptr;
};

enum slave_status{
	NOT_CONNECTED = 0,
	CONNECTED_WITH_ISSUES = 1,
	CONNECTED_AND_UPDATING = 2
};

struct slave_info_struct {
	enum slave_status status;
	uint16_t data[MAX_MODBUS_REGISTERS]; 
};

struct debug_data_struct {
	char fw_ver[5];
    char imei[MODEM_IMEI_LENGTH + 1]; 	
 	char oper[MODEM_MAX_OPERATOR_LENGTH];
	uint32_t rssi;
	uint32_t ber;
	uint32_t battery_voltage;
	struct slave_info_struct slave_info[MAX_MODBUS_SLAVES];
	bool connected_to_internet;
	bool connected_to_aws;
    char reset_reason_str[30];
};

typedef struct {
    char esp_restart_reason[ESP_RESTART_REASON_LEN];
} zombie_info_struct;

struct json_struct {
    char key[MAX_KEY_LEN];
    char value[MAX_VALUE_LEN];
};

#endif //#ifndef _RAAHI_FW_H_
