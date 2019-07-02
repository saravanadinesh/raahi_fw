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
#define MAX_MODBUS_REGISTERS 3
#define DATA_JSON_QUEUE_SIZE 10
#define DATA_JSON_STR_SIZE 100
#define EVENT_JSON_QUEUE_SIZE 10
#define EVENT_JSON_STR_SIZE 200

#define MODEM_MAX_OPERATOR_LENGTH (32) /*!< Max Operator Name Length */
#define MODEM_IMEI_LENGTH (15)         /*!< IMEI Number Length */

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
struct config_struct {
	uint8_t slave_id[MAX_MODBUS_SLAVES];
	uint16_t reg_address[MAX_MODBUS_REGISTERS];
	uint16_t sampling_period_in_sec;
	
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
    char imei[MODEM_IMEI_LENGTH + 1]; 	
 	char oper[MODEM_MAX_OPERATOR_LENGTH];
	uint32_t rssi;
	uint32_t ber;
	struct slave_info_struct slave_info[MAX_MODBUS_SLAVES];
};
#endif //#ifndef _RAAHI_FW_H_
