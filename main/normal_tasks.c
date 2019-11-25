/*
 * Copyright 2018-2019
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file normal_tasks.c
 * @brief Raahi normal task
 *
 * This example takes the parameters from the build configuration and establishes a connection to the AWS IoT MQTT Platform.
 * It subscribes and publishes to the same topic - /raahi/data and /raahi/event"
 *
 * Some setup is required. See example README for details.
 *
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "tcpip_adapter.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "esp_modem.h"
#include "sim800.h"
#include "soc/uart_struct.h"
#include <esp_http_server.h>
#include "esp_sntp.h"
#include "raahi.h"
 
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD
#define MODEM_SMS_MAX_LENGTH (128)
#define MODEM_COMMAND_TIMEOUT_SMS_MS (10000)
#define MODEM_PROMPT_TIMEOUT_MS (10)
#define ESP_CORE_0 0
#define ESP_CORE_1 1
#define IP_WAIT_TIME_MS 60000
#define SNTP_WAIT_TIME_IN_MS 30000
#define WDT_TIMEOUT_IN_SEC 120 // Must be greater than IP_WAIT_TIME_MS+SNTP_WAIT_TIME_MS
#define SIM800_RESET_GPIO GPIO_NUM_33

// Function declarations
httpd_handle_t start_webserver(void);
void set_status_LED(status_led_struct status_led);
void data_sampling_task(void*);
void ota_by_fragments(void *pvParameter);
void mobile_radio_init(void);
void display_sysconfig(void);
extern int32_t str2num(char* input_str, const char delimiter, uint8_t max_parse_len);
uint8_t parseJson(char* json_str, uint16_t json_str_len, struct json_struct* parsed_result);
void sysconfig_json_write(struct json_struct* parsed_json, uint8_t no_of_items);
void create_sysconfig_json(char* json_str, uint16_t json_str_len);
void display_sysconfig(void);
void read_sysconfig(void);
void getMacAddress(char* macAddress);

// Task Handles
TaskHandle_t dataSamplingTaskHandle;
TaskHandle_t awsTaskHandle;
TaskHandle_t otaTaskHandle;
bool otaTaskCreated = false;

// Global variables
static EventGroupHandle_t modem_event_group = NULL;
EventGroupHandle_t esp_event_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int STOP_BIT = BIT1;
const int SNTP_CONNECT_BIT = BIT0;

static const char *TAG = "normal_task";
modem_dte_t *dte_g;
modem_dce_t *dce_g;
static gpio_config_t sim800_reset_gpio;

char user_mqtt_str[MAX_DEVICE_ID_LEN] = {'\0'};

//Failure Counters
uint8_t aws_failures_counter = 0, other_aws_failures_counter = 0;
uint8_t modem_failures_counter = 0;
uint8_t fragmented_ota_error_counter = 0;
time_t last_publish_timestamp = 0;

zombie_info_struct zombie_info;

#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)

static const char * DEVICE_CERTIFICATE_PATH = CONFIG_EXAMPLE_CERTIFICATE_PATH;
static const char * DEVICE_PRIVATE_KEY_PATH = CONFIG_EXAMPLE_PRIVATE_KEY_PATH;
static const char * ROOT_CA_PATH = CONFIG_EXAMPLE_ROOT_CA_PATH;

#else
#error "Invalid method for loading certs"
#endif

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

struct config_struct sysconfig;
struct data_json_struct data_json;
struct event_json_struct event_json;
struct query_json_struct query_json;
struct debug_data_struct debug_data;
int today, this_hour;
/**
 * This is a example example which echos any data it receives on UART back to the sender.
 *
 * - port: UART2
 * - rx buffer: on
 * - tx buffer: off
 * - flow control: off
 *
 * This example has been tested on a 3 node RS485 Serial Bus
 * 
 */

uint32_t port = AWS_IOT_MQTT_PORT;
static void obtain_time(void);
static void initialize_sntp(void);
extern void raahi_restart(void);
void create_info_json(char* json_str, uint16_t json_str_len);
void write_zombie_info();
void read_zombie_info();

void time_sync_notification_cb(struct timeval *tv)
{
    RAAHI_LOGI(TAG, "Notification of a time synchronization event");
}


static void obtain_time(void)
{
    //ESP_ERROR_CHECK( nvs_flash_init() );
    //tcpip_adapter_init();
    //ESP_ERROR_CHECK( esp_event_loop_create_default() );

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
	if (retry == 10) {
		ESP_LOGI(TAG, "SNTP time could not be obtained");
		abort(); // If 
	}

    time(&now);
    localtime_r(&now, &timeinfo);
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "time.google.com");
    sntp_setservername(1, "in.pool.ntp.org");
    sntp_setservername(2, "sg.pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

static void setup_sntp(void)
{
  	time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    //if (timeinfo.tm_year < (2016 - 1900)) {
    //    ESP_LOGI(TAG, "Time is not set yet. Connecting to GPRS and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    //}
    char strftime_buf[64];
    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "IST-5:30", 1);
    tzset();
    localtime_r(&now, &timeinfo);
	today = timeinfo.tm_mday;
	this_hour = timeinfo.tm_hour;
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in India is: %s", strftime_buf);
    xEventGroupSetBits(esp_event_group, SNTP_CONNECT_BIT);
}


void compose_mqtt_event(const char *TAG, char *msg)
{
    char cPayload[EVENT_JSON_STR_SIZE];
    time_t now;

	time(&now);
	sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": \"%s\", \"%s\": \"%s\"}", \
			"deviceId", user_mqtt_str, \
        	"timestamp", now, \
			"tag", TAG, \
			"event_str", msg); 
	
	strcpy(event_json.packet[event_json.write_ptr], cPayload);
	event_json.write_ptr = (event_json.write_ptr+1) % EVENT_JSON_QUEUE_SIZE;
} 

static esp_err_t modem_default_handle(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    return err;
}

static esp_err_t modem_handle_cmgs(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    } else if (!strncmp(line, "+CMGS", strlen("+CMGS"))) {
        err = ESP_OK;
    }
    return err;
}

static esp_err_t modem_send_text_message(modem_dce_t *dce, const char *phone_num, char *text)
{
    modem_dte_t *dte = dce->dte;
    dce->handle_line = modem_default_handle;
    /* Set text mode */
    if (dte->send_cmd(dte, "AT+CMGF=1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
        RAAHI_LOGE(TAG, "send command failed");
        goto err;
    }   
    if (dce->state != MODEM_STATE_SUCCESS) {
        RAAHI_LOGE(TAG, "set message format failed");
        goto err;
    }   
    RAAHI_LOGD(TAG, "set message format ok");
    /* Specify character set */
    dce->handle_line = modem_default_handle;
    if (dte->send_cmd(dte, "AT+CSCS=\"GSM\"\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
        RAAHI_LOGE(TAG, "send command failed");
        goto err;
    }   
    if (dce->state != MODEM_STATE_SUCCESS) {
        RAAHI_LOGE(TAG, "set character set failed");
        goto err;
    }  
    RAAHI_LOGD(TAG, "set character set ok");
    /* send message */
    char command[MODEM_SMS_MAX_LENGTH] = {0};
    int length = snprintf(command, MODEM_SMS_MAX_LENGTH, "AT+CMGS=\"%s\"\r", phone_num);
    
    /* set phone number and wait for "> " */
    dte->send_wait(dte, command, length, "\r\n> ", MODEM_PROMPT_TIMEOUT_MS);
    /* end with CTRL+Z */
    snprintf(command, MODEM_SMS_MAX_LENGTH, "%s\x1A", text);
    dce->handle_line = modem_handle_cmgs;
    if (dte->send_cmd(dte, command, MODEM_COMMAND_TIMEOUT_SMS_MS) != ESP_OK) {
        RAAHI_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        RAAHI_LOGE(TAG, "send message failed");
        goto err;
    }
    RAAHI_LOGD(TAG, "send message ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}


static void modem_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MODEM_EVENT_PPP_START:
        RAAHI_LOGI(TAG, "Modem PPP Started");
        break;
    case MODEM_EVENT_PPP_CONNECT:
        RAAHI_LOGI(TAG, "Modem Connect to PPP Server");
        ppp_client_ip_info_t *ipinfo = (ppp_client_ip_info_t *)(event_data);
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&ipinfo->ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&ipinfo->netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&ipinfo->gw));
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&ipinfo->ns1));
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&ipinfo->ns2));
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(modem_event_group, CONNECT_BIT);
        break;
    case MODEM_EVENT_PPP_DISCONNECT:
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
		debug_data.connected_to_internet = false;
		// Restart modem
        vTaskDelay(5000/portTICK_RATE_MS);
        strcpy(zombie_info.esp_restart_reason, "Modem PPP Diconnect");
		esp_restart();
		break;
    case MODEM_EVENT_PPP_STOP:
        RAAHI_LOGI(TAG, "Modem PPP Stopped");
		debug_data.connected_to_internet = false;
        xEventGroupSetBits(modem_event_group, STOP_BIT);
        break;
    case MODEM_EVENT_UNKNOWN:
        ESP_LOGW(TAG, "Unknown line received"); //: %s", (char *)event_data);
        break;
    default:
        break;
    }
}

void execute_json_command(struct json_struct* parsed_json, uint8_t no_of_items)
{
    // Error Checks	
    if (no_of_items < 2)
    {
        RAAHI_LOGE(TAG, "Empty command json");
    }
    else // When no_of_items == 2, we can do something
    {
        if(strcmp(parsed_json[1].value, "restart") == 0)
        {
            RAAHI_LOGI(TAG, "Restart command received. Restarting in 5 seconds");
            strcpy(zombie_info.esp_restart_reason, "Command from AWS");
            vTaskDelay(5000/ portTICK_RATE_MS);
            raahi_restart();
        }
        else if(strcmp(parsed_json[1].value, "send_sysconfig") == 0)
        {
			char sysconfig_json[QUERY_JSON_STR_SIZE];

 			create_sysconfig_json(sysconfig_json, QUERY_JSON_STR_SIZE);
			strcpy(query_json.packet[query_json.write_ptr], sysconfig_json);
			query_json.write_ptr = (query_json.write_ptr+1) % QUERY_JSON_QUEUE_SIZE;
			
		}
        else if (strcmp(parsed_json[1].value, "update_fw") == 0)
        {
            if (otaTaskCreated == true) // This implies that there is a a firmware update already in progress. Kill it. 
            { 
                vTaskDelete(otaTaskHandle);
            }
            fragmented_ota_error_counter = 0; 
	        xTaskCreatePinnedToCore(&ota_by_fragments, "fragmented_ota_task", 8192, NULL, 10, &otaTaskHandle, ESP_CORE_0);	
        }
        else
        {
            RAAHI_LOGI(TAG, "Unknown command received. No action taken");
        }    
    }
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    struct json_struct parsed_json[MAX_SUBSCRIBE_JSON_ITEMS];
    uint8_t no_of_items, item_idx;
	
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
    no_of_items = parseJson(params->payload, params->payloadLen, parsed_json);
    if (no_of_items > 0) {
        if (strcmp(parsed_json[0].key, "type") == 0 && strcmp(parsed_json[0].value, "config") == 0)
        {
            sysconfig_json_write(parsed_json, no_of_items);
        }
        else if (strcmp(parsed_json[0].key, "type") == 0 && strcmp(parsed_json[0].value, "command") == 0) 
        {
            execute_json_command(parsed_json, no_of_items);
        }
        else
        {
            RAAHI_LOGE(TAG, "Message type isn't recognized or wasn't populated");
        }
            
        // Debug prints
        printf("Parsed Json items\n");
        for(item_idx = 0; item_idx < no_of_items; item_idx++)
        {
            printf("%s: %s\n", parsed_json[item_idx].key, parsed_json[item_idx].value);
        } // End of for loop
    } 
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

	debug_data.connected_to_aws = false;
    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
			debug_data.connected_to_aws = true;
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
			debug_data.connected_to_aws = false;
        }
    }
}


void aws_iot_task(void *param) {

	time_t now;
    
    char topic[MAX_TOPIC_LEN + 1] = {'\0'};
	char data_topic[MAX_TOPIC_LEN + 1] = {'\0'};
	char event_topic[MAX_TOPIC_LEN + 1] = {'\0'};
	char query_topic[MAX_TOPIC_LEN + 1] = {'\0'};
	char subscribe_topic[MAX_TOPIC_LEN + 1] = {'\0'};


	char dPayload[DATA_JSON_STR_SIZE] = {'\0'};
	char ePayload[EVENT_JSON_STR_SIZE] = {'\0'};
	char qPayload[QUERY_JSON_STR_SIZE] = {'\0'};

	strcat(topic, "/");
	strcat(topic, CONFIG_MQTT_TOPIC_ROOT); 

	strcpy(data_topic, topic);
	strcat(data_topic, "/data/");
	strcat(data_topic, user_mqtt_str);

	strcpy(event_topic, topic);
	strcat(event_topic, "/event/");
	strcat(event_topic, user_mqtt_str);
	
	strcpy(query_topic, topic);
	strcat(query_topic, "/query/");
	strcat(query_topic, user_mqtt_str);
	
	strcpy(subscribe_topic, topic);
	strcat(subscribe_topic, "/");
	strcat(subscribe_topic, user_mqtt_str);
	
	IoT_Publish_Message_Params dataPacket;
	IoT_Publish_Message_Params eventPacket;
	IoT_Publish_Message_Params queryPacket;

    IoT_Error_t rc, yield_rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

	if ((strlen(sysconfig.topic) + strlen(topic)) > (MAX_TOPIC_LEN - 2)) { // minus 2 for the two backward slashes
		ESP_LOGE(TAG, "MQTT topic is too long");
		return;
	}

	ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
    mqttInitParams.pRootCALocation = ROOT_CA_PATH;
    mqttInitParams.pDeviceCertLocation = DEVICE_CERTIFICATE_PATH;
    mqttInitParams.pDevicePrivateKeyLocation = DEVICE_PRIVATE_KEY_PATH;
#endif

    mqttInitParams.mqttCommandTimeout_ms = 30000;
    mqttInitParams.tlsHandshakeTimeout_ms = 30000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;


    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    connectParams.keepAliveIntervalInSec = 30;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = sysconfig.client_id;
    connectParams.clientIDLen = (uint16_t) strlen(sysconfig.client_id);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

	debug_data.connected_to_aws = true;
    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        RAAHI_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    RAAHI_LOGI(TAG, "Subscribing...");
    rc = aws_iot_mqtt_subscribe(&client, subscribe_topic, strlen(subscribe_topic), QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        RAAHI_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }
	RAAHI_LOGI(TAG, "Subscribed to %s", subscribe_topic);
	RAAHI_LOGI(TAG, "FW Version: %s", debug_data.fw_ver);
	RAAHI_LOGI(TAG, "RSSI: %u", debug_data.rssi);
	RAAHI_LOGI(TAG, "BER: %u", debug_data.ber);
	RAAHI_LOGI(TAG, "Battery Voltage: %u", debug_data.battery_voltage);

    //TODO: We have to send a hello message: sprintf(cPayload, "%s : %d ", "hello from SDK", i);
    dataPacket.qos = QOS0;
    dataPacket.payload = (void *) dPayload;
    dataPacket.isRetained = 0;

    eventPacket.qos = QOS0;
    eventPacket.payload = (void *) ePayload;
    eventPacket.isRetained = 0;

    queryPacket.qos = QOS0;
    queryPacket.payload = (void *) qPayload;
    queryPacket.isRetained = 0;
	
	rc = SUCCESS;
    /* Modem and HTTP server up */
	status_led_struct status_led;
	status_led.colour = GREEN;
	set_status_LED(status_led);
    while(1) { 
        //Reset watchdog timer for _this_ task 
        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);	
		
        //Max time the yield function will wait for read messages
        yield_rc = aws_iot_mqtt_yield(&client, 15000);
		if (SUCCESS != yield_rc) {
			ESP_LOGI(TAG, "MQTT yeild wasn't successful");
		}

			
		while ((data_json.write_ptr != data_json.read_ptr) && rc == SUCCESS){ // Implies there are unsent mqtt messages
		    strcpy(dPayload, data_json.packet[data_json.read_ptr]);  
		    dataPacket.payloadLen = strlen(dPayload);
        	    rc = aws_iot_mqtt_publish(&client, data_topic, strlen(data_topic), &dataPacket);
        	    if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            	        ESP_LOGW(TAG, "publish ack not received.");
            	        rc = SUCCESS;
        	    }
			
		    if (rc == SUCCESS) { 
		        data_json.read_ptr = (data_json.read_ptr+1) % DATA_JSON_QUEUE_SIZE;
				time(&last_publish_timestamp); // Update last publish timestamp
				ESP_LOGI(TAG, "Sent a data json");
		    }
		}

		while ((event_json.write_ptr != event_json.read_ptr) && rc == SUCCESS) { // Implies there are unsent mqtt messages
			strcpy(ePayload, event_json.packet[event_json.read_ptr]);     	
			eventPacket.payloadLen = strlen(ePayload);
        	    rc = aws_iot_mqtt_publish(&client, event_topic, strlen(event_topic), &eventPacket);
        	    if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            	        ESP_LOGW(TAG, "publish ack not received.");
            	        rc = SUCCESS;
        	    }
			
		    if (rc == SUCCESS) { 
				event_json.read_ptr = (event_json.read_ptr+1) % EVENT_JSON_QUEUE_SIZE;
				time(&last_publish_timestamp); // Update last publish timestamp
				ESP_LOGI(TAG, "Sent an event json");
        	}
		}

		while ((query_json.write_ptr != query_json.read_ptr) && rc == SUCCESS) { // Implies there are unsent mqtt messages
			strcpy(qPayload, query_json.packet[query_json.read_ptr]);     	
			queryPacket.payloadLen = strlen(qPayload);
        	    rc = aws_iot_mqtt_publish(&client, query_topic, strlen(query_topic), &queryPacket);
        	    if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            	        ESP_LOGW(TAG, "publish ack not received.");
            	        rc = SUCCESS;
        	    }
			
		    if (rc == SUCCESS) { 
				query_json.read_ptr = (query_json.read_ptr+1) % QUERY_JSON_QUEUE_SIZE;
				time(&last_publish_timestamp); // Update last publish timestamp
				ESP_LOGI(TAG, "Sent an query json");
        	}
		}

        time(&now);
        if((now - last_publish_timestamp) > MAX_IDLING_TIME)
        { // If there hasn't bee anything to send for a long time, data sampling task may be in a hung state
			RAAHI_LOGE(TAG, "MQTT hasn't sent a message in a long time.");
            strcpy(zombie_info.esp_restart_reason, "MQTT Long Idle (AWS Task)");
            esp_restart();
        }
 
		switch(rc)
		{
			case SUCCESS:
        	case NETWORK_RECONNECTED:		
			case NETWORK_PHYSICAL_LAYER_CONNECTED:
				aws_failures_counter = 0;
				other_aws_failures_counter = 0;	
				modem_failures_counter = 0;
				debug_data.connected_to_aws = true;
				debug_data.connected_to_internet = true;
        		//ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
				break; // End of case SUCCESS: ...
	
			case NETWORK_ATTEMPTING_RECONNECT:
				debug_data.connected_to_aws = false;
				debug_data.connected_to_internet = false;
				vTaskDelay(1000/portTICK_RATE_MS); // Wait a sec and go back to loop
				// TODO: There is no way to simply check the status of aws_iot. If it is available in the future, add it here
				rc = SUCCESS; // Due the reason mentioned in the above line, this is necessary. Otherwise we will keep hitting this case statement	
				break;

			case FAILURE:
				aws_failures_counter++;
				vTaskDelay(1000/portTICK_RATE_MS);
				rc = SUCCESS;
				break;

			case TCP_SETUP_ERROR:
			case TCP_CONNECTION_ERROR:
			case NULL_VALUE_ERROR:
				debug_data.connected_to_aws = false;
				debug_data.connected_to_internet = false;
    			ESP_LOGE(TAG, "Unrecoverable error in AWS loop. rc = %d", rc);
    			abort();

			default:
				other_aws_failures_counter++;
				RAAHI_LOGE(TAG, "General failure occurred. rc = %d", rc);
				vTaskDelay(1000/portTICK_RATE_MS);
				rc = SUCCESS;
				break;

		} // End of switch statement 

    } // End of infinite while loop

}


void sim800_hardreset()
{
    ESP_LOGI(TAG, "Hard resetting Sim800");
    gpio_set_level(SIM800_RESET_GPIO, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS); // pull down time must be >150ms
    gpio_set_level(SIM800_RESET_GPIO, 1);
}

void mobile_radio_init()
{
	dte_g = NULL;
	dce_g = NULL;
 
	/* create dte object */
    esp_modem_dte_config_t config = ESP_MODEM_DTE_DEFAULT_CONFIG();

	if((dte_g = esp_modem_dte_init(&config)) == NULL)
	{
		ESP_LOGE(TAG, "DTE initialization did not work\n");
        strcpy(zombie_info.esp_restart_reason, "DTE Init Failed");
		esp_restart();
	}

    /* Register event handler */
    ESP_ERROR_CHECK(esp_modem_add_event_handler(dte_g, modem_event_handler, NULL));
    /* create dce object */

    // Set up the SIm800 GPIO
    sim800_reset_gpio.intr_type = GPIO_PIN_INTR_DISABLE;
    sim800_reset_gpio.mode = GPIO_MODE_OUTPUT;
    sim800_reset_gpio.pin_bit_mask = (1ULL << SIM800_RESET_GPIO);
    sim800_reset_gpio.pull_up_en = 0;   // Keep it floating until it is used
    sim800_reset_gpio.pull_down_en = 0;
    gpio_config(&sim800_reset_gpio);

    /* Wait for 2 secs for Modem info to populate*/
     
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    uint8_t retries = 0;
	for (retries = 0; retries < 5; retries++) 
    {
        if((dce_g = sim800_init(dte_g)) != NULL) 
	    {
            break;
        }
    }
    
    if (retries == 5)
    {
        ESP_LOGE(TAG, "DCE initialization did not work\n");
        strcpy(zombie_info.esp_restart_reason, "DCE Init Failed");
        sim800_hardreset();
        vTaskDelay(30000 / portTICK_PERIOD_MS);
		esp_restart();
	}

    //dte_g->change_mode(dte_g, MODEM_COMMAND_MODE);
    ESP_ERROR_CHECK(dce_g->set_flow_ctrl(dce_g, MODEM_FLOW_CONTROL_NONE));
    ESP_ERROR_CHECK(dce_g->store_profile(dce_g));
    /* Print Module ID, Operator, IMEI, IMSI */
    ESP_LOGI(TAG, "Module: %s", dce_g->name);
    ESP_LOGI(TAG, "Operator: %s", dce_g->oper);
    ESP_LOGI(TAG, "IMEI: %s", dce_g->imei);
    ESP_LOGI(TAG, "IMSI: %s", dce_g->imsi);
    /* Get signal quality */
    uint32_t rssi = 0, ber = 0;
    ESP_ERROR_CHECK(dce_g->get_signal_quality(dce_g, &rssi, &ber));
    ESP_LOGI(TAG, "rssi: %u, ber: %u", rssi, ber);
	
	strcpy(debug_data.imei, dce_g->imei);
	strcpy(debug_data.oper, dce_g->oper);
	debug_data.rssi = rssi;
	debug_data.ber = ber;

    /* Get battery voltage */
    uint32_t voltage = 0, bcs = 0, bcl = 0;
    ESP_ERROR_CHECK(dce_g->get_battery_status(dce_g, &bcs, &bcl, &voltage));
    ESP_LOGI(TAG, "Battery voltage: %d mV", voltage);
	debug_data.battery_voltage = voltage;	

    // Send an SMS at the beginning 
    //char info_json[INFO_JSON_LEN];
    //create_info_json(info_json, INFO_JSON_LEN);
    //modem_send_text_message(dce_g, CONFIG_MONITOR_PHONE_NUMBER, info_json);
    //ESP_LOGI(TAG, "Send message [%s] ok", info_json);
    //vTaskDelay(10000 / portTICK_PERIOD_MS);

    /* Setup PPP environment */
    if(esp_modem_setup_ppp(dte_g) == ESP_FAIL) {
		ESP_LOGE(TAG, "Modem PPP setup failed");
		abort();
	}

    /* Modem and HTTP server up */
	status_led_struct status_led;
	status_led.colour = BLUE;
	set_status_LED(status_led);
	
    /* Wait for IP address */
    EventBits_t ipWaitBits;
    ipWaitBits = xEventGroupWaitBits(modem_event_group, CONNECT_BIT, pdTRUE, pdTRUE, IP_WAIT_TIME_MS/portTICK_PERIOD_MS);
    if(!(ipWaitBits & CONNECT_BIT)) // If it timed out and we didn't get an IP address
    {
        strcpy(zombie_info.esp_restart_reason, "IP not obtained");
		esp_restart();
    }

	debug_data.connected_to_internet = true;
	/* Start NTP sync */
    setup_sntp();
}

void normal_tasks()
{
	static const char* config_file_name = "/spiffs/sysconfig.txt";
	FILE* config_file = NULL;
	FILE* ota_record_file = NULL;
 
    // Init (or Reinit) watchdog timer
    CHECK_ERROR_CODE(esp_task_wdt_init(WDT_TIMEOUT_IN_SEC, true), ESP_OK); // Here 'true' is to invoke panic handler if WDT expires
    
    //Subscribe this task to TWDT, then check if it is subscribed
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK); // NULL implies _this_ task
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);

    read_zombie_info();

    esp_register_shutdown_handler((shutdown_handler_t)write_zombie_info);

    // Initialize all error counters to zero
    aws_failures_counter = 0;
    other_aws_failures_counter = 0;
    modem_failures_counter = 0;
    fragmented_ota_error_counter = 0;
    last_publish_timestamp = 0;

	getMacAddress(user_mqtt_str); // Mac address is used as a unique id of the device in json packets.

	read_sysconfig();

	ESP_LOGI(TAG, "Waiting 10 sec for the modem to warm up");
	vTaskDelay(10000 / portTICK_PERIOD_MS);
    
    modem_event_group = xEventGroupCreate();
    esp_event_group = xEventGroupCreate();

	debug_data.connected_to_internet = false;
	debug_data.connected_to_aws = false;
	
    // Get the homepage up: Initialize webserver, register all handlers
    start_webserver();
  
	data_json.read_ptr = 0;
	data_json.write_ptr = 0;
	event_json.read_ptr = 0;
	event_json.write_ptr = 0;
	query_json.read_ptr = 0;
	query_json.write_ptr = 0;

	xTaskCreatePinnedToCore(&data_sampling_task, "data_sampling_task", 8192, NULL, 9, &dataSamplingTaskHandle, ESP_CORE_1);	
    CHECK_ERROR_CODE(esp_task_wdt_add(dataSamplingTaskHandle), ESP_OK); 
    CHECK_ERROR_CODE(esp_task_wdt_status(dataSamplingTaskHandle), ESP_OK);
	
	mobile_radio_init();

	if(strcmp(sysconfig.client_id, dce_g->imei) != 0) // This happens just once after erase_flash
	{
		strcpy(sysconfig.client_id, dce_g->imei);
		
        display_sysconfig(); // so that we will know in AWS if the changes have indeed taken place
	    config_file = fopen(config_file_name, "wb");

	    if(fwrite(&sysconfig, sizeof(struct config_struct), 1, config_file) != 1) {
	    	RAAHI_LOGE(TAG, "Couldn't update sysconfig file although it is present");
	    	abort();
	    } else {
	    	RAAHI_LOGI(TAG, "Successfully updated sysconfig file");
	    }
	    fclose(config_file);
	}
    
    EventBits_t sntpWaitBits;
    sntpWaitBits = xEventGroupWaitBits(esp_event_group, SNTP_CONNECT_BIT, pdFALSE, pdTRUE, SNTP_WAIT_TIME_IN_MS/portTICK_PERIOD_MS);
    if(!(sntpWaitBits & CONNECT_BIT)) // If it timed out and we didn't get an IP address
    {
        strcpy(zombie_info.esp_restart_reason, "Time not obtained");
		esp_restart();
    }

    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, NULL, 6, &awsTaskHandle, ESP_CORE_0);
    CHECK_ERROR_CODE(esp_task_wdt_add(awsTaskHandle), ESP_OK); 
    CHECK_ERROR_CODE(esp_task_wdt_status(awsTaskHandle), ESP_OK);
    
    // If OTA was in progress before the reset/power cycle, restart OTA
    ota_record_file = fopen(OTA_RECORD_FILE_NAME, "rb");
    if (ota_record_file != NULL) { // There was an OTA in progress before reboot.  
	    xTaskCreatePinnedToCore(&ota_by_fragments, "fragmented_ota_task", 8192, NULL, 8, &otaTaskHandle, ESP_CORE_0);	
        CHECK_ERROR_CODE(esp_task_wdt_add(otaTaskHandle), ESP_OK); 
        CHECK_ERROR_CODE(esp_task_wdt_status(otaTaskHandle), ESP_OK);
    }

    // We don't need to feed the watchdog for the normal task because we don't have any infinite loops. If we delete this task
    // from TWDT watchlist before the it fires, we are good
    CHECK_ERROR_CODE(esp_task_wdt_delete(NULL), ESP_OK);     //Unsubscribe _this_ task from TWDT
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_ERR_NOT_FOUND);  //Confirm task is unsubscribed

}
