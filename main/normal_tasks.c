/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file subscribe_publish_sample.c
 * @brief simple MQTT publish and subscribe on the same topic
 *
 * This example takes the parameters from the build configuration and establishes a connection to the AWS IoT MQTT Platform.
 * It subscribes and publishes to the same topic - "test_topic/esp32"
 *
 * Some setup is required. See example README for details.
 *
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#include "raahi.h"
 
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD
#define MODEM_SMS_MAX_LENGTH (128)
#define MODEM_COMMAND_TIMEOUT_SMS_MS (120000)
#define MODEM_PROMPT_TIMEOUT_MS (10)
#define MAX_TOPIC_LEN 30 

char raahi_log_str[EVENT_JSON_STR_SIZE];

// Function declarations
httpd_handle_t start_webserver(void);
void data_sampling_task(void*);

static EventGroupHandle_t modem_event_group = NULL;
//EventGroupHandle_t mqtt_rw_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int STOP_BIT = BIT1;
static const int GOT_DATA_BIT = BIT2;
//const int READ_OP_DONE = BIT0;
//const int WRITE_OP_DONE = BIT1;

static const char *TAG = "normal_task";


char *user_mqtt_str;


/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */



/* CA Root certificate, device ("Thing") certificate and device
 * ("Thing") key.

   Example can be configured one of two ways:

   "Embedded Certs" are loaded from files in "certs/" and embedded into the app binary.

   "Filesystem Certs" are loaded from the filesystem (SD card, etc.)

   See example README for more details.
*/
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
struct debug_data_struct debug_data;
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

void compose_mqtt_event(const char *TAG, char *msg)
{
    char cPayload[EVENT_JSON_STR_SIZE];
	sprintf(cPayload, "{\"%s\": \"%s\", \"%s\" : \"%s\"}", \
			"tag", TAG, \
			"event_str", msg); 
	
	strcpy(event_json.packet[event_json.write_ptr], TAG);
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


static esp_err_t modem_send_message_text(modem_dce_t *dce, const char *phone_num, const char *text)
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
        RAAHI_LOGI(TAG, "~~~~~~~~~~~~~~");
        RAAHI_LOGI(TAG, "IP          : " IPSTR, IP2STR(&ipinfo->ip));
        RAAHI_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&ipinfo->netmask));
        RAAHI_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&ipinfo->gw));
        RAAHI_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&ipinfo->ns1));
        RAAHI_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&ipinfo->ns2));
        RAAHI_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(modem_event_group, CONNECT_BIT);
        break;
    case MODEM_EVENT_PPP_DISCONNECT:
        RAAHI_LOGI(TAG, "Modem Disconnect from PPP Server");
        break;
    case MODEM_EVENT_PPP_STOP:
        RAAHI_LOGI(TAG, "Modem PPP Stopped");
        xEventGroupSetBits(modem_event_group, STOP_BIT);
        break;
    case MODEM_EVENT_UNKNOWN:
        RAAHI_LOGW(TAG, "Unknow line received: %s", (char *)event_data);
        break;
    default:
        break;
    }
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    RAAHI_LOGI(TAG, "Subscribe callback");
    RAAHI_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    RAAHI_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        RAAHI_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        RAAHI_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            RAAHI_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            RAAHI_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}


void aws_iot_task(void *param) {

	char topic[MAX_TOPIC_LEN + 1] = {'\0'};
	char data_topic[MAX_TOPIC_LEN + 1] = {'\0'};
	char event_topic[MAX_TOPIC_LEN + 1] = {'\0'};


	char dPayload[DATA_JSON_STR_SIZE] = {'\0'};
	char ePayload[EVENT_JSON_STR_SIZE] = {'\0'};
 

	strcat(topic, "/");
	strcat(topic, CONFIG_MQTT_TOPIC_ROOT); 

	strcpy(data_topic, topic);
	strcat(data_topic, "/data");

	strcpy(event_topic, topic);
	strcat(event_topic, "/event");
	
	IoT_Publish_Message_Params dataPacket;
	IoT_Publish_Message_Params eventPacket;

    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

	if ((strlen(sysconfig.topic) + strlen(topic)) > (MAX_TOPIC_LEN - 2)) { // minus 2 for the two backward slashes
		RAAHI_LOGE(TAG, "MQTT topic is too long");
		return;
	}

	RAAHI_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

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

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;


    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        RAAHI_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
//    xEventGroupWaitBits(wifi_modem_event_group, CONNECTED_BIT,
//                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    RAAHI_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            RAAHI_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

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
    rc = aws_iot_mqtt_subscribe(&client, topic, strlen(topic), QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        RAAHI_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }
	RAAHI_LOGI(TAG, "Subscribed to %s", topic);

    //TODO: We have to send a hello message: sprintf(cPayload, "%s : %d ", "hello from SDK", i);
    dataPacket.qos = QOS0;
    dataPacket.payload = (void *) dPayload;
    dataPacket.isRetained = 0;

    eventPacket.qos = QOS0;
    eventPacket.payload = (void *) ePayload;
    eventPacket.isRetained = 0;
    while(rc >=0) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);

        RAAHI_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
				
		if (data_json.write_ptr != data_json.read_ptr) { // Implies there are unsent mqtt messages
			//xEventGroupWaitBits(mqtt_rw_group, WRITE_OP_DONE, pdFALSE, pdTRUE, portMAX_DELAY); // Wait until aws task reads from queue
			//xEventGroupClearBits(mqtt_rw_group, READ_OP_DONE);
   			
			strcpy(dPayload, data_json.packet[data_json.read_ptr]);     	
			dataPacket.payloadLen = strlen(dPayload);
        	rc = aws_iot_mqtt_publish(&client, data_topic, strlen(data_topic), &dataPacket);
        	if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            	RAAHI_LOGW(TAG, "publish ack not received.");
            	rc = SUCCESS;
        	}
			
			if (rc == SUCCESS) { 
				data_json.read_ptr = (data_json.read_ptr+1) % DATA_JSON_QUEUE_SIZE;
			}

			//xEventGroupSetBits(mqtt_rw_group, READ_OP_DONE);
		}
		if (event_json.write_ptr != event_json.read_ptr) { // Implies there are unsent mqtt messages
			strcpy(ePayload, event_json.packet[event_json.read_ptr]);     	
			eventPacket.payloadLen = strlen(ePayload);
        	rc = aws_iot_mqtt_publish(&client, event_topic, strlen(event_topic), &eventPacket);
        	if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            	RAAHI_LOGW(TAG, "publish ack not received.");
            	rc = SUCCESS;
        	}
			
			if (rc == SUCCESS) { 
				event_json.read_ptr = (event_json.read_ptr+1) % EVENT_JSON_QUEUE_SIZE;
			}

		}
	
        vTaskDelay((sysconfig.sampling_period_in_sec * 1000) / portTICK_RATE_MS);
	
    }

    RAAHI_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}



/* -----------------------------------------------------------
| 	display_sysconfig()
| 	Prints sysconfig values. Used for debugging only	
------------------------------------------------------------*/
void display_sysconfig(void)
{
	uint8_t slave_id_idx, reg_address_idx;

	RAAHI_LOGI(TAG, "**************** Syconfig Data ******************");

	// Display slave IDs	
	for (slave_id_idx = 0; slave_id_idx < MAX_MODBUS_SLAVES; slave_id_idx++)
	{	
		if (sysconfig.slave_id[slave_id_idx] == 0) {// Slave ID of 0 is considered to be an uninitialized entry
			break;
		}
		RAAHI_LOGI(TAG, "Slave ID of Slave %d = %d", slave_id_idx + 1, sysconfig.slave_id[slave_id_idx]);
	}
	
	// Display register addresses
	for (reg_address_idx = 0; reg_address_idx < MAX_MODBUS_REGISTERS; reg_address_idx++)
	{
		if (sysconfig.reg_address[reg_address_idx] == 0) {// reg address 0 is considered to be unintialized entry
			break;
		}
		RAAHI_LOGI(TAG, "Reg Address %d is 0x%.4X", reg_address_idx + 1, sysconfig.reg_address[reg_address_idx]);
	}

	// Display the rest of the information
	RAAHI_LOGI(TAG, "Sampling period (sec): %d", sysconfig.sampling_period_in_sec);
	
	RAAHI_LOGI(TAG, "Topic: %s", sysconfig.topic);
	RAAHI_LOGI(TAG, "Apn: %s", sysconfig.apn);
	
	RAAHI_LOGI(TAG, "*************************************************");
}

/* -----------------------------------------------------------
| 	read_sysconfig()
| 	We store all the system configuration options that can be
| 	changed at run-time to a file in spiffs so that it can 
|	persist between boots 
------------------------------------------------------------*/
void read_sysconfig()
{
	static const char* config_file_name = "/spiffs/sysconfig.txt";
	FILE* config_file = NULL;
	struct config_struct default_config;
	size_t content_size;

	// Populate the default config
	default_config.slave_id[0] = CONFIG_FIRST_SLAVE_ID; // From sdkconfig	
	default_config.slave_id[1] = CONFIG_SECOND_SLAVE_ID;
	default_config.reg_address[0] = CONFIG_FIRST_REG;
	default_config.reg_address[1] = CONFIG_SECOND_REG;
	default_config.reg_address[2] = CONFIG_THIRD_REG;
	default_config.sampling_period_in_sec = CONFIG_SAMPLING_PERIOD;
	strcpy(default_config.topic, CONFIG_MQTT_TOPIC_ROOT);
	strcpy(default_config.apn, CONFIG_ESP_MODEM_APN);

	config_file = fopen(config_file_name, "rb");
	if (config_file == NULL) { // If config file isnt' present, create one with default values
		RAAHI_LOGI(TAG, "Config file not present. Creating one with default values");
		config_file = fopen(config_file_name, "wb");
		
		if (fwrite(&default_config, sizeof(struct config_struct), 1, config_file) != 1) {
			RAAHI_LOGE(TAG, "Couldn't write to Spiffs file sysconfig.txt");
			fclose(config_file);
			abort();
		} else {
			RAAHI_LOGI(TAG, "Config file successfully populated with default values");
			sysconfig = default_config; // Also populate the global variable that will be used by many functions
			fclose(config_file);
		}

	} else { // sysconfig.txt file is present. So populate global variable with data from it
		if((content_size = fread(&sysconfig, sizeof(struct config_struct), 1, config_file)) != 1) {
			RAAHI_LOGE(TAG, "Couldn't read sysconfig.txt contents");
			abort();
		} else {
			RAAHI_LOGI(TAG, "Successfully read sysconfig.txt contents into internal variable");
			// Display what was read
			display_sysconfig();
		}
	}
}


void normal_tasks()
{
    static httpd_handle_t http_server = NULL;
	uint8_t retries; 

	read_sysconfig();
	
	// Get the homepage up: Initialize webserver, register all handlers
    http_server = start_webserver();
  
	data_json.read_ptr = 0;
	data_json.write_ptr = 0;
	event_json.read_ptr = 0;
	event_json.write_ptr = 0;

	//mqtt_rw_group = xEventGroupCreate();
	//xEventGroupSetBits(mqtt_rw_group, READ_OP_DONE);
	//xEventGroupSetBits(mqtt_rw_group, WRITE_OP_DONE);

	xTaskCreate(data_sampling_task, "data_sampling_task", 4096, NULL, 10, NULL);	
	
 
    modem_event_group = xEventGroupCreate();
    /* create dte object */
    esp_modem_dte_config_t config = ESP_MODEM_DTE_DEFAULT_CONFIG();

    modem_dte_t *dte;
	retries = 0;
	while((dte = esp_modem_dte_init(&config)) == NULL)
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	    retries++;
		if (retries > 30) {
			RAAHI_LOGE(TAG, "DTE initialization did not work\n");
			abort();
		}	
	}

    /* Register event handler */
    ESP_ERROR_CHECK(esp_modem_add_event_handler(dte, modem_event_handler, NULL));
    /* create dce object */

	modem_dce_t *dce;
	retries = 0;
	while ((dce = sim800_init(dte)) == NULL) 
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	    retries++;
		if (retries > 30) {
			RAAHI_LOGE(TAG, "DCE initialization did not work\n");
			abort();
		}	
	}

    ESP_ERROR_CHECK(dce->set_flow_ctrl(dce, MODEM_FLOW_CONTROL_NONE));
    ESP_ERROR_CHECK(dce->store_profile(dce));
    /* Print Module ID, Operator, IMEI, IMSI */
    RAAHI_LOGI(TAG, "Module: %s", dce->name);
    RAAHI_LOGI(TAG, "Operator: %s", dce->oper);
    RAAHI_LOGI(TAG, "IMEI: %s", dce->imei);
    RAAHI_LOGI(TAG, "IMSI: %s", dce->imsi);
    /* Get signal quality */
    uint32_t rssi = 0, ber = 0;
    dce->get_signal_quality(dce, &rssi, &ber);
    RAAHI_LOGI(TAG, "rssi: %u, ber: %u", rssi, ber);
	
	strcpy(debug_data.imei, dce->imei);
	strcpy(debug_data.oper, dce->oper);
	debug_data.rssi = rssi;
	debug_data.ber = ber;

    /* Get battery voltage */
    uint32_t voltage = 0, bcs = 0, bcl = 0;
    dce->get_battery_status(dce, &bcs, &bcl, &voltage);
    RAAHI_LOGI(TAG, "Battery voltage: %d mV", voltage);
    /* Setup PPP environment */
    if(esp_modem_setup_ppp(dte) == ESP_FAIL) {
		RAAHI_LOGE(TAG, "Modem PPP setup failed");
		abort();
	}
	
    /* Wait for IP address */
    xEventGroupWaitBits(modem_event_group, CONNECT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
//    xEventGroupWaitBits(modem_event_group, GOT_DATA_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    /* Exit PPP mode */
//    ESP_ERROR_CHECK(esp_modem_exit_ppp(dte));
//    xEventGroupWaitBits(modem_event_group, STOP_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
#if CONFIG_SEND_MSG
    const char *message = "Welcome to ESP32!";
    ESP_ERROR_CHECK(modem_send_message_text(dce, CONFIG_SEND_MSG_PEER_PHONE_NUMBER, message));
    RAAHI_LOGI(TAG, "Send send message [%s] ok", message);
#endif
    /* Power down module */
//    ESP_ERROR_CHECK(dce->power_down(dce));
//    RAAHI_LOGI(TAG, "Power down");
//    ESP_ERROR_CHECK(dce->deinit(dce));
//    ESP_ERROR_CHECK(dte->deinit(dte));
	user_mqtt_str = dce->imei;
    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, NULL, 5, NULL, 1);
}
