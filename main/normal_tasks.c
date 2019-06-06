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
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "tcpip_adapter.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
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
#define FORM_DATA_BUF_SIZE 256
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD
#define MODEM_SMS_MAX_LENGTH (128)
#define MODEM_COMMAND_TIMEOUT_SMS_MS (120000)
#define MODEM_PROMPT_TIMEOUT_MS (10)


static const int CONNECT_BIT = BIT0;
static const int CONNECTED_BIT = BIT0;
static const int STOP_BIT = BIT1;
static const int GOT_DATA_BIT = BIT2;
static const char *TAG = "subpub";
static EventGroupHandle_t event_group = NULL;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

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

// Note: UART2 default pins IO16, IO17 do not work on ESP32-WROVER module 
// because these pins connected to PSRAM
#define ECHO_TEST_TXD   (23)
#define ECHO_TEST_RXD   (22)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (21)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS  UART_PIN_NO_CHANGE

#define BUF_SIZE        (127)
#define BAUD_RATE       (9600)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (UART_NUM_2)
/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;
const int uart_num = ECHO_UART_PORT;
// Allocate buffers for UART
uint16_t modbus_crc;
const char *TOPIC = "test_topic/esp32";
// Modbus CRC stuff
static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

uint16_t usMBCRC16( uint8_t* pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

// An example of echo test with hardware flow control on UART
uint16_t echo_task()
{
	    // Compose modbus command for sapcon slave
        uint8_t* data_out = (uint8_t*) malloc(BUF_SIZE);
        uint8_t* data_in = (uint8_t*) malloc(BUF_SIZE);
        data_out[0] = 0x01;
        data_out[1] = 0x03;
        data_out[2] = 0x00;
        data_out[3] = 0xd5;
        data_out[4] = 0x00;
        data_out[5] = 0x02;
        data_out[6] = 0xd5;
        data_out[7] = 0xf3;
        modbus_crc = usMBCRC16(data_out, 6);
		printf("modbus_crc = 0x%.04X", modbus_crc);
		printf("\n");

		// Write modbus master command on rs485
                uart_write_bytes(uart_num, (const char*)&data_out[0], 8);
		for (int count = 0; count <5; count++)
		{
			//Read data from UART
        	int len = uart_read_bytes(uart_num, data_in, BUF_SIZE, PACKET_READ_TICS);
        
        	//Write data back to UART
        	if ((len > 0) && (data_in[0] == data_out[0])) {
            	ESP_LOGI(TAG, "Received %u bytes:", len);
            	printf("[ ");
            	for (int i = 0; i < len; i++) {
                	printf("0x%.2X ", (uint8_t)data_in[i]);
            	}
           		printf("] \n");
				break;
        	} else {
				printf("Waiting for slave to respond \n");
			}

			vTaskDelay(1000 / portTICK_PERIOD_MS);
		} // end of for count..
               return (data_in[3] <<8| data_in[4]); 
/*
	        // Compose modbus command for sapcon slave
               data_out[0] = 0x05;
               data_out[1] = 0x03;
               data_out[2] = 0x00;
               data_out[3] = 0xff;
               data_out[4] = 0x00;
               data_out[5] = 0x02;
               data_out[6] = 0xbf;
               data_out[7] = 0xf5;
	
		// Write modbus master command on rs485
                uart_write_bytes(uart_num, (const char*)&data_out[0], 8);

		for (int count = 0; count <5; count++)
		{
			//Read data from UART
        	int len = uart_read_bytes(uart_num, data_in, BUF_SIZE, PACKET_READ_TICS);
        
        	//Write data back to UART
        	if ((len > 0) && (data_in[0] == data_out[0])) {
            	ESP_LOGI(TAG, "Received %u bytes:", len);
            	printf("[ ");
            	for (int i = 0; i < len; i++) {
                	printf("0x%.2X ", (uint8_t)data_in[i]);
            	}
           		printf("] \n");
				break;
        	} else {
				printf("Waiting for slave to respond \n");
			}

			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
*/

}
static esp_err_t example_default_handle(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    return err;
}

static esp_err_t example_handle_cmgs(modem_dce_t *dce, const char *line)
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

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static esp_err_t example_send_message_text(modem_dce_t *dce, const char *phone_num, const char *text)
{
    modem_dte_t *dte = dce->dte;
    dce->handle_line = example_default_handle;
    /* Set text mode */
    if (dte->send_cmd(dte, "AT+CMGF=1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        ESP_LOGE(TAG, "set message format failed");
        goto err;
    }
    ESP_LOGD(TAG, "set message format ok");
    /* Specify character set */
    dce->handle_line = example_default_handle;
    if (dte->send_cmd(dte, "AT+CSCS=\"GSM\"\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        ESP_LOGE(TAG, "set character set failed");
        goto err;
    }
    ESP_LOGD(TAG, "set character set ok");
    /* send message */
    char command[MODEM_SMS_MAX_LENGTH] = {0};
    int length = snprintf(command, MODEM_SMS_MAX_LENGTH, "AT+CMGS=\"%s\"\r", phone_num);
    /* set phone number and wait for "> " */
    dte->send_wait(dte, command, length, "\r\n> ", MODEM_PROMPT_TIMEOUT_MS);
    /* end with CTRL+Z */
    snprintf(command, MODEM_SMS_MAX_LENGTH, "%s\x1A", text);
    dce->handle_line = example_handle_cmgs;
    if (dte->send_cmd(dte, command, MODEM_COMMAND_TIMEOUT_SMS_MS) != ESP_OK) {
        ESP_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        ESP_LOGE(TAG, "send message failed");
        goto err;
    }
    ESP_LOGD(TAG, "send message ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}

static void modem_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MODEM_EVENT_PPP_START:
        ESP_LOGI(TAG, "Modem PPP Started");
        break;
    case MODEM_EVENT_PPP_CONNECT:
        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ppp_client_ip_info_t *ipinfo = (ppp_client_ip_info_t *)(event_data);
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&ipinfo->ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&ipinfo->netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&ipinfo->gw));
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&ipinfo->ns1));
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&ipinfo->ns2));
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(event_group, CONNECT_BIT);
        break;
    case MODEM_EVENT_PPP_DISCONNECT:
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
        break;
    case MODEM_EVENT_PPP_STOP:
        ESP_LOGI(TAG, "Modem PPP Stopped");
        xEventGroupSetBits(event_group, STOP_BIT);
        break;
    case MODEM_EVENT_UNKNOWN:
        ESP_LOGW(TAG, "Unknow line received: %s", (char *)event_data);
        break;
    default:
        break;
    }
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

void aws_iot_task(void *param) {
    char cPayload[100];

    int32_t i = 0;
    char *topic = (char *)param; 
    int TOPIC_LEN = strlen(topic);

    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    IoT_Publish_Message_Params paramsQOS0;
    IoT_Publish_Message_Params paramsQOS1;

    ESP_LOGI(TAG, "AWS TOPIC is IMEI:%s of length %d", topic, TOPIC_LEN);
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

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

#ifdef CONFIG_EXAMPLE_SDCARD_CERTS
    ESP_LOGI(TAG, "Mounting SD card...");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
        abort();
    }
#endif

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
//    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
//                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
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
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "Subscribing...");
    rc = aws_iot_mqtt_subscribe(&client, topic, TOPIC_LEN, QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }

    sprintf(cPayload, "%s : %d ", "hello from SDK", i);

    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *) cPayload;
    paramsQOS0.isRetained = 0;

    paramsQOS1.qos = QOS1;
    paramsQOS1.payload = (void *) cPayload;
    paramsQOS1.isRetained = 0;

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1000 / portTICK_RATE_MS);
        //sprintf(cPayload, "%s : %d ", "hello from ESP32 (QOS0)", i++);
        //paramsQOS0.payloadLen = strlen(cPayload);
        //rc = aws_iot_mqtt_publish(&client, TOPIC, TOPIC_LEN, &paramsQOS0);
        sprintf(cPayload, "%s : 0x%04X ", "register read from Modbus ESP32)", echo_task());
        paramsQOS1.payloadLen = strlen(cPayload);
        rc = aws_iot_mqtt_publish(&client, topic, TOPIC_LEN, &paramsQOS1);
        if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            ESP_LOGW(TAG, "QOS1 publish ack not received.");
            rc = SUCCESS;
        }
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}


void init_modbus() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

	uint8_t count;
	uint16_t modbus_crc;
    
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Configure UART parameters
    uart_param_config(uart_num, &uart_config);
    
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // Set UART1 pins(TX: IO23, RX: I022, RTS: IO18, CTS: IO19)
    uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Set RS485 half duplex mode
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);

    // Allocate buffers for UART

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    uart_write_bytes(uart_num, "Start RS485 UART test.\r\n", 24);

}

/* -----------------------------------------------------------
| 	homepage_get_handler()
|	HTTP server side handler for GET reuests on /
------------------------------------------------------------*/
static esp_err_t homepage_get_handler(httpd_req_t *req)
{
     /* Get handle to embedded file upload script */
    extern const unsigned char homepage_start[] asm("_binary_index_html_start");
    extern const unsigned char homepage_end[]   asm("_binary_index_html_end");
    const size_t homepage_size = (homepage_end - homepage_start);
	
	//httpd_resp_set_type(req, "text/html"); 
    httpd_resp_send_chunk(req, (const char *)homepage_start, homepage_size);
    httpd_resp_sendstr_chunk(req, NULL);
	
	return(ESP_OK);
}

static const httpd_uri_t homepage = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = homepage_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "Home Page"
};

/* -----------------------------------------------------------
| 	favicon_get_handler()
|	HTTP clients expect an icon from all HTTP servers as a
| 	standard feature 
------------------------------------------------------------*/
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}

static const httpd_uri_t favicon_ico = {
    .uri       = "/favicon.ico",
    .method    = HTTP_GET,
    .handler   = favicon_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "Favicon"
};

/* -----------------------------------------------------------
| 	str2num()
| 	Converts a number in the form of a string to numerical data
| 	type. 
| 	WARNING: The numbers represented as characters are expected
|	to be decimal values. i.e., hex is not allowed
------------------------------------------------------------*/
uint16_t str2num(char* input_str, const char delimiter, uint8_t max_parse_len)
{
	uint8_t index = 0;
	uint16_t result = 0; // TODO: Re-examine. 0 could be a legit input

	while(index < max_parse_len)
	{
		if(input_str[index] == delimiter) {
			break;
		}
		
		result = result*10 + (input_str[index] - 48);
		index++;
	}
	
	return result;
}

/* -----------------------------------------------------------
| 	update_sysconfig()
|	Parses the form string from HTTP client and updates the 
| 	sysconfig values (runtime values as well as in the 
| 	sysconfig.txt
------------------------------------------------------------*/
void update_sysconfig(char* form_str)
{
	char* tmpStr;
	uint8_t first_slave_id, second_slave_id;
	
	static const char* config_file_name = "/spiffs/sysconfig.txt";
	FILE* config_file = NULL;
	
	
	// Parse the received string to obtain sysconfig information
	tmpStr = strstr(form_str, "first_slave_id=") + strlen("first_slave_id=");
	first_slave_id = (uint8_t)str2num(tmpStr, '&', 4);

	tmpStr = strstr(form_str, "second_slave_id=") + strlen("second_slave_id=");
	second_slave_id = (uint8_t)str2num(tmpStr, '&', 4);

	// Debug prints
	printf("First slave id: %d\n", first_slave_id);
	printf("Second slave id: %d\n", second_slave_id);

	sysconfig.first_slave_id = first_slave_id;
	sysconfig.second_slave_id = second_slave_id;

	config_file = fopen(config_file_name, "rb");
	if (config_file == NULL) { // If config file isnt' present, create one with default values
		ESP_LOGE(TAG, "Config file not present. Can't update sysconfig!");
		abort();
	}
	
	fclose(config_file);
	config_file = fopen(config_file_name, "wb");

	if(fwrite(&sysconfig, sizeof(struct config_struct), 1, config_file) != 1) {
		ESP_LOGE(TAG, "Couldn't update sysconfig file although it is present");
		abort();
	} else {
		ESP_LOGI(TAG, "Successfully updated sysconfig file");
	}
}

/* -----------------------------------------------------------
| 	submit_post_handler()
|	HTTP server side handler for GET reuests on /
------------------------------------------------------------*/
static esp_err_t submit_post_handler(httpd_req_t *req)
{

	char buf[FORM_DATA_BUF_SIZE + 1];
	size_t ret, remaining = req->content_len;
	uint16_t content_pos = 0;    

	if (remaining > FORM_DATA_BUF_SIZE) {
		ESP_LOGE(TAG, "HTTP form data larger than internal receive buffer");
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Form data too long");
		return(ESP_FAIL);
	}
 
	// Read the data for the request
	while(remaining > 0)
	{
    	if ((ret = httpd_req_recv(req, &buf[content_pos],
    	                MIN(remaining, FORM_DATA_BUF_SIZE))) <= 0) {
    	    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
    	        // Retry receiving if timeout occurred 
    	        continue;
    	    }
    		
    	    return(ESP_FAIL);
    	}
		
		content_pos = content_pos + ret;
		remaining -= ret;
	}
	
	buf[FORM_DATA_BUF_SIZE]	= '\0'; // As a safety measure against pointer run away issues
	printf("Received string: %s\n", buf);

	update_sysconfig(buf);	

	httpd_resp_send(req, NULL, 0);	
	return(ESP_OK);

}

httpd_uri_t submit = {
    .uri       = "/submit",   // Match all URIs of type /upload/path/to/file
    .method    = HTTP_POST,
    .handler   = submit_post_handler,
    .user_ctx  = "Thanks for submitting the form"
};

/* -----------------------------------------------------------
| 	start_webserver()
|	Starts HTTP server for OTA updates 
------------------------------------------------------------*/
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &homepage);
		httpd_register_uri_handler(server, &favicon_ico);
		httpd_register_uri_handler(server, &submit);
			
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
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
	
	// Populate the default config
	default_config.first_slave_id = CONFIG_FIRST_SLAVE_ID; // From sdkconfig	
	default_config.second_slave_id = CONFIG_SECOND_SLAVE_ID;
	default_config.apn = CONFIG_ESP_MODEM_APN;

	config_file = fopen(config_file_name, "rb");
	if (config_file == NULL) { // If config file isnt' present, create one with default values
		ESP_LOGI(TAG, "Config file not present. Creating one with default values");
		config_file = fopen(config_file_name, "wb");
		
		if (fwrite(&default_config, sizeof(struct config_struct), 1, config_file) != 1) {
			ESP_LOGE(TAG, "Couldn't write to Spiffs file sysconfig.txt");
			fclose(config_file);
			abort();
		} else {
			ESP_LOGI(TAG, "Config file successfully populated with default values");
			sysconfig = default_config; // Also populate the global variable that will be used by many functions
			fclose(config_file);
		}

	} else { // sysconfig.txt file is present. So populate global variable with data from it
		if(fread(&sysconfig, sizeof(struct config_struct), 1, config_file) != 1) {
			ESP_LOGE(TAG, "Couldn't read sysconfig.txt contents");
			abort();
		} else {
			ESP_LOGI(TAG, "Successfully read sysconfig.txt contents into internal variable");
		}
	}
}


void normal_tasks()
{
    static httpd_handle_t http_server = NULL;

	read_sysconfig();
	
	// Get the homepage up: Initialize webserver, register all handlers
    http_server = start_webserver();
    
	init_modbus();
    event_group = xEventGroupCreate();
    /* create dte object */
    esp_modem_dte_config_t config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    modem_dte_t *dte = esp_modem_dte_init(&config);

    /* Register event handler */
    ESP_ERROR_CHECK(esp_modem_add_event_handler(dte, modem_event_handler, NULL));
    /* create dce object */
    modem_dce_t *dce = sim800_init(dte);
    ESP_ERROR_CHECK(dce->set_flow_ctrl(dce, MODEM_FLOW_CONTROL_NONE));
    ESP_ERROR_CHECK(dce->store_profile(dce));
    /* Print Module ID, Operator, IMEI, IMSI */
    ESP_LOGI(TAG, "Module: %s", dce->name);
    ESP_LOGI(TAG, "Operator: %s", dce->oper);
    ESP_LOGI(TAG, "IMEI: %s", dce->imei);
    ESP_LOGI(TAG, "IMSI: %s", dce->imsi);
    /* Get signal quality */
    uint32_t rssi = 0, ber = 0;
    ESP_ERROR_CHECK(dce->get_signal_quality(dce, &rssi, &ber));
    ESP_LOGI(TAG, "rssi: %d, ber: %d", rssi, ber);
    /* Get battery voltage */
    uint32_t voltage = 0, bcs = 0, bcl = 0;
    ESP_ERROR_CHECK(dce->get_battery_status(dce, &bcs, &bcl, &voltage));
    ESP_LOGI(TAG, "Battery voltage: %d mV", voltage);
    /* Setup PPP environment */
    esp_modem_setup_ppp(dte);
    /* Wait for IP address */
    xEventGroupWaitBits(event_group, CONNECT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
//    xEventGroupWaitBits(event_group, GOT_DATA_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    /* Exit PPP mode */
//    ESP_ERROR_CHECK(esp_modem_exit_ppp(dte));
//    xEventGroupWaitBits(event_group, STOP_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
#if CONFIG_SEND_MSG
    const char *message = "Welcome to ESP32!";
    ESP_ERROR_CHECK(example_send_message_text(dce, CONFIG_SEND_MSG_PEER_PHONE_NUMBER, message));
    ESP_LOGI(TAG, "Send send message [%s] ok", message);
#endif
    /* Power down module */
//    ESP_ERROR_CHECK(dce->power_down(dce));
//    ESP_LOGI(TAG, "Power down");
//    ESP_ERROR_CHECK(dce->deinit(dce));
//    ESP_ERROR_CHECK(dte->deinit(dte));
    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, (void *) dce->imei, 5, NULL, 1);
}
