/**************************************************************
* data_sampling.c
*
* Contains code that is pertaining to sampling all kinds of 
* data namely data from customer sensors, time data, GPS data
* etc
**************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "esp32/rom/gpio.h"

#include "raahi.h"

// MODBUS related Defines 
#define MODBUS_TXD   (23)
#define MODBUS_RXD   (22)
#define MODBUS_RTS   (21)
#define MODBUS_CTS  UART_PIN_NO_CHANGE

// GPS receiver related Defines
#define GPS_TASK_TXD   (18)
#define GPS_TASK_RXD   (19)
#define GPS_TASK_RTS  UART_PIN_NO_CHANGE
#define GPS_TASK_CTS  UART_PIN_NO_CHANGE

// Defines that are common to both MODBUS and GPS (They share the same UART)
#define BUF_SIZE        (127)
#define BAUD_RATE       (9600)
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define DATA_SAMPLING_UART      (UART_NUM_2)

#define MODBUS_BUF_SIZE 128

/* Global variables */
static const char *TAG = "data_sampling_task";

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

extern struct config_struct sysconfig;
extern EventGroupHandle_t mqtt_rw_group;
extern const int READ_OP_DONE;
extern const int WRITE_OP_DONE;

extern struct data_json_struct data_json;
extern struct event_json_struct event_json;
extern char *user_mqtt_str;


/********************************************************************/
/* Actual code starts here */

/* -----------------------------------------------------------
| 	usMBCRC16()
|	Calculates MODBUS 16 bit CRC  
------------------------------------------------------------*/
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

/* -----------------------------------------------------------
| 	modbus_read
|	Sends out multi-register read command (but for only one 
| 	register  
------------------------------------------------------------*/
static esp_err_t modbus_read(uint8_t slave_id, uint16_t reg_address, uint16_t* result)
{
    uint8_t data_out[MODBUS_BUF_SIZE] = {0}; 
    uint8_t data_in[MODBUS_BUF_SIZE] = {0}; 
	uint16_t modbus_crc;
	uint8_t count;
	esp_err_t return_val = ESP_FAIL;

	*result = 0;
	
	// Error Checks

	if (slave_id == 0) { // Zero is not allowed
		return(ESP_FAIL);
	}
	
	// Compose modbus command 
    data_out[0] = slave_id;
    data_out[1] = 0x03;
    data_out[2] = (uint8_t) (reg_address >> 8);
    data_out[3] = (uint8_t) (reg_address & 0xFF);
    data_out[4] = 0x00;
    data_out[5] = 0x01;

	// Calculate CRC for the command
    modbus_crc = usMBCRC16(data_out, 6);
    data_out[6] = (uint8_t) (modbus_crc >> 8);   
    data_out[7] = (uint8_t) (modbus_crc & 0xFF); 
	
	// Write modbus master command on rs485
    uart_write_bytes(DATA_SAMPLING_UART, (const char*)&data_out[0], 8);
	for (count = 0; count <5; count++)
	{
		//Read data from UART
    	int len = uart_read_bytes(DATA_SAMPLING_UART, data_in, MODBUS_BUF_SIZE, PACKET_READ_TICS);
    
    	//Write data back to UART
    	if ((len > 0) && (data_in[0] == data_out[0])) {
        	ESP_LOGI(TAG, "Received %u bytes:", len);
			modbus_crc = usMBCRC16(data_in, len-2);
			if(((uint8_t)(modbus_crc & 0xFF) == data_in[len-1]) && ((uint8_t)(modbus_crc >> 8) == data_in[len-2])) {
				*result = data_in[len-4];
				*result = (*result << 8) | data_in[len-3]; 
				return_val = ESP_OK;
				break; 
			} else // CRC error occurred
				ESP_LOGI(TAG, "Modbus CRC Error occurred");
				return_val = ESP_FAIL;
				break;
    	} else {
			printf("Waiting for slave to respond \n");
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	} // end of for count..
	
	if (count == 5) {
		ESP_LOGE(TAG, "Slave %d did not respond", slave_id);
		return_val = ESP_FAIL;
	}

	return(return_val);
}


/* -----------------------------------------------------------
| 	modbus_sensor_task
| 	For every relevant slave and every relevant register of that 
|	slave, calls modbus_read function, processes the result and
|	composes mqtt json packet and adds it to a json stack
------------------------------------------------------------*/
void modbus_sensor_task()
{
	uint16_t modbus_read_result;
    char cPayload[DATA_JSON_STR_SIZE];
	uint8_t slave_id_idx, reg_address_idx;
	
	for (slave_id_idx = 0; slave_id_idx < MAX_MODBUS_SLAVES; slave_id_idx++)
	{	
		if (sysconfig.slave_id[slave_id_idx] == 0) {// Slave ID of 0 is considered to be an uninitialized entry
			break;
		}
		
		for (reg_address_idx = 0; reg_address_idx < MAX_MODBUS_REGISTERS; reg_address_idx++)
		{
			if (sysconfig.reg_address[reg_address_idx] == 0) {// reg address 0 is considered to be unintialized entry
				break;
			}

			
			if(modbus_read(sysconfig.slave_id[slave_id_idx], sysconfig.reg_address[reg_address_idx], &modbus_read_result) == ESP_OK) {	
				if (user_mqtt_str != NULL) {
					sprintf(cPayload, "{\"%s\": \"%s\", \"%s\" : %d, \"%s\" : 0x%.4X, \"%s\" : 0x%.4X}", \
							"user_id", user_mqtt_str, \
							"slave_id", sysconfig.slave_id[slave_id_idx], \
							"reg_address", sysconfig.reg_address[reg_address_idx], \
							"reg_value", modbus_read_result);
    			} else {
					sprintf(cPayload, "{\"%s\": \"%s\", \"%s\" : %d, \"%s\" : 0x%.4X, \"%s\" : 0x%.4X}", \
							"user_id", "user_id_NA", \
							"slave_id", sysconfig.slave_id[slave_id_idx], \
							"reg_address", sysconfig.reg_address[reg_address_idx], \
							"reg_value", modbus_read_result);
				}
				xEventGroupWaitBits(mqtt_rw_group, READ_OP_DONE, pdFALSE, pdTRUE, portMAX_DELAY); // Wait until aws task reads from queue
				xEventGroupClearBits(mqtt_rw_group, WRITE_OP_DONE);
        		strcpy(data_json.packet[data_json.write_ptr], cPayload);
				data_json.write_ptr = (data_json.write_ptr+1) % DATA_JSON_QUEUE_SIZE;
				xEventGroupSetBits(mqtt_rw_group, WRITE_OP_DONE);

				ESP_LOGI(TAG, "Json Message: %s", cPayload);
			} 
	
		} // End of for loop on reg_address_idx
	} // End of for loop on slave_id_idx

} 

/* -----------------------------------------------------------
| 	gps_sampling_task
| 	Collects gps samples
------------------------------------------------------------*/
void gps_sampling_task()
{
	uart_write_bytes(DATA_SAMPLING_UART, "I am now in the gps task\r\n", strlen("I am now in the gps task\r\n"));
	return;
}

/* -----------------------------------------------------------
| 	data_sampling_task
| 	This is the main task in this file. This calls modbus task
| 	and GPS task in an infinite loop  
------------------------------------------------------------*/
void data_sampling_task(void *param)
{
    
	uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };


	while(1) {

    	// Configure UART parameters for sampling on MODBUS
    	uart_param_config(DATA_SAMPLING_UART, &uart_config);
    	uart_set_pin(DATA_SAMPLING_UART, MODBUS_TXD, MODBUS_RXD, MODBUS_RTS, MODBUS_CTS);
    	uart_driver_install(DATA_SAMPLING_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
    	uart_set_mode(DATA_SAMPLING_UART, UART_MODE_RS485_HALF_DUPLEX);

		modbus_sensor_task();

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

		uart_wait_tx_done(DATA_SAMPLING_UART, 500 / portTICK_RATE_MS);
		if (uart_driver_delete(DATA_SAMPLING_UART) == ESP_OK) { //End modbus related activity so that uart can be reused for GPS
	    	// Start the GPS sensor related uart configuration
          	gpio_matrix_out(MODBUS_TXD, 0X100, 0, 0);
			
			// Configure UART parameters for sampling on GPS UART
			uart_param_config(DATA_SAMPLING_UART, &uart_config);
    		uart_set_pin(DATA_SAMPLING_UART, GPS_TASK_TXD, GPS_TASK_RXD, GPS_TASK_RTS, GPS_TASK_CTS);
    		uart_driver_install(DATA_SAMPLING_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
			uart_set_mode(DATA_SAMPLING_UART, UART_MODE_UART);
			uart_flush_input(DATA_SAMPLING_UART);
    		
			gps_sampling_task();
			
			uart_wait_tx_done(DATA_SAMPLING_UART, 500 / portTICK_RATE_MS);
			uart_flush_input(DATA_SAMPLING_UART);
			
			if(uart_driver_delete(DATA_SAMPLING_UART) != ESP_OK) {
				ESP_LOGE(TAG, "UART driver couldn't be deletedi to carry out sensor task");
				abort();
			}
       		gpio_matrix_out(GPS_TASK_TXD, 0X100, 0, 0);
		} else {
			ESP_LOGI(TAG, "UART Driver couldn't be deleted for switching to GPS task");
		}

        vTaskDelay((sysconfig.sampling_period_in_sec * 1000) / portTICK_RATE_MS);
		
	}// End of infinite while loop		
  	
}
