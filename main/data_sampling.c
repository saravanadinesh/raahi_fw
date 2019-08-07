/**************************************************************
* data_sampling.c
*
* Contains code that is pertaining to sampling all kinds of 
* data namely data from customer sensors, time data, GPS data
* etc
**************************************************************/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp32/rom/gpio.h"
#include "raahi.h"
#include "esp_sntp.h"

// MODBUS related Defines 
#define MODBUS_TXD   (23)
#define MODBUS_RXD   (22)
#define MODBUS_RTS   (21)
#define MODBUS_CTS  UART_PIN_NO_CHANGE
// GPS receiver related Defines
#define GPS_TASK_TXD   (19)
#define GPS_TASK_RXD   (27)
#define GPS_TASK_RTS  UART_PIN_NO_CHANGE
#define GPS_TASK_CTS  UART_PIN_NO_CHANGE

// Defines that are common to both MODBUS and GPS (They share the same UART)
#define BUF_SIZE        (127)
#define BAUD_RATE       (9600)
#define GPGLL_LEN       (47)
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define DATA_SAMPLING_UART      (UART_NUM_2)
static const int RX_BUF_SIZE = 1024;

#define MODBUS_BUF_SIZE 128

// ADC related defines
#define ONE_SHOT_CONVERSION 0x0
#define CONTINUOUS_CONVERSION 0x1

#define RESOLUTION_12BITS_240SPS 0X0
#define RESOLUTION_14BITS_60SPS 0X1
#define RESOLUTION_16BITS_15SPS 0X2
#define ADC_RESOLUTION RESOLUTION_16BITS_15SPS

#define NO_GAIN 0x0
#define GAIN_2X 0x1
#define GAIN_4X 0x2
#define GAIN_8X 0x3

#define ADC_ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ADC_ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ADC_ACK_VAL 0x0                             /*!< I2C ack value */
#define ADC_NACK_VAL 0x1                            /*!< I2C nack value */

#define MCP342X_ADDRESS 0x68

// I2C master related defines
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

union adc_config_reg_datatype
{
  struct 
  {
    unsigned int gain: 2;
    unsigned int resolution: 2;
    unsigned int conv_mode: 1;
    unsigned int channel: 2;
    unsigned int rdy: 1;
  }fields;
  unsigned int word;
};

/* Global variables */
static const char *TAG = "data_sampling_task";
extern char raahi_log_str[EVENT_JSON_STR_SIZE];
extern EventGroupHandle_t esp_event_group;
extern const int SNTP_CONNECT_BIT;
extern int today, this_hour;

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
extern struct debug_data_struct debug_data;
extern char user_mqtt_str[MAX_DEVICE_ID_LEN];


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
    data_out[2] = (uint8_t) (reg_address >> 8);   // Reg address is transmitted MSByte first
    data_out[3] = (uint8_t) (reg_address & 0xFF);
    data_out[4] = 0x00; // Number of registers is transmitted MSByte first
    data_out[5] = 0x01;

	// Calculate CRC for the command
    modbus_crc = usMBCRC16(data_out, 6);
    data_out[6] = (uint8_t) (modbus_crc & 0xFF); // IMPORTANT: CRC is transmitted LSByte first (unlike reg address and no. of registers)
    data_out[7] = (uint8_t) (modbus_crc >> 8);    
	
	// Write modbus master command on rs485
    uart_write_bytes(DATA_SAMPLING_UART, (const char*)&data_out[0], 8);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	for (count = 0; count <5; count++)
	{
		//Read data from UART
    	int len = uart_read_bytes(DATA_SAMPLING_UART, data_in, MODBUS_BUF_SIZE, PACKET_READ_TICS);
    
    	//Write data back to UART
    	if ((len == 7) && (data_in[0] == data_out[0])) {
			modbus_crc = usMBCRC16(data_in, len-2);
			if(((uint8_t)(modbus_crc & 0xFF) == data_in[len-2]) && ((uint8_t)(modbus_crc >> 8) == data_in[len-1])) {
				*result = data_in[len-4];
				*result = (*result << 8) | data_in[len-3]; 
				return_val = ESP_OK;
				break; 
			} else // CRC error occurred
				RAAHI_LOGI(TAG, "Modbus CRC Error occurred. Expected: 0x%.4X,\t Got:0x%.2X%.2X", modbus_crc, data_in[len-2], data_in[len-1]);
				return_val = ESP_ERR_INVALID_RESPONSE;
				break;
    	} 

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	} // end of for count..
	
	if (count == 5) {
		RAAHI_LOGE(TAG, "Slave %d did not respond", slave_id);
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
	esp_err_t modbus_read_ret_val;
	time_t now;

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

			modbus_read_ret_val = modbus_read(sysconfig.slave_id[slave_id_idx], sysconfig.reg_address[reg_address_idx], &modbus_read_result);
			if(modbus_read_ret_val == ESP_OK) {	
        	   	time(&now);
				if (user_mqtt_str != NULL) {
					sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %d, \"%s\": %u, \"%s\": %u}", \
							"deviceId", user_mqtt_str, \
        	                "timestamp", now, \
							"slave_id", sysconfig.slave_id[slave_id_idx], \
							"reg_address", sysconfig.reg_address[reg_address_idx], \
							"reg_value", modbus_read_result);
    			} else {
					sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %d, \"%s\": %u, \"%s\": %u}", \
							"deviceId", "user_id_NA", \
        	                "timestamp", now, \
							"slave_id", sysconfig.slave_id[slave_id_idx], \
							"reg_address", sysconfig.reg_address[reg_address_idx], \
							"reg_value", modbus_read_result);
				}
				//xEventGroupWaitBits(mqtt_rw_group, READ_OP_DONE, pdFALSE, pdTRUE, portMAX_DELAY); // Wait until aws task reads from queue
				//xEventGroupClearBits(mqtt_rw_group, WRITE_OP_DONE);
				strcpy(data_json.packet[data_json.write_ptr], cPayload);
				data_json.write_ptr = (data_json.write_ptr+1) % DATA_JSON_QUEUE_SIZE;
				//xEventGroupSetBits(mqtt_rw_group, WRITE_OP_DONE);

				ESP_LOGI(TAG, "Json: %s", cPayload);

				// Updata debug data
				debug_data.slave_info[slave_id_idx].status = CONNECTED_AND_UPDATING;
				debug_data.slave_info[slave_id_idx].data[reg_address_idx] = modbus_read_result;
	
			} else if (modbus_read_ret_val == ESP_ERR_INVALID_RESPONSE) {// There was CRC error
				debug_data.slave_info[slave_id_idx].status = CONNECTED_WITH_ISSUES;
				debug_data.slave_info[slave_id_idx].data[reg_address_idx] = 0;
			} else { // Slave didn't even respond
				debug_data.slave_info[slave_id_idx].status = NOT_CONNECTED;
				debug_data.slave_info[slave_id_idx].data[reg_address_idx] = 0;
			}

	
		} // End of for loop on reg_address_idx
	} // End of for loop on slave_id_idx

} 


void parse_gpgll(float *gps_lat,float *gps_lng, char *gpgll_line) {
  char *ptr = gpgll_line;
  *gps_lat = floor(strtof(gpgll_line+6,NULL)/100)+strtof(gpgll_line+8,NULL)/60;
  *gps_lng = floor(strtof(gpgll_line+19,NULL)/100)+strtof(gpgll_line+22,NULL)/60;
  if(gpgll_line[17] == 'S') {
     *gps_lat = 0 - (*gps_lat); 
  }
  if(gpgll_line[31] == 'W') {
     *gps_lng = 0 - (*gps_lng); 
  }
     

}

/* -----------------------------------------------------------
| 	gps_sampling_task
| 	Collects gps samples
------------------------------------------------------------*/
void gps_sampling_task()
{
	//uart_write_bytes(DATA_SAMPLING_UART, "I am now in the gps task\r\n", strlen("I am now in the gps task\r\n"));
	static const char *RX_TASK_TAG = "GPS SAMPLING TASK";
    time_t now;
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
	uint8_t count;
      
    char cPayload[DATA_JSON_STR_SIZE];
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    
	for (count = 0; count <5; count++)
	{
    	const int rxBytes = uart_read_bytes(DATA_SAMPLING_UART, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
    	if (rxBytes > 0) {
			data[rxBytes] = 0;
    		bool valid = true;
    		char *ptr = strstr((char*)data, "GPGLL");
	    	if (ptr != NULL) /* Substring found */
	    	{
        		char *gpgll_line, *gpgll_start;
        		gpgll_line = (char*) malloc(GPGLL_LEN+1);
        		gpgll_start=gpgll_line;
        		/*copy GPGLL line */
        		uint8_t cntr = 0 ; 
        		while ((ptr != NULL) && (*ptr != '*') && (cntr < 7)) {
        		   *gpgll_start = *ptr;
        		   if (*ptr == ',') {
        		     ++cntr;
        		   }
        		   ++gpgll_start, ++ptr;
        		}
				*gpgll_start = '\0'; 
        		ESP_LOGI(RX_TASK_TAG, "GPGLL line '%s': size: %d",gpgll_line,strlen(gpgll_line));
        	  	char valid = gpgll_line[strlen(gpgll_line)-2];
        	  	if (valid == 'V') {
        	   		ESP_LOGI(RX_TASK_TAG, "GPGLL Void line '%s'",gpgll_line);
        	    	free(gpgll_line);
        	    	gpgll_start = NULL;
        	  	} else if (valid == 'A') {
        	   		ESP_LOGI(RX_TASK_TAG, "GPGLL Active line '%s'",gpgll_line);
        	   		float gps_lat, gps_lng;
        	   		parse_gpgll(&gps_lat, &gps_lng, gpgll_line); 
        	   		time(&now);
        	   		ESP_LOGI(RX_TASK_TAG, "GPGLL lat:'%.5f' lng:'%.5f'",gps_lat,gps_lng);
        	   		if (user_mqtt_str != NULL) {
        	      		sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %.6f, \"%s\": %.6f}", \
        	                "deviceId", user_mqtt_str, \
        	                "timestamp", now, \
        	                "lat", gps_lat, \
        	                "lng", gps_lng);
        	   		} else {
        	      		sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %.6f, \"%s\": %.6f}", \
        	                "deviceId", "user_id_NA", \
        	                "timestamp", now, \
        	                "lat", gps_lat, \
        	                "lng", gps_lng);
        	   		}

        	   		strcpy(data_json.packet[data_json.write_ptr], cPayload);
        	   		data_json.write_ptr = (data_json.write_ptr+1) % DATA_JSON_QUEUE_SIZE;
        	   		/* parse line to extract location */
        	  	}
				break;
	    	} // End of if (ptr != NULL)
		} // End of if (rxBytes > 0) 
		
		vTaskDelay(1000 / portTICK_PERIOD_MS);

	} // end of for loop

    free(data);
    return;
}

static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
} 

esp_err_t read_mcp342x(uint8_t channel, uint16_t* data)
{
	
    i2c_cmd_handle_t cmd;
	union adc_config_reg_datatype config_reg;
	uint8_t data_h, data_l, reg_val;  
	esp_err_t ret;
	i2c_port_t i2c_num = I2C_MASTER_NUM;

	// Configure the ADC
	config_reg.fields.rdy = 1; 
	config_reg.fields.conv_mode = ONE_SHOT_CONVERSION;
	config_reg.fields.resolution = RESOLUTION_16BITS_15SPS;
	config_reg.fields.channel = channel;
	config_reg.fields.gain = NO_GAIN;

	// Write command to ADC to set ADC parameters and start conversion
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP342X_ADDRESS << 1 | I2C_MASTER_WRITE, ADC_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, config_reg.word, ADC_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
   
	vTaskDelay(100 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MCP342X_ADDRESS << 1 | I2C_MASTER_READ, ADC_ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_h, ADC_ACK_VAL);
    i2c_master_read_byte(cmd, &data_l, ADC_ACK_VAL);
    i2c_master_read_byte(cmd, &reg_val, ADC_NACK_VAL);
    
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	if (reg_val  != (config_reg.word & 0x7F)) // Mask out the RDY bit and check if the rest of the reg value is as expected
	{
		RAAHI_LOGE(TAG, "ADC config register was writtent with %x, but read back %x", config_reg.word, reg_val);
		return(ESP_FAIL);
	}

	if ((reg_val & 0x80) == 0)	// Check the RDY bit to ensure data result is a fresh conversion
	{
		*data = (data_h << 8) | data_l ;
		return(ESP_OK); 
	}
	else
	{
		return(ESP_FAIL);
	}
 
}

void adc_sensor_task()
{
	uint8_t channel;
	uint16_t data;
	float voltage_mV, mV_per_bit, current_mA, resistance_ohms, terminal_voltage_mV;
	esp_err_t result;

	time_t now;	
    char cPayload[DATA_JSON_STR_SIZE];

	switch(ADC_RESOLUTION)	//Reference: MCP342x datasheet
	{
		case RESOLUTION_16BITS_15SPS:
			mV_per_bit = 0.0625; // 62.5uV
			break;

		case RESOLUTION_14BITS_60SPS:
			mV_per_bit = 0.250; // 250uV
			break;

		case RESOLUTION_12BITS_240SPS:
			mV_per_bit = 1; // 1mV
			break;
	
		default:
			RAAHI_LOGE(TAG, "Unknown ADC resolution encountered");
			return;
	}

	for (channel = 0; channel < MAX_ADC_CHANNELS; channel++)
	{
		if(sysconfig.analog_sensor_type[channel] == NONE)
		{
			continue;
		}

		result = read_mcp342x(channel, &data);
		if (result != ESP_OK)
		{
			RAAHI_LOGE(TAG, "ADC didn't respond while reading channel %u", channel);
			continue;
		}

		voltage_mV = data * mV_per_bit;
		time(&now);
		switch(sysconfig.analog_sensor_type[channel])
		{
			case FOURTWENTY:
				current_mA = voltage_mV / CONFIG_CURRENT_LOOP_RECEIVER_RESISTOR;					
        	    sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %u, \"%s\": %.3f, \"%s\": \"%s\"}", \
        	        "deviceId", user_mqtt_str, \
        	        "timestamp", now, \
					"adc_channel", channel, \
        	        "adc_reading", current_mA, \
        	        "unit", "mA");
				break;

			case RESISTIVE:
				resistance_ohms = CONFIG_RESISTIVE_DIVIDER / ((3300/voltage_mV) - 1);	// We are assuming that V_SUPPLY is 3.3V				
        	    sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %u, \"%s\": %.3f, \"%s\": \"%s\"}", \
        	        "deviceId", user_mqtt_str, \
        	        "timestamp", now, \
					"adc_channel", channel, \
        	        "adc_reading", resistance_ohms, \
        	        "unit", "ohms");
				break;

			case DIRECT:
				terminal_voltage_mV = voltage_mV * ((CONFIG_DIRECT_VOLTAGE_RECEIVER_RESISTOR + CONFIG_DIRECT_VOLTAGE_DIVIDER_RESISTOR) / CONFIG_DIRECT_VOLTAGE_RECEIVER_RESISTOR);
        	    sprintf(cPayload, "{\"%s\": \"%s\", \"%s\": %lu, \"%s\": %u, \"%s\": %.3f, \"%s\": \"%s\"}", \
        	        "deviceId", user_mqtt_str, \
        	        "timestamp", now, \
					"adc_channel", channel, \
        	        "adc_reading", terminal_voltage_mV, \
        	        "unit", "mV");
				break;

			default:
				RAAHI_LOGE(TAG, "Encountered unknown analog sensory type for channel %u", channel);
				continue;			
		}
        
		strcpy(data_json.packet[data_json.write_ptr], cPayload);
        data_json.write_ptr = (data_json.write_ptr+1) % DATA_JSON_QUEUE_SIZE;
		
	}	

}


/* -----------------------------------------------------------
| 	data_sampling_task
| 	This is the main task in this file. This calls modbus task
| 	and GPS task in an infinite loop  
------------------------------------------------------------*/
void data_sampling_task(void *param)
{
    
  	time_t now;
    struct tm timeinfo;

	uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(i2c_master_init());
	while(1) {

    	// Configure UART parameters for sampling on MODBUS
    	uart_param_config(DATA_SAMPLING_UART, &uart_config);
    	uart_set_pin(DATA_SAMPLING_UART, MODBUS_TXD, MODBUS_RXD, MODBUS_RTS, MODBUS_CTS);
    	uart_driver_install(DATA_SAMPLING_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
    	uart_set_mode(DATA_SAMPLING_UART, UART_MODE_RS485_HALF_DUPLEX);

		modbus_sensor_task();

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
				RAAHI_LOGE(TAG, "UART driver couldn't be deleted to carry out sensor task");
				abort();
			}
       		gpio_matrix_out(GPS_TASK_TXD, 0X100, 0, 0);
		} else {
			RAAHI_LOGI(TAG, "UART Driver couldn't be deleted for switching to GPS task");
		}

		//adc_sensor_task();

        vTaskDelay((sysconfig.sampling_period_in_sec * 1000) / portTICK_RATE_MS);
	
		// We should restart every 1 day to make sure the code isn't stuck in some place forever
    	if(xEventGroupGetBits(esp_event_group)  & SNTP_CONNECT_BIT)
		{
			time(&now);
    		localtime_r(&now, &timeinfo);
			if (timeinfo.tm_mday != today) {
				today = timeinfo.tm_mday;
				RAAHI_LOGI(TAG, "Restarting in 10 sec since 24hrs have passed");
				vTaskDelay(1000/portTICK_RATE_MS);
				abort();
			}
		}
	
	}// End of infinite while loop		
  	
}

