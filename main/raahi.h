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

// Data type definitions
struct config_struct {
	uint8_t slave_id[MAX_MODBUS_SLAVES];
	uint16_t reg_address[MAX_MODBUS_REGISTERS];
	uint16_t sampling_period_in_sec;
	
	char topic[MAX_TOPIC_LEN + 1];
	char apn[MAX_APN_LEN + 1];
};

#endif //#ifndef _RAAHI_FW_H_