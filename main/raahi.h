/**************************************************************
* Raahi firmware header file
* Contains data types and defines specific to raahi fw, but 
* global to all files that make up the firmware 
**************************************************************/
#ifndef _RAAHI_FW_H_
#define _RAAHI_FW_H_

// Data type definitions
struct config_struct {
	uint8_t slave_id[CONFIG_MAX_MODBUS_SLAVES];
	uint16_t reg_address[CONFIG_MAX_MODBUS_REGISTERS];
	uint16_t sampling_period_in_sec;
	
	char topic[30];
	char apn[20];
};

#endif //#ifndef _RAAHI_FW_H_
