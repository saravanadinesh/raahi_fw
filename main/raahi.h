/**************************************************************
* Raahi firmware header file
* Contains data types and defines specific to raahi fw, but 
* global to all files that make up the firmware 
**************************************************************/
#ifndef _RAAHI_FW_H_
#define _RAAHI_FW_H_

// Data type definitions
struct config_struct {
	uint8_t first_slave_id;
	uint8_t second_slave_id;
	uint16_t first_reg;
	uint16_t second_reg;
	uint16_t third_reg;
	uint16_t samples_per_min;
	
	char topic[30];
	char apn[20];
};

#endif //#ifndef _RAAHI_FW_H_
