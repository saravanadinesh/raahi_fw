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
	char* apn;
};

#endif //#ifndef _RAAHI_FW_H_
