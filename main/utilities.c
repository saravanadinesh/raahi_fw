
/**************************************************************
* Contains utility functions used by all other code in raahi fw
**************************************************************/

#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "esp_modem.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "raahi.h"
#include "freertos/task.h"
#include "esp_sntp.h"

#define RGB_LED_GREEN_PIN 27 // TODO: Move this to sdkconfig
#define RGB_LED_RED_PIN 32 // TODO: Move this to sdkconfig
#define RGB_LED_BLUE_PIN 26 // TODO: Move this to sdkconfig

static const char *TAG = "utilities";
extern struct config_struct sysconfig;
extern char user_mqtt_str[MAX_DEVICE_ID_LEN];
extern struct debug_data_struct debug_data;
extern zombie_info_struct zombie_info;

extern void raahi_restart(void);
esp_reset_reason_t stringify_reset_reason(char* reset_reason_str);

esp_reset_reason_t stringify_reset_reason(char* reset_reason_str)
{
    esp_reset_reason_t reset_reason;

    reset_reason = esp_reset_reason();
    
    switch(reset_reason)
    {
        case ESP_RST_UNKNOWN:    //!< Reset reason can not be determined
            strcpy(reset_reason_str, "Unknown");
            break;
        case ESP_RST_POWERON:    //!< Reset due to power-on event
            strcpy(reset_reason_str, "Power Cycle ");
            break;
        case ESP_RST_EXT:        //!< Reset by external pin (not applicable for ESP32)
            strcpy(reset_reason_str, "External Pin");
            break;
        case ESP_RST_SW:         //!< Software reset via esp_restart
            strcpy(reset_reason_str, "esp_restart");
            break;
        case ESP_RST_PANIC:      //!< Software reset due to exception/panic
            strcpy(reset_reason_str, "Panic");
            break;
        case ESP_RST_INT_WDT:    //!< Reset (software or hardware) due to interrupt watchdog
            strcpy(reset_reason_str, "Watchdog Intr");
            break;
        case ESP_RST_TASK_WDT:   //!< Reset due to task watchdog
            strcpy(reset_reason_str, "Watchdog Task");
            break;
        case ESP_RST_WDT:        //!< Reset due to other watchdogs
            strcpy(reset_reason_str, "Other Watchdog");
            break;
        case ESP_RST_DEEPSLEEP:  //!< Reset after exiting deep sleep mode
            strcpy(reset_reason_str, "Deep Sleep Reset");
            break;
        case ESP_RST_BROWNOUT:   //!< Brownout reset (software or hardware)
            strcpy(reset_reason_str, "Brownout");
            break;
        case ESP_RST_SDIO:       //!< Reset over SDIO
            strcpy(reset_reason_str, "SDIO");
            break;
        default:
            strcpy(reset_reason_str, "Undefined");
    }

    return reset_reason;
}

/* -----------------------------------------------------------
| 	set_status_LED(status_led_struct)
|   Sets the status LED to a certain color and blink rate, if	
| 	blinking is asked for. 
------------------------------------------------------------*/
void set_status_LED(status_led_struct status_led)
{
	switch(status_led.colour)
	{
		case RED:
    		gpio_set_level(RGB_LED_RED_PIN, 0);
    		gpio_set_level(RGB_LED_GREEN_PIN, 1);
    		gpio_set_level(RGB_LED_BLUE_PIN, 1);
			break;
		case GREEN:
    		gpio_set_level(RGB_LED_RED_PIN, 1);
    		gpio_set_level(RGB_LED_GREEN_PIN, 0);
    		gpio_set_level(RGB_LED_BLUE_PIN, 1);
			break;
		case BLUE:
    		gpio_set_level(RGB_LED_RED_PIN, 1);
    		gpio_set_level(RGB_LED_GREEN_PIN, 1);
    		gpio_set_level(RGB_LED_BLUE_PIN, 0);
			break;
		default:
			RAAHI_LOGE(TAG, "LED requested to be set to an undefined colour");
			break;
	}
}

/* -----------------------------------------------------------
| 	str2num()
| 	Converts a number in the form of a string to numerical data
| 	type. 
| 	WARNING: The numbers represented as characters are expected
|	to be decimal values. i.e., hex is not allowed
------------------------------------------------------------*/
int32_t str2num(char* input_str, const char delimiter, uint8_t max_parse_len)
{
	uint8_t index = 0;
	int32_t result = 0; // TODO: Re-examine. 0 could be a legit input

	while(index < max_parse_len)
	{
		if(input_str[index] == delimiter) {
			break;
		}
		
		if((uint8_t)input_str[index] > 57 || (uint8_t)input_str[index] < 48) {//If the character is not even a number
			RAAHI_LOGI(TAG, "String entered has non (decimal) numbers");
			return (-1);
		}  
		
		result = result*10 + (input_str[index] - 48);
		index++;
	}

	if (index == max_parse_len) { // Delimiter wasn't detected
		RAAHI_LOGI(TAG, "Delimiter wasn't detected");
		return(-1);
	}
	if (index == 0) { // User didn't enter any value
		return(-1);
	}
	
	return result;
}


/* -----------------------------------------------------------
|   create_info_json()
|   Composes an info json string that can then be sent via SMS 
|   or to AWS
------------------------------------------------------------*/
void create_info_json(char* json_str, uint16_t json_str_len)
{
    struct json_struct info_json[MAX_INFO_JSON_ITEMS];
    uint8_t item, no_of_items = 0;
    uint16_t running_str_len;
    char tempStr[MAX_KEY_LEN + MAX_VALUE_LEN];
    esp_reset_reason_t reset_reason;
    time_t now;

    printf("Info json max strlen: %u\n", json_str_len); 
    time(&now);

    sprintf(info_json[no_of_items].key, "\"IMEI\"");
    sprintf(info_json[no_of_items].value, "\"%s\"", debug_data.imei);
    no_of_items++;

    sprintf(info_json[no_of_items].key, "\"timestamp\"");
    sprintf(info_json[no_of_items].value, "%lu", now);
    no_of_items++;

    sprintf(info_json[no_of_items].key, "\"Operator\"");
    sprintf(info_json[no_of_items].value, "%s", debug_data.oper);
    no_of_items++;

    sprintf(info_json[no_of_items].key, "\"RSSI\"");
    sprintf(info_json[no_of_items].value, "%u", debug_data.rssi);
    no_of_items++;

    sprintf(info_json[no_of_items].key, "\"BER\"");
    sprintf(info_json[no_of_items].value, "%u", debug_data.ber);
    no_of_items++;
   
    sprintf(info_json[no_of_items].key, "\"Battery Voltage\"");
    sprintf(info_json[no_of_items].value, "%u", debug_data.battery_voltage);
    no_of_items++;

    reset_reason = stringify_reset_reason(debug_data.reset_reason_str);

    sprintf(info_json[no_of_items].key, "\"Reset Reason\"");
    sprintf(info_json[no_of_items].value, "%s", debug_data.reset_reason_str);
    no_of_items++;
    
    if(reset_reason == ESP_RST_SW)
    {
        sprintf(info_json[no_of_items].key, "\"esp_restart reason\"");
        sprintf(info_json[no_of_items].value, "%s", zombie_info.esp_restart_reason);
        no_of_items++;
    }
 
    json_str[0] = '\0';
    strcat(json_str, "{");
    running_str_len = strlen("{");
    for (item = 0; item < no_of_items; item++)
    {
        tempStr[0] = '\0';
        strcat(tempStr, info_json[item].key);
        strcat(tempStr, ": ");
        strcat(tempStr, info_json[item].value);

        if (item < no_of_items - 1)
        {
            strcat(tempStr, ",");
        }
        else
        {
            strcat(tempStr, "}");
        }

        if(running_str_len + strlen(tempStr) < json_str_len)
        {
            running_str_len = running_str_len + strlen(tempStr);
            strcat(json_str, tempStr);
        }
        else
        {
            RAAHI_LOGE(TAG, "info json string larger than stipulated size of %u", json_str_len);
            break;
        }
    }
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

	RAAHI_LOGI(TAG, "Client ID: %s", sysconfig.client_id);	
	RAAHI_LOGI(TAG, "Topic: %s", sysconfig.topic);
	RAAHI_LOGI(TAG, "Apn: %s", sysconfig.apn);
	
	RAAHI_LOGI(TAG, "*************************************************");
}

/* -----------------------------------------------------------
| 	read_zombie_info()
| 	We store all the debug info we want to survive a restart
| 	into a struct which is then stored into a file in the  
|   SPIFFS. We then retrieve it on every boot
------------------------------------------------------------*/
void read_zombie_info()
{
	static const char* zombie_file_name = "/spiffs/zombie_info.txt";
	FILE* zombie_file = NULL;
	size_t content_size;
    
	zombie_file = fopen(zombie_file_name, "rb");
	if (zombie_file == NULL) 
    { // If zombie file isnt' present, fill zombie struct with default values
        zombie_info.esp_restart_reason[0] = '\0';
    }
    else
    { // If zombie file is present, copy its values to the structure
        
	    if((content_size = fread(&zombie_info, sizeof(zombie_info_struct), 1, zombie_file)) != 1) {
	    	ESP_LOGE(TAG, "Couldn't read zombie info file contents");
	    	abort();
	    } else {
	    	fclose(zombie_file);
	    }
     } 
}


/* -----------------------------------------------------------
| 	write_zombie_info()
| 	We store all the debug info we want to survive a restart
| 	into a struct which is then stored into a file in the  
|   SPIFFS. We then retrieve it on every boot
------------------------------------------------------------*/
void write_zombie_info()
{
	static const char* zombie_file_name = "/spiffs/zombie_info.txt";
	FILE* zombie_file = NULL;
    
	zombie_file = fopen(zombie_file_name, "wb");
	if (fwrite(&zombie_info, sizeof(zombie_info_struct), 1, zombie_file) != 1) {
		ESP_LOGE(TAG, "Couldn't write to Spiffs file zombie_info.txt");
		fclose(zombie_file);
		abort();
	} else {
		fclose(zombie_file);
	}
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
	
	default_config.analog_sensor_type[0] = NONE;
	default_config.analog_sensor_type[1] = NONE;
	default_config.analog_sensor_type[2] = NONE;
	default_config.analog_sensor_type[3] = NONE;
	
	strcpy(default_config.client_id, "raahi_new");
	strcpy(default_config.topic, CONFIG_MQTT_TOPIC_ROOT);
	strcpy(default_config.apn, CONFIG_EXAMPLE_MODEM_APN);

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
		fseek(config_file, 0, SEEK_END);
		unsigned int file_size = ftell(config_file);
		fseek(config_file, 0, SEEK_SET);

		printf("File Size: %u\n", file_size);
		printf("Sysconfig Size: %u\n",  sizeof(struct config_struct));

		if (file_size != sizeof(struct config_struct)) // If sysconfig file in the flash is old
		{
			fclose(config_file);
			
			ESP_LOGI(TAG, "Config file belonged to old firmware version. Overwriting it with default values of the new firmware");
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
		}
		else
		{
			if((content_size = fread(&sysconfig, sizeof(struct config_struct), 1, config_file)) != 1) {
				ESP_LOGE(TAG, "Couldn't read sysconfig.txt contents");
				abort();
			} else {
				ESP_LOGI(TAG, "Successfully read sysconfig.txt contents into internal variable");
				// Display what was read
				display_sysconfig();
				fclose(config_file);
			}
		}
	}
}
uint8_t parseJson(char* json_str, uint16_t json_str_len, struct json_struct* parsed_result)
{
    uint16_t char_count = 0;
    uint8_t item_count = 0;
    bool key_s_turn = true;
    uint8_t local_char_idx = 0;

    while (item_count < MAX_SUBSCRIBE_JSON_ITEMS && char_count < json_str_len)
    {
        // First parse the key
        while(char_count < json_str_len) // Move to the first double quote
        {
            if (json_str[char_count] == '\"') {
                char_count++;
                break;
            }
            
            char_count++;
        }

        if (key_s_turn == true) // If it is the key's turn to be parsed
        {
            while(char_count < json_str_len)
            {

                if (json_str[char_count] == '\"') {
                    parsed_result[item_count].key[local_char_idx] = '\0';
                    char_count++;
                    key_s_turn = false;
                    local_char_idx = 0;
                    break;
                }
            
                parsed_result[item_count].key[local_char_idx] = json_str[char_count];
                local_char_idx++;
                if (local_char_idx >= MAX_KEY_LEN) 
                {
                    RAAHI_LOGE(TAG, "Json parse error. Key too long");
                    return(item_count);
                } 
                char_count++;
            }
        
        } // Parsing keys loop 

        else // It is the value's turn to be parsed
        {
            while(char_count < json_str_len)
            {

                if (json_str[char_count] == '\"') {
                    parsed_result[item_count].value[local_char_idx] = '\0';
                    char_count++;
                    key_s_turn = true;
                    local_char_idx = 0;
                    item_count++;
                    break;
                }
            
                parsed_result[item_count].value[local_char_idx] = json_str[char_count];
                local_char_idx++;
                if (local_char_idx >= MAX_VALUE_LEN) 
                {
                    RAAHI_LOGE(TAG, "Json parse error. Value too long");
                    return(item_count);
                } 
                char_count++;
            }
        } // Parsing values loop

    } // Iteration through items  
  
    return(item_count);
} // End of parse_Json function

void sysconfig_json_write(struct json_struct* parsed_json, uint8_t no_of_items)
{
    uint8_t item_idx;
    int32_t tmpVal;
	static const char* config_file_name = "/spiffs/sysconfig.txt";
	FILE* config_file = NULL;
    bool client_id_updated = false;

    for(item_idx = 1; item_idx < no_of_items; item_idx++) // We are starting from 1 because the 0th index contained type info that has already been parsed
    {
        if(strcmp(parsed_json[item_idx].key, "first_slave_id") == 0)
        {
	        if((tmpVal = str2num(parsed_json[item_idx].value, '\0', 4)) >= 0) { // Update only if the value is not negative (indicating conversion error)
		        sysconfig.slave_id[0] = (uint8_t)tmpVal;
	        }
        }
        else if(strcmp(parsed_json[item_idx].key, "second_slave_id") == 0)
        {
	        if((tmpVal = str2num(parsed_json[item_idx].value, '\0', 4)) >= 0) {
		        sysconfig.slave_id[1] = (uint8_t)tmpVal;
	        }
        }
        else if(strcmp(parsed_json[item_idx].key, "first_reg_address") == 0)
        {
	        if((tmpVal = str2num(parsed_json[item_idx].value, '\0', 6)) >= 0) {
		        sysconfig.reg_address[0] = (uint16_t)tmpVal;
	        }
        }
        else if(strcmp(parsed_json[item_idx].key, "second_reg_address") == 0)
        {
	        if((tmpVal = str2num(parsed_json[item_idx].value, '\0', 6)) >= 0) {
		        sysconfig.reg_address[1] = (uint16_t)tmpVal;
	        }
        }
        else if(strcmp(parsed_json[item_idx].key, "third_reg_address") == 0)
        {
	        if((tmpVal = str2num(parsed_json[item_idx].value, '\0', 6)) >= 0) {
		        sysconfig.reg_address[2] = (uint16_t)tmpVal;
	        }
        }
        else if(strcmp(parsed_json[item_idx].key, "sampling_period_in_sec") == 0)
        {
	        if((tmpVal = str2num(parsed_json[item_idx].value, '\0', 4)) >= 0) {
		        sysconfig.sampling_period_in_sec = (uint8_t)tmpVal;
	        }
        }
        else if(strcmp(parsed_json[item_idx].key, "client_id") == 0)
        {
            strcpy(sysconfig.client_id,  parsed_json[item_idx].value);
            client_id_updated = true;
        }
        else if(strcmp(parsed_json[item_idx].key, "topic") == 0)
        {
            strcpy(sysconfig.topic,  parsed_json[item_idx].value);
        }
        else if(strcmp(parsed_json[item_idx].key, "apn") == 0)
        {
            strcpy(sysconfig.apn,  parsed_json[item_idx].value);
        }
        else // Error check
        {
            RAAHI_LOGE(TAG, "Unknown key encountered. key = %s", parsed_json[item_idx].key);
        }
    }

    if(no_of_items > 1) // Just a protection against a json that only has type field
    {
        display_sysconfig(); // so that we will know in AWS if the changes have indeed taken place
	    config_file = fopen(config_file_name, "wb");

	    if(fwrite(&sysconfig, sizeof(struct config_struct), 1, config_file) != 1) {
	    	RAAHI_LOGE(TAG, "Couldn't update sysconfig file although it is present");
	    	abort();
	    } else {
	    	RAAHI_LOGI(TAG, "Successfully updated sysconfig file");
	    }
	    fclose(config_file);

	    if(client_id_updated == true) {
	    	ESP_LOGI(TAG, "Client ID updated. So restarting");
            strcpy(zombie_info.esp_restart_reason, "Client ID Updated (Util)");
           	vTaskDelay(5000 / portTICK_RATE_MS);
	    	raahi_restart();
	    }
    }

}

void create_sysconfig_json(char* json_str, uint16_t json_str_len)
{
    struct json_struct sysconfig_json[MAX_SUBSCRIBE_JSON_ITEMS];
	uint8_t item, no_of_items = 0;
	uint16_t running_str_len;
	char tempStr[MAX_KEY_LEN + MAX_VALUE_LEN];  
  	time_t now;

	time(&now);

	sprintf(sysconfig_json[no_of_items].key, "\"deviceId\"");
	sprintf(sysconfig_json[no_of_items].value, "\"%s\"", user_mqtt_str);
	no_of_items++;

	sprintf(sysconfig_json[no_of_items].key, "\"timestamp\"");
	sprintf(sysconfig_json[no_of_items].value, "%lu", now);
	no_of_items++;
	
	sprintf(sysconfig_json[no_of_items].key, "\"first_slave_id\"");
	sprintf(sysconfig_json[no_of_items].value, "%u", sysconfig.slave_id[0]);
	no_of_items++;

	sprintf(sysconfig_json[no_of_items].key, "\"second_slave_id\"");
	sprintf(sysconfig_json[no_of_items].value, "%u", sysconfig.slave_id[1]);
	no_of_items++;

	sprintf(sysconfig_json[no_of_items].key, "\"first_reg_address\"");
	sprintf(sysconfig_json[no_of_items].value, "%u", sysconfig.reg_address[0]);
	no_of_items++;
	
	sprintf(sysconfig_json[no_of_items].key, "\"second_reg_address\"");
	sprintf(sysconfig_json[no_of_items].value, "%u", sysconfig.reg_address[1]);
	no_of_items++;
	
	sprintf(sysconfig_json[no_of_items].key, "\"third_reg_address\"");
	sprintf(sysconfig_json[no_of_items].value, "%u", sysconfig.reg_address[2]);
	no_of_items++;

	sprintf(sysconfig_json[no_of_items].key, "\"sampling_period_in_sec\"");
	sprintf(sysconfig_json[no_of_items].value, "%u", sysconfig.sampling_period_in_sec);
	no_of_items++;
	
	sprintf(sysconfig_json[no_of_items].key, "\"client_id\"");
	sprintf(sysconfig_json[no_of_items].value, "\"%s\"", sysconfig.client_id);
	no_of_items++;
	
	sprintf(sysconfig_json[no_of_items].key, "\"topic\"");
	sprintf(sysconfig_json[no_of_items].value, "\"%s\"", sysconfig.topic);
	no_of_items++;
	
	sprintf(sysconfig_json[no_of_items].key, "\"apn\"");
	sprintf(sysconfig_json[no_of_items].value, "\"%s\"", sysconfig.apn);
	no_of_items++;
	
	json_str[0] = '\0';
	strcat(json_str, "{");
	running_str_len = strlen("{"); 
	for (item = 0; item < no_of_items; item++)
	{
		tempStr[0] = '\0';
		strcat(tempStr, sysconfig_json[item].key);
		strcat(tempStr, ": ");
		strcat(tempStr, sysconfig_json[item].value);

		if (item < no_of_items - 1)
		{
			strcat(tempStr, ",");
		}
		else
		{
			strcat(tempStr, "}");
		}

		if(running_str_len + strlen(tempStr) < json_str_len)
		{
			running_str_len = running_str_len + strlen(tempStr);
			strcat(json_str, tempStr);
		}
		else
		{
			RAAHI_LOGE(TAG, "sysconfig json string larger than stipulated size of %u", json_str_len);
			break;
		}
	}
}



void getMacAddress(char* macAddress) {
	uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	
	sprintf(macAddress, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	ESP_LOGI(TAG, "mac address is %s", macAddress);
}

