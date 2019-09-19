
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"

#include "raahi.h"
#include "esp32/rom/crc.h"

#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */
#define MAX_CRC32_HTTP_TRIES 5
#define MAX_OTA_FRAGMENTS 20
#define MAX_URL_LEN 100
#define MAX_HTTP_TRIES 5

static const char *TAG = "fragmented_ota";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

typedef struct
{
    char fw_ver[32];
    uint16_t no_of_fragments;
    uint16_t max_fragment_size;
    char fragments_root_url[MAX_URL_LEN];
    char crc32_url[MAX_URL_LEN];
}ota_header_t;

typedef struct
{
    char fw_ver_being_updated[32];
    uint16_t fragments_completed;
    const esp_partition_t *part;
    uint32_t wrote_size;
}ota_record_t;

extern uint8_t fragmented_ota_error_counter;

static esp_err_t prepare_for_ota(int data_read, ota_record_t* ota_record);
static void read_flash_ota_record(ota_record_t* ota_record);
static void update_flash_ota_record(ota_record_t* ota_record);
static void get_ota_header(esp_http_client_config_t url_config, ota_header_t* ota_header, uint16_t storage_size); 
static void get_ota_crc32(esp_http_client_config_t url_config, uint32_t* ota_crc32, uint16_t storage_size); 
static esp_err_t transfer_url_contents_to_local(esp_http_client_config_t url_config, char* local_var, uint16_t storage_size); 

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}

void print_ota_header(ota_header_t* ota_header)
{
    printf("fw_ver: %s\n", ota_header->fw_ver);
    printf("no_of_fragments: %u\n", ota_header->no_of_fragments);
    printf("max_fragment_size: %u\n", ota_header->max_fragment_size);
    printf("fragments_root_url: %s\n", ota_header->fragments_root_url);
    printf("crc32_url: %s\n", ota_header->crc32_url);
}


/* -----------------------------------------------------------
| ota_by_fragments
|   This is the main function for fragmented OTA update. It
|   interacts with the designated http server to obtain firmware
|   update header, ensure that the firmware is a newer one than
|   what is currently running, get one fragment of the new fw
|   at a time and maintain records in flash so that the update
|   can continue even if there is a power recycle or an abort.
|   Finally, when all the fragments have been received, it 
|   checks if the SHA is correct, changes the fw entry point to
|   the new firmware and restarts ESP
------------------------------------------------------------*/
void ota_by_fragments(void *pvParameter)
{
    esp_err_t err;

    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *running;
    
    ota_header_t ota_header; 
    ota_record_t ota_record;

    esp_http_client_config_t header_url_config, crc32_url_config, fragment_url_config;
    esp_http_client_handle_t client;
    char url_temp[MAX_URL_LEN];
    int data_read;
    uint32_t crc32_original[MAX_OTA_FRAGMENTS];
    char fragments_completed_str[4]; // 1 element for '_' char and 3 for storing the number of fragments completed. So this can support upto 999 fragments.

    // Get running partition info and the next update partition
    running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    ESP_LOGI(TAG, "Starting Fragmented OTA example");
    
    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "FW will be written to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);
    
    const esp_partition_pos_t part_pos = { 
      .offset = update_partition->address,
      .size = update_partition->size,
    };  

    // Get the fragmented OTA header that has info about new firmware version, number of fragments etc. 
    header_url_config = (esp_http_client_config_t) {
        .url = (const char*)CONFIG_OTA_HEADER_URL,
        .cert_pem = (char *)server_cert_pem_start,
    };
    get_ota_header(header_url_config, &ota_header, sizeof(ota_header_t));
    print_ota_header(&ota_header);

    // Check if a ota record file already exists. If it does, populates local structure. Otherwise creates one with initial values
    strcpy(ota_record.fw_ver_being_updated, ota_header.fw_ver); //memcpy is less error (ovrflow) prone than strcpy
    ota_record.fragments_completed = 0;
    ota_record.part = update_partition;   
    ota_record.wrote_size = 0;
    read_flash_ota_record(&ota_record);
    if (ota_record.fragments_completed >= ota_header.no_of_fragments)
    {
        ESP_LOGE(TAG, "Fragments completed is greater than or equal to the number of fragments specified in the header");
        remove(OTA_RECORD_FILE_NAME);
        task_fatal_error();
    }

    crc32_url_config = (esp_http_client_config_t) {
        .url = (const char*) ota_header.crc32_url,
        .cert_pem = (char *)server_cert_pem_start,
    };
    
    // Get the CRC32 checksums
    get_ota_crc32(crc32_url_config, crc32_original, ota_header.no_of_fragments*sizeof(crc32_original[0]));

    bool partition_readied = false;
    uint32_t fragment_crc32 = 0;
    while(1) // Loop until failure counters overflow and monitoring task restarts ESP
    {
        // Always check if there already have been too many errors before proceeding to next iteration
        if (fragmented_ota_error_counter > MAX_OTA_FAIL_COUNT)
        {
            ESP_LOGE(TAG, "Too many fragmented OTA update errors. Quitting OTA update");
            remove(OTA_RECORD_FILE_NAME);
            task_fatal_error();
        }
 
        // Get header in every iteration to ensure that FW hasn't changed in the mean time
        get_ota_header(header_url_config, &ota_header, sizeof(ota_header_t));
       
        // If firmware has indeed changed, then we need to start from the beginning 
        if (strcmp(ota_record.fw_ver_being_updated, ota_header.fw_ver) != 0) // FW version mismatch. Restart with new FW
        { // The only difference between what we are doing here and what we did before the beginning of the infinite while loop is that,
          // we don't have access flash unnecessarily. This will improve flash lifetime
            remove(OTA_RECORD_FILE_NAME);
            task_fatal_error(); 
        } 
      
        strcpy(url_temp, ota_header.fragments_root_url);
        sprintf(fragments_completed_str, "_%02hu", ota_record.fragments_completed + 1); // Fragment numbering will start from 1
        strcat(url_temp, fragments_completed_str);

        // Proceed with OTA for a given fragment        
        fragment_url_config = (esp_http_client_config_t) { 
            .url = (const char*) url_temp,
            .cert_pem = (char *)server_cert_pem_start,
        };
        client = esp_http_client_init(&fragment_url_config);
        if (client == NULL) {
            ESP_LOGE(TAG, "Failed to initialise HTTP connection to %s", url_temp);
            http_cleanup(client);
            fragmented_ota_error_counter++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
            http_cleanup(client);
            fragmented_ota_error_counter++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        esp_http_client_fetch_headers(client);

        // Read data from the URL and write to OTA partition
        while((data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE)) > 0 )
        {
            if(ota_record.fragments_completed == 0 &&  partition_readied == false) // Do error checks and begin OTA
            {
                err = prepare_for_ota(data_read, &ota_record);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Fatal error: Erasing designated partition failed. Partition Subtype: %d", update_partition->subtype); 
                    task_fatal_error(); 
                }
                partition_readied = true;
            }
            
            err = esp_partition_write(ota_record.part, ota_record.wrote_size, (const uint8_t*) ota_write_data, data_read);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Fatal error: Writing into designated partition failed. Partition Subtype: %d", update_partition->subtype); 
                http_cleanup(client);
                task_fatal_error();
            }
            ota_record.wrote_size += data_read;
            ESP_LOGD(TAG, "Written so far %d bytes", ota_record.wrote_size);
            fragment_crc32 = crc32_le(fragment_crc32, (const uint8_t*)ota_write_data, data_read);
        }
        if (data_read < 0)
        {
            ESP_LOGE(TAG, "Error: SSL data read error");
            fragmented_ota_error_counter++;
            continue;
        }    
        
        // One fragment received successfully and written to OTA partition. Now make sure it isnt corrup 
        http_cleanup(client);
        if (fragment_crc32 != crc32_original[ota_record.fragments_completed]) // Checksum failed
        {
            ESP_LOGE(TAG, "Checksum error on fragment number %u. Expected %#06X. Got %#06X.", ota_record.fragments_completed + 1, fragment_crc32, crc32_original[ota_record.fragments_completed]);            
            fragment_crc32 = 0;
        }

        // Update our records
        fragment_crc32 = 0;
        ota_record.fragments_completed++;
        ESP_LOGI(TAG, "Successfully completed %u fragment(s) out of %u fragments", ota_record.fragments_completed, ota_header.no_of_fragments);
        update_flash_ota_record(&ota_record);

        // Check if we have all the fragments
        if (ota_record.fragments_completed == ota_header.no_of_fragments) // it means we are done
        {
            esp_image_metadata_t metadata;
       
            err = esp_image_verify(ESP_IMAGE_VERIFY, &part_pos, &metadata);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_end failed!");
                remove(OTA_RECORD_FILE_NAME);
                task_fatal_error();
            }

            err = esp_ota_set_boot_partition(update_partition);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
                task_fatal_error(); 
            }
            ESP_LOGI(TAG, "Prepare to restart system!");
            remove(OTA_RECORD_FILE_NAME);
            esp_restart();
            return ;
        }
    } // End of infitie while loop
}

/* -----------------------------------------------------------
| get_ota_header 
|   Opens a url containing the header for fragmented OTA update
|   and copies the contents to a local header variable
------------------------------------------------------------*/
static void get_ota_header(esp_http_client_config_t url_config, ota_header_t* ota_header, uint16_t storage_size) 
{
    esp_err_t err;

    ESP_LOGI(TAG, "Getting the header");
    err = transfer_url_contents_to_local(url_config, (char*)ota_header, storage_size);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get the header");
        task_fatal_error();
    }
    if (ota_header->no_of_fragments > MAX_OTA_FRAGMENTS)
    {
        ESP_LOGE(TAG, "Number of OTA fragments more than what this code can handle");
        task_fatal_error();
    }
    
}

/* -----------------------------------------------------------
| get_ota_crc32
|   Opens a url containing the crc32s for fragmented OTA update
|   and copies the contents to a local variable
------------------------------------------------------------*/
static void get_ota_crc32(esp_http_client_config_t url_config, uint32_t* ota_crc32, uint16_t storage_size) 
{
    esp_err_t err;

    ESP_LOGI(TAG, "Getting the CRC32 file");
    err = transfer_url_contents_to_local(url_config, (char *)ota_crc32, storage_size);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get the CRC32 values");
        task_fatal_error(); 
    }

}

/* -----------------------------------------------------------
| transfer_url_contents_to_local 
|   Opens a url and copies the contents to a local variable
------------------------------------------------------------*/
static esp_err_t transfer_url_contents_to_local(esp_http_client_config_t url_config, char* local_var, uint16_t storage_size) 
{
    uint8_t try_count;
    char* temp_data;
    esp_http_client_handle_t client;
    esp_err_t err;
    uint16_t data_idx;
    int data_read;

    client = esp_http_client_init(&url_config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection to %s", url_config.url);
        fragmented_ota_error_counter++;
        return(ESP_FAIL); 
    }
      
    for(try_count = 0; try_count < MAX_HTTP_TRIES; try_count++)
    {
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open HTTP connection to %s due to %s", url_config.url, esp_err_to_name(err));
            fragmented_ota_error_counter++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        else
        {
            break;
        }
    }
    if (try_count == MAX_HTTP_TRIES)
    {
        ESP_LOGE(TAG, "Exhausted number of http tries");
        http_cleanup(client); 
        return(ESP_FAIL);
    }

    // We have successfully opened the http connection
    esp_http_client_fetch_headers(client);
    data_idx = 0;
    temp_data = (char *)calloc(storage_size, sizeof(char));
    while(data_idx < storage_size)
    {
        data_read = esp_http_client_read(client, temp_data, storage_size); 
        if (data_read < 0)
        {
            ESP_LOGE(TAG, "Error: SSL data read error while accessing url values");
            free(temp_data);
            http_cleanup(client);
            return(ESP_FAIL);
        }
        else if (data_read == 0)
        {
            break;
        }    
        else if (data_read > storage_size - data_idx)
        {
            ESP_LOGE(TAG, "More data read than what local variable can accommodte");
            free(temp_data);
            http_cleanup(client);
            return(ESP_FAIL);
        } 
        
        memcpy(&local_var[data_idx], temp_data, data_read);  
        data_idx = data_idx + data_read;
    }
    if (data_idx != storage_size)
    {
        ESP_LOGE(TAG, "Data sent by url is %u bytes, which is less than expected %u bytes", data_idx, storage_size);
        free(temp_data);
        http_cleanup(client);
        return(ESP_FAIL);
    }
    
    free(temp_data);
    http_cleanup(client);
    return(ESP_OK); 
}
  
/* -----------------------------------------------------------
| prepare_for_ota
|   Does some error checks and initializes ota related params
------------------------------------------------------------*/
static esp_err_t prepare_for_ota(int data_read, ota_record_t* ota_record)
{
    esp_app_desc_t new_app_info;
    esp_err_t err;

    if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {

        if(ESP_IMAGE_HEADER_MAGIC != ota_write_data[0])
        {
            ESP_LOGE(TAG, "OTA image has invalid magic byte (expected 0xE9, saw 0x%02x", ota_write_data[0]);
            return(ESP_FAIL);
        }
        // check current version with downloading
        memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
        ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

        if (strcmp(new_app_info.version, ota_record->fw_ver_being_updated) != 0)
        {
            ESP_LOGE(TAG, "FW image has a different FW version than that was mentioned in the OTA header"); 
            return(ESP_FAIL);
        }

        err = esp_partition_erase_range(ota_record->part, 0, ota_record->part->size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed. Partition couldn't be erased");
            return(ESP_FAIL);
        }
        
        ESP_LOGI(TAG, "esp_ota_begin succeeded");
        return(ESP_OK);
    } else {
        ESP_LOGE(TAG, "received package is not fit len");
        return(ESP_FAIL);
    }
}

/* -----------------------------------------------------------
| read_flash_ota_record
| 	Checks if there is indeed a OTA record in the flash. If not
|   creates one with default values as supplied by the input. 
|   If the record does exist in the flash, overwrites updates  
|   input with values from the flash.
|      
|   Additionally, it has a correcting mechanism which checks
|   if the size of the input structure is different from the 
|   one in the flash (if it exists), indicating that the one
|   in the flash is from a different firmware. If it is an 
|   older one, it overwrites it with default values supplied
|   by the input.  	
------------------------------------------------------------*/
static void read_flash_ota_record(ota_record_t* ota_record)
{
	FILE* ota_record_file = NULL;
    ota_record_t ota_record_temp;
    
    ota_record_file = fopen(OTA_RECORD_FILE_NAME, "rb");
    if (ota_record_file == NULL) { 
    	ESP_LOGI(TAG, "OTA record file not present. Creating one with default values");
        update_flash_ota_record(ota_record);
    } else { // OTA record file is present. So populate input ota_record structure with its values
		
		fseek(ota_record_file, 0, SEEK_END);
		unsigned int file_size = ftell(ota_record_file);
		fseek(ota_record_file, 0, SEEK_SET);

		if (file_size != sizeof(ota_record_t)) // If ota file in the flash is old
		{
			fclose(ota_record_file);
			ESP_LOGI(TAG, "Config file belonged to old firmware version. Overwriting it with default values of the new firmware");
            update_flash_ota_record(ota_record);
		}
        else
        {
            if(fread(&ota_record_temp, sizeof(ota_record_t), 1, ota_record_file) != 1) 
            {
		    	ESP_LOGE(TAG, "Couldn't read OTA record file contents");
		    	fclose(ota_record_file);
                abort();
		    } 
            else 
            {
                if ( strcmp(ota_record_temp.fw_ver_being_updated, ota_record->fw_ver_being_updated) != 0)
                {
			        fclose(ota_record_file);
    	            ESP_LOGI(TAG, "OTA record file had a different FW version that one being updated. Creating new file default values");
                    update_flash_ota_record(ota_record);
                }
                else if (ota_record_temp.part != ota_record->part)
                {
			        fclose(ota_record_file);
    	            ESP_LOGI(TAG, "OTA record file had a different partition than one being updated. Creating new file default values");
                    update_flash_ota_record(ota_record);
                }
                else
                {
                    *ota_record = ota_record_temp;
		    	    ESP_LOGI(TAG, "Successfully read OTA record file contents into internal variable");
			        fclose(ota_record_file);
                }
		    }
        }
	} // End of else (if (ota_record_file == NULL))
}


/* -----------------------------------------------------------
| update_flash_ota_record
|   Creates a ota record file in the flash with supplied input
|   values
------------------------------------------------------------*/
static void update_flash_ota_record(ota_record_t* ota_record)
{
	FILE* ota_record_file = NULL;
    
    ota_record_file = fopen(OTA_RECORD_FILE_NAME, "wb");

    if (ota_record_file == NULL)
    {
        ESP_LOGE(TAG, "Could not create %s", OTA_RECORD_FILE_NAME);
        abort();
    }
    
    if (fwrite(ota_record, sizeof(ota_record_t), 1, ota_record_file) != 1) {
    	ESP_LOGE(TAG, "Couldn't write to Spiffs ota file");
    	fclose(ota_record_file);
    	abort();
    } else {
    	ESP_LOGI(TAG, "OTA file successfully updated");
    	fclose(ota_record_file);
    } 
}

