/**************************************************************
* Raahi firmware main code. 
* OVERVIEW:
*	1. Displays current firmware version and some general info
* 	2. Reads config GPIO
* 	3. If it is set, carries out OTA related tasks. They are
*	   present in this file itself
* 	4. If config GPIO is not set, carries out normal tasks. They
*	   are in normal_tasks.c
*
* IMPORTANT NOTE:
* 	Try as much as possible to never modify the code in this 
*   file. The idea is that, no matter the firmware version, 
* 	this code is expected to remain the same. The objective of
* 	this code is to remain minimal, focused on OTA and well 
*	tested. The reason why we don't want to change this code is
* 	that, if the OTA part gets messed up, the device will be 
*   bricked and no more firmware updates will be possible. So 
*	with every firmware release, we will have to to a filecmp
*	of this file with that of its previous firmware version and
*	release the firmware only if there are no changes. 
**************************************************************/

// Include necessary header files
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_spiffs.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_system.h"
#include <esp_http_server.h>
#include "raahi.h"
#include "driver/gpio.h"

// Defines
#define GPIO_CONFIG_PIN_TX GPIO_NUM_18 // TODO: Move this to sdkconfig
#define GPIO_CONFIG_PIN_RX GPIO_NUM_19 // TODO: Move this to sdkconfig
#define GPIO_STATUS_PIN_0 32 // RGB LED RED 1:OFF 0:ON
#define GPIO_STATUS_PIN_1 27 // RGB LED GREEN 1: OFF ,0:ON
#define GPIO_STATUS_PIN_2 26 // RGB LED BLUE 1: OFF, 0: ON
#define GPIO_STATUS_PIN_3 25 // RED power LED 1:ON , 0:OFF 
#define POST_BUF_SIZE (uint16_t)512 
#define MIN(a,b) ((a) < (b) ? (a) : (b))

// Global variables
static const char *TAG = "raahi_main";
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static char buf[POST_BUF_SIZE + 1] = { 0 };
extern struct debug_data_struct debug_data;
// Function declarations
void normal_tasks(void);

/********************************************************************/
// Function Definitions


/* -----------------------------------------------------------
| 	inir_spiffs()
|	  
------------------------------------------------------------*/
static esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,   // This decides the maximum number of files that can be created on the storage
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

/* -----------------------------------------------------------
| 	ota_homepage_get_handler()
|	HTTP server side handler for GET requests on / when in OTA
|   mode. 
------------------------------------------------------------*/
static esp_err_t ota_homepage_get_handler(httpd_req_t *req)
{
     /* Get handle to embedded file upload script */
    extern const unsigned char homepage_start[] asm("_binary_ota_index_html_start");
    extern const unsigned char homepage_end[]   asm("_binary_ota_index_html_end");
    const size_t homepage_size = (homepage_end - homepage_start);
	
	//httpd_resp_set_type(req, "text/html"); 
    httpd_resp_send_chunk(req, (const char *)homepage_start, homepage_size);
    httpd_resp_sendstr_chunk(req, NULL);
    ESP_LOGI(TAG, "Raahi http get handler");
	
	return(ESP_OK);
}

static const httpd_uri_t ota_homepage = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = ota_homepage_get_handler,
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
| 	task_fatal_error()
|	Exclusively used by submit post handler to indicate that
|	OTA has failed. 
------------------------------------------------------------*/
static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}

/* -----------------------------------------------------------
| 	submit_post_handler()
|	HTTP handler for firware update
------------------------------------------------------------*/
static esp_err_t submit_post_handler(httpd_req_t *req)
{
	// TODO: 	1. Change generic types like int to types with size specified like uint16_t
    //			2. Add Error checks everywhere
    esp_err_t err;
    ESP_LOGI(TAG, "Raahi http post handler");
	printf("In post handler\n");

	int tmpIndex;
	
	char *content_type_str, *boundary_str, *temp_str;
	char* payload = NULL;
    int ret, remaining = req->content_len, content_type_len;
    bool first_boundary_found = false;
	int last_boundary_str_len, first_payload_end_pos, payload_size = 0;
    
    esp_ota_handle_t update_handle = 0 ;
	const esp_partition_t *update_partition = NULL;
    const esp_partition_t *running = esp_ota_get_running_partition();
    int binary_file_length = 0;
    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

	// TODO: Check if new fw size is greater than 1M
		
	printf("\n-------------- Receiving new firmware -------------\n");
	printf("Content Length = %d\n", remaining);

	// Read the header to find out the boundary string
	content_type_len = httpd_req_get_hdr_value_len(req, "Content-Type") + 1;
	printf("Content Type Length: %d\n", content_type_len);
	
	content_type_str = malloc(content_type_len);
    if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type_str, content_type_len) == ESP_OK) {
        //ESP_LOGI(TAG, "Found header => Content-Type: %s", content_type_str);
		printf("Content-Type:%s\n", content_type_str);
    }
    boundary_str = strstr(content_type_str, "boundary=") + strlen("boundary=");
	printf("Boundary=%s\n", boundary_str);
	
	last_boundary_str_len = strlen(boundary_str) + 4;

    while (remaining > 0) {
        // Read the data for the request 
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, POST_BUF_SIZE))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                // Retry receiving if timeout occurred 
                continue;
            }
    		
			free(content_type_str);
            return(ESP_FAIL);
        }

		remaining -= ret;

		//printf("Packet Count: %d\t ret val: %d\n", packet_count, ret);

		// Examine buffer for data starting point
		if (first_boundary_found == false) {
			if(strstr(buf, boundary_str) != NULL) {
				printf("First boundary found\n");
				temp_str = strstr(buf, boundary_str) + strlen(boundary_str); 
				first_boundary_found = true;
				payload = strstr(temp_str, "Content-Type: application/octet-stream") + strlen("Content-Type: application/octet-stream") + 4; //+4 for \r\n\r\n
				if (payload == NULL) {
					ESP_LOGE(TAG, "Content-Type not found or it isn't application/octet-stream");
    				free(content_type_str);
					return(ESP_FAIL);
				}
				first_payload_end_pos = ret - (int)(payload - buf);   
				printf("First payload_end_pos = %d\n", first_payload_end_pos);
				payload_size = first_payload_end_pos;
				payload[payload_size] = '\0';
				
				for(tmpIndex = 0; tmpIndex < (first_payload_end_pos/4); tmpIndex++)
				{
					printf("0x%.2hhX ", payload[tmpIndex]);
				}
				printf("\n");

				// First payload is ready to be processed. First check the new firmware version
                if ( payload_size > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    // check current version with downloading
                	esp_app_desc_t new_app_info;
                    memcpy(&new_app_info, &payload[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }
                    
					// Begin the OTA process
					err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
    					free(content_type_str);
						return(ESP_FAIL);
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
				} else { // payload size is too small
					ESP_LOGE(TAG, "OTA begin function couldn't be carried out as OTA buffer is too small");
    				free(content_type_str);
					return(ESP_FAIL);
				}
			}
		} else {
			payload = buf;
	    
		    // Code related to eliminating bytes pertaining to the last boundary
			if (remaining > last_boundary_str_len) { // Still more bytes to be received before OTA can end
				payload_size = ret;
			} else if (remaining == last_boundary_str_len){ // Only the last boundary remains to be read. But we don't need to. So end OTA
				payload_size = ret;
				break;
			} else { // Implies that in the current payload, some bytes pertaining to the last boundary string needs to be eliminated
				payload_size = ret - (last_boundary_str_len - remaining);
				// Debug prints
				for(tmpIndex = 0; tmpIndex < payload_size; tmpIndex++)
				{
					printf("0x%.2hhX ", payload[tmpIndex]);
				}
				printf("\n");
			}


			payload[payload_size] = '\0';
		    // Print the buffer contents only if first boundary has been found
		    //printf("%s", payload);
		    
		} // else of if(first_boundary_found..
		
		// Write payload to the ota partition
		if (payload_size == 0 || payload == NULL) {// This is an additional just-in-case check
			ESP_LOGE(TAG, "Payload size is zero or payload pointing to NULL");
			task_fatal_error();
		}
        err = esp_ota_write( update_handle, (const void *)payload, payload_size);
        if (err != ESP_OK) {
            task_fatal_error();
        }
        binary_file_length += payload_size;
        ESP_LOGD(TAG, "Written image length %d", binary_file_length);

    }// End of While loop
    
	ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        task_fatal_error();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();

	httpd_resp_send(req, NULL, 0);
    free(content_type_str);
	
	return(ESP_OK);
}

static const httpd_uri_t submit = {
    .uri       = "/submit",
    .method    = HTTP_POST,
    .handler   = submit_post_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "Thank You"
};


/* -----------------------------------------------------------
| 	factory_reset_handler()
|	Makes factory partition as the boot partition
------------------------------------------------------------*/
static esp_err_t factory_reset_handler(httpd_req_t *req)
{
    esp_err_t err;
    const esp_partition_t *factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
                                                               ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);

    err = esp_ota_set_boot_partition(factory);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
    } // TODO: Should we add IMAGE_VERIFY?
	
	httpd_resp_send(req, NULL, 0);
	ESP_LOGI(TAG, "Factory reset successful");
    ESP_LOGI(TAG, "Restarting system in 3 seconds!");
    vTaskDelay(3000 / portTICK_RATE_MS);
    esp_restart();

	return ESP_OK;
}

static const httpd_uri_t factory_reset = {
    .uri       = "/factory_reset",
    .method    = HTTP_POST,
    .handler   = factory_reset_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};


/* -----------------------------------------------------------
| 	start_ota_webserver()
|	Starts HTTP server for OTA updates 
------------------------------------------------------------*/
static httpd_handle_t start_ota_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ota_homepage);
		httpd_register_uri_handler(server, &favicon_ico);
		httpd_register_uri_handler(server, &submit);
		httpd_register_uri_handler(server, &factory_reset);		
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}


/* -----------------------------------------------------------
| 	ota_task()
|	Starts a webserver, If a bin file is uploaded, stores it 
| 	into the next OTA partition available and make that 
| 	partition the boot partition  
------------------------------------------------------------*/
static void ota_task()
{
	// Get the homepage up: Initialize webserver, register all handlers
    start_ota_webserver();
    	 	
}

/* -----------------------------------------------------------
| 	init_config_gpio()
|	Initializes the config GPIO  
------------------------------------------------------------*/
void init_config_gpio()
{
    gpio_config_t gps_io_rx, gps_io_tx;
    gpio_config_t config_io_led_0;
    gpio_config_t config_io_led_1;
    gpio_config_t config_io_led_2;
    gpio_config_t config_io_led_3;
    //disable interrupt
    gps_io_rx.intr_type = GPIO_PIN_INTR_DISABLE;
    gps_io_tx.intr_type = GPIO_PIN_INTR_DISABLE;
    config_io_led_0.intr_type = GPIO_PIN_INTR_DISABLE;
    config_io_led_1.intr_type = GPIO_PIN_INTR_DISABLE;
    config_io_led_2.intr_type = GPIO_PIN_INTR_DISABLE;
    config_io_led_3.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as input mode
    gps_io_rx.mode = GPIO_MODE_INPUT;
    //set as output mode
    gps_io_tx.mode = GPIO_MODE_OUTPUT;
    config_io_led_0.mode = GPIO_MODE_OUTPUT;
    config_io_led_1.mode = GPIO_MODE_OUTPUT;
    config_io_led_2.mode = GPIO_MODE_OUTPUT;
    config_io_led_3.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    gps_io_rx.pin_bit_mask = (1ULL<<GPIO_CONFIG_PIN_RX);
    gps_io_tx.pin_bit_mask = (1ULL<<GPIO_CONFIG_PIN_TX);
    config_io_led_0.pin_bit_mask = (1ULL<<GPIO_STATUS_PIN_0);
    config_io_led_1.pin_bit_mask = (1ULL<<GPIO_STATUS_PIN_1);
    config_io_led_2.pin_bit_mask = (1ULL<<GPIO_STATUS_PIN_2);
    config_io_led_3.pin_bit_mask = (1ULL<<GPIO_STATUS_PIN_3);
    //enable/disable pull-down mode
    //config_io_rx.pull_down_en = 1;
    //config_io_tx.pull_down_en = 0;
    //disable pull-up mode
    //config_io_rx.pull_up_en = 0;
    //config_io_tx.pull_up_en = 1;
    config_io_led_0.pull_up_en = 1;
    config_io_led_1.pull_up_en = 1;
    config_io_led_2.pull_up_en = 1;
    config_io_led_3.pull_up_en = 0;
    config_io_led_0.pull_down_en = 0;
    config_io_led_1.pull_down_en = 0;
    config_io_led_2.pull_down_en = 0;
    config_io_led_3.pull_down_en = 1;
    //configure GPIO with the given settings
    gpio_config(&gps_io_rx);
    gpio_config(&gps_io_tx);
    gpio_config(&config_io_led_0);
    gpio_config(&config_io_led_1);
    gpio_config(&config_io_led_2);
    gpio_config(&config_io_led_3);
}


/* -----------------------------------------------------------
| 	wifi_event_handler()
|	  
------------------------------------------------------------*/
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

/* -----------------------------------------------------------
| 	wifi_init_softap()
|	  
------------------------------------------------------------*/
void wifi_init_softap()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 
        .ap = { 
            .ssid = CONFIG_WIFI_SSID,
            .ssid_len = strlen(CONFIG_WIFI_SSID),
            .password = CONFIG_WIFI_PASSWORD,
            .max_connection = CONFIG_LWIP_DHCPS_MAX_STATION_NUM,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };  
    if (strlen(CONFIG_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }   

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
             CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);

}

/**************************************************************
* Main function  
*
**************************************************************/
void app_main()
{
	// TODO: Display current firmware version
	init_config_gpio();

	// Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize file storage */
    ESP_ERROR_CHECK(init_spiffs());

    init_config_gpio();

	// Init WiFi soft AP
	wifi_init_softap();

    // Display the current firmware's version
	const esp_partition_t *running = esp_ota_get_running_partition();
	switch (running -> subtype) { // We will assume that there will never be more than two OTA partitions
		case ESP_PARTITION_SUBTYPE_APP_FACTORY:
			ESP_LOGI(TAG, "Running FW from factory partition");
			break;

		case ESP_PARTITION_SUBTYPE_APP_OTA_0:
			ESP_LOGI(TAG, "Running FW from OTA0 partition");
			break;

		case ESP_PARTITION_SUBTYPE_APP_OTA_1:
			ESP_LOGI(TAG, "Running FW from OTA1 partition");
			break;
		
		default: // We aren't supposed to be here!
			ESP_LOGE(TAG, "We aren't running FW from factory or any OTA partition! Something is very wrong");
			abort();
	}
		 
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }
    strcpy(debug_data.fw_ver, running_app_info.version);
    /* Set RGB Red LED */
    gpio_set_level(GPIO_STATUS_PIN_0,0);
    gpio_set_level(GPIO_STATUS_PIN_1,1);
    gpio_set_level(GPIO_STATUS_PIN_2,1);
    
    /* Set Power LED */
    gpio_set_level(GPIO_STATUS_PIN_3,1);
    normal_tasks();

}
