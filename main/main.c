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

//#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_spiffs.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include <esp_http_server.h>

#include "driver/gpio.h"

// Defines
#define GPIO_CONFIG_PIN GPIO_NUM_27 // TODO: Move this to sdkconfig

// Global variables
static const char *TAG = "raahi_main";
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

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
    static httpd_handle_t ota_http_server = NULL;
	
	// Get the homepage up: Initialize webserver, register all handlers
    ota_http_server = start_ota_webserver();
	 	
}

/* -----------------------------------------------------------
| 	init_config_gpio()
|	Initializes the config GPIO  
------------------------------------------------------------*/
void init_config_gpio()
{
    gpio_config_t config_io;
    //disable interrupt
    config_io.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    config_io.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    config_io.pin_bit_mask = (1ULL<<GPIO_CONFIG_PIN);
    //disable pull-down mode
    config_io.pull_down_en = 1;
    //disable pull-up mode
    config_io.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&config_io);
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
	
	// Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize file storage */
    ESP_ERROR_CHECK(init_spiffs());
    
	// Init WiFi soft AP
	wifi_init_softap();
    
	// Check the Config GPIO pin. 
    if(gpio_get_level(GPIO_CONFIG_PIN == 1)) {
		ESP_LOGI(TAG, "Config Pin set. Going into OTA mode\n");
		ota_task();
	} else {
		ESP_LOGI(TAG, "Config Pin set. Going into Normal mode\n");
		normal_tasks();
	}
}
