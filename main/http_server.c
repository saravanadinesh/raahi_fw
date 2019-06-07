/**************************************************************
* http_server.c 
* Contains all the code pertaining to running an HTTP server
* when in normal mode 
**************************************************************/
// Header files
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
//#include <unistd.h>
//#include <limits.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include <esp_http_server.h>
#include "raahi.h"

// Defines
#define FORM_DATA_BUF_SIZE 256


// External variables
static const char *TAG = "http_server";
extern struct config_struct sysconfig;

/* -----------------------------------------------------------
| 	homepage_get_handler()
|	HTTP server side handler for GET reuests on /
------------------------------------------------------------*/
static esp_err_t homepage_get_handler(httpd_req_t *req)
{
     /* Get handle to embedded file upload script */
    extern const unsigned char homepage_start[] asm("_binary_index_html_start");
    extern const unsigned char homepage_end[]   asm("_binary_index_html_end");
    const size_t homepage_size = (homepage_end - homepage_start);
	
	//httpd_resp_set_type(req, "text/html"); 
    httpd_resp_send_chunk(req, (const char *)homepage_start, homepage_size);
    httpd_resp_sendstr_chunk(req, NULL);
	
	return(ESP_OK);
}

static const httpd_uri_t homepage = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = homepage_get_handler,
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
| 	str2num()
| 	Converts a number in the form of a string to numerical data
| 	type. 
| 	WARNING: The numbers represented as characters are expected
|	to be decimal values. i.e., hex is not allowed
------------------------------------------------------------*/
static uint16_t str2num(char* input_str, const char delimiter, uint8_t max_parse_len)
{
	uint8_t index = 0;
	uint16_t result = 0; // TODO: Re-examine. 0 could be a legit input

	while(index < max_parse_len)
	{
		if(input_str[index] == delimiter) {
			break;
		}
		
		result = result*10 + (input_str[index] - 48);
		index++;
	}
	
	return result;
}

/* -----------------------------------------------------------
| 	update_sysconfig()
|	Parses the form string from HTTP client and updates the 
| 	sysconfig values (runtime values as well as in the 
| 	sysconfig.txt
------------------------------------------------------------*/
void update_sysconfig(char* form_str)
{
	char* tmpStr;
	uint8_t first_slave_id, second_slave_id;
	
	static const char* config_file_name = "/spiffs/sysconfig.txt";
	FILE* config_file = NULL;
	
	
	// Parse the received string to obtain sysconfig information
	tmpStr = strstr(form_str, "first_slave_id=") + strlen("first_slave_id=");
	first_slave_id = (uint8_t)str2num(tmpStr, '&', 4);

	tmpStr = strstr(form_str, "second_slave_id=") + strlen("second_slave_id=");
	second_slave_id = (uint8_t)str2num(tmpStr, '&', 4);

	// Debug prints
	printf("First slave id: %d\n", first_slave_id);
	printf("Second slave id: %d\n", second_slave_id);

	sysconfig.first_slave_id = first_slave_id;
	sysconfig.second_slave_id = second_slave_id;

	config_file = fopen(config_file_name, "rb");
	if (config_file == NULL) { // If config file isnt' present, create one with default values
		ESP_LOGE(TAG, "Config file not present. Can't update sysconfig!");
		abort();
	}
	
	fclose(config_file);
	config_file = fopen(config_file_name, "wb");

	if(fwrite(&sysconfig, sizeof(struct config_struct), 1, config_file) != 1) {
		ESP_LOGE(TAG, "Couldn't update sysconfig file although it is present");
		abort();
	} else {
		ESP_LOGI(TAG, "Successfully updated sysconfig file");
	}
	fclose(config_file);
}

/* -----------------------------------------------------------
| 	submit_post_handler()
|	HTTP server side handler for GET reuests on /
------------------------------------------------------------*/
static esp_err_t submit_post_handler(httpd_req_t *req)
{

	char buf[FORM_DATA_BUF_SIZE + 1];
	size_t ret, remaining = req->content_len;
	uint16_t content_pos = 0;    

	if (remaining > FORM_DATA_BUF_SIZE) {
		ESP_LOGE(TAG, "HTTP form data larger than internal receive buffer");
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Form data too long");
		return(ESP_FAIL);
	}
 
	// Read the data for the request
	while(remaining > 0)
	{
    	if ((ret = httpd_req_recv(req, &buf[content_pos],
    	                MIN(remaining, FORM_DATA_BUF_SIZE))) <= 0) {
    	    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
    	        // Retry receiving if timeout occurred 
    	        continue;
    	    }
    		
    	    return(ESP_FAIL);
    	}
		
		content_pos = content_pos + ret;
		remaining -= ret;
	}
	
	buf[FORM_DATA_BUF_SIZE]	= '\0'; // As a safety measure against pointer run away issues
	printf("Received string: %s\n", buf);

	update_sysconfig(buf);	

	httpd_resp_send(req, NULL, 0);	
	return(ESP_OK);

}

httpd_uri_t submit = {
    .uri       = "/submit",   // Match all URIs of type /upload/path/to/file
    .method    = HTTP_POST,
    .handler   = submit_post_handler,
    .user_ctx  = "Thanks for submitting the form"
};

/* -----------------------------------------------------------
| 	start_webserver()
|	Starts HTTP server for OTA updates 
------------------------------------------------------------*/
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &homepage);
		httpd_register_uri_handler(server, &favicon_ico);
		httpd_register_uri_handler(server, &submit);
			
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

