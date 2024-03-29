/**************************************************************
* http_server.c 
* Contains all the code pertaining to running an HTTP server
* when in normal mode 
**************************************************************/
// Header files
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include <esp_http_server.h>
#include "raahi.h"

// Defines
#define FORM_DATA_BUF_SIZE 256 

static const char *TAG = "http_server";

// External variables
extern struct config_struct sysconfig;
extern struct debug_data_struct debug_data;
extern char raahi_log_str[EVENT_JSON_STR_SIZE];
extern char user_mqtt_str[MAX_DEVICE_ID_LEN];
extern zombie_info_struct zombie_info;

extern void create_sysconfig_json(char* json_str, uint16_t json_str_len);
extern void raahi_restart(void);
extern void display_sysconfig();

// Function declarations
int32_t str2num(char* input_str, const char delimiter, uint8_t max_parse_len);

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
| 	infopage_get_handler()
|	HTTP server side handler for GET reuests on /info
------------------------------------------------------------*/
static esp_err_t infopage_get_handler(httpd_req_t *req)
{
     /* Get handle to embedded file upload script */
    extern const unsigned char infopage_start[] asm("_binary_info_html_start");
    extern const unsigned char infopage_end[]   asm("_binary_info_html_end");
    const size_t infopage_size = (infopage_end - infopage_start);
	uint8_t slave_id_idx, reg_address_idx;
	char tempStr[200];
	
	//httpd_resp_set_type(req, "text/html"); 
    httpd_resp_send_chunk(req, (const char *)infopage_start, infopage_size);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>Client ID</td><td>%s</td></tr>\n", sysconfig.client_id); 
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>FW Ver</td><td>%s</td></tr>\n", debug_data.fw_ver); 
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>MAC</td><td>%s</td></tr>\n", user_mqtt_str); 
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>IMEI</td><td>%s</td></tr>\n", debug_data.imei); 
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>Operator</td><td>%s</td></tr>\n", debug_data.oper);
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>RSSI</td><td>%u</td></tr>\n", debug_data.rssi);
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>BER</td><td>%u</td></tr>\n", debug_data.ber);
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>Battery Voltage</td><td>%u</td></tr>\n", debug_data.battery_voltage);
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	if(debug_data.connected_to_internet == true)
	{
		sprintf(tempStr, "\t\t<tr><td>Connected to internet:</td><td>%s</td></tr>\n", "Yes");
	}
	else
	{
		sprintf(tempStr, "\t\t<tr><td>Connected to internet:</td><td>%s</td></tr>\n", "No");
	}
	httpd_resp_sendstr_chunk(req, tempStr);
	
	tempStr[0] = '\0';
	if(debug_data.connected_to_aws == true)
	{
		sprintf(tempStr, "\t\t<tr><td>Connected to AWS:</td><td>%s</td></tr>\n", "Yes");
	}
	else
	{
		sprintf(tempStr, "\t\t<tr><td>Connected to AWS:</td><td>%s</td></tr>\n", "No");
	}
	httpd_resp_sendstr_chunk(req, tempStr);
	
    tempStr[0] = '\0';
	sprintf(tempStr, "\t\t<tr><td>Reset Reason</td><td>%s</td></tr>\n", debug_data.reset_reason_str); 
	httpd_resp_sendstr_chunk(req, tempStr);
	
	
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
	
			tempStr[0] = '\0';
			sprintf(tempStr, "\t\t<tr><td>Slave %u, Reg %u</td><td>0x%.4X</td></tr>\n", (slave_id_idx+1), sysconfig.reg_address[reg_address_idx], debug_data.slave_info[slave_id_idx].data[reg_address_idx]);
			httpd_resp_sendstr_chunk(req, tempStr);
		}
	}

	httpd_resp_sendstr_chunk(req, "\t</tbody>\n");
	httpd_resp_sendstr_chunk(req, "\t</table>\n");
	httpd_resp_sendstr_chunk(req, "</body>\n");
	httpd_resp_sendstr_chunk(req, "</html>\n");
	
    httpd_resp_sendstr_chunk(req, NULL);
	
	return(ESP_OK);
}


static const httpd_uri_t infopage = {
    .uri       = "/info",
    .method    = HTTP_GET,
    .handler   = infopage_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};


/* -----------------------------------------------------------
| 	config_get_handler()
|	HTTP server side handler for GET reuests on /sysconfig
------------------------------------------------------------*/
static esp_err_t config_get_handler(httpd_req_t *req)
{
	char sysconfig_json[QUERY_JSON_STR_SIZE];

 	create_sysconfig_json(sysconfig_json, QUERY_JSON_STR_SIZE);
	httpd_resp_sendstr_chunk(req, sysconfig_json);
	httpd_resp_sendstr_chunk(req, NULL);
	return(ESP_OK);
	
}

static const httpd_uri_t sysconfig_get = {
    .uri       = "/sysconfig",
    .method    = HTTP_GET,
    .handler   = config_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};


/* -----------------------------------------------------------
| 	update_sysconfig()
|	Parses the form string from HTTP client and updates the 
| 	sysconfig values (runtime values as well as in the 
| 	sysconfig.txt
------------------------------------------------------------*/
void update_sysconfig(char* form_str)
{
	char* tmpStr;
	uint8_t tmpIndex;
	int32_t  tmpVal;
	uint8_t client_id_updated = 0;
	
	static const char* config_file_name = "/spiffs/sysconfig.txt";
	FILE* config_file = NULL;
	
	
	// Parse the received string to obtain sysconfig information
	tmpStr = strstr(form_str, "first_slave_id=") + strlen("first_slave_id=");
	if((tmpVal = str2num(tmpStr, '&', 4)) >= 0) {
		sysconfig.slave_id[0] = (uint8_t)tmpVal;
	}
 
	tmpStr = strstr(form_str, "second_slave_id=") + strlen("second_slave_id=");
	if((tmpVal = str2num(tmpStr, '&', 4)) >= 0) {
		sysconfig.slave_id[1] = (uint8_t)tmpVal;
	}

	tmpStr = strstr(form_str, "first_reg_address=") + strlen("first_reg_address=");
	if((tmpVal = str2num(tmpStr, '&', 6)) >= 0) {
		sysconfig.reg_address[0] = (uint16_t)tmpVal;
	}	
	
	tmpStr = strstr(form_str, "second_reg_address=") + strlen("second_reg_address=");
	if((tmpVal = str2num(tmpStr, '&', 6)) >= 0) {
		sysconfig.reg_address[1] = (uint16_t)tmpVal;
	}	
	
	tmpStr = strstr(form_str, "third_reg_address=") + strlen("third_reg_address=");
	if((tmpVal = str2num(tmpStr, '&', 6)) >= 0) {
		sysconfig.reg_address[2] = (uint16_t)tmpVal;
	}	

	tmpStr = strstr(form_str, "sampling_period_in_sec=") + strlen("sampling_period_in_sec=");
	if((tmpVal = str2num(tmpStr, '&', 4)) >= 0) {
		sysconfig.sampling_period_in_sec = (uint8_t)tmpVal;
	}
 	
	tmpStr = strstr(form_str, "client_id=") + strlen("client_id=");
	tmpIndex = 0;
	while(tmpStr[tmpIndex] != '&') 
	{
		sysconfig.client_id[tmpIndex] = tmpStr[tmpIndex];
		if (tmpIndex > MAX_CLIENT_ID_LEN) {
			break;
		}
		tmpIndex++;
	}
	if (tmpIndex != 0) { // Update only if user entered a value	
		sysconfig.client_id[tmpIndex] = '\0';
		client_id_updated = 1;
	}

	tmpStr = strstr(form_str, "topic=") + strlen("topic=");
	tmpIndex = 0;
	while(tmpStr[tmpIndex] != '&') 
	{
		sysconfig.topic[tmpIndex] = tmpStr[tmpIndex];
		if (tmpIndex > MAX_TOPIC_LEN) {
			break;
		}
		tmpIndex++;
	}
	if (tmpIndex != 0) { // Update only if user entered a value	
		sysconfig.topic[tmpIndex] = '\0';
	}
	

	tmpStr = strstr(form_str, "apn=") + strlen("apn=");
	tmpIndex = 0;
	while(tmpStr[tmpIndex] != '&') 
	{
		sysconfig.apn[tmpIndex] = tmpStr[tmpIndex];
		if (tmpIndex > MAX_APN_LEN) {
			break;
		}
		tmpIndex++;
	}
	if (tmpIndex != 0) { // Update only if user entered a value	
		sysconfig.apn[tmpIndex] = '\0';
	}

	// Debug prints
	display_sysconfig();

	config_file = fopen(config_file_name, "rb");
	if (config_file == NULL) { // If config file isnt' present 
		RAAHI_LOGE(TAG, "Config file not present. Can't update sysconfig!");
		abort();
	}
	
	fclose(config_file);
	config_file = fopen(config_file_name, "wb");

	if(fwrite(&sysconfig, sizeof(struct config_struct), 1, config_file) != 1) {
		RAAHI_LOGE(TAG, "Couldn't update sysconfig file although it is present");
		abort();
	} else {
		RAAHI_LOGI(TAG, "Successfully updated sysconfig file");
	}
	fclose(config_file);

	if(client_id_updated == 1) {
		ESP_LOGI(TAG, "Client ID updated. So restarting");
        strcpy(zombie_info.esp_restart_reason, "Client ID Updated (http)");
       	vTaskDelay(5000 / portTICK_RATE_MS);
		raahi_restart();
	}
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
	RAAHI_LOGI(TAG, "HTTP post handler");

	if (remaining > FORM_DATA_BUF_SIZE) {
		RAAHI_LOGE(TAG, "HTTP form data larger than internal receive buffer");
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
	//RAAHI_LOGI(TAG, "Received string: %s\n", buf);

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
    RAAHI_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        RAAHI_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &homepage);
        httpd_register_uri_handler(server, &infopage);
        httpd_register_uri_handler(server, &sysconfig_get);
		httpd_register_uri_handler(server, &favicon_ico);
		httpd_register_uri_handler(server, &submit);
			
        return server;
    }

    RAAHI_LOGI(TAG, "Error starting server!");
    return NULL;
}

