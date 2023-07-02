/* WiFi station Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "cJSON.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "esp_tls.h"
#include "TE_mqtt.h"
#include "wifi_manager.h"
#include "uart_service.h"
#include "esp_storage.h"
#include "te_gw_setup.h"
#include "display-service.h"

#ifdef __cplusplus
extern "C" {
#endif

mqtt_models_t backup_stream_model;
mqtt_models_t heartbeat_stream_model;

QueueHandle_t message_queue;
TaskHandle_t message_queue_handler;

StaticTask_t xTaskBuffer;

time_t now;
struct tm timeinfo;
char strftime_buf[64];
char strftime_buf_query[64];

static const char *TAG = "MQTT_TCP";

char* AUTH_URI = "";
char* sub_topic = "";
char COM_PUB_URI[150];
char COM_SUB_URI[150];
// char MQTT_PEM[2050]; 
const char mqtt_pub_str[] = "%s/%s/%s/%s/%s";
const char mqtt_sub_str[] = "%s/+/cmd/+/%s";

// static char AUTH_HINT[20] = "";
char* TEST_URI = "mqtts://quance:quancewu@quance.ideasky.app:58883";
char* BACKUP_URI = "mqtts://quance:quancewu@roster.tornadoedge.app:8883";

char* LWT_msg = "";

extern const uint8_t quance_mqtt_pem_start[]   asm("_binary_quance_mqtt_pem_start");
extern const uint8_t quance_mqtt_pem_end[]   asm("_binary_quance_mqtt_pem_end");

extern const uint8_t backup_mqtt_pem_start[]   asm("_binary_backup_mqtt_pem_start");
extern const uint8_t backup_mqtt_pem_end[]   asm("_binary_backup_mqtt_pem_end");

extern const uint8_t server_mqtt_pem_start[]   asm("_binary_server_mqtt_pem_start");
extern const uint8_t server_mqtt_pem_end[]   asm("_binary_server_mqtt_pem_end");

bool mqtt_destroying = false;
bool refreshing = false;
bool network_status = false;
bool wifi_status = false;
bool ethernet_status = false;

uint32_t mqtt_stream_send_data(mqtt_models_t* mqtt_model, message_type_v2_t event, cJSON *data);

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id){
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED sub %s", COM_SUB_URI);
            msg_id = esp_mqtt_client_subscribe(client, COM_SUB_URI, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            // for (int j=0; j<10; j++){
            //     if (strcmp((char*)tornadoedge_auth_settings.ecu_pairs[j],"")){
            //         ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED sub %s", (char*)tornadoedge_auth_settings.ecu_pairs[j]);
            //         msg_id = esp_mqtt_client_subscribe(client, (char*)tornadoedge_auth_settings.ecu_pairs[j], 0);
            //         ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            //         vTaskDelay(50 / portTICK_PERIOD_MS);
            //     }
            // }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            blink_led_G(2);
            parameter_update(event->topic, event->data, SETUP_MQTT_MODE);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
        }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

char *phex(const void *p, size_t n)
{
    const unsigned char *cp = p; /* Access as bytes. */
    char *s = malloc(2 * n + 1); /* 2*n hex digits, plus NUL. */
    size_t k;

    /*
     * Just in case - if allocation failed.
     */
    if (s == NULL)
        return s;

    for (k = 0; k < n; ++k)
    {
        /*
         * Convert one byte of data into two hex-digit characters.
         */
        sprintf(s + 2 * k, "%02x", cp[k]);
    }

    /*
     * Terminate the string with a NUL character.
     */
    s[2 * n] = '\0';

    return s;
}

cJSON *new_rawdata_json(char client[], uint8_t raw_data[])
{
    char *str = phex(raw_data, 64);

    cJSON *data = cJSON_CreateObject();
    cJSON_AddStringToObject(data, "MessageType", "raw");
    cJSON_AddStringToObject(data, "Client", client);
    cJSON_AddStringToObject(data, "Data", str);
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in UTC+0 is: %s", strftime_buf);
    cJSON_AddStringToObject(data, "Timestamp", strftime_buf);
    free(str);

    return data;
}

cJSON *new_data_json(char gw_id[], char su_id[], uint8_t raw_data[], message_type_v2_t event, status_code_t status)
{
    cJSON *data = cJSON_CreateObject();
    cJSON *payload;
    cJSON *weight_array;
    cJSON *raw_array;

    // uint8_t time_code[6];
    // for(int i=0; i<6; i++){
    //     time_code[i] = raw_data[i+24];
    // }
    char time_var[20];
    const char time_str[] = "%d-%02d-%02dT%02d:%02d:%02dZ";
    snprintf(time_var, (size_t)80,time_str,
            raw_data[24]+2000,raw_data[25],raw_data[26],raw_data[27],raw_data[28],raw_data[29]);
    // char *strtime = phex(time_code, 6);

    cJSON_AddStringToObject(data, "MessageType", get_message_type_str(event));
    cJSON_AddStringToObject(data, "GatewayId", gw_id);
    cJSON_AddStringToObject(data, "DeviceId", su_id);

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

    cJSON_AddItemToObject(data, "Data", payload=cJSON_CreateObject());
    weight_array = cJSON_CreateArray();
    if ((status==FINE || status==ILLEGAL) && (event==SG_SEQ_JSON_DATA || event==SG_REPORT_JSON_DATA)){
        if (raw_data[10]==0x00){
            // hourly data
            raw_array = cJSON_CreateArray();
            cJSON_AddItemToArray(weight_array, cJSON_CreateNumber(((uint16_t)raw_data[13] << 8) | raw_data[12]));
            cJSON_AddItemToArray(raw_array, cJSON_CreateNumber(((uint16_t)raw_data[15] << 8) | raw_data[14]));
            cJSON_AddItemToObject(payload,"Weight", weight_array);
            cJSON_AddItemToObject(payload,"Raw", raw_array);
            cJSON_AddNumberToObject(payload,"K1", ((uint16_t)raw_data[17] << 8) | raw_data[16]);
            cJSON_AddNumberToObject(payload,"K2", ((uint16_t)raw_data[19] << 8) | raw_data[18]);
            cJSON_AddNumberToObject(payload,"Rfch", raw_data[20]);
            cJSON_AddNumberToObject(payload,"RfOrder", raw_data[5] & ~(0xF0));
            cJSON_AddNumberToObject(payload,"Bat", raw_data[21]);
            cJSON_AddNumberToObject(payload,"Temperature", raw_data[22]);
            cJSON_AddNumberToObject(payload,"ErrorCode", raw_data[23]);
            cJSON_AddStringToObject(payload,"Time", time_var);
        }
        else if (raw_data[10]==0x01){
            // real time data
            cJSON_AddItemToArray(weight_array,cJSON_CreateNumber(((uint16_t)raw_data[13] << 8) | raw_data[12]));
            cJSON_AddItemToArray(weight_array,cJSON_CreateNumber(((uint16_t)raw_data[15] << 8) | raw_data[14]));
            cJSON_AddItemToArray(weight_array,cJSON_CreateNumber(((uint16_t)raw_data[17] << 8) | raw_data[16]));
            cJSON_AddItemToArray(weight_array,cJSON_CreateNumber(((uint16_t)raw_data[19] << 8) | raw_data[18]));
            cJSON_AddItemToArray(weight_array,cJSON_CreateNumber(((uint16_t)raw_data[21] << 8) | raw_data[20]));
            cJSON_AddItemToObject(payload,"Weight", weight_array);
            cJSON_AddNumberToObject(payload,"RfOrder", raw_data[5] & ~(0xF0));
            // cJSON_AddNumberToObject(payload,"Bat", raw_data[22]);
            // cJSON_AddNumberToObject(payload,"ErrorCode", raw_data[23]);
            cJSON_AddStringToObject(payload,"Time", time_var);
            cJSON_AddNumberToObject(payload,"TimeDelta", 1000);
        }
        
    }

    cJSON_AddStringToObject(data, "Status", get_status_str(status));
    cJSON_AddStringToObject(data, "Timestamp", strftime_buf);

    // free(strtime);
    return data;
}

cJSON *factory_json(char client[], message_type_v2_t event, status_code_t status, char su_id[], char msg[])
{
    cJSON *data = cJSON_CreateObject();
    cJSON *payload;
    cJSON *su_array;
    time_t temp_time;

    
    cJSON_AddStringToObject(data, "MessageType", get_message_type_str(event));
    cJSON_AddStringToObject(data, "GatewayId", client);
    
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    
    switch (event)
    {
        case NONE_MESSAGE:
            cJSON_AddStringToObject(data,"Message", msg);
            break;
        case GATEWAY_HEARTBEAT:
            if (strcmp(msg,"")){
                cJSON_AddStringToObject(data,"Message", msg);
            } 
            break;
        case GATEWAY_SYSTEM_INFO:
            if (strcmp(msg,"")){
                cJSON_AddStringToObject(data,"Message", msg);
            }
            cJSON_AddStringToObject(data,"BuildTime", compile_ver);
            cJSON_AddStringToObject(data,"Version", (char*)device_configs.tornado_ver);
            cJSON_AddStringToObject(data,"DeviceMac", (char*)device_configs.device_mac);
            break;
        case GATEWAY_GET_PARAMETER:
            if (strcmp(msg,"")){
                cJSON_AddStringToObject(data,"Message", msg);
            }
            cJSON_AddItemToObject(data, "data", payload=cJSON_CreateObject());
            cJSON_AddNumberToObject(payload,"Rfch", tornadoedge_configs_v1.gw_rfch);
            break;
        case GATEWAY_GET_PAIRING:
            if (strcmp(msg,"")){
                cJSON_AddStringToObject(data,"Message", msg);
            }
            break;
        case GATEWAY_SET_PAIRING:
            break;
        case GATEWAY_GET_DISPLAY:
            if (strcmp(msg,"")){
                cJSON_AddStringToObject(data,"Message", msg);
            }
            break;
        case GATEWAY_SET_DISPLAY:
            break;
        case GATEWAY_PAIRING:
            break;
        case GATEWAY_DISPLAY:
            cJSON_AddItemToObject(data, "payload", payload=cJSON_CreateObject());
            su_array = cJSON_CreateArray();
            for (int i=0; i<16;i++){
                if (strcmp((char*)tornadoedge_display_unit_v2.display_pairs[i],"")){
                    cJSON_AddItemToArray(su_array, cJSON_CreateString((char*)tornadoedge_display_unit_v2.display_pairs[i]));
                }
            }
            cJSON_AddItemToObject(payload, "su_array", su_array);
            break;
        case GATEWAY_QUERY:
            cJSON_AddStringToObject(data, "Device", su_id);
            temp_time = now-tornadoedge_configs_v1.mqtt_query_data_age;
            localtime_r(&temp_time, &timeinfo);
            strftime(strftime_buf_query, sizeof(strftime_buf_query), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
            cJSON_AddStringToObject(data, "After", strftime_buf_query);
            cJSON_AddStringToObject(data, "Before", strftime_buf);
            break;
        case GATEWAY_DATA_RESPONSE:
            break;
        case GATEWAY_REBOOT:
            break;
        case GATEWAY_TIME_SYNC:
            break;
        case GATEWAY_PARAM_UPDATE:
            break;
        case GATEWAY_REPLY:
            break;
        case AEROBOX_HEARTBEAT:
            cJSON_AddStringToObject(data, "DeviceId", su_id);
            break;
        case SG_HEARTBEAT:
            cJSON_AddStringToObject(data, "DeviceId", su_id);
            break;
        case AEROBOX_RAW_DATA:
            cJSON_AddStringToObject(data, "DeviceId", su_id);
            cJSON_AddStringToObject(data, "Data", msg);
            break;
        case AEROBOX_JSON_DATA:
            // cJSON_AddStringToObject(data, "DeviceId", su_id);
            // cJSON_AddStringToObject(data, "Data", msg);
            break;
        case SG_SEQ_JSON_DATA:
            cJSON_AddStringToObject(data, "DeviceId", su_id);
            break;
        case SG_REPORT_JSON_DATA:
            cJSON_AddStringToObject(data, "DeviceId", su_id);
            break;
        default:
            break;
    }
    cJSON_AddStringToObject(data, "Status", get_status_str(status));
    cJSON_AddStringToObject(data, "Timestamp", strftime_buf);

    return data;
}

static esp_mqtt_client_handle_t new_mqtt_client(char uri[], bool backup)
{
    // static const psk_hint_key_t psk_hint_key = {
    //         .key = (uint8_t*)S_KEY_CHAR,
    //         .key_size = sizeof(S_KEY),
    //         .hint = (char*)AUTH_HINT
    //         };

    char mqtt_pub[100];

    snprintf(mqtt_pub, (size_t)100, mqtt_pub_str,
                    (char*)tornadoedge_auth_settings_v2.mqtt_pub_uri,
                    "v1",
                    "state",
                    "heartbeat",
                    (char*)tornadoedge_auth_settings_v2.auth_client_id);
    strcpy(COM_PUB_URI, "");
    strcat(COM_PUB_URI, mqtt_pub);
    strcat(COM_PUB_URI, "\0");

    const esp_mqtt_client_config_t mqtt_cfg_backup = {
        .uri = TEST_URI,
        .lwt_topic = COM_PUB_URI,
        .lwt_msg = LWT_msg,
        .cert_pem = (const char *)quance_mqtt_pem_start,
    };

    // const esp_mqtt_client_config_t mqtt_cfg_backup = {
    //     .uri = BACKUP_URI,
    //     .lwt_topic = COM_PUB_URI,
    //     .lwt_msg = LWT_msg,
    //     .cert_pem = (const char *)backup_mqtt_pem_start,
    // };

    const esp_mqtt_client_config_t mqtt_cfg_server = {
        .uri = AUTH_URI,
        .lwt_topic = COM_PUB_URI,
        .lwt_msg = LWT_msg,
        .cert_pem = (const char *)server_mqtt_pem_start,
    };
    esp_mqtt_client_handle_t client;
    if (tornadoedge_configs_v1.backup_mqtt){
        ESP_LOGI(TAG, "Connecting to %s...", mqtt_cfg_backup.uri);
        client = esp_mqtt_client_init(&mqtt_cfg_backup);
    }
    else{
        ESP_LOGI(TAG, "Connecting to %s...", mqtt_cfg_server.uri);
        client = esp_mqtt_client_init(&mqtt_cfg_server);
    }
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    return client;
}

bool esp_mqtt_client_is_connected(esp_mqtt_client_handle_t client)
{
    if (client == NULL) {
        return false;
    }
    // return client->state == MQTT_STATE_CONNECTED;
    return true;
}

void mqtt_stream_send_directly(mqtt_models_t *models, cJSON *json_obj, message_type_v2_t msg_id)
{   
    char mqtt_pub[100];
    if (json_obj == NULL){
        ESP_LOGI(TAG, "JSON_OBJ is NULL When Sending.");  
        return; 
    }      

    for (int retry=0; retry<3; retry++){
        if (models->client == NULL){
            ESP_LOGI(TAG, "Client is NULL When Sending.");    
            vTaskDelay(2000 / portTICK_RATE_MS);
            continue;
        } 

        if (esp_mqtt_client_is_connected(models->client)){       
            char *json_text = cJSON_Print(json_obj);
            snprintf(mqtt_pub, (size_t)100, mqtt_pub_str,
                (char*)tornadoedge_auth_settings_v2.mqtt_pub_uri,
                get_message_var_str(msg_id),
                get_message_cate_str(msg_id),
                get_message_type_str(msg_id),
                (char*)tornadoedge_auth_settings_v2.auth_client_id);
            strcpy(COM_PUB_URI, "");
            strcat(COM_PUB_URI, mqtt_pub);
            strcat(COM_PUB_URI, "\0");
            ESP_LOGI(TAG, "pub: %s",COM_PUB_URI);
            ESP_LOGI(TAG, "client send: %s...", json_text);
            esp_mqtt_client_publish(models->client, COM_PUB_URI, json_text, 0, 0, 0);
            blink_led_G(1);
            free(json_text);
            
            if (json_obj){
                cJSON_Delete(json_obj);
            }

            return;
        }        

        vTaskDelay(2000 / portTICK_RATE_MS);   
    }
    
    ESP_LOGI(TAG, "Over MAX_RETRIES in stream send");    

}

void mqtt_stream_sender(void *pvParameters)
{
    mqtt_message_t message;
    BaseType_t xStatus;
    ESP_LOGI(TAG, "queue in sender: %p", (void *)&message_queue);

    for (;;){
        xStatus = xQueueReceive(message_queue, &(message), portMAX_DELAY);
        
        if (xStatus == pdPASS) {
            if (message.code == DESTORY_EVENT){
                destory_mqtt();
                vTaskDelay(3000 / portTICK_RATE_MS); 
            }
            if (message.code == STOP_EVENT){
                break;
            }
            switch (message.code){
                case SEND_DATA_EVENT:{
                    mqtt_stream_send_directly(message.mqtt_model, message.json_data, message.msg_type);
                }
                default:
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

uint32_t mqtt_stream_send_destory_all_event(){
    mqtt_message_t msg;
    msg.code = DESTORY_EVENT;
    if (message_queue == NULL){
        return 0;
    }
    return xQueueSend(message_queue, (void *) &msg, portMAX_DELAY);
}

uint32_t mqtt_stream_stop(){
    mqtt_message_t msg;
    msg.code = STOP_EVENT;
    if (message_queue == NULL){
        return 0;
    }
    return xQueueSend(message_queue, (void *) &msg, portMAX_DELAY);
}

uint32_t mqtt_stream_send_data(mqtt_models_t* mqtt_model, message_type_v2_t event, cJSON *data){
    mqtt_message_t msg;
    msg.mqtt_model = mqtt_model;
    msg.code = SEND_DATA_EVENT;
    msg.msg_type = event;
    msg.json_data = data;
    if (message_queue == NULL){
        ESP_LOGW(TAG, "No Message Queue");
        cJSON_Delete(data);
        return 0;
    }
    return xQueueSend(message_queue, (void *) &msg, portMAX_DELAY);
}

void message_queue_init(){    
    if (message_queue == NULL){
        ESP_LOGI(TAG, "message_queue init");
        message_queue = xQueueCreate(50, sizeof(mqtt_message_t));
    }
}

void mqtt_stream_sender_start(){    
    xTaskCreate(
        mqtt_stream_sender,        // Function that implements the task.
        "Stream Sender",                           // Text name for the task.
        4096,                           // Stack size in bytes, not words.
        NULL,                        // Parameter passed into the task.
        3,          // Priority at which the task is created.
        message_queue_handler);       // Variable to hold the task's data structure.
}

mqtt_models_t mqtt_stream_model_init(char uri[], char *name, bool backup)
{
    esp_mqtt_client_handle_t client = new_mqtt_client(uri, backup);

    TaskHandle_t mqtt_task_manager = NULL;
    ESP_LOGW(TAG, "client in stream_model_init: %p",client);
    mqtt_models_t models = {client, mqtt_task_manager, name, message_queue};
    ESP_LOGW(TAG, "client in model of stream_model_init: %p", models.client);

    return models;
}


mqtt_models_t heartbeat_stream_models_init()
{
    return mqtt_stream_model_init(AUTH_URI, "heartbeat", false);
}

mqtt_models_t backup_stream_models_init()
{
    return mqtt_stream_model_init(AUTH_URI, "backup", true);
}

char *get_device_id(char su_id[],char gw_id[]){
    char *device_id = malloc(20);
    strcpy(device_id, (char*)(tornadoedge_auth_settings_v2.auth_client_id));
    // strcat(device_id, ":");
    // strcat(device_id, gw_id);
    // strcat(device_id, ":");
    // strcat(device_id, su_id);
    
    return device_id;
}


// void mqtt_send_heartbeat(message_type_t event, status_code_t status){    
//     char *device_id = get_device_id("","");
//     cJSON *heartbeat_json = new_heartbeat_json(device_id, event, status);

//     mqtt_stream_send_data(&heartbeat_stream_model, event, heartbeat_json);

//     free(device_id);
// }

void mqtt_send_tornado_rawdata(uint8_t data[],char su_id[],char gw_id[]){
    // char *device_id = get_device_id(su_id,gw_id);        
    cJSON *data_json = new_rawdata_json(su_id, data);

    mqtt_stream_send_data(&heartbeat_stream_model, AEROBOX_RAW_DATA, data_json);

    // free(device_id);
}

void mqtt_send_tornado_data(message_type_v2_t event, status_code_t status, uint8_t data[], char su_id[], char gw_id[]){
    char *device_id = get_device_id(su_id,gw_id);        
    cJSON *data_json = new_data_json(device_id, su_id, data, event, status);

    mqtt_stream_send_data(&heartbeat_stream_model, event, data_json);

    free(device_id);
}

void mqtt_send_tornado_factory(message_type_v2_t event, status_code_t status,char msg[], char su_id[], char gw_id[]){
    char *device_id = get_device_id(su_id,gw_id);
    // cJSON *data_json = factory_json(device_id, event, status, msg);
    cJSON *data_json = factory_json(device_id, event, status, su_id, msg);

    mqtt_stream_send_data(&heartbeat_stream_model, event, data_json);

    free(device_id);
}

void mqtt_send_tornado_factory_json(message_type_v2_t event, cJSON *data){
    mqtt_stream_send_data(&heartbeat_stream_model, event, data);
}


void send_heartbeat_task(void *pvParameters){
    while (true){
        blink_led_B(1);
        mqtt_send_tornado_factory(GATEWAY_HEARTBEAT,FINE,"","","");
        vTaskDelay((tornadoedge_configs_v1.heartbeat_resolution*1000) / portTICK_PERIOD_MS);
    }
}

void create_mqtt(){
    heartbeat_stream_model = heartbeat_stream_models_init();
    // backup_stream_model = backup_stream_models_init();
    ESP_LOGI(TAG, "MQTT Created");
}
 
void destory_mqtt(){
    if (mqtt_destroying){
        ESP_LOGI(TAG, "mqtt_destroying!!!!");
        return;
    }
    mqtt_destroying = true;
    if (heartbeat_stream_model.client != NULL){
        esp_mqtt_client_stop(heartbeat_stream_model.client);
        esp_mqtt_client_destroy(heartbeat_stream_model.client);
        ESP_LOGI(TAG, "MQTT Heartbeat Clients Destoryed");   
        heartbeat_stream_model.client = NULL;
    } 
    if (backup_stream_model.client != NULL){
        esp_mqtt_client_stop(backup_stream_model.client);
        esp_mqtt_client_destroy(backup_stream_model.client);
        ESP_LOGI(TAG, "MQTT Data Clients Destoryed");    
        backup_stream_model.client = NULL;
    }
    if (heartbeat_stream_model.mqtt_task_manager != NULL){
        vTaskDelete(heartbeat_stream_model.mqtt_task_manager);
        ESP_LOGI(TAG, "MQTT Heartbeat Tasks Destoryed");   
        heartbeat_stream_model.mqtt_task_manager = NULL;
    }
    if (backup_stream_model.mqtt_task_manager != NULL){
        vTaskDelete(backup_stream_model.mqtt_task_manager);
        ESP_LOGI(TAG, "MQTT Data Tasks Destoryed");    
        backup_stream_model.mqtt_task_manager = NULL;
    }
    if (message_queue != NULL){
        //mqtt_stream_stop();
    }
    mqtt_destroying = false;
}

void get_ip_cb(void *pvParameter){
	ip_event_got_ip_t* param = (ip_event_got_ip_t*)pvParameter;

	/* transform IP to human readable string */
	char str_ip[16];
	esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

	ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
}

static void connection_refresh(void *pvParameter){
    ESP_LOGI(TAG, "Async Get Token Event");
    
    ESP_LOGI(TAG, "MQTT Create Event");
    create_mqtt();
    ESP_LOGI(TAG, "MQTT Initialized");
    mqtt_send_tornado_factory(GATEWAY_SYSTEM_INFO,FINE,"MQTT Initialized","","");
    // PAIRING_GET 
    mqtt_send_tornado_factory(GATEWAY_GET_PAIRING,FINE,"",
                                "",
                                (char*)tornadoedge_auth_settings_v2.hex_client_id
                                );
    // DISPLAY_GET 
    mqtt_send_tornado_factory(GATEWAY_GET_DISPLAY,FINE,"",
                                "",
                                (char*)tornadoedge_auth_settings_v2.hex_client_id
                                );

    mqtt_send_tornado_factory(GATEWAY_GET_PARAMETER,FINE,"boot-up refresh parameter","",(char*)tornadoedge_auth_settings_v2.hex_client_id);
}

static void wifi_connection_cb(void *pvParameter){  
    ESP_LOGI(TAG, "MQTT trying connect by wifi callback");
    wifi_status = true;
    if (!network_status){
        message_queue_init();
        mqtt_stream_sender_start();

        get_ip_cb(pvParameter);
        network_status = true;

        connection_refresh(pvParameter);
        // PAIRING_STATUS
        mqtt_send_tornado_factory(GATEWAY_HEARTBEAT,REBOOTED,"callback from wifi connection",
                                    "",
                                    (char*)tornadoedge_auth_settings_v2.hex_client_id
                                    );
    }
    else{
        network_status = true;
        ESP_LOGI(TAG, "MQTT Initialized already");
    }
    
}

static void eth_connection_cb(void *pvParameter){  
    ESP_LOGI(TAG, "MQTT trying connect by ethernet callback");
    ethernet_status = true;
    if (!network_status){
        message_queue_init();
        mqtt_stream_sender_start();

        get_ip_cb(pvParameter);
        network_status = true;

        connection_refresh(pvParameter);
        // PAIRING_STATUS
        mqtt_send_tornado_factory(GATEWAY_HEARTBEAT,REBOOTED,"callback from ethernet connection",
                                    "",
                                    (char*)tornadoedge_auth_settings_v2.hex_client_id
                                    );
    }
    else{
        network_status = true;
        ESP_LOGI(TAG, "MQTT Initialized already");
    }
    
}

static void wifi_disconnection_cb(void *pvParameter){
    wifi_status = false;
    ESP_LOGI(TAG, "MQTT Destory Event bool %d %d",wifi_status,ethernet_status);
    if (!(wifi_status || ethernet_status)){
        network_status = false;
        ESP_LOGI(TAG, "MQTT Destory Event");
        destory_mqtt();
    }
    
}

static void eth_disconnection_cb(void *pvParameter){
    ethernet_status = false;
    ESP_LOGI(TAG, "MQTT Destory Event bool %d %d",wifi_status,ethernet_status);
    if (!(wifi_status || ethernet_status)){
        network_status = false;
        ESP_LOGI(TAG, "MQTT Destory Event");
        destory_mqtt();
    }
    
}

void wifi_manager_mqtt_event_handler_init(){
    char mqtt_sub[100];
    AUTH_URI = (char*)tornadoedge_auth_settings_v2.mqtt_auth_uri;
    snprintf(mqtt_sub, (size_t)100, mqtt_sub_str, (char*)tornadoedge_auth_settings_v2.mqtt_sub_uri,(char*)tornadoedge_auth_settings_v2.auth_client_id);
    strcpy(COM_SUB_URI, "");
    strcat(COM_SUB_URI, mqtt_sub);
    strcat(COM_SUB_URI, "\0");
    // memcpy(AUTH_HINT,tornadoedge_auth_settings.auth_psk_hint,sizeof(tornadoedge_auth_settings.auth_psk_hint));
    
    cJSON *lwt_json = cJSON_CreateObject();
    cJSON_AddStringToObject(lwt_json, "MessageType", "heartbeat");
    cJSON_AddStringToObject(lwt_json, "GatewayId", (char*)(tornadoedge_auth_settings_v2.auth_client_id));
    cJSON_AddStringToObject(lwt_json, "Status", "disconnect");
    cJSON_AddBoolToObject(lwt_json, "IsLastWill", true);
    LWT_msg = cJSON_Print(lwt_json);
    cJSON_Delete(lwt_json);

    ESP_LOGI(TAG, "Test PUB_URI %s", COM_PUB_URI);
    ESP_LOGI(TAG, "Test SUB_URI %s", COM_SUB_URI);

    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &wifi_connection_cb);
    wifi_manager_set_callback(WM_EVENT_ETH_GOT_IP, &eth_connection_cb);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &wifi_disconnection_cb);
    wifi_manager_set_callback(WM_EVENT_ETH_DISCONNECTED, &eth_disconnection_cb);
}

#ifdef __cplusplus
}
#endif