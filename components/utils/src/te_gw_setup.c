#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "cJSON.h"

#include "esp_event.h"
#include "esp_log.h"

#include "esp_storage.h"
#include "uart_service.h"
#include "te_gw_setup.h"
#include "display-service.h"
#include "TE_mqtt.h"
#include "websocket_server.h"

#ifdef __cplusplus
extern "C" {
#endif

time_t now;
struct tm timeinfo;
char strftime_buf[64];
char strftime_buf_query[64];

static const char *TAG = "TE-para-update";

esp_err_t parameter_update(char *topic, char *payload, te_gw_setup_mode_t setup_mode)
{
    message_type_v2_t mesgt = NONE_MESSAGE;
    cJSON* cjson_sub = NULL;
    cJSON* cjson_msgType = NULL;
    cJSON* cjson_gatewayId = NULL;
    cJSON* cjson_aeroboxId = NULL;
    cJSON* cjson_param = NULL;
    cJSON* cjson_timestamp = NULL;
    cJSON* cjson_payload = NULL;
    cjson_sub = cJSON_Parse(payload);
    cjson_msgType = cJSON_GetObjectItem(cjson_sub, "MessageType");
    cjson_gatewayId = cJSON_GetObjectItem(cjson_sub, "GatewayId");
    cjson_payload = cJSON_GetObjectItem(cjson_sub, "Data");
    if (cjson_gatewayId){
        if (strcasecmp(cjson_gatewayId->valuestring, (char*)tornadoedge_auth_settings_v2.auth_client_id) != 0){
            ESP_LOGW(TAG, "TornadeoEdge Gateway ID incorrect");
            ESP_LOGW(TAG, "TornadeoEdge Gateway ID : %s", tornadoedge_auth_settings_v2.auth_client_id);
            ESP_LOGW(TAG, "Request      Gateway ID : %s", cjson_gatewayId->valuestring);
            mqtt_send_tornado_factory(GATEWAY_SYSTEM_INFO,ERROR,"TornadeoEdge Gateway ID incorrect","","");
            // break;
            return ESP_OK;  
        }
    }
    if (!(cjson_msgType)){
        ESP_LOGI(TAG, "No MessageType key in cJSON payload");
        // break;
        return ESP_OK;
    }
    switch (setup_mode)
        {
        case SETUP_MQTT_MODE:
            mesgt = detect_msg_type_from_topic(topic);
            printf("%s\n\n",get_message_type_str(mesgt));
            break;
        case SETUP_WS_MODE:
            mesgt = detect_msg_type(cjson_msgType->valuestring);
            break;
        default:
            break;
        }
    // switch (detect_msg_type(cjson_msgType->valuestring)){
    switch (mesgt){
        case NONE_MESSAGE:
            // return "none-message";
            // not showing in cmd callback
            break;
        case GATEWAY_HEARTBEAT:
            // return "heartbeat";
            // not showing in cmd callback
            break;
        case GATEWAY_SYSTEM_INFO:
            // return "system-info";
            // not showing in cmd callback
            break;
        case GATEWAY_GET_PARAMETER:
            // return "get-parameter";
            // user can get parameter pairing data in this part
            cjson_param = cJSON_GetObjectItem(cjson_sub, "Parameter");
            if (cjson_msgType && cjson_gatewayId && cjson_payload){
                cJSON *cjson_response = cJSON_CreateObject();
                cJSON *response_payload;
                cJSON* cjson_parser = NULL;
                int count = 0;
                int ret;

                cJSON_AddStringToObject(cjson_response, "MessageType", get_message_type_str(GATEWAY_REPLY));
                cJSON_AddStringToObject(cjson_response, "GatewayId", (char*)(tornadoedge_auth_settings_v2.auth_client_id));

                cjson_parser = cJSON_GetObjectItem(cjson_sub, "CorrelationId");
                if (cjson_parser){
                    cJSON_AddStringToObject(cjson_response, "CorrelationId", cjson_parser->valuestring);
                }
                cJSON_AddItemToObject(cjson_response, "Data", response_payload=cJSON_CreateObject());

                for (int i=0; i<END_PARAM_TYPE; i++){
                    cjson_parser = cJSON_GetObjectItem(cjson_payload, get_param_type_str(i));
                    if (cjson_parser){
                        ret = parse_mqtt_param_get(i, cjson_parser, response_payload);
                        if (ret!=0){
                            count++;
                        }
                    }
                }
                if (count>0){
                    // set_param_update_timestamp((long int*)&now);
                    // save_tornadoedge_value();
                    // blink_led_B(2);
                    // save_te_config_value();
                    // blink_led_B(2);
                    // save_aerobox_gw_value();
                    cJSON_AddBoolToObject(cjson_response, "OK", true);
                }
                else{
                    cJSON_AddBoolToObject(cjson_response, "OK", false);
                    cJSON_AddStringToObject(cjson_response, "FirmwareVersion", (char*)device_configs.tornado_ver);
                }
                char *json_text;
                switch (setup_mode)
                {
                case SETUP_MQTT_MODE:
                    mqtt_send_tornado_factory_json(GATEWAY_REPLY, cjson_response);
                    break;
                case SETUP_WS_MODE:
                    // char *json_text;
                    json_text = cJSON_Print(cjson_response);
                    ws_server_send_text_all_from_callback(json_text,strlen(json_text));
                    cJSON_Delete(cjson_response);
                    free(json_text);
                    break;
                default:
                    free(cjson_response);
                    break;
                }
            }
            break;
        case GATEWAY_GET_PAIRING:
            // return "get-pairing";
            // not showing in cmd callback
            break;
        case GATEWAY_SET_PAIRING:
            // return "set-pairing";
            // not showing in cmd callback
            break;
        case GATEWAY_GET_DISPLAY:
            // return "get-display";
            // not showing in cmd callback
            break;
        case GATEWAY_SET_DISPLAY:
            // return "set-display";
            // not showing in cmd callback
            break;
        case GATEWAY_PAIRING:
            // return "pairing-list";
            time(&now);
            cjson_aeroboxId = cJSON_GetObjectItem(cjson_sub, "Device");
            if (cjson_msgType && cjson_gatewayId && cjson_aeroboxId) {
                // set_led(2, 1, false, "#00004f");
                ESP_LOGI(TAG, "tornado event, event=%s clientId=%s", cjson_msgType->valuestring, cjson_gatewayId->valuestring);
                int array_size = cJSON_GetArraySize(cjson_aeroboxId);
                cJSON* cjson_item = NULL;

                printf("delete all SU_ID\n");
                for (int i=1; i<11; i++){
                    // set_aerobox_id(&i, "");
                    set_su_id(&i, "");
                }
                if (array_size!=0){
                    for(int i=0; i<array_size; i++){
                        cjson_item = cJSON_GetArrayItem(cjson_aeroboxId,i);
                        printf("su_id %02d: %s\n",i,cjson_item->valuestring);
                        int setting_num = i+1;
                        // set_aerobox_id(&setting_num, cjson_item->valuestring);
                        set_su_id(&setting_num, cjson_item->valuestring);
                    }
                }
                
                esp_err_t ret;
                ret = save_tornadoedge_su_value();
                blink_led_B(2);
            }
            break;
        case GATEWAY_DISPLAY:
            // return "display-list";
            cjson_aeroboxId = cJSON_GetObjectItem(cjson_sub, "Device");
            cjson_timestamp = cJSON_GetObjectItem(cjson_sub, "TimeStamp");
            if (cjson_msgType && cjson_gatewayId && cjson_aeroboxId) {
                // set_led(2, 1, false, "#00004f");
                ESP_LOGI(TAG, "tornado event, event=%s clientId=%s", cjson_msgType->valuestring, cjson_gatewayId->valuestring);
                int array_size = cJSON_GetArraySize(cjson_aeroboxId);
                cJSON* cjson_item = NULL;

                printf("delete all display_ID\n");
                for (int i=1; i<11; i++){
                    // set_su_id(&i, "");
                    set_display_id(&i, "");
                }
                if (array_size!=0){
                    for(int i=0; i<array_size; i++){
                        cjson_item = cJSON_GetArrayItem(cjson_aeroboxId,i);
                        printf("display_id %02d: %s\n",i,cjson_item->valuestring);
                        int setting_num = i+1;
                        // set_su_id(&setting_num, cjson_item->valuestring);
                        set_display_id(&setting_num, cjson_item->valuestring);
                    }
                }
                
                esp_err_t ret;
                // ret = save_tornadoedge_value();
                ret = save_tornadoedge_display_value();
                blink_led_B(2);
            }
            break;
        case GATEWAY_QUERY:
            // return "data-query";
            // not showing in cmd callback
            break;
        case GATEWAY_DATA_RESPONSE:
            // return "data-response";
            cjson_aeroboxId = cJSON_GetObjectItem(cjson_sub, "Device");
            if (cjson_msgType && cjson_gatewayId && cjson_aeroboxId && cjson_payload) {
                    ESP_LOGI(TAG, "tornado event, event=%s clientId=%s", cjson_msgType->valuestring, cjson_gatewayId->valuestring);
                    ESP_LOGI(TAG, "data from %s",cjson_aeroboxId->valuestring);
                    cJSON* cjson_temp   = NULL;
                    cJSON* cjson_rh     = NULL;
                    cJSON* cjson_pm2_5  = NULL;
                    cJSON* cjson_pm10   = NULL;
                    cJSON* cjson_co2    = NULL;
                    cjson_temp = cJSON_GetObjectItem(cjson_payload, "temperature");
                    cjson_rh = cJSON_GetObjectItem(cjson_payload, "rh");
                    cjson_pm2_5 = cJSON_GetObjectItem(cjson_payload, "pm2_5");
                    cjson_pm10 = cJSON_GetObjectItem(cjson_payload, "pm10");
                    cjson_co2 = cJSON_GetObjectItem(cjson_payload, "co2");
                    ESP_LOGI(TAG, "temperature->%.2f",cjson_temp->valuedouble);
                    ESP_LOGI(TAG, "RH         ->%.2f",cjson_rh->valuedouble);
                    ESP_LOGI(TAG, "PM2.5      ->%.f",cjson_pm2_5->valuedouble);
                    ESP_LOGI(TAG, "PM10       ->%.f",cjson_pm10->valuedouble);
                    ESP_LOGI(TAG, "CO2        ->%.f",cjson_co2->valuedouble);
                    display_send_mqtt_factory(cjson_aeroboxId->valuestring, cjson_temp->valuedouble, cjson_rh->valuedouble,
                                    cjson_pm2_5->valuedouble, cjson_pm10->valuedouble,cjson_co2->valuedouble);
            }
            else if (cjson_msgType && cjson_gatewayId && cjson_aeroboxId){
                ESP_LOGI(TAG, "tornado event, event=%s clientId=%s", cjson_msgType->valuestring, cjson_gatewayId->valuestring);
                ESP_LOGI(TAG, "No data from %s",cjson_aeroboxId->valuestring);
            }
            break;
        case GATEWAY_REBOOT:
            // return "reboot";
            ESP_LOGI(TAG, "reboot system");
            esp_restart();
            break;
        case GATEWAY_TIME_SYNC:
            // return "time-sync";
            // will do this func in newer version
            break;
        case GATEWAY_PARAM_UPDATE:
            // return "param-update";
            // user can add parameter pairing in this part
            cjson_param = cJSON_GetObjectItem(cjson_sub, "Parameter");
            if (cjson_msgType && cjson_gatewayId && cjson_payload){
                cJSON *cjson_response = cJSON_CreateObject();
                cJSON *response_payload;
                cJSON* cjson_parser = NULL;
                int count = 0;
                int ret;

                cJSON_AddStringToObject(cjson_response, "MessageType", get_message_type_str(GATEWAY_REPLY));
                cJSON_AddStringToObject(cjson_response, "GatewayId", (char*)(tornadoedge_auth_settings_v2.auth_client_id));

                cjson_parser = cJSON_GetObjectItem(cjson_sub, "CorrelationId");
                if (cjson_parser){
                    cJSON_AddStringToObject(cjson_response, "CorrelationId", cjson_parser->valuestring);
                }
                cJSON_AddItemToObject(cjson_response, "Data", response_payload=cJSON_CreateObject());

                for (int i=0; i<END_PARAM_TYPE; i++){
                    cjson_parser = cJSON_GetObjectItem(cjson_payload, get_param_type_str(i));
                    if (cjson_parser){
                        ret = parse_mqtt_param_update(i, cjson_parser);
                        if (ret==1){
                            cJSON_AddStringToObject(response_payload, get_param_type_str(i), cjson_parser->valuestring);
                            count = count + 1;
                        }
                        else if (ret==2){
                            if (cjson_parser->valueint){
                                cJSON_AddTrueToObject(response_payload, get_param_type_str(i));
                            }
                            else{
                                cJSON_AddFalseToObject(response_payload, get_param_type_str(i));
                            }
                            count = count + 1;
                        }
                        else if (ret==3){
                            cJSON_AddNumberToObject(response_payload, get_param_type_str(i), cjson_parser->valueint);
                            count = count + 1;
                        }
                    }
                }
                // while (cjson_parser->next!=NULL){

                // }
                if (count>0){
                    set_param_update_timestamp((long int*)&now);
                    save_tornadoedge_value();
                    blink_led_B(2);
                    save_te_config_value();
                    blink_led_B(2);
                    // save_aerobox_gw_value();
                    cJSON_AddBoolToObject(cjson_response, "OK", true);
                }
                else{
                    cJSON_AddBoolToObject(cjson_response, "OK", false);
                    cJSON_AddStringToObject(cjson_response, "FirmwareVersion", (char*)device_configs.tornado_ver);
                }
                char *json_text;
                switch (setup_mode)
                {
                case SETUP_MQTT_MODE:
                    mqtt_send_tornado_factory_json(GATEWAY_REPLY, cjson_response);
                    break;
                case SETUP_WS_MODE:
                    // char *json_text;
                    json_text = cJSON_Print(cjson_response);
                    ws_server_send_text_all_from_callback(json_text,strlen(json_text));
                    cJSON_Delete(cjson_response);
                    free(json_text);
                    break;
                default:
                    break;
                }
            }
            break;
        case GATEWAY_REPLY:
            // return "reply";
            // not showing in cmd callback
            break;
        case AEROBOX_HEARTBEAT:
            // return "heartbeat";
            // not showing in cmd callback
            break;
        case SG_HEARTBEAT:
            // return "heartbeat";
            // not showing in cmd callback
            break;
        case AEROBOX_RAW_DATA:
            // return "raw";
            // not showing in cmd callback
            break;
        case AEROBOX_JSON_DATA:
            // return "aerobox-json";
            // not showing in cmd callback
            break;
        case SG_SEQ_JSON_DATA:
            // return "sg-json-data";
            // not showing in cmd callback
            break;
        case SG_REPORT_JSON_DATA:
            // return "sg-json-data";
            // not showing in cmd callback
            break;
        case END_MESSAGE_TYPE:
            // return "end-messsage";
            // the message_type_t lens detect using name
            ESP_LOGI(TAG, "unknow MessageType: %s", cjson_msgType->valuestring);
            break;
    }
    cJSON_Delete(cjson_sub);
    // break;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif