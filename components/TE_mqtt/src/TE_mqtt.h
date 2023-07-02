#ifndef TE_mqtt_H_INCLUDED
#define TE_mqtt_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "cJSON.h"
#include "mqtt_client.h"
#include "esp_storage.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

typedef struct mqtt_models_t 
{
    esp_mqtt_client_handle_t client;
    TaskHandle_t mqtt_task_manager;
    char *name;
    QueueHandle_t message_queue;
} mqtt_models_t;

typedef enum mqtt_message_code_t {
    SEND_DATA_EVENT = 1,
    DESTORY_EVENT = 2,
    STOP_EVENT = 3
} mqtt_message_code_t;

typedef struct mqtt_message_t
{
    mqtt_models_t *mqtt_model;
    mqtt_message_code_t code;
    message_type_v2_t msg_type;
    void *param;
    cJSON *json_data;
} mqtt_message_t;

char *phex(const void *p, size_t n);

void destory_mqtt(void);

uint32_t websocket_stream_send_destory_all_event(void);

void wifi_manager_mqtt_event_handler_init(void);

void mqtt_send_heartbeat(char event[], status_code_t status);

void mqtt_send_tornado_rawdata(uint8_t data[], char su_id[], char gw_id[]);

void mqtt_send_tornado_data(message_type_v2_t event, status_code_t status, uint8_t data[], char su_id[], char gw_id[]);

void mqtt_send_tornado_factory(message_type_v2_t event, status_code_t status,char msg[], char su_id[], char gw_id[]);

void mqtt_send_tornado_factory_json(message_type_v2_t event, cJSON *data);

void send_heartbeat_task(void *pvParameters);

// void refresh_access_token_task(void *pvParameters);
#ifdef __cplusplus
}
#endif

#endif