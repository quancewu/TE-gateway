#ifndef DISPLAY_SERVICE_H_INCLUDED
#define DISPLAY_SERVICE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "cJSON.h"
#include "esp_storage.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// typedef struct display_models_t 
// {
//     esp_mqtt_client_handle_t client;
//     TaskHandle_t mqtt_task_manager;
//     char *name;
//     QueueHandle_t message_queue;
// } mqtt_models_t;

typedef enum display_message_code_t {
    DISPLAY_SEND_RAW_DATA_EVENT = 1,
    DISPLAY_SEND_MQTT_DATA_EVENT = 2,
    DISPLAY_TIMEOUT_EVENT = 3,
    DISPLAY_DESTORY_EVENT = 4,
    DISPLAY_STOP_EVENT = 5
} display_message_code_t;

typedef struct display_message_t
{
    // mqtt_models_t *mqtt_model;
    display_message_code_t code;
    void *param;
    uint8_t device_id[10];
    double cJSON_temp;
    double cJSON_rh;
    double cJSON_pm2_5;
    double cJSON_pm10;
    double cJSON_co2;
    aerobox_data_model_v1_t *receive_data;
} display_message_t;

void display_send_mqtt_factory(char *deviceId, double temp, double rh, double pm2_5, double pm10,double co2);

void display_service_init(void);

void set_to_display_service(uint8_t* data);


void set_sq_data_display_service_disable(int *pos);

void set_sq_data_display_service_status(int *pos, bool status);

void set_sq_data_display_service(int *pos,const char * msg_id, uint8_t* raw_data);

void lcd1602_task(void * pvParameter);

#ifdef __cplusplus
}
#endif

#endif