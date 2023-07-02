#ifndef te_gw_settup_H_INCLUDED
#define te_gw_settup_H_INCLUDED


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

#ifdef __cplusplus
extern "C" {
#endif

typedef enum te_gw_setup_mode_t {
    SETUP_NONE_CODE = 0,
    SETUP_MQTT_MODE = 1,
    SETUP_WS_MODE = 2,
    SETUP_NONE_MODE = 3,
}te_gw_setup_mode_t;

esp_err_t parameter_update(char *topic,char *payload, te_gw_setup_mode_t setup_mode);

#ifdef __cplusplus
}
#endif

#endif
