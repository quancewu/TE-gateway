#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_pthread.h"
#include "cJSON.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_netif.h"

#include "esp_tls.h"
#include "esp_http_client.h"

#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_ota_ops.h"

#include "websocket_server.h"
#include "wifi_manager.h"
#include "esp_storage.h"
#include "esp_utils.h"
#include "te-ws-sv.h"
#include "TE_mqtt.h"
#include "display-service.h"
#include "uart_service.h"

extern "C" {

static const char *TAG = "main";

#if CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
#define LEDC_LS_CH0_GPIO       (18)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (19)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (100)
#define LEDC_TEST_FADE_TIME    (3000)

/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code! This is an example on how you can integrate your code with wifi-manager
 */
void monitoring_task(void *pvParameter)
{   
    time_t now;
    struct tm timeinfo;
    int timer_after_start = 0;
    
    for(;;){
        ESP_LOGI(TAG, "free heap: %d",esp_get_free_heap_size());
        time(&now);
        localtime_r(&now, &timeinfo);
        timer_after_start++;
        if (!(timeinfo.tm_year < (2016 - 1900))){
            if (timeinfo.tm_hour == 16 && timer_after_start>400){
                ESP_LOGI(TAG, "Start restart daily");
                esp_restart();
                
            }
            else if (timeinfo.tm_hour%2 == 0 &&
                        timeinfo.tm_min == 0 &&
                        timeinfo.tm_sec <= 10){
                // time_sync_lrgw();
                esp_timesync_start();
            }
        }
        vTaskDelay( pdMS_TO_TICKS(10000) );
    }
}

void app_main(void)
{
    esp_storage_init();
    esp_log_level_set(TAG, ESP_LOG_INFO);

    const esp_partition_t *partition = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Currently running partition: %s", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
    }
    }
    esp_app_desc_t app_info;
    if (esp_ota_get_partition_description(partition, &app_info) == ESP_OK)
    {
        char *device_ver = (char*)malloc(50);
        strcpy(device_ver, "Compile ver: ");
        strcat(device_ver, app_info.date);
        strcat(compile_ver, device_ver);
        free(device_ver);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    initialize_uart();

    if (tornadoedge_configs_v1.lcd_1602_mode){
        xTaskCreate(&lcd1602_task, "lcd1602_task", 4096, NULL, 5, NULL);
    }

    /* start the wifi manager */
	wifi_manager_start();

    ws_server_start();

    ESP_LOGI(TAG, "Gateway ID: %s",tornadoedge_auth_settings_v2.auth_client_id);
    if (strcmp ((char*)tornadoedge_auth_settings_v2.auth_client_id,"te-xx-000001") == 0){
        ESP_LOGI(TAG, "set default sta ssid password");
        set_default_sta_ssid_password();
    }

    wifi_manager_mqtt_event_handler_init();

    esp_timesync_start();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    display_service_init();

    xTaskCreate(&server_task,"server_task",3000,NULL,9,NULL);
    xTaskCreate(&server_handle_task,"server_handle_task",3000,NULL,6,NULL);
    xTaskCreate(&count_task, "count_task", 2048, NULL, 2, NULL);

    xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);

    xTaskCreate(send_heartbeat_task, "send_heartbeat_task", 1024*2, NULL, 4, NULL);

    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 1024*3, NULL, 1, NULL, 1);

    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*3, NULL, configMAX_PRIORITIES, NULL);

    xTaskCreate(send_aerobox_heartbeat_task, "send_aerobox_heartbeat_task", 1024*2, NULL, 4, NULL);

    while (true)
    {
        blink_led_B(1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    

}

}
