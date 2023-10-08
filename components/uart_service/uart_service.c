#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "cJSON.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "uart_service.h"
#include "wifi_manager.h"
#include "display-service.h"
// #include "ws2812_service.h"
#include "TE_mqtt.h"
// #include "sd_card.h"
// #include "sd_service.h"
#include "esp_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

static const char *TAG = "uart-service";

// Aerobox 2021 2023
#define BLINK_GPIO_G 2 //25
#define BLINK_GPIO_R 23 //26
#define BLINK_GPIO_B 4 //26
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16) //#purple
// Aerobox_2019
// #define RXD_PIN (GPIO_NUM_14)
// #define TXD_PIN (GPIO_NUM_19)

static const int RX_BUF_SIZE = 1024;
SemaphoreHandle_t uart_async_json_mutex = NULL;
char *aerobox_data_json = NULL;
uint8_t DEVICE_LIVE_DATA[32];
bool DEVICE_DATA_STATUS = false;
bool SET_AEROBOX_ID = false;

time_t now;
struct tm timeinfo;

struct data_model_t recode[10];
struct lr_gw_pair_model_t lr_gw_pair;
uint8_t receive_id[8];
 
void initialize_uart()
{
    ESP_LOGI(TAG, "initialize_uart started");
    gpio_pad_select_gpio(BLINK_GPIO_G);
    gpio_set_direction(BLINK_GPIO_G, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO_R);
    gpio_set_direction(BLINK_GPIO_R, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO_B);
    gpio_set_direction(BLINK_GPIO_B, GPIO_MODE_OUTPUT);

    const uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    for (int i=0;i<10;i++){
        for (int j=0;j<6;j++){
            recode[i].time_record[j] = 0xFF;
        }
        for (int j=0;j<4;j++){
            recode[i].lr_gw_data[j] = 0xFF;
        }
        recode[i].retry = 0x00;
        recode[i].request_state = 0x00;
    }
    // default settings means pairing same
    lr_gw_pair.status = true;
    lr_gw_pair.sync_from_server = false;
    lr_gw_pair.pair_quantity = 0xFF;
}

void blink_led_G(int nums) {
    for (int i=0;i<nums;i++){
        gpio_set_level(BLINK_GPIO_G, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO_G, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void blink_led_R(int nums) {
    for (int i=0;i<nums;i++){
        gpio_set_level(BLINK_GPIO_R, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO_R, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void blink_led_B(int nums) {
    for (int i=0;i<nums;i++){
        gpio_set_level(BLINK_GPIO_B, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO_B, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void blink_led(int nums, int index, char* colors) {
    for (int i=0;i<nums;i++){
        // set_index(index);
        // set_color(colors);
        vTaskDelay(30 / portTICK_PERIOD_MS);
        // set_index(index);
        // set_color((char*)("#000000"));
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}

void set_led(int nums, int index, bool methods, char* colors) {
    for (int i=0;i<nums;i++){
        // set_index(index);
        // set_color(colors);
        vTaskDelay(30 / portTICK_PERIOD_MS);
        // set_index(index);
        // set_color((char*)("#000000"));
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}

void set_sync_from_server(bool status) {
    lr_gw_pair.sync_from_server = status;
}

void time_sync_lrgw()
{
    ESP_LOGI(TAG, "Notification of a 2 hours time synchronization event");
    TIME_SYNC = true;
}

int sendData_byte(const char* logName, uint8_t* data, uint8_t len, uint8_t cmd[2])
{
    int i, sum = 0;
    data[9] = cmd[0];data[10] = cmd[1];
    for(i=1; i<len-2; i++){
        sum += data[i];
    }
    data[len-2] = sum%255;
    blink_led_R(1);
    char *strdata = phex(data, 32);
    printf("send data: %s\n",strdata);
    free(strdata);
    int txBytes = 0;
    for(i=0;i<len;i++){
        int sent = uart_write_bytes(UART_NUM_1, &data[i], 1);
        txBytes += sent;
    }
    // ESP_LOGI(logName, "Wrote %d bytes CMD %2X %2X CKS %d", txBytes, cmd[0], cmd[1],data[len-2]);
    return txBytes;
}

bool uart_async_lock_json_buffer(TickType_t xTicksToWait){
	if(uart_async_json_mutex){
		if( xSemaphoreTake( uart_async_json_mutex, xTicksToWait ) == pdTRUE ) {
			return true;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}

}
void uart_async_unlock_json_buffer(){
	xSemaphoreGive( uart_async_json_mutex );
}


void aerobox_clear_live_data_json(){
    strcpy(aerobox_data_json, "[]\n");
}

void aerobox_generate_access_points_json(uint8_t* data){
    
    strcpy(aerobox_data_json, "");

    char one_var[80];
    const char data_str[] = "{\"id\":\"%s\",\"temp\":%d,\"rh\":%d,\"pm25\":%d,\"pm10\":%d,\"co2\":%d}";
    char* aerobox_pair_json = (char*)aerobox_auth_settings.aerobox_pair[0];
    int16_t  temperature = ((uint16_t)data[13] << 8) | data[12];
    uint16_t          rh = ((uint16_t)data[15] << 8) | data[14];
    uint16_t        pm25 = ((uint16_t)data[21] << 8) | data[20];
    uint16_t        pm10 = ((uint16_t)data[25] << 8) | data[24];
    uint16_t         co2 = ((uint16_t)data[29] << 8) | data[28];
    snprintf(one_var, (size_t)80,data_str,
            aerobox_pair_json,
            temperature,
            rh,
            pm25,
            pm10,
            co2);
    strcat(aerobox_data_json, one_var);
    // printf("jsondata: %s\n",aerobox_data_json);

}

char* uart_async_get_data_json(){
	return aerobox_data_json;
}

void send_aerobox_heartbeat_task(void *pvParameters){
    vTaskDelay((tornadoedge_configs_v1.local_muticast_resolution*1000*30) / portTICK_PERIOD_MS);
    while (true){
        for (int j=0; j<16; j++){
            if (strcmp((char*)aerobox_auth_settings.aerobox_pair[j],"") && tornadoedge_configs_v1.aerobox_gw_mode){
                if (recode[j].retry >= 0x0A) {
                    mqtt_send_tornado_factory(AEROBOX_HEARTBEAT,NO_RESPONSE,"",(char*)aerobox_auth_settings.aerobox_pair[j],"");
                }
                else {
                    mqtt_send_tornado_factory(AEROBOX_HEARTBEAT,FINE,"",(char*)aerobox_auth_settings.aerobox_pair[j],"");
                }
            }
            if (strcmp((char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],"") && tornadoedge_configs_v1.lcd_1602_mode){
                if (recode[j].retry >= 0x1F) {
                    blink_led_B(2);
                    mqtt_send_tornado_factory(SG_HEARTBEAT,NO_RESPONSE,"",(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],"");
                    set_sq_data_display_service_status(&j, false);
                }
                else {
                    blink_led_B(1);
                    mqtt_send_tornado_factory(SG_HEARTBEAT,FINE,"",(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],"");
                }
            }
            // else{
            //     set_sq_data_display_service_disable(&j);
            // }
        }
        vTaskDelay((tornadoedge_configs_v1.heartbeat_resolution*1000) / portTICK_PERIOD_MS);
    }
}

/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code! This is an example on how you can integrate your code with wifi-manager
 */
void uart_tx_task(void *pvParameter)
{   
    char strftime_buf[64];
    uint8_t send_data[32] = {   0x55, 0xAB, 0x19, 0x05, 0x01,
                                0xDA, 0x19, 0x05, 0x01,
                                0xc0, 0x01, 0x14, 
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0xab, 0xc0, 0x00, 0x24, 0x01, 0x02, 0xcc, 0xed};
    uint8_t cmd[2] = {0xC1, 0x00};
    static const char *TX_TASK_TAG = "TX_TASK";
    uint8_t time_sync_counter = 0;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // broadcast to Aerobox
    send_data[1] = 0x0D; send_data[2] = 0x0D; send_data[3] = 0x0D; send_data[4] = 0x0D;
    send_data[5] = 0x05; send_data[6] = 0x05; send_data[7] = 0x05; send_data[8] = 0x05;

    // send_data[24] = (timeinfo.tm_year-100); send_data[25] = (timeinfo.tm_mon+1); send_data[26] = (timeinfo.tm_mday);
    // send_data[27] = (timeinfo.tm_hour    ); send_data[28] = (timeinfo.tm_min  ); send_data[29] = (timeinfo.tm_sec );
    // cmd[0] = 0xC0;  cmd[1] =  0x00;
    // sendData_byte(TX_TASK_TAG, send_data, sizeof(send_data), cmd);  
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

	for(;;){
        uint8_t device_code[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        char* auth_client_id = (char*)tornadoedge_auth_settings_v2.hex_client_id;
        // ESP_LOGI(TX_TASK_TAG,"GW: %s",tornadoedge_auth_settings.ecu_gw_pair);
        for (int i = 0; i < 4; i++){
            sscanf(auth_client_id, "%2hhx", &device_code[i]);
            send_data[5+i] = device_code[i];
            auth_client_id += 2;
        }
        if (TIME_SYNC){
            ESP_LOGI(TX_TASK_TAG, "Trigger time sync");
            cmd[0] = 0xC0;  cmd[1] =  0x01;
            // TIME_SYNC = false;
            char* ecu_pair = (char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[0];
            ESP_LOGI(TX_TASK_TAG,"send to %s",tornadoedge_sensor_unit_v2.sensor_unit_pairs[0]);
            for (int i = 0; i < 4; i++){
                sscanf(ecu_pair, "%2hhx", &device_code[i]);
                send_data[4-i] = device_code[i];
                ecu_pair += 2;
            }
            send_data[1] = 0x01;
            // send_data[2] = 0x04;
            send_data[2] = 0x04;
            send_data[3] = 0x17;
            send_data[4] = 0xc1;
            send_data[5] = 0x01; send_data[6] = 0x04; send_data[7] = 0x17; send_data[8] = 0xe1; 
            time(&now);
            localtime_r(&now, &timeinfo);
            send_data[15] = 0x00;
            send_data[19] = 0x00;
            send_data[14] = (timeinfo.tm_year-100); send_data[13] = (timeinfo.tm_mon+1); send_data[12] = (timeinfo.tm_mday);
            send_data[18] = (timeinfo.tm_hour    ); send_data[17] = (timeinfo.tm_min  ); send_data[16] = (timeinfo.tm_sec );

            send_data[24] = (timeinfo.tm_year-100); send_data[25] = (timeinfo.tm_mon+1); send_data[26] = (timeinfo.tm_mday);
            send_data[27] = (timeinfo.tm_hour    ); send_data[28] = (timeinfo.tm_min  ); send_data[29] = (timeinfo.tm_sec );
            if (timeinfo.tm_year < (2016 - 1900)) {
                time_sync_counter++;
                ESP_LOGW(TX_TASK_TAG, "ntp not set for %d times", time_sync_counter);
                if (time_sync_counter>10) {
                    TIME_SYNC = false;
                    time_sync_counter = 0;
                }
                
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            else{
                // TIME_SYNC = false;
                time_sync_counter++;
                ESP_LOGW(TX_TASK_TAG, "LR GW no response for %d times", time_sync_counter);
                if (time_sync_counter>10) {
                    TIME_SYNC = false;
                    time_sync_counter = 0;
                }
                sendData_byte(TX_TASK_TAG, send_data, sizeof(send_data), cmd);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            continue;
        }
        if (SET_AEROBOX_ID){
            cmd[0] = 0xC1;  cmd[1] =  0x02;
            SET_AEROBOX_ID = false;
            // recived from aerobox ID name
            for (int i=1;i<9;i++){
                send_data[i] = receive_id[i-1];
            }
            char* ecu_pair = (char*)aerobox_auth_settings.aerobox_pair[0];
            ESP_LOGI(TX_TASK_TAG,"set %s to Aerobox",aerobox_auth_settings.aerobox_pair[0]);
            for (int i = 0; i < 4; i++){
                sscanf(ecu_pair, "%2hhx", &device_code[i]);
                send_data[12+i] = device_code[i];
                ecu_pair += 2;
            }
            send_data[16] = 0xDA; send_data[17] = 0x13; send_data[18] = 0x05; send_data[19] = 0x01;
            time(&now);
            localtime_r(&now, &timeinfo);
            send_data[24] = (timeinfo.tm_year-100); send_data[25] = (timeinfo.tm_mon+1); send_data[26] = (timeinfo.tm_mday);
            send_data[27] = (timeinfo.tm_hour    ); send_data[28] = (timeinfo.tm_min  ); send_data[29] = (timeinfo.tm_sec );
            sendData_byte(TX_TASK_TAG, send_data, sizeof(send_data), cmd);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            continue;
        }
        if (SET_RFCH){
            cmd[0] = 0xC0;  cmd[1] =  0x06;
            SET_RFCH = false;
            ESP_LOGI(TX_TASK_TAG,"set RF CH");
            send_data[12] = tornadoedge_configs_v1.gw_rfch;
            send_data[13] = 0x00; send_data[14] = 0x00; send_data[15] = 0x00;
            send_data[1] = 0x01; send_data[2] = 0x04; send_data[3] = 0x17; send_data[4] = 0xc1;
            send_data[5] = 0x01; send_data[6] = 0x04; send_data[7] = 0x17; send_data[8] = 0xe1; 
            time(&now);
            localtime_r(&now, &timeinfo);
            send_data[24] = (timeinfo.tm_year-100); send_data[25] = (timeinfo.tm_mon+1); send_data[26] = (timeinfo.tm_mday);
            send_data[27] = (timeinfo.tm_hour    ); send_data[28] = (timeinfo.tm_min  ); send_data[29] = (timeinfo.tm_sec );
            sendData_byte(TX_TASK_TAG, send_data, sizeof(send_data), cmd);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;

        }
        // for (int j=0; j<10; j++){
        //     if (strcmp((char*)aerobox_auth_settings.aerobox_pair[j],"")){
        //         char* ecu_pair = (char*)aerobox_auth_settings.aerobox_pair[j];
        //         ESP_LOGI(TX_TASK_TAG,"send to %s",aerobox_auth_settings.aerobox_pair[j]);
        //         for (int i = 0; i < 4; i++){
        //             sscanf(ecu_pair, "%2hhx", &device_code[i]);
        //             send_data[1+i] = device_code[i];
        //             ecu_pair += 2;
        //         }
        //         time(&now);
        //         localtime_r(&now, &timeinfo);
        //         // strftime(strftime_buf, sizeof(strftime_buf), "%x %X", &timeinfo);
        //         // ESP_LOGI(TX_TASK_TAG, "The current date/time in UTC+0 is: %s", strftime_buf);
        //         cmd[0] = 0xC0;  cmd[1] = 0x00; 
        //         send_data[24] = (timeinfo.tm_year-100); send_data[25] = (timeinfo.tm_mon+1); send_data[26] = (timeinfo.tm_mday);
        //         send_data[27] = (timeinfo.tm_hour    ); send_data[28] = (timeinfo.tm_min  ); send_data[29] = (timeinfo.tm_sec );
        //         sendData_byte(TX_TASK_TAG, send_data, sizeof(send_data), cmd);
        //         // recode[j].request_state = 0x01;
        //         if (recode[j].retry < 0xFF) recode[j].retry++;
                
        //         vTaskDelay(500 / portTICK_PERIOD_MS);
        //     }
        // }
        for (int j=0; j<10; j++){
            if (strcmp((char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],"")){
                if (recode[j].retry < 0xFF) recode[j].retry++;
            }
            else{
                set_sq_data_display_service_disable(&j);
            }
        }

        // for (int j=0; j<10; j++){
        //     if (strcmp((char*)aerobox_auth_settings.aerobox_pair[j],"")){
        //         if (recode[j].request_state == 0x01){
        //             if (recode[j].retry < 0xFF) recode[j].retry++;
        //             if (recode[j].retry < 3){
        //                 // set_led(2, j+2, true, "#000025");
        //                 blink_led_R(3);
        //                 vTaskDelay(100 / portTICK_PERIOD_MS);
        //                 mqtt_send_tornado_factory(TORNADOEDGE_HEARTBEAT,WARNING,"",
        //                             (char*)aerobox_auth_settings.aerobox_pair[j],
        //                             (char*)tornadoedge_auth_settings.ecu_gw_pair
        //                             );
        //             }
        //             else{
        //                 // set_led(2, j+2, true, "#8e00a6");
        //                 blink_led_R(3);
        //                 vTaskDelay(100 / portTICK_PERIOD_MS);
        //                 mqtt_send_tornado_factory(TORNADOEDGE_HEARTBEAT,FATEL,"",
        //                             (char*)aerobox_auth_settings.aerobox_pair[j],
        //                             (char*)tornadoedge_auth_settings.ecu_gw_pair
        //                             );
        //             }
        //         }
                
        //         vTaskDelay(100 / portTICK_PERIOD_MS);
        //     }
        // }
        // char* ecu_pair = (char*)tornadoedge_auth_settings.ecu_gw_pair;
        // ESP_LOGI(TX_TASK_TAG,"send to %s",tornadoedge_auth_settings.ecu_gw_pair);
        // for (int i = 0; i < 4; i++){
        //     sscanf(ecu_pair, "%2hhx", &device_code[3-i]);
        //     send_data[15-i] = device_code[3-i];
        //     ecu_pair += 2;
        // }
        // time(&now);
        // localtime_r(&now, &timeinfo);
        // strftime(strftime_buf, sizeof(strftime_buf), "%x %X", &timeinfo);
        // ESP_LOGI(TX_TASK_TAG, "The current date/time in UTC+0 is: %s", strftime_buf);
        // cmd[0] = 0xC1;  cmd[1] = 0x06; 
        // send_data[24] = (timeinfo.tm_year-100); send_data[25] = (timeinfo.tm_mon+1); send_data[26] = (timeinfo.tm_mday);
        // send_data[27] = (timeinfo.tm_hour    ); send_data[28] = (timeinfo.tm_min  ); send_data[29] = (timeinfo.tm_sec );
        // // sendData_byte(TX_TASK_TAG, send_data, sizeof(send_data), cmd);                
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        int timedelay = 0;
        if (tornadoedge_configs_v1.su_query_resolution>tornadoedge_configs_v1.local_muticast_resolution){
            timedelay = tornadoedge_configs_v1.local_muticast_resolution;
        }
        else{
            timedelay = tornadoedge_configs_v1.su_query_resolution;
        }
        vTaskDelay((timedelay*1000) / portTICK_PERIOD_MS);
	}
}

void uart_rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uart_async_json_mutex = xSemaphoreCreateMutex();
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    aerobox_data_json = (char*)malloc(200); 
    uint8_t index = {0xFF};
    int counter = 0;
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 32, 20 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            if (rxBytes == 32 && data[31] == 0xED){
                // read recive device code
                uint8_t code[4];
                int sum = 0;
                for(int i=1; i<rxBytes-2; i++){
                    sum += data[i];
                }
                // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                if (sum%255 == data[rxBytes-2]){
                    // printf("check sum correct %d %d\n",sum%255, data[rxBytes-2]);
                }
                else{
                    printf("check sum not correct %d %d\n",sum%255, data[rxBytes-2]);
                }
                // counter ++;
                for(int i=5; i<9; i++){
                    code[8-i] = data[i];
                    receive_id[i-5] = data[i];
                }
                for(int i=1; i<5; i++){
                    receive_id[i+3] = data[i];
                }
                code[3] &= ~(0x0F);
                char* str_su_code = phex(code, 4);
                str_su_code[(2 * 4)-1] = '\0';
                char* output = phex(data, 32);
                ESP_LOGI(RX_TASK_TAG, "recived: %s",str_su_code);
                index = 0xFF;
                for(int i=0; i<16; i++){
                    if (!strcmp((char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[i],str_su_code)){
                        index = i;
                        break;
                        // ESP_LOGI(RX_TASK_TAG, "comparing %d code: %s",index, (char*)tornadoedge_auth_settings.ecu_pairs[i]);
                    }
                }
                bool untag = true;
                int order = 0;
                switch (((uint16_t)data[9] << 8) | data[10])
                {
                case 0xc001:
                    ESP_LOGI(RX_TASK_TAG, "C0 01");
                    if (data[20] == 0x22){
                        TIME_SYNC = true;
                        ESP_LOGI(RX_TASK_TAG, "RTC Sync Request");
                        blink_led_R(2);
                    }
                    else {
                        ESP_LOGI(RX_TASK_TAG, "RTC Sync accepted");
                        TIME_SYNC = false;
                        SET_RFCH = true;
                        blink_led_R(3);
                    }
                    break;
                case 0xc006:
                    ESP_LOGI(RX_TASK_TAG, "C0 06");
                    ESP_LOGI(RX_TASK_TAG, "RF Ch Report");
                    ESP_LOGI(RX_TASK_TAG, "RF chaneel No.%d", data[12]);
                    blink_led_R(2);
                    break;
                case 0xc100:
                    ESP_LOGI(RX_TASK_TAG, "C1 00");
                    ESP_LOGI(RX_TASK_TAG, "Hour Data Report");
                    blink_led_R(2);
                    for (int j=0; j<10; j++){
                        if (strcasecmp( (char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],str_su_code) == 0){
                            recode[j].request_state = 0x00;
                            recode[j].retry = 0x00;
                            untag = false;
                            order = j;
                            break;
                        }
                    }
                    if (untag){
                        mqtt_send_tornado_data(SG_REPORT_JSON_DATA,ILLEGAL,data,
                                str_su_code,
                                (char*)tornadoedge_auth_settings_v2.hex_client_id);
                    }
                    else{
                        mqtt_send_tornado_data(SG_REPORT_JSON_DATA,FINE,data,
                                str_su_code,
                                (char*)tornadoedge_auth_settings_v2.hex_client_id);
                        set_sq_data_display_service(&order, str_su_code, data);
                    }   
                    break;
                case 0xc101:
                    ESP_LOGI(RX_TASK_TAG, "C1 01");
                    ESP_LOGI(RX_TASK_TAG, "30 Second Data Report");
                    blink_led_R(2);
                    for (int j=0; j<10; j++){
                        if (strcasecmp( (char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],str_su_code) == 0){
                            recode[j].request_state = 0x00;
                            recode[j].retry = 0x00;
                            untag = false;
                            order = j;
                            break;
                        }
                    }
                    if (untag){
                        mqtt_send_tornado_data(SG_SEQ_JSON_DATA,ILLEGAL,data,
                                str_su_code,
                                (char*)tornadoedge_auth_settings_v2.hex_client_id);
                        // set_sq_data_display_service(j, str_su_code, data);
                    }
                    else{
                        mqtt_send_tornado_data(SG_SEQ_JSON_DATA,FINE,data,
                                str_su_code,
                                (char*)tornadoedge_auth_settings_v2.hex_client_id);
                        set_sq_data_display_service(&order, str_su_code, data);
                    }   
                    break;
                default:
                    ESP_LOGI(RX_TASK_TAG, "%4X",((uint16_t)data[9] << 8) | data[10]);
                    ESP_LOGI(RX_TASK_TAG, "unknow cmd");
                    blink_led_R(5);
                    break;
                }
                
                // if (strcasecmp( (char*)aerobox_auth_settings.aerobox_pair[0],strcode) != 0 &&
                //     (strcmp((char*)aerobox_auth_settings.aerobox_pair[1],"") == 0)){
                //     ESP_LOGI(RX_TASK_TAG, "Store: %s",(char*)aerobox_auth_settings.aerobox_pair[0]);
                //     ESP_LOGI(RX_TASK_TAG, "recived: %s",strcode);
                //     ESP_LOGI(RX_TASK_TAG, "Own Aerobox ID is different reset procces start");
                //     SET_AEROBOX_ID = true;
                // }
                // if (strcasecmp( (char*)aerobox_auth_settings.aerobox_pair[0],strcode) == 0) {
                //     recode[0].request_state = 0x00;
                //     recode[0].retry = 0x00;
                // }

                // blink_led_R(2);
                // ESP_LOGI(RX_TASK_TAG, "recive device code: %s", strcode);
                // // aerobox_generate_access_points_json(data);
                // if (tornadoedge_configs_v1.local_muticast_mode) set_to_display_service(data);
                // if (tornadoedge_configs_v1.su_query_resolution/tornadoedge_configs_v1.local_muticast_resolution==counter){
                //     counter = 0;
                //     mqtt_send_tornado_factory(AEROBOX_RAW_DATA,FINE,output,
                //                     strcode,
                //                     (char*)tornadoedge_auth_settings_v2.hex_client_id);
                // }
                free(str_su_code);
                free(output);
            }
            else{
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 50 / portTICK_RATE_MS);
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            }
        }
        else{
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

#ifdef __cplusplus
}
#endif