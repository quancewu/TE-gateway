#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include "driver/i2c.h"
#include "rom/uart.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"

#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "smbus.h"
#include "i2c-lcd1602.h"

#include "wifi_manager.h"
#include "TE_mqtt.h"
#include "esp_storage.h"

#include "display-service.h"

#ifdef __cplusplus
extern "C" {
#endif

QueueHandle_t display_queue;
TaskHandle_t display_queue_handler;
struct aerobox_data_model_v1_t udp_recv_data;
struct aerobox_data_model_v1_t udp_send_data;
struct sq_data_model_v2_t sq_data[16];
struct te_gw_model_v1_t udp_discover_data;
bool local_data_refreshed = false;
uint8_t timeout_counter = 0;


static const char *TAG = "display-service";
#define TXD_PIN (GPIO_NUM_15)
#define RXD_PIN (GPIO_NUM_14)
static const int RX_BUF_SIZE = 1024;

// LCD1602
#define LCD_NUM_ROWS               2
#define LCD_NUM_COLUMNS            32
#define LCD_NUM_VISIBLE_COLUMNS    16

// LCD2004
//#define LCD_NUM_ROWS               4
//#define LCD_NUM_COLUMNS            40
//#define LCD_NUM_VISIBLE_COLUMNS    20

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22

int display_temp = 0;
int display_rh = 0;
int display_pm2_5 = 0;
int display_pm10 = 0;
int display_co2 = 0;
float temperature = 0;
float rh = 0;

char tempstr[3];
char rhstr[3];
char pmstr[4];
char co2str[5];

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

static uint8_t _wait_for_user(void)
{
    uint8_t c = 0;
    vTaskDelay(1000 / portTICK_RATE_MS);
    return c;
}

void initialize_display_uart()
{
    ESP_LOGI(TAG, "initialize_uart started");

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void set_to_display_service(uint8_t* data)
{
    memcpy(udp_send_data.aerobox_data, data, 64);
    local_data_refreshed = true;
}

void set_sq_data_display_service_disable(int *pos)
{
    local_data_refreshed = true;
    sq_data[*pos].weight        = -999;
    sq_data[*pos].raw           = -999;
    sq_data[*pos].battery       = -999;
    sq_data[*pos].rfch          = 0xFF;
    sq_data[*pos].rforder       = 0xFF;
    sq_data[*pos].error_code    = 0xFF;
    sq_data[*pos].k1            = -999;
    sq_data[*pos].k2            = -999;
}

void set_sq_data_display_service_status(int *pos, bool status)
{
    local_data_refreshed = true;
    if (!status){
        sq_data[*pos].weight        = -999;
        sq_data[*pos].raw           = -999;
        sq_data[*pos].battery       = -999;
        sq_data[*pos].rfch          = 0xFF;
        sq_data[*pos].rforder       = 0xFF;
        sq_data[*pos].error_code    = 0xEF;
        sq_data[*pos].k1            = -999;
        sq_data[*pos].k2            = -999;
    }
}

void set_sq_data_display_service(int *pos,const char * msg_id, uint8_t* raw_data)
{
    local_data_refreshed = true;
    memcpy(sq_data[*pos].msu_id, msg_id, 6);
    if (raw_data[10]==0x00){
        sq_data[*pos].weight = ((uint16_t)raw_data[13] << 8) | raw_data[12];
        sq_data[*pos].raw = ((uint16_t)raw_data[15] << 8) | raw_data[14];
        sq_data[*pos].battery = raw_data[21];
        sq_data[*pos].rfch = raw_data[20];
        sq_data[*pos].rforder = raw_data[5] & ~(0xF0);
        sq_data[*pos].error_code = raw_data[23];
        sq_data[*pos].k1 = (int16_t)(((uint16_t)raw_data[17] << 8) | raw_data[16]);
        sq_data[*pos].k2 = (int16_t)(((uint16_t)raw_data[19] << 8) | raw_data[18]);
    }
    else if (raw_data[10]==0x01){
        sq_data[*pos].weight = (int16_t)((uint16_t)raw_data[21] << 8) | raw_data[20];
        sq_data[*pos].rforder = raw_data[5] & ~(0xF0);
    }
}

void display_stream_handler(void *pvParameters)
{
    display_message_t message;
    BaseType_t xStatus;
    ESP_LOGI(TAG, "queue in sender: %p", (void *)&display_queue);
    bool is_recoded;
    uint8_t aerobox_code[4];
    uint8_t init_display[14] = {0x55 ,0xAA, 0x01, 0x0A, 0xC0, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x58, 0x58, 0x73};


    ESP_LOGI(TAG,"init_display -> %s ",(char*)init_display);
    // for(int i=0;i<14;i++){
    //     int sent = uart_write_bytes(UART_NUM_2, &init_display[i], 1);
    //     // txBytes += sent;
    // }
    vTaskDelay(500 / portTICK_PERIOD_MS);

    for (;;){
        xStatus = xQueueReceive(display_queue, &(message), 60000/portTICK_RATE_MS);
        
        if (xStatus == pdPASS) {
            if (message.code == DISPLAY_DESTORY_EVENT){
                // destory_mqtt();
                vTaskDelay(3000 / portTICK_RATE_MS); 
            }
            if (message.code == DISPLAY_STOP_EVENT){
                break;
            }
            switch (message.code){
                case DISPLAY_SEND_RAW_DATA_EVENT:{
                    is_recoded = false;
                    for(int i=5; i<9; i++){
                        aerobox_code[i-5] = message.receive_data->aerobox_data[i];
                    }
                    char* strcode = phex(aerobox_code, 4);
                    for (int i=0; i<10; i++){
                        if (strcmp((char*)tornadoedge_display_unit_v2.display_pairs[i],"")){
                            if (strcasecmp( strcode,
                                    (char*)tornadoedge_display_unit_v2.display_pairs[i]) == 0){
                                is_recoded = true;
                                break;
                            }
                            
                        }
                    }
                    // check is display eng mode will not filter display ID
                    if (!is_recoded && !tornadoedge_configs_v1.display_eng_mode) {
                        free(strcode);    
                        break;
                    }
                    if (tornadoedge_configs_v1.display_eng_mode){
                        memcpy(display_data_model.temp,&strcode[0],sizeof(display_data_model.temp));
                        if (is_recoded){
                            display_data_model.pm_color = color_green;
                        }
                        else{
                            display_data_model.pm_color = color_red;
                        }
                        memcpy(display_data_model.pm2_5,"LDS",sizeof(display_data_model.pm2_5));
                        memcpy(display_data_model.rh,&strcode[2],sizeof(display_data_model.rh));
                        display_data_model.co2_color = color_green;
                        memcpy(display_data_model.co2,&strcode[4],sizeof(display_data_model.co2));
                        int data_len = sizeof(display_data_model);
                        int txBytes = 0;                    
                        uint8_t data[27];
                        memcpy(data, &display_data_model, sizeof(display_data_model));
                        ESP_LOGI(TAG,"data -> %s",(char*)data);
                        for(int i=0;i<data_len;i++){
                            int sent = uart_write_bytes(UART_NUM_2, &data[i], 1);
                            txBytes += sent;
                        }
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                    }
                    free(strcode);

                    temperature = ((uint16_t)message.receive_data->aerobox_data[13] << 8) | message.receive_data->aerobox_data[12];
                    display_temp = round(temperature/100);
                    rh = ((uint16_t)message.receive_data->aerobox_data[15] << 8) | message.receive_data->aerobox_data[14];
                    display_rh = round(rh/100);
                    display_pm2_5 = ((uint16_t)message.receive_data->aerobox_data[21] << 8) | message.receive_data->aerobox_data[20];
                    display_pm10 = ((uint16_t)message.receive_data->aerobox_data[25] << 8) | message.receive_data->aerobox_data[24];
                    display_co2 = ((uint16_t)message.receive_data->aerobox_data[29] << 8) | message.receive_data->aerobox_data[28];

                    if (display_temp>99 || display_temp<0){
                        display_data_model.t_color = color_red;
                        memcpy(display_data_model.temp,"//",sizeof(display_data_model.temp));
                    }
                    else{
                        display_data_model.t_color = color_green;
                        sprintf(tempstr,"%2d",display_temp);
                        ESP_LOGD(TAG, "temperature: %s",tempstr);
                        memcpy(display_data_model.temp,tempstr,sizeof(display_data_model.temp));
                    }
                    
                    if (display_rh>99 || display_rh<0){
                        display_data_model.rh_color = color_red;
                        memcpy(display_data_model.rh,"//",sizeof(display_data_model.rh));
                    }
                    else{
                        display_data_model.rh_color = color_green;
                        sprintf(rhstr,"%2d",display_rh);
                        ESP_LOGD(TAG, "RH: %s",rhstr);
                        memcpy(display_data_model.rh,rhstr,sizeof(display_data_model.rh));
                    }
                    
                    if (display_pm2_5>999 || display_pm2_5<0){
                        display_data_model.rh_color = color_red;
                        display_data_model.light_color = color_red;
                        memcpy(display_data_model.pm2_5,"///",sizeof(display_data_model.pm2_5));
                    }
                    else{
                        if (display_pm2_5>=55){
                            display_data_model.pm_color = color_red;
                            display_data_model.light_color = color_red;
                        }
                        else if (display_pm2_5>=35){
                            display_data_model.pm_color = color_yellow;
                            display_data_model.light_color = color_yellow;
                        }
                        else{
                            display_data_model.pm_color = color_green;
                            display_data_model.light_color = color_green;
                        }
                        sprintf(pmstr,"%3d",display_pm2_5);
                        ESP_LOGD(TAG, "PM2.5: %s",pmstr);
                        memcpy(display_data_model.pm2_5,pmstr,sizeof(display_data_model.pm2_5));
                    }

                    if (display_co2>9999 || display_co2<0){
                        display_data_model.co2_color = color_red;
                        memcpy(display_data_model.co2,"////",sizeof(display_data_model.co2));
                    }
                    else{
                        if (display_co2>=900){
                            display_data_model.co2_color = color_red;
                        }
                        else if (display_co2>=600){
                            display_data_model.co2_color = color_yellow;
                        }
                        else{
                            display_data_model.co2_color = color_green;
                        }
                        sprintf(co2str,"%4d",display_co2);
                        ESP_LOGD(TAG, "CO2: %s",co2str);
                        memcpy(display_data_model.co2,co2str,sizeof(display_data_model.co2));
                    }

                    int data_len = sizeof(display_data_model);
                    int txBytes = 0;                    
                    uint8_t data[27];
                    memcpy(data, &display_data_model, sizeof(display_data_model));
                    ESP_LOGI(TAG,"data -> %s",(char*)data);
                    timeout_counter = 0;
                    for(int i=0;i<data_len;i++){
                        int sent = uart_write_bytes(UART_NUM_2, &data[i], 1);
                        txBytes += sent;
                    }
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    break;
                }
                case DISPLAY_SEND_MQTT_DATA_EVENT:{
                    is_recoded = false;
                    for (int i=0; i<10; i++){
                        if (strcmp((char*)tornadoedge_display_unit_v2.display_pairs[i],"")){
                            if (strcasecmp( (char*)message.device_id,
                                    (char*)tornadoedge_display_unit_v2.display_pairs[i]) == 0){
                                is_recoded = true;
                                break;
                            }
                        }
                    }
                    // check is display eng mode will not filter display ID
                    if (!is_recoded && !tornadoedge_configs_v1.display_eng_mode) break;

                    if (tornadoedge_configs_v1.display_eng_mode){
                        memcpy(display_data_model.temp,&message.device_id[0],sizeof(display_data_model.temp));
                        if (is_recoded){
                            display_data_model.pm_color = color_green;
                        }
                        else{
                            display_data_model.pm_color = color_red;
                        }
                        memcpy(display_data_model.pm2_5,"CDS",sizeof(display_data_model.pm2_5));
                        memcpy(display_data_model.rh,&message.device_id[2],sizeof(display_data_model.rh));
                        display_data_model.co2_color = color_green;
                        memcpy(display_data_model.co2,&message.device_id[4],sizeof(display_data_model.co2));
                        int data_len = sizeof(display_data_model);
                        int txBytes = 0;                    
                        uint8_t data[27];
                        memcpy(data, &display_data_model, sizeof(display_data_model));
                        ESP_LOGI(TAG,"data -> %s",(char*)data);
                        for(int i=0;i<data_len;i++){
                            int sent = uart_write_bytes(UART_NUM_2, &data[i], 1);
                            txBytes += sent;
                        }
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                    }

                    display_temp = round(message.cJSON_temp);
                    display_rh = round(message.cJSON_rh);
                    display_pm2_5 = round(message.cJSON_pm2_5);
                    display_pm10 = round(message.cJSON_pm10);
                    display_co2 = round(message.cJSON_co2);

                    if (display_temp>99 || display_temp<0){
                        display_data_model.t_color = color_red;
                        memcpy(display_data_model.temp,"//",sizeof(display_data_model.temp));
                    }
                    else{
                        display_data_model.t_color = color_green;
                        sprintf(tempstr,"%2d",display_temp);
                        ESP_LOGD(TAG, "temperature: %s",tempstr);
                        memcpy(display_data_model.temp,tempstr,sizeof(display_data_model.temp));
                    }
                    
                    if (display_rh>99 || display_rh<0){
                        display_data_model.rh_color = color_red;
                        memcpy(display_data_model.rh,"//",sizeof(display_data_model.rh));
                    }
                    else{
                        display_data_model.rh_color = color_green;
                        sprintf(rhstr,"%2d",display_rh);
                        ESP_LOGD(TAG, "RH: %s",rhstr);
                        memcpy(display_data_model.rh,rhstr,sizeof(display_data_model.rh));
                    }
                    
                    if (display_pm2_5>999 || display_pm2_5<0){
                        display_data_model.rh_color = color_red;
                        display_data_model.light_color = color_red;
                        memcpy(display_data_model.pm2_5,"///",sizeof(display_data_model.pm2_5));
                    }
                    else{
                        if (display_pm2_5>=55){
                            display_data_model.pm_color = color_red;
                            display_data_model.light_color = color_red;
                        }
                        else if (display_pm2_5>=35){
                            display_data_model.pm_color = color_yellow;
                            display_data_model.light_color = color_yellow;
                        }
                        else{
                            display_data_model.pm_color = color_green;
                            display_data_model.light_color = color_green;
                        }
                        sprintf(pmstr,"%3d",display_pm2_5);
                        ESP_LOGD(TAG, "PM2.5: %s",pmstr);
                        memcpy(display_data_model.pm2_5,pmstr,sizeof(display_data_model.pm2_5));
                    }

                    if (display_co2>9999 || display_co2<0){
                        display_data_model.co2_color = color_red;
                        memcpy(display_data_model.co2,"////",sizeof(display_data_model.co2));
                    }
                    else{
                        if (display_co2>=900){
                            display_data_model.co2_color = color_red;
                        }
                        else if (display_co2>=600){
                            display_data_model.co2_color = color_yellow;
                        }
                        else{
                            display_data_model.co2_color = color_green;
                        }
                        sprintf(co2str,"%4d",display_co2);
                        ESP_LOGD(TAG, "CO2: %s",co2str);
                        memcpy(display_data_model.co2,co2str,sizeof(display_data_model.co2));
                    }

                    int data_len = sizeof(display_data_model);
                    int txBytes = 0;                    
                    uint8_t data[27];
                    memcpy(data, &display_data_model, sizeof(display_data_model));
                    ESP_LOGI(TAG,"data -> %s",(char*)data);
                    timeout_counter = 0;
                    for(int i=0;i<data_len;i++){
                        int sent = uart_write_bytes(UART_NUM_2, &data[i], 1);
                        txBytes += sent;
                    }
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    break;
                }
                case DISPLAY_TIMEOUT_EVENT:{
                    ESP_LOGI(TAG, "Queue receive timeout mutlicast is not empty");
                    memcpy(display_data_model.temp,"No",sizeof(display_data_model.temp));
                    display_data_model.pm_color = color_green;
                    display_data_model.light_color = color_green;
                    display_data_model.co2_color = color_green;
                    memcpy(display_data_model.pm2_5,"PAB",sizeof(display_data_model.pm2_5));
                    memcpy(display_data_model.rh,"//",sizeof(display_data_model.rh));
                    memcpy(display_data_model.co2,"Data",sizeof(display_data_model.co2));
                    int data_len = sizeof(display_data_model);
                    int txBytes = 0;                    
                    uint8_t data[27];
                    memcpy(data, &display_data_model, sizeof(display_data_model));
                    ESP_LOGI(TAG,"data -> %s",(char*)data);
                    for(int i=0;i<data_len;i++){
                        int sent = uart_write_bytes(UART_NUM_2, &data[i], 1);
                        txBytes += sent;
                    }
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }
                default:
                    break;
            }
        }
        else{
            timeout_counter = 0;
            ESP_LOGI(TAG, "Queue receive timeout");
            memcpy(display_data_model.temp,"//",sizeof(display_data_model.temp));
            display_data_model.pm_color = color_green;
            display_data_model.light_color = color_green;
            display_data_model.co2_color = color_green;
            memcpy(display_data_model.pm2_5,"///",sizeof(display_data_model.pm2_5));
            memcpy(display_data_model.rh,"//",sizeof(display_data_model.rh));
            memcpy(display_data_model.co2,"////",sizeof(display_data_model.co2));
            int data_len = sizeof(display_data_model);
            int txBytes = 0;                    
            uint8_t data[27];
            memcpy(data, &display_data_model, sizeof(display_data_model));
            ESP_LOGI(TAG,"data -> %s",(char*)data);
            for(int i=0;i<data_len;i++){
                int sent = uart_write_bytes(UART_NUM_2, &data[i], 1);
                txBytes += sent;
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

uint32_t display_stream_send_data(aerobox_data_model_v1_t *data){
    display_message_t msg;
    msg.code = DISPLAY_SEND_RAW_DATA_EVENT;
    msg.receive_data = data;
    if (display_queue == NULL){
        ESP_LOGW(TAG, "No Message Queue");
        return 0;
    }
    return xQueueSend(display_queue, (void *) &msg, portMAX_DELAY);
}

void display_send_factory(aerobox_data_model_v1_t *data){
    display_stream_send_data(data);
}

void display_send_mqtt_factory(char *deviceId, double temp, double rh, double pm2_5, double pm10,double co2){
    display_message_t msg;
    msg.code = DISPLAY_SEND_MQTT_DATA_EVENT;
    memcpy(msg.device_id, deviceId, sizeof(msg.device_id));
    msg.cJSON_temp = temp;
    msg.cJSON_rh = rh;
    msg.cJSON_pm2_5 = pm2_5;
    msg.cJSON_pm10 = pm10;
    msg.cJSON_co2 = co2;
    if (display_queue == NULL){
        ESP_LOGW(TAG, "No Message Queue");
        return 0;
    }
    return xQueueSend(display_queue, (void *) &msg, portMAX_DELAY);
}

void display_message_queue_init(){
    if (display_queue == NULL){
        ESP_LOGI(TAG, "display_queue init");
        display_queue = xQueueCreate(50, sizeof(display_message_t));
    }
}

void timeout_count_task(void* pvParameters) {
    const static char* TAG = "timeout_count_task";
    const int DELAY = 1000 / portTICK_PERIOD_MS; // 1 second

    ESP_LOGI(TAG,"starting task");
    for(;;) {
        if (timeout_counter<62){
            timeout_counter++;
        }
        else{
            timeout_counter = 0;
            display_message_t msg;
            msg.code = DISPLAY_TIMEOUT_EVENT;
            xQueueSend(display_queue, (void *) &msg, portMAX_DELAY);
        }
        vTaskDelay(DELAY);
    }
}

void display_stream_handler_start(){    
    xTaskCreate(
        display_stream_handler,        // Function that implements the task.
        "diplay Sender",                           // Text name for the task.
        4096,                           // Stack size in bytes, not words.
        NULL,                        // Parameter passed into the task.
        3,          // Priority at which the task is created.
        display_queue_handler);       // Variable to hold the task's data structure.
    xTaskCreate(&timeout_count_task, "count_task", 2048, NULL, 4, NULL);
}


#define PORT 50012

// static const char *TAG = "example";
// static const char *payload = "Message from ESP32 ";

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;

        while (1) {
            ESP_LOGI(TAG, "Waiting for data");

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                // ESP_LOGI(TAG, "%s", rx_buffer);
                if (len==84){
                    memcpy(&udp_recv_data, rx_buffer, 84);
                    ESP_LOGI(TAG, "data id: %s", (char*)udp_recv_data.te_module_id);
                    display_send_factory(&udp_recv_data);
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                }
                // discover function
                if (len==64){
                    memcpy(&udp_discover_data, rx_buffer, 64);
                    if (udp_discover_data.discover_st==0xE1 && udp_discover_data.discover_ed==0xED){
                        ESP_LOGI(TAG, "discover gw id: %s", (char*)udp_discover_data.te_module_id);
                        memcpy(udp_discover_data.te_module_id, tornadoedge_auth_settings_v2.device_hostname, sizeof(udp_discover_data.te_module_id));
                        memcpy(udp_discover_data.device_mac, device_configs.device_mac, sizeof(udp_discover_data.device_mac));
                        memcpy(udp_discover_data.te_gw_ver, device_configs.tornado_ver, sizeof(udp_discover_data.te_gw_ver));
                        
                        memcpy(rx_buffer, &udp_discover_data, 64);
                        int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                        if (err < 0) {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                            break;
                        }
                    }
                    
                }
                
                
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

#define UDP_PORT 50013

#define MULTICAST_LOOPBACK CONFIG_EXAMPLE_LOOPBACK

#define MULTICAST_TTL 1

#define MULTICAST_IPV4_ADDR "232.140.115.64"
#define LISTEN_ALL_IF   EXAMPLE_MULTICAST_LISTEN_ALL_IF
#define CONFIG_EXAMPLE_IPV4 true;
#define CONFIG_EXAMPLE_IPV4_ONLY true;

// #ifdef CONFIG_EXAMPLE_IPV4
static const char *V4TAG = "mcast-ipv4";
// #endif



#ifdef CONFIG_EXAMPLE_IPV4
/* Add a socket, either IPV4-only or IPV6 dual mode, to the IPV4
   multicast group */
static int socket_add_ipv4_multicast_group(int sock, bool assign_source_if)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface
    imreq.imr_interface.s_addr = IPADDR_ANY;
    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        ESP_LOGE(V4TAG, "Configured IPV4 multicast address '%s' is invalid.", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    ESP_LOGI(TAG, "Configured IPV4 Multicast address %s", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(V4TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
            goto err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        goto err;
    }

 err:
    return err;
}
#endif /* CONFIG_EXAMPLE_IPV4 */

#ifdef CONFIG_EXAMPLE_IPV4_ONLY
static int create_multicast_ipv4_socket(void)
{
    struct sockaddr_in saddr = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(V4TAG, "Failed to create socket. Error %d", errno);
        return -1;
    }

    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to bind socket. Error %d", errno);
        goto err;
    }


    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
        goto err;
    }

#if MULTICAST_LOOPBACK
    // select whether multicast traffic should be received by this device, too
    // (if setsockopt() is not called, the default is no)
    uint8_t loopback_val = MULTICAST_LOOPBACK;
    err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP,
                     &loopback_val, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_LOOP. Error %d", errno);
        goto err;
    }
#endif

    // this is also a listening socket, so add it to the multicast
    // group for listening...
    err = socket_add_ipv4_multicast_group(sock, true);
    if (err < 0) {
        goto err;
    }

    // All set, socket is configured for sending and receiving
    return sock;

err:
    close(sock);
    return -1;
}
#endif /* CONFIG_EXAMPLE_IPV4_ONLY */

#ifdef CONFIG_EXAMPLE_IPV6
static int create_multicast_ipv6_socket(void)
{
    struct sockaddr_in6 saddr = { 0 };
    int  netif_index;
    struct in6_addr if_inaddr = { 0 };
    struct ip6_addr if_ipaddr = { 0 };
    struct ipv6_mreq v6imreq = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET6, SOCK_DGRAM, IPPROTO_IPV6);
    if (sock < 0) {
        ESP_LOGE(V6TAG, "Failed to create socket. Error %d", errno);
        return -1;
    }

    // Bind the socket to any address
    saddr.sin6_family = AF_INET6;
    saddr.sin6_port = htons(UDP_PORT);
    bzero(&saddr.sin6_addr.un, sizeof(saddr.sin6_addr.un));
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in6));
    if (err < 0) {
        ESP_LOGE(V6TAG, "Failed to bind socket. Error %d", errno);
        goto err;
    }

    // Selct the interface to use as multicast source for this socket.
#if LISTEN_ALL_IF
    bzero(&if_inaddr.un, sizeof(if_inaddr.un));
#else
    // Read interface adapter link-local address and use it
    // to bind the multicast IF to this socket.
    //
    // (Note the interface may have other non-LL IPV6 addresses as well,
    // but it doesn't matter in this context as the address is only
    // used to identify the interface.)
    err = esp_netif_get_ip6_linklocal(EXAMPLE_INTERFACE, (esp_ip6_addr_t*)&if_ipaddr);
    inet6_addr_from_ip6addr(&if_inaddr, &if_ipaddr);
    if (err != ESP_OK) {
        ESP_LOGE(V6TAG, "Failed to get IPV6 LL address. Error 0x%x", err);
        goto err;
    }
#endif // LISTEN_ALL_IF

    // search for netif index
    netif_index = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
    if(netif_index < 0) {
        ESP_LOGE(V6TAG, "Failed to get netif index");
        goto err;
    }
    // Assign the multicast source interface, via its IP
    err = setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_IF, &netif_index,sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V6TAG, "Failed to set IPV6_MULTICAST_IF. Error %d", errno);
        goto err;
    }

    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V6TAG, "Failed to set IPV6_MULTICAST_HOPS. Error %d", errno);
        goto err;
    }

#if MULTICAST_LOOPBACK
    // select whether multicast traffic should be received by this device, too
    // (if setsockopt() is not called, the default is no)
    uint8_t loopback_val = MULTICAST_LOOPBACK;
    err = setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_LOOP,
                     &loopback_val, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V6TAG, "Failed to set IPV6_MULTICAST_LOOP. Error %d", errno);
        goto err;
    }
#endif

    // this is also a listening socket, so add it to the multicast
    // group for listening...
#ifdef CONFIG_EXAMPLE_IPV6
    // Configure multicast address to listen to
    err = inet6_aton(MULTICAST_IPV6_ADDR, &v6imreq.ipv6mr_multiaddr);
    if (err != 1) {
        ESP_LOGE(V6TAG, "Configured IPV6 multicast address '%s' is invalid.", MULTICAST_IPV6_ADDR);
        goto err;
    }
    ESP_LOGI(TAG, "Configured IPV6 Multicast address %s", inet6_ntoa(v6imreq.ipv6mr_multiaddr));
    ip6_addr_t multi_addr;
    inet6_addr_to_ip6addr(&multi_addr, &v6imreq.ipv6mr_multiaddr);
    if (!ip6_addr_ismulticast(&multi_addr)) {
        ESP_LOGW(V6TAG, "Configured IPV6 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV6_ADDR);
    }
    // Configure source interface
    v6imreq.ipv6mr_interface = (unsigned int)netif_index;
    err = setsockopt(sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP,
                     &v6imreq, sizeof(struct ipv6_mreq));
    if (err < 0) {
        ESP_LOGE(V6TAG, "Failed to set IPV6_ADD_MEMBERSHIP. Error %d", errno);
        goto err;
    }
#endif

#if CONFIG_EXAMPLE_IPV4_V6
    // Add the common IPV4 config options
    err = socket_add_ipv4_multicast_group(sock, false);
    if (err < 0) {
        goto err;
    }
#endif

#if CONFIG_EXAMPLE_IPV4_V6
    int only = 0;
#else
    int only = 1; /* IPV6-only socket */
#endif
    err = setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &only, sizeof(int));
    if (err < 0) {
        ESP_LOGE(V6TAG, "Failed to set IPV6_V6ONLY. Error %d", errno);
        goto err;
    }
    ESP_LOGI(TAG, "Socket set IPV6-only");

    // All set, socket is configured for sending and receiving
    return sock;

err:
    close(sock);
    return -1;
}
#endif

static void mcast_example_task(void *pvParameters)
{
    while (1) {
        int sock;

        sock = create_multicast_ipv4_socket();
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create IPv4 multicast socket");
        }

        if (sock < 0) {
            // Nothing to do!
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        // set destination multicast addresses for sending from these sockets
        struct sockaddr_in sdestv4 = {
            .sin_family = PF_INET,
            .sin_port = htons(UDP_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);

        int err = 1;
        while (err > 0) {
            struct timeval tv = {
                .tv_sec = 1,
                .tv_usec = 0,
            };
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);

            int s = select(sock + 1, &rfds, NULL, NULL, &tv);
            if (s < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
                err = -1;
                break;
            }
            else if (s > 0) {
                if (FD_ISSET(sock, &rfds)) {
                    // Incoming datagram received
                    char recvbuf[128];
                    char raddr_name[32] = { 0 };

                    struct sockaddr_storage raddr; // Large enough for both IPv4 or IPv6
                    socklen_t socklen = sizeof(raddr);
                    int len = recvfrom(sock, recvbuf, sizeof(recvbuf)-1, 0,
                                       (struct sockaddr *)&raddr, &socklen);
                    if (len < 0) {
                        ESP_LOGE(TAG, "multicast recvfrom failed: errno %d", errno);
                        err = -1;
                        break;
                    }


                    if (raddr.ss_family == PF_INET) {
                        inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr,
                                    raddr_name, sizeof(raddr_name)-1);
                    }

                    ESP_LOGI(TAG, "received %d bytes from %s:", len, raddr_name);

                    recvbuf[len] = 0; // Null-terminate whatever we received and treat like a string...
                    // ESP_LOGI(TAG, "test_data: %s", recvbuf);
                    if (len==84){
                        memcpy(&udp_recv_data, recvbuf, 84);
                        ESP_LOGI(TAG, "data id: %s", (char*)udp_recv_data.te_module_id);
                        display_send_factory(&udp_recv_data);
                        vTaskDelay(20 / portTICK_PERIOD_MS);
                    }   
                }
            }
            else { // s == 0
                // Timeout passed with no incoming data, so send something!
                uint8_t sendbuf[84];
                char addrbuf[32] = { 0 };
                memcpy(udp_send_data.te_module_id, tornadoedge_auth_settings_v2.auth_client_id, 
                        sizeof(udp_send_data.te_module_id));
                memcpy(sendbuf, &udp_send_data, 84);

                struct addrinfo hints = {
                    .ai_flags = AI_PASSIVE,
                    .ai_socktype = SOCK_DGRAM,
                };
                struct addrinfo *res;


                hints.ai_family = AF_INET; // For an IPv4 socket

                int err = getaddrinfo(MULTICAST_IPV4_ADDR,
                                      NULL,
                                      &hints,
                                      &res);
                if (err < 0) {
                    ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                    break;
                }
                if (res == 0) {
                    ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                    break;
                }

                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                // ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);

                if (tornadoedge_configs_v1.local_muticast_mode && local_data_refreshed) {
                    local_data_refreshed = false;
                    err = sendto(sock, sendbuf, 84, 0, res->ai_addr, res->ai_addrlen);
                }
                freeaddrinfo(res);
                // if (err < 0) {
                //     ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                //     break;
                // }
            }
        }

        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }

}

void send_mqtt_display_query_task(void *pvParameters){
    while (true){
        if (tornadoedge_configs_v1.mqtt_display_mode){
            for (int j=0; j<10; j++){
                if (strcmp((char*)tornadoedge_display_unit_v2.display_pairs[j],"")){
                    mqtt_send_tornado_factory(GATEWAY_QUERY,FINE,"",
                                    (char*)tornadoedge_display_unit_v2.display_pairs[0],
                                    (char*)tornadoedge_auth_settings_v2.auth_client_id
                                    );
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }
            }
            
        }
        vTaskDelay((tornadoedge_configs_v1.mqtt_query_resolution*1000) / portTICK_PERIOD_MS);
    }
}

void lcd1602_task(void * pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = 0x27;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight off
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    // turn off backlight
    ESP_LOGI(TAG, "backlight off");
    i2c_lcd1602_set_backlight(lcd_info, false);
    vTaskDelay(1000 / portTICK_RATE_MS);

    // turn on backlight
    ESP_LOGI(TAG, "backlight on");
    i2c_lcd1602_set_backlight(lcd_info, true);

    i2c_lcd1602_write_string(lcd_info, "  TornadoEdge         TornadoEdge       Gateway power on");
    vTaskDelay(2000 / portTICK_RATE_MS);

    // i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    // i2c_lcd1602_write_string(lcd_info, "T:      RH:     ");
    // i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    // i2c_lcd1602_write_string(lcd_info, "PM:     CO2:    ");

    // while (true)
    // {
    //     i2c_lcd1602_move_cursor(lcd_info, 3, 0);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.temp[0]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.temp[1]);
    //     // i2c_lcd1602_write_string(lcd_info, (char*)display_data_model.temp);
    //     i2c_lcd1602_move_cursor(lcd_info, 12, 0);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.rh[0]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.rh[1]);
    //     // i2c_lcd1602_write_string(lcd_info, (char*)display_data_model.rh);
    //     i2c_lcd1602_move_cursor(lcd_info, 3, 1);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.pm2_5[0]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.pm2_5[1]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.pm2_5[2]);
    //     // i2c_lcd1602_write_string(lcd_info, (char*)display_data_model.pm2_5);
    //     i2c_lcd1602_move_cursor(lcd_info, 12, 1);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.co2[0]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.co2[1]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.co2[2]);
    //     i2c_lcd1602_write_char(lcd_info, display_data_model.co2[3]);
    //     // i2c_lcd1602_write_string(lcd_info, (char*)display_data_model.co2);
    //     vTaskDelay(500 / portTICK_RATE_MS);
    // }

    i2c_lcd1602_set_display(lcd_info, false);
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_string(lcd_info, "W1.   2.   3.         ");
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_string(lcd_info, "                      ");
    i2c_lcd1602_set_display(lcd_info, true);
    char lcd_str[7];
    while (true)
    {
        for (int i=0; i<3; i++){
            i2c_lcd1602_move_cursor(lcd_info, 1+5*i, 1);
            if (sq_data[i].weight>-100 && sq_data[i].error_code<100){
                sprintf(lcd_str,"%5d",sq_data[i].weight);
                i2c_lcd1602_write_string(lcd_info, lcd_str);
            }
            else if (sq_data[i].error_code==0xEF){
                i2c_lcd1602_write_string(lcd_info, " ////");
            }
            else if (sq_data[i].error_code==0xFF){
                i2c_lcd1602_write_string(lcd_info, " ----");
                i2c_lcd1602_move_cursor(lcd_info, 1+5*i, 0);
                i2c_lcd1602_write_string(lcd_info, "    ");
            }
        }
        vTaskDelay(200 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void display_service_init(){
    esp_log_level_set("display-service", ESP_LOG_DEBUG);
    
    initialize_display_uart();

    display_message_queue_init();
    
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

    xTaskCreate(&mcast_example_task, "mcast_task", 4096, NULL, 5, NULL);

    display_stream_handler_start();

    xTaskCreate(send_mqtt_display_query_task, "send_mqtt_display_query_task", 1024*2, NULL, 6, NULL);

}

#ifdef __cplusplus
}
#endif