#include <stdio.h>
#include <string.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include "cJSON.h"

#include "websocket_server.h"
#include "esp_storage.h"
#include "te_gw_setup.h"

#include "te-ws-sv.h"


#ifdef __cplusplus
extern "C" {
#endif

static QueueHandle_t client_queue;
const static int client_queue_size = 10;

bool enable_eng = false;

time_t now;


// handles websocket events
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char* msg,uint64_t len) {
    const static char* TAG = "websocket_callback";
    int value;

    switch(type) {
    case WEBSOCKET_CONNECT:
        ESP_LOGI(TAG,"client %i connected!",num);
        break;
    case WEBSOCKET_DISCONNECT_EXTERNAL:
        ESP_LOGI(TAG,"client %i sent a disconnect message",num);
        // led_duty(0);
        break;
    case WEBSOCKET_DISCONNECT_INTERNAL:
        ESP_LOGI(TAG,"client %i was disconnected",num);
        break;
    case WEBSOCKET_DISCONNECT_ERROR:
        ESP_LOGI(TAG,"client %i was disconnected due to an error",num);
        // led_duty(0);
        break;
    case WEBSOCKET_TEXT:
        if(len) { // if the message length was greater than zero
        switch(msg[0]) {
            case 'L':
            if(sscanf(msg,"L%i",&value)) {
                ESP_LOGI(TAG,"LED value: %i",value);
                // led_duty(value);
                ws_server_send_text_all_from_callback(msg,len); // broadcast it!
            }
            break;
            case 'M':
            ESP_LOGI(TAG, "got message length %i: %s", (int)len-1, &(msg[1]));
            if (!strcmp((char*)&(msg[1]),"tornadopwd")){
                enable_eng = true;
                int num_su = 6;
                char* response_data = get_all_info(&num_su);
                ws_server_send_text_all_from_callback(msg,len);
                ws_server_send_text_all_from_callback(response_data,strlen(response_data));
                ESP_LOGI(TAG, "flash data: %s %d",response_data, strlen(response_data));
                free(response_data);
            }
            break;
            // case 'M':
            // ESP_LOGI(TAG, "got message length %i: %s", (int)len-1, &(msg[1]));
            // if(sscanf(msg,"M%i",&value)) {
            //     ESP_LOGI(TAG,"method value: %i",value);
            //     if (value > 3){
            //         value = 0;
            //     }
            //     else if (value < 0){
            //         value = 0;
            //     }
            //     set_display_method(value);
            // }
            break;
            case 'C':
            ESP_LOGI(TAG, "got Color: %s", &(msg[1]));
            // set_color(&(msg[1]));
            // set_index(1);
            ws_server_send_text_all_from_callback(msg,len); // broadcast it!
            break;
            case 'E':
                ESP_LOGI(TAG, "got Engineering mode: %s", &(msg[1]));
                if (!enable_eng){
                    ESP_LOGI(TAG, "enable flag false");
                    ws_server_send_text_all_from_callback("Not login",sizeof("Not login"));
                    break;
                }
                if (!strcmp((char*)&(msg[1]),"erase")){
                    ESP_LOGI(TAG, "Start erase");
                    erase_tornadoedge_value();
                    erase_aerobox_gw_value();
                    erase_te_config_value();
                    ws_server_send_text_all_from_callback(msg,len); // broadcast it!
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    // ws_server_send_text_all_from_callback(msg,len); // broadcast it!
                    esp_restart();
                    break;
                }
                else if (!strcmp((char*)&(msg[1]),"format")){
                    ESP_LOGI(TAG, "Start erase_flash_nvs_value");
                    esp_err_t ret;
                    erase_flash_nvs_value();
                    init_flash_nvs_value();
                    ret = save_tornadoedge_value();
                    if (ret != ESP_OK) ESP_LOGI(TAG, "Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    // ret = save_aerobox_gw_value();
                    // if (ret != ESP_OK) ESP_LOGI(TAG, "Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    ret = save_te_config_value();
                    if (ret != ESP_OK) ESP_LOGI(TAG, "Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    ws_server_send_text_all_from_callback(msg,len);
                    break;
                }
                else if (!strcmp((char*)&(msg[1]),"save")){
                    esp_err_t ret;
                    ret = save_tornadoedge_value();
                    if (ret != ESP_OK) ESP_LOGI(TAG, "Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    // ret = save_aerobox_gw_value();
                    // if (ret != ESP_OK) ESP_LOGI(TAG, "Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    ret = save_te_config_value();
                    if (ret != ESP_OK) ESP_LOGI(TAG, "Error (%s) reading data from NVS!\n", esp_err_to_name(ret));    
                    ws_server_send_text_all_from_callback(msg,len);
                    enable_eng = false;
                    break;
                }
                else if (!strcmp((char*)&(msg[1]),"reboot")){
                    ESP_LOGI(TAG, "Start reboot");
                    ws_server_send_text_all_from_callback(msg,len); // broadcast it!
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    // ws_server_send_text_all_from_callback(msg,len); // broadcast it!
                    esp_restart();
                }
                else if (!strcmp((char*)&(msg[1]),"get")){
                    int num_su = 6;
                    char* response_data = get_all_info(&num_su);
                    ws_server_send_text_all_from_callback(msg,len);
                    ws_server_send_text_all_from_callback(response_data,strlen(response_data));
                    ESP_LOGI(TAG, "flash data: %s %d",response_data, strlen(response_data));
                    free(response_data);
                    break;
                }
                char mode[10], reviced_ID[10];
                int position;
                // E:su 1,C1160501
                sscanf(msg, "E:%s %d, %s", mode, &position, reviced_ID);
                
                if (!strcmp(mode,"su")){
                    if (position<11){
                        set_aerobox_id(&position, reviced_ID);
                        ESP_LOGI(TAG, "got %s, num:%d, id:%s",mode,position,reviced_ID);
                    }
                }
                else if (!strcmp(mode,"lr")){
                    if (position==0){
                        set_lr_id(reviced_ID);
                        ESP_LOGI(TAG, "got %s, num:%d, id:%s",mode,position,reviced_ID);
                    }
                }
                else if (!strcmp(mode,"gw")){
                    if (position==500){
                        set_gw_id(reviced_ID);
                        ESP_LOGI(TAG, "got %s, num:%d, id:%s",mode,position,reviced_ID);
                    }
                }
                else{
                    ESP_LOGI(TAG, "got an unknown %s, num:%d, id:%s",mode,position,reviced_ID);
                }
                
                break;
            case 'Q':
            ESP_LOGI(TAG, "got Engineering MQTT mode: %s", &(msg[1]));
            char mqttmode[10], reviced_uri[150];
            // Q:mqtt,MQTT://
            sscanf(msg, "Q: %s : %s", mqttmode, reviced_uri);
            if (!strcmp(mqttmode,"authuri")){
                set_auth_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, auth_uri:%s",mqttmode,reviced_uri);
            }
            else if (!strcmp(mqttmode,"puburi")){
                set_pub_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, pub_uri:%s",mqttmode,reviced_uri);
                
            }
            else if (!strcmp(mqttmode,"suburi")){
                set_sub_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, sub_uri:%s",mqttmode,reviced_uri);
            }
            else if (!strcmp(mqttmode,"psk")){
                set_psk_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, psk:%s",mqttmode,reviced_uri);
            }
            else if (!strcmp(mqttmode,"pskhint")){
                set_pskhint_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, psk:%s",mqttmode,reviced_uri);
            }
            else if (!strcmp(mqttmode,"hostname")){
                set_hostname_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, psk:%s",mqttmode,reviced_uri);
            }
            else if (!strcmp(mqttmode,"certpem")){
                // set_certpem_uri(reviced_uri);
                ESP_LOGI(TAG, "got %s, certpem:%s",mqttmode,reviced_uri);
            }
            else{
                ESP_LOGI(TAG, "got an unknown mode: %s, uri:%s",mqttmode,reviced_uri);
            }
            break;
            case '{':
                ESP_LOGI(TAG, "TE_EVENT_DATA");
                if (!enable_eng){
                    ESP_LOGI(TAG, "enable flag false");
                    ws_server_send_text_all_from_callback("Not login",sizeof("Not login"));
                    break;
                }
                printf("DATA=%s\r\n", msg);
                parameter_update("topic", msg, SETUP_WS_MODE);
                break;
            default:
            ESP_LOGI(TAG, "got an unknown message with length %i", (int)len);
            ESP_LOGI(TAG, "got an unknown message %s", msg);
            break;
            
        }
        }
        break;
    case WEBSOCKET_BIN:
        ESP_LOGI(TAG,"client %i sent binary message of size %i:\n%s",num,(uint32_t)len,msg);
        break;
    case WEBSOCKET_PING:
        ESP_LOGI(TAG,"client %i pinged us with message of size %i:\n%s",num,(uint32_t)len,msg);
        break;
    case WEBSOCKET_PONG:
        ESP_LOGI(TAG,"client %i responded to the ping",num);
        break;
    }
}

// serves any clients
void http_serve(struct netconn *conn) {
    const static char* TAG = "http_server_ws";
    const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
    struct netbuf* inbuf;
    static char* buf;
    static uint16_t buflen;
    static err_t err;

    netconn_set_recvtimeout(conn,1000); // allow a connection timeout of 1 second
    ESP_LOGI(TAG,"reading from client...");
    err = netconn_recv(conn, &inbuf);
    ESP_LOGI(TAG,"read from client");
    if(err==ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    if(buf) {
        // default page websocket
        if(strstr(buf,"GET / ")
            && strstr(buf,"Upgrade: websocket")) {
        ESP_LOGI(TAG,"Requesting websocket on /");
        ws_server_add_client(conn,buf,buflen,(char*)"/",websocket_callback);
        netbuf_delete(inbuf);
        }

        else if(strstr(buf,"GET /")) {
        ESP_LOGI(TAG,"Unknown request, sending error page: %s",buf);
        netconn_write(conn, ERROR_HEADER, sizeof(ERROR_HEADER)-1,NETCONN_NOCOPY);
        // netconn_write(conn, error_html_start, error_html_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
        }

        else {
        ESP_LOGI(TAG,"Unknown request");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
        }
    }
    else {
        ESP_LOGI(TAG,"Unknown request (empty?...)");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
    }
    }
    else { // if err==ERR_OK
    ESP_LOGI(TAG,"error on read, closing connection");
    netconn_close(conn);
    netconn_delete(conn);
    netbuf_delete(inbuf);
    }
}

// handles clients when they first connect. passes to a queue
void server_task(void* pvParameters) {
    const static char* TAG = "server_task";
    struct netconn *conn, *newconn;
    static err_t err;
    client_queue = xQueueCreate(client_queue_size,sizeof(struct netconn*));

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn,NULL,30678);
    netconn_listen(conn);
    ESP_LOGI(TAG,"server listening");
    do {
    err = netconn_accept(conn, &newconn);
    ESP_LOGI(TAG,"new client");
    if(err == ERR_OK) {
        xQueueSendToBack(client_queue,&newconn,portMAX_DELAY);
        //http_serve(newconn);
    }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
    ESP_LOGE(TAG,"task ending, rebooting board");
    esp_restart();
}

// receives clients from queue, handles them
void server_handle_task(void* pvParameters) {
    const static char* TAG = "server_handle_task";
    struct netconn* conn;
    ESP_LOGI(TAG,"task starting");
    for(;;) {
    xQueueReceive(client_queue,&conn,portMAX_DELAY);
    if(!conn) continue;
    http_serve(conn);
    }
    vTaskDelete(NULL);
}


void count_task(void* pvParameters) {
    const static char* TAG = "count_task";
    char out[20];
    int len;
    int clients;
    // const static char* word = "%i";
    const static char* word = "%ld";
    uint8_t n = 0;
    const int DELAY = 1000 / portTICK_PERIOD_MS; // 1 second

    ESP_LOGI(TAG,"starting task");
    for(;;) {
    // len = sprintf(out,word,n);
    time(&now);
    len = sprintf(out,word,now);
    clients = ws_server_send_text_all(out,len);
    if(clients > 0) {
    // ESP_LOGI(TAG,"sent: \"%s\" to %i clients",out,clients);
    }
    n++;
    vTaskDelay(DELAY);
    }
}

#ifdef __cplusplus
}
#endif
