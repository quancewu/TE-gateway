// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <esp_err.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define color_red               0x01
#define color_green             0x02
#define color_yellow            0x03
#define blackspace_default      0x20
#define blackspace2_default     "\x20\x20"
#define lightcode               "\xA1\xF1"

/** Intialise ESP QCloud Storage
 *
 * This API is internally called by esp_init(). Applications may call this
 * only if access to the ESP QCloud storage is required before esp_init().
 *
 * @return
 *     - ESP_FAIL
 *     - ESP_OK
 */
esp_err_t esp_storage_init(void);

/**
 * @brief save the information with given key
 *
 * @param  key    Key name. Maximal length is 15 characters. Shouldn't be empty.
 * @param  value  The value to set.
 * @param  length length of binary value to set, in bytes; Maximum length is
 *                1984 bytes (508000 bytes or (97.6% of the partition size - 4000) bytes
 *                whichever is lower, in case multi-page blob support is enabled).
 *
 * @return
 *     - ESP_FAIL
 *     - ESP_OK
 */
esp_err_t esp_storage_set(const char *key, const void *value, size_t length);

/**
 * @brief  Load the information,
 *         esp_err_t esp_storage_load(const char *key, void *value, size_t *length);
 *         esp_err_t esp_storage_load(const char *key, void *value, size_t length);
 *
 * @attention  The interface of this api supports size_t and size_t * types.
 *             When the length parameter of the pass is size_t, it is only the
 *             length of the value. When the length parameter of the pass is size_t *,
 *             the length of the saved information can be obtained.
 *
 * @param  key    The corresponding key of the information that want to load
 * @param  value  The corresponding value of key
 * @param  length The length of the value, Pointer type will return length
 *
 * @return
 *     - ESP_FAIL
 *     - ESP_OK
 */
esp_err_t esp_storage_get(const char *key, void *value, size_t length);

/*
 * @brief  Erase the information with given key
 *
 * @param  key The corresponding key of the information that want to erase
 *
 * @return
 *     - ESP_FAIL
 *     - ESP_OK
 */
esp_err_t esp_storage_erase(const char *key);

typedef enum status_code_v2_t {
	NONE_CODE = 0,
	FINE = 1,
	FATEL = 2,
	ERROR = 3,
	WARNING = 4,
	INFO = 5,
	DEBUG = 6,
	REBOOTED = 7,
	WIRELESS_DOWN = 8,
	NO_RESPONSE = 9,
	ILLEGAL = 10
}status_code_t;

typedef enum message_type_v2_t {
	NONE_MESSAGE = 0,
	GATEWAY_HEARTBEAT = 1,
	GATEWAY_SYSTEM_INFO = 2,
	GATEWAY_GET_PARAMETER = 3,
	GATEWAY_GET_PAIRING = 4,
	GATEWAY_SET_PAIRING = 5,
	GATEWAY_GET_DISPLAY = 6,
	GATEWAY_SET_DISPLAY = 7,
	GATEWAY_PAIRING = 8,
	GATEWAY_DISPLAY = 9,
	GATEWAY_QUERY = 10,
	GATEWAY_DATA_RESPONSE = 11,
	GATEWAY_REBOOT = 12,
	GATEWAY_TIME_SYNC = 13,
	GATEWAY_PARAM_UPDATE = 14,
	GATEWAY_REPLY = 15,
	AEROBOX_HEARTBEAT = 16,
	SG_HEARTBEAT = 17,
	AEROBOX_RAW_DATA = 18,
	AEROBOX_JSON_DATA = 19,
	SG_SEQ_JSON_DATA = 20,
	SG_REPORT_JSON_DATA = 21,
	END_MESSAGE_TYPE = 22
}message_type_v2_t;

typedef enum parameter_type_t {
	NONE_PARAM = 0,
	HEX_GATEWAY_ID = 1,
	AUTH_GATEWAY_ID = 2,
	AUTH_GATEWAY_PWD = 3,
	ECU_GATEWAY_PAIR_ID = 4,
	ECU_PAIRS = 5,
	MQTT_AUTH_URI = 6,
	MQTT_PUB_URI = 7,
	AUTH_PSK = 8,
	AUTH_PSK_HINT = 9,
	MQTT_SUB_URI = 10,
	DEVICE_HOSTNAME = 11,
	AEROBOX_GATEWAY_PAIR_ID = 12,
	AEROBOX_PAIRS = 13,
	AEROBOX_MQTT_URI = 14,
	PARAM_UPDATE_TIME = 15,
	OTA_UPGRADE = 16,
	CA_CERT = 17,
	HEARTBEAT_RESOLUTION = 18,
	LOCAL_MUTICAST = 19,
	DATA_UPLOAD_RESOLUTION = 20,
	MQTT_QUERY_RESOLUTION = 21,
	MQTT_DATA_AGE = 22,
	MQTT_DISPLAY_MODE = 23,
	AEROBOX_GW_MODE = 24,
	SQ_GW_MODE = 25,
	BACKUP_MQTT = 26,
	DISPLAY_ENG_MODE = 27,
	GW_RFCH = 28,
	DISPALY_PAIRS = 29,
	END_PARAM_TYPE = 30
}parameter_type_t;

esp_err_t save_tornadoedge_value(void);

esp_err_t save_tornadoedge_su_value(void);

esp_err_t save_tornadoedge_display_value(void);

esp_err_t save_aerobox_gw_value(void);

esp_err_t save_te_config_value(void);

esp_err_t erase_flash_nvs_value(void);

esp_err_t init_flash_nvs_value(void);

esp_err_t erase_tornadoedge_value(void);

esp_err_t erase_aerobox_gw_value(void);

esp_err_t erase_te_config_value(void);

esp_err_t set_su_id(int *pos, const char *su_ID);

esp_err_t set_display_id(int *pos, const char *su_ID);

esp_err_t set_aerobox_id(int *pos, const char *su_ID);

esp_err_t set_lr_id(const char *lr_ID);

esp_err_t set_gw_id(const char *gw_ID);

// old function name set_gw_id
esp_err_t set_hex_gateway_id(const char *hex_ID);

esp_err_t set_auth_gateway_id(const char *gw_name);

esp_err_t set_auth_gateway_pwd(const char *gw_pwd);

// old function name set_lr_id
esp_err_t set_auth_gateway_pair_id(const char *hex_lr_id);

esp_err_t set_auth_uri(const char *uri);

esp_err_t set_pub_uri(const char *uri);

esp_err_t set_sub_uri(const char *uri);

esp_err_t set_psk_uri(const char *uri);

esp_err_t set_pskhint_uri(const char *uri);

esp_err_t set_hostname_uri(const char *uri);

esp_err_t set_certpem_uri(const char *uri);

esp_err_t set_param_update_timestamp(long int *servertime);

esp_err_t set_heartbeat_resolution(uint8_t *intnum);

esp_err_t set_msu_query_resolution(uint8_t *intnum);

esp_err_t set_local_muticast_resolution(uint8_t *intnum);

esp_err_t set_mqtt_query_resolution(uint8_t *intnum);

esp_err_t set_mqtt_data_age(int *intnum);

esp_err_t set_local_muticast(bool boolen);

esp_err_t set_mqtt_display_mode(bool boolen);

esp_err_t set_aerobox_gw_mode(bool boolen);

esp_err_t set_sq_gw_mode(bool boolen);

esp_err_t set_backup_mqtt(bool boolen);

esp_err_t set_display_eng_mode(bool boolen);

esp_err_t set_gw_rfch(uint8_t *intnum);

char* get_all_info(int *num);

char* get_status_str(status_code_t status_id);

// char* get_event_str(send_event_code_t event_id);
char* get_message_type_str(message_type_v2_t msg_id);

char* get_message_cate_str(message_type_v2_t msg_id);

char* get_message_var_str(message_type_v2_t msg_id);

char* get_param_type_str(parameter_type_t param_id);

message_type_v2_t detect_msg_type(char *parse_str);

message_type_v2_t detect_msg_type_from_topic(char *parse_str);

parameter_type_t detect_param_type(char *parse_str);

int parse_mqtt_param_update(parameter_type_t param, cJSON *parsed_data);

int parse_mqtt_param_get(parameter_type_t param, cJSON *parsed_data, cJSON *response);

struct aerobox_auth_settings_t
{
	uint8_t aerobox_gw[10];
	uint8_t aerobox_pair[10][10];
	uint8_t mqtt_uri[50];
};
extern struct aerobox_auth_settings_t aerobox_auth_settings;

/**
 *  @brief tornadoedge flash save value
 * The actual auther settings in use
 */
struct tornadoedge_auth_settings_v1_t
{
	uint8_t hex_client_id[10];
	uint8_t auth_client_id[20];
	uint8_t auth_client_pwd[20];
	uint8_t ecu_gw_pair[20];
	uint8_t ecu_pairs[10][20];
	uint8_t mqtt_auth_uri[150];
	uint8_t mqtt_pub_uri[50];
	uint8_t auth_psk[30];
	uint8_t auth_psk_hint[20];
	uint8_t mqtt_sub_uri[50];
	uint8_t device_hostname[20];
	long int param_update_timestamp;
};
extern struct tornadoedge_auth_settings_v1_t tornadoedge_auth_settings_v1;

/**
 *  @brief tornadoedge auth settgins v2 in flash save value
 * The actual auther settings in use
 */
struct tornadoedge_auth_settings_v2_t
{
	uint8_t hex_client_id[10]; 			// E1130001
	uint8_t auth_client_id[20];			// te-xx-000001
	uint8_t auth_client_pwd[20];		// tornadoedge_password
	uint8_t mqtt_auth_uri[150];
	uint8_t mqtt_pub_uri[50];
	uint8_t mqtt_sub_uri[50];
	uint8_t device_hostname[20];
	long int param_update_timestamp;
};
extern struct tornadoedge_auth_settings_v2_t tornadoedge_auth_settings_v2;

/**
 *  @brief tornadoedge sensor unit v2 flash save value
 * The actual auther settings in use
 */
struct tornadoedge_sensor_unit_v2_t
{
	uint8_t sensor_unit_pairs[16][16];
};
extern struct tornadoedge_sensor_unit_v2_t tornadoedge_sensor_unit_v2;

/**
 *  @brief tornadoedge display unit v2 flash save value
 * The actual auther settings in use
 */
struct tornadoedge_display_unit_v2_t
{
	uint8_t display_pairs[16][16];
};
extern struct tornadoedge_display_unit_v2_t tornadoedge_display_unit_v2;

/**
 *  @brief tornadoedge flash save value
 * The actual auther settings in use
 */
struct tornadoedge_config_v1_t
{
	uint8_t heartbeat_resolution;
	bool local_muticast_mode;
	uint8_t su_query_resolution;
	uint8_t local_muticast_resolution;
	uint8_t data_upload_resolution;
	uint8_t mqtt_query_resolution;  // sec
	int mqtt_query_data_age;   // sec
	bool mqtt_display_mode;
	bool aerobox_gw_mode;
	bool lcd_1602_mode;
	bool backup_mqtt;
	bool display_eng_mode;
	uint8_t gw_rfch;
};
extern struct tornadoedge_config_v1_t tornadoedge_configs_v1;

/**
 *  @brief tornadoedge flash save value
 * The actual auther settings in use
 */
struct device_config_t
{
	uint8_t device_mac[20];
	uint8_t tornado_ver[20];
};
extern struct device_config_t device_configs;

typedef struct data_model_t 
{
    uint8_t time_record[6];
	uint8_t lr_gw_data[4];
	uint8_t retry;
	uint8_t request_state;

} data_model_t;

typedef struct lr_gw_pair_model_t 
{
    bool status;
	bool sync_from_server;
	uint8_t pair_num;
	uint8_t pair_quantity;

} lr_gw_pair_model_t;

typedef struct aerobox_data_model_v1_t
{
	uint8_t te_module_id[20];
	uint8_t aerobox_data[64];

} aerobox_data_model_v1_t;

typedef struct sq_data_model_v2_t
{
	uint8_t msu_id[6];
	int16_t weight;
	int16_t raw;
	int16_t battery;
	uint8_t rfch;
	uint8_t rforder;
	uint8_t error_code;
	int16_t k1;
	int16_t k2;
} sq_data_model_v2_t;

typedef struct te_gw_model_v1_t
{
	uint8_t discover_st;
	uint8_t discover_space;
	uint8_t te_module_id[20];
	uint8_t device_mac[20];
	uint8_t te_gw_ver[20];
	uint8_t discover_cks;
	uint8_t discover_ed;

} te_gw_model_v1_t;

struct display_data_model_t
{
    uint8_t start_byte;
	uint8_t display_id[4];
	uint8_t t_color;
	uint8_t temp[2];
    uint8_t light_color;
    uint8_t blackspace;
    uint8_t light[2];
	uint8_t pm_color;
	uint8_t pm2_5[3];
    uint8_t rh_color;
	uint8_t rh[2];
    uint8_t blackspace2[2];
    uint8_t co2_color;
	uint8_t co2[4];
	uint8_t end_byte;

};
extern struct display_data_model_t display_data_model;

char compile_ver[50];

bool TIME_SYNC;

bool SET_RFCH;

#ifdef __cplusplus
}
#endif
