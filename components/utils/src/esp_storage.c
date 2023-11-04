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

#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "errno.h"

#include "esp_tls.h"
#include "esp_crt_bundle.h"

#include "esp_utils.h"
#include "esp_storage.h"
#include "wifi_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

static const char *TAG = "esp_storage";
char *respones_json = NULL;

#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */

static char ota_write_data[BUFFSIZE + 1] = { 0 };

#define OTA_URL_SIZE 256

unsigned char mac_base[6] = {0};
// esp_efuse_mac_get_default(mac_base);
// esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
const char mac_str[] = "%02X:%02X:%02X:%02X:%02X:%02X";
char mac_base_str[20];


// /**
//  * The actual auther settings in use
//  */
// struct tornadoedge_auth_settings_t tornadoedge_auth_settings = {
//     .hex_client_id = CONFIG_HEX_CLIENT_ID,
//     .auth_client_id = CONFIG_AUTH_CLIENT_ID,
//     .auth_client_pwd = CONFIG_AUTH_CLIENT_SECRET,
//     .ecu_gw_pair = CONFIG_ECU_GW_PAIR,
//     .ecu_pairs[0] = CONFIG_ECU_PAIR_01,
//     .mqtt_auth_uri = CONFIG_MQTT_AUTH_URI,
//     .mqtt_pub_uri = CONFIG_TORNADOEDGE_MQTT_URI,
//     .auth_psk = CONFIG_AUTH_PSK,
//     .auth_psk_hint = CONFIG_AUTH_PSK_HINT,
//     .mqtt_sub_uri = CONFIG_TORNADOEDGE_SUB_URI,
//     .device_hostname = CONFIG_DEVICE_HOSTNAME,
//     .param_update_timestamp = 0,
// };

struct tornadoedge_auth_settings_v2_t tornadoedge_auth_settings_v2 = {
    .hex_client_id = CONFIG_HEX_CLIENT_ID,
    .auth_client_id = CONFIG_AUTH_CLIENT_ID,
    .auth_client_pwd = CONFIG_AUTH_CLIENT_SECRET,
    .mqtt_auth_uri = CONFIG_MQTT_AUTH_URI,
    .mqtt_pub_uri = CONFIG_TORNADOEDGE_PUB_URI,
    .mqtt_sub_uri = CONFIG_TORNADOEDGE_SUB_URI,
    .device_hostname = CONFIG_DEVICE_HOSTNAME,
    .param_update_timestamp = 0,
};

struct tornadoedge_sensor_unit_v2_t tornadoedge_sensor_unit_v2 = {
    .sensor_unit_pairs[0] = CONFIG_SENSOR_UINT_01,
};

struct tornadoedge_display_unit_v2_t tornadoedge_display_unit_v2 = {
    .display_pairs[0] = CONFIG_DISPLAY_UINT_01,
};


struct display_data_model_t display_data_model = {
        .start_byte = 0x24,
	    .display_id = "001\x2c",
	    .t_color = color_green,
	    .temp = "00",
        .light_color = color_green,
        .blackspace = blackspace_default,
        .light = lightcode,
	    .pm_color = color_red,
	    .pm2_5 = "000",
        .rh_color = color_yellow,
	    .rh = "00",
        .blackspace2 = blackspace2_default,
        .co2_color = color_green,
	    .co2 = "0000",
	    .end_byte = 0x23,
};

struct aerobox_auth_settings_t aerobox_auth_settings = {
    .aerobox_gw = CONFIG_AEROBOX_GW,
    .aerobox_pair[0] = CONFIG_AEROBOX_PAIR,
};

struct tornadoedge_config_v1_t tornadoedge_configs_v1 = {
    .heartbeat_resolution = 20,
	.local_muticast_mode = false,
	.su_query_resolution = 10,
	.local_muticast_resolution = 2,
	.data_upload_resolution = 10,
	.mqtt_query_resolution = 30,
	.mqtt_query_data_age = 3600,
	.mqtt_display_mode = false,
	.aerobox_gw_mode = false,
	.lcd_1602_mode = true,
    .backup_mqtt = true,
    .display_eng_mode = true,
    .gw_rfch = 20,
};

struct device_config_t device_configs = {
    .device_mac = "00:00:00:00:00:00",
    .tornado_ver = "v2.0.6",
};

esp_err_t esp_storage_init()
{
    static bool init_flag = false;
    size_t sz;
    esp_log_level_set(TAG, ESP_LOG_INFO);
    if (!init_flag) {
        esp_err_t ret = nvs_flash_init();

        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }

        ESP_ERROR_CHECK(ret);

        init_flag = true;
    }
    sz = sizeof(tornadoedge_auth_settings_v2);
    esp_storage_get("teauthv2", &tornadoedge_auth_settings_v2, sz);

    sz = sizeof(tornadoedge_sensor_unit_v2);
    esp_storage_get("tesuv2", &tornadoedge_sensor_unit_v2, sz);

    sz = sizeof(tornadoedge_display_unit_v2);
    esp_storage_get("tedisplayv2", &tornadoedge_display_unit_v2, sz);

    // sz = sizeof(aerobox_auth_settings);
    // esp_storage_get("aerobox", &aerobox_auth_settings, sz);

    // force save to new config
    // save_te_config_value();
    sz = sizeof(tornadoedge_configs_v1);
    esp_storage_get("tecfgv1", &tornadoedge_configs_v1, sz);
    esp_efuse_mac_get_default(mac_base);
    esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
    snprintf(mac_base_str, (size_t)20,mac_str, mac_base[0],mac_base[1],mac_base[2],mac_base[3],mac_base[4],mac_base[5]);
    memcpy(device_configs.device_mac,
                                mac_base_str,
                                sizeof(device_configs.device_mac));
    return ESP_OK;
}

esp_err_t esp_storage_erase(const char *key)
{
    // ESP_PARAM_CHECK(key);

    esp_err_t ret    = ESP_OK;
    nvs_handle handle = 0;

    /**< Open non-volatile storage with a given namespace from the default NVS partition */
    ret = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Open non-volatile storage");

    /**
     * @brief If key is CONFIG_NVS_NAMESPACE, erase all info in CONFIG_NVS_NAMESPACE
     */
    if (!key) {
        ret = nvs_erase_all(handle);
    } else {
        ret = nvs_erase_key(handle, key);
    }

    /**< Write any pending changes to non-volatile storage */
    nvs_commit(handle);

    /**< Close the storage handle and free any allocated resources */
    nvs_close(handle);

    ESP_ERROR_RETURN(ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND,
                     ret, "Erase key-value pair, key: %s", key);

    return ESP_OK;
}

esp_err_t esp_storage_set(const char *key, const void *value, size_t length)
{
    ESP_PARAM_CHECK(key);
    ESP_PARAM_CHECK(value);
    ESP_PARAM_CHECK(length > 0);

    esp_err_t ret     = ESP_OK;
    nvs_handle handle = 0;

    /**< Open non-volatile storage with a given namespace from the default NVS partition */
    ret = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Open non-volatile storage");

    /**< set variable length binary value for given key */
    ret = nvs_set_blob(handle, key, value, length);

    /**< Write any pending changes to non-volatile storage */
    nvs_commit(handle);

    /**< Close the storage handle and free any allocated resources */
    nvs_close(handle);

    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Set value for given key, key: %s", key);

    return ESP_OK;
}

esp_err_t esp_storage_get(const char *key, void *value, size_t length)
{
    ESP_PARAM_CHECK(key);
    ESP_PARAM_CHECK(value);
    ESP_PARAM_CHECK(length > 0);

    esp_err_t ret     = ESP_OK;
    nvs_handle handle = 0;

    /**< Open non-volatile storage with a given namespace from the default NVS partition */
    ret = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Open non-volatile storage");

    /**< get variable length binary value for given key */
    ret = nvs_get_blob(handle, key, value, &length);

    /**< Close the storage handle and free any allocated resources */
    nvs_close(handle);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "<ESP_ERR_NVS_NOT_FOUND> Get value for given key, key: %s", key);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Get value for given key, key: %s", key);

    return ESP_OK;
}

/* Save new run time value in NVS
   by first reading a table of previously saved values
   and then adding the new value at the end of the table.
   Return an error if anything goes wrong
   during this process.
 */
esp_err_t save_tornadoedge_value(void)
{
    esp_err_t esp_err;
    size_t sz;
    char ap_ssid_name[32];
    snprintf(ap_ssid_name,(size_t)32,"TornadoEdge_%s",(char*)tornadoedge_auth_settings_v2.auth_client_id);
    set_new_ap_ssid(ap_ssid_name);
    /* variables used to check if write is really needed */
	// aerobox_auth_config_t tmp_conf;
	struct tornadoedge_auth_settings_v2_t tmp_settings;
	memset(&tmp_settings, 0x00, sizeof(tmp_settings));
	bool change = false;
    if(true){
        sz = sizeof(tmp_settings);
        esp_err = esp_storage_get("teauthv2", &tmp_settings, sz);
        ESP_LOGI(TAG, "flash_manager auth_settings: hex_client_id: %s",tmp_settings.hex_client_id);
        ESP_LOGI(TAG, "flash_manager auth_settings: auth_client_id: %s",tmp_settings.auth_client_id);
        ESP_LOGI(TAG, "flash_manager auth_settings: auth_client_secret: %s",tmp_settings.auth_client_pwd);
        ESP_LOGD(TAG, "flash_manager auth_settings: mqtt_auth_uri: %s",tmp_settings.mqtt_auth_uri);
        ESP_LOGD(TAG, "flash_manager auth_settings: mqtt_sub_uri: %s",tmp_settings.mqtt_sub_uri);
        ESP_LOGD(TAG, "flash_manager auth_settings: device_hostname: %s",tmp_settings.device_hostname);
        if( (esp_err == ESP_OK  || esp_err == ESP_ERR_NVS_NOT_FOUND) &&
                (
                    strcmp( (char*)tmp_settings.hex_client_id,(char*)tornadoedge_auth_settings_v2.hex_client_id) != 0 ||
                    strcmp( (char*)tmp_settings.auth_client_id,(char*)tornadoedge_auth_settings_v2.auth_client_id) != 0 ||
                    strcmp( (char*)tmp_settings.auth_client_pwd,(char*)tornadoedge_auth_settings_v2.auth_client_pwd) != 0 ||
                    strcmp( (char*)tmp_settings.mqtt_auth_uri,(char*)tornadoedge_auth_settings_v2.mqtt_auth_uri) != 0 ||
                    strcmp( (char*)tmp_settings.mqtt_sub_uri,(char*)tornadoedge_auth_settings_v2.mqtt_sub_uri) != 0 ||
                    strcmp( (char*)tmp_settings.device_hostname,(char*)tornadoedge_auth_settings_v2.device_hostname) != 0
                )
        ){
            esp_err = esp_storage_set("teauthv2", &tornadoedge_auth_settings_v2, sizeof(tornadoedge_auth_settings_v2));
            if (esp_err != ESP_OK) return esp_err;
            change = true;
            ESP_LOGD(TAG, "flash_manager_wrote auth_settings: hex_client_id: %s",tornadoedge_auth_settings_v2.hex_client_id);
            ESP_LOGD(TAG, "flash_manager_wrote auth_settings: auth_client_id: %s",tornadoedge_auth_settings_v2.auth_client_id);
            ESP_LOGD(TAG, "flash_manager_wrote auth_settings: auth_client_secret: %s",tornadoedge_auth_settings_v2.auth_client_pwd);
            ESP_LOGD(TAG, "flash_manager_wrote auth_settings: mqtt_auth_uri: %s",tornadoedge_auth_settings_v2.mqtt_auth_uri);
            ESP_LOGD(TAG, "flash_manager_wrote auth_settings: mqtt_sub_uri: %s",tornadoedge_auth_settings_v2.mqtt_sub_uri);
            ESP_LOGD(TAG, "flash_manager_wrote auth_settings: device_hostname: %s",tornadoedge_auth_settings_v2.device_hostname);
        }

        if(change){
			// esp_err = nvs_commit(my_handle);
            ESP_LOGI(TAG, "tornadoedge config V2 was saved to flash.");
		}
		else{
			ESP_LOGI(TAG, "tornadoedge config V2 was not saved to flash because no change has been detected.");
		}

		// if (esp_err != ESP_OK) return esp_err;

		// nvs_close(my_handle);
    }
	else{
		ESP_LOGE(TAG, "flash_manager_save_tornadoedge_config failed to acquire nvs_sync mutex");
	}

	return ESP_OK;
}

esp_err_t save_tornadoedge_su_value(void)
{
    esp_err_t esp_err;
    size_t sz;
    /* variables used to check if write is really needed */
	// aerobox_auth_config_t tmp_conf;
	struct tornadoedge_sensor_unit_v2_t tmp_settings;
	memset(&tmp_settings, 0x00, sizeof(tmp_settings));
	bool change = false;
    if(true){
        sz = sizeof(tmp_settings);
        esp_err = esp_storage_get("tesuv2", &tmp_settings, sz);
        ESP_LOGI(TAG, "flash_manager su_pairs[0]: %s",tmp_settings.sensor_unit_pairs[0]);
        ESP_LOGI(TAG, "flash_manager su_pairs[1]: %s",tmp_settings.sensor_unit_pairs[1]);
        if( (esp_err == ESP_OK  || esp_err == ESP_ERR_NVS_NOT_FOUND) &&
                (
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[0],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[0]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[1],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[1]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[2],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[2]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[3],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[3]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[4],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[4]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[5],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[5]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[6],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[6]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[7],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[7]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[8],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[8]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[9],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[9]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[10],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[10]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[11],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[11]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[12],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[12]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[13],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[13]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[14],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[14]) != 0 ||
                    strcmp( (char*)tmp_settings.sensor_unit_pairs[15],(char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[15]) != 0
                )
        ){
            esp_err = esp_storage_set("tesuv2", &tornadoedge_sensor_unit_v2, sizeof(tornadoedge_sensor_unit_v2));
            if (esp_err != ESP_OK) return esp_err;
            change = true;
            ESP_LOGI(TAG, "flash_manager set su_pairs[0]: %s",tornadoedge_sensor_unit_v2.sensor_unit_pairs[0]);
            ESP_LOGI(TAG, "flash_manager set su_pairs[1]: %s",tornadoedge_sensor_unit_v2.sensor_unit_pairs[1]);
        }

        if(change){
			// esp_err = nvs_commit(my_handle);
            ESP_LOGI(TAG, "tornadoedge_sensor_unit_v2 was saved to flash.");
		}
		else{
			ESP_LOGI(TAG, "tornadoedge_sensor_unit_v2 was not saved to flash because no change has been detected.");
		}

		// if (esp_err != ESP_OK) return esp_err;

		// nvs_close(my_handle);
    }
	else{
		ESP_LOGE(TAG, "flash_manager_save_tornadoedge_config failed to acquire nvs_sync mutex");
	}

	return ESP_OK;
}

esp_err_t save_tornadoedge_display_value(void)
{
    esp_err_t esp_err;
    size_t sz;
    /* variables used to check if write is really needed */
	// aerobox_auth_config_t tmp_conf;
	struct tornadoedge_display_unit_v2_t tmp_settings;
	memset(&tmp_settings, 0x00, sizeof(tmp_settings));
	bool change = false;
    if(true){
        sz = sizeof(tmp_settings);
        esp_err = esp_storage_get("tedisplayv2", &tmp_settings, sz);
        ESP_LOGI(TAG, "flash_manager display_pairs[0]: %s",tmp_settings.display_pairs[0]);
        ESP_LOGI(TAG, "flash_manager display_pairs[1]: %s",tmp_settings.display_pairs[1]);
        if( (esp_err == ESP_OK  || esp_err == ESP_ERR_NVS_NOT_FOUND) &&
                (
                    strcmp( (char*)tmp_settings.display_pairs[0],(char*)tornadoedge_display_unit_v2.display_pairs[0]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[1],(char*)tornadoedge_display_unit_v2.display_pairs[1]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[2],(char*)tornadoedge_display_unit_v2.display_pairs[2]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[3],(char*)tornadoedge_display_unit_v2.display_pairs[3]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[4],(char*)tornadoedge_display_unit_v2.display_pairs[4]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[5],(char*)tornadoedge_display_unit_v2.display_pairs[5]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[6],(char*)tornadoedge_display_unit_v2.display_pairs[6]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[7],(char*)tornadoedge_display_unit_v2.display_pairs[7]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[8],(char*)tornadoedge_display_unit_v2.display_pairs[8]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[9],(char*)tornadoedge_display_unit_v2.display_pairs[9]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[10],(char*)tornadoedge_display_unit_v2.display_pairs[10]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[11],(char*)tornadoedge_display_unit_v2.display_pairs[11]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[12],(char*)tornadoedge_display_unit_v2.display_pairs[12]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[13],(char*)tornadoedge_display_unit_v2.display_pairs[13]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[14],(char*)tornadoedge_display_unit_v2.display_pairs[14]) != 0 ||
                    strcmp( (char*)tmp_settings.display_pairs[15],(char*)tornadoedge_display_unit_v2.display_pairs[15]) != 0
                )
        ){
            esp_err = esp_storage_set("tedisplayv2", &tornadoedge_display_unit_v2, sizeof(tornadoedge_display_unit_v2));
            if (esp_err != ESP_OK) return esp_err;
            change = true;
            ESP_LOGI(TAG, "flash_manager set display_pairs[0]: %s",tornadoedge_display_unit_v2.display_pairs[0]);
            ESP_LOGI(TAG, "flash_manager set display_pairs[1]: %s",tornadoedge_display_unit_v2.display_pairs[1]);
        }

        if(change){
			// esp_err = nvs_commit(my_handle);
            ESP_LOGI(TAG, "tornadoedge_display_unit_v2 was saved to flash.");
		}
		else{
			ESP_LOGI(TAG, "tornadoedge_display_unit_v2 was not saved to flash because no change has been detected.");
		}

		// if (esp_err != ESP_OK) return esp_err;

		// nvs_close(my_handle);
    }
	else{
		ESP_LOGE(TAG, "flash_manager_save_tornadoedge_config failed to acquire nvs_sync mutex");
	}

	return ESP_OK;
}

esp_err_t save_te_config_value(void)
{
    esp_err_t esp_err;
    size_t sz;
    /* variables used to check if write is really needed */
	// aerobox_auth_config_t tmp_conf;
	struct tornadoedge_config_v1_t tmp_settings;
	memset(&tmp_settings, 0x00, sizeof(tmp_settings));
	bool change = false;
    if(true){
        sz = sizeof(tmp_settings);
        esp_err = esp_storage_get("tecfgv1", &tmp_settings, sz);
        // ESP_LOGI(TAG, "flash_manager auth_settings: aerobox_gw: %s",tmp_settings.aerobox_gw);
        // ESP_LOGI(TAG, "flash_manager auth_settings: mqtt_uri: %s",tmp_settings.mqtt_uri);
        if( (esp_err == ESP_OK || esp_err == ESP_ERR_NVS_NOT_FOUND) && (1)
                // (
                //     strcmp( (char*)tmp_settings.heartbeat_resolution,(char*)tornadoedge_configs_v1_v1.heartbeat_resolution) != 0 ||
                //     strcmp( (char*)tmp_settings.local_muticast,(char*)tornadoedge_configs_v1_v1.local_muticast) != 0 ||
                // )
        ){
            esp_err = esp_storage_set("tecfgv1", &tornadoedge_configs_v1, sizeof(tornadoedge_configs_v1));
            if (esp_err != ESP_OK) return esp_err;
            change = true;
            // ESP_LOGI(TAG, "flash_manager_wrote auth_settings: aerobox_gw: %s",aerobox_auth_settings.aerobox_gw);
        }

        if(change){
			// esp_err = nvs_commit(my_handle);
            ESP_LOGI(TAG, "tornadoedge config was saved to flash.");
		}
		else{
			ESP_LOGI(TAG, "tornadoedge config was not saved to flash because no change has been detected.");
		}

		// if (esp_err != ESP_OK) return esp_err;

		// nvs_close(my_handle);
    }
	else{
		ESP_LOGE(TAG, "flash_manager tornadoedge config failed to acquire nvs_sync mutex");
	}

	return ESP_OK;
}

esp_err_t erase_flash_nvs_value(void)
{
    esp_err_t esp_err;
    // erase flash partition all data
    esp_err = nvs_flash_erase_partition("nvs");
    if (esp_err != ESP_OK) ESP_LOGI(TAG, "Error (%s) erasing!\n", esp_err_to_name(esp_err));
    return ESP_OK;
}

esp_err_t init_flash_nvs_value(void)
{
    esp_err_t esp_err;
    // erase flash partition all data
    esp_err = nvs_flash_init();
    if (esp_err == ESP_ERR_NVS_NO_FREE_PAGES || esp_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            esp_err = nvs_flash_init();
        }
    return ESP_OK;
}

esp_err_t erase_tornadoedge_value(void)
{
    esp_err_t esp_err;
    esp_err = esp_storage_erase("teauthv2");
    if (esp_err != ESP_OK) ESP_LOGI(TAG, "Error (%s) erasing!\n", esp_err_to_name(esp_err));
    
    return ESP_OK;
}

esp_err_t erase_tornadoedge_su_value(void)
{
    esp_err_t esp_err;
    esp_err = esp_storage_erase("tesuv2");
    if (esp_err != ESP_OK) ESP_LOGI(TAG, "Error (%s) erasing!\n", esp_err_to_name(esp_err));
    
    return ESP_OK;
}


esp_err_t erase_aerobox_gw_value(void)
{
    esp_err_t esp_err;
    esp_err = esp_storage_erase("aerobox");
    if (esp_err != ESP_OK) ESP_LOGI(TAG, "Error (%s) erasing!\n", esp_err_to_name(esp_err));
    
    return ESP_OK;
}

esp_err_t erase_te_config_value(void)
{
    esp_err_t esp_err;
    esp_err = esp_storage_erase("tecfgv1");
    if (esp_err != ESP_OK) ESP_LOGI(TAG, "Error (%s) erasing!\n", esp_err_to_name(esp_err));
    
    return ESP_OK;
}

esp_err_t set_su_id(int *pos, const char *su_ID)
{
    if (*pos <= 16){
        memcpy(tornadoedge_sensor_unit_v2.sensor_unit_pairs[*pos-1],
                    su_ID,
                    sizeof(tornadoedge_sensor_unit_v2.sensor_unit_pairs[0]));
    }
    else {
        return 1;
    } // tornadoedge_auth_settings
    
    return ESP_OK;
}

esp_err_t set_display_id(int *pos, const char *su_ID)
{
    if (*pos <= 16){
        memcpy(tornadoedge_display_unit_v2.display_pairs[*pos-1],
                    su_ID,
                    sizeof(tornadoedge_display_unit_v2.display_pairs[0]));
    }
    else {
        return 1;
    } // tornadoedge_auth_settings
    
    return ESP_OK;
}

esp_err_t set_aerobox_id(int *pos, const char *su_ID)
{
    // esp_err_t esp_err;
    switch (*pos)
    {
    case 1:
        memcpy(aerobox_auth_settings.aerobox_pair[0],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[0]));
        break;
    case 2:
        memcpy(aerobox_auth_settings.aerobox_pair[1],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[1]));
        break;
    case 3:
        memcpy(aerobox_auth_settings.aerobox_pair[2],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[2]));
        break;
    case 4:
        memcpy(aerobox_auth_settings.aerobox_pair[3],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[3]));
        break;
    case 5:
        memcpy(aerobox_auth_settings.aerobox_pair[4],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[4]));
        break;
    case 6:
        memcpy(aerobox_auth_settings.aerobox_pair[5],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[5]));
        break;
    case 7:
        memcpy(aerobox_auth_settings.aerobox_pair[6],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[6]));
        break;
    case 8:
        memcpy(aerobox_auth_settings.aerobox_pair[7],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[7]));
        break;
    case 9:
        memcpy(aerobox_auth_settings.aerobox_pair[8],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[8]));
        break;
    case 10:
        memcpy(aerobox_auth_settings.aerobox_pair[9],
                                su_ID,
                                sizeof(aerobox_auth_settings.aerobox_pair[9]));
        break;
    
    default:
        break;
    }
    
    return ESP_OK;
}

esp_err_t set_lr_id(const char *lr_ID)
{
    // memcpy(tornadoedge_auth_settings.ecu_gw_pair,
    //                             lr_ID,
    //                             sizeof(tornadoedge_auth_settings.ecu_gw_pair));
    return ESP_OK;

}

esp_err_t set_hex_gateway_id(const char *hex_ID)
{
    memcpy(tornadoedge_auth_settings_v2.hex_client_id,
                                hex_ID,
                                sizeof(tornadoedge_auth_settings_v2.hex_client_id));
    return ESP_OK;

}

esp_err_t set_auth_gateway_id(const char *gw_name)
{
    memcpy(tornadoedge_auth_settings_v2.auth_client_id,
                                gw_name,
                                sizeof(tornadoedge_auth_settings_v2.auth_client_id));
    return ESP_OK;

}

esp_err_t set_auth_gateway_pwd(const char *gw_pwd)
{
    memcpy(tornadoedge_auth_settings_v2.auth_client_pwd,
                                gw_pwd,
                                sizeof(tornadoedge_auth_settings_v2.auth_client_pwd));
    return ESP_OK;

}

esp_err_t set_auth_gateway_pair_id(const char *hex_lr_id)
{
    // memcpy(tornadoedge_auth_settings.ecu_gw_pair,
    //                             hex_lr_id,
    //                             sizeof(tornadoedge_auth_settings.ecu_gw_pair));
    return ESP_OK;

}

esp_err_t set_gw_id(const char *gw_ID)
{
    // memcpy(tornadoedge_auth_settings.hex_client_id,
    //                             gw_ID,
    //                             sizeof(tornadoedge_auth_settings.hex_client_id));
    return ESP_OK;

}


esp_err_t set_auth_uri(const char *uri)
{
    memcpy(tornadoedge_auth_settings_v2.mqtt_auth_uri,
                                uri,
                                sizeof(tornadoedge_auth_settings_v2.mqtt_auth_uri));
    return ESP_OK;

}

esp_err_t set_pub_uri(const char *uri)
{
    memcpy(tornadoedge_auth_settings_v2.mqtt_pub_uri,
                                uri,
                                sizeof(tornadoedge_auth_settings_v2.mqtt_pub_uri));
    return ESP_OK;

}

esp_err_t set_sub_uri(const char *uri)
{
    memcpy(tornadoedge_auth_settings_v2.mqtt_sub_uri,
                                uri,
                                sizeof(tornadoedge_auth_settings_v2.mqtt_sub_uri));
    return ESP_OK;

}

esp_err_t set_psk_uri(const char *uri)
{
    // memcpy(tornadoedge_auth_settings.auth_psk,
    //                             uri,
    //                             sizeof(tornadoedge_auth_settings.auth_psk));
    return ESP_OK;

}

esp_err_t set_pskhint_uri(const char *uri)
{
    // memcpy(tornadoedge_auth_settings.auth_psk_hint,
    //                             uri,
    //                             sizeof(tornadoedge_auth_settings.auth_psk_hint));
    return ESP_OK;

}

esp_err_t set_hostname_uri(const char *uri)
{
    memcpy(tornadoedge_auth_settings_v2.device_hostname,
                                uri,
                                sizeof(tornadoedge_auth_settings_v2.device_hostname));
    return ESP_OK;

}

esp_err_t set_param_update_timestamp(long int *servertime)
{
    tornadoedge_auth_settings_v2.param_update_timestamp = *servertime;
    return ESP_OK;
}

esp_err_t set_heartbeat_resolution(uint8_t *intnum)
{
    tornadoedge_configs_v1.heartbeat_resolution = *intnum;
    return ESP_OK;
}

esp_err_t set_msu_query_resolution(uint8_t *intnum)
{
    tornadoedge_configs_v1.su_query_resolution = *intnum;
    return ESP_OK;
}

esp_err_t set_local_muticast_resolution(uint8_t *intnum)
{
    tornadoedge_configs_v1.local_muticast_resolution = *intnum;
    return ESP_OK;
}

esp_err_t set_mqtt_query_resolution(uint8_t *intnum)
{
    tornadoedge_configs_v1.mqtt_query_resolution = *intnum;
    return ESP_OK;
}

esp_err_t set_mqtt_data_age(int *intnum)
{
    tornadoedge_configs_v1.mqtt_query_data_age = *intnum;
    return ESP_OK;
}

esp_err_t set_local_muticast(bool boolen)
{
    tornadoedge_configs_v1.local_muticast_mode = boolen;
    return ESP_OK;
}

esp_err_t set_mqtt_display_mode(bool boolen)
{
    tornadoedge_configs_v1.mqtt_display_mode = boolen;
    return ESP_OK;
}

esp_err_t set_aerobox_gw_mode(bool boolen)
{
    tornadoedge_configs_v1.aerobox_gw_mode = boolen;
    return ESP_OK;
}

esp_err_t set_sq_gw_mode(bool boolen)
{
    tornadoedge_configs_v1.lcd_1602_mode = boolen;
    return ESP_OK;
}

esp_err_t set_backup_mqtt(bool boolen)
{
    tornadoedge_configs_v1.backup_mqtt = boolen;
    return ESP_OK;
}

esp_err_t set_display_eng_mode(bool boolen)
{
    tornadoedge_configs_v1.display_eng_mode = boolen;
    return ESP_OK;    
}

esp_err_t set_gw_rfch(uint8_t *intnum)
{
    tornadoedge_configs_v1.gw_rfch = *intnum;
    SET_RFCH = true;
    return ESP_OK;    
}

char* get_all_info(int *num)
{

    respones_json = (char*)malloc(500); 
    strcpy(respones_json, "");

    char one_var[400];
    const char gw_lr_mqtt_str[] = "E:{\"tegw\":\"%s\",\"hostname\":\"%s\",\"update_time\":\"%ld\",\"mqttUri\":\"%s\",\"psk\":\"%s\",\"pskhint\":\"%s\",\"pub\":\"%s\",\"sub\":\"%s\",\"DeviceMac\":\"%s\"";
    const char su_str[] =  ",\"su%d\":\"%s\"";
    const char dp_str[] =  ",\"dp%d\":\"%s\"";
    snprintf(one_var, (size_t)400,gw_lr_mqtt_str,
            tornadoedge_auth_settings_v2.auth_client_id,
            tornadoedge_auth_settings_v2.device_hostname,
            tornadoedge_auth_settings_v2.param_update_timestamp,
            tornadoedge_auth_settings_v2.mqtt_auth_uri,
            tornadoedge_auth_settings_v2.auth_client_id,
            tornadoedge_auth_settings_v2.auth_client_id,
            tornadoedge_auth_settings_v2.mqtt_pub_uri,
            tornadoedge_auth_settings_v2.mqtt_sub_uri,
            device_configs.device_mac
            );
    strcat(respones_json, one_var);
    for (int i=0; i<*num; i++){
        snprintf(one_var, (size_t)400,su_str,
                i+1,
                (char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[i]
                );
        strcat(respones_json, one_var);
    }
    for (int i=0; i<*num; i++){
        snprintf(one_var, (size_t)400,dp_str,
                i+1,
                (char*)tornadoedge_display_unit_v2.display_pairs[i]
                );
        strcat(respones_json, one_var);
    }
    strcat(respones_json, "}\0");
    return respones_json;
}

char* get_status_str(status_code_t status_id)
{
    switch (status_id)
    {
        case NONE_CODE:
            return "none";
            break;
        case FINE:
            return "fine";
            break;
        case FATEL:
            return "fatel";
            break;
        case ERROR:
            return "error";
            break;
        case WARNING:
            return "warning";
            break;
        case INFO:
            return "info";
            break;
        case DEBUG:
            return "debug";
            break;
        case REBOOTED:
            return "rebooted";
            break;
        case WIRELESS_DOWN:
            return "wireless-down";
            break;
        case NO_RESPONSE:
            return "no-response";
            break;
        case ILLEGAL:
            return "illegal";
            break;
        default:
            break;
    }
    return "none";
}

char* get_message_type_str(message_type_v2_t msg_id){
    switch (msg_id)
    {
        case NONE_MESSAGE:
            return "none-message";
            break;
        case GATEWAY_HEARTBEAT:
            return "heartbeat";
            break;
        case GATEWAY_SYSTEM_INFO:
            return "system-info";
            break;
        case GATEWAY_GET_PARAMETER:
            return "get-parameter";
            break;
        case GATEWAY_GET_PAIRING:
            return "get-pairing";
            break;
        case GATEWAY_SET_PAIRING:
            return "set-pairing";
            break;
        case GATEWAY_GET_DISPLAY:
            return "get-display";
            break;
        case GATEWAY_SET_DISPLAY:
            return "set-display";
            break;
        case GATEWAY_PAIRING:
            return "pairing-list";
            break;
        case GATEWAY_DISPLAY:
            return "display-list";
            break;
        case GATEWAY_QUERY:
            return "data-query";
            break;
        case GATEWAY_DATA_RESPONSE:
            return "data-response";
            break;
        case GATEWAY_REBOOT:
            return "reboot";
            break;
        case GATEWAY_TIME_SYNC:
            return "time-sync";
            break;
        case GATEWAY_PARAM_UPDATE:
            return "param-update";
            break;
        case GATEWAY_REPLY:
            return "reply";
            break;
        case AEROBOX_HEARTBEAT:
            return "aerobox-heartbeat";
            break;
        case SG_HEARTBEAT:
            return "sg-heartbeat";
            break;
        case AEROBOX_RAW_DATA:
            return "aerobox-raw";
            break;
        case AEROBOX_JSON_DATA:
            return "aerobox-json";
            break;
        case SG_SEQ_JSON_DATA:
            return "sg-seq-json";
            break;
        case SG_REPORT_JSON_DATA:
            return "sg-report-json";
            break;
        case END_MESSAGE_TYPE:
            return "end-messsage";
            break;
    }
    return "non-event";
}

char* get_message_cate_str(message_type_v2_t msg_id){
    switch (msg_id)
    {
        case NONE_MESSAGE:
            return "not-do";
            break;
        case GATEWAY_HEARTBEAT:
            return "state";
            break;
        case GATEWAY_SYSTEM_INFO:
            return "state";
            break;
        case GATEWAY_GET_PARAMETER:
            return "event";
            break;
        case GATEWAY_GET_PAIRING:
            return "event";
            break;
        case GATEWAY_SET_PAIRING:
            return "event";
            break;
        case GATEWAY_GET_DISPLAY:
            return "event";
            break;
        case GATEWAY_SET_DISPLAY:
            return "event";
            break;
        case GATEWAY_PAIRING:
            return "cmd";
            break;
        case GATEWAY_DISPLAY:
            return "cmd";
            break;
        case GATEWAY_QUERY:
            return "event";
            break;
        case GATEWAY_DATA_RESPONSE:
            return "cmd";
            break;
        case GATEWAY_REBOOT:
            return "cmd";
            break;
        case GATEWAY_TIME_SYNC:
            return "cmd";
            break;
        case GATEWAY_PARAM_UPDATE:
            return "cmd";
            break;
        case GATEWAY_REPLY:
            return "event";
            break;
        case AEROBOX_HEARTBEAT:
            return "state";
            break;
        case SG_HEARTBEAT:
            return "state";
            break;
        case AEROBOX_RAW_DATA:
            return "data";
            break;
        case AEROBOX_JSON_DATA:
            return "data";
            break;
        case SG_SEQ_JSON_DATA:
            return "data";
            break;
        case SG_REPORT_JSON_DATA:
            return "data";
            break;
        case END_MESSAGE_TYPE:
            return "event";
            break;
    }
    return "event";
}

char* get_param_type_str(parameter_type_t param_id){
    switch (param_id)
    {
        case NONE_PARAM:
            return "NoneParam";
            break;
        case HEX_GATEWAY_ID:
            return "HexGatewayId";
            break;
        case AUTH_GATEWAY_ID:
            return "AuthGatewayId";
            break;
        case AUTH_GATEWAY_PWD:
            return "AuthGatewayPwd";
            break;
        case ECU_GATEWAY_PAIR_ID:
            return "EcuGatewayPairId";
            break;
        case ECU_PAIRS:
            return "EcuPairs";
            break;
        case MQTT_AUTH_URI:
            return "MqttAuthUri";
            break;
        case MQTT_PUB_URI:
            return "MqttPubUri";
            break;
        case AUTH_PSK:
            return "AuthPsk";
            break;
        case AUTH_PSK_HINT:
            return "AuthPskHint";
            break;
        case MQTT_SUB_URI:
            return "MqttSubUri";
            break;
        case DEVICE_HOSTNAME:
            return "DeviceHostname";
            break;
        case AEROBOX_GATEWAY_PAIR_ID:
            return "AeroboxGatewayPairId";
            break;
        case AEROBOX_PAIRS:
            return "AeroboxPairs";
            break;
        case AEROBOX_MQTT_URI:
            return "AeroboxMqttUri";
            break;
        case PARAM_UPDATE_TIME:
            return "ParamUpdateTime";
            break;
        case OTA_UPGRADE:
            return "OtaUpgrade";
            break;
        case CA_CERT:
            return "CaCert";
            break;
        case HEARTBEAT_RESOLUTION:
            return "HeartbeatResolution";
            break;
        case LOCAL_MUTICAST:
            return "LocalMuticast";
            break;
        case DATA_UPLOAD_RESOLUTION:
            return "DataUploadResolution";
            break;
        case MQTT_QUERY_RESOLUTION:
            return "MqttQueryResolution";
            break;
        case MQTT_DATA_AGE:
            return "MqttDataAge";
            break;
        case MQTT_DISPLAY_MODE:
            return "DisplayMode";
            break;
        case AEROBOX_GW_MODE:
            return "AeroboxGwMode";
            break;
        case SQ_GW_MODE:
            return "SqGwMode";
            break;
        case BACKUP_MQTT:
            return "BackupMqtt";
            break;
        case DISPLAY_ENG_MODE:
            return "DisplayEngMode";
            break;
        case GW_RFCH:
            return "GwRfch";
            break;
        case DISPALY_PAIRS:
            return "DisplayPairs";
            break;
        case END_PARAM_TYPE:
            return "EndParamType";
            break;
    }
    return "NoneParam";
}

char* get_message_var_str(message_type_v2_t msg_id){
    switch (msg_id)
    {
        case NONE_MESSAGE:
            return "v0";
            break;
        case GATEWAY_HEARTBEAT:
            return "v1";
            break;
        case GATEWAY_SYSTEM_INFO:
            return "v1";
            break;
        case GATEWAY_GET_PARAMETER:
            return "v2";
            break;
        case GATEWAY_GET_PAIRING:
            return "v1";
            break;
        case GATEWAY_SET_PAIRING:
            return "v1";
            break;
        case GATEWAY_GET_DISPLAY:
            return "v1";
            break;
        case GATEWAY_SET_DISPLAY:
            return "v1";
            break;
        case GATEWAY_PAIRING:
            return "v1";
            break;
        case GATEWAY_DISPLAY:
            return "v1";
            break;
        case GATEWAY_QUERY:
            return "v1";
            break;
        case GATEWAY_DATA_RESPONSE:
            return "v1";
            break;
        case GATEWAY_REBOOT:
            return "v1";
            break;
        case GATEWAY_TIME_SYNC:
            return "v1";
            break;
        case GATEWAY_PARAM_UPDATE:
            return "v1";
            break;
        case GATEWAY_REPLY:
            return "v1";
            break;
        case AEROBOX_HEARTBEAT:
            return "v1";
            break;
        case SG_HEARTBEAT:
            return "v2";
            break;
        case AEROBOX_RAW_DATA:
            return "v1";
            break;
        case AEROBOX_JSON_DATA:
            return "v1";
            break;
        case SG_SEQ_JSON_DATA:
            return "v2";
            break;
        case SG_REPORT_JSON_DATA:
            return "v2";
            break;
        case END_MESSAGE_TYPE:
            return "v0";
            break;
    }
    return "v0";
}

message_type_v2_t detect_msg_type(char *parse_str){
    for(int i=0; i<END_MESSAGE_TYPE;i++){
        if (!strcmp(get_message_type_str(i), parse_str)){
            return i;
        }
    }
    return END_MESSAGE_TYPE;
}

message_type_v2_t detect_msg_type_from_topic(char *parse_str){
    for(int i=0; i<END_MESSAGE_TYPE;i++){
        if (strstr(parse_str, get_message_type_str(i))){
            return i;
        }
    }
    return END_MESSAGE_TYPE;
}

parameter_type_t detect_param_type(char *parse_str){
    for(int i=0; i<END_PARAM_TYPE; i++){
        if (!strcmp(get_param_type_str(i), parse_str)){
            return i;
        }
    }
    return END_PARAM_TYPE;
}

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void __attribute__((noreturn)) task_fatal_error(void)
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}

// static void print_sha256 (const uint8_t *image_hash, const char *label)
// {
//     char hash_print[HASH_LEN * 2 + 1];
//     hash_print[HASH_LEN * 2] = 0;
//     for (int i = 0; i < HASH_LEN; ++i) {
//         sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
//     }
//     ESP_LOGI(TAG, "%s: %s", label, hash_print);
// }


// void ota_task(char *upgrade_url){
void ota_task(void *pvParameter){

    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    char *upgrade_url = malloc(200);
    strcpy(upgrade_url,(char*)pvParameter);

    ESP_LOGI(TAG, "Starting OTA example");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    esp_http_client_config_t config = {
        .url = upgrade_url,
        // .cert_pem = (char *)server_cert_pem_start,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        task_fatal_error();
    }
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    esp_http_client_fetch_headers(client);

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    int binary_file_length = 0;
    /*deal with all receive packet*/
    bool image_header_was_checked = false;
    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
            http_cleanup(client);
            task_fatal_error();
        } else if (data_read > 0) {
            if (image_header_was_checked == false) {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    // check current version with downloading
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            http_cleanup(client);
                            // infinite_loop();
                        }
                    }
// #ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
//                     if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
//                         ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
//                         http_cleanup(client);
//                         infinite_loop();
//                     }
// #endif
                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
                } else {
                    ESP_LOGE(TAG, "received package is not fit len");
                    http_cleanup(client);
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                }
            }
            err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                http_cleanup(client);
                esp_ota_abort(update_handle);
                task_fatal_error();
            }
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_length);
        } else if (data_read == 0) {
           /*
            * As esp_http_client_read never returns negative error code, we rely on
            * `errno` to check for underlying transport connectivity closure if any
            */
            if (errno == ECONNRESET || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
        }
    }
    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Error in receiving complete file");
        http_cleanup(client);
        esp_ota_abort(update_handle);
        task_fatal_error();
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        http_cleanup(client);
        task_fatal_error();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return ;
}

int parse_mqtt_param_update(parameter_type_t param, cJSON *parsed_data){
    switch (param){
        case NONE_PARAM:
            return 0;
            break;
        case HEX_GATEWAY_ID:
            // return "hex-gateway-id";
            set_hex_gateway_id(parsed_data->valuestring);
            return 1;
            break;
        case AUTH_GATEWAY_ID:
            // return "auth-gateway-id";
            set_auth_gateway_id(parsed_data->valuestring);
            return 1;
            break;
        case AUTH_GATEWAY_PWD:
            // return "hex-gateway-pwd";
            set_auth_gateway_pwd(parsed_data->valuestring);
            return 1;
            break;
        case ECU_GATEWAY_PAIR_ID:
            // return "ecu-gateway-pair-id";
            set_auth_gateway_pair_id(parsed_data->valuestring);
            return 1;
            break;
        case ECU_PAIRS:
            // return "ecu-pairs";
            return 0;
            break;
        case MQTT_AUTH_URI:
            // return "mqtt-auth-uri";
            set_auth_uri(parsed_data->valuestring);
            return 1;
            break;
        case MQTT_PUB_URI:
            // return "mqtt-pub-uri";
            set_pub_uri(parsed_data->valuestring);
            return 1;
            break;
        case AUTH_PSK:
            // return "auth-psk";
            set_psk_uri(parsed_data->valuestring);
            return 1;
            break;
        case AUTH_PSK_HINT:
            // return "auth-psk-hint";
            set_pskhint_uri(parsed_data->valuestring);
            return 1;
            break;
        case MQTT_SUB_URI:
            // return "mqtt-sub-uri";
            set_sub_uri(parsed_data->valuestring);
            return 1;
            break;
        case DEVICE_HOSTNAME:
            // return "device-hostname";
            set_hostname_uri(parsed_data->valuestring);
            return 1;
            break;
        case AEROBOX_GATEWAY_PAIR_ID:
            // return "aerobox-gateway-pair-id";
            // set_auth_uri(cjson_payload->valuestring);
            return 0;
            break;
        case AEROBOX_PAIRS:
            // return "aerobox-pairs";
            return 0;
            break;
        case AEROBOX_MQTT_URI:
            // return "aerobox-mqtt-uri";
            set_auth_uri(parsed_data->valuestring);
            return 1;
            break;
        case PARAM_UPDATE_TIME:
            // return "param-update-time";
            return 0;
            break;
        case OTA_UPGRADE:
            xTaskCreate(&ota_task, "ota_task", 8192, (void*)parsed_data->valuestring, 5, NULL);
            return 1;
            break;
        case CA_CERT:
            // set_certpem_uri(parsed_data->valuestring);
            return 0;
            break;
        case HEARTBEAT_RESOLUTION:
            // reture "heartbeat_resolution";
            set_heartbeat_resolution(&parsed_data->valueint);
            return 3;
            break;
        case LOCAL_MUTICAST:
            // return "local_muticast";
            set_local_muticast(parsed_data->valueint);
            return 2;
            break;
        case DATA_UPLOAD_RESOLUTION:
            // return "data_upload_resolution";
            set_msu_query_resolution(&parsed_data->valueint);
            return 3;
            break;
        case MQTT_QUERY_RESOLUTION:
            // return "mqtt_query_resolution";
            set_mqtt_query_resolution(&parsed_data->valueint);
            return 3;
            break;
        case MQTT_DATA_AGE:
            // return "mqtt_data_age";
            set_mqtt_data_age(&parsed_data->valueint);
            return 3;
            break;
        case MQTT_DISPLAY_MODE:
            // return "display_mode";
            set_mqtt_display_mode(parsed_data->valueint);
            return 2;
            break;
        case AEROBOX_GW_MODE:
            // return "aerobox_gw_mode";
            set_aerobox_gw_mode(parsed_data->valueint);
            return 2;
            break;
        case SQ_GW_MODE:
            // return "sq_gw_mode";
            set_sq_gw_mode(parsed_data->valueint);
            return 2;
            break;
        case BACKUP_MQTT:
            // return "backup_mqtt";
            set_backup_mqtt(parsed_data->valueint);
            return 2;
            break;
        case DISPLAY_ENG_MODE:
            // return "display_eng_mode";
            set_display_eng_mode(parsed_data->valueint);
            return 2;
            break;
        case GW_RFCH:
            // return "gw_rcfh";
            set_gw_rfch(&parsed_data->valueint);
            return 3;
            break;
        case DISPALY_PAIRS:
            // return "display_pairs";
            return 0;
            break;
        case END_PARAM_TYPE:
            // return "end-param-type";
            return 0;
            break;
        default:
            return 0;
            break;
    }
}

int parse_mqtt_param_get(parameter_type_t param, cJSON *parsed_data, cJSON *response){
    cJSON *ecu_pairs;
    switch (param){
        case NONE_PARAM:
            return 0;
            break;
        case HEX_GATEWAY_ID:
            // return "hex-gateway-id";
            // set_hex_gateway_id(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.hex_client_id);
            return 1;
            break;
        case AUTH_GATEWAY_ID:
            // return "auth-gateway-id";
            // set_auth_gateway_id(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.auth_client_id);
            return 1;
            break;
        case AUTH_GATEWAY_PWD:
            // return "hex-gateway-pwd";
            // set_auth_gateway_pwd(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.auth_client_pwd);
            return 1;
            break;
        case ECU_GATEWAY_PAIR_ID:
            // return "ecu-gateway-pair-id";
            // set_auth_gateway_pair_id(parsed_data->valuestring);
            // cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.hex_client_id);
            return 0;
            break;
        case ECU_PAIRS:
            // return "ecu-pairs";
            ecu_pairs = cJSON_CreateArray();
            for (int j=0; j<16; j++){
                if (strcmp((char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j],"")){
                    cJSON_AddItemToArray(ecu_pairs, cJSON_CreateString((char*)tornadoedge_sensor_unit_v2.sensor_unit_pairs[j]));
                }
            }
            cJSON_AddItemToObject(response, get_param_type_str(param), ecu_pairs);
            // free(ecu_pairs);
            return 4;
            break;
        case MQTT_AUTH_URI:
            // return "mqtt-auth-uri";
            // set_auth_uri(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.mqtt_auth_uri);
            return 1;
            break;
        case MQTT_PUB_URI:
            // return "mqtt-pub-uri";
            // set_pub_uri(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.mqtt_pub_uri);
            return 1;
            break;
        case AUTH_PSK:
            // return "auth-psk";
            // set_psk_uri(parsed_data->valuestring);
            // cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.mqtt_sub_uri);
            return 0;
            break;
        case AUTH_PSK_HINT:
            // return "auth-psk-hint";
            // set_pskhint_uri(parsed_data->valuestring);
            // cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.auth_client_pwd);
            return 0;
            break;
        case MQTT_SUB_URI:
            // return "mqtt-sub-uri";
            // set_sub_uri(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.mqtt_sub_uri);
            return 1;
            break;
        case DEVICE_HOSTNAME:
            // return "device-hostname";
            // set_hostname_uri(parsed_data->valuestring);
            cJSON_AddStringToObject(response,get_param_type_str(param), (char*)tornadoedge_auth_settings_v2.device_hostname);
            return 1;
            break;
        case AEROBOX_GATEWAY_PAIR_ID:
            // return "aerobox-gateway-pair-id";
            // set_auth_uri(cjson_payload->valuestring);
            // cJSON_AddStringToObject(response,get_param_type_str(param), (char*)aerobox_auth_settings.aerobox_gw);
            return 0;
            break;
        case AEROBOX_PAIRS:
            // return "aerobox-pairs";
            // ecu_pairs = cJSON_CreateArray();
            // for (int j=0; j<16; j++){
            //     if (strcmp((char*)aerobox_auth_settings.aerobox_pair[j],"")){
            //         cJSON_AddItemToArray(ecu_pairs, cJSON_CreateString((char*)aerobox_auth_settings.aerobox_pair[j]));
            //     }
            // }
            // cJSON_AddItemToObject(response, get_param_type_str(param), ecu_pairs);
            return 0;
            break;
        case AEROBOX_MQTT_URI:
            // return "aerobox-mqtt-uri";
            // set_auth_uri(parsed_data->valuestring);
            return 0;
            break;
        case PARAM_UPDATE_TIME:
            // return "param-update-time";
            return 0;
            break;
        case OTA_UPGRADE:
            // xTaskCreate(&ota_task, "ota_task", 8192, (void*)parsed_data->valuestring, 5, NULL);
            return 0;
            break;
        case CA_CERT:
            // set_certpem_uri(parsed_data->valuestring);
            return 0;
            break;
        case HEARTBEAT_RESOLUTION:
            // reture "heartbeat_resolution";
            // set_heartbeat_resolution(&parsed_data->valueint);
            cJSON_AddNumberToObject(response,get_param_type_str(param), tornadoedge_configs_v1.heartbeat_resolution);
            return 3;
            break;
        case LOCAL_MUTICAST:
            // return "local_muticast";
            // set_local_muticast(parsed_data->valueint);
            // cJSON_AddBoolToObject(response, get_param_type_str(param), true);
            cJSON_AddBoolToObject(response, get_param_type_str(param), tornadoedge_configs_v1.local_muticast_mode);
            return 2;
            break;
        case DATA_UPLOAD_RESOLUTION:
            // return "data_upload_resolution";
            // set_msu_query_resolution(&parsed_data->valueint);
            cJSON_AddNumberToObject(response,get_param_type_str(param), tornadoedge_configs_v1.data_upload_resolution);
            return 3;
            break;
        case MQTT_QUERY_RESOLUTION:
            // return "mqtt_query_resolution";
            // set_mqtt_query_resolution(&parsed_data->valueint);
            cJSON_AddNumberToObject(response,get_param_type_str(param), tornadoedge_configs_v1.mqtt_query_resolution);
            return 3;
            break;
        case MQTT_DATA_AGE:
            // return "mqtt_data_age";
            // set_mqtt_data_age(&parsed_data->valueint);
            cJSON_AddNumberToObject(response,get_param_type_str(param), tornadoedge_configs_v1.mqtt_query_data_age);
            return 3;
            break;
        case MQTT_DISPLAY_MODE:
            // return "display_mode";
            // set_display_mode(parsed_data->valueint);
            cJSON_AddBoolToObject(response, get_param_type_str(param), tornadoedge_configs_v1.mqtt_display_mode);
            return 2;
            break;
        case AEROBOX_GW_MODE:
            // return "aerobox_gw_mode";
            // set_aerobox_gw_mode(parsed_data->valueint);
            cJSON_AddBoolToObject(response, get_param_type_str(param), tornadoedge_configs_v1.aerobox_gw_mode);
            return 2;
            break;
        case SQ_GW_MODE:
            // return "sq_gw_mode";
            // set_sq_gw_mode(parsed_data->valueint);
            cJSON_AddBoolToObject(response, get_param_type_str(param), tornadoedge_configs_v1.lcd_1602_mode);
            return 2;
            break;
        case BACKUP_MQTT:
            // return "backup_mqtt";
            // set_backup_mqtt(parsed_data->valueint);
            cJSON_AddBoolToObject(response, get_param_type_str(param), tornadoedge_configs_v1.backup_mqtt);
            return 2;
            break;
        case DISPLAY_ENG_MODE:
            // return "display_eng_mode";
            // set_display_eng_mode(parsed_data->valueint);
            cJSON_AddBoolToObject(response, get_param_type_str(param), tornadoedge_configs_v1.display_eng_mode);
            return 2;
            break;
        case GW_RFCH:
            // return "gw_rcfh";
            // set_gw_rfch(&parsed_data->valueint);
            cJSON_AddNumberToObject(response,get_param_type_str(param), tornadoedge_configs_v1.gw_rfch);
            return 3;
            break;
        case DISPALY_PAIRS:
            // return "display_pairs";
            ecu_pairs = cJSON_CreateArray();
            for (int j=0; j<16; j++){
                if (strcmp((char*)tornadoedge_display_unit_v2.display_pairs[j],"")){
                    cJSON_AddItemToArray(ecu_pairs, cJSON_CreateString((char*)tornadoedge_display_unit_v2.display_pairs[j]));
                }
            }
            cJSON_AddItemToObject(response, get_param_type_str(param), ecu_pairs);
            // free(ecu_pairs);
            return 4;
            break;
        case END_PARAM_TYPE:
            // return "end-param-type";
            return 0;
            break;
        default:
            return 0;
            break;
    }
}

#ifdef __cplusplus
}
#endif