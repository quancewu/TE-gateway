#ifndef ETH_PLUGIN_H_INCLUDED
#define ETH_PLUGIN_H_INCLUDED

#include <stdbool.h>
#include <esp_err.h> /* for esp_err_t */
#include "freertos/queue.h"


#ifdef __cplusplus
extern "C" {
#endif

// static xQueueHandle flow_control_queue = NULL;
// static bool s_sta_is_connected = false;
// static bool s_ethernet_is_connected = false;
// static uint8_t s_eth_mac[6];
// static uint8_t s_broadcast_address[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

static esp_err_t pkt_wifi2eth(void *buffer, uint16_t len, void *eb);

esp_err_t pkt_eth2wifi(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t len, void *priv);

static void eth2wifi_flow_control_task(void *args);

esp_err_t initialize_flow_control(void);


#ifdef __cplusplus
}
#endif

#endif