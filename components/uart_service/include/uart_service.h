#ifndef uart_service_H_INCLUDED
#define uart_service_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif


void initialize_uart();

void time_sync_lrgw();

void blink_led_G(int nums);

void blink_led_R(int nums);

void blink_led_B(int nums);

void set_led(int nums, int index, bool methods, char* colors);

void send_aerobox_heartbeat_task(void *pvParameters);

void uart_tx_task(void *pvParameters);

void uart_rx_task(void *arg);

char* uart_async_get_data_json();

void set_sync_from_server(bool status);

#ifdef __cplusplus
}
#endif

#endif
