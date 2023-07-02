#ifndef TE_WS_SV_H
#define TE_WS_SV_H

#ifdef __cplusplus
extern "C" {
#endif

void server_task(void* pvParameters);

void server_handle_task(void* pvParameters);

void count_task(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif