// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_utils.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

// #ifdef CONFIG_MEM_DEBUG
#define MEM_DEBUG true
// #else
// #define MEM_DEBUG false
// #endif /**< CONFIG_MEM_DEBUG */

#ifndef CONFIG_MEM_DBG_INFO_MAX
#define CONFIG_MEM_DBG_INFO_MAX     (256)
#endif  /**< CONFIG_MEM_DBG_INFO_MAX */
#define MEM_DBG_INFO_MAX CONFIG_MEM_DBG_INFO_MAX

#ifdef CONFIG_MEM_ALLOCATION_DEFAULT
inline void *esp_heap_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
}
inline void *esp_heap_calloc(size_t n, size_t size)
{
    return heap_caps_calloc(n, size, MALLOC_CAP_DEFAULT);
}
inline void *esp_heap_realloc(void *ptr, size_t size)
{
    return heap_caps_realloc(ptr, size, MALLOC_CAP_DEFAULT);
}
#endif

#ifdef CONFIG_MEM_ALLOCATION_SPIRAM
inline void *esp_heap_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
}
inline void *esp_heap_calloc(size_t n, size_t size)
{
    return heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM);
}
inline void *esp_heap_realloc(void *ptr, size_t size)
{
    return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM);
}
#endif

/**
 * @brief Add to memory record
 *
 * @param ptr  Memory pointer
 * @param size Memory size
 * @param tag  Description tag
 * @param line Line number
 */
void esp_mem_add_record(void *ptr, int size, const char *tag, int line);

/**
 * @brief Remove from memory record
 *
 * @param ptr  Memory pointer
 * @param tag  Description tag
 * @param line Line number
 */
void esp_mem_remove_record(void *ptr, const char *tag, int line);

/**
 * @brief Print the all allocation but not released memory
 *
 * @attention Must configure CONFIG_MEM_DEBUG == y annd esp_log_level_set(esp_mem, ESP_LOG_INFO);
 */
void esp_mem_print_record(void);

/**
 * @brief Print memory and free space on the stack
 */
void esp_mem_print_heap(void);

/**
 * @brief Print the state of tasks in the system
 */
void esp_mem_print_task(void);

/**
 * @brief  Malloc memory
 *
 * @param  size  Memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_MALLOC(size) ({ \
        void *ptr = esp_heap_malloc(size); \
        if (MEM_DEBUG) { \
            if(!ptr) { \
                ESP_LOGW(TAG, "<ESP_ERR_NO_MEM> Malloc size: %d, ptr: %p, heap free: %d", (int)size, ptr, esp_get_free_heap_size()); \
            } else { \
                esp_mem_add_record(ptr, size, TAG, __LINE__); \
            } \
        } \
        ptr; \
    })

/**
 * @brief  Calloc memory
 *
 * @param  n     Number of block
 * @param  size  Block memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_CALLOC(n, size) ({ \
        void *ptr = esp_heap_calloc(n, size); \
        if (MEM_DEBUG) { \
            if(!ptr) { \
                ESP_LOGW(TAG, "<ESP_ERR_NO_MEM> Calloc size: %d, ptr: %p, heap free: %d", (int)(n) * (size), ptr, esp_get_free_heap_size()); \
            } else { \
                esp_mem_add_record(ptr, (n) * (size), TAG, __LINE__); \
            } \
        } \
        ptr; \
    })

/**
 * @brief  Reallocate memory
 *
 * @param  ptr   Memory pointer
 * @param  size  Block memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_REALLOC(ptr, size) ({ \
        void *new_ptr = esp_heap_realloc(ptr, size); \
        if (MEM_DEBUG) { \
            if(!new_ptr) { \
                ESP_LOGW(TAG, "<ESP_ERR_NO_MEM> Realloc size: %d, new_ptr: %p, heap free: %d", (int)size, new_ptr, esp_get_free_heap_size()); \
            } else { \
                esp_mem_remove_record(ptr, TAG, __LINE__); \
                esp_mem_add_record(new_ptr, size, TAG, __LINE__); \
            } \
        } \
        new_ptr; \
    })


/**
 * @brief  Reallocate memory, If it fails, it will retry until it succeeds
 *
 * @param  ptr   Memory pointer
 * @param  size  Block memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_REALLOC_RETRY(ptr, size) ({ \
        void *new_ptr = NULL; \
        while (size > 0 && !(new_ptr = esp_heap_realloc(ptr, size))) { \
            ESP_LOGW(TAG, "<ESP_ERR_NO_MEM> Realloc size: %d, new_ptr: %p, heap free: %d", (int)size, new_ptr, esp_get_free_heap_size()); \
            vTaskDelay(pdMS_TO_TICKS(100)); \
        } \
        if (MEM_DEBUG) { \
            esp_mem_remove_record(ptr, TAG, __LINE__); \
            esp_mem_add_record(new_ptr, size, TAG, __LINE__); \
        } \
        new_ptr; \
    })

/**
 * @brief  Free memory
 *
 * @param  ptr  Memory pointer
 */
#define ESP_FREE(ptr) { \
        if(ptr) { \
            free(ptr); \
            if (MEM_DEBUG) { \
                esp_mem_remove_record(ptr, TAG, __LINE__); \
            } \
            ptr = NULL; \
        } \
    } while(0)

#ifdef __cplusplus
}
#endif /**< _cplusplus */
