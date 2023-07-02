idf_component_register(SRC_DIRS src
        REQUIRES esp-tls log nvs_flash freertos json mqtt esp_http_client esp32-wifi-manager-master utils uart_service mbedtls display-service
        INCLUDE_DIRS src
        EMBED_FILES src/quance_mqtt.pem src/server_mqtt.pem)
