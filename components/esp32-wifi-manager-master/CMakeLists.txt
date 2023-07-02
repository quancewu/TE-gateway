idf_component_register(SRC_DIRS src
    REQUIRES log nvs_flash app_update mdns wpa_supplicant lwip esp_http_server utils uart_service
    INCLUDE_DIRS src
    EMBED_FILES src/style.css src/code.js src/index.html src/aerobox.js src/dataindex.html src/otaindex.html src/favicon.ico src/routing.js src/control.html src/control.js)
