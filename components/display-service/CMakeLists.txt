idf_component_register(SRCS "display-service.c"
                    REQUIRES "json" "esp32-wifi-manager-master" "utils" "TE_mqtt" "lwip" "esp32-smbus" "esp32-i2c-lcd1602"
                    INCLUDE_DIRS "include")
