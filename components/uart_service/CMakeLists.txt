set(COMPONENT_SRCS 	"uart_service.c")
set(COMPONENT_ADD_INCLUDEDIRS "./include")
set(COMPONENT_REQUIRES log driver freertos json esp32-wifi-manager-master TE_mqtt utils)
register_component()