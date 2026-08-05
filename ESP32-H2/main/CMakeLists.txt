idf_component_register(
    SRCS
        "main.c"
        "uart_link.c"
        "led.c"
        "ota.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        nvs_flash
        app_update
        espressif__esp-zigbee-lib
        espressif__esp-zboss-lib
)
