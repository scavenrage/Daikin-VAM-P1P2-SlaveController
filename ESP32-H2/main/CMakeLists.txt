idf_component_register(
    SRCS
        "main.c"
        "uart_link.c"
        "led.c"
        "ota.c"
        "ble_update.c"
        "ble_log.c"
        "maint.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        bt
        esp_timer
        driver
        nvs_flash
        app_update
        espressif__esp-zigbee-lib
)

# Gli header di compatibilità 1.x di esp-zigbee-lib 2.x (include/compat/*.h) emettono
# '#warning "This file is deprecated..."' a ogni inclusione. -Wno-cpp nasconde solo i #warning
# e solo per i file di questo componente; gli altri warning (con -Werror) restano attivi.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-cpp)
