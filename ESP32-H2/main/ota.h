/*
 * ota.h — Zigbee OTA Upgrade client per ESP32-H2
 * Copiato da smart_switch/ota.h, image_type diverso per non collidere.
 */

#pragma once

#include "esp_err.h"
#include "esp_zigbee_core.h"

#define OTA_MANUFACTURER_CODE   0x1001
#define OTA_IMAGE_TYPE          0x0105   /* 0x0104 = smart_switch, 0x0105 = this project */
#define OTA_HW_VERSION          0x0001
#define OTA_MAX_DATA_SIZE       223

#define OTA_FILE_VERSION        0x02000000   /* v2.0.0: esp-zigbee-lib 2.x, senza ZBOSS */
#define OTA_SW_BUILD_ID         "\x06""v2.0.0"

/* OTA via Zigbee: disattivato con esp-zigbee-lib 2.x. Nel livello di compatibilità
 * 1.x della 2.0.5 il cluster OTA non è supportato (esp_zb_ota_cluster_create -> NULL,
 * esp_zb_cluster_list_add_ota_cluster -> ESP_ERR_NOT_SUPPORTED). Aggiornamento via cavo.
 * OTA_FILE_VERSION / OTA_SW_BUILD_ID restano: identificano la versione (Basic SW build ID). */
#define ZB_OTA_ENABLED          0

#if ZB_OTA_ENABLED
esp_zb_attribute_list_t *ota_cluster_create(void);
esp_err_t ota_upgrade_handler(const esp_zb_zcl_ota_upgrade_value_message_t *message);
#endif
void ota_mark_valid(void);
