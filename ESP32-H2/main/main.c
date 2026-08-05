/*
 * main.c — ESP32-H2: bridge Zigbee <-> ATmega328 (P1P2 Daikin VAM)
 *
 * Espone 6 endpoint Zigbee, tutti cluster On/Off semplici:
 *   EP1 On/Off → accensione VMC
 *   EP2 On/Off → velocità (OFF=Lento, ON=Veloce)
 *   EP3 On/Off → fresh-up
 *   EP4 On/Off → modalità Auto
 *   EP5 On/Off → modalità Scambio
 *   EP6 On/Off → modalità Bypass
 * (i tre endpoint di modalità sono mutuamente esclusivi: solo uno risulta
 * ON alla volta, riflettendo lo stato reale letto dall'ATmega)
 *
 * Gli attributi Zigbee vengono scritti SOLO quando arriva una riga di stato
 * "S," dall'ATmega (vedi uart_link.h) — non ottimisticamente al comando.
 * Unica eccezione, fuori dal nostro controllo: esp-zigbee-lib scrive da
 * sola l'attributo nella tabella locale non appena riceve un comando da
 * Zigbee, prima ancora che il nostro handler giri (stesso comportamento già
 * visto in smart_switch/relay.c) — il valore reale lo sovrascrive appena
 * arriva la conferma dal bus, di norma entro un ciclo di poll.
 *
 * Infrastruttura (segnali BDB, keepalive verso il coordinator, watchdog
 * applicativo, factory reset da GPIO9, LED di stato su GPIO22) copiata da
 * smart_switch/main.c, che è già in produzione: stessa regola, tutte le
 * chiamate verso lo stack Zigbee da task diversi passano da
 * esp_zb_scheduler_alarm, mai esp_zb_lock_acquire.
 */

#include "config.h"
#include "led.h"
#include "uart_link.h"
#include "ota.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "esp_partition.h"
#include "esp_sleep.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "MAIN";

/* ── Configurazione Zigbee ───────────────────────────────────────────── */
#define INSTALLCODE_POLICY_ENABLE    false
#define MAX_CHILDREN                 10
/* Stesso canale fisso di smart_switch: deve restare la stessa rete Zigbee
 * già in uso in casa. Se la rete reale è su un altro canale, va corretto. */
#define ESP_ZB_PRIMARY_CHANNEL_MASK  (1UL << 25)

#define ESP_MANUFACTURER_NAME  "\x09""Handmade!"
#define ESP_MODEL_IDENTIFIER   "\x0D""VmcP1P2Bridge"
#define OTA_UPGRADE_QUERY_INTERVAL   240   /* minuti */

#define ESP_ZB_ZR_CONFIG() {                                        \
    .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,               \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,               \
    .nwk_cfg.zczr_cfg    = { .max_children = MAX_CHILDREN },        \
}
#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG()  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

static bool zigbee_ready             = false;
static bool s_coordinator_confirmed  = false;   /* true solo dopo ping ZDO riuscito */
static uint8_t s_ping_tick           = 0;
static uint8_t s_ping_fail_count     = 0;       /* fallimenti ZDO consecutivi       */
static bool    s_ping_in_flight      = false;   /* ping inviato, attesa callback    */
static uint8_t s_steering_fail_count = 0;       /* fallimenti steering consecutivi  */

/* ── Stato VMC noto (ultimo aggiornamento dall'ATmega) ───────────────── */
static vmc_state_t s_last_state  = {0};
static bool         s_have_state = false;
static volatile vmc_state_t s_pending_state;

/* ── Keepalive: costanti (vedi smart_switch/main.c per il razionale) ──── */
#define COORDINATOR_PING_TICK_MS       60000
#define COORDINATOR_PING_TICKS         5
#define COORDINATOR_PING_MAX_FAILS     6
#define COORDINATOR_PING_TIMEOUT_MS    40000
#define STEERING_MAX_FAILS             10
static void coordinator_ping_cb(uint8_t param);
static void coordinator_ping_timeout_cb(uint8_t param);

/* ── Zigbee hard/phy reset (vedi smart_switch/main.c) ────────────────── */
static void zigbee_hard_reset(const char *reason)
{
    ESP_LOGW(TAG, "*** ZIGBEE HARD RESET (%s) — cancello partizioni Zigbee ***", reason);
    led_set_state(LED_ERROR);
    const char *zb_parts[] = { "zb_storage", "zb_fct", "phy_init" };
    for (int i = 0; i < 3; i++) {
        const esp_partition_t *p = esp_partition_find_first(
            ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, zb_parts[i]);
        if (p) {
            esp_err_t err = esp_partition_erase_range(p, 0, p->size);
            ESP_LOGW(TAG, "  '%s' (0x%"PRIx32", %"PRIu32"B): %s",
                     zb_parts[i], p->address, p->size,
                     err == ESP_OK ? "OK" : esp_err_to_name(err));
        } else {
            ESP_LOGE(TAG, "  '%s': partizione non trovata!", zb_parts[i]);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void zigbee_phy_reset(const char *reason)
{
    ESP_LOGW(TAG, "*** ZIGBEE PHY RESET (%s) — cancello phy_init ***", reason);
    led_set_state(LED_ERROR);
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "phy_init");
    if (p) {
        esp_err_t err = esp_partition_erase_range(p, 0, p->size);
        ESP_LOGW(TAG, "  'phy_init' (0x%"PRIx32", %"PRIu32"B): %s",
                 p->address, p->size, err == ESP_OK ? "OK" : esp_err_to_name(err));
    } else {
        ESP_LOGE(TAG, "  'phy_init': partizione non trovata!");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

/* ── Stato VMC → Zigbee ──────────────────────────────────────────────── */
/*
 * Scrivere 6 attributi ZCL di fila nello stesso callback ha fatto crashare lo
 * stack Zigbee (buffer pool esaurito, assert in zb_bufpool_mult_storage.c:105
 * — osservato sul campo). Fix: scriviamo un attributo alla volta con
 * esp_zb_scheduler_alarm, scaglionati di STATE_SYNC_STAGGER_MS l'uno
 * dall'altro, e solo quelli il cui valore è davvero cambiato rispetto
 * all'ultimo sync riuscito (force_all=true ignora la cache e riscrive
 * comunque tutto, usato nel resync periodico verso il coordinator).
 *
 * check=false in ogni scrittura: come do_relay_zigbee_sync in
 * smart_switch/main.c. L'attributo OnOff è read-only via Write Attribute per
 * spec ZCL — con check=true la scrittura locale falliva silenziosamente
 * (nessun log, nessun cambiamento), motivo per cui lo stato reale non
 * arrivava mai a Home Assistant.
 */
#define STATE_SYNC_STAGGER_MS 40

static uint8_t s_zb_last_onoff = 0xFF, s_zb_last_speed = 0xFF, s_zb_last_freshup = 0xFF;
static uint8_t s_zb_last_auto = 0xFF, s_zb_last_scambio = 0xFF, s_zb_last_bypass = 0xFF;

/* param: bit7 = valore, bit0-3 = endpoint (stesso schema di smart_switch/relay) */
static void do_zb_write_one(uint8_t param)
{
    if (!zigbee_ready) return;
    uint8_t ep  = param & 0x0F;
    uint8_t val = (param & 0x80) ? 1 : 0;
    esp_zb_zcl_set_attribute_val(ep,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &val, false);
}

static void queue_state_sync(const vmc_state_t *st, bool force_all)
{
    uint32_t delay = 0;
    uint8_t onoff   = st->onoff ? 1 : 0;
    uint8_t speed   = (st->speed >= 0x05) ? 1 : 0;   /* OFF=Lento, ON=Veloce */
    uint8_t freshup = st->freshup ? 1 : 0;
    uint8_t auto_   = (st->mode == 0) ? 1 : 0;
    uint8_t scambio = (st->mode == 1) ? 1 : 0;
    uint8_t bypass  = (st->mode == 2) ? 1 : 0;

    if (force_all || onoff != s_zb_last_onoff) {
        esp_zb_scheduler_alarm(do_zb_write_one, (uint8_t)(EP_ONOFF | (onoff ? 0x80 : 0)), delay);
        delay += STATE_SYNC_STAGGER_MS;
        s_zb_last_onoff = onoff;
    }
    if (force_all || speed != s_zb_last_speed) {
        esp_zb_scheduler_alarm(do_zb_write_one, (uint8_t)(EP_SPEED | (speed ? 0x80 : 0)), delay);
        delay += STATE_SYNC_STAGGER_MS;
        s_zb_last_speed = speed;
    }
    if (force_all || freshup != s_zb_last_freshup) {
        esp_zb_scheduler_alarm(do_zb_write_one, (uint8_t)(EP_FRESHUP | (freshup ? 0x80 : 0)), delay);
        delay += STATE_SYNC_STAGGER_MS;
        s_zb_last_freshup = freshup;
    }
    if (force_all || auto_ != s_zb_last_auto) {
        esp_zb_scheduler_alarm(do_zb_write_one, (uint8_t)(EP_MODE_AUTO | (auto_ ? 0x80 : 0)), delay);
        delay += STATE_SYNC_STAGGER_MS;
        s_zb_last_auto = auto_;
    }
    if (force_all || scambio != s_zb_last_scambio) {
        esp_zb_scheduler_alarm(do_zb_write_one, (uint8_t)(EP_MODE_SCAMBIO | (scambio ? 0x80 : 0)), delay);
        delay += STATE_SYNC_STAGGER_MS;
        s_zb_last_scambio = scambio;
    }
    if (force_all || bypass != s_zb_last_bypass) {
        esp_zb_scheduler_alarm(do_zb_write_one, (uint8_t)(EP_MODE_BYPASS | (bypass ? 0x80 : 0)), delay);
        delay += STATE_SYNC_STAGGER_MS;
        s_zb_last_bypass = bypass;
    }
}

static void do_state_zigbee_sync(uint8_t param)
{
    (void)param;
    if (!zigbee_ready) return;
    vmc_state_t st = s_pending_state;
    queue_state_sync(&st, false);
}

/* Callback da uart_link.c: gira nel task UART, non nel task Zigbee.
 * Salva solo dati e schedula la sincronizzazione vera tramite scheduler_alarm. */
static void on_vmc_state(const vmc_state_t *st)
{
    s_last_state  = *st;
    s_have_state  = true;
    if (!zigbee_ready) return;
    s_pending_state = *st;
    esp_zb_scheduler_alarm(do_state_zigbee_sync, 0, 0);
}

static void on_vmc_error(uint8_t code)
{
    /* Solo log: non tocchiamo lo stack Zigbee da qui (task UART).
     * Il codice 3 (silenzio dal master) è il più indicativo di un problema
     * reale sul bus P1P2, non solo rumore passeggero. */
    if (code == 3) {
        ESP_LOGW(TAG, "ATmega: silenzio dal master P1P2 (bus/master assente?)");
    }
}

/* ── OTA ─────────────────────────────────────────────────────────────── */
static void ota_find_server(void)
{
    esp_zb_ota_upgrade_client_query_interval_set(EP_ONOFF, OTA_UPGRADE_QUERY_INTERVAL);
    esp_zb_ota_upgrade_client_query_image_req(0x0000, 1);
    ESP_LOGI(TAG, "OTA: query → coordinator, polling ogni %d min", OTA_UPGRADE_QUERY_INTERVAL);
}

/* ── Costruzione cluster Basic ───────────────────────────────────────── */
static esp_zb_attribute_list_t *create_basic_cluster(void)
{
    esp_zb_basic_cluster_cfg_t cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_attribute_list_t *c = esp_zb_basic_cluster_create(&cfg);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  ESP_MODEL_IDENTIFIER);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,          OTA_SW_BUILD_ID);
    return c;
}

/* ── Endpoint On/Off generico (accensione, velocità, fresh-up, modalità) ─
 * EP_ONOFF porta anche il cluster OTA client (un solo client OTA per
 * dispositivo, come in smart_switch). */
static void add_switch_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep, const char *label)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t id_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl,
        esp_zb_identify_cluster_create(&id_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_cfg_t oo = { .on_off = 0 };
    esp_zb_cluster_list_add_on_off_cluster(cl,
        esp_zb_on_off_cluster_create(&oo), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    if (ep == EP_ONOFF)
        esp_zb_cluster_list_add_ota_cluster(cl,
            ota_cluster_create(), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint           = ep,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
    ESP_LOGI(TAG, "EP%d: On/Off (%s)", ep, label);
}

static void create_endpoints(esp_zb_ep_list_t *ep_list)
{
    add_switch_endpoint(ep_list, EP_ONOFF,        "Accensione");
    add_switch_endpoint(ep_list, EP_SPEED,        "Velocità (OFF=Lento, ON=Veloce)");
    add_switch_endpoint(ep_list, EP_FRESHUP,      "Fresh-up");
    add_switch_endpoint(ep_list, EP_MODE_AUTO,    "Modo Auto");
    add_switch_endpoint(ep_list, EP_MODE_SCAMBIO, "Modo Scambio");
    add_switch_endpoint(ep_list, EP_MODE_BYPASS,  "Modo Bypass");
}

/* ── Handler comandi Zigbee → comandi ATmega ─────────────────────────── */

static esp_err_t handle_on_off(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) return ESP_OK;
    if (msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_BOOL) return ESP_OK;

    uint8_t ep    = msg->info.dst_endpoint;
    bool    state = *(bool *)msg->attribute.data.value;
    ESP_LOGI(TAG, "Zigbee On/Off EP%d -> %s", ep, state ? "ON" : "OFF");

    switch (ep) {
    case EP_ONOFF:
        uart_link_send_cmd(state ? "ON" : "OFF");
        break;
    case EP_SPEED:
        uart_link_send_cmd(state ? "VH" : "VL");
        break;
    case EP_FRESHUP:
        uart_link_send_cmd(state ? "FU1" : "FU0");
        break;
    case EP_MODE_AUTO:
        if (state) uart_link_send_cmd("MA");
        break;
    case EP_MODE_SCAMBIO:
        if (state) uart_link_send_cmd("MS");
        break;
    case EP_MODE_BYPASS:
        if (state) uart_link_send_cmd("MB");
        break;
    default:
        break;
    }
    /* Se si spegne una modalità attiva senza accenderne un'altra non c'è un
     * comando ATmega corrispondente: l'endpoint tornerà ON da solo alla
     * prossima riga di stato reale (nessun aggiornamento ottimistico qui). */
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    ESP_RETURN_ON_FALSE(msg, ESP_FAIL, TAG, "msg vuoto");
    ESP_RETURN_ON_FALSE(msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
        ESP_ERR_INVALID_ARG, TAG, "status errore");

    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        return handle_on_off(msg);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *msg)
{
    switch (cb_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)msg);
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID: {
        const esp_zb_zcl_ota_upgrade_value_message_t *ota =
            (esp_zb_zcl_ota_upgrade_value_message_t *)msg;
        if (ota->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START)
            led_set_state(LED_OTA_IN_PROGRESS);
        return ota_upgrade_handler(ota);
    }
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID: {
        const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *r = msg;
        if (r->query_status == ESP_ZB_ZCL_STATUS_SUCCESS)
            ESP_LOGI(TAG, "OTA: nuova immagine v0x%08"PRIx32" (%"PRIu32" byte)",
                     r->file_version, r->image_size);
        else
            ESP_LOGI(TAG, "OTA: nessuna nuova immagine (0x%02x)", r->query_status);
        return ESP_OK;
    }
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        return ESP_OK;
    default:
        ESP_LOGW(TAG, "Azione non gestita: 0x%x", cb_id);
        return ESP_OK;
    }
}

/* ── Segnali Zigbee (identico a smart_switch/main.c) ─────────────────── */
static void bdb_start_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
        , TAG, "Errore commissioning BDB");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p   = sig->p_app_signal;
    esp_err_t err = sig->esp_err_status;
    esp_zb_app_signal_type_t type = *p;

    switch (type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack inizializzato");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Avvio steering...");
            led_set_state(LED_ZIGBEE_SEARCHING);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Init fallita, riprovo...");
            led_set_state(LED_ERROR);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        zigbee_ready            = false;
        s_coordinator_confirmed = false;
        ESP_LOGW(TAG, "Segnale LEAVE ricevuto — riprovo steering tra 30s...");
        led_set_state(LED_ZIGBEE_SEARCHING);
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            if (esp_zb_get_short_address() == 0xFFFF) {
                zigbee_hard_reset("addr=0xFFFF");
            }
            s_steering_fail_count = 0;
            zigbee_ready = true;
            ESP_LOGI(TAG, "Rete trovata: canale %d, PAN 0x%04hx, addr 0x%04hx — verifico coordinator...",
                     esp_zb_get_current_channel(),
                     esp_zb_get_pan_id(),
                     esp_zb_get_short_address());
            s_ping_tick = COORDINATOR_PING_TICKS - 1;
            esp_zb_scheduler_alarm(coordinator_ping_cb, 0, 1000);
        } else {
            s_steering_fail_count++;
            ESP_LOGW(TAG, "Steering fallito (%d/%d), riprovo tra 30s...",
                     s_steering_fail_count, STEERING_MAX_FAILS);
            if (s_steering_fail_count >= STEERING_MAX_FAILS) {
                zigbee_phy_reset("steering loop");
            }
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        if (zigbee_ready) {
            zigbee_ready            = false;
            s_coordinator_confirmed = false;
            ESP_LOGW(TAG, "Nessun link attivo (0x18) — forzo riconnessione");
            led_set_state(LED_ZIGBEE_SEARCHING);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGD(TAG, "NLME Status Indication (0x32) — ignorato");
        break;

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        ESP_LOGD(TAG, "ZDO Device Unavailable (0x3c) — ignorato");
        break;

    default:
        ESP_LOGI(TAG, "Segnale: %s (0x%x) %s",
                 esp_zb_zdo_signal_to_string(type), type, esp_err_to_name(err));
        break;
    }
}

/* ── Keepalive ZDO verso il coordinator (identico a smart_switch) ────── */
static void coordinator_ping_result(esp_zb_zdp_status_t zdo_status,
                                    esp_zb_zdo_ieee_addr_rsp_t *resp,
                                    void *user_ctx)
{
    s_ping_in_flight = false;
    if (!zigbee_ready) return;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        s_ping_tick       = 0;
        s_ping_fail_count = 0;
        ESP_LOGD(TAG, "Keepalive coordinator: OK");

        if (!s_coordinator_confirmed) {
            s_coordinator_confirmed = true;
            ESP_LOGI(TAG, "Coordinator raggiungibile — operativo");
            led_set_state(LED_OPERATIONAL);
            ota_find_server();
        }

        /* force_all=true: riscrive tutti gli attributi (scaglionati) per
         * aggiornare ZHA e stimolare route discovery, ad ogni ping riuscito,
         * non solo al primo join. */
        if (s_have_state) queue_state_sync(&s_last_state, true);

        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
    } else {
        s_ping_fail_count++;
        ESP_LOGW(TAG, "Coordinator irraggiungibile (0x%02x) — fail %d/%d",
                 zdo_status, s_ping_fail_count, COORDINATOR_PING_MAX_FAILS);
        zigbee_ready            = false;
        s_coordinator_confirmed = false;
        s_ping_tick             = 0;
        led_set_state(LED_ZIGBEE_SEARCHING);

        if (s_ping_fail_count >= COORDINATOR_PING_MAX_FAILS) {
            zigbee_hard_reset("ping fail");
        }

        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
    }
}

static void coordinator_ping_timeout_cb(uint8_t param)
{
    (void)param;
    if (!s_ping_in_flight) return;
    ESP_LOGW(TAG, "Keepalive: nessuna risposta ZDO in %ds — forzo fallimento",
             COORDINATOR_PING_TIMEOUT_MS / 1000);
    coordinator_ping_result(ESP_ZB_ZDP_STATUS_TIMEOUT, NULL, NULL);
}

static void coordinator_ping_cb(uint8_t param)
{
    (void)param;
    if (!zigbee_ready) return;

    s_ping_tick++;
    if (s_ping_tick < COORDINATOR_PING_TICKS) {
        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
        return;
    }
    s_ping_tick = 0;

    esp_zb_zdo_ieee_addr_req_param_t req = {
        .dst_nwk_addr     = 0x0000,
        .addr_of_interest = 0x0000,
        .request_type     = 0,
        .start_index      = 0,
    };
    ESP_LOGD(TAG, "Keepalive: ZDO ieee_addr_req → coordinator");
    s_ping_in_flight = true;
    esp_zb_zdo_ieee_addr_req(&req, coordinator_ping_result, NULL);
    esp_zb_scheduler_alarm(coordinator_ping_timeout_cb, 0, COORDINATOR_PING_TIMEOUT_MS);
}

/* ── Watchdog applicativo Zigbee (identico a smart_switch) ───────────── */
#define ZB_WDT_FEED_INTERVAL_MS   60000
#define ZB_WDT_TIMEOUT_MS         180000
#define ZB_WDT_CHECK_INTERVAL_MS  30000

static volatile TickType_t s_zb_last_feed = 0;

static void zb_wdt_feed_cb(uint8_t param)
{
    (void)param;
    s_zb_last_feed = xTaskGetTickCount();
    esp_zb_scheduler_alarm(zb_wdt_feed_cb, 0, ZB_WDT_FEED_INTERVAL_MS);
}

static void zb_watchdog_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(ZB_WDT_FEED_INTERVAL_MS + 30000));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ZB_WDT_CHECK_INTERVAL_MS));
        TickType_t elapsed = xTaskGetTickCount() - s_zb_last_feed;
        if (elapsed > pdMS_TO_TICKS(ZB_WDT_TIMEOUT_MS)) {
            ESP_LOGE("ZB_WDT", "Stack Zigbee non risponde da %lu s — deep sleep 3s",
                     (unsigned long)(elapsed * portTICK_PERIOD_MS / 1000));
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
            esp_deep_sleep_start();
        }
    }
}

/* ── Factory reset Zigbee (pulsante GPIO9, identico a smart_switch) ──── */
#define FACTORY_RESET_HOLD_S     5

static void factory_reset_task(void *pv)
{
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << FACTORY_RESET_GPIO_NUM),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    vTaskDelay(pdMS_TO_TICKS(10000));

    for (;;) {
        if (gpio_get_level(FACTORY_RESET_GPIO_NUM) != 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        ESP_LOGW(TAG, "GPIO9 premuto — tieni premuto %ds per factory reset Zigbee...",
                 FACTORY_RESET_HOLD_S);
        led_set_state(LED_FACTORY_RESET);

        bool held = true;
        for (int i = 0; i < FACTORY_RESET_HOLD_S * 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (gpio_get_level(FACTORY_RESET_GPIO_NUM) != 0) {
                held = false;
                break;
            }
        }

        if (!held) {
            ESP_LOGI(TAG, "GPIO9 rilasciato — annullato");
            led_set_state(zigbee_ready ? LED_OPERATIONAL : LED_ZIGBEE_SEARCHING);
            continue;
        }

        ESP_LOGW(TAG, "*** FACTORY RESET ZIGBEE — cancello partizioni... ***");
        const char *zb_parts[] = { "zb_storage", "zb_fct" };
        for (int i = 0; i < 2; i++) {
            const esp_partition_t *p = esp_partition_find_first(
                ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, zb_parts[i]);
            if (p) {
                esp_err_t err = esp_partition_erase_range(p, 0, p->size);
                ESP_LOGW(TAG, "Partizione '%s' (0x%"PRIx32", %"PRIu32" byte): %s",
                         zb_parts[i], p->address, p->size,
                         err == ESP_OK ? "CANCELLATA" : esp_err_to_name(err));
            } else {
                ESP_LOGE(TAG, "Partizione '%s' NON TROVATA!", zb_parts[i]);
            }
        }
        ESP_LOGW(TAG, "Riavvio...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

/* ── Task Zigbee ─────────────────────────────────────────────────────── */
static void esp_zb_task(void *pv)
{
    esp_zb_cfg_t cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    create_endpoints(ep_list);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    s_zb_last_feed = xTaskGetTickCount();
    esp_zb_scheduler_alarm(zb_wdt_feed_cb, 0, ZB_WDT_FEED_INTERVAL_MS);
    esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);

    esp_zb_stack_main_loop();
}

/* ── Entry point ─────────────────────────────────────────────────────── */
void app_main(void)
{
    {
        esp_reset_reason_t rr = esp_reset_reason();
        if (rr != ESP_RST_POWERON &&
            rr != ESP_RST_DEEPSLEEP &&
            rr != ESP_RST_EXT) {
            ESP_LOGW(TAG, "Reset non-pulito (reason=%d) — deep sleep 3s per reset radio", (int)rr);
            esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
            esp_deep_sleep_start();
        }
    }

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Anti-rollback OTA */
    ota_mark_valid();

    led_init();

    xTaskCreate(factory_reset_task, "factory_rst", 2048, NULL, 1, NULL);
    xTaskCreate(zb_watchdog_task,   "zb_wdt",       2048, NULL, 2, NULL);

    /* Link seriale verso l'ATmega: gli aggiornamenti di stato arrivano da
     * qui via on_vmc_state/on_vmc_error. */
    uart_link_init(on_vmc_state, on_vmc_error);

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
