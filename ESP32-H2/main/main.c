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
 * più 4 interruttori virtuali di manutenzione, come smart switch ZB2:
 *   EP10 aggiornamento BLE, EP11 programmazione, EP12 factory reset (RESCRED),
 *   EP13 log via BLE
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
 * smart_switch/main.c, che è già in produzione.
 *
 * v2.0.0: esp-zigbee-lib 2.x (senza ZBOSS) tramite il livello di compatibilità
 * 1.x (CONFIG_ZB_SDK_1xx), come ESP32-H2-Custom-Zigbee 5.5-ZB2. In 2.x il lock
 * è obbligatorio per ogni API Zigbee chiamata fuori dai callback Zigbee: dal
 * task UART si passa da zb_alarm_any_task() (lock + esp_zb_scheduler_alarm).
 */

#include "config.h"
#include "led.h"
#include "uart_link.h"
#include "ota.h"
#include "ble_update.h"
#include "ble_log.h"
#include "maint.h"
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

/* ── Lock Zigbee (2.x) ───────────────────────────────────────────────── */
/* esp_zigbee.h (2.x): "It's mandatory to acquire the lock before calling any
 * Zigbee SDK APIs, except that the call site is in Zigbee callbacks." Su ZB2,
 * senza lock, gli allarmi programmati da altri task facevano sparire altri
 * allarmi (watchdog, keepalive). Dal task Zigbee: chiamata diretta; da altri
 * task: con il lock (stessa funzione di ZB2). */
static TaskHandle_t s_zb_task = NULL;   /* task Zigbee_main, impostato in esp_zb_task() */

static void zb_alarm_any_task(esp_zb_callback_t cb, uint8_t param, uint32_t ms)
{
    if (s_zb_task != NULL && xTaskGetCurrentTaskHandle() == s_zb_task) {
        esp_zb_scheduler_alarm(cb, param, ms);
        return;
    }
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
        esp_zb_scheduler_alarm(cb, param, ms);
        esp_zb_lock_release();
    } else {
        ESP_LOGW(TAG, "lock Zigbee non ottenuto in 1 s: aggiornamento verso HA saltato (lo recupera il keepalive)");
    }
}

/* ── Zigbee hard/phy reset (vedi smart_switch/main.c) ────────────────── */
/* 2.x: credenziali nel namespace NVS "zigbee" (+ zb_fct, vecchia zb_storage):
 * cancellate da maint_erase_zigbee_credentials (stessa funzione di ZB2). */
static void zigbee_hard_reset(const char *reason)
{
    ESP_LOGW(TAG, "*** ZIGBEE HARD RESET (%s) — cancello credenziali Zigbee ***", reason);
    led_set_state(LED_ERROR);
    maint_erase_zigbee_credentials(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

/* Factory reset Zigbee (BOOT tenuto 5 s oppure EP12): riavvio voluto, senza deep sleep. */
static void zigbee_factory_reset_and_restart(void)
{
    ESP_LOGW(TAG, "*** FACTORY RESET ZIGBEE — cancello credenziali... ***");
    led_set_state(LED_FACTORY_RESET);
    maint_mark_commanded_restart();
    maint_erase_zigbee_credentials(false);
    ESP_LOGW(TAG, "Riavvio...");
    vTaskDelay(pdMS_TO_TICKS(50));
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

/* Un comando da Zigbee: esp-zigbee-lib ha già scritto da sola l'attributo locale
 * con il valore richiesto, quindi la cache s_zb_last_* non corrisponde più a
 * quello che vede HA. La si invalida, così la prossima riga di stato (risposta
 * al GET che segue ogni comando) riscrive il valore vero anche se non è cambiato:
 * un comando non eseguito dalla VMC non resta "ON" in HA. */
static void invalidate_state_cache(uint8_t ep)
{
    switch (ep) {
    case EP_ONOFF:   s_zb_last_onoff   = 0xFF; break;
    case EP_SPEED:   s_zb_last_speed   = 0xFF; break;
    case EP_FRESHUP: s_zb_last_freshup = 0xFF; break;
    case EP_MODE_AUTO:
    case EP_MODE_SCAMBIO:
    case EP_MODE_BYPASS:
        s_zb_last_auto = s_zb_last_scambio = s_zb_last_bypass = 0xFF;
        break;
    default: break;
    }
}

static void do_state_zigbee_sync(uint8_t param)
{
    (void)param;
    if (!zigbee_ready) return;
    vmc_state_t st = s_pending_state;
    queue_state_sync(&st, false);
}

/* ── Stato del collegamento ATmega → Zigbee (cluster privato) ────────── */
/*
 * Cluster LINK_CLUSTER_ID su EP1, attributi a sola lettura con flag
 * "reportable" (senza, esp_zb_zcl_report_attr_cmd_req risponde
 * ESP_ERR_NOT_SUPPORTED: verificato su ZB2, diag.c). Nessun reporting
 * configurato da ZHA (la quirk non ha reporting_config): a ogni cambio, e a
 * ogni ping riuscito, si manda un report singolo al coordinator, come per
 * EP10-13. Un attributo alla volta, scaglionati (vedi nota sopra).
 */
#define LINK_CLUSTER_ID        0xFF12   /* > ESP_ZB_CUSTOM_CLUSTER_ID_MIN_VAL, come 0xFF11 di ZB2 */
#define LINK_ATTR_ALIVE        0x0000   /* bool: l'ATmega risponde a GET */
#define LINK_ATTR_REGISTERED   0x0001   /* bool: ATmega agganciato come slave sul bus P1P2 */
#define LINK_ATTR_LAST_ERROR   0x0002   /* u8: ultimo E,<code> (0 = nessuno, azzerato da una riga di stato) */
#define LINK_REPORT_BASE_MS    300      /* dopo il sync dei 6 On/Off (0..200 ms) */

static const uint16_t s_link_attr_ids[3] = { LINK_ATTR_ALIVE, LINK_ATTR_REGISTERED, LINK_ATTR_LAST_ERROR };
static uint8_t s_link_zb_val[3];                 /* storage iniziale degli attributi */
static volatile uint8_t s_link_alive      = 0;
static volatile uint8_t s_link_registered = 0;
static volatile uint8_t s_link_last_error = 0;

static esp_zb_attribute_list_t *create_link_cluster(void)
{
    esp_zb_attribute_list_t *al = esp_zb_zcl_attr_list_create(LINK_CLUSTER_ID);
    if (!al) {
        ESP_LOGE(TAG, "creazione cluster 0x%04x fallita", LINK_CLUSTER_ID);
        return NULL;
    }
    const uint8_t types[3] = { ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_TYPE_U8 };
    for (int i = 0; i < 3; i++) {
        s_link_zb_val[i] = 0;
        esp_err_t err = esp_zb_custom_cluster_add_custom_attr(al, s_link_attr_ids[i], types[i],
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s_link_zb_val[i]);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "attributo 0x%04x non aggiunto: %s", s_link_attr_ids[i], esp_err_to_name(err));
    }
    return al;
}

/* param = indice in s_link_attr_ids. Contesto Zigbee (allarme). */
static void do_link_report_one(uint8_t idx)
{
    if (!zigbee_ready || idx >= 3) return;
    uint8_t v = (idx == 0) ? s_link_alive : (idx == 1) ? s_link_registered : s_link_last_error;
    esp_zb_zcl_set_attribute_val(EP_ONOFF, LINK_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 s_link_attr_ids[idx], &v, false);

    esp_zb_zcl_report_attr_cmd_t cmd = {0};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.zcl_basic_cmd.src_endpoint = EP_ONOFF;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = LINK_CLUSTER_ID;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
    cmd.attributeID = s_link_attr_ids[idx];
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&cmd);
    ESP_LOGD(TAG, "report link 0x%04x=%u: %s", s_link_attr_ids[idx], v, esp_err_to_name(err));
}

/* Contesto Zigbee. */
static void queue_link_reports(uint32_t base_ms)
{
    for (uint8_t i = 0; i < 3; i++)
        esp_zb_scheduler_alarm(do_link_report_one, i, base_ms + i * STATE_SYNC_STAGGER_MS);
}

static void do_link_reports(uint8_t param)
{
    (void)param;
    if (!zigbee_ready) return;
    queue_link_reports(0);
}

/* ATmega non risponde: HA non deve mostrare comandi mai eseguiti. Si riscrivono i
 * 6 On/Off con l'ultimo stato vero (o tutto OFF se non è mai arrivato), poi il report
 * del collegamento. Contesto Zigbee (allarme). */
static void do_link_changed(uint8_t alive)
{
    if (!zigbee_ready) return;
    if (!alive) {
        vmc_state_t st = s_last_state;
        if (!s_have_state) {
            memset(&st, 0, sizeof(st));
            st.mode = 0xFF;   /* nessuna modalità ON */
        }
        queue_state_sync(&st, true);
    }
    queue_link_reports(LINK_REPORT_BASE_MS);
}

/* Callback da uart_link.c (task di controllo del collegamento). */
static void on_link_alive(bool ok)
{
    s_link_alive = ok ? 1 : 0;
    if (!zigbee_ready) return;   /* lo manda il primo ping riuscito */
    zb_alarm_any_task(do_link_changed, ok ? 1 : 0, 0);
}

/* Callback da uart_link.c: gira nel task UART, non nel task Zigbee.
 * Salva solo dati e schedula la sincronizzazione vera tramite scheduler_alarm. */
static void on_vmc_state(const vmc_state_t *st)
{
    s_last_state  = *st;
    s_have_state  = true;
    bool link_changed = false;
    if (s_link_registered != (st->registered ? 1 : 0)) {
        s_link_registered = st->registered ? 1 : 0;
        link_changed = true;
    }
    if (s_link_last_error != 0) {   /* una riga di stato valida azzera l'ultimo errore */
        s_link_last_error = 0;
        link_changed = true;
    }
    if (!zigbee_ready) return;
    s_pending_state = *st;
    zb_alarm_any_task(do_state_zigbee_sync, 0, 0);   /* 2.x: task UART → serve il lock */
    if (link_changed)
        zb_alarm_any_task(do_link_reports, 0, LINK_REPORT_BASE_MS);
}

static void on_vmc_error(uint8_t code)
{
    /* Il codice 3 (silenzio dal master) è il più indicativo di un problema
     * reale sul bus P1P2, non solo rumore passeggero. */
    if (code == 3) {
        ESP_LOGW(TAG, "ATmega: silenzio dal master P1P2 (bus/master assente?)");
    }
    if (s_link_last_error == code) return;
    s_link_last_error = code;
    if (!zigbee_ready) return;
    zb_alarm_any_task(do_link_reports, 0, 0);   /* task UART → serve il lock */
}

/* ── OTA ─────────────────────────────────────────────────────────────── */
#if ZB_OTA_ENABLED
static void ota_find_server(void)
{
    esp_zb_ota_upgrade_client_query_interval_set(EP_ONOFF, OTA_UPGRADE_QUERY_INTERVAL);
    esp_zb_ota_upgrade_client_query_image_req(0x0000, 1);
    ESP_LOGI(TAG, "OTA: query → coordinator, polling ogni %d min", OTA_UPGRADE_QUERY_INTERVAL);
}
#endif

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

#if ZB_OTA_ENABLED
    if (ep == EP_ONOFF)
        esp_zb_cluster_list_add_ota_cluster(cl,
            ota_cluster_create(), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
#endif
    if (ep == EP_ONOFF) {
        esp_zb_attribute_list_t *link = create_link_cluster();
        if (link)
            esp_zb_cluster_list_add_custom_cluster(cl, link, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }

    esp_zb_endpoint_config_t epcfg = {
        .endpoint           = ep,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
    ESP_LOGI(TAG, "EP%d: On/Off (%s)", ep, label);
}

/* ── Endpoint virtuali di manutenzione (come smart switch ZB2) ───────── */
static void add_virtual_switch_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep, const char *what,
                                        bool initial_on)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_cfg_t oo = { .on_off = initial_on ? 1 : 0 };
    esp_zb_cluster_list_add_on_off_cluster(cl,
        esp_zb_on_off_cluster_create(&oo), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint           = ep,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
    ESP_LOGI(TAG, "EP%d: interruttore virtuale %s", ep, what);
}

static void create_endpoints(esp_zb_ep_list_t *ep_list)
{
    add_switch_endpoint(ep_list, EP_ONOFF,        "Accensione");
    add_switch_endpoint(ep_list, EP_SPEED,        "Velocità (OFF=Lento, ON=Veloce)");
    add_switch_endpoint(ep_list, EP_FRESHUP,      "Fresh-up");
    add_switch_endpoint(ep_list, EP_MODE_AUTO,    "Modo Auto");
    add_switch_endpoint(ep_list, EP_MODE_SCAMBIO, "Modo Scambio");
    add_switch_endpoint(ep_list, EP_MODE_BYPASS,  "Modo Bypass");

    add_virtual_switch_endpoint(ep_list, BLE_SWITCH_EP,     "aggiornamento BLE",       false);
    add_virtual_switch_endpoint(ep_list, PROG_SWITCH_EP,    "modalità programmazione", false);
    add_virtual_switch_endpoint(ep_list, FACTORY_SWITCH_EP, "factory reset Zigbee",    false);
    add_virtual_switch_endpoint(ep_list, LOG_SWITCH_EP,     "log via BLE",             ble_log_enabled());
}

/* ── Interruttori virtuali: azioni ───────────────────────────────────── */
/* L'azione parte da un task a parte, così la risposta Zigbee al comando fa in tempo a partire. */
typedef void (*maint_action_t)(void);

static void maint_action_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ((maint_action_t)pv)();
    vTaskDelete(NULL);
}

static void log_switch_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ble_log_set_enabled_and_restart((bool)(uintptr_t)pv);
    vTaskDelete(NULL);
}

/* Report singolo On/Off degli EP virtuali verso il coordinator (0x0000, endpoint 1 di ZHA):
 * senza, dopo un riavvio HA continua a mostrare l'ultimo valore (es. EP10 ON dopo
 * l'aggiornamento BLE), perché l'attributo riparte al valore di creazione (OFF, EP13 = stato
 * del log BLE) senza nessuna variazione che generi un report.
 * Chiamato dalla callback del ping (contesto Zigbee): niente lock. */
static void report_virtual_switches(void)
{
    static const uint8_t eps[] = { BLE_SWITCH_EP, PROG_SWITCH_EP, FACTORY_SWITCH_EP, LOG_SWITCH_EP };
    for (size_t i = 0; i < sizeof(eps); i++) {
        esp_zb_zcl_report_attr_cmd_t cmd = {0};
        cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
        cmd.zcl_basic_cmd.dst_endpoint = 1;
        cmd.zcl_basic_cmd.src_endpoint = eps[i];
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
        cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
        esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&cmd);
        ESP_LOGI(TAG, "EP%d: report stato interruttore virtuale: %s", eps[i], esp_err_to_name(err));
    }
}

static esp_err_t handle_virtual_switch(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    static bool requested = false;
    if (requested) return ESP_OK;
    const bool on = *(bool *)msg->attribute.data.value;

    /* Il log via BLE si attiva e si disattiva; gli altri reagiscono solo a ON. */
    if (msg->info.dst_endpoint == LOG_SWITCH_EP) {
        if (on == ble_log_enabled()) return ESP_OK;
        requested = true;
        ESP_LOGW(TAG, "Zigbee: log via BLE %s — riavvio tra 1.5 s", on ? "ON" : "OFF");
        xTaskCreate(log_switch_task, "log_sw", 4096, (void *)(uintptr_t)on, 5, NULL);
        return ESP_OK;
    }
    if (!on) return ESP_OK;

    maint_action_t action;
    const char *what;
    switch (msg->info.dst_endpoint) {
    case BLE_SWITCH_EP:
        action = ble_update_request_and_restart;   what = "modalità aggiornamento BLE"; break;
    case PROG_SWITCH_EP:
        action = maint_enter_download_mode;        what = "modalità programmazione";    break;
    case FACTORY_SWITCH_EP:
        action = zigbee_factory_reset_and_restart; what = "factory reset Zigbee";       break;
    default:
        return ESP_OK;
    }
    requested = true;
    ESP_LOGW(TAG, "Zigbee: richiesta %s — avvio tra 1.5 s", what);
    xTaskCreate(maint_action_task, "maint_act", 4096, action, 5, NULL);
    return ESP_OK;
}

/* ── Handler comandi Zigbee → comandi ATmega ─────────────────────────── */

static esp_err_t handle_on_off(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) return ESP_OK;
    if (msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_BOOL) return ESP_OK;

    uint8_t ep    = msg->info.dst_endpoint;
    if (IS_VIRTUAL_EP(ep)) return handle_virtual_switch(msg);
    invalidate_state_cache(ep);   /* contesto Zigbee: nessun lock */
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
#if ZB_OTA_ENABLED
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
#endif
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
            /* 2.x: una nuova schedulazione dello stesso callback crea una seconda
             * catena (visto su ZB2): cancella prima quella eventualmente attiva */
            esp_zb_scheduler_alarm_cancel(coordinator_ping_cb, 0);
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
#if ZB_OTA_ENABLED
            ota_find_server();
#endif
            /* Stato reale degli interruttori virtuali EP10-13 (dopo un riavvio HA mostrava l'ultimo). */
            report_virtual_switches();
        }

        /* force_all=true: riscrive tutti gli attributi (scaglionati) per
         * aggiornare ZHA e stimolare route discovery, ad ogni ping riuscito,
         * non solo al primo join. */
        if (s_have_state) queue_state_sync(&s_last_state, true);
        /* Stato del collegamento ATmega (cluster 0xFF12), dopo il sync dei 6 On/Off. */
        queue_link_reports(LINK_REPORT_BASE_MS);

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
        int held_ticks = 0;
        for (int i = 0; i < FACTORY_RESET_HOLD_S * 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (gpio_get_level(FACTORY_RESET_GPIO_NUM) != 0) {
                held = false;
                held_ticks = i + 1;
                break;
            }
        }

        if (!held) {
            /* Pressione breve (0.3 s - 5 s) con il log via BLE attivo: lo disattiva e riavvia
             * (come smart switch ZB2). */
            if (held_ticks >= 3 && ble_log_enabled()) {
                ESP_LOGW(TAG, "GPIO9: pressione breve — disattivo il log via BLE");
                ble_log_set_enabled_and_restart(false);
            }
            ESP_LOGI(TAG, "GPIO9 rilasciato — annullato");
            led_set_state(zigbee_ready ? LED_OPERATIONAL : LED_ZIGBEE_SEARCHING);
            continue;
        }

        zigbee_factory_reset_and_restart();
    }
}

/* ── Task Zigbee ─────────────────────────────────────────────────────── */
/* Migrazione una tantum dei dati Zigbee 1.x (zb_storage, ZBOSS) → 2.x (nvs, namespace
 * "zigbee"), come in ESP32-H2-Custom-Zigbee 5.5-ZB2 (provata: il modulo rientra in rete da
 * solo) e nell'esempio Espressif examples/nvram_migration. Dopo nvs_flash_init, prima
 * dell'avvio dello stack. Il marcatore "zb2_mig/done" evita di ripeterla; un solo
 * tentativo, riuscito o no, senza bloccare l'avvio. */
static void zb2_migrate_legacy_nvram(void)
{
    nvs_handle_t h;
    if (nvs_open("zb2_mig", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "migrazione 1.x: nvs_open fallito, salto");
        return;
    }
    uint8_t done = 0;
    if (nvs_get_u8(h, "done", &done) == ESP_OK) {
        nvs_close(h);
        return;                                       /* già fatta */
    }
    esp_err_t err = esp_zigbee_nvram_migrate(NULL);   /* default: zb_storage → nvs, IEEE = MAC */
    ESP_LOGW(TAG, "migrazione dati Zigbee 1.x → 2.x: %s", esp_err_to_name(err));
    nvs_set_u8(h, "done", 1);
    nvs_commit(h);
    nvs_close(h);
}

static void esp_zb_task(void *pv)
{
    s_zb_task = xTaskGetCurrentTaskHandle();   /* per zb_alarm_any_task() */
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
    /* Keepalive avviato solo dopo STEERING riuscito: avviarlo anche qui creava
     * due catene di allarmi in 2.x (visto su ZB2) */

    esp_zb_stack_main_loop();
}

/* ── Entry point ─────────────────────────────────────────────────────── */
void app_main(void)
{
    /* NVS prima del controllo sul reset: serve a leggere il segno di riavvio voluto. */
    ESP_ERROR_CHECK(nvs_flash_init());

    {
        esp_reset_reason_t rr = esp_reset_reason();
        bool commanded = maint_take_commanded_restart();   /* letto e azzerato sempre */
        if (rr != ESP_RST_POWERON &&
            rr != ESP_RST_DEEPSLEEP &&
            rr != ESP_RST_EXT) {
            if (commanded) {
                /* Riavvio deciso dall'utente (BLE, log BLE, factory reset, seriale): avvio normale. */
                ESP_LOGI(TAG, "Riavvio comandato (reason=%d) — avvio normale, niente deep sleep", (int)rr);
            } else {
                ESP_LOGW(TAG, "Reset non-pulito (reason=%d) — deep sleep 3s per reset radio", (int)rr);
                esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
                esp_deep_sleep_start();
            }
        }
    }

    /* Modalità aggiornamento BLE: niente Zigbee, niente link ATmega. Non ritorna. */
    if (ble_update_pending()) {
        ble_update_run();
    }

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    zb2_migrate_legacy_nvram();
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    maint_serial_start();
    ble_log_start_if_enabled();

    /* Anti-rollback OTA */
    ota_mark_valid();

    led_init();

    xTaskCreate(factory_reset_task, "factory_rst", 2048, NULL, 1, NULL);
    xTaskCreate(zb_watchdog_task,   "zb_wdt",       2048, NULL, 2, NULL);

    /* Link seriale verso l'ATmega: gli aggiornamenti di stato arrivano da
     * qui via on_vmc_state/on_vmc_error. */
    uart_link_init(on_vmc_state, on_vmc_error, on_link_alive);

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
