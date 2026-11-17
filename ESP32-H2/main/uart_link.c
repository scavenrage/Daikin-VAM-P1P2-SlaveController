/*
 * uart_link.c — Link seriale verso l'ATmega328 (vedi uart_link.h per il protocollo)
 */

#include "uart_link.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "UART_LINK";

#define UART_PORT       UART_NUM_1
#define UART_TX_GPIO    5   /* IO5 -> PD0 (RXD) ATmega */
#define UART_RX_GPIO    4   /* IO4 <- PD1 (TXD) ATmega, tramite partitore 5V->3.3V */
#define UART_BAUD       115200
#define UART_RX_BUF_SIZE 256
#define LINE_MAX        32

#define LINK_CHECK_PERIOD_MS   60000   /* GET periodico */
#define LINK_AFTER_CMD_MS      1500    /* GET dopo un comando (risposta del bus) */
#define LINK_REPLY_TIMEOUT_MS  2000    /* attesa di una riga dopo GET */

static uart_link_state_cb_t s_state_cb = NULL;
static uart_link_error_cb_t s_error_cb = NULL;
static uart_link_alive_cb_t s_alive_cb = NULL;
static SemaphoreHandle_t    s_rx_line  = NULL;   /* data dal task RX a ogni riga completa */
static TaskHandle_t         s_mon_task = NULL;   /* task di controllo del collegamento */

static void uart_write_line(const char *cmd)
{
    char buf[LINE_MAX];
    int len = snprintf(buf, sizeof(buf), "%s\n", cmd);
    if (len > 0) uart_write_bytes(UART_PORT, buf, (size_t)len);
}

/* line è già senza il prefisso "S,": "<onoff>,<mode>,<speed>,<freshup>,<registered>" */
static bool parse_state_line(char *line, vmc_state_t *out)
{
    char *save = NULL;
    char *tok = strtok_r(line, ",", &save);
    int values[5];
    for (int i = 0; i < 5; i++) {
        if (!tok) return false;
        values[i] = atoi(tok);
        tok = strtok_r(NULL, ",", &save);
    }
    out->onoff      = values[0] != 0;
    out->mode       = (uint8_t)values[1];
    out->speed      = (uint8_t)values[2];
    out->freshup    = values[3] != 0;
    out->registered = values[4] != 0;
    return true;
}

static void handle_line(char *line)
{
    if (line[0] == 'S' && line[1] == ',') {
        vmc_state_t st;
        if (parse_state_line(line + 2, &st)) {
            ESP_LOGI(TAG, "Stato: on=%d mode=%d speed=%d fu=%d reg=%d",
                     st.onoff, st.mode, st.speed, st.freshup, st.registered);
            if (s_state_cb) s_state_cb(&st);
        } else {
            ESP_LOGW(TAG, "Riga di stato malformata: \"%s\"", line);
        }
    } else if (line[0] == 'E' && line[1] == ',') {
        uint8_t code = (uint8_t)atoi(line + 2);
        ESP_LOGW(TAG, "Errore dall'ATmega: %d", code);
        if (s_error_cb) s_error_cb(code);
    } else if (line[0] != '\0') {
        ESP_LOGW(TAG, "Riga non riconosciuta: \"%s\"", line);
    }
}

static void uart_rx_task(void *pv)
{
    char lineBuf[LINE_MAX];
    uint8_t idx = 0;
    uint8_t byte;

    for (;;) {
        int n = uart_read_bytes(UART_PORT, &byte, 1, portMAX_DELAY);
        if (n <= 0) continue;

        if (byte == '\n') {
            lineBuf[idx] = '\0';
            if (idx > 0 && lineBuf[idx - 1] == '\r') lineBuf[idx - 1] = '\0';
            handle_line(lineBuf);
            if (lineBuf[0] != '\0') xSemaphoreGive(s_rx_line);   /* l'ATmega è vivo */
            idx = 0;
        } else if (idx < LINE_MAX - 1) {
            lineBuf[idx++] = (char)byte;
        }
        /* riga troppo lunga: i byte in eccesso vengono scartati finché non arriva \n */
    }
}

/* Controllo del collegamento: GET periodico (o poco dopo un comando) e attesa di
 * una riga qualsiasi. Log e callback solo al cambio di stato. */
static void link_monitor_task(void *pv)
{
    int known = -1;   /* -1 = mai controllato, 0 = non risponde, 1 = ok */
    TickType_t wait = 0;   /* primo controllo subito */

    for (;;) {
        /* Un comando inviato (notifica) anticipa il controllo. */
        if (ulTaskNotifyTake(pdTRUE, wait) > 0) {
            vTaskDelay(pdMS_TO_TICKS(LINK_AFTER_CMD_MS));
            ulTaskNotifyTake(pdTRUE, 0);   /* comandi a raffica: un solo GET */
        }

        xSemaphoreTake(s_rx_line, 0);      /* scarta righe arrivate prima del GET */
        uart_write_line("GET");
        bool ok = xSemaphoreTake(s_rx_line, pdMS_TO_TICKS(LINK_REPLY_TIMEOUT_MS)) == pdTRUE;

        bool changed = ((int)ok != known);
        if (changed) {
            if (ok) {
                ESP_LOGI(TAG, "ATmega raggiungibile");
            } else {
                ESP_LOGW(TAG, "ATmega non risponde (nessuna riga entro %d ms da GET)",
                         LINK_REPLY_TIMEOUT_MS);
            }
            known = ok;
        }
        /* Log solo al cambio; la callback anche a ogni controllo fallito: un comando
         * arrivato mentre l'ATmega non risponde va annullato anche in HA (main.c). */
        if (s_alive_cb && (changed || !ok)) s_alive_cb(ok);
        wait = pdMS_TO_TICKS(LINK_CHECK_PERIOD_MS);
    }
}

void uart_link_init(uart_link_state_cb_t state_cb, uart_link_error_cb_t error_cb,
                    uart_link_alive_cb_t alive_cb)
{
    s_state_cb = state_cb;
    s_error_cb = error_cb;
    s_alive_cb = alive_cb;
    s_rx_line  = xSemaphoreCreateBinary();

    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));

    xTaskCreate(uart_rx_task, "uart_link_rx", 3072, NULL, 5, NULL);

    /* Il primo controllo manda subito GET: riallinea lo stato corrente invece di
     * aspettare il prossimo cambiamento sul bus (prima era un GET qui). */
    xTaskCreate(link_monitor_task, "uart_link_mon", 2560, NULL, 4, &s_mon_task);
}

void uart_link_send_cmd(const char *cmd)
{
    uart_write_line(cmd);
    if (s_mon_task && strcmp(cmd, "GET") != 0)
        xTaskNotifyGive(s_mon_task);   /* controllo con GET poco dopo il comando */
}
