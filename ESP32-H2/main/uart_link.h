/*
 * uart_link.h — Link seriale verso l'ATmega328 (firmware P1P2_ATmega_v19)
 *
 * Protocollo concordato (vedi VMC/P1P2_ATmega_v19.txt):
 *   ESP32 -> ATmega: ON OFF VL VH MA MS MB FU1 FU0 GET
 *   ATmega -> ESP32: S,<onoff>,<mode>,<speed>,<freshup>,<registered>
 *                    E,<code>   (1=CRC bus, 2=canale perso, 3=silenzio master)
 *
 * UART1 su IO4(RX, da PD1/TXD ATmega tramite partitore)/IO5(TX, verso
 * PD0/RXD ATmega), 115200 8N1.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool    onoff;
    uint8_t mode;       /* 0=Auto 1=Scambio 2=Bypass */
    uint8_t speed;       /* valori grezzi bus: 01=Lento 05=Veloce */
    bool    freshup;
    bool    registered;  /* l'ATmega è agganciato come slave sul bus P1P2 */
} vmc_state_t;

typedef void (*uart_link_state_cb_t)(const vmc_state_t *state);
typedef void (*uart_link_error_cb_t)(uint8_t code);
/* ok = l'ATmega ha risposto a GET entro LINK_REPLY_TIMEOUT_MS. Chiamata al cambio (e dopo
 * il primo controllo) e a ogni controllo fallito, dal task di controllo del collegamento. */
typedef void (*uart_link_alive_cb_t)(bool ok);

/*
 * Inizializza la UART e avvia il task di ricezione e quello di controllo del
 * collegamento. Le callback vengono chiamate da quei task, non dal task
 * Zigbee: verso lo stack passare da zb_alarm_any_task (vedi main.c).
 *
 * Controllo del collegamento: l'ATmega manda lo stato solo ai cambiamenti o su
 * GET, quindi il silenzio da solo non dice niente. Ogni LINK_CHECK_PERIOD_MS, e
 * LINK_AFTER_CMD_MS dopo ogni comando, si manda GET e si aspetta una riga
 * qualsiasi entro LINK_REPLY_TIMEOUT_MS.
 */
void uart_link_init(uart_link_state_cb_t state_cb, uart_link_error_cb_t error_cb,
                    uart_link_alive_cb_t alive_cb);

/*
 * Invia un comando all'ATmega. Chiamabile direttamente dal task Zigbee:
 * è solo una scrittura sulla UART, non tocca lo stack. Per i comandi diversi
 * da GET programma anche un controllo (GET) poco dopo.
 */
void uart_link_send_cmd(const char *cmd);
