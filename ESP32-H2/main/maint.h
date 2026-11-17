/*
 * maint.h — manutenzione: modalità programmazione (download seriale) e comandi da seriale
 */

#pragma once

/* Imposta il flag "al prossimo avvio entra nel bootloader ROM" e riavvia. Non ritorna. */
void maint_enter_download_mode(void);

/* Avvia il task che legge i comandi dalla UART di console (scrivi "help" + invio). */
void maint_serial_start(void);

#include <stdbool.h>

/* ZB2: cancella le credenziali Zigbee. Non riavvia.
 * esp-zigbee-lib 2.0.5 (livello di compatibilità 1.x) salva i dati Zigbee nella partizione NVS
 * di default "nvs", namespace "zigbee" (verificato con un dump del modulo di sviluppo l'11/11/2026;
 * nome del namespace non documentato). Cancella quel namespace, la partizione zb_fct e anche la
 * vecchia partizione FAT zb_storage (1.x), così una futura migrazione non ripristina credenziali vecchie.
 * erase_phy = true cancella anche phy_init (hard reset). Gli altri namespace NVS non vengono toccati. */
void maint_erase_zigbee_credentials(bool erase_phy);

/* Daikin (al posto di diag_set_restart_cause dello smart switch): segna in NVS che il prossimo
 * riavvio è voluto (aggiornamento BLE, log BLE, factory reset, comando seriale), così all'avvio
 * non si fa il deep sleep di recupero previsto per i reset non puliti. */
void maint_mark_commanded_restart(void);

/* Legge e azzera il segno lasciato da maint_mark_commanded_restart(). Una sola volta, a inizio
 * app_main, dopo nvs_flash_init(). */
bool maint_take_commanded_restart(void);
