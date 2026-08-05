/*
 * config.h — Costanti del firmware ESP32-H2 (bridge Zigbee<->ATmega)
 *
 * Endpoint ed GPIO fissi, verificati sullo schema "Atmega+ESP32 Schematic"
 * (Hardware/). Non ci sono canali configurabili: gli endpoint sono
 * sempre gli stessi sei.
 */

#pragma once

/* ------------------------------------------------------------------ */
/*  Endpoint Zigbee                                                    */
/* ------------------------------------------------------------------ */
#define EP_ONOFF         1   /* On/Off: accensione VMC */
#define EP_SPEED         2   /* On/Off: velocità (OFF=Lento, ON=Veloce) */
#define EP_FRESHUP       3   /* On/Off: fresh-up */
#define EP_MODE_AUTO     4   /* On/Off: modalità Auto */
#define EP_MODE_SCAMBIO  5   /* On/Off: modalità Scambio */
#define EP_MODE_BYPASS   6   /* On/Off: modalità Bypass */

/* ------------------------------------------------------------------ */
/*  GPIO (da schema: IO9=bottone con pull-up 10k, IO22=LED via R21 1k) */
/* ------------------------------------------------------------------ */
#define FACTORY_RESET_GPIO_NUM   9
