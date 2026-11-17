"""Handmade! VmcP1P2Bridge (Daikin VAM via P1P2): nomi entità e stato del collegamento ATmega.

Firmware: ESP32-H2/main/main.c (cluster privato LINK_CLUSTER_ID 0xFF12 su EP1, endpoint 10-13).
Modello: smartswitch_diag.py del progetto ESP32-H2-Custom-Zigbee (ZB2).
"""

from typing import Final

import zigpy.types as t
from zigpy.zcl.clusters.general import OnOff
from zigpy.zcl.foundation import BaseAttributeDefs, ZCLAttributeDef

from zhaquirks.builder import BinarySensorDeviceClass, EntityType, QuirkBuilder
from zhaquirks.clusters import CustomCluster


def _rp(attr_id: int, attr_type) -> ZCLAttributeDef:
    # is_manufacturer_specific=False: sul firmware gli attributi sono registrati senza
    # codice costruttore, e zigpy per i cluster 0xFC00+ lo aggiungerebbe ai frame.
    return ZCLAttributeDef(
        id=attr_id, type=attr_type, access="rp", is_manufacturer_specific=False
    )


class VmcLink(CustomCluster):
    """Cluster privato: stato del collegamento ESP32 <-> ATmega <-> bus P1P2."""

    cluster_id: t.uint16_t = 0xFF12
    name: str = "VMC Link"
    ep_attribute: str = "vmc_link"

    class AttributeDefs(BaseAttributeDefs):
        """Attribute definitions."""

        # l'ATmega risponde a GET (controllo ogni 60 s e dopo ogni comando)
        atmega_alive: Final = _rp(0x0000, t.Bool)
        # l'ATmega è agganciato come slave sul bus P1P2
        p1p2_registered: Final = _rp(0x0001, t.Bool)
        # ultimo errore dall'ATmega: 0 nessuno, 1 CRC bus, 2 canale perso, 3 silenzio master.
        # Torna a 0 alla prossima riga di stato valida.
        atmega_last_error: Final = _rp(0x0002, t.uint8_t)


def _name_switch(builder, endpoint_id, key, name, primary=False, enabled=True):
    """Dà un nome all'interruttore On/Off dell'endpoint."""
    return builder.change_entity_metadata(
        endpoint_id=endpoint_id,
        cluster_id=OnOff.cluster_id,
        new_primary=primary,
        new_translation_key=key,
        new_fallback_name=name,
        new_entity_registry_enabled_default=enabled,
    )


builder = QuirkBuilder("Handmade!", "VmcP1P2Bridge").replaces(VmcLink)

# EP1-6: comandi della VMC (i tre modi sono mutuamente esclusivi, lo stato vero arriva dall'ATmega).
builder = _name_switch(builder, 1, "vmc_power", "Accensione", primary=True)
builder = _name_switch(builder, 2, "vmc_speed_high", "Velocità alta")
builder = _name_switch(builder, 3, "vmc_fresh_up", "Fresh-up")
builder = _name_switch(builder, 4, "vmc_mode_auto", "Modo Auto")
builder = _name_switch(builder, 5, "vmc_mode_exchange", "Modo Scambio")
builder = _name_switch(builder, 6, "vmc_mode_bypass", "Modo Bypass")

# EP10 = BLE (aggiornamento via Bluetooth, attivo); EP11 = PROG (modalità programmazione) e
# EP12 = RESCRED (cancella le credenziali Zigbee) restano disabilitati finché non li abiliti tu.
builder = _name_switch(builder, 10, "ble_update", "BLE")
builder = _name_switch(builder, 11, "programming_mode", "PROG", enabled=False)
builder = _name_switch(builder, 12, "zigbee_factory_reset", "RESCRED", enabled=False)
# EP13 = LOG (log via BLE: ON attiva e resta attivo anche dopo i riavvii, OFF o BOOT breve lo spegne).
builder = _name_switch(builder, 13, "ble_log", "LOG", enabled=False)

# Stato del collegamento. SENZA reporting_config di proposito: il firmware manda da solo un
# report a ogni cambio e a ogni keepalive riuscito (ogni 5 min), come per EP10-13.
builder = builder.binary_sensor(
    attribute_name=VmcLink.AttributeDefs.atmega_alive.name,
    cluster_id=VmcLink.cluster_id,
    endpoint_id=1,
    entity_type=EntityType.DIAGNOSTIC,
    device_class=BinarySensorDeviceClass.CONNECTIVITY,
    translation_key="atmega_alive",
    fallback_name="Collegamento ATmega",
)
builder = builder.binary_sensor(
    attribute_name=VmcLink.AttributeDefs.p1p2_registered.name,
    cluster_id=VmcLink.cluster_id,
    endpoint_id=1,
    entity_type=EntityType.DIAGNOSTIC,
    device_class=BinarySensorDeviceClass.CONNECTIVITY,
    translation_key="p1p2_registered",
    fallback_name="Registrato bus P1P2",
)
builder = builder.sensor(
    attribute_name=VmcLink.AttributeDefs.atmega_last_error.name,
    cluster_id=VmcLink.cluster_id,
    endpoint_id=1,
    suggested_display_precision=0,
    entity_type=EntityType.DIAGNOSTIC,
    translation_key="atmega_last_error",
    fallback_name="Errore ATmega",
)
builder.add_to_registry()
