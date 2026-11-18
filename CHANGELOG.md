# Changelog — ESP32-H2 Zigbee bridge

The ATmega328 firmware (`Atmega328/`) is versioned separately and is not
covered here.

## [v2.0.1]

- **Fix (important):** when the coordinator stayed unreachable for 6
  consecutive keepalive pings (≈ 4–5 minutes) the firmware erased the Zigbee
  credentials (logic inherited from the smart switch firmware, written for
  ZBOSS state corruption). Switching off the coordinator for a few minutes
  (Home Assistant maintenance, faulty USB stick) therefore un-paired the
  bridge. Now the bridge enters deep sleep for 3 s and restarts **keeping the
  credentials**; after the restart the stack uses the stored network data and
  the bridge rejoins when the coordinator is back. While the coordinator is
  off the cycle repeats every ≈ 5 minutes. The erase after steering remains
  only for the invalid short address `0xFFFF`.

## [v2.0.0]

- **Changed:** port to `esp-zigbee-lib` 2.x (no ZBOSS) through the 1.x
  compatibility layer (`CONFIG_ZB_SDK_1xx`); tested with ESP-IDF 5.5.6.
  One-time migration of the 1.x Zigbee data at the first start.
- **Removed:** Zigbee OTA (the OTA cluster is not supported by the
  compatibility layer). Updates over Bluetooth or by cable.
- **New:** maintenance switches EP10–13: BLE update, PROG (serial download
  mode), RESCRED (Zigbee factory reset), LOG (log over Bluetooth); Web
  Bluetooth pages in `tools/`.
- **New:** ATmega link check (`GET` every 60 s and after each command) and
  private cluster `0xFF12` on EP1 (ATmega reachable, registered on P1/P2,
  last error); commands not confirmed by the VAM are rolled back in Home
  Assistant.
- **New:** ZHA quirk `ha/daikin_vam_quirk.py`.
- **Fix:** Zigbee lock for calls from the UART task; coordinator keepalive
  started only once, after successful steering.

## [v0.1.0]

- First release: 6 On/Off endpoints, Zigbee OTA client (`esp-zigbee-lib`
  1.x + ZBOSS).
