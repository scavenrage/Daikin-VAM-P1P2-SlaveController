# Daikin VAM — P1P2 Slave Controller with Zigbee bridge

Full read **and write** control of a Daikin VAM heat-recovery ventilation
unit over the P1/P2 bus, using an ATmega328 registered as a genuine
BRC301B61-compatible **slave controller**.

**The ATmega328 (`Atmega328/`) is the core of this project** — the actual
P1/P2 reverse-engineering result — and is fully usable on its own, e.g. as
a serial bridge into your own MQTT/Home Assistant setup. **The ESP32-H2
(`ESP32-H2/`) is a completely optional add-on**: it bridges that same
ATmega over a serial link to native Zigbee, for anyone who wants a Zigbee
device instead of rolling their own integration.

> **Status: deployed.** Installed behind the wall panel and in daily use
> since early August 2026, working reliably so far. Hardware/firmware are
> still considered early revisions — see [Known limitations](#known-limitations).

Built on top of [Arnold-n/P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT)
(P1P2MQTT library, CC BY-NC-ND 4.0). VAM units are currently read-only in
P1P2MQTT upstream; this project adds working write support for on/off, fan
speed, mode, and fresh-up, plus a Zigbee bridge on top.

## Architecture

```
Daikin VAM unit
      │  P1/P2 bus (9600 baud, HBS)
      │
 MAX22088 transceiver board  (Hardware/P1P2 Schematic.epro2 + P1P2 PCB.epro2)
      │  DOUT / DIN / RST
      │
  ATmega328                            ┊  optional, ESP32-H2/
 (P1/P2 slave controller,              ┊
  Atmega328/ — this is the core        ┊──UART (115200, S,/E,/commands)──  ESP32-H2
  of the project, usable standalone    ┊                                 (Zigbee router,
  e.g. over its own serial link)       ┊                                   6 On/Off endpoints
                                        ┊                                   + maintenance)
                                        ┊                                       │
                                        ┊                                 Zigbee coordinator
                                        ┊                                 (Home Assistant / ZHA)
```

Both MCUs live on one custom board (`Hardware/Atmega+ESP32 Schematic.epro2` +
`Atmega+ESP32 PCB.epro2`), connected to a separate small board carrying the
MAX22088 P1/P2 transceiver via a 7-pin header — but the ESP32-H2 half of
that board (and everything to its right in the diagram above) is optional:
the ATmega alone, wired directly to the MAX22088 board, is a complete,
working P1/P2 slave controller on its own.

The PCB is split in two mainly because of how this was validated: the P1/P2
transceiver board was built and tested on its own first, then the
ATmega+ESP32 board was added and tested against it once the bus protocol
was confirmed working. A single, smaller, more integrated board combining
both is possible and could be a future revision if there's interest.

Why two MCUs instead of running everything on the ESP32-H2? The P1/P2 bus
timing (response delays, silence-after-registration behavior, etc.) is what
was validated and got working reliably on the ATmega — that timing-sensitive
part has not been tried on the ESP32. Bridging a second MCU purely for
Zigbee was the path of least resistance, not a claim that single-chip
isn't possible.

## Repository layout

| Path | Contents |
|---|---|
| `Atmega328/` | **Core.** Arduino sketch for the ATmega328 — P1/P2 bus slave-controller logic |
| `ESP32-H2/` | *Optional.* ESP-IDF project for the ESP32-H2 — Zigbee bridge to the ATmega |
| `Hardware/` | EasyEDA Pro schematics/PCB for both boards (main board + P1/P2 transceiver board) |
| `protocol.md` | Full P1/P2 protocol reverse-engineering write-up (packet captures, byte-level mapping) |

## What this does

**Core (ATmega328, `Atmega328/`):**
- Registers on the P1/P2 bus as a real slave controller (not a generic aux
  responder) alongside the existing wall panel (master)
- Reads the unit's real-time state directly from the bus — no manual sync needed
- Commands on/off, fan speed (Low/High), mode (Auto/Heat-exchange/Bypass),
  and fresh-up — each verified with closed-loop confirmation from the bus
- Exposes that state and those commands over a simple serial line, ready to
  be picked up by whatever you want on the other end (the optional ESP32-H2
  bridge below, an MQTT gateway, a Raspberry Pi, ...)

**Optional (ESP32-H2, `ESP32-H2/`):**
- Bridges the ATmega's serial state/commands to native Zigbee
- Exposes 6 Zigbee On/Off endpoints to Home Assistant, reflecting the
  **real, bus-confirmed state** — never an optimistic guess at what a
  command is expected to do: a command the VAM does not confirm is rolled
  back in Home Assistant
- Reports the state of the ESP32 ↔ ATmega ↔ P1/P2 link to Home Assistant
  (ATmega reachable, registered on the bus, last error)
- Maintenance switches: firmware update over Bluetooth, serial download
  mode, Zigbee factory reset, live log over Bluetooth — so the bridge can be
  maintained once it's installed behind the wall panel (see
  [Maintenance switches](#maintenance-switches))
- Built on `esp-zigbee-lib` 2.x (Espressif's own Zigbee stack, no ZBOSS)
- Survives a coordinator outage: if the coordinator does not answer the
  keepalive for ≈ 4–5 minutes the bridge restarts (3 s deep sleep)
  **keeping its Zigbee credentials**, and rejoins by itself when the
  coordinator is back — no re-pairing after Home Assistant maintenance

Version history: [CHANGELOG.md](CHANGELOG.md).

## Hardware

- Custom PCB: **ATmega328P + ESP32-H2-MINI-1-N4** on one board (`Hardware/Atmega+ESP32*`)
- Separate small board: **MAX22088** P1/P2 bus transceiver (`Hardware/P1P2*`),
  connected to the main board via a 7-pin header (`RST`/`DOUT`/`DIN`/`GND`/`5V`)
- ATmega P1/P2 pins (unchanged from the original Arduino Uno prototype):
  `RST → D7`, `DOUT → D8`, `DIN → D9`
- ATmega ↔ ESP32-H2 serial link: ATmega `D0`/`D1` ↔ ESP32-H2 `IO5`/`IO4`
  (resistor divider on the ATmega→ESP32 direction, 5V logic down to 3.3V)
- ESP32-H2: `GPIO9` = factory-reset button (hold 5s to leave the Zigbee
  network; a short press, 0.3–5 s, turns the Bluetooth log off),
  `GPIO22` = status LED
- Bus: 9600 baud · Daikin/HBS CRC polynomial `0xD9`, feed `0x00`
- Power: the ESP32-H2 side is powered from an external 5V/12V supply; the
  ATmega side is designed to be powered from the P1/P2 bus wiring itself in
  the final wall-mounted installation (the same way the original wall panel
  is powered) — the two supplies are intentionally independent

**Bus layout:** wall panel (Master) + this board (Slave). No other aux
controller should be on the bus at the same time.

## Building — ATmega328

- Arduino IDE, board "Arduino Uno" (or a bare ATmega328P equivalent)
- Requires the [P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT) library
  (CC BY-NC-ND 4.0) — not redistributed here, install it separately
- Open `Atmega328/P1P2 Arduino.ino`, select the board/port, upload
- No manual setup needed: the slave registers on the bus automatically at
  boot and drives itself entirely from the ATmega↔ESP32 link below — there
  is no interactive serial console in normal operation

## Building — ESP32-H2 (Zigbee bridge, optional)

Tested with ESP-IDF **v5.5.6** and the `esp32h2` target.

```bash
cd ESP32-H2
idf.py set-target esp32h2
idf.py build
idf.py -p <PORT> flash monitor
```

The first build downloads `espressif/esp-zigbee-lib` 2.x automatically
(see `main/idf_component.yml`); this needs an internet connection once.
The firmware uses the library's 1.x compatibility layer
(`CONFIG_ZB_SDK_1xx` in `sdkconfig.defaults`).

**Before flashing for the first time, check:**
- `ESP_ZB_PRIMARY_CHANNEL_MASK` in `main/main.c` is hard-coded to Zigbee
  channel 25 — change it to match your own network
- The partition table (`partitions.csv`) uses a dual-OTA layout
  (`ota_0`/`ota_1`); if you're re-flashing a board that previously had a
  different (non-OTA) partition table, do a full `idf.py erase-flash`
  first, otherwise the old table in flash won't match the new one

**Updating a bridge that runs the older firmware (`esp-zigbee-lib` 1.x):**
- The 1.x firmware has no Bluetooth and its Zigbee OTA cannot be used
  (the 2.x compatibility layer does not support the OTA cluster), so the
  first update is done **by cable**. After that, updates go over Bluetooth.
- At the first start the firmware migrates the Zigbee network data from
  the old format (`esp_zigbee_nvram_migrate`), so the bridge rejoins the
  network by itself.
- The new endpoints (10–13) and the link cluster are only discovered at
  pairing time: remove the device in ZHA and add it again.
- When switching ESP-IDF version, delete `build/` and `sdkconfig` before
  building.

## Zigbee endpoints (Home Assistant / ZHA, optional ESP32-H2 bridge)

| EP | Cluster | Function |
|---|---|---|
| 1 | On/Off + link cluster `0xFF12` | Power (on/off) + link state |
| 2 | On/Off | Fan speed — Off = Low, On = High |
| 3 | On/Off | Fresh-up |
| 4 | On/Off | Mode: Auto |
| 5 | On/Off | Mode: Heat-exchange |
| 6 | On/Off | Mode: Bypass |
| 10 | On/Off | BLE (maintenance) |
| 11 | On/Off | PROG (maintenance) |
| 12 | On/Off | RESCRED (maintenance) |
| 13 | On/Off | LOG (maintenance) |

Endpoints 4–6 are mutually exclusive: only one is ever `ON`, reflecting
whichever mode is actually active on the bus. Switching one on sends the
corresponding command to the ATmega; turning one off without turning
another on has no direct equivalent on the unit, so the switch will simply
flip back to `ON` once the next real-state update arrives — the firmware
never assumes a command succeeded, it only reports what the ATmega actually
confirms.

### ATmega link

The ATmega only sends its state when something changes or when asked with
`GET`, so silence alone means nothing. The ESP32-H2 sends `GET` every 60 s
and 1.5 s after every command, and expects a reply within 2 s. The result
is logged on change (`ATmega non risponde` / `ATmega raggiungibile`) and
exposed in the private cluster `0xFF12` on EP1:

| Attribute | Type | Meaning |
|---|---|---|
| `0x0000` | bool | ATmega reachable (replies to `GET`) |
| `0x0001` | bool | ATmega registered as slave on the P1/P2 bus |
| `0x0002` | uint8 | Last `E,<code>` from the ATmega (0 = none; reset by the next valid state line) |

The bridge reports these attributes itself on every change and on every
keepalive with the coordinator (every 5 min); no reporting configuration
is needed in ZHA. While the ATmega does not reply, the six On/Off
endpoints are rewritten with the last real state (all off if none was ever
received), so Home Assistant never shows a command that did not happen.

### ZHA quirk

`ha/daikin_vam_quirk.py` names the endpoints (Accensione, Velocità alta,
Fresh-up, Modo Auto/Scambio/Bypass, BLE, PROG, RESCRED, LOG) and adds the
diagnostic entities "Collegamento ATmega", "Registrato bus P1P2" and
"Errore ATmega" from the link cluster. PROG, RESCRED and LOG are disabled
by default. Copy it into your custom quirks folder
(`zha: custom_quirks_path:` in `configuration.yaml`) and restart Home
Assistant. Without the quirk the switches work the same with generic names,
and the link state is not shown.

## Maintenance switches

Four virtual switches let you maintain the bridge from Home Assistant,
without removing the wall panel. Each action starts 1.5 s after the
command, so that the Zigbee reply is sent first.

| Switch | EP | What it does |
|---|---|---|
| **BLE** | 10 | Restarts in **Bluetooth update mode**: Zigbee and the ATmega link are not started, only Bluetooth LE for 10 minutes, then the bridge restarts normally. Use it with `tools/ble_update.html`. |
| **PROG** | 11 | Restarts in **serial download mode** (ROM bootloader), ready for `esptool`. |
| **RESCRED** | 12 | **Zigbee factory reset**: erases the Zigbee credentials and restarts; the bridge must be paired again. Same as holding `GPIO9` for 5 s. |
| **LOG** | 13 | **Log over Bluetooth**: ON starts Bluetooth LE next to Zigbee and sends the firmware log to `tools/ble_log.html`; it stays on after restarts until it is switched OFF (or `GPIO9` is pressed briefly, 0.3–5 s). Switching it on or off restarts the bridge. |

After joining, the bridge reports the real state of these switches to Home
Assistant, so they do not stay ON in the UI. On the UART0 console you can
also type `prog` (download mode), `reset` (restart) or `help`.

### Bluetooth web pages

The pages in `tools/` (`ble_update.html` and `ble_log.html`) use
**Web Bluetooth**: open them in **Chrome or Edge** (desktop or Android),
directly from disk or from any web server. Firefox, Safari and Brave do
not support Web Bluetooth.

**Firmware update — `tools/ble_update.html`**

1. Turn on the **BLE** switch in Home Assistant. The LED starts blinking
   slowly; you have 10 minutes.
2. Choose the application image (`build/vmc_esp32.bin`, the same file
   flashed at `0x20000`).
3. Click **Find device and send** and pick `VAM-XXXX` (last 4 digits of
   the MAC).
4. At the end the image is verified and the bridge restarts with the new
   firmware, still paired.

The bridge refuses images of another project (the project name inside the
image must be `vmc_esp32`) and images larger than the OTA partition
(1.5 MB).

**Live log — `tools/ble_log.html`**

1. Turn on the **LOG** switch in Home Assistant (the bridge restarts).
2. Click **Connect device** and pick `VAMLog-XXXX`.
3. Turn **LOG** off when you are done (or press `GPIO9` briefly).

## ATmega serial protocol

This is the ATmega's own serial API — 115200 8N1, line-based. The optional
ESP32-H2 bridge is just one consumer of it; anything that can talk serial
(a Raspberry Pi, an MQTT gateway, ...) can drive the unit the same way.

**Host → ATmega** (commands): `ON` `OFF` `VL` `VH` `MA` `MS` `MB` `FU1` `FU0` `GET`
(`GET` asks for an immediate state resync, e.g. after the host reboots).

**ATmega → host:**
```
S,<onoff>,<mode>,<speed>,<freshup>,<registered>
```
Sent automatically on every real change (commanded from the host, changed on
the physical wall panel, or a change in bus registration), or immediately
in response to `GET`.

```
E,<code>
```
`1` = bus CRC error (rate-limited to 1 per 2s) · `2` = slave channel lost
(master downgraded it, see [protocol.md](protocol.md)) · `3` = no poll from
the master for over 5s (bus/master silence)

## Protocol reverse-engineering — key discoveries

The two findings that unlocked write control on this VAM model:

1. **The master only offers the writable state channel (`0x32`) to a
   genuine second controller**, not to a generic "aux" responder — Daikin
   panels natively support a Master + Slave pair, selected by a physical
   switch on the BRC301B61.
2. **The real slave panel answers the registration poll (`0x30`) only
   once**, then goes silent on every subsequent one. Getting this detail
   wrong (answering every time) gets the controller silently downgraded to
   a read-only channel.

Full byte-level protocol, packet captures, and the list of dead ends that
looked plausible but don't work: see [`protocol.md`](protocol.md).

## Known limitations

- Deployed and running behind the wall panel, but still a short track
  record — not yet validated over months/seasons of operation
- `ESP_ZB_PRIMARY_CHANNEL_MASK` is hard-coded and must be edited per network
  before flashing
- No Zigbee OTA: `esp-zigbee-lib` 2.x is used through its 1.x compatibility
  layer, which does not support the OTA cluster (updates over Bluetooth or
  by cable). Espressif describes that layer as not meant for production
- No bridging to MQTT directly — this targets Zigbee (ZHA/Zigbee2MQTT)
  coordinators; an MQTT bridge would need to sit on the coordinator side

## Credits

- [Arnold-n/P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT) — the library
  and ecosystem this project builds on
- Protocol reverse-engineered and documented independently for the VAM
  ventilation model

## License

This repository's code (firmware) is released under the MIT License unless
noted otherwise. It depends on the P1P2MQTT library
([Arnold-n/P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT)), which is
licensed under CC BY-NC-ND 4.0 — that library is **not** redistributed here
and must be obtained separately.
