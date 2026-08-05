# Daikin VAM — P1P2 Slave Controller with Zigbee bridge

Full read **and write** control of a Daikin VAM heat-recovery ventilation
unit over the P1/P2 bus, exposed to Home Assistant (or any Zigbee
coordinator) via a dual-MCU board: an ATmega328 registered as a genuine
BRC301B61-compatible **slave controller** on the P1/P2 bus, bridged over a
serial link to an ESP32-H2 running native Zigbee.

> **Status: prototype.** Bring-up and bench-tested; not yet deployed
> long-term behind the wall panel. See [Known limitations](#known-limitations).

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
  ATmega328  ──────UART (115200, S,/E,/commands)──────  ESP32-H2
 (P1/P2 slave                                          (Zigbee router,
  controller,                                            6 On/Off endpoints
  Atmega328/)                                             + OTA, ESP32-H2/)
                                                              │
                                                        Zigbee coordinator
                                                        (Home Assistant / ZHA)
```

Both MCUs live on one custom board (`Hardware/Atmega+ESP32 Schematic.epro2` +
`Atmega+ESP32 PCB.epro2`), connected to a separate small board carrying the
MAX22088 P1/P2 transceiver via a 7-pin header.

## Repository layout

| Path | Contents |
|---|---|
| `Atmega328/` | Arduino sketch for the ATmega328 — P1/P2 bus slave-controller logic |
| `ESP32-H2/` | ESP-IDF project for the ESP32-H2 — Zigbee bridge to the ATmega |
| `Hardware/` | EasyEDA Pro schematics/PCB for both boards (main board + P1/P2 transceiver board) |
| `protocol.md` | Full P1/P2 protocol reverse-engineering write-up (packet captures, byte-level mapping) |

## What this does

- Registers on the P1/P2 bus as a real slave controller (not a generic aux
  responder) alongside the existing wall panel (master)
- Reads the unit's real-time state directly from the bus — no manual sync needed
- Commands on/off, fan speed (Low/High), mode (Auto/Heat-exchange/Bypass),
  and fresh-up — each verified with closed-loop confirmation from the bus
- Bridges state and commands to the ESP32-H2 over a compact serial protocol
- Exposes 6 Zigbee On/Off endpoints to Home Assistant, reflecting the
  **real, bus-confirmed state** — never an optimistic guess at what a
  command is expected to do
- Zigbee OTA client on the ESP32-H2 side, so the bridge can be updated
  in place once it's installed behind the wall panel

## Hardware

- Custom PCB: **ATmega328P + ESP32-H2-MINI-1-N4** on one board (`Hardware/Atmega+ESP32*`)
- Separate small board: **MAX22088** P1/P2 bus transceiver (`Hardware/P1P2*`),
  connected to the main board via a 7-pin header (`RST`/`DOUT`/`DIN`/`GND`/`5V`)
- ATmega P1/P2 pins (unchanged from the original Arduino Uno prototype):
  `RST → D7`, `DOUT → D8`, `DIN → D9`
- ATmega ↔ ESP32-H2 serial link: ATmega `D0`/`D1` ↔ ESP32-H2 `IO5`/`IO4`
  (resistor divider on the ATmega→ESP32 direction, 5V logic down to 3.3V)
- ESP32-H2: `GPIO9` = factory-reset button (hold 5s to leave the Zigbee
  network), `GPIO22` = status LED
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

## Building — ESP32-H2 (Zigbee bridge)

Requires ESP-IDF v5.1+ with the `esp32h2` target.

```bash
cd ESP32-H2
idf.py set-target esp32h2
idf.py build
idf.py -p <PORT> flash monitor
```

The first build downloads `espressif/esp-zigbee-lib` and
`espressif/esp-zboss-lib` automatically (see `main/idf_component.yml`);
this needs an internet connection once.

**Before flashing for the first time, check:**
- `ESP_ZB_PRIMARY_CHANNEL_MASK` in `main/main.c` is hard-coded to Zigbee
  channel 25 — change it to match your own network
- The partition table (`partitions.csv`) uses a dual-OTA layout
  (`ota_0`/`ota_1`); if you're re-flashing a board that previously had a
  different (non-OTA) partition table, do a full `idf.py erase-flash`
  first, otherwise the old table in flash won't match the new one

## Zigbee endpoints (Home Assistant / ZHA)

| EP | Cluster | Function |
|---|---|---|
| 1 | On/Off + OTA client | Power (on/off) |
| 2 | On/Off | Fan speed — Off = Low, On = High |
| 3 | On/Off | Fresh-up |
| 4 | On/Off | Mode: Auto |
| 5 | On/Off | Mode: Heat-exchange |
| 6 | On/Off | Mode: Bypass |

Endpoints 4–6 are mutually exclusive: only one is ever `ON`, reflecting
whichever mode is actually active on the bus. Switching one on sends the
corresponding command to the ATmega; turning one off without turning
another on has no direct equivalent on the unit, so the switch will simply
flip back to `ON` once the next real-state update arrives — the firmware
never assumes a command succeeded, it only reports what the ATmega actually
confirms.

## ATmega ↔ ESP32-H2 serial protocol

115200 8N1, line-based, on the dedicated UART link described above.

**ESP32 → ATmega** (commands): `ON` `OFF` `VL` `VH` `MA` `MS` `MB` `FU1` `FU0` `GET`
(`GET` asks for an immediate state resync, used by the ESP32 after a reboot).

**ATmega → ESP32:**
```
S,<onoff>,<mode>,<speed>,<freshup>,<registered>
```
Sent automatically on every real change (commanded from Zigbee, changed on
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

- Prototype stage: bench-validated, not yet running long-term in the final
  wall-mounted installation
- `ESP_ZB_PRIMARY_CHANNEL_MASK` is hard-coded and must be edited per network
  before flashing
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
