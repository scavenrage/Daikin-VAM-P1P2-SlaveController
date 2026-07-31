# Daikin VAM — P1P2 Slave Controller

Full read **and write** control of a Daikin VAM heat-recovery ventilation unit
over the P1/P2 bus, using an Arduino Uno registered as a genuine
BRC301B61-compatible **slave controller**.

Built on top of [Arnold-n/P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT)
(P1P2MQTT library, CC BY-NC-ND 4.0). VAM units are currently read-only in
P1P2MQTT upstream; this project adds working write support for on/off,
fan speed, mode, and fresh-up.

## What this does

- Registers on the P1/P2 bus as a real slave controller (not a generic aux
  responder) alongside the existing wall panel (master)
- Reads the unit's real-time state directly from the bus (no manual sync needed)
- Commands on/off, fan speed (Low/High), mode (Auto/Heat-exchange/Bypass),
  and fresh-up — each verified with closed-loop confirmation from the bus
- Exposes simple serial commands, ready to bridge to MQTT / Home Assistant

## Hardware

- Arduino Uno
- P1/P2 interface (MAX22088 or equivalent — see P1P2MQTT project for schematics)
- Wiring: `DOUT → pin 8`, `DIN → pin 9`, `RST → pin 7`
- Bus: 9600 baud · Serial monitor: 115200 baud

**Bus layout:** wall panel (Master) + this Arduino (Slave). No other aux
controller should be on the bus at the same time.

## Serial commands

| Command | Action |
|---|---|
| `X1` / `X0` | Enable / disable bus transmission |
| `R1` / `R0` | Register as slave / stop |
| `ON` / `OFF` | Turn the unit on / off |
| `VL` / `VH` | Fan speed Low / High |
| `MA` / `MS` / `MB` | Mode Auto / Heat-exchange / Bypass |
| `FU1` / `FU0` | Fresh-up on / off |
| `ST` | Print current state |
| `V1` / `V0` | Raw bus log on / off |

**Startup sequence:** `X1` → `R1` → wait for "Registered as slave" → commands.
State is auto-read from the bus, no manual initialization needed.

## Protocol summary

See [`docs/protocol.md`](docs/protocol.md) for the full reverse-engineering
write-up (packet captures, byte-level mapping, and the key discovery that
unlocks write access on VAM units).

## Credits

- [Arnold-n/P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT) — the library and
  ecosystem this project builds on
- Protocol reverse-engineered and documented independently for the VAM
  ventilation model

## License

This repository's code (firmware) is released under the MIT License unless
noted otherwise. It depends on the P1P2MQTT library
([Arnold-n/P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT)), which is licensed
under CC BY-NC-ND 4.0 — that library is **not** redistributed here and must
be obtained separately.
