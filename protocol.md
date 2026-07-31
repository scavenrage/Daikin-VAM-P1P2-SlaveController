# Protocol details — Daikin VAM slave control via P1/P2

## Background

Daikin systems natively support two controllers on the same P1/P2 bus: a
**Master** and a **Slave**, selected via a physical M/S switch on the
BRC301B61 wall panel (factory default: Master). This is not an edge case —
it's a documented feature for installations with two remote controllers.

This turned out to be the key to controlling the VAM unit: a generic
"aux controller" answering on slot F0 is *not* the same thing as being
recognized as a genuine second controller, and the two get treated very
differently by the master.

## Key discovery #1 — packet type 0x32 requires a real second controller

With only one aux controller on the bus, the master interrogates it using
packet type **0x38**. On this VAM model, 0x38 does not carry a usable
on/off/mode/speed state — every attempt to answer it (mirroring, echoing,
zeroing change-flags) either does nothing or triggers a **UA** error on
the wall panel.

Packet type **0x32** — which does carry the full state and accepts write
commands — is only offered by the master when it recognizes a genuine
second controller. This only happened when a real BRC301B61 (set to Slave)
was present on the bus, which is how the write protocol below was captured.

## Key discovery #2 — the "silence on poll 0x30" detail

Boot / registration sequence, observed on the real slave panel:

```
00F030 (poll)  →  40F0FF   (presence, ONLY on the first poll)
00F034 (poll)  →  40F034   (empty)
00F032 (poll)  →  40F032   (state) — 0x32 channel now active
00F033 (poll)  →  40F033   (filter counter)
00F030 (poll)  →  (no response — silence from here on)
```

The critical detail: the real slave answers `40F0FF` **only on the very
first** `00F030` poll. On every subsequent `00F030` poll, it stays
**completely silent**.

An Arduino firmware that answers `40F030` (empty) on every poll — a
reasonable-looking but incorrect behavior — gets **downgraded** by the
master to the 0x38 channel instead of keeping 0x32 active. Fixing this
single behavior (silence after the first registration poll) was what
unlocked write control.

## State / command packet — `40 F0 32`

Response payload (6 bytes): `[OnOff] 03 [Mode] [Speed] 00 00`

| Byte | Meaning | Values |
|---|---|---|
| 0 | On/Off | `00`=off, `01`=on |
| 1 | fixed | `03` |
| 2 | Mode | `00`=Auto, `01`=Heat-exchange, `02`=Bypass; bit `0x08`=Fresh-up (combinable) |
| 3 | Fan speed | `01`=Low, `05`=High |
| 4–5 | fixed | `00 00` |

Verified states:

| State | `40F032` payload |
|---|---|
| Off | `00 03 00 01 00 00` |
| On, Auto, Low | `01 03 00 01 00 00` |
| On, Auto, High | `01 03 00 05 00 00` |
| On, Heat-exchange, High | `01 03 01 05 00 00` |
| On, Bypass, High | `01 03 02 05 00 00` |
| On, Bypass+Fresh-up, High | `01 03 0A 05 00 00` |

## How commanding works

The slave commands the unit by **transitioning** its `40F032` response from
the current state to the desired one *while being polled* by the master.
The master detects the transition (its own poll counter byte increments)
and drives the unit to the new state accordingly.

This means the slave controller must always start **aligned with the real
state** of the unit — otherwise there's no transition for the master to
detect, and no command happens. In this implementation, the real state is
read continuously from the `4000 40` packet (see below) and mirrored until
a command is issued, at which point the mirroring is paused until the bus
confirms the new state.

## Real-time state readout — `40 00 40`

This packet, emitted by the indoor unit (not an echo of the controller's
own response), is the ground truth for the current state:

| Index | Meaning |
|---|---|
| 3 | On/Off (`00`/`01`) |
| 12 | Mode (`00`/`01`/`02`, bit `0x08`=Fresh-up) |
| 13 | Fan speed (`01`/`05`) |
| 22 | CRC |

## Other F0 packets

- `00F033` → `40F033`: payload `00 00 00 [filterHi] [filterLo]` — echoes the
  filter/maintenance counter sent by the master (`0F0D` typical)
- `00F034` → `40F034`: empty response (header + CRC only)

## Dead ends (do not retry)

- Answering 0x38/0x39 as a generic aux controller → **UA** error
- Answering `40F030` (empty) on every poll instead of staying silent after
  registration → master downgrades the channel to 0x38
- Trying to command via 0x38 with a single controller — never observed a
  real controller doing this; would require guessing an undocumented
  structure
- The P1P2MQTT library **cannot act as a bus master** — it's designed to
  answer polls from an external master, not generate the bus timing itself.
  A firmware attempting to generate master-side polling failed consistently
  (timeouts, malformed packets) regardless of CRC/timing parameter tuning.
- Starting the slave's internal state as a fixed default (e.g. always "on")
  instead of reading the real state first — no transition is seen by the
  master, so no command is recognized.

## CRC

Daikin/HBS CRC: polynomial `0xD9`, feed `0x00`.
