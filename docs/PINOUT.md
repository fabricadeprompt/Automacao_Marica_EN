# Pin Map (GPIO)

> Extracted directly from the firmware published in this repository — not a document
> maintained separately by hand. In case of any discrepancy, the source code is the
> source of truth.

## Water Box (`main_agua.cpp`)

| GPIO | Function | Electrical detail |
|---|---|---|
| 17 | `PIN_TRIG` — trigger for the AJ-SR04M-2 ultrasonic sensor | Direct connection (yellow wire) |
| 16 | `PIN_ECHO` — echo for the AJ-SR04M-2 ultrasonic sensor | Voltage divider R1=1kΩ + R2=2kΩ (5V→3.33V), green wire |
| 18 | `PIN_LADRAO` — output of the XKC-Y26S-V capacitive sensor (overflow) | Voltage divider to 3.3V; NPN output, `LOW` = overflow active |

## Pump Box (`main_bomba.cpp`)

> The Pump box contains only the ESP32 — no power-side components. The devices these
> pins control (relay board, PZEM-004T) live physically in the **Electrical Box**,
> connected by two multi-core cables (see `docs/HARDWARE.md`). The pins below belong to
> the Pump box's ESP32; the other end of each one is on the far side of the cable.

| GPIO | Function | Electrical detail |
|---|---|---|
| 18 | `GPIO_K1` — drives relay K1 (dual relay board, in the Electrical Box) | Dual relay in series with K2 on the contactor coil — both must close to energize the pump |
| 19 | `GPIO_K2` — drives relay K2 (pre-existing, same board) | In series with K1 — either one opening de-energizes the pump |
| 16 | `GPIO_PZEM_RX` — ESP32 RX, receives TX from the PZEM-004T (in the Electrical Box) | Voltage divider R1=10kΩ + R2=20kΩ (5V→3.33V) |
| 17 | `GPIO_PZEM_TX` — ESP32 TX, sends to the PZEM-004T's RX (Electrical Box) | Direct connection, no divider (the PZEM's RX already tolerates 3.3V) |

## Control Box (`main_controle.cpp`)

| GPIO | Function |
|---|---|
| 27 | Green LED (traffic-light indicator) |
| 26 | Yellow LED (traffic-light indicator) |
| 25 | Red LED (traffic-light indicator) |
| 33 | Blue LED (auxiliary indicator — currently no active function) |
| 13 | Dedicated error LED (red, unified — see `docs/PROTOCOL.md`) |
| 32 | White LED (automatic-mode indicator / Wi-Fi cycle) |
| 15 | Buzzer |
| 14 | Button 1 — short press turns the pump on, 3s hold opens the local web server |
| 12 | Button 2 — emergency stop |

## Pending (Water Box — architecture decided, not yet implemented in firmware)

- 2nd **XKC-Y25-V** unit (minimum level), added to the 1st already on hand (maximum
  level) — level redundancy against a single-point sensor. No GPIO defined in code
  yet: physical installation depends on a field visit.
- 2nd **XKC-Y26S-V** unit on the overflow pipe — cross-validation against false
  positives from residual moisture. Same situation: purchased, installation/GPIO
  pending.
