# Hardware (Bill of Materials)

> Covers only what is physically installed and operating. Components already
> purchased but not yet installed appear under [Planned](#planned-not-installed),
> kept separate from the rest — same honesty policy as the README.

## Electrical Box (power panel)

The only enclosure in the system that touches mains voltage. No high voltage enters
any box with a microcontroller.

| Component | Model | Specification |
|---|---|---|
| Breaker | WEG MDWP | 2-pole, Curve C, 25A |
| RCD (residual current device) | WEG RDWS | 2-pole, 25A, 30mA (separate device from the breaker) |
| Mini contactor | WEG CWC07-10-30V26 (part no. 12459272) | 3-pole, 7A (AC-3), 190V 50Hz/220V 60Hz coil, 1 NO auxiliary contact |
| Rotary selector switch | Lukma Electric ZB2-BE101 | 3 positions (Manual/Off/Automatic), 2 independent NO contact pairs |
| Dual relay board | 2x Songle SRD-05VDC-SL-C relay | 1 physical board, 2 integrated circuits (K1+K2 in series, see `docs/PROTOCOL.md`) |
| Power supply | DIN rail, 5V/3A | Powers the relay board logic and, over cable, the Pump box's ESP32 |
| Energy meter | PZEM-004T (Peacefair) | Lives here, not in the Pump box — the measurement side touches mains voltage directly |

## Pump Box

Low-voltage electronics only — no power-side components.

| Component | Model |
|---|---|
| ESP32 | DevKit V1302, 38-pin, external antenna |
| Adapter | Screw-terminal expansion board (replaces direct Dupont connectors) |

### Electrical Box ↔ Pump Box interconnect

Two 6-wire cables connect the two boxes:

| Cable | Wires used | Signals |
|---|---|---|
| Cable 1 | 6 of 6 | 5V, GND, PZEM TX, PZEM RX, K1 (GPIO18), K2 (GPIO19) |
| Cable 2 | 1 of 6 | Logic 3.3V (ESP32 → relay board VCC) — uses Cable 1's GND as reference, no dedicated return of its own |

GND is a single common reference: it terminates at the power supply's ground
(Electrical box), where it's distributed locally to the PZEM's GND and the relay
board's GND, and extends over the cable to the ESP32's GND pin (Pump box).

## Water Box

| Component | Model | Specification |
|---|---|---|
| ESP32 | DevKit V1302, 38-pin, external antenna | |
| Adapter | Screw-terminal expansion board | |
| Power supply | Hi-Link HLK-PM01 | 5V/600mA |
| Ultrasonic sensor | AJ-SR04M-2 | Waterproof — ships from the factory with the Echo divider board (1kΩ+2kΩ) as a single purchasable unit |
| Capacitive sensor (overflow) | XKC-Y26S-V | NPN, 6-36V, wired directly to the ESP32, no intermediate board |
| Resistors (XKC-Y26S-V divider) | 10kΩ + 20kΩ | Discrete, loose — not part of any kit |

## Control Box

| Component | Model | Specification |
|---|---|---|
| ESP32 | 30-pin, generic | Internal antenna — different from the Water/Pump ESP32 |
| Adapter | Screw-terminal expansion board | |
| Power supply | Hi-Link HLK-PM01 | 5V/600mA — same model as the Water box |
| Traffic-light indicator | — | Single component: 3 LEDs (green/yellow/red) + resistors already integrated |
| Individual LEDs | — | 3x: blue, dedicated error red, white — each discrete, with its own resistor |
| Resistors (individual LEDs) | 220Ω | 220Ω–330Ω range also acceptable |
| Buzzer | generic | |
| Physical buttons | generic | 2x |

## Enclosures

The three electronic boxes (Water, Pump, Control) use the same standard: surface-mount
4x4 enclosure, IP66 electrical rating, 110x110x60mm.

## Remote console

Outside the scope of this repository's firmware (own firmware, not included here).

| Component | Model |
|---|---|
| Remote console | M5Stack Cardputer (M5Stack StampS3) — also works fixed in place as a graphical level monitor for the Water box (magnet, fridge door), no hardware change, firmware only |

## Planned (not installed)

Components already purchased for condensation mitigation on the Water box's
ultrasonic sensor, but with no physical implementation yet:

- 2x XKC-Y25-V capacitive sensor (max/min level redundancy)
- DHT22/AM2302 temperature/humidity sensor + SHT31 probe (IP67)
- Resistive heating for the transducer: IRLZ44N MOSFET + resistors + dedicated
  additional 4x4 enclosure, connected by its own 6-wire cable
