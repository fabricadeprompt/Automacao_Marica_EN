# Maricá Automation

Automated, distributed monitoring and control system for a residential water
reservoir (water tank + recirculation pump), built with 3 ESP32 microcontrollers
communicating over ESP-NOW.

**Live dashboard:** https://fabricadeprompt.github.io/Marica-Dashboard/

---

## Architecture

```
Water Box   ──ESP-NOW──▶  Pump Box     ──ESP-NOW──▶  Control Box    ──Wi-Fi──▶  Supabase
(level)                        │                      (LED, dashboard,           (telemetry)
                                │ cable (power)             web server)
                                ▼
                          Electrical Box
                    (breaker, RCD, contactor,
                       relays, energy meter)
```

Four physically separate enclosures: Water, Pump, Control, and Electrical. Three of
them carry ESP32 microcontrollers and exchange data over radio (ESP-NOW); the
Electrical box is a power panel, connected to the Pump box by two 6-wire cables — no
mains voltage ever enters any electronic enclosure. Each ESP32 box runs its own local
safety logic — if one goes down, the others keep operating on valid data (never
silently "stuck": each box detects the others' radio silence and flags it explicitly,
instead of assuming the last known value is still current).

| Box | Function | Main hardware |
|---|---|---|
| Water | Measures reservoir level (ultrasonic) | ESP32 DevKit V1302 |
| Pump | Switches the pump on/off, reads the energy meter, runs the safety self-test | ESP32 DevKit V1302 — electronics only, no power-side components |
| Electrical | Power panel: breaker, RCD, contactor, relays, energy meter | No microcontroller — connected to the Pump box by 2 six-wire cables |
| Control | Aggregates telemetry, signals errors (LED), serves the local dashboard, pushes data to Supabase | ESP32, 30-pin |

An M5Stack Cardputer acts as a remote console: putting any of the three ESP32 boxes
into OTA mode, or opening the Control box's web server, over ESP-NOW. It also doubles
as a graphical level monitor for the Water box, which can be mounted — for example —
on a fridge door using the Cardputer's own built-in magnet.

## Engineering highlights

- **Stuck-relay self-test:** after every pump shutdown, the Pump box's microcontroller isolates each relay individually and checks, via the energy meter, whether power drops to ~0 — if it doesn't, it flags a fault on the relay board without blocking normal operation (the physical defect still exists until the board is replaced; the firmware just makes sure it never stays invisible).
- **External review before field deployment:** every firmware change was validated by two different AIs before being flashed.

## Current state / known limitations

This project is under active development — not every box is at the same level of maturity:

- **Pump and Control boxes:** relay redundancy, safety self-test, and unified error signaling are implemented and reviewed. Awaiting field flashing.
- **Water box:** still **without** the hardware redundancy the Pump box received, and without condensation mitigation installed on the ultrasonic sensor (root-cause hypothesis documented internally; components for the resistive heating fix already purchased, not yet physically installed). A sensor lockup caused by condensation has already happened in the field — hardware mitigation is the real next step for this box, not a formality.
- **Redundant level sensors:** architecture decision closed, acquisition/physical installation still pending.

If something here looks unfinished, it's because it is — the intent is to document the real engineering process, not a finished product.

## Technical documentation

Details that don't fit in a lean README live in `docs/`:

- [`docs/HARDWARE.md`](docs/HARDWARE.md) — full component list, per box, with exact model numbers
- [`docs/PINOUT.md`](docs/PINOUT.md) — GPIO pin map
- [`docs/PROTOCOL.md`](docs/PROTOCOL.md) — shared ESP-NOW protocol between boxes
- [`docs/BUILD.md`](docs/BUILD.md) — toolchain, libraries, and flashing steps

## Quick setup

This repository **contains no credentials**. Before compiling:

```
cp secrets.h.example secrets.h
```

Fill in `secrets.h` with your Wi-Fi network and (optionally) your Supabase project — the firmware automatically detects if Supabase isn't configured and disables telemetry sending, no need to comment out code. `secrets.h` is already in `.gitignore`.

Full step-by-step (MACs, IPs, flashing order) in [`docs/BUILD.md`](docs/BUILD.md).

## Structure

```
main_agua.cpp          — Water box firmware
main_bomba.cpp         — Pump box firmware
main_controle.cpp      — Control box firmware
marica_protocol.h      — shared protocol (structs, enums, MACs — shared by all 3 boxes)
secrets.h.example      — credentials template (copy to secrets.h)
.gitignore
docs/
  HARDWARE.md          — components per box
  PINOUT.md            — GPIO pin map
  PROTOCOL.md          — shared protocol
  BUILD.md             — build and flash
```

> Note: source file names (`main_agua.cpp`, `main_bomba.cpp`, `main_controle.cpp`) and
> all code — comments, function and variable names — stay in Portuguese in this
> repository. This English repo is a translated snapshot of the documentation only;
> the firmware itself was not translated (renaming thousands of identifiers would mean
> retesting the whole safety logic from scratch, not a text translation). See the
> Portuguese repository for the actively maintained firmware:
> https://github.com/fabricadeprompt/Automacao-Marica

## License

MIT — see `LICENSE`.
