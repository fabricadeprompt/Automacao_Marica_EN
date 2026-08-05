# Maricá Automation

Automated, distributed monitoring and control system for a residential water
reservoir (water tank + recirculation pump), built with 3 ESP32 microcontrollers
communicating over ESP-NOW.

Operating on a custom ESP-NOW network layer, the system bypasses the unreliability of
standard home Wi-Fi routers for critical M2M control. Wi-Fi is strictly enabled
on-demand for two distinct external tasks: pushing batched telemetry to a cloud
database (which feeds an external public dashboard) and hosting an on-device local
web server for parameter configuration and manual overrides.

**Live dashboard:** [https://fabricadeprompt.github.io/Marica-Dashboard/](https://fabricadeprompt.github.io/Marica-Dashboard/)

---

## Architecture

```text
Water Box  ──ESP-NOW──▶  Pump Box     ──ESP-NOW──▶  Control Box  ──Wi-Fi──▶  Supabase
(level)                        │                     (LED, local web            (telemetry)
                                │ cable (power)        server, presence
                                ▼                      ping)
                          Electrical Box                    │
                    (breaker, RCD, contactor,               │ ESP-NOW (status push)
                       relays, energy meter)                ▼
                                                       M5Stack Cardputer
                                                    (monitor mode / OTA trigger)
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
| **Water** | Measures reservoir level and detects physical overflow (*ladrão*) | ESP32 DevKit V1302, AJ-SR04M-2, XKC-Y26S-V |
| **Pump** | Switches the pump on/off, reads the energy meter, runs the safety self-test | ESP32 DevKit V1302 — electronics only, no power-side components |
| **Electrical** | Power panel: breaker, RCD, contactor, relays, energy meter | WEG breaker, WEG RCD, WEG CWC07 contactor, 5V dual-relay board, PZEM-004T |
| **Control** | Aggregates telemetry, signals errors (LED), serves the local web server, detects smartphone presence, pushes data to Supabase | ESP32, 30-pin |
| **Cardputer** | Magnetically-mounted graphical level monitor; remote OTA/web-server trigger | M5Stack Cardputer |

## Engineering highlights (fail-safe design)

- **Dual-relay safety topology & self-test:** the pump's contactor coil is driven by two independent relays wired in series. To mitigate catastrophic "welded contact" failures, the firmware performs an automated post-operation isolation test, leveraging the energy meter to check if either relay is stuck closed after every cycle, without blocking normal operation.
- **Autonomous veto power:** the Pump box acts as the ultimate gatekeeper. Even if the Control box sends a "start" command, the pump evaluates its own hardware interlocks: absolute operation timeouts, zero-level lockouts, and a dead-man's switch that cuts power if radio telemetry from the water tank drops for more than 60 seconds.
- **Smart presence detection:** the Control box uses non-blocking TCP socket pings to look for registered smartphones on the local network, automatically transitioning the system to "Auto Mode" only when someone is physically present at the property.
- **External review before field deployment:** every firmware change and architectural decision was scrutinized and validated by two different AIs, acting as senior reviewers, before being flashed to the microcontrollers.

## Current state & next steps

This project is under active development — not every box is at the same level of maturity:

- **Pump and Control boxes:** relay redundancy, safety self-test, and unified error signaling are implemented, reviewed, and already running on the currently flashed firmware.
- **Water box:** consistent condensation-driven echo doubling is now compensated in software, through an acoustic multipath compensation algorithm (`modo_reflexao`); instability beyond that threshold still falls back to an absolute timeout and a fail-safe flag, rather than reporting an unreliable reading.
- **Sensor redundancy still pending:** the overflow pipe (*ladrão*) currently relies on a single XKC-Y26S-V sensor as the last physical fail-safe, with no backup unit installed yet. Two XKC-Y25-V capacitive sensors, meant as absolute low/high level failsafes, have also been sourced but not yet installed. Both are planned upgrades, not yet executed.

If something here looks unfinished, it's because it is — the intent is to document the real engineering process, not a finished product.

## Technical documentation

Details that don't fit in a lean README live in `docs/`:

- [`docs/HARDWARE.md`](docs/HARDWARE.md) — full component list, per box, with exact model numbers
- [`docs/PINOUT.md`](docs/PINOUT.md) — GPIO pin map
- [`docs/PROTOCOL.md`](docs/PROTOCOL.md) — shared ESP-NOW protocol between boxes
- [`docs/BUILD.md`](docs/BUILD.md) — toolchain, libraries, and flashing steps

## Quick setup

This repository **contains no credentials**. Before compiling:

```bash
cp secrets.h.example secrets.h
```

Fill in `secrets.h` with your Wi-Fi network and (optionally) your Supabase project — the firmware automatically detects if Supabase isn't configured and disables telemetry sending, no need to comment out code. `secrets.h` is already in `.gitignore`.

Full step-by-step (MACs, IPs, flashing order) in [`docs/BUILD.md`](docs/BUILD.md).

## Structure

```text
main_agua.cpp          — Water box firmware
main_bomba.cpp         — Pump box firmware
main_controle.cpp      — Control box firmware
main_cardputer.cpp     — M5Stack Cardputer firmware
marica_protocol.h      — shared protocol (structs, enums, MACs — shared by all boxes)
secrets.h.example      — credentials template (copy to secrets.h)
.gitignore
docs/
  HARDWARE.md          — components per box
  PINOUT.md            — GPIO pin map
  PROTOCOL.md          — shared protocol
  BUILD.md             — build and flash
```

> **Note:** source file names (`main_agua.cpp`, `main_bomba.cpp`, `main_controle.cpp`,
> `main_cardputer.cpp`) and all code — comments, function and variable names — stay in
> Portuguese in this repository. This English repo is a translated snapshot of the
> documentation only; the firmware itself was not translated (renaming thousands of
> identifiers would mean retesting the whole safety logic from scratch, not a text
> translation). See the Portuguese repository for the actively maintained firmware:
> [https://github.com/fabricadeprompt/Automacao-Marica](https://github.com/fabricadeprompt/Automacao-Marica)

## License

MIT — see `LICENSE`.
