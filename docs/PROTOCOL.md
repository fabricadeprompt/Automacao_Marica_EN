# Shared Protocol (ESP-NOW)

> Note: struct field names, constants, and packet type identifiers below (e.g.
> `agua_distancia_cm`, `CMD_LIGA_BOMBA`, `calcular_pct()`) are kept exactly as they
> appear in the source code, which stays in Portuguese in this repository (see the
> note in the main README). Only the descriptions are translated.

All communication between the three nodes runs over **ESP-NOW, fixed channel 2**
(`CANAL_SEGURANCA_PADRAO`, defined in `marica_protocol.h`) — it is not negotiated and
does not scan channels, it's fixed on the radio before initialization.

## Packet flow

```
Water ──PKT_TELEMETRIA_AGUA──▶  Pump  ──PKT_STATUS_COMPLETO──▶  Control ──Wi-Fi──▶ Supabase
                                   ▲                                  │
                                   └──── CMD_LIGA_BOMBA / CMD_DESLIGA_BOMBA
                                          CMD_PING_CONTROLE / CMD_RESET_ERROS
                                          CMD_SET_NIVEIS ──────────────┘

Control ──PKT_STATUS_CARDPUTER (pushed every 10s)──▶ Cardputer (Monitor mode)

Cardputer ──CMD_WEB_SERVER / CMD_OTA / CMD_REBOOT──▶ any box
```

## `PacketStatusCompleto` (Pump → Control)

The densest packet in the system — the Pump box aggregates the telemetry relayed from
the Water box with its own data before pushing it up to Supabase.

| Field | Description |
|---|---|
| `agua_distancia_cm` | Filtered level, relayed from the Water box |
| `agua_erro_sensor` | Physical failure of the ultrasonic sensor, relayed |
| `agua_ladrao_ativo` | Confirmed overflow, relayed |
| `bomba_rele_estado` | Physical state of relay K1 |
| `bomba_erro_bitmask` | Errors that actively block/shut down the pump (timeout, overflow, PZEM failure) |
| `pzem_potencia_w`, `pzem_tensao_v`, `pzem_corrente_a`, `pzem_fp`, `pzem_energia_kwh` | Instantaneous and cumulative readings from the PZEM-004T |
| `agua_offline` | Water→Pump radio silence for more than 60s — distinct from an actual sensor error |
| `bomba_estado_bitmask` | **Informational** states only (quarantine, lockout, forced mode, stuck relay) — never used for safety decisions |
| `bomba_causa_desligamento` | Reason for the last shutdown (manual, tank full, timeout, overflow, etc.) |

**Why two separate error/state bitmasks?** `bomba_erro_bitmask` is read by real safety
decisions (interlocking and automatic shutdown); `bomba_estado_bitmask` is purely
informational for the dashboard. Keeping them separate prevents a field added only for
display from accidentally influencing a safety decision.

## `PacketStatusCardputer` (Control → Cardputer)

Periodic push (every 10s), not on demand — same pattern as the `CMD_PING_CONTROLE`
keep-alive already used with the Pump box. Feeds the Cardputer's Monitor mode (an
always-on idle screen, mounted by magnet on a fridge door, showing the reservoir
level).

| Field | Description |
|---|---|
| `nivel_pct` | 0-100, already calculated by `calcular_pct()` on the Control box — single source of truth, the same one used by the local web server. The Cardputer does not reimplement the distance→percentage conversion |
| `nivel_distancia_cm` | Raw distance (cm), only to display "xx cm" alongside the percentage |
| `bomba_ligada` | Relayed from the Control box |
| `modo_atual` | `MODO_AUTOMATICO` / `MODO_SEMIAUTOMATICO` |
| `agua_erro_sensor` | Relayed — Water box sensor with a physical fault |
| `agua_offline` | Relayed — Water→Pump radio silence |
| `bomba_offline` | From `bomba_esta_offline()` — the same single source already used elsewhere on the Control box. If `true`, the rest of the packet is cached, not current, data — the Cardputer treats it as unavailable, never displays it as if it were current |

**Concurrency:** all 7 fields arrive in a single packet, but they're written by the
ESP-NOW receive task and read by the Cardputer's `loop()` — two different contexts.
The Cardputer uses a critical section (`portMUX`) on both the write and the read side,
to guarantee the 7 fields are always read as one consistent set, never a mix of a new
packet with an old one.

## Protocol evolution rule

The struct is shared by firmware that isn't always updated at the same time — each box
is flashed physically in the field, not via simultaneous OTA. Extending it follows two
rules:

1. **New fields always go at the end of the struct** — a receiver running older
   firmware, upon receiving a larger packet, still passes the minimum-size check and
   safely ignores the extra field, instead of misinterpreting the wrong bytes.
2. **Coordinated flashing order only when the struct's *size* changes** — whoever
   *emits* the packet must be updated first. A new bit inside a bitmask field that
   already existed doesn't require this coordination.
