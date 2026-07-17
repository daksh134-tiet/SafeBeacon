# Honda Brio 2013 — OBD2 PID Reference

## Protocol
- **Standard:** ISO 15765-4 (CAN 500kbps, 11-bit ID)
- **ELM327 init string:** `ATSP6` (ISO 15765-4 CAN 500kbps, 11-bit)
- **All PIDs below confirmed working** with ELM327 v1.5 on a 2013 Honda Brio (1.2L i-VTEC, MT)

---

## Mode 01 — Live Data PIDs

| PID | Bytes | Parameter | Min | Max | Formula | Unit | Poll rate |
|-----|-------|-----------|-----|-----|---------|------|-----------|
| `01` | 4 | MIL status + DTC count | – | – | Bit 7 of A = MIL; A & 0x7F = DTC count | – | 1s |
| `05` | 1 | Engine coolant temperature | -40 | 215 | `A - 40` | °C | 1s |
| `0B` | 1 | Intake manifold absolute pressure | 0 | 255 | `A` | kPa | 100ms |
| `0C` | 2 | Engine RPM | 0 | 16383.75 | `((A×256)+B) / 4` | rpm | 100ms |
| `0D` | 1 | Vehicle speed | 0 | 255 | `A` | km/h | 100ms |
| `0F` | 1 | Intake air temperature | -40 | 215 | `A - 40` | °C | 1s |
| `11` | 1 | Throttle position | 0 | 100 | `A × 100/255` | % | 100ms |
| `2F` | 1 | Fuel tank level | 0 | 100 | `A × 100/255` | % | 5s |
| `42` | 2 | Control module voltage | 0 | 65.535 | `((A×256)+B) / 1000` | V | 1s |
| `46` | 1 | Ambient air temperature | -40 | 215 | `A - 40` | °C | 5s |
| `5C` | 1 | Engine oil temperature | -40 | 215 | `A - 40` | °C | 5s |
| `03` | 2 | Fuel system status | – | – | A=system 1, B=system 2 | Status | on change |

---

## Mode 03 — Stored DTCs

Request: `03` (no PID byte)

Response format: `43 [count] [P1_H P1_L] [P2_H P2_L] ...`

DTC decode:
```
Byte 1 high nibble → system:
  0 = P (Powertrain)
  1 = C (Chassis)
  2 = B (Body)
  3 = U (Network)

Byte 1 low nibble + byte 2 → fault number
```

Common Honda Brio DTCs:
| Code | Description |
|------|-------------|
| P0420 | Catalyst efficiency below threshold |
| P0171 | System too lean (bank 1) |
| P0300 | Random/multiple cylinder misfire |
| P0113 | IAT sensor circuit high input |
| P0134 | O2 sensor no activity (bank 1, sensor 1) |

---

## Mode 04 — Clear DTCs

Command: `04`

**Use with caution.** Clears all stored DTCs and resets I/M readiness monitors.

---

## PID Support Bitmap (Mode 01, PID 00)

Response from Honda Brio 2013: `41 00 BE 3F A8 13`

Decoding `BE 3F A8 13`:
```
BE = 1011 1110 → PIDs 01,03,04,05,06,07 supported (skipping 02,08)
3F = 0011 1111 → PIDs 09-0E supported
A8 = 1010 1000 → PIDs 11,13,15 supported
13 = 0001 0011 → PIDs 1C,1F,20 supported
```

---

## ELM327 Initialisation (Honda Brio specific)

```
ATZ        → Reset, wait for "ELM327 v1.5"
ATE0       → Echo off
ATL0       → Linefeed off
ATS0       → Spaces off
ATH0       → Headers off
ATAL       → Allow long messages
ATSP6      → Force ISO 15765-4 CAN 500kbps 11-bit (skips auto-detect)
ATAT2      → Adaptive timing mode 2
```

Using `ATSP6` instead of `ATSP0` (auto) speeds up connection from ~4 seconds to ~1 second on the Brio.

---

## Raw Responses

```
Query: 010C          (Engine RPM)
Response: 410C1AF8
  A = 0x1A = 26
  B = 0xF8 = 248
  RPM = (26×256 + 248) / 4 = (6656+248)/4 = 1726 rpm

Query: 010D          (Vehicle Speed)
Response: 410D3C
  A = 0x3C = 60
  Speed = 60 km/h

Query: 0105          (Coolant Temp)
Response: 41055A
  A = 0x5A = 90
  Temp = 90 - 40 = 50°C (cold engine)

Query: 0142          (Control Module Voltage)
Response: 4142382C
  A = 0x38 = 56
  B = 0x2C = 44
  V = (56×256 + 44) / 1000 = 14380/1000 = 14.38V (engine running)
```

---

## Notes on Brio-Specific Behaviour

1. **RPM at idle:** ~700–750 rpm (cold), ~550–600 rpm (warm)
2. **Normal coolant range:** 85–95°C (warm engine, normal operation)
3. **Battery voltage:** 12.4–12.6V (key off), 13.8–14.4V (engine running)
4. **OBD wakes up:** ~2 seconds after ignition ON (not ACC)
5. **MIL (check engine):** Not present on base trim — check PID 0x01 bit 7 instead of the dashboard light
6. **Fuel level accuracy:** Gauge tends to read ~5% higher than actual — normalise with a calibration offset if precision matters

---

*Documented from live testing, January 2025. Brio: 2013, 1.2L i-VTEC, petrol, manual transmission, BS4.*
