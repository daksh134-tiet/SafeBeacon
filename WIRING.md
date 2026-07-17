# SafeBeacon — Hardware Wiring Guide
## Honda Brio 2013 Installation

---

## Complete Pin Map

```
┌─────────────────────────────────────────────────────┐
│                   ESP32-WROOM-32                     │
│                                                       │
│  GPIO 0  ←── SOS Button (INPUT_PULLUP → GND)        │
│  GPIO 2  ───► RGB LED Red                            │
│  GPIO 4  ───► SIM7600 PWRKEY                        │
│  GPIO 5  ───► SIM7600 RST                           │
│  GPIO 16 ←── SIM7600 TX  (UART2 RX)                │
│  GPIO 17 ───► SIM7600 RX  (UART2 TX)               │
│  GPIO 19 ───► Passive Buzzer (+)                    │
│  GPIO 21 ←──► MPU-6050 SDA  (I2C)                  │
│  GPIO 22 ───► MPU-6050 SCL  (I2C)                  │
│  GPIO 25 ───► RGB LED Red                           │
│  GPIO 26 ←── ELM327 TX  (UART1 RX)                 │
│  GPIO 27 ───► ELM327 RX  (UART1 TX)                │
│  GPIO 32 ───► RGB LED Blue                          │
│  GPIO 33 ───► RGB LED Green                         │
│  GPIO 34 ←── Voltage Divider output (ADC, input only)│
│  3V3     ───► MPU-6050 VCC                          │
│  GND     ───► System Ground (all modules)           │
│  5V (VIN)←── LM2596 5V output                      │
└─────────────────────────────────────────────────────┘
```

---

## Power System

```
OBD2 Pin 16 (B+ / 12V unswitched)
    │
    ├──► 1A inline blade fuse
    │
    ▼
LM2596 Buck Converter
    ├── VIN: 12V from OBD
    ├── VOUT: 5V (adjust potentiometer)
    │       │
    │       ├──► ESP32 VIN (5V rail)
    │       └──► SIM7600 VIN (5V, via 1000µF cap to GND)
    │                  ↑
    │           IMPORTANT: Add 1000µF electrolytic
    │           capacitor here. SIM7600 draws 2A
    │           spikes during 4G registration.
    │           Without this cap, ESP32 will reset.
    │
    └──► 3.3V LDO (AMS1117-3.3 or from ESP32 3V3 pin)
             │
             └──► MPU-6050 VCC, ELM327 VCC (3.3V)

OBD2 Pin 4+5 (Chassis GND) ──► System GND
```

---

## 12V Battery Monitor (Voltage Divider)

```
OBD2 Pin 16 (+12V)
    │
   [R1 = 100kΩ]
    │
    ├──────────────────► ESP32 GPIO 34 (ADC)
    │
   [R2 = 10kΩ]
    │
   GND

Voltage formula:
  V_adc  = V_batt × R2 / (R1 + R2)
         = V_batt × 10k / 110k
         = V_batt × 0.0909

  V_batt = V_adc × 11.0

At 12.6V: V_adc = 1.145V → ADC reads ~1424/4095
At 14.4V: V_adc = 1.309V → ADC reads ~1626/4095
```

---

## OBD2 Connector Pinout (Honda Brio 2013)

```
OBD2 16-pin DLC (under dashboard, driver side)

  ┌──┬──┬──┬──┬──┬──┬──┬──┐
  │ 1│ 2│ 3│ 4│ 5│ 6│ 7│ 8│
  └──┴──┴──┴──┴──┴──┴──┴──┘
  ┌──┬──┬──┬──┬──┬──┬──┬──┐
  │ 9│10│11│12│13│14│15│16│
  └──┴──┴──┴──┴──┴──┴──┴──┘

Pin  4  → Chassis GND          ──► System GND
Pin  5  → Signal GND           ──► System GND
Pin  6  → CAN High (ISO 15765) ──► ELM327 CAN H
Pin 14  → CAN Low  (ISO 15765) ──► ELM327 CAN L
Pin 16  → Battery + (12V)      ──► LM2596 VIN
```

---

## MPU-6050 Mounting (Critical)

**Mount orientation for Honda Brio:**
- X-axis: along vehicle length (forward/backward)
- Y-axis: across vehicle width (left/right)
- Z-axis: vertical (up/down)

**Physical placement:**
- Mount behind dashboard on a flat surface
- Use double-sided foam tape to reduce vibration
- Avoid mounting near subwoofer or engine vibration paths
- Keep wires short and secured (loose wires = false triggers)

```
              FORWARD →
           ┌─────────────┐
    LEFT ← │   MPU-6050  │ → RIGHT
           │             │
           │    X ──►    │
           │    │        │
           │    Y (left) │
           │    │        │
           │    Z (up) ↑ │
           └─────────────┘
```

---

## Installation Checklist (Honda Brio 2013)

1. [ ] Locate OBD2 port (under dashboard, right of steering column)
2. [ ] Plug OBD2 male connector — snug fit, slight click
3. [ ] Test with multimeter: Pin 16 to Pin 4/5 = 12V ✓
4. [ ] Confirm LM2596 output = 5.0V before connecting ESP32
5. [ ] Add 1000µF cap across SIM7600 power pins
6. [ ] Mount MPU-6050 horizontally with Z-axis pointing up
7. [ ] Thread SOS button cable to within reach of driver
8. [ ] Secure all cables with zip ties (vibration kills connections)
9. [ ] Route antenna wire away from ignition cables
10. [ ] Test: turn ignition on → device LEDs show white flash → green = OK

---

## Estimated Build Time

| Task | Time |
|------|------|
| Component procurement (Robu.in) | 2–3 days delivery |
| Breadboard prototype + wiring | 4–6 hours |
| Firmware flash + initial test | 1–2 hours |
| OBD2 PID verification on Brio | 1 hour |
| Enclosure + final install | 2 hours |
| **Total** | **~1 weekend** |
