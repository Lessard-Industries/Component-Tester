# Component Tester
### An Arduino Nano-based automatic electronic component identifier

**© Lessard Industries**

---

## Overview

The Component Tester automatically identifies and measures common electronic components using a three-point test circuit. Insert an unknown component into the test terminals, press the TEST button, and the device identifies the component type, measures its value, and reports which pins are which.

**Supported components:**
- Resistors (with value measurement)
- Capacitors (with value measurement)
- Diodes (standard silicon)
- LEDs (with color detection: Red / Yellow / Green / Blue/White)
- NPN and PNP BJT transistors
- Multi-junction devices (FETs, unknown)

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Nano |
| Display | 20×4 I2C LCD (address `0x23`) |
| Test network | 3× 680Ω (pullup) + 3× 470kΩ (pulldown) |
| Power | 2S 18650 Li-ion battery pack |
| UI | Rotary encoder + push button |
| Audio | Active/passive buzzer |
| Battery monitor / Voltage meter | Resistor divider (100kΩ / 22kΩ) on A7 |

### Wiring Summary

```
        +5V
         |
   [680Ω] [680Ω] [680Ω]
     |      |      |
    TP1    TP2    TP3   ← Test terminals
     |      |      |
    A0     A1     A2   ← Arduino analog pins
     |      |      |
  [470kΩ][470kΩ][470kΩ]
     |      |      |
    GND    GND    GND
```

A full wiring guide will be added to the repository in a future update.

---

## Pin Assignments

| Arduino Pin | Function |
|---|---|
| A0 | Test Point 1 (TP1) |
| A1 | Test Point 2 (TP2) |
| A2 | Test Point 3 (TP3) |
| A3 | Internal battery monitor (POWER_PIN) |
| A4 | LCD SDA |
| A5 | LCD SCL |
| A6 | Continuity sense |
| A7 | Battery monitor / Voltage meter input |
| D2 | TEST button |
| D3 | Buzzer |
| D4 | Encoder CLK |
| D5 | Encoder DT |
| D6 | Encoder SW |
| D7 | Continuity drive |

---

## Modes

The rotary encoder cycles through four operating modes:

**1. Component Test**
Insert a component across any two (or three) test terminals and press TEST. The device runs a full measurement sweep and identifies the component.

**2. Continuity Test**
Touch the two probe leads together to test for continuity. A bar graph shows connection strength and the buzzer sounds when continuity is detected.

**3. Battery Voltage**
Displays the voltage and estimated charge percentage of the connected battery pack. Automatically detects battery type (AA/AAA, Li-Ion 1S, 9V, 12V SLA).

**4. Voltage Meter**
A fast, general-purpose DC voltmeter using the onboard 100kΩ/22kΩ divider on A7. Measures 0–28V DC with auto-range display — millivolt resolution below 1V, three decimal places above. Update rate ~120ms with no smoothing for immediate response.

---

## Calibration

Before building, measure your actual resistor values with a multimeter and update these constants in the sketch:

```cpp
#define R_LOW   670.0      // Your measured 680Ω resistor value
#define R_HIGH  465000.0   // Your measured 470kΩ resistor value
#define VCC     5.05       // Your actual supply voltage
```

Accurate values here directly affect measurement accuracy.

---

## Measurement Accuracy

| Component | Range | Notes |
|---|---|---|
| Resistors | ~10Ω – 200kΩ | Best accuracy in mid-range |
| Capacitors | ~0.5µF – 1000µF | RC time-constant method |
| Diode Vf | 0.4V – 3.8V | ±20mV typical |
| LED color | — | Detected by Vf threshold |
| DC Voltage | 0 – 28V | Via onboard divider on A7 |

> **Known limitation:** Resistors above ~200kΩ have reduced accuracy due to ADC resolution constraints when measuring voltages near the supply rail. This is a hardware limitation of the Arduino's 10-bit ADC.

---

## Version History

| Version | Key Changes |
|---|---|
| v4.2 | Initial working version, dual-range R measurement |
| v4.4 | Fixed active pin driving during measurement |
| v4.5 | Improved high-R measurement, dual-path correction |
| v5.0 | RAM-safe output (no `String` class), NPN/PNP detection, improved battery analyzer, bargraph continuity display |
| v5.5 | Added Voltage Meter mode (0–28V, auto-range), redesigned 4-mode menu layout |

---

## Development Environment

The project uses **PlatformIO** (VS Code extension) for building and uploading.

### Dependencies
- `marcoschwartz/LiquidCrystal_I2C` (or equivalent I2C LCD library)
- Arduino AVR core

---

## License

Private project — © Lessard Industries. All rights reserved.
