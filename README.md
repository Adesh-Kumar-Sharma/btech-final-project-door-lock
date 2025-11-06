# Bluetooth-controlled Automatic Door Lock with Signal-Noise Reduction (B.Tech Final Project)

**Project type:** B.Tech final-year project (Group of 4)  
**Role (Adesh Kumar Sharma):** Hardware design, embedded firmware, signal-processing algorithm, testing & documentation.  
**Other contributors:** Adarsh Mishra (Android app & authentication), Ajay Sahu (Power electronics & PCB), Ajay Singh Parihar (UI testing & integration).

---

## Project summary

A Bluetooth-controlled automatic door-lock system implemented using an Arduino Uno (ATmega328P), an HC-05 Bluetooth module, an electromechanical latch/actuator, relay driver, buzzer, and a compact SMPS power supply. The system implements signal-noise reduction and pattern-based validation to reduce false activations caused by noisy wireless environments. An Android app with authentication (PIN) was developed to lock/unlock the door; the system also supports auto-lock on out-of-range detection (RSSI/connection loss).

---

## Files included in this repo

- `README_Bluetooth_DoorLock_Project.md` (this file)
- `Report_Bluetooth_DoorLock_Project.pdf` (project report)
- `firmware/` (Arduino `.ino` source with comments)
- `hardware/` (photos of hardware, schematics, PCB photos, component list)

---

## How the noise-reduction works

1. **Signal pre-filtering:** Incoming Bluetooth command bytes are validated using a small packet format: `[START][CMD][ARG][CHK][END]` where `CHK` is an 8-bit checksum (XOR) to catch corrupted frames.
2. **Debounce and time-windowing:** Commands are accepted only if the same command pattern is received consistently within a configured time window (e.g., 100–300 ms) to prevent transient interference from triggering actions.
3. **RSSI & connection checks:** The system uses HC-05 connection state and RSSI heuristics (when available) to decide whether the phone is in proximity; a lost connection triggers auto-lock after a short timeout.
4. **Hysteresis & thresholds:** Hysteresis prevents oscillation near boundary RSSI values; auto-lock requires sustained out-of-range detection.

---

## Hardware (high-level)

- **Microcontroller:** Arduino Uno (ATmega328P)
- **Bluetooth:** HC-05 module
- **Actuator:** Electromechanical latch (solenoid-based)
- **Driver:** Relay module to switch actuator power
- **Power supply:** Small SMPS providing 5V for MCU/Bluetooth and a higher supply if actuator requires it
- **Peripherals:** Buzzer, status LED, push-button fallback for manual unlock

---

## Software & Firmware

- **Arduino firmware:** C/C++ (Arduino IDE). Implements serial parsing, checksum, debounce, actuator control, and status reporting.
- **Android app:** Simple app with PIN-based authentication using Bluetooth SPP; sends framed commands and shows lock status.

---

## Test & validation

- Test scenarios under different interference (near-by Wi‑Fi traffic, multiple Bluetooth devices).
- False-activation rate before and after filtering (qualitative or quantitative).
- Power measurements and actuator current draw.
- User experience testing with the Android app.

---
