# SkyGuard-INAV

**Intelligent RC Failsafe & Flight Recovery System for INAV**

SkyGuard-INAV is an experimental ESP32-based flight-safety supervisor designed for **INAV-powered multirotors**.

The system monitors the RC link, INAV telemetry, GPS status, aircraft heading, altitude, and navigation information. When a prolonged RC-link failure is detected, SkyGuard can evaluate whether the aircraft has sufficient navigation information and, when configured and validated, initiate an INAV recovery/RTH procedure.

The goal is to provide an additional intelligent safety layer between the RC system and the flight controller while leaving **flight stabilization, attitude control, altitude control, and motor mixing to INAV**.

> **Status: Experimental / Development**

---

## Features

- ESP32-based supervisory controller
- INAV 8.x integration
- MSP communication
- INAV UART2 support
- RC-link watchdog
- Configurable RC-loss timeout
- GPS validity monitoring
- GPS satellite monitoring
- Home-position validation
- Heading monitoring
- Altitude monitoring
- Ground-speed monitoring
- Home-direction monitoring
- RC channel forwarding
- Drift estimation during normal flight
- Filtered heading/ground-track deviation
- Recovery state machine
- INAV RTH integration
- Failsafe/fallback state
- Serial debugging and telemetry logging
- Designed for X4 multirotors

---

## Concept

SkyGuard operates as a supervisory layer:

```text
                 RC TRANSMITTER
                       │
                       ▼
                ┌─────────────┐
                │   SkyGuard  │
                │    ESP32    │
                │             │
                │ RC Watchdog │
                │ GPS Monitor │
                │ Drift Model │
                │ Recovery    │
                └──────┬──────┘
                       │
                       │ MSP
                       │
                       ▼
                ┌─────────────┐
                │    INAV     │
                │ Flight Ctrl │
                └──────┬──────┘
                       │
                ┌──────┼──────┐
                ▼      ▼      ▼
               M1     M2     M3/M4
```

SkyGuard does **not** directly control the motors.

INAV remains responsible for:

- attitude stabilization
- PID control
- motor mixing
- altitude control
- navigation
- sensor fusion
- flight-mode execution
- landing

SkyGuard is responsible for monitoring and higher-level failsafe decisions.

---

## Recovery concept

The basic recovery sequence is:

```text
                    NORMAL FLIGHT
                         │
                         │
                  RC signal lost
                         │
                         ▼
                 ┌───────────────┐
                 │ RC WATCHDOG   │
                 └───────┬───────┘
                         │
                  > 5 seconds
                         │
                         ▼
                ┌────────────────┐
                │ VALIDATE STATE │
                └───────┬────────┘
                        │
              ┌─────────┴─────────┐
              │                   │
          GPS + HOME OK       GPS INVALID
              │                   │
              ▼                   ▼
        ┌───────────┐       ┌───────────┐
        │    RTH    │       │ FALLBACK  │
        └─────┬─────┘       └───────────┘
              │
              ▼
            INAV
              │
              ▼
        Return to Home
```

The actual recovery behavior should be implemented through INAV's navigation and failsafe mechanisms wherever possible.

---

# Why SkyGuard?

A conventional RC failsafe can detect:

> "The transmitter signal is gone."

SkyGuard attempts to answer a more useful question:

> "The transmitter is gone — does the aircraft still have enough information to recover safely?"

It can therefore consider multiple sources of information:

```text
RC status
GPS status
GPS satellites
Home position
Heading
Altitude
Ground speed
INAV telemetry
```

This allows the recovery decision to be based on the overall navigation state rather than RC signal status alone.

---

# Drift estimation

One of the experimental features of SkyGuard is drift estimation.

During normal operation, the system compares:

```text
Aircraft heading
        +
GPS ground track
```

For example:

```text
Aircraft heading:     225°
GPS ground track:     232°

Estimated deviation:   +7°
```

The value is filtered over time instead of using a single GPS measurement.

Conceptually:

```text
drift_estimate =
    old_estimate × (1 - α)
    +
    new_measurement × α
```

This allows SkyGuard to build a running estimate of the aircraft's normal directional deviation.

The long-term goal is to improve recovery navigation by considering observed drift/wind conditions.

> Drift estimation is currently experimental and should not be treated as a substitute for a proper navigation/wind estimator.

---

# GPS validation

A GPS "fix" by itself is not considered sufficient.

SkyGuard can evaluate:

```text
GPS fix
GPS satellite count
GPS position
GPS stability
Home position
Ground speed
```

A recovery decision should only use GPS navigation when the required navigation information is valid.

Conceptually:

```text
GPS FIX
   │
   ├── insufficient satellites ──► INVALID
   │
   ├── unstable position ────────► INVALID
   │
   └── stable navigation ────────► VALID
```

The exact thresholds should be configurable according to the aircraft, GPS receiver, environment, and flight requirements.

---

# Hardware

Example hardware configuration:

### Flight controller

- INAV 8.x compatible flight controller
- GPS
- Compass where supported/required
- Barometer where available

### Companion/supervisor

- ESP32
- Hardware UART for RC receiver
- Hardware UART for INAV MSP

### Example connections

```text
ESP32                  INAV

TX ------------------> UART2 RX
RX <------------------ UART2 TX
GND ------------------ GND
```

The ESP32 and flight controller must use compatible logic levels.

---

# Example architecture

```text
                 ExpressLRS / CRSF
                         │
                         ▼
                 ┌─────────────┐
                 │    ESP32    │
                 │             │
                 │ CRSF        │
                 │ MSP         │
                 │ Watchdog    │
                 │ GPS monitor │
                 │ Drift       │
                 │ Recovery    │
                 └──────┬──────┘
                        │
                        │ MSP
                        ▼
                 ┌─────────────┐
                 │    INAV     │
                 │             │
                 │ GPS         │
                 │ IMU         │
                 │ Compass     │
                 │ Barometer   │
                 └──────┬──────┘
                        │
                        ▼
                   X4 motors
```

---

# Software architecture

The firmware is divided conceptually into several modules:

```text
SkyGuard
│
├── RC Interface
│   └── RC protocol decoder
│
├── MSP Interface
│   ├── MSP requests
│   ├── MSP parser
│   └── RC override
│
├── Telemetry
│   ├── GPS
│   ├── attitude
│   ├── altitude
│   └── home information
│
├── Watchdog
│   └── RC link monitoring
│
├── Navigation
│   ├── home validation
│   ├── heading
│   └── drift estimation
│
├── Recovery
│   ├── validation
│   ├── RTH
│   └── fallback
│
└── Diagnostics
    ├── serial logging
    └── telemetry logging
```

---

# Recovery state machine

SkyGuard uses a state-machine approach rather than a single failsafe flag.

```text
NORMAL
  │
  │ RC lost
  ▼
RC_LOST
  │
  ▼
VALIDATE
  │
  ├───────────────┐
  │               │
  ▼               ▼
RTH            FALLBACK
  │
  │ RC recovered
  ▼
NORMAL
```

This approach makes it possible to add additional safety conditions later.

---

# Configuration

Important parameters include:

```cpp
RC_TIMEOUT_MS = 5000;

MIN_SATS = 8;

GPS_VALID_TIME_MS = 3000;

DRIFT_ALPHA = 0.02;
```

These values are starting points only.

They should be adjusted and validated experimentally.

---

# INAV integration

SkyGuard communicates with INAV through **MSP**.

The intended architecture is:

```text
ESP32
  │
  │ MSP
  ▼
INAV UART2
```

The INAV serial port used by SkyGuard must be configured for MSP.

Before changing the INAV configuration, verify the serial-port numbering for the particular flight-controller target.

---

# RTH philosophy

SkyGuard is designed to use INAV's navigation capabilities rather than replacing the flight controller's navigation system.

The preferred architecture is:

```text
SkyGuard detects RC loss
          │
          ▼
SkyGuard validates navigation
          │
          ▼
SkyGuard requests recovery
          │
          ▼
          INAV
          │
          ├── altitude management
          ├── heading control
          ├── navigation
          ├── stabilization
          └── landing
```

This is preferable to attempting to generate motor commands directly from the ESP32.

---

# Safety

## ⚠️ Experimental project

SkyGuard is an experimental flight-control/failsafe project.

It can potentially affect the behavior of a flying aircraft.

**Do not use the firmware on an aircraft carrying people or valuable equipment.**

Always test in progressively safer stages.

### Recommended testing sequence

```text
1. Compile firmware
       ↓
2. Test ESP32 without flight controller
       ↓
3. Connect to INAV on bench
       ↓
4. Test MSP telemetry
       ↓
5. Test RC forwarding
       ↓
6. Test with propellers removed
       ↓
7. Test failsafe state transitions
       ↓
8. Test with INAV simulator/SITL where possible
       ↓
9. Controlled low-risk flight testing
```

**Never perform the first autonomous recovery test with propellers installed and an uncontrolled aircraft.**

---

# Current development status

| Feature | Status |
|---|---|
| ESP32 MSP communication | 🟡 Development |
| INAV UART2 | 🟡 Development |
| RC forwarding | 🟡 Development |
| RC watchdog | 🟢 Prototype |
| GPS monitoring | 🟢 Prototype |
| Home validation | 🟡 Development |
| Drift estimation | 🟡 Experimental |
| RTH integration | 🟡 Development |
| CRSF/ELRS integration | 🟡 Development |
| Autonomous recovery | 🔴 Experimental |
| Wind estimation | 🔴 Planned |
| Configuration tool | 🔴 Planned |
| Flight logging | 🔴 Planned |

---

# Roadmap

## Phase 1 — Receiver

- [ ] ESP32 RC protocol implementation
- [ ] CRSF/ExpressLRS support
- [ ] RC channel forwarding
- [ ] RC signal-quality monitoring

## Phase 2 — INAV telemetry

- [ ] MSP telemetry parser
- [ ] GPS status
- [ ] Home position
- [ ] Heading
- [ ] Altitude
- [ ] Ground speed
- [ ] Navigation state

## Phase 3 — Failsafe

- [ ] RC timeout
- [ ] GPS validation
- [ ] Home validation
- [ ] MSP communication watchdog
- [ ] Recovery state machine

## Phase 4 — Navigation intelligence

- [ ] Drift estimation
- [ ] Wind estimation
- [ ] Position-quality estimation
- [ ] Navigation confidence score

## Phase 5 — Recovery

- [ ] INAV RTH integration
- [ ] Recovery altitude configuration
- [ ] RC recovery handling
- [ ] Recovery logging

## Phase 6 — Tools

- [ ] ESP32 configuration interface
- [ ] Web-based configuration
- [ ] Flight-log viewer
- [ ] Drift visualization
- [ ] Recovery-event analysis

---

# Project philosophy

SkyGuard is built around one principle:

> **The flight controller flies the aircraft. SkyGuard watches over the flight controller and makes higher-level safety decisions.**

This separation keeps the ESP32 system relatively simple while allowing INAV to perform the difficult real-time stabilization and navigation tasks.

---

# License

Choose a license appropriate for your project before publishing the repository.

For example:

```text
MIT License
```

or another open-source license depending on your intended hardware/software distribution.

---

# Disclaimer

This project is provided for research, experimentation, and educational purposes.

The author(s) make no guarantee that SkyGuard will prevent aircraft loss, damage, injury, or other consequences resulting from an RC-link failure or software/hardware malfunction.

Users are responsible for validating the system, complying with applicable aviation regulations, and ensuring that all failsafe behavior is appropriate for their aircraft and operating environment.

---

## Project name

**SkyGuard-INAV**

**Intelligent RC Failsafe & Flight Recovery System**