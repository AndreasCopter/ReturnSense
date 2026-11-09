# SkyGuard-INAV

**Intelligent RC Failsafe & GPS-Loss Escape System for INAV**

SkyGuard-INAV is an experimental ESP32-based flight-safety supervisor designed for **INAV-powered multirotors**.

The system monitors the RC link, INAV telemetry, GPS status, aircraft heading, altitude, and navigation information. When RC is lost, SkyGuard evaluates the available navigation state and selects an appropriate recovery action.

If GPS and home information are healthy, SkyGuard is intended to hand control to INAV RTH.

If GPS is lost, unreliable, or suspected of being jammed or spoofed, SkyGuard may activate an experimental **GPS-loss escape maneuver**:

- Activate ALT HOLD.
- Activate HEADING mode.
- Fly toward a configured safe escape heading.
- Maintain a configured recovery altitude.
- Periodically retry GPS.
- If GPS recovers, hand control to INAV RTH.
- If GPS does not recover, activate a controlled landing fallback.

> **Status: Experimental / Development — not flight-ready.**
>
> CRSF decoding, MSP RC override, AUX mode activation, RTH command handling, GPS-loss escape behavior, and recovery handoff must be validated on the bench before flight testing.

---

## Features

- ESP32-based supervisory controller
- INAV 8.0.1 integration
- MSP communication
- INAV UART support
- CRSF / ExpressLRS receiver interface
- RC-link watchdog
- Configurable RC-loss timeout
- RC recovery qualification
- GPS validity monitoring
- GPS satellite monitoring
- GPS staleness monitoring
- Basic GPS plausibility checks
- Home-vector monitoring
- Heading monitoring
- Altitude monitoring
- Ground-speed monitoring
- MSP communication watchdog
- RC channel forwarding
- Experimental GPS-loss escape maneuver
- Experimental ALT HOLD activation
- Experimental HEADING mode activation
- Experimental MSP RC override
- Experimental RTH handoff after GPS recovery
- Controlled-landing fallback after escape timeout
- Heading / ground-track deviation diagnostic
- Recovery state machine
- Serial debugging and telemetry logging
- Designed for X4 multirotors

---

## Important limitation

SkyGuard does **not** directly control motors.

INAV remains responsible for:

- Attitude stabilization
- PID control
- Motor mixing
- Altitude control
- Navigation
- Sensor fusion
- Flight-mode execution
- Landing

SkyGuard is responsible for monitoring and higher-level recovery decisions. It may command selected RC channels through MSP, but INAV executes the resulting flight behavior.

---

## Concept

```text
                 RC TRANSMITTER
                       │
                       ▼
                ┌─────────────┐
                │   SkyGuard  │
                │    ESP32    │
                │             │
                │ CRSF decoder│
                │ RC watchdog │
                │ GPS monitor │
                │ MSP monitor │
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

---

## Receiver architecture

SkyGuard is designed as an ESP32-based RC interface and supervisory controller.

```text
RC transmitter
      │
      ▼
ExpressLRS / CRSF receiver
      │
      ▼
ESP32 SkyGuard
      │
      ├── CRSF decoder
      ├── RC watchdog
      ├── GPS / MSP monitoring
      └── MSP RC override
      │
      ▼
INAV flight controller
```

During normal operation, SkyGuard decodes CRSF frames and forwards RC channels to INAV through MSP.

During an RC-loss recovery event, SkyGuard may stop forwarding pilot channels and send controlled recovery-channel values instead.

### Important limitation

The ESP32 must not become a single point of failure for normal RC control.

Before flight, verify that:

- A powered-off ESP32 cannot prevent receiver control.
- An ESP32 crash or reset cannot leave INAV in an unsafe mode.
- MSP override cannot activate while the pilot still has valid RC control.
- All AUX mode mappings have been verified in the INAV Modes tab.
- RC recovery requires sustained valid RC frames, not one isolated frame.

---

## Recovery concept

SkyGuard uses different recovery actions depending on the navigation state:

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
              RC frame loss confirmed
                       │
                       ▼
              ┌────────────────┐
              │ VALIDATE STATE │
              └───────┬────────┘
                      │
          ┌───────────┴────────────┐
          │                        │
  GPS + HOME HEALTHY       GPS LOST / SUSPECT
          │                        │
          ▼                        ▼
   ┌─────────────┐        ┌──────────────────┐
   │ INAV RTH    │        │ GPS-LOSS ESCAPE  │
   └──────┬──────┘        └────────┬─────────┘
          │                        │
          ▼                        ▼
   Return to home         Activate ALT HOLD
                                  │
                                  ▼
                          Activate HEADING mode
                                  │
                                  ▼
                     Fly toward safe escape heading
                                  │
                                  ▼
                            Periodically retry GPS
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
             GPS recovers                GPS remains lost
                    │                           │
                    ▼                           ▼
              INAV RTH                  Controlled landing
```

### GPS-loss escape

When RC is lost and GPS is unavailable, unreliable, or suspected of being compromised, SkyGuard may activate an escape maneuver:

1. Stop forwarding stale pilot RC channels.
2. Activate ALT HOLD.
3. Activate HEADING mode.
4. Command a bounded roll correction toward a configured safe escape heading.
5. Maintain a configured recovery altitude.
6. Periodically retry GPS.
7. If GPS becomes valid and stable, hand control to INAV RTH.
8. If GPS does not recover within the configured escape period, activate a controlled landing.

The escape heading is intended to move the aircraft away from a suspected GPS interference or jamming area. It is **not** a substitute for GPS-based RTH and must not be used to navigate toward an unknown or unverified home position.

For example, if the configured safe escape heading is 225° and the target escape altitude is 25 m, the intended behavior is:

```text
RC loss + GPS loss
    -> ALT HOLD
    -> HEADING mode
    -> turn toward 225°
    -> maintain 25 m altitude
    -> retry GPS
    -> RTH if GPS recovers
    -> controlled landing if GPS remains unavailable
```

---

## Why SkyGuard?

A conventional RC failsafe can detect:

> “The transmitter signal is gone.”

SkyGuard attempts to answer a more useful question:

> “The transmitter is gone — does the aircraft still have enough information to recover safely?”

It can consider multiple sources of information:

```text
RC status
GPS status
GPS satellites
Home position
Heading
Altitude
Ground speed
INAV telemetry
MSP communication health
```

This allows the recovery decision to be based on the overall navigation state rather than RC signal status alone.

---

## Heading / ground-track deviation

During normal flight, SkyGuard compares the aircraft heading with the GPS ground track.

For example:

```text
Aircraft heading:     225°
GPS ground track:     232°

Difference:            +7°
```

The value is filtered over time rather than using a single GPS sample.

On a multirotor, heading and ground track can legitimately differ because the aircraft may fly sideways or backward. Therefore, this value is **not** a wind estimate and must not be used as the sole basis for recovery navigation.

Potential future uses include:

- Detecting implausible GPS course changes.
- Comparing GPS course with compass heading.
- Supporting a future wind or navigation-quality estimator.

> This feature is experimental and does not currently influence recovery behavior.

---

## GPS validation and spoofing resistance

A GPS fix alone is not sufficient for autonomous recovery.

SkyGuard evaluates:

- GPS fix status
- Satellite count
- GPS position stability
- GPS ground speed
- Home-vector freshness
- Attitude/heading freshness
- MSP communication freshness
- Plausible position changes
- Plausible altitude behavior

These checks are intended to detect obvious GPS glitches, poor signal quality, and some forms of spoofing-like inconsistency.

They do **not** provide cryptographic GNSS authentication or guarantee protection against sophisticated spoofing. A single GPS receiver cannot independently prove that its reported position is authentic.

If GPS becomes invalid or implausible, SkyGuard must not use it for RTH. It should use the configured GPS-loss escape or landing behavior instead.

---

## Hardware

### Flight controller

- INAV 8.0.1 compatible flight controller
- GPS
- Compass where supported/required
- Barometer where available

### Companion / supervisor

- ESP32
- Hardware UART for RC receiver
- Hardware UART for INAV MSP

### Example connections

```text
ESP32                  INAV

TX ------------------> UART RX
RX <------------------ UART TX
GND ------------------ GND
```

The ESP32 and flight controller must use compatible logic levels.

---

## Example architecture

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

## Software architecture

```text
SkyGuard
│
├── RC Interface
│   └── CRSF / ExpressLRS decoder
│
├── MSP Interface
│   ├── MSP requests
│   ├── MSP parser
│   └── RC override
│
├── Telemetry
│   ├── GPS
│   ├── Attitude
│   ├── Altitude
│   └── Home information
│
├── Watchdog
│   ├── RC link monitoring
│   └── MSP communication monitoring
│
├── Navigation
│   ├── Home validation
│   ├── Heading monitoring
│   └── Heading / ground-track deviation
│
├── Recovery
│   ├── Recovery validation
│   ├── RTH handoff
│   ├── GPS-loss escape
│   └── Controlled landing
│
└── Diagnostics
    ├── Serial logging
    └── Telemetry logging
```

---

## Recovery state machine

```text
NORMAL
  │
  │ RC frame lost
  ▼
RC_LOST
  │
  │ loss persists
  ▼
VALIDATE
  │
  ├──────────────────────────────┐
  │                              │
  │ GPS/home healthy             │ GPS lost / suspect
  ▼                              ▼
RTH                         ESCAPE_ALT_HOLD
  │                              │
  │                              ▼
  │                         ESCAPE_HEADING
  │                              │
  │                              ▼
  │                         ESCAPE_FLYING
  │                              │
  │              ┌───────────────┴───────────────┐
  │              │                               │
  │       GPS recovers                    Escape timeout
  │              │                               │
  │              ▼                               ▼
  │       GPS_RECOVERED                         LAND
  │              │
  │              ▼
  │             RTH
  │
  │ RC recovered and stable
  ▼
NORMAL
```

MSP communication failure is handled separately. If SkyGuard cannot reliably communicate with INAV, it must not attempt to control the aircraft; INAV’s native failsafe behavior takes precedence.

---

## Configuration

Important parameters include:

```cpp
// RC link monitoring
RC_FRAME_TIMEOUT_MS = 200;
RECOVERY_DECISION_DELAY_MS = 2000;
RC_RECOVER_MS = 3000;

// GPS validation
MIN_SATS = 8;
GPS_VALID_TIME_MS = 3000;
GPS_STALE_TIMEOUT_MS = 1000;
MAX_GPS_JUMP_METERS_PER_SECOND = 50.0f;
MAX_GPS_SPEED_MS = 40.0f;

// GPS-loss escape
SAFE_ESCAPE_HEADING_DEG = 225.0f;
ESCAPE_TARGET_ALTITUDE_M = 25.0f;
MAX_ESCAPE_ROLL_US = 150.0f;
MAX_ESCAPE_PITCH_US = 120.0f;
HEADING_DEADBAND_DEG = 4.0f;
HEADING_KP = 3.0f;
ESCAPE_MAX_DURATION_MS = 30000;
GPS_RETRY_INTERVAL_MS = 1000;
GPS_RECOVERY_CONFIRM_MS = 3000;

// RTH
ENABLE_AUTONOMOUS_RTH = false;
```

These values are starting points only. They must be adjusted and validated experimentally for the aircraft, flight area, GPS receiver, and mission requirements.

The configured escape heading must point toward a known safe area. Do not use an arbitrary heading without checking terrain, obstacles, people, restricted airspace, and the likely location of GPS interference.

---

## INAV integration

SkyGuard communicates with INAV through **MSP**.

```text
ESP32
  │
  │ MSP
  ▼
INAV UART
```

The INAV serial port used by SkyGuard must be configured for MSP.

Before changing the INAV configuration, verify the serial-port numbering for the particular flight-controller target.

### MSP RC override

The GPS-loss escape behavior requires MSP RC override.

INAV 8.0.1 supports MSP RC override, but the implementation must be verified for the exact firmware build and board target.

Before enabling the escape behavior, verify:

- The INAV firmware is compiled with MSP RC override support.
- The correct override channels are configured.
- The AUX channels used for ALT HOLD and HEADING mode are correctly mapped.
- MSP RC updates meet the required refresh rate.
- Mode activation is confirmed through INAV telemetry.
- A failure of the ESP32 cannot prevent normal RC control.

Do **not** assume that setting an AUX channel automatically activates the intended INAV mode. Mode activation must be verified through telemetry or observed behavior during bench testing.

---

## RTH philosophy

SkyGuard is designed to use INAV’s navigation capabilities rather than replacing the flight controller’s navigation system.

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

## Safety

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
3. Test CRSF decoding
       ↓
4. Connect to INAV on bench
       ↓
5. Test MSP telemetry
       ↓
6. Test RC forwarding
       ↓
7. Test MSP RC override
       ↓
8. Test AUX mode activation
       ↓
9. Test ALT HOLD and HEADING mode
       ↓
10. Test with propellers removed
       ↓
11. Test failsafe state transitions
       ↓
12. Test GPS-loss escape behavior
       ↓
13. Test GPS recovery and RTH handoff
       ↓
14. Test controlled-landing fallback
       ↓
15. Test with INAV simulator/SITL where possible
       ↓
16. Controlled low-risk flight testing
```

**Never perform the first autonomous recovery test with propellers installed and an uncontrolled aircraft.**

---

## Current development status

| Feature | Status |
|---|---|
| ESP32 MSP communication | 🟡 Development |
| INAV UART integration | 🟡 Development |
| CRSF/ELRS decoding | 🟡 Experimental |
| RC forwarding | 🟡 Experimental |
| RC watchdog | 🟢 Prototype |
| GPS monitoring | 🟢 Prototype |
| GPS plausibility checks | 🟡 Experimental |
| Home-vector monitoring | 🟡 Development |
| Heading monitoring | 🟢 Prototype |
| Altitude monitoring | 🟡 Development |
| Ground-speed monitoring | 🟢 Prototype |
| Heading/ground-track deviation | 🟡 Experimental |
| Recovery state machine | 🟡 Experimental |
| GPS-loss escape maneuver | 🔴 Experimental |
| ALT HOLD activation | 🔴 Experimental |
| HEADING mode activation | 🔴 Experimental |
| MSP RC override | 🔴 Experimental |
| RTH activation | 🔴 Experimental |
| RTH confirmation | 🔴 Not implemented |
| Controlled landing fallback | 🔴 Experimental |
| Wind estimation | 🔴 Planned |
| Configuration interface | 🔴 Planned |
| Flight logging | 🔴 Planned |

---

## Roadmap

### Phase 1 — Receiver

- [ ] ESP32 RC protocol implementation
- [ ] CRSF/ExpressLRS support
- [ ] RC channel forwarding
- [ ] RC signal-quality monitoring
- [ ] RC recovery qualification

### Phase 2 — INAV telemetry

- [ ] MSP telemetry parser
- [ ] GPS status
- [ ] Home position
- [ ] Heading
- [ ] Altitude
- [ ] Ground speed
- [ ] Navigation state
- [ ] MSP communication watchdog

### Phase 3 — Failsafe

- [ ] RC timeout
- [ ] GPS validation
- [ ] Home validation
- [ ] GPS plausibility checks
- [ ] Recovery state machine
- [ ] MSP failure handling

### Phase 4 — Navigation intelligence

- [ ] Heading / ground-track deviation
- [ ] GPS spoofing-resistance checks
- [ ] Position-quality estimation
- [ ] Navigation confidence score
- [ ] Wind estimation

### Phase 5 — Recovery

- [ ] INAV RTH activation
- [ ] RTH status confirmation
- [ ] GPS-loss escape activation
- [ ] ALT HOLD activation and verification
- [ ] HEADING mode activation and verification
- [ ] Safe escape heading selection
- [ ] Bounded heading-control loop
- [ ] Escape altitude management
- [ ] GPS-recovery confirmation
- [ ] Escape timeout and controlled landing
- [ ] RC recovery qualification
- [ ] Recovery event logging

### Phase 6 — Tools

- [ ] ESP32 configuration interface
- [ ] Web-based configuration
- [ ] Flight-log viewer
- [ ] Heading/track visualization
- [ ] Recovery-event analysis

---

## Project philosophy

SkyGuard is built around one principle:

> **The flight controller flies the aircraft. SkyGuard watches over the flight controller and makes higher-level safety decisions.**

This separation keeps the ESP32 system relatively simple while allowing INAV to perform the difficult real-time stabilization and navigation tasks.

---

## License

This project is licensed under the MIT License.

---

## Disclaimer

This project is provided for research, experimentation, and educational purposes.

The author(s) make no guarantee that SkyGuard will prevent aircraft loss, damage, injury, or other consequences resulting from an RC-link failure, GPS interference, software failure, hardware failure, or other unexpected condition.

Users are responsible for validating the system, complying with applicable aviation regulations, and ensuring that all failsafe behavior is appropriate for their aircraft and operating environment.

---

## Project name

**SkyGuard-INAV**

**Intelligent RC Failsafe & GPS-Loss Escape System**