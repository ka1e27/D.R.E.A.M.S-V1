# D.R.E.A.M.S-V1
# Autonomous Survey Catamaran

An autonomous catamaran for environmental water quality surveying, built around three Raspberry Pi Picos communicating via HC-12 433MHz radio and shared UART. Designed for river and coastal surveys including the Thames and Dead Sea.

## System Architecture

**Boat Pico** — Main controller running nine concurrent async tasks. Handles GPS navigation, compass heading, dual motor drive with pulley compensation, winch-deployed underwater sensor pod, mission execution, telemetry broadcast, and safety systems. Navigates using GPS ground track as primary heading source with magnetometer compass fallback.

**Shore Pico** — Operator station with button-triggered command menu, e-stop button, telemetry display, CSV data logging, mission file loading from JSON, return-to-home with energy estimation, and automatic low-battery return prompting.

**Pod Pico** — Underwater sensor package housing water temperature (NTC thermistor), electrical conductivity (polarity-switching probe), light level, and bottom contact switch. Communicates with the boat via UART through a slip ring connection. Responds to POLL commands with checksummed sensor frames.

## Features

- **Waypoint navigation** with autonomous transit between data collection points and path-only waypoints
- **Contact-based sampling** — winch lowers until bottom contact confirmed (3 readings), collects sensor data at intervals, retracts when contact lost or safety timeout reached
- **Operator skip** — WINCH_UP during a mission aborts the current drop, retracts, and advances to next waypoint with partial data preserved
- **Station keeping** — holds position at data waypoints with drift re-acquisition
- **Water column profiling** — records sensor readings at depth intervals during descent
- **Half-duplex radio management** — TX burst/listen cycling with UART drain delays for reliable HC-12 communication
- **Shore gap detection** — shore waits for boat's listen window before transmitting commands
- **Heartbeat link monitoring** — shore sends periodic pings, boat auto-enters RC override after 60s silence
- **E-stop** — physical button sends ESTOP command forcing immediate RC override with motor pin release
- **Energy tracking** — estimates voltage consumption per meter, calculates range to home, warns when return margin is low
- **Safety systems** — GPS/compass fault detection, battery critical/warning thresholds, hull leak sensors, motor temperature monitoring with throttle reduction, configurable safety overrides
- **Remote tuning** — motor speed (nav/manual/winch), trim, and compass north calibration adjustable via radio commands
- **Mission files** — JSON-based waypoint definitions with labeled data and path waypoints, max timeout safety limits
- **CSV logging** — telemetry, water profiles, and skip events logged to shore filesystem

## Hardware

| Component | Boat | Shore | Pod |
|-----------|------|-------|-----|
| MCU | Pico (GP0/GP1 dead) | Pico | Pico |
| Radio | HC-12 on GP12/GP13 | HC-12 on GP0/GP1 | — |
| GPS | NEO-6M on UART1 GP4/GP5 | — | — |
| Compass | QMC5883P I2C 0x2C | — | — |
| Motors | 2× brushless ESC GP6/GP7 | — | — |
| Winch | ESC on GP14 | — | — |
| Battery | 3S LiPo via 15k/100k divider on GP26 | — | — |
| Temp | DHT11 on GP16 | — | NTC on GP26 |
| Motor temp | NTC on GP27/GP28 | — | — |
| Hull | Leak sensors GP17/GP18 | — | — |
| EC probe | — | — | GP17 drive, GP27 ADC, 4.7kΩ series |
| Light | — | — | LDR on GP28 |
| Bottom | — | — | Contact switch GP21 |
| Buttons | — | Command GP15, E-stop GP16 | — |
| Pod link | UART1 shared with GPS | — | UART0 GP0/GP1 |

Motor pulley ratios: Left 18:27, Right 15:30 (compensation applied in firmware).

## Radio Protocol

All messages use `$PAYLOAD*CHECKSUM\n` framing with XOR checksum. Commands from shore are acknowledged with `ACK,` or `NAK,` prefixes. Telemetry uses `D1` (position/nav), `D2` (environment), `D3` (energy), and `STATUS` message types.

## File Structure

```
final-boat.py      — Boat controller firmware
final-shore.py     — Shore station firmware  
final-pod.py       — Underwater pod firmware
waypoints.json     — Mission definitions and home waypoint
```

## Shore Commands

| Command | Description |
|---------|-------------|
| `FORWARD` `BACKWARD` `LEFT` `RIGHT` `STOP` | Manual drive |
| `GOTO` | Navigate to coordinates |
| `MISSION` / `LOAD <name>` / `MISSION_START` | Mission control |
| `HOME` | Return to home waypoint |
| `WINCH_UP` (during mission) | Skip current drop |
| `ALL_STOP` | Stop all motors |
| `E-STOP button` | Emergency stop + RC override |
| `SPEED <nav/manual/winch/all> <1-100>` | Set motor speeds |
| `TRIM <value>` | Adjust left/right balance |
| `SET_NORTH <bearing>` | Calibrate compass |
| `SAFETY_OFF/ON <sensor>` | Override safety checks |
| `RC_OVERRIDE` / `RC_RELEASE` | Manual RC control |
| `RANGE` | Check if home is reachable |
| `STATUS` | Request detailed status |

## Built With

- MicroPython on Raspberry Pi Pico
- uasyncio for concurrent task management
- HC-12 433MHz half-duplex radio modules
- NEO-6M GPS, QMC5883P compass
