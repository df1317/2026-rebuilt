# Logging & Error Handling

This document describes the centralized logging and error handling system.

## Overview

All logging flows through two utilities in `frc.robot.util`:

| Class | Purpose |
|-------|---------|
| `RobotLog` | Errors, warnings, and alerts (driver-facing) |
| `DogLog` | Telemetry data (poses, speeds, sensor values) |

## RobotLog

Unified error/warning reporting that outputs to:
- **Elastic dashboard** - popup notifications for drivers
- **DriverStation console** - errors/warnings in DS logs
- **WPILib Alerts** - persistent dashboard indicators
- **DogLog/DataLog** - recorded for post-match analysis

### One-time Events

Use for errors/warnings that happen once (e.g., initialization failures, caught exceptions):

```java
// Info (not shown to drivers, just logged)
RobotLog.info("Swerve/Calibration", "Calibration complete", "All modules zeroed");

// Warning (yellow popup + DS warning)
RobotLog.warn("Vision/NoClients", "Vision warning", "No cameras found");

// Error (red popup + DS error)
RobotLog.error("Auto/PathPlannerConfig", "PathPlanner failed", "Config error", exception);

// Fatal (red popup + throws RuntimeException)
throw RobotLog.fatal("Swerve/Init", "Swerve init failed", "Check motor connections", exception);
```

Events are **throttled by key** (1 second default) to prevent spam in periodic loops.

### Persistent Alerts

Use for ongoing conditions that should stay visible until resolved:

```java
// Set alert active
RobotLog.setWarningAlert("Vision/Latency/PEBBLE", "PEBBLE camera high latency", true);

// Clear alert when condition resolves
RobotLog.setWarningAlert("Vision/Latency/PEBBLE", "PEBBLE camera high latency", false);

// Error alerts (more severe)
RobotLog.setErrorAlert("Auto/Unavailable", "Autos unavailable", true);
```

Alerts appear in the **Alerts widget** on Elastic/Shuffleboard and are logged to DogLog:
- `Alerts/<key>/active` - boolean, when alert is on/off
- `Alerts/<key>/type` - "kWarning" or "kError"

### Key Naming Convention

Use hierarchical keys for organization:
- `Swerve/Init`, `Swerve/SetpointGenerator`
- `Vision/NoClients`, `Vision/Latency/PEBBLE`
- `Auto/PathPlannerConfig`, `Auto/Unavailable`

## DogLog (Telemetry)

For regular telemetry data, use DogLog directly:

```java
// Standard telemetry (auto-disables NT at FMS, always logs to DataLog)
DogLog.log("Swerve/Velocity", velocity);
DogLog.log("Vision/ConnectedCameras", count);
DogLog.log("Autopilot/Acceleration", acceleration);

// Essential values that MUST show on dashboard even at competition
DogLog.forceNt.log("Dash/MatchTime", DriverStation.getMatchTime());
DogLog.forceNt.log("Dash/RobotRelative", robotRelative);

// Runtime-tunable values (adjustable via NT in dev mode)
BooleanSubscriber visionEnabled = DogLog.tunable("Swerve/VisionEnabled", true);
```

## When to Use What

| Situation | Tool |
|-----------|------|
| Telemetry (poses, speeds, counts) | `DogLog.log()` |
| Dashboard essentials (match time) | `DogLog.forceNt.log()` |
| Runtime-adjustable values | `DogLog.tunable()` |
| One-time error/warning | `RobotLog.warn()` / `RobotLog.error()` |
| Fatal startup failure | `RobotLog.fatal()` |
| Ongoing condition (camera disconnected) | `RobotLog.setWarningAlert()` |

## Elastic Notifications

Popup notifications appear in the corner of the Elastic dashboard:

- **ERROR** (red) - 5 second display, 10 seconds for fatal
- **WARNING** (yellow) - 5 second display
- **INFO** (blue) - not sent by default

Notifications are automatically sent when you call `RobotLog.warn()` or `RobotLog.error()`.

## Post-Match Analysis

All events and alerts are logged to DataLog for AdvantageScope analysis:

- `Events/<key>/severity` - INFO, WARNING, ERROR, FATAL
- `Events/<key>/title` - short title
- `Events/<key>/message` - detailed message
- `Events/<key>/exception` - exception string (if provided)
- `Alerts/<key>/active` - boolean timeline of alert state
- `Alerts/<key>/type` - alert severity type

Graph `Alerts/*/active` as booleans to see exactly when issues occurred during a match.

## Status Colors

All subsystem status colors use the `RobotLog` palette and follow a consistent scheme:

### Color Palette (`RobotLog`)

| Color | Hex | Meaning |
|-------|-----|---------|
| Red | `#F44336` | Idle / off / not available |
| Yellow | `#E5B844` | Active but not ready (spinning up, moving, buffered) |
| Green | `#4CAF50` | Ready / at target / active |
| Blue | `#1064FF` | Reverse direction |

### Per-Subsystem Status

| Subsystem | Red | Yellow | Green | Blue |
|-----------|-----|--------|-------|------|
| **Shooter** | Target RPM ~0 (idle) | Spinning up | At speed | Reverse |
| **Intake** | Retracted & idle | Extending or not at speed | Extended & at speed | Reverse |
| **Hub** | Not available | Buffered (available within 3s) | Active (can score now) | — |

### Dashboard Keys

Status colors are logged via `DogLog.forceNt` as hex strings:

- `Shooter/Status`, `Intake/Status`
- `Hub/StatusColor` (also logs `Hub/Status` as enum name)

## Configuration

Thresholds and constants are in `Constants.java`:

```java
public static final class VisionConstants {
    // High latency threshold for camera alerts
    public static final double HIGH_LATENCY_THRESHOLD_MS = 100.0;
}
```
