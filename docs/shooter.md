# Shooter Subsystem

Controls a flywheel shooter with a feeder motor and a pivoting hood.

## Hardware

| Component      | Motor    | CAN ID | Inversion Constant    |
|----------------|----------|--------|-----------------------|
| Flywheel       | TalonFX  | `40`   | `INVERTED`            |
| Feeder         | SparkMax | `28`   | `FEEDER_INVERTED`     |
| Hood           | SparkMax | `24`   | `HOOD_INVERTED`       |

> If the hood moves the wrong direction, flip `HOOD_INVERTED` in `ShooterConstants`.

## Hood Homing

The hood uses soft limits derived at runtime from its physical hard stops. It must be homed before percent-based positioning works (`setHoodPercent`, `setForDistance`).

### Auto-home (Button 4 in test mode)
Drives to both hard stops via stall detection, zeroes the encoder at min, records max, then applies soft limits. Completes in ~5s.

### Manual homing (test mode)
Use this when auto-homing doesn't reach the stops cleanly:

1. Hold **Button 9** + move joystick to jog the hood to the minimum position
2. Press **Button 5** — marks current position as 0% (min)
3. Jog the hood to the maximum position
4. Press **Button 6** — marks current position as 100% (max), applies soft limits

> After manual homing, `isHoodHomed()` returns `true` and percent-based control is active.

## Teleop Controls

### Xbox Controller (Driver) — Shooter

| Binding          | Action                        |
|------------------|-------------------------------|
| `Left Trigger`   | Decrease hood angle (–5% per press) |
| `Right Trigger`  | Increase hood angle (+5% per press) |
| `Joystick L Button 2` (toggle) | Spin up flywheel → start feeder → run hopper |

## Test Mode Controls

| Button     | Action                        | Notes                          |
|------------|-------------------------------|--------------------------------|
| `Button 4` | Auto-home hood                | Drives to hard stops via stall |
| `Button 5` | Mark current position as min  | Use after jogging to min stop  |
| `Button 6` | Mark current position as max  | Use after jogging to max stop  |
| `Button 7` | Test flywheel                 | Tunable: `Test/ShooterRPM`    |
| `Button 8` | Test feeder                   | Tunable: `Test/FeederRPM`     |
| `Button 9` | Jog hood (hold + joystick)    | Holds position on release      |

## Distance Lookup Table

Both flywheel RPM and hood position are interpolated from distance. Values are in `populateLookupTable()` and need tuning on the real robot.

| Distance (m) | Flywheel (RPM) | Hood (%) |
|--------------|----------------|----------|
| 1.0          | 2500           | 100      |
| 2.0          | 3000           | 83       |
| 3.0          | 3500           | 70       |
| 4.0          | 4000           | 58       |
| 5.0          | 4500           | 50       |
| 6.0          | 5000           | 42       |

## Key Constants (`ShooterConstants`)

| Constant              | Value   | Description                              |
|-----------------------|---------|------------------------------------------|
| `HOOD_INVERTED`       | `true`  | Flip if hood moves wrong direction       |
| `HOOD_GEAR_RATIO`     | `24.0`  | Used for encoder → degrees conversion   |
| `HOOD_TOLERANCE`      | `3 deg` | Deadband for `isHoodAtPosition()`        |
| `HOOD_HOMING_VOLTAGE` | `2.0 V` | Voltage used during homing and jogging   |
| `HOOD_STALL_RPM`      | `2.0`   | Motor RPM threshold for stall detection  |
| `VELOCITY_TOLERANCE`  | `100 RPM` | Deadband for `isAtSpeed()`             |
