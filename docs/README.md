# Docs

Quick reference documentation for robot controls and configuration.

## Documentation

- [Autonomous](autos.md) - Auto routines, AutoBuilder API, positions, and telemetry
- [Logging & Error Handling](logging.md) - Centralized logging system with Elastic notifications
- [Shooter](shooter.md) - Flywheel, feeder, hood homing, and distance lookup table

## Controls

![kle](https://cdn.hackclub.com/019cfe64-8d7d-76df-8109-625e5cd1f564/keyboard-layout.svg)

### Teleop

#### Xbox Controller (Driver) [`Port 0`]

| Binding         | Action                | Description                                                      |
|-----------------|-----------------------|------------------------------------------------------------------|
| `Left Stick`    | Drive translate       | Controls robot translation                                       |
| `Right Stick X` | Drive rotate          | Controls robot rotation                                          |
| `Button A`      | Zero gyro             | Resets gyro heading (once)                                       |
| `Right Bumper`  | Toggle field relative | Switches between field and robot relative driving                |
| `Button X`      | Toggle intake stow    | Extends/retracts intake pivot (no roller)                        |
| `Left Trigger`  | Intake roller (hold)  | Runs intake roller (speed scales with robot velocity)            |
| `Right Trigger` | Fire (hold)           | Shoot + feed + aim — zone-aware, only fires in own alliance zone |

### Zone Automation (`TeleopZoneAutomation`)

Triggers activate automatically based on field position during teleop.

| Condition                                | Action                                                    |
|------------------------------------------|-----------------------------------------------------------|
| Shuttle mode + in own alliance zone      | Auto-extend intake and run rollers                        |
| Right Bumper held + in own alliance zone | Spin up shooter to hub distance, feed hopper concurrently |

The shooter speed is calculated from the live robot-to-hub distance using the distance LUT
(see [Shooter docs](shooter.md#distance-lut)).

#### Maypad (Operator Panel) [`Port 1`]

Firmware: [df1317/maypad-frc](https://github.com/df1317/maypad-frc) — grab the latest `.hex` from the Actions tab.

|           | Col 0                    | Col 1                  | Col 2                  | Col 3                  |
|-----------|--------------------------|------------------------|------------------------|------------------------|
| **Row 0** | `intakeHome` (once)*     | -                      | -                      | -                      |
| **Row 1** | `autoDistance` (once)    | `hoodHome` (once)*     | -                      | -                      |
| **Row 2** | `distanceAdvance` (once) | `intakeForward` (hold) | `hopperForward` (hold) | `Shoot+Feed` (hold)    |
| **Row 3** | `distanceReduce` (once)  | `intakeReverse` (hold) | `hopperReverse` (hold) | `feederReverse` (hold) |
| **Row 4** | `climbBottom` (once)     | `climbTop` (once)      | `climbHang` (once)     | `climbRelease` (once)  |

| Key         | Action           | Notes                                                                    |
|-------------|------------------|--------------------------------------------------------------------------|
| **row 0**   |                  |                                                                          |
| `key(0, 0)` | Home Intake*     | Homes pivot and zeroes encoder (works in teleop and test)                |
| `key(0, 1)` | -                |                                                                          |
| `key(0, 2)` | -                |                                                                          |
| `key(0, 3)` | -                |                                                                          |
| **row 1**   |                  |                                                                          |
| `key(1, 0)` | Auto Distance    | Returns to auto setting the distance from vision                         |
| `key(1, 1)` | Home Hood*       | Homes the shooter hood (works in both teleop and test)                   |
| `key(1, 2)` | -                |                                                                          |
| `key(1, 3)` | -                |                                                                          |
| **row 2**   |                  |                                                                          |
| `key(2, 0)` | Distance Advance | Switches to manual distance and advances distance map by one increment   |
| `key(2, 1)` | Intake Forward   | Runs intake roller forward                                               |
| `key(2, 2)` | Hopper Forward   | Runs hopper forward                                                      |
| `key(2, 3)` | Shoot + Feed     | Shoots and feeds but doesn't aim                                         |
| **row 3**   |                  |                                                                          |
| `key(3, 0)` | Distance Reduce  | Switches to manual distance and decrements distance map by one increment |
| `key(3, 1)` | Intake Reverse   | Ejects from intake                                                       |
| `key(3, 2)` | Hopper Reverse   | Runs hopper in reverse                                                   |
| `key(3, 3)` | Feeder Reverse   | Runs feeder in reverse                                                   |
| **row 4**   |                  |                                                                          |
| `key(4, 0)` | Climber Bottom   | Drives climber to the bottom                                             |
| `key(4, 1)` | Climber Top      | Drives climber to the top                                                |
| `key(4, 2)` | Climber Hang     | Drives climber to the hang position                                      |
| `key(4, 3)` | Climber Release  | Drives climber to release position                                       |

### Test Mode

In test mode, buttons on the Maypad run individual subsystem commands. Tunable values are adjustable live in Elastic or
Glass under the `Tunable/` table. The xbox controls are inherited in this mode.

#### Maypad (Operator Panel) [`Port 1`]

|           | Col 0                | Col 1                | Col 2               | Col 3                |
|-----------|----------------------|----------------------|---------------------|----------------------|
| **Row 0** | `intakeHome` (once)* | `climberZero` (once) | `climberUp` (hold)  | `climberDown` (hold) |
| **Row 1** | `aimTest` (hold)     | `hoodHome` (once)    | `hoodTest` (hold)   | `shootAll` (hold)    |
| **Row 2** | -                    | -                    | -                   | `shooterTest` (hold) |
| **Row 3** | -                    | `intakeTest` (hold)  | `hopperTest` (hold) | `feederTest` (hold)  |
| **Row 4** | -                    | -                    | -                   | -                    |

| Key         | Action                | Notes                                                                                                                       |
|-------------|-----------------------|-----------------------------------------------------------------------------------------------------------------------------|
| **row 0**   |                       |                                                                                                                             |
| `key(0, 0)` | Home Intake*          | Drives pivot to extended hard stop and zeroes encoder (works in teleop and test)                                            |
| `key(0, 1)` | Zero Climber          | Zeroes the climber encoders                                                                                                 |
| `key(0, 2)` | Climber Jog Up        | Jogs climber up with position control and low current limit                                                                 |
| `key(0, 3)` | Climber Jog Down      | Jogs climber down with position control and low current limit                                                               |
| **row 1**   |                       |                                                                                                                             |
| `key(1, 0)` | Aim Test              | Aims drivetrain at hub while driving (test only)                                                                            |
| `key(1, 1)` | Home Hood             | Homes the shooter hood (works in teleop and test)                                                                           |
| `key(1, 2)` | Test Hood Position    | Tunable: `Shooter/Hood/Percent`                                                                                             |
| `key(1, 3)` | Shoot All             | Sets hood, spins up shooter+feeder, then feeds hopper. Tunables: `Shooter/TuneRPM`, `Shooter/TuneHoodPercent`, `Hopper/RPM` |
| **row 3**   |                       |                                                                                                                             |
| `key(2, 3)` | Test Shooter Flywheel | Tunable: `Shooter/RPM`                                                                                                      |
| `key(3, 1)` | Test Intake Pivot     | Tunable: `Intake/Pivot/Degrees`                                                                                             |
| `key(3, 2)` | Test Hopper           | Tunable: `Hopper/RPM`                                                                                                       |
| `key(3, 3)` | Test Feeder           | Tunable: `Shooter/Feeder/RPM`                                                                                               |
| **row 4**   |                       |                                                                                                                             |
| `key(4, 0)` | -                     |                                                                                                                             |
| `key(4, 1)` | -                     |                                                                                                                             |
| `key(4, 2)` | -                     |                                                                                                                             |
| `key(4, 3)` | -                     |                                                                                                                             |

#### Dashboard Toggles

| Tunable                      | Default | Description                                                 |
|------------------------------|---------|-------------------------------------------------------------|
| `Drive/ObstacleClampEnabled` | `false` | Clamps teleop drive to prevent driving into obstacles/walls |

## CAN IDs

| Component      | Location    | ID   |
|----------------|-------------|------|
| Drive Motor    | Front Right | `12` |
| Drive Motor    | Front Left  | `13` |
| Drive Motor    | Back Right  | `14` |
| Drive Motor    | Back Left   | `15` |
|                |             |      |
| Turn Motor     | Front Right | `16` | 
| Turn Motor     | Front Left  | `17` |
| Turn Motor     | Back Right  | `18` |
| Turn Motor     | Back Left   | `19` |
|                |             |      |
| CanCoder       | Front Right | `20` |
| CanCoder       | Front Left  | `21` |
| CanCoder       | Back Right  | `22` |
| CanCoder       | Back Left   | `23` |
|                |             |      |
| Shooter Hood   | Motor       | `24` |
| Intake Pivot   | Motor       | `25` | 
| Hopper         | Motor       | `26` |
| Shooter Feeder | Motor       | `28` |
| Climber        | Left Motor  | `29` |
| Intake Roller  | Motor       | `30` |
| Shooter        | Motor       | `40` |

## DIO Constants

| Sensor | Port |
|--------|------|
| TBD    | TBD  |

## Keyboard Layout Editor

Use [`editor.keyboard-tools.xyz`](https://editor.keyboard-tools.xyz/)

```json
[
  [
    {
      "c": "#1860b5",
      "t": "#ffffff",
      "a": 7,
      "f": 4,
      "w": 4
    },
    "Teleop Mode",
    {
      "x": 0.5,
      "c": "#ff9800",
      "t": "#333333",
      "w": 4
    },
    "Test Mode"
  ],
  [
    {
      "c": "#9c27b0",
      "t": "#ffffff",
      "f": 3,
      "a": 7
    },
    "Home Intake",
    {
      "c": "#cccccc",
      "t": "#666666",
      "a": 0
    },
    "",
    "",
    "",
    {
      "x": 0.5,
      "c": "#9c27b0",
      "t": "#ffffff",
      "a": 7
    },
    "Home Intake",
    {
      "c": "#ff9800",
      "t": "#333333"
    },
    "Climb Zero",
    "Climb Up",
    "Climb Down"
  ],
  [
    {
      "c": "#1860b5",
      "t": "#ffffff"
    },
    "Auto",
    {
      "c": "#9c27b0"
    },
    "Home Hood",
    {
      "c": "#cccccc",
      "t": "#666666",
      "a": 0
    },
    "",
    "",
    {
      "x": 0.5,
      "c": "#ff9800",
      "t": "#333333",
      "a": 7
    },
    "Aim Test",
    {
      "c": "#9c27b0",
      "t": "#ffffff"
    },
    "Hood Home",
    {
      "c": "#ff9800",
      "t": "#333333"
    },
    "Hood Test",
    "Shoot Test"
  ],
  [
    {
      "c": "#1860b5",
      "t": "#ffffff"
    },
    "Dist +",
    "Intake Fwd",
    "Hoper Fwd",
    {
      "c": "#9c27b0"
    },
    "Shoot Feed",
    {
      "x": 0.5,
      "c": "#cccccc",
      "t": "#666666",
      "a": 0
    },
    "",
    "",
    "",
    {
      "c": "#9c27b0",
      "t": "#ffffff",
      "a": 7
    },
    "Shoot Feed"
  ],
  [
    {
      "c": "#1860b5"
    },
    "Dist -",
    "Intake Rev",
    "Hoper Rev",
    "Feed Rev",
    {
      "x": 0.5,
      "c": "#cccccc",
      "t": "#666666",
      "a": 0
    },
    "",
    {
      "c": "#ff9800",
      "t": "#333333",
      "a": 7
    },
    "Intake Test",
    "Hoper Test",
    "Feed Test"
  ],
  [
    {
      "c": "#1860b5",
      "t": "#ffffff"
    },
    "Climb Bottom",
    "Climb Top",
    "Climb Hang",
    "Climb Give",
    {
      "x": 0.5,
      "c": "#cccccc",
      "t": "#666666",
      "a": 0
    },
    "",
    "",
    "",
    ""
  ]
]
```