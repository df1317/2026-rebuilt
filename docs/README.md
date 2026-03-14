# Docs

Quick reference documentation for robot controls and configuration.

## Documentation

- [Logging & Error Handling](logging.md) - Centralized logging system with Elastic notifications
- [Shooter](shooter.md) - Flywheel, feeder, hood homing, and distance lookup table

## Controls

### Teleop

#### Xbox Controller (Driver) [`Port 0`]

| Binding         | Action                | Description                                                      |
|-----------------|-----------------------|------------------------------------------------------------------|
| `Left Stick`    | Drive translate       | Controls robot translation                                       |
| `Right Stick X` | Drive rotate          | Controls robot rotation                                          |
| `Button A`      | Zero gyro             | Resets gyro heading (once)                                       |
| `Left Bumper`   | Climber Align         | Aligns robot to nearest side of the climber                      |
| `Right Bumper`  | Toggle field relative | Switches between field and robot relative driving                |
| `Button X`      | Toggle intake stow    | Extends/retracts intake (no roller)                              |
| `Left Trigger`  | Intake (hold)         | Extends intake and runs roller                                   |
| `Right Trigger` | Fire (hold)           | Shoot + feed + aim — zone-aware, only fires in own alliance zone |

### Zone Automation (`TeleopZoneAutomation`)

Triggers activate automatically based on field position during teleop.

| Condition                                | Action                                                    |
|------------------------------------------|-----------------------------------------------------------|
| Shuttle mode + in own alliance zone      | Auto-extend intake and run rollers                        |
| Right Bumper held + in own alliance zone | Spin up shooter to hub distance, feed hopper concurrently |

The shooter speed is calculated from the live robot-to-hub distance using the distance LUT
(see [Shooter docs](shooter.md#distance-lut)).

### Test Mode

In test mode, buttons on the Maypad run individual subsystem commands. Tunable values are adjustable live in Elastic or
Glass under the `Tunable/` table.

#### Maypad (Operator Panel) [`Port 1`]

Firmware: [df1317/maypad-frc](https://github.com/df1317/maypad-frc) — grab the latest `.hex` from the Actions tab.

|           | Col 0                 | Col 1               | Col 2                | Col 3                |
|-----------|-----------------------|---------------------|----------------------|----------------------|
| **Row 0** | `testFlywheel` (hold) | `testFeeder` (hold) | `spinUpShoot` (hold) | `stop` (once)        |
| **Row 1** | `homeHood` (once)     | `testHood` (once)   | `aimBangBang` (hold) | `aimPID` (hold)      |
| **Row 2** | `extend` (hold)       | `retract` (hold)    | `runRoller` (hold)   | `eject` (hold)       |
| **Row 3** | `testHopper` (hold)   | `feed` (hold)       | `reverse` (hold)     | —                    |
| **Row 4** | `homeClimber` (once)  | `extend` (once)     | `retract` (once)     | `zeroEncoder` (once) |

| Key          | Action                 | Notes                                                               |
|--------------|------------------------|---------------------------------------------------------------------|
| **shooter**  |                        |                                                                     |
| `key(0, 0)`  | Test flywheel          | Tunable: `Shooter/RPM`                                              |
| `key(0, 1)`  | Test feeder            | Tunable: `Shooter/Feeder/RPM`                                       |
| `key(0, 2)`  | Spin up + shoot        | Spins up then feeds hopper                                          |
| `key(0, 3)`  | Stop shooter           |                                                                     |
| **hood/aim** |                        |                                                                     |
| `key(1, 0)`  | Auto-home hood         | Stall-detection homing; see [Shooter docs](shooter.md#homing)       |
| `key(1, 1)`  | Test hood position     | Tunable: `Shooter/Hood/Percent`                                     |
| `key(1, 2)`  | Aim at hub (bang-bang) | Stepped omega — 4 tiers by error magnitude. Logs to `Aim/`          |
| `key(1, 3)`  | Aim at hub (PID)       | `ProfiledPIDController`, trapezoidal constraints. Logs to `AimPID/` |
| **intake**   |                        |                                                                     |
| `key(2, 0)`  | Extend intake          |                                                                     |
| `key(2, 2)`  | Run roller             |                                                                     |
| `key(2, 3)`  | Eject                  |                                                                     |
| **hopper**   |                        |                                                                     |
| `key(3, 0)`  | Test hopper            | Tunable: `Hopper/RPM`                                               |
| `key(3, 1)`  | Feed                   |                                                                     |
| `key(3, 2)`  | Reverse hopper         |                                                                     |
| **climber**  |                        |                                                                     |
| `key(4, 0)`  | Auto-home climber      | Drives to bottom hard stop, zeros encoder                           |
| `key(4, 1)`  | Climber extend         | Goes to `MAX_HEIGHT`                                                |
| `key(4, 2)`  | Climber retract        | Goes to `MIN_HEIGHT`                                                |
| `key(4, 3)`  | Zero encoder           | Sets current position as zero                                       |

#### Dashboard Toggles

| Tunable                      | Default | Description                                                 |
|------------------------------|---------|-------------------------------------------------------------|
| `Drive/ObstacleClampEnabled` | `false` | Clamps teleop drive to prevent driving into obstacles/walls |

## CAN IDs

| Component   | Location    | ID   |
|-------------|-------------|------|
| Drive Motor | Front Right | `12` |
| Drive Motor | Front Left  | `13` |
| Drive Motor | Back Right  | `14` |
| Drive Motor | Back Left   | `15` |
|             |             |      |
| Turn Motor  | Front Right | `16` |
| Turn Motor  | Front Left  | `17` |
| Turn Motor  | Back Right  | `18` |
| Turn Motor  | Back Left   | `19` |
|             |             |      |
| CanCoder    | Front Right | `20` |
| CanCoder    | Front Left  | `21` |
| CanCoder    | Back Right  | `22` |
| CanCoder    | Back Left   | `23` |
|             |             |      |
| Climber     | Left Motor  | `24` |
| Climber     | Right Motor | `25` |
| Hopper      | Motor       | `26` |
| Shooter     | Motor       | `30` |

## DIO Constants

| Sensor | Port |
|--------|------|
| TBD    | TBD  |
