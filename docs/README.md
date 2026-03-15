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

#### Maypad (Operator Panel) [`Port 1`]

Firmware: [df1317/maypad-frc](https://github.com/df1317/maypad-frc) — grab the latest `.hex` from the Actions tab.

|           | Col 0                    | Col 1                  | Col 2                  | Col 3                  |
|-----------|--------------------------|------------------------|------------------------|------------------------|
| **Row 0** | -                        | -                      | -                      | -                      |
| **Row 1** | `autoDistance` (once)    | -                      | -                      | -                      |
| **Row 2** | `distanceAdvance` (once) | `intakeForward` (hold) | `hopperForward` (hold) | `Shoot+Feed` (hold)    |
| **Row 3** | `distanceReduce` (once)  | `intakeReverse` (hold) | `hopperReverse` (hold) | `feederReverse` (hold) |
| **Row 4** | `climbBottom` (once)     | `climbTop` (once)      | `climbHang` (once)     | `climbRelease` (once)  |

| Key         | Action                | Notes                                                                    |
|-------------|-----------------------|--------------------------------------------------------------------------|
| **row 0**   |                       |                                                                          |
| `key(0, 0)` | -                     |                                                                          |
| `key(0, 1)` | -                     |                                                                          |
| `key(0, 2)` | -                     |                                                                          |
| `key(0, 3)` | -                     |                                                                          |
| **row 1**   |                       |                                                                          |
| `key(1, 0)` | Auto Distance         | Returns to auto setting the distance from vision                         |
| `key(1, 1)` | -                     |                                                                          |
| `key(1, 2)` | -                     |                                                                          |
| `key(1, 3)` | -                     |                                                                          |
| **row 2**   |                       |                                                                          |
| `key(2, 0)` | Distance Advance      | Switches to manual distance and advances distance map by one increment   |
| `key(2, 1)` | Intake Forward        |                                                                          |
| `key(2, 2)` | Hopper Forward        |                                                                          |
| `key(2, 3)` | Shoot + Feed + Hopper | Shoots and Feeds and runs the hopper but doesn't aim                     |
| **row 3**   |                       |                                                                          |
| `key(3, 0)` | Distance Reduce       | Switches to manual distance and decrements distance map by one increment |
| `key(3, 1)` | Intake Reverse        |                                                                          |
| `key(3, 2)` | Hopper Reverse        |                                                                          |
| `key(3, 3)` | Feeder Reverse        |                                                                          |
| **row 4**   |                       |                                                                          |
| `key(4, 0)` | Climber Bottom        | Drives to the bottom                                                     |
| `key(4, 1)` | Climber Top           | Drives to the top                                                        |
| `key(4, 2)` | Climber Hang          | Drives to the hang position                                              |
| `key(4, 3)` | Climber Release       | Drives to release position                                               |

### Test Mode

In test mode, buttons on the Maypad run individual subsystem commands. Tunable values are adjustable live in Elastic or
Glass under the `Tunable/` table. The xbox controls are inherited in this mode.

#### Maypad (Operator Panel) [`Port 1`]

|           | Col 0                | Col 1                | Col 2               | Col 3                |
|-----------|----------------------|----------------------|---------------------|----------------------|
| **Row 0** | `climberHome` (once) | `climberZero` (once) | `climberUp` (hold)  | `climberDown` (hold) |
| **Row 1** | -                    | `hoodHome` (once)    | `hoodTest` (hold)   | -                    |
| **Row 2** |                      | -                    | -                   | `shootAll` (hold)    |
| **Row 3** | `shooterTest` (hold) | `feederTest` (hold)  | `hopperTest` (hold) | `testIntake` (hold)  |
| **Row 4** | -                    | -                    | -                   | -                    |

| Key         | Action                       | Notes                                                |
|-------------|------------------------------|------------------------------------------------------|
| **row 0**   |                              |                                                      |
| `key(0, 0)` | Home Climber                 |                                                      |
| `key(0, 1)` | Zero Climber                 | Zeroes the encoders                                  |
| `key(0, 2)` | Climber Up                   |                                                      |
| `key(0, 3)` | Climber Down                 |                                                      |
| **row 1**   |                              |                                                      |
| `key(1, 0)` | -                            |                                                      |
| `key(1, 1)` | Home the Hood                |                                                      |
| `key(1, 2)` | Test hood position           | Tunable: `Shooter/Hood/Percent`                      |
| `key(1, 3)` | Shooter tune (hold)          | Runs shooter at tunables `Shooter/TuneRPM` and `Shooter/TuneHoodPercent` |
| **row 2**   |                              |                                                      |
| `key(2, 0)` | -                            |                                                      |
| `key(2, 1)` | -                            |                                                      |
| `key(2, 2)` | -                            |                                                      |
| `key(2, 3)` | Shoot + Feed + Hopper + Hood | Uses tunables below                                  |
| **row 3**   |                              |                                                      |
| `key(3, 0)` | Test shooter flywheel        | Tunable: `Shooter/RPM`                               |
| `key(3, 1)` | Test feeder                  | Tunable: `Shooter/Feeder/RPM`                        |
| `key(3, 2)` | Test hopper                  | Tunable: `Hopper/RPM`                                |
| `key(3, 3)` | Test intake                  | Tunable: `Intake/Pivot/Degrees`, `Intake/Roller/RPM` |
| **row 4**   |                              |                                                      |
| `key(4, 0)` | -                            |                                                      |
| `key(4, 1)` | -                            |                                                      |
| `key(4, 2)` | -                            |                                                      |
| `key(4, 3)` | -                            |                                                      |

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
