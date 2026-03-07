# Docs

Quick reference documentation for robot controls and configuration.

## Documentation

- [Logging & Error Handling](logging.md) - Centralized logging system with Elastic notifications

## Controls

### Teleop

#### Xbox Controller (Driver) [`Port 0`]

| Binding         | Action                | Description                                               |
|-----------------|-----------------------|-----------------------------------------------------------|
| `Left Stick`    | Drive translate       | Controls robot translation                                |
| `Right Stick`   | Drive rotate          | Controls robot rotation                                   |
| `Button A`      | Zero gyro             | Resets gyro heading (once)                                |
| `Left Bumper`   | Toggle field relative | Switches between field and robot relative driving         |
| `Button Y`      | Auto aim              | Aims at hub while allowing translation (hold)             |
| `Right Trigger` | Auto shoot            | Spins shooter + sets hood based on distance to hub (hold) |
| `Left Trigger`  | Intake                | Extends intake and runs rollers; stows on release (hold)  |

#### Xbox Controller (Operator) [`Port 1`]

| Binding         | Action           | Description                                          |
|-----------------|------------------|------------------------------------------------------|
| `Left Bumper`   | Shooter toggle   | Toggles shooter motor on/off at 3000 RPM             |
| `D-Pad Up`      | Increase speed   | Increases shooter speed by 100 RPM                   |
| `D-Pad Down`    | Decrease speed   | Decreases shooter speed by 100 RPM                   |
| `Left Trigger`  | Reverse shoot    | Spins shooter backwards to declog (hold)             |
| `Button A`      | Intake toggle    | Toggles intake extend/retract                        |
| `Button B`      | Reverse intake   | Runs intake rollers in reverse to eject (hold)       |
| `Button Y`      | Climber toggle   | Toggles climber extend/retract                       |
| `Right Bumper`  | Manual climber   | Enables manual climber control (hold)                |
| `Right Stick Y` | Climber speed    | Controls climber speed (while right bumper held)     |

### Test Mode

In test mode, hold a button on the driver controller to run a subsystem at values set via dashboard tunables. Adjust the tunable values in AdvantageScope or Glass under the `Tunable/Test/` table.

#### Xbox Controller (Driver) [`Port 0`]

| Binding    | Action        | Tunables                                           |
|------------|---------------|----------------------------------------------------|
| `Button A` | Test shooter  | `Test/ShooterRPM`, `Test/FeederRPM`, `Test/HoodAngleDeg` |
| `Button B` | Test intake   | `Test/IntakePivotDeg`, `Test/IntakeRollerRPM`      |
| `Button X` | Test climber  | `Test/ClimberHeightM`                              |

#### Dashboard Toggles

| Tunable                      | Default | Description                                        |
|------------------------------|---------|----------------------------------------------------|
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
| Shooter     | Motor       | `30` |

## DIO Constants

| Sensor | Port |
|--------|------|
| TBD    | TBD  |
