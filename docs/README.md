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

### Test Mode

In test mode, hold a button on the joystick to run individual motors at values set via dashboard tunables. Adjust the tunable values in Elastic or Glass under the `Tunable/Test/` table.

#### Joystick (Operator) [`Port 1`]

| Binding     | Action              | Tunables                |
|-------------|---------------------|-------------------------|
| `Button 3`  | Test hopper         | `Test/HopperRPM`       |
| `Button 4`  | Home hood           | *(auto — finds limits via stall detection)* |
| `Button 7`  | Test shooter motor  | `Test/ShooterRPM`      |
| `Button 8`  | Test feeder motor   | `Test/FeederRPM`       |
| `Button 9`  | Test hood angle     | `Test/HoodAngleDeg`    |
| `Button 10` | Test intake pivot   | `Test/IntakePivotDeg`  |
| `Button 11` | Test intake roller  | `Test/IntakeRollerRPM` |
| `Button 12` | Test climber        | `Test/ClimberHeightM`  |

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
| Hopper      | Motor       | `26` |
| Shooter     | Motor       | `30` |

## DIO Constants

| Sensor | Port |
|--------|------|
| TBD    | TBD  |
