# Docs

Quick reference documentation for robot controls and configuration.

## Documentation

- [Logging & Error Handling](logging.md) - Centralized logging system with Elastic notifications
- [Shooter](shooter.md) - Flywheel, feeder, hood homing, and distance lookup table

## Controls

### Teleop

#### Xbox Controller (Driver) [`Port 0`]

| Binding        | Action                | Description                                             |
|----------------|-----------------------|---------------------------------------------------------|
| `Left Stick`   | Drive translate       | Controls robot translation                              |
| `Right Stick`  | Drive rotate          | Controls robot rotation                                 |
| `Button A`     | Zero gyro             | Resets gyro heading (once)                              |
| `Left Bumper`  | Toggle field relative | Switches between field and robot relative driving       |
| `Button Y`     | Auto aim              | Aims at hub while allowing translation (hold)           |
| `Right Bumper` | Fire (zone-aware)     | Hold to shoot + feed — only active in own alliance zone |

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

#### Maypad (Operator Panel) [`Port 2`]

Firmware: [df1317/maypad-frc](https://github.com/df1317/maypad-frc) — grab the latest `.hex` from the Actions tab.

```
         Col 0            Col 1            Col 2            Col 3
Row 0  [ testFlywheel ] [ testFeeder   ] [ spinUpShoot  ] [ stop          ]  ← Shooter
Row 1  [ homeHood     ] [ testHood     ] [ testFullMtr  ] [ ---           ]  ← Hood
Row 2  [ extend       ] [ retract      ] [ runRoller    ] [ eject         ]  ← Intake
Row 3  [ testHopper   ] [ feed         ] [ aimBangBang  ] [ aimPID        ]  ← Hopper / Aim
Row 4  [ homeClimber  ] [ extend       ] [ retract      ] [ zero          ]  ← Climber
```

**Row 0 — Shooter**

| key(row, col) | Action          | Trigger | Notes                         |
|---------------|-----------------|---------|-------------------------------|
| `key(0, 0)`   | Test flywheel   | hold    | Tunable: `Shooter/RPM`        |
| `key(0, 1)`   | Test feeder     | hold    | Tunable: `Shooter/Feeder/RPM` |
| `key(0, 2)`   | Spin up + shoot | hold    | Spins up then feeds hopper    |
| `key(0, 3)`   | Stop shooter    | once    |                               |

**Row 1 — Hood**

| key(row, col) | Action             | Trigger | Notes                                                                                |
|---------------|--------------------|---------|--------------------------------------------------------------------------------------|
| `key(1, 0)`   | Auto-home hood     | once    | Drives to both hard stops via stall detection; see [Shooter docs](shooter.md#homing) |
| `key(1, 1)`   | Test hood position | once    | Tunable: `Shooter/Hood/Percent`                                                      |
| `key(1, 2)`   | Test full motor    | once    |                                                                                      |

**Row 2 — Intake**

| key(row, col) | Action     | Trigger | Notes |
|---------------|------------|---------|-------|
| `key(2, 0)`   | Extend     | hold    |       |
| `key(2, 1)`   | Retract    | hold    |       |
| `key(2, 2)`   | Run roller | hold    |       |
| `key(2, 3)`   | Eject      | hold    |       |

**Row 3 — Hopper / Aim**

| key(row, col) | Action            | Trigger | Notes                                                                      |
|---------------|-------------------|---------|----------------------------------------------------------------------------|
| `key(3, 0)`   | Test hopper       | hold    | Tunable: `Hopper/RPM`                                                      |
| `key(3, 1)`   | Feed              | hold    |                                                                            |
| `key(3, 2)`   | Aim at hub (bang-bang) | hold | Stepped speed controller — 4 tiers based on angle error. Logs to `Aim/`  |
| `key(3, 3)`   | Aim at hub (PID)  | hold    | `ProfiledPIDController` with trapezoidal constraints. Logs to `AimPID/`   |

**Row 4 — Climber**

| key(row, col) | Action            | Trigger | Notes                                     |
|---------------|-------------------|---------|-------------------------------------------|
| `key(4, 0)`   | Auto-home climber | once    | Drives to bottom hard stop, zeros encoder |
| `key(4, 1)`   | Extend            | once    | Goes to `MAX_HEIGHT`                      |
| `key(4, 2)`   | Retract           | once    | Goes to `MIN_HEIGHT`                      |
| `key(4, 3)`   | Zero encoder      | once    | Sets current position as zero             |

#### Xbox Controller (Test fine-control)

| Binding                              | Action                | Notes                              |
|--------------------------------------|-----------------------|------------------------------------|
| `Left Trigger (>0.7)` + Joystick L Y | Climber fine position | Position control; holds on release |

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
