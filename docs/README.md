# Docs

Quick reference documentation for robot controls and configuration.

## Documentation

- [Logging & Error Handling](logging.md) - Centralized logging system with Elastic notifications
- [Shooter](shooter.md) - Flywheel, feeder, hood homing, and distance lookup table

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
| `Right Bumper`  | Fire (zone-aware)     | Hold to shoot + feed — only active in own alliance zone   |

#### Dashboard Choosers

| Chooser                | Options              | Description                                                   |
|------------------------|----------------------|---------------------------------------------------------------|
| `misc/Teleop Mode`     | Shuttle *(default)*, Shoot | **Shuttle**: auto-extends intake when in own zone. **Shoot**: no auto-intake. |

### Zone Automation (`TeleopZoneAutomation`)

Triggers activate automatically based on field position during teleop.

| Condition                                      | Action                                                   |
|------------------------------------------------|----------------------------------------------------------|
| Shuttle mode + in own alliance zone            | Auto-extend intake and run rollers                       |
| Right Bumper held + in own alliance zone       | Spin up shooter to hub distance, feed hopper concurrently |

The shooter speed is calculated from the live robot-to-hub distance using the distance LUT (see [Shooter docs](shooter.md#distance-lut)).

### Test Mode

In test mode, buttons on the Maypad run individual subsystem commands. Tunable values are adjustable live in Elastic or Glass under the `Tunable/` table.

#### Maypad (Operator Panel) [`Port 2`]

Firmware: [df1317/maypad-frc](https://github.com/df1317/maypad-frc) — grab the latest `.hex` from the Actions tab.

```
         Col 1            Col 2            Col 3            Col 4
Row 0  [ testFlywheel ] [ testFeeder   ] [ spinUpShoot  ] [ stop          ]  ← Shooter
Row 1  [ homeHood     ] [ testHood     ] [ testFullMtr  ] [ ---           ]  ← Hood
Row 2  [ extend       ] [ retract      ] [ runRoller    ] [ eject         ]  ← Intake
Row 3  [ testHopper   ] [ feed         ] [ ---          ] [ ---           ]  ← Hopper
Row 4  [ homeClimber  ] [ extend       ] [ retract      ] [ zero          ]  ← Climber
```

**Row 0 — Shooter**

| Button | Action | Trigger | Notes |
|--------|--------|---------|-------|
| `1` | Test flywheel | hold | Tunable: `Shooter/TestShooterRPM` |
| `2` | Test feeder | hold | Tunable: `Shooter/TestFeederRPM` |
| `3` | Spin up + shoot | hold | Spins up then feeds hopper |
| `4` | Stop shooter | once | |

**Row 1 — Hood**

| Button | Action | Trigger | Notes |
|--------|--------|---------|-------|
| `5` | Auto-home hood | once | Drives to both hard stops via stall detection; see [Shooter docs](shooter.md#homing) |
| `6` | Test hood position | once | Tunable: `Shooter/TestHoodPercent` |
| `7` | Test full motor | once | |

**Row 2 — Intake**

| Button | Action | Trigger | Notes |
|--------|--------|---------|-------|
| `9` | Extend | hold | |
| `10` | Retract | hold | |
| `11` | Run roller | hold | |
| `12` | Eject | hold | |

**Row 3 — Hopper**

| Button | Action | Trigger | Notes |
|--------|--------|---------|-------|
| `13` | Test hopper | hold | Tunable: `Hopper/TestRPM` |
| `14` | Feed | hold | |

**Row 4 — Climber**

| Button | Action | Trigger | Notes |
|--------|--------|---------|-------|
| `17` | Auto-home climber | once | Drives to bottom hard stop, zeros encoder |
| `18` | Extend | once | Goes to `MAX_HEIGHT` |
| `19` | Retract | once | Goes to `MIN_HEIGHT` |
| `20` | Zero encoder | once | Sets current position as zero |

#### Xbox Controller (Test fine-control)

| Binding | Action | Notes |
|---------|--------|-------|
| `Left Trigger (>0.7)` + Joystick L Y | Climber fine position | Position control; holds on release |

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
