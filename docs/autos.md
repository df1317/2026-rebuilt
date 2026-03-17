# Autonomous

Auto routines use positions from `AutoPositions.java` and the fluent `AutoBuilder` API backed by Repulsor path planning.

## Positions

All positions are defined as blue-alliance `Pose2d` constants. Alliance flipping happens once at auto start via `Commands.defer()`.

| Constant         | Location                          | Rotation    |
|------------------|-----------------------------------|-------------|
| `HUB_FRONT`      | 0.9m in front of hub center       | Faces hub   |
| `CENTER_COLLECT`  | Field center                      | 0 degrees   |
| `CLIMB_LEFT`      | 2.5m left of hub center           | 0 degrees   |
| `CLIMB_RIGHT`     | 2.5m right of hub center          | 0 degrees   |

## Auto Chooser

Selected via SmartDashboard at `misc/Auto Chooser`:

| Option              | Behavior                                          |
|---------------------|---------------------------------------------------|
| Score Front         | Drive to hub front, stop within 15cm              |
| Score + Climb Left  | Score front, wait 1s, drive to left climb         |
| Score + Climb Right | Score front, wait 1s, drive to right climb        |
| Defence Only        | Drive to center and hold                          |
| Do Nothing          | No-op                                             |

All autos run at `AUTO_SPEED_SCALE` (set in `RobotContainer`), which scales the Repulsor planner's max speed for the duration of the auto.

## AutoBuilder API

`AutoBuilder` is a standalone class for composing auto sequences. Build custom autos by chaining steps:

```java
new AutoBuilder(repulsor)
    .driveTo(HUB_FRONT)                                    // drive to pose, end within 15cm
    .driveTo(pose, Meters.of(0.10))                        // custom tolerance
    .driveToFacing(pose, HUB_CENTER)                       // face a target on arrival
    .driveToFacing(pose, HUB_CENTER, Meters.of(0.10))      // facing + custom tolerance
    .driveToFacing(pose, HUB_CENTER, tol, Meters.of(1.0))  // facing + custom heading blend
    .driveToAndHold(CLIMB_LEFT)                             // drive and hold position
    .driveToAndHoldFacing(pose, HUB_CENTER)                 // hold + face target
    .waitSeconds(1.0)                                       // pause
    .run(shooter.shootCommand())                            // inject any WPILib command
    .build();
```

### Methods

| Method                                                          | Description                                              |
|-----------------------------------------------------------------|----------------------------------------------------------|
| `driveTo(Pose2d)`                                               | Navigate to pose, finish when within 15cm                |
| `driveTo(Pose2d, Distance)`                                     | Navigate to pose with custom tolerance                   |
| `driveToFacing(Pose2d, Translation2d)`                          | Navigate facing an aim target, finish within 15cm        |
| `driveToFacing(Pose2d, Translation2d, Distance)`                | Facing + custom tolerance                                |
| `driveToFacing(Pose2d, Translation2d, Distance, Distance)`      | Facing + custom tolerance + heading blend distance       |
| `driveToAndHold(Pose2d)`                                        | Navigate to pose and keep driving (no end trigger)       |
| `driveToAndHoldFacing(Pose2d, Translation2d)`                   | Hold position while facing an aim target                 |
| `driveToAndHoldFacing(Pose2d, Translation2d, Distance)`         | Hold + facing + custom heading blend distance            |
| `waitSeconds(double)`                                           | Pause for the given duration                             |
| `run(Command)`                                                  | Insert any WPILib command into the sequence              |
| `build()`                                                       | Returns the composed `Command`                           |

### How it works

1. Poses are stored as blue-alliance constants
2. `build()` wraps everything in `Commands.defer()` so the command is constructed lazily
3. At auto start, the current alliance is read **once** and all poses are flipped if red
4. The resolved poses are passed to `Repulsor.navigateTo()` which handles obstacle-aware pathing
5. The planner blends the robot heading toward the goal pose rotation over the last 0.75m

### Adding a new position

Add a `public static final Pose2d` to `AutoPositions`. For `driveTo`, the robot blends toward the goal rotation on approach. For `driveToFacing`, the rotation is computed from the aim target instead.

```java
public static final Pose2d MY_POSE = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90));
```

## Telemetry

Logged every cycle during `navigateTo`:

| Key                      | Type       | Destination | Description                        |
|--------------------------|------------|-------------|------------------------------------|
| `Repulsor/Target`        | `Pose2d`   | NT + DataLog | Current goal pose                  |
| `Repulsor/Trajectory`    | `Pose2d[]` | NT + DataLog | Forward-simulated path preview     |
| `Repulsor/Error`         | `double`   | DataLog      | Distance to goal (meters)          |
| `Repulsor/CommandedVx`   | `double`   | DataLog      | Field-relative X velocity (m/s)    |
| `Repulsor/CommandedVy`   | `double`   | DataLog      | Field-relative Y velocity (m/s)    |
| `Repulsor/CommandedOmega`| `double`   | DataLog      | Angular velocity (rad/s)           |
| `Repulsor/Stuck`         | `boolean`  | DataLog      | True when planner detects no progress |

`Target` and `Trajectory` use `DogLog.forceNt` so they're always on NetworkTables — add them as Field2d overlays in AdvantageScope.

## Speed Scaling

`RobotContainer.wrapAutoSpeed()` sets `AUTO_SPEED_SCALE` on the Repulsor drive tuning for the duration of the auto. The base max speed (5.14 m/s) is multiplied by this scale and further modulated by the heatmap and deceleration profile. Reset happens in `finallyDo` so teleop is unaffected.
