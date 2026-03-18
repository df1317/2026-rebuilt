# Autonomous

Auto routines use positions from `AutoPositions.java` and the fluent `AutoBuilder` API backed by Repulsor path planning.

## Positions

All positions are defined as blue-alliance `Pose2d` constants. Alliance flipping happens once at auto start via `Commands.defer()`.

| Constant              | Location                                  | Rotation    |
|-----------------------|-------------------------------------------|-------------|
| `HUB_CENTER`          | Hub center aimpoint                       | -           |
| `HUB_FRONT`           | 0.9m in front of hub center               | Faces hub   |
| `CENTER_COLLECT`      | Field center                              | 0 degrees   |
| `CLIMB_LEFT`          | Left climb position (1.062, 4.922)        | 0 degrees   |
| `CLIMB_LEFT_ENGAGE`   | 0.2m negative x from CLIMB_LEFT           | 0 degrees   |
| `CLIMB_RIGHT`         | Right climb position (1.062, 2.629)       | 180 degrees |
| `CLIMB_RIGHT_ENGAGE`  | 0.2m positive x from CLIMB_RIGHT          | 180 degrees |
| `CORNER_HIDE_NEAR_BALLS` | Left corner hide (0.749, 7.324)        | 0 degrees   |
| `CORNER_HIDE`         | Right corner hide (0.645, 0.645)          | 0 degrees   |

## Auto Chooser

Selected via SmartDashboard at `misc/Auto Chooser`:

| Option              | Behavior                                                            |
|---------------------|---------------------------------------------------------------------|
| Score Front         | Drive to hub front, stop within 15cm                                |
| Left Hide + Shoot   | Drive to left corner facing hub, shoot repeatedly                   |
| Right Hide + Shoot  | Drive to right corner facing hub, shoot repeatedly                  |
| Go to center        | Drive to center and hold                                            |
| Just Shoot          | Shoot from starting position for entire auto                        |
| Collect + Shoot x1  | Shoot to clear, collect from closest side, return to start, shoot   |
| Collect + Shoot x2  | Same as x1 but repeats the collect/return/shoot cycle a second time |
| Climb Left          | Drive to left climb, retract, engage, extend, hang                  |
| Climb Right         | Drive to right climb, retract, engage, extend, hang                 |
| Custom Chain        | Build a sequence from 6 step choosers on the dashboard              |
| Do Nothing          | No-op                                                               |

All autos run at `AUTO_SPEED_SCALE` (set in `Constants.AutoConstants`), which scales the Repulsor planner's max speed for the duration of the auto.

## Custom Chain

Select "Custom Chain" in the auto chooser to enable 6 step dropdowns (`misc/Auto Step 1` through `misc/Auto Step 6`). Each step can be:

| Step                   | Description                                                |
|------------------------|------------------------------------------------------------|
| `---`                  | Skip (default)                                             |
| Drive to Hub Front     | Navigate to hub front                                      |
| Drive to Center        | Navigate to field center and hold                          |
| Drive to Left Corner   | Navigate to left corner facing hub                         |
| Drive to Right Corner  | Navigate to right corner facing hub                        |
| Return to Start        | Navigate back to where the robot was at auto start         |
| Collect (closest side) | Navigate to the closest side of field center for pickup    |
| Shoot                  | Fire once                                                  |
| Shoot (repeat)         | Fire repeatedly until interrupted                          |
| Wait 0.5s / 1s / 2s   | Pause                                                      |
| Climb                  | Extend to top, then hang                                   |
| Descend                | Release, then retract to bottom                            |

The sequence is built when auto starts — chooser values are read at schedule time.

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
    .driveToStart()                                         // return to starting pose
    .driveToCollect()                                       // collect from closest field side
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
| `driveToStart()`                                                | Navigate back to the pose captured at auto start         |
| `driveToStart(Distance)`                                        | Return to start with custom tolerance                    |
| `driveToCollect()`                                              | Navigate to closest side of field center (1.5m offset)   |
| `driveToCollect(Distance)`                                      | Collect with custom tolerance                            |
| `waitSeconds(double)`                                           | Pause for the given duration                             |
| `speedScale(double)`                                            | Scale Repulsor max speed (0.0–1.0) for this auto         |
| `run(Command)`                                                  | Insert any WPILib command into the sequence              |
| `stepCount()`                                                   | Returns the current number of steps                      |
| `build()`                                                       | Returns the composed `Command`                           |

### How it works

1. Poses are stored as blue-alliance constants
2. `build()` wraps everything in `Commands.defer()` so the command is constructed lazily
3. At auto start, the current alliance is read **once** and all poses are flipped if red
4. `driveToStart()` and `driveToCollect()` capture the robot's actual field pose at schedule time (no flipping)
5. The resolved poses are passed to `Repulsor.navigateTo()` which handles obstacle-aware pathing
6. The planner blends the robot heading toward the goal pose rotation over the last 0.75m

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

`AutoBuilder.speedScale()` sets the speed scale on the Repulsor drive tuning for the duration of the auto. The base max speed (5.14 m/s) is multiplied by this scale and further modulated by the heatmap and deceleration profile. Reset happens in `finallyDo` so teleop is unaffected.
