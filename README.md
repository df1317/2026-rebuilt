# 2026 Rebuilt

> [!NOTE]
> Full code documentation is available in [`docs`](/docs).

![rebuilt logo](https://www.firstinspires.org/hs-fs/hubfs/image-library/web/frc_rebuilt_1240x860.webp?width=630)

## What's this?

This is the code for 1317's 2026 robot for this years Rebuilt FRC game!

## Contributing / Code Style

### Formatting

> [!WARNING]
> Make sure to do these instructions before you start working on the code. (Looking at you Holden and Rebekah 👀)

#### IntelliJ IDEA

The Eclipse formatter profile is auto-imported and format-on-save is enabled. Just open the project and start coding -
formatting happens automatically.

If for some reason it doesn't work:

1. **Settings** → **Editor** → **Code Style** → **Java** → ⚙️ → **Import Scheme** → **Eclipse XML Profile**
2. Select `eclipse-formatter.xml`
3. **Settings** → **Tools** → **Actions on Save** → Enable "Reformat code" and "Optimize imports"

#### Non-Java Files (JSON, YAML, Markdown, etc.)

This project uses [Prettier](https://prettier.io/) for formatting config files, documentation, and other non-Java files.

Install dependencies:

```bash
bun install
```

Format all non-Java files:

```bash
bun format
```

### Commit Messages

Please follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) standard for commit messages.
This will help us keep track of changes and make it easier to generate changelogs. You are welcome to be funny in your
commit message but please make sure the commit type is correct.

Here are some examples of commit messages that are and aren't acceptable:

```plaintext
✅ feat: add new feature
✅ bug: fixed the photonvision latency bug 🐞
⛔ added new feature
⛔ fixed smthing
✅ docs: update readme with commit warnings
⛔ updated readme :doc
😵 fixed elevator limits and weired negitive velocity :bug (ha take the mr k i did the fnacy commits)
✅ chore: ran prettier on the code for the 11 millionth time
```

### Branches

Make branches early and often! If you are working on a new feature that isn't directly reliant on a side branch then
make a new branch and open a PR as quick as possible. Be wary of working on too many branches at once though; it can get
confusing. Aim to merge the branch once you have verified that the code works on the robot.

### Github Issues

Make issues for any current/near term tasks and identified bugs. Issues should not be created for tasks planned for too far in the future.

### Logging with DogLog

We use [DogLog](https://doglog.dev) for telemetry and logging. It automatically writes to DataLog files (`.wpilog`) for
AdvantageScope analysis and publishes to NetworkTables during development.

#### Key Concepts

- **`DogLog.log(key, value)`** - Development telemetry
    - Publishes to NetworkTables during practice/development
    - Auto-disables NT when FMS connected (competition mode)
    - Always writes to DataLog for post-match analysis

- **`DogLog.forceNt.log(key, value)`** - Competition essentials
    - Always publishes to NetworkTables (even at competition)
    - Use sparingly for driver dashboard critical values
    - Also writes to DataLog

- **`DevMode.isEnabled()`** - Gate expensive operations
    - Returns `true` in practice/development (not at FMS)
    - Use for Field2d updates, extra computations, debug visualizations
    - DogLog handles NT publishing automatically; use DevMode for non-logging overhead

- **`DogLog.tunable(key, defaultValue)`** - Runtime-adjustable values
    - Creates a NetworkTables subscriber for live tweaking (published under `Tunable/` table)
    - Auto-reverts to default value when FMS connected (competition mode)
    - Perfect for PID tuning, thresholds, and debug toggles
    - Values are logged to DataLog under `Robot/Tunable/`

#### Workflow

**Development:**

1. Connect laptop to robot network
2. Open [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
3. File → Connect to Robot → NetworkTables 4 → Enter robot IP
4. All `DogLog.log()` values stream live for debugging

**Post-Match Analysis:**

1. Download `.wpilog` files from roboRIO (`/home/lvuser/logs/`)
2. AdvantageScope → File → Open Files → Select `.wpilog`
3. Analyze all logged data with full history and Field2d visualization

**Competition:**

- Only `forceNt` values appear on Elastic dashboard
- Everything still logs to DataLog for post-match review

#### Example: Subsystem with Telemetry

```java
public class Elevator extends SubsystemBase {
	private final CANSparkMax motor;

	@Override
	public void periodic() {
		// Auto-disables at competition
		DogLog.log("Elevator/Position", motor.getEncoder().getPosition());
		DogLog.log("Elevator/Velocity", motor.getEncoder().getVelocity());
		DogLog.log("Elevator/Current", motor.getOutputCurrent());

		// Essential for driver dashboard (always published)
		DogLog.forceNt.log("Dash/ElevatorAtSetpoint", atSetpoint());
	}
}
```

#### Example: Tunable Values

```java
public class SwerveSubsystem extends SubsystemBase {
	// Toggle for enabling/disabling vision (editable in AdvantageScope/Glass)
	private final BooleanSubscriber visionEnabled = DogLog.tunable("Swerve/VisionEnabled", true);

	// PID tuning without redeploying code
	private final DoubleSubscriber driveKp = DogLog.tunable("Drive/kP", 0.1);

	@Override
	public void periodic() {
		if (visionEnabled.get()) {
			updateVision();
		}
	}
}
```

To change tunables at runtime:
1. Open AdvantageScope or Glass
2. Navigate to `Tunable/` table in NetworkTables
3. Edit values live - changes apply immediately
4. At FMS, tunables revert to defaults for safety

### Commands and Subsystems

As much as possible try to use the WPILIB built-ins for commands and subsystems. WPILib has some awesome docs
on [Commands](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html)
and [Command Compositions](https://docs.wpilib.org/en/latest/docs/software/commandbased/command-compositions.html) as
well as [Subsystems](https://docs.wpilib.org/en/latest/docs/software/commandbased/subsystems.html).

Here is a quick example of a subsystem with a command and then a command binding to a button:

```java
public class ExampleSubsystem extends SubsystemBase {

	private final CANSparkMax motor;

	public ExampleSubsystem() {
		motor = new CANSparkMax(0, MotorType.kBrushless);
	}

	public Command runMotorCommand(DoubleSupplier speed) {
		return new RunCommand(() -> motor.set(speed.getAsDouble()), this);
	}
}

```

```java
public class RobotContainer {

	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	private final CommandXboxController controller = new CommandXboxController(0);

	public RobotContainer() {
		controller.a().whileTrue(exampleSubsystem.runMotorCommand(() -> 0.5));
	}
}

```

<p align="center">
	<img src="https://raw.githubusercontent.com/taciturnaxolotl/carriage/main/.github/images/line-break.svg" />
</p>

<p align="center">
	<code>&copy 2026-present <a href="https://github.com/df1317">Digital Fusion FRC team 1317</a></code>
</p>

<p align="center">
	<a href="https://github.com/df1317/2026-rebuilt/blob/main/LICENSE.md"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=MIT&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>
