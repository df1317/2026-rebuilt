package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

/**
 * Subsystem controlling a single-motor flywheel shooter.
 *
 * <p>
 * Provides closed-loop velocity control with distance-based RPM interpolation (1-6m range).
 *
 * @see frc.robot.Constants.ShooterConstants
 */
public class ShooterSubsystem extends SubsystemBase {

	private final SparkMax motor;
	private final RelativeEncoder encoder;
	private final SparkClosedLoopController controller;
	private final Debouncer atSpeedDebouncer;
	private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
	private final SysIdRoutine sysIdRoutine;

	private AngularVelocity targetVelocity = RPM.of(0);

	/**
	 * Creates a new ShooterSubsystem.
	 *
	 * <p>
	 * Initializes the motor, encoder, PID controller, and populates the distance-to-RPM
	 * lookup table. Motor configuration includes current limiting and coast mode for safety.
	 */
	public ShooterSubsystem() {
		motor = new SparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushless);
		encoder = motor.getEncoder();
		controller = motor.getClosedLoopController();
		atSpeedDebouncer = new Debouncer(ShooterConstants.AT_SPEED_DEBOUNCE_TIME, DebounceType.kRising);

		configureMotor();
		populateLookupTable();

		sysIdRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						null,
						ShooterConstants.SYSID_STEP_VOLTAGE,
						null,
						state -> DogLog.log("Shooter/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism(
						voltage -> motor.setVoltage(voltage.in(Volts)),
						null,
						this));
	}

	/**
	 * Configures the SparkMAX motor controller settings.
	 *
	 * <p>
	 * Sets idle mode, current limit, inversion, and closed-loop PID/feedforward gains
	 * from {@link frc.robot.Constants.ShooterConstants}.
	 */
	private void configureMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kCoast)
				.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
				.inverted(ShooterConstants.INVERTED);
		config.closedLoop
				.pid(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
		config.closedLoop.feedForward
				.kV(ShooterConstants.KV);

		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	/**
	 * Populates the distance-to-RPM interpolation lookup table.
	 *
	 * <p>
	 * Maps distances from 1-6 meters to corresponding flywheel RPM values.
	 * Values between entries are linearly interpolated. These values should be
	 * tuned based on field testing for your specific shooter geometry.
	 *
	 * <p>
	 * Current mapping:
	 * <ul>
	 * <li>1.0m → 2500 RPM</li>
	 * <li>2.0m → 3000 RPM</li>
	 * <li>3.0m → 3500 RPM</li>
	 * <li>4.0m → 4000 RPM</li>
	 * <li>5.0m → 4500 RPM</li>
	 * <li>6.0m → 5000 RPM</li>
	 * </ul>
	 */
	private void populateLookupTable() {
		// Distance (meters) -> RPM
		distanceToRPM.put(1.0, 2500.0);
		distanceToRPM.put(2.0, 3000.0);
		distanceToRPM.put(3.0, 3500.0);
		distanceToRPM.put(4.0, 4000.0);
		distanceToRPM.put(5.0, 4500.0);
		distanceToRPM.put(6.0, 5000.0);
	}

	/**
	 * Periodic method called every scheduler loop (typically 20ms).
	 *
	 * <p>
	 * Logs shooter telemetry to DogLog including:
	 * <ul>
	 * <li>Status color for LED feedback (forced to NetworkTables)</li>
	 * <li>Current and target velocity in RPM</li>
	 * <li>Velocity error</li>
	 * <li>At-speed status</li>
	 * <li>Motor current and voltage</li>
	 * </ul>
	 */
	@Override
	public void periodic() {
		AngularVelocity currentVelocity = getVelocity();

		// Status for LED strip
		DogLog.forceNt.log("Shooter/Status", getStatusColor().toHexString());

		// Velocity tracking
		DogLog.log("Shooter/VelocityRPM", currentVelocity.in(RPM));
		DogLog.log("Shooter/TargetVelocityRPM", targetVelocity.in(RPM));
		DogLog.log("Shooter/VelocityErrorRPM", targetVelocity.minus(currentVelocity).in(RPM));
		DogLog.log("Shooter/AtSpeed", isAtSpeed());

		// Motor data
		DogLog.log("Shooter/MotorCurrentAmps", motor.getOutputCurrent());
		DogLog.log("Shooter/MotorVoltage", motor.getBusVoltage() * motor.getAppliedOutput());
	}

	/**
	 * Calculates the target flywheel RPM for a given distance to target.
	 *
	 * <p>
	 * Uses linear interpolation between lookup table entries. Distances outside
	 * the 1-6 meter range are clamped to prevent unsafe extrapolation.
	 *
	 * @param distance
	 *          the distance to the target
	 * @return the interpolated flywheel velocity for that distance
	 */
	public AngularVelocity getRPMForDistance(Distance distance) {
		double distanceMeters = distance.in(Meters);
		double clampedDistance = Math.max(1.0, Math.min(6.0, distanceMeters));
		return RPM.of(distanceToRPM.get(clampedDistance));
	}

	/**
	 * Sets the flywheel velocity based on distance to target.
	 *
	 * <p>
	 * Convenience method that combines {@link #getRPMForDistance(Distance)} and
	 * {@link #setVelocity(AngularVelocity)}.
	 *
	 * @param distance
	 *          the distance to the target
	 */
	public void setVelocityForDistance(Distance distance) {
		setVelocity(getRPMForDistance(distance));
	}

	/**
	 * Stops the shooter motor and resets the target velocity to zero.
	 */
	public void stop() {
		targetVelocity = RPM.of(0);
		motor.stopMotor();
	}

	/**
	 * Gets the current flywheel velocity from the encoder.
	 *
	 * @return the current angular velocity of the flywheel
	 */
	public AngularVelocity getVelocity() {
		return RPM.of(encoder.getVelocity());
	}

	/**
	 * Sets the target flywheel velocity using closed-loop control.
	 *
	 * <p>
	 * Uses the SparkMAX onboard PID controller with feedforward for velocity control.
	 *
	 * @param velocity
	 *          the desired angular velocity
	 */
	public void setVelocity(AngularVelocity velocity) {
		targetVelocity = velocity;
		controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	/**
	 * Checks if the flywheel has reached the target velocity.
	 *
	 * <p>
	 * Returns true when:
	 * <ul>
	 * <li>The target velocity is greater than zero</li>
	 * <li>The velocity error is within tolerance (±100 RPM by default)</li>
	 * <li>The above conditions have been stable for the debounce period (0.1s)</li>
	 * </ul>
	 *
	 * <p>
	 * The debouncer prevents false positives from momentary velocity spikes.
	 *
	 * @return true if the flywheel is at the target speed and stable
	 */
	public boolean isAtSpeed() {
		double error = Math.abs(targetVelocity.minus(getVelocity()).in(RPM));
		boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

	/**
	 * Gets the current status color for LED feedback.
	 *
	 * <p>
	 * Color meanings:
	 * <ul>
	 * <li><b>Red:</b> Shooter idle (target velocity ≤ 0)</li>
	 * <li><b>Yellow:</b> Spinning up (target set but not yet at speed)</li>
	 * <li><b>Green:</b> Ready to shoot (at target speed)</li>
	 * </ul>
	 *
	 * @return the status color for LED indication
	 */
	public Color getStatusColor() {
		if (targetVelocity.in(RPM) <= 0) {
			return Color.kRed;
		} else if (isAtSpeed()) {
			return Color.kGreen;
		} else {
			return Color.kYellow;
		}
	}

	// ==================== Commands ====================

	/**
	 * Creates a command that sets the flywheel to a specific velocity.
	 *
	 * <p>
	 * This is an instant command that sets the velocity and completes immediately.
	 * The flywheel will continue spinning at the set velocity until another command
	 * changes it or {@link #stop()} is called.
	 *
	 * @param velocity
	 *          the target flywheel velocity
	 * @return a command that sets the flywheel velocity
	 */
	public Command spinUpCommand(AngularVelocity velocity) {
		return Commands.runOnce(() -> setVelocity(velocity), this);
	}

	/**
	 * Creates a command that continuously adjusts flywheel velocity based on distance.
	 *
	 * <p>
	 * This command runs continuously, updating the target velocity each loop based
	 * on the supplied distance. Useful for tracking a moving target or maintaining
	 * optimal velocity as the robot moves.
	 *
	 * <p>
	 * The command does not stop the shooter when it ends; use {@link #shootForDistanceCommand}
	 * if automatic stopping is desired.
	 *
	 * @param distance
	 *          a supplier providing the current distance to target
	 * @return a command that continuously adjusts velocity based on distance
	 */
	public Command spinUpForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> setVelocityForDistance(distance.get()), this);
	}

	/**
	 * Creates a command that spins up the flywheel and waits until it reaches speed.
	 *
	 * <p>
	 * This command blocks until {@link #isAtSpeed()} returns true, making it useful
	 * in command sequences where you need to ensure the shooter is ready before
	 * proceeding (e.g., before feeding a game piece).
	 *
	 * @param velocity
	 *          the target flywheel velocity
	 * @return a command that sets velocity and waits for the flywheel to reach speed
	 */
	public Command spinUpAndWaitCommand(AngularVelocity velocity) {
		return Commands.sequence(spinUpCommand(velocity), Commands.waitUntil(this::isAtSpeed));
	}

	/**
	 * Creates a command that stops the shooter.
	 *
	 * <p>
	 * This is an instant command that calls {@link #stop()}.
	 *
	 * @return a command that stops the shooter motor
	 */
	public Command stopCommand() {
		return Commands.runOnce(this::stop, this);
	}

	/**
	 * Creates a command that runs the shooter at a fixed velocity until interrupted.
	 *
	 * <p>
	 * The shooter automatically stops when the command ends (via button release or
	 * interruption). This is the primary command for fixed-distance shooting.
	 *
	 * <p>
	 * Bound to Y button in {@link frc.robot.RobotContainer} at 3500 RPM.
	 *
	 * @param velocity
	 *          the target flywheel velocity
	 * @return a command that runs the shooter and stops on end
	 */
	public Command shootCommand(AngularVelocity velocity) {
		return Commands.startEnd(() -> setVelocity(velocity), this::stop, this);
	}

	/**
	 * Creates a command that continuously adjusts velocity based on distance and stops on end.
	 *
	 * <p>
	 * Combines distance-based velocity adjustment with automatic stopping when the
	 * command ends. This is the primary command for variable-distance shooting.
	 *
	 * <p>
	 * Bound to X button in {@link frc.robot.RobotContainer} using distance to alliance hub.
	 *
	 * @param distance
	 *          a supplier providing the current distance to target
	 * @return a command that adjusts velocity based on distance and stops on end
	 */
	public Command shootForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> setVelocityForDistance(distance.get()), this).finallyDo(this::stop);
	}

	// ==================== SysId ====================

	/**
	 * Creates a quasistatic SysId test command.
	 *
	 * <p>
	 * Quasistatic tests slowly ramp voltage to measure the system's static friction (kS)
	 * and velocity constant (kV). Run in both directions for accurate characterization.
	 *
	 * @param direction
	 *          the direction to run the test (forward or reverse)
	 * @return a command that runs the quasistatic SysId test
	 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/">WPILib
	 *      SysId Documentation</a>
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	/**
	 * Creates a dynamic SysId test command.
	 *
	 * <p>
	 * Dynamic tests apply a step voltage to measure the system's acceleration constant (kA).
	 * The step voltage is configured in {@link frc.robot.Constants.ShooterConstants#SYSID_STEP_VOLTAGE}.
	 *
	 * @param direction
	 *          the direction to run the test (forward or reverse)
	 * @return a command that runs the dynamic SysId test
	 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/">WPILib
	 *      SysId Documentation</a>
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	/**
	 * Creates a command that runs a complete SysId characterization routine.
	 *
	 * <p>
	 * Executes all four SysId tests in sequence with 1-second pauses between:
	 * <ol>
	 * <li>Quasistatic forward</li>
	 * <li>Quasistatic reverse</li>
	 * <li>Dynamic forward</li>
	 * <li>Dynamic reverse</li>
	 * </ol>
	 *
	 * <p>
	 * After running, export the logged data and analyze with the WPILib SysId tool
	 * to calculate feedforward constants (kS, kV, kA).
	 *
	 * @return a command that runs the complete SysId characterization
	 */
	public Command sysIdFullCommand() {
		return Commands.sequence(
				sysIdQuasistatic(SysIdRoutine.Direction.kForward),
				Commands.waitSeconds(1),
				sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
				Commands.waitSeconds(1),
				sysIdDynamic(SysIdRoutine.Direction.kForward),
				Commands.waitSeconds(1),
				sysIdDynamic(SysIdRoutine.Direction.kReverse));
	}
}
