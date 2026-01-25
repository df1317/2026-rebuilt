package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

/**
 * Subsystem controlling a single-motor flywheel shooter.
 */
public class ShooterSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry) ====================
	final SparkMax motor;
	final RelativeEncoder encoder;
	private final SparkClosedLoopController controller;
	private final Debouncer atSpeedDebouncer;
	private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
	private final SysIdRoutine sysIdRoutine;

	// ==================== Control State (package-private for telemetry) ====================
	AngularVelocity targetVelocity = RPM.of(0);

	// ==================== Telemetry ====================
	private final ShooterTelemetry telemetry;

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

		telemetry = new ShooterTelemetry(this);
	}

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

	private void populateLookupTable() {
		distanceToRPM.put(1.0, 2500.0);
		distanceToRPM.put(2.0, 3000.0);
		distanceToRPM.put(3.0, 3500.0);
		distanceToRPM.put(4.0, 4000.0);
		distanceToRPM.put(5.0, 4500.0);
		distanceToRPM.put(6.0, 5000.0);
	}

	@Override
	public void periodic() {
		telemetry.log();
	}

	// ==================== State Queries ====================

	public boolean isAtSpeed() {
		double error = Math.abs(targetVelocity.in(RPM) - encoder.getVelocity());
		boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

	public AngularVelocity getRPMForDistance(Distance distance) {
		double distanceMeters = distance.in(Meters);
		double clampedDistance = Math.max(1.0, Math.min(6.0, distanceMeters));
		return RPM.of(distanceToRPM.get(clampedDistance));
	}

	// ==================== Control Methods ====================

	public void setVelocityForDistance(Distance distance) {
		setVelocity(getRPMForDistance(distance));
	}

	public void stop() {
		targetVelocity = RPM.of(0);
		motor.stopMotor();
	}

	public void setVelocity(AngularVelocity velocity) {
		targetVelocity = velocity;
		controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	// ==================== Commands ====================

	public Command spinUpCommand(AngularVelocity velocity) {
		return Commands.runOnce(() -> setVelocity(velocity), this);
	}

	public Command spinUpForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> setVelocityForDistance(distance.get()), this);
	}

	public Command spinUpAndWaitCommand(AngularVelocity velocity) {
		return Commands.sequence(spinUpCommand(velocity), Commands.waitUntil(this::isAtSpeed));
	}

	public Command stopCommand() {
		return Commands.runOnce(this::stop, this);
	}

	public Command shootCommand(AngularVelocity velocity) {
		return Commands.startEnd(() -> setVelocity(velocity), this::stop, this);
	}

	public Command shootForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> setVelocityForDistance(distance.get()), this).finallyDo(this::stop);
	}

	// ==================== SysId ====================

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

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
