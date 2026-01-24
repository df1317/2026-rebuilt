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

public class ShooterSubsystem extends SubsystemBase {

	private final SparkMax motor;
	private final RelativeEncoder encoder;
	private final SparkClosedLoopController controller;
	private final Debouncer atSpeedDebouncer;
	private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
	private final SysIdRoutine sysIdRoutine;

	private AngularVelocity targetVelocity = RPM.of(0);

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
		// Distance (meters) -> RPM
		// Tune these values based on testing
		distanceToRPM.put(1.0, 2500.0);
		distanceToRPM.put(2.0, 3000.0);
		distanceToRPM.put(3.0, 3500.0);
		distanceToRPM.put(4.0, 4000.0);
		distanceToRPM.put(5.0, 4500.0);
		distanceToRPM.put(6.0, 5000.0);
	}

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

	public AngularVelocity getRPMForDistance(Distance distance) {
		return RPM.of(distanceToRPM.get(distance.in(Meters)));
	}

	public void setVelocityForDistance(Distance distance) {
		setVelocity(getRPMForDistance(distance));
	}

	public void stop() {
		targetVelocity = RPM.of(0);
		motor.stopMotor();
	}

	public AngularVelocity getVelocity() {
		return RPM.of(encoder.getVelocity());
	}

	public void setVelocity(AngularVelocity velocity) {
		targetVelocity = velocity;
		controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public boolean isAtSpeed() {
		double error = Math.abs(targetVelocity.minus(getVelocity()).in(RPM));
		boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE.in(RPM) && targetVelocity.in(RPM) > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

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
		return Commands.startEnd(() -> setVelocityForDistance(distance.get()), this::stop, this);
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
