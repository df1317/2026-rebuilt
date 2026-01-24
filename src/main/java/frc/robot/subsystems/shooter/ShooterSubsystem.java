package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

	private final SparkMax motor;
	private final RelativeEncoder encoder;
	private final SparkClosedLoopController controller;
	private final Debouncer atSpeedDebouncer;
	private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

	private double targetVelocityRPM = 0.0;

	public ShooterSubsystem() {
		motor = new SparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushless);
		encoder = motor.getEncoder();
		controller = motor.getClosedLoopController();
		atSpeedDebouncer = new Debouncer(ShooterConstants.AT_SPEED_DEBOUNCE_TIME, DebounceType.kRising);

		configureMotor();
		populateLookupTable();
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
		DogLog.forceNt.log("Shooter/Status", getStatusColor().toHexString());

		DogLog.log("Shooter/VelocityRPM", getVelocity());
		DogLog.log("Shooter/TargetVelocityRPM", targetVelocityRPM);
		DogLog.log("Shooter/AtSpeed", isAtSpeed());
	}

	public double getRPMForDistance(Distance distance) {
		return distanceToRPM.get(distance.in(Meters));
	}

	public void setVelocity(double rpm) {
		targetVelocityRPM = rpm;
		controller.setSetpoint(rpm, ControlType.kVelocity);
	}

	public void setVelocityForDistance(Distance distance) {
		setVelocity(getRPMForDistance(distance));
	}

	public void stop() {
		targetVelocityRPM = 0.0;
		motor.stopMotor();
	}

	public double getVelocity() {
		return encoder.getVelocity();
	}

	public boolean isAtSpeed() {
		double error = Math.abs(targetVelocityRPM - getVelocity());
		boolean withinTolerance = error < ShooterConstants.VELOCITY_TOLERANCE && targetVelocityRPM > 0;
		return atSpeedDebouncer.calculate(withinTolerance);
	}

	private static final Color COLOR_RED = new Color(255, 0, 0);
	private static final Color COLOR_YELLOW = new Color(255, 255, 0);
	private static final Color COLOR_GREEN = new Color(0, 255, 0);

	private Color getStatusColor() {
		if (targetVelocityRPM <= 0) {
			return COLOR_RED;
		} else if (isAtSpeed()) {
			return COLOR_GREEN;
		} else {
			return COLOR_YELLOW;
		}
	}

	// ==================== Commands ====================

	public Command spinUpCommand(double rpm) {
		return Commands.runOnce(() -> setVelocity(rpm), this);
	}

	public Command spinUpForDistanceCommand(Supplier<Distance> distance) {
		return Commands.run(() -> setVelocityForDistance(distance.get()), this);
	}

	public Command spinUpAndWaitCommand(double rpm) {
		return Commands.sequence(spinUpCommand(rpm), Commands.waitUntil(this::isAtSpeed));
	}

	public Command stopCommand() {
		return Commands.runOnce(this::stop, this);
	}

	public Command shootCommand(double rpm) {
		return Commands.startEnd(() -> setVelocity(rpm), this::stop, this);
	}

	public Command shootForDistanceCommand(Supplier<Distance> distance) {
		return Commands.startEnd(() -> setVelocityForDistance(distance.get()), this::stop, this);
	}
}
