package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HopperConstants.*;

/**
 * Subsystem controlling the hopper feed wheel.
 *
 * <p>
 * A simple wheel in a circular funnel that feeds balls one by one into the shooter.
 * Uses duty cycle control (not velocity) since precise speed isn't required.
 */
public class HopperSubsystem extends SubsystemBase {

	private final SparkMax motor;
	private final HopperTelemetry telemetry;

	private double currentSpeed = 0.0;

	public HopperSubsystem() {
		motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
		configureMotor();
		telemetry = new HopperTelemetry(motor);
	}

	private void configureMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kCoast)
				.smartCurrentLimit(CURRENT_LIMIT)
				.inverted(INVERTED);

		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		telemetry.log(currentSpeed, isRunning(), getStatusColor());
	}

	// ==================== State Query Methods ====================

	public boolean isRunning() {
		return Math.abs(currentSpeed) > 0.01;
	}

	public boolean isReversing() {
		return currentSpeed < -0.01;
	}

	public Color getStatusColor() {
		if (isReversing()) {
			return Color.kOrange;
		} else if (isRunning()) {
			return Color.kGreen;
		}
		return Color.kRed;
	}

	// ==================== Control Methods ====================

	public void setSpeed(double speed) {
		currentSpeed = speed;
		motor.set(speed);
	}

	public void feed() {
		setSpeed(FEED_SPEED);
	}

	public void reverse() {
		setSpeed(REVERSE_SPEED);
	}

	public void stop() {
		setSpeed(0);
	}

	// ==================== Commands ====================

	/**
	 * Runs the hopper to feed balls into the shooter.
	 * Stops when the command ends.
	 */
	public Command feedCommand() {
		return Commands.startEnd(this::feed, this::stop, this)
				.withName("Hopper Feed");
	}

	/**
	 * Reverses the hopper to unjam or eject balls.
	 * Stops when the command ends.
	 */
	public Command reverseCommand() {
		return Commands.startEnd(this::reverse, this::stop, this)
				.withName("Hopper Reverse");
	}

	public Command stopCommand() {
		return Commands.runOnce(this::stop, this)
				.withName("Hopper Stop");
	}
}
