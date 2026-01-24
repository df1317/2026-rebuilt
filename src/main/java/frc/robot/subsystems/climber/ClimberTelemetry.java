package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Handles telemetry logging for the climber subsystem.
 */
public class ClimberTelemetry {

	private final SparkMax motor;
	private final RelativeEncoder encoder;

	public ClimberTelemetry(SparkMax motor, RelativeEncoder encoder) {
		this.motor = motor;
		this.encoder = encoder;
	}

	/**
	 * Logs all climber telemetry data.
	 */
	public void log(
			double measuredHeight,
			double goalHeight,
			double profilePosition,
			double profileVelocity,
			boolean atGoal,
			boolean atTop,
			boolean atBottom,
			Color statusColor) {

		// Status color for drivers
		DogLog.forceNt.log("Climber/Status", statusColor.toHexString());

		// Position & velocity
		DogLog.log("Climber/HeightMeters", measuredHeight);
		DogLog.log("Climber/GoalHeightMeters", goalHeight);
		DogLog.log("Climber/ProfilePositionMeters", profilePosition);
		DogLog.log("Climber/ProfileVelocityMps", profileVelocity);

		// State flags
		DogLog.log("Climber/AtGoal", atGoal);
		DogLog.log("Climber/AtTop", atTop);
		DogLog.log("Climber/AtBottom", atBottom);

		// Motor data
		DogLog.log("Climber/EncoderRotations", encoder.getPosition());
		DogLog.log("Climber/MotorCurrentAmps", motor.getOutputCurrent());
		DogLog.log("Climber/MotorVoltage", motor.getBusVoltage() * motor.getAppliedOutput());
	}
}
