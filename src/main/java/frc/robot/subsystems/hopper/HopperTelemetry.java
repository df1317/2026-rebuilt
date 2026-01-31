package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Handles telemetry logging for the hopper subsystem.
 */
public class HopperTelemetry {

	private final SparkMax motor;

	public HopperTelemetry(SparkMax motor) {
		this.motor = motor;
	}

	/**
	 * Logs all hopper telemetry data.
	 */
	public void log(double speed, boolean running, Color statusColor) {
		// Status for LED strip
		DogLog.forceNt.log("Hopper/Status", statusColor.toHexString());

		// State
		DogLog.log("Hopper/Speed", speed);
		DogLog.log("Hopper/Running", running);

		// Motor data
		DogLog.log("Hopper/MotorCurrentAmps", motor.getOutputCurrent());
		DogLog.log("Hopper/MotorVoltage", motor.getBusVoltage() * motor.getAppliedOutput());
	}
}
