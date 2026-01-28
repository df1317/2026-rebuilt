package frc.robot.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.RPM;

/**
 * Handles telemetry logging for the shooter subsystem.
 */
public class ShooterTelemetry {

	private final ShooterSubsystem shooter;

	public ShooterTelemetry(ShooterSubsystem shooter) {
		this.shooter = shooter;
	}

	/**
	 * Logs all shooter telemetry data.
	 */
	public void log() {
		double currentRPM = shooter.encoder.getVelocity();
		double targetRPM = shooter.targetVelocity.in(RPM);

		// Status for LED strip
		DogLog.forceNt.log("Shooter/Status", getStatusColor().toHexString());

		// Velocity tracking
		DogLog.log("Shooter/VelocityRPM", currentRPM);
		DogLog.log("Shooter/TargetVelocityRPM", targetRPM);
		DogLog.log("Shooter/VelocityErrorRPM", targetRPM - currentRPM);
		DogLog.log("Shooter/AtSpeed", shooter.isAtSpeed());

		// Motor data
		DogLog.log("Shooter/MotorCurrentAmps", shooter.motor.getOutputCurrent());
		DogLog.log("Shooter/MotorVoltage", shooter.motor.getBusVoltage() * shooter.motor.getAppliedOutput());
	}

	private Color getStatusColor() {
		double targetRPM = shooter.targetVelocity.in(RPM);
		if (targetRPM <= 0) {
			return Color.kRed;
		} else if (shooter.isAtSpeed()) {
			return Color.kGreen;
		} else {
			return Color.kYellow;
		}
	}
}
