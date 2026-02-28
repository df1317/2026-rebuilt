package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

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
		double currentRPM = shooter.motor.getVelocity().getValueAsDouble();
		double targetRPM = shooter.targetVelocity.in(RPM);

		double feederCurrentRPM = shooter.feederEncoder.getVelocity();
		double feederTargetRPM = shooter.targetFeederVelocity.in(RPM);

		// Status for LED strip
		DogLog.forceNt.log("Shooter/Status", getStatusColor().toHexString());

		// Velocity tracking
		DogLog.log("Shooter/VelocityRPM", currentRPM);
		DogLog.log("Shooter/TargetVelocityRPM", targetRPM);
		DogLog.log("Shooter/VelocityErrorRPM", targetRPM - currentRPM);
		DogLog.log("Shooter/AtSpeed", shooter.isAtSpeed());

		DogLog.log("Shooter/Feeder/VelocityRPM", feederCurrentRPM);
		DogLog.log("Shooter/Feeder/TargetVelocityRPM", feederTargetRPM);
		DogLog.log("Shooter/Feeder/VelocityErrorRPM", feederTargetRPM - feederCurrentRPM);
		// Motor data
		DogLog.log("Shooter/MotorCurrentAmps", shooter.motor.getStatorCurrent().getValueAsDouble());
		DogLog.log("Shooter/MotorVoltage",
				shooter.motor.getMotorVoltage().getValueAsDouble());

		DogLog.log("Shooter/Hood/CurrentPosition", shooter.hoodEncoder.getPosition());
		DogLog.log("Shooter/Hood/TargetPosition", shooter.targetHoodAngle);
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
