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
		double currentRPM = shooter.motor.getVelocity().getValueAsDouble() * 60; // RPS to RPM
		double targetRPM = shooter.targetVelocity.in(RPM);

		double feederCurrentRPM = shooter.feederEncoder.getVelocity();
		double feederTargetRPM = shooter.targetFeederVelocity.in(RPM);

		// Status for LED strip
		DogLog.forceNt.log("Shooter/Status", getStatusColor().toHexString());

		// Shooter motor
		DogLog.log("Shooter/Motor/VelocityRPM", currentRPM);
		DogLog.log("Shooter/Motor/TargetVelocityRPM", targetRPM);
		DogLog.log("Shooter/Motor/VelocityErrorRPM", targetRPM - currentRPM);
		DogLog.log("Shooter/Motor/AtSpeed", shooter.isAtSpeed());
		DogLog.log("Shooter/Motor/CurrentAmps", shooter.motor.getStatorCurrent().getValueAsDouble());
		DogLog.log("Shooter/Motor/Voltage", shooter.motor.getMotorVoltage().getValueAsDouble());

		// Feeder motor
		DogLog.log("Shooter/Feeder/VelocityRPM", feederCurrentRPM);
		DogLog.log("Shooter/Feeder/TargetVelocityRPM", feederTargetRPM);
		DogLog.log("Shooter/Feeder/VelocityErrorRPM", feederTargetRPM - feederCurrentRPM);
		DogLog.log("Shooter/Feeder/CurrentAmps", shooter.feeder.getOutputCurrent());

		// Hood
		DogLog.log("Shooter/Hood/CurrentPercent",
				shooter.isHoodHomed() ? shooter.hoodEncoder.getPosition() / shooter.hoodMaxDeg : 0.0);
		DogLog.log("Shooter/Hood/TargetPercent", shooter.getTargetHoodPercent());
		DogLog.log("Shooter/Hood/CurrentAmps", shooter.hood.getOutputCurrent());
		DogLog.log("Shooter/Hood/Homed", shooter.isHoodHomed());
		DogLog.forceNt.log("Shooter/Hood/MaxDeg", shooter.hoodMaxDeg);
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
