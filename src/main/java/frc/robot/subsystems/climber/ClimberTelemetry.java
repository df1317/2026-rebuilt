package frc.robot.subsystems.climber;

import dev.doglog.DogLog;

/**
 * Handles telemetry logging for the climber subsystem.
 */
public class ClimberTelemetry {

	private final ClimberSubsystem climber;

	public ClimberTelemetry(ClimberSubsystem climber) {
		this.climber = climber;
	}

	/**
	 * Logs all climber telemetry data.
	 */
	public void log() {
		// Status color for drivers
		DogLog.forceNt.log("Climber/Status", climber.getStatusColor().toHexString());

		// Position & velocity
		DogLog.log("Climber/HeightMeters", climber.getHeightMeters());
		DogLog.log("Climber/GoalHeightMeters", climber.goalState.position);
		DogLog.log("Climber/ProfilePositionMeters", climber.currentState.position);
		DogLog.log("Climber/ProfileVelocityMps", climber.currentState.velocity);

		// State flags
		DogLog.log("Climber/AtGoal", climber.isAtGoal());
		DogLog.log("Climber/AtTop", climber.isAtTop());
		DogLog.log("Climber/AtBottom", climber.isAtBottom());

		// Motor data
		DogLog.log("Climber/EncoderRotations", climber.encoder.getPosition());
		DogLog.log("Climber/MotorCurrentAmps", climber.motorLeft.getOutputCurrent());
		DogLog.log("Climber/MotorVoltage", climber.motorLeft.getBusVoltage() * climber.motorLeft.getAppliedOutput());
	}
}
