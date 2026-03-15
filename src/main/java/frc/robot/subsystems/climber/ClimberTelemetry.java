package frc.robot.subsystems.climber;

import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ClimberConstants.*;

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
		DogLog.forceNt.log("Climber/HeightMeters", climber.getHeightMeters());
		DogLog.log("Climber/GoalHeightMeters", climber.goalState.position);
		DogLog.log("Climber/ProfilePositionMeters", climber.currentState.position);
		DogLog.log("Climber/ProfileVelocityMps", climber.currentState.velocity);

		// Named position
		DogLog.forceNt.log("Climber/Position", getNamedPosition(climber.goalState.position));

		// State flags
		DogLog.log("Climber/AtGoal", climber.isAtGoal());
		DogLog.log("Climber/AtTop", climber.isAtTop());
		DogLog.log("Climber/AtBottom", climber.isAtBottom());

		// Motor data
		DogLog.log("Climber/EncoderRotations", climber.motorLeft.getPosition().getValueAsDouble());
		DogLog.log("Climber/MotorCurrentAmps", climber.motorLeft.getStatorCurrent().getValueAsDouble());
		DogLog.log("Climber/MotorVoltage", climber.motorLeft.getMotorVoltage().getValueAsDouble());

		DogLog.log("Climber/isStalled", climber.isStalled);
		DogLog.forceNt.log("Climber/IsHomed", climber.isHomed);
		DogLog.log("Climber/IsHoming", climber.isHoming);
	}

	private String getNamedPosition(double goalMeters) {
		double tol = 0.01;
		if (Math.abs(goalMeters - MIN_HEIGHT.in(Meters)) < tol)
			return "Bottom";
		if (Math.abs(goalMeters - MAX_HEIGHT.in(Meters)) < tol)
			return "Top";
		if (Math.abs(goalMeters - HANG_HEIGHT.in(Meters)) < tol)
			return "Hang";
		if (Math.abs(goalMeters - RELEASE_HEIGHT.in(Meters)) < tol)
			return "Release";
		return "Custom";
	}
}
