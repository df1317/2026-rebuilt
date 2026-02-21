package frc.robot.subsystems.hopper;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.HopperConstants.*;

/**
 * Handles telemetry logging for the hopper subsystem.
 */
public class HopperTelemetry {

	private final HopperSubsystem hopper;

	public HopperTelemetry(HopperSubsystem hopper) {
		this.hopper = hopper;
	}

	/**
	 * Logs all hopper telemetry data.
	 */
	public void log() {
		double hopperVelocityRPM = hopper.hopperEncoder.getVelocity();
		double hopperTargetRPM = hopper.targetHopperVelocity.in(RPM);

		// Status color for the drivers
		DogLog.forceNt.log("Hopper/Status", getStatusColor().toHexString());

		// Hopper tracking
		DogLog.log("Hopper/HopperVelocityRPM", hopperVelocityRPM);
		DogLog.log("Hopper/HopperTargetRPM", hopperTargetRPM);
		DogLog.log("Hopper/HopperErrorRPM", hopperTargetRPM - hopperVelocityRPM);
		DogLog.log("Hopper/HopperRunning", isHopperRunning());
		DogLog.log("Hopper/HopperAtSpeed", hopper.isHopperAtSpeed());

		// Motor data
		DogLog.log("Hopper/HopperCurrentAmps", hopper.hopperMotor.getOutputCurrent());
		DogLog.log("Hopper/HopperVoltage", hopper.hopperMotor.getBusVoltage() * hopper.hopperMotor.getAppliedOutput());
	}

	private boolean isHopperRunning() {
		return Math.abs(hopper.targetHopperVelocity.in(RPM)) > HOPPER_VELOCITY_TOLERANCE.in(RPM);
	}

	private Color getStatusColor() {
		double targetRPM = hopper.targetHopperVelocity.in(RPM);
		if (targetRPM > 0) {
			return hopper.isHopperAtSpeed() ? Color.kGreen : Color.kYellow;
		} else if (targetRPM < 0) {
			return Color.kOrange;
		}
		return Color.kRed;
	}
}
