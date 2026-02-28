package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** Dev/competition mode manager. Use DogLog.log() for telemetry (auto-disabled at FMS). */
public final class DevMode {

	private DevMode() {
	}

	/** True when not connected to FMS. */
	public static boolean isEnabled() {
		return !DriverStation.isFMSAttached();
	}

	public static boolean isCompetition() {
		return DriverStation.isFMSAttached();
	}

	public static boolean isSimulation() {
		return RobotBase.isSimulation();
	}
}
