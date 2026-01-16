package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Centralized dev/competition mode manager.
 *
 * <p>Note: DogLog handles NT publishing automatically (disables when FMS connected).
 * This class is for additional dev-only logic like Field2d debug overlays or expensive computations that shouldn't run
 * during matches.
 *
 * <p>Logging strategy:
 * <ul>
 *   <li>Use {@code DogLog.log()} for debug telemetry (auto-disabled at FMS)</li>
 *   <li>Use {@code DogLog.forceNt.log()} for Elastic essentials (always published)</li>
 *   <li>Use {@code DevMode.isEnabled()} for expensive non-logging operations</li>
 * </ul>
 */
public final class DevMode {

	private DevMode() {
	}

	/**
	 * Returns true when in dev/practice mode (not connected to FMS). Use for expensive operations that shouldn't run
	 * during matches (e.g., Field2d debug overlays, extra computations).
	 *
	 * <p>Note: For logging use DogLog.log() - it handles this automatically.
	 */
	public static boolean isEnabled() {
		return !DriverStation.isFMSAttached();
	}

	/**
	 * Returns true when connected to FMS (competition mode).
	 */
	public static boolean isCompetition() {
		return DriverStation.isFMSAttached();
	}

	/**
	 * Returns true when running in simulation.
	 */
	public static boolean isSimulation() {
		return RobotBase.isSimulation();
	}
}
