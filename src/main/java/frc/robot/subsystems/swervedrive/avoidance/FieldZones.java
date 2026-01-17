package frc.robot.subsystems.swervedrive.avoidance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Field zone detection for 2026 REBUILT.
 *
 * <p>
 * Field dimensions: 650.12" x 316.64" (16.52m x 8.04m)
 *
 * <p>
 * Zones (X from blue wall):
 * <ul>
 * <li>Blue Alliance Zone: 0 to 158.32"</li>
 * <li>Neutral Zone: 158.32" to 491.8"</li>
 * <li>Red Alliance Zone: 491.8" to 650.12"</li>
 * </ul>
 */
public final class FieldZones {

	public static final double FIELD_LENGTH = Units.inchesToMeters(650.12);
	public static final double FIELD_WIDTH = Units.inchesToMeters(316.64);

	public static final double ALLIANCE_ZONE_DEPTH = Units.inchesToMeters(158.32);
	public static final double NEUTRAL_ZONE_START = ALLIANCE_ZONE_DEPTH;
	public static final double NEUTRAL_ZONE_END = FIELD_LENGTH - ALLIANCE_ZONE_DEPTH;
	public static final double NEUTRAL_ZONE_CENTER = FIELD_LENGTH / 2;

	private static final double OUTPOST_AREA_WIDTH = Units.inchesToMeters(71.0);
	private static final double OUTPOST_AREA_DEPTH = Units.inchesToMeters(134.0);

	public enum Zone {
		BLUE_ALLIANCE, RED_ALLIANCE, NEUTRAL, BLUE_OUTPOST, RED_OUTPOST, OUT_OF_BOUNDS
	}

	private FieldZones() {
	}

	/**
	 * Get the zone the robot is currently in.
	 *
	 * @param pose
	 *          Current robot pose
	 * @return The zone containing the robot
	 */
	public static Zone getZone(Pose2d pose) {
		return getZone(pose.getTranslation());
	}

	/**
	 * Get the zone for a given position.
	 *
	 * @param position
	 *          Position on field
	 * @return The zone containing the position
	 */
	public static Zone getZone(Translation2d position) {
		double x = position.getX();
		double y = position.getY();

		if (x < 0 || x > FIELD_LENGTH || y < 0 || y > FIELD_WIDTH) {
			return Zone.OUT_OF_BOUNDS;
		}

		if (isInBlueOutpost(x, y)) {
			return Zone.BLUE_OUTPOST;
		}
		if (isInRedOutpost(x, y)) {
			return Zone.RED_OUTPOST;
		}

		if (x < NEUTRAL_ZONE_START) {
			return Zone.BLUE_ALLIANCE;
		}
		if (x > NEUTRAL_ZONE_END) {
			return Zone.RED_ALLIANCE;
		}

		return Zone.NEUTRAL;
	}

	private static boolean isInBlueOutpost(double x, double y) {
		return x < OUTPOST_AREA_DEPTH && y < OUTPOST_AREA_WIDTH;
	}

	private static boolean isInRedOutpost(double x, double y) {
		return x > FIELD_LENGTH - OUTPOST_AREA_DEPTH && y < OUTPOST_AREA_WIDTH;
	}

	/**
	 * Check if robot is in its own alliance zone.
	 *
	 * @param pose
	 *          Current robot pose
	 * @param isRedAlliance
	 *          True if on red alliance
	 * @return True if in own alliance zone
	 */
	public static boolean isInOwnAllianceZone(Pose2d pose, boolean isRedAlliance) {
		Zone zone = getZone(pose);
		return isRedAlliance
				? (zone == Zone.RED_ALLIANCE || zone == Zone.RED_OUTPOST)
				: (zone == Zone.BLUE_ALLIANCE || zone == Zone.BLUE_OUTPOST);
	}

	/**
	 * Check if robot is in opponent's alliance zone.
	 *
	 * @param pose
	 *          Current robot pose
	 * @param isRedAlliance
	 *          True if on red alliance
	 * @return True if in opponent's alliance zone
	 */
	public static boolean isInOpponentAllianceZone(Pose2d pose, boolean isRedAlliance) {
		Zone zone = getZone(pose);
		return isRedAlliance
				? (zone == Zone.BLUE_ALLIANCE || zone == Zone.BLUE_OUTPOST)
				: (zone == Zone.RED_ALLIANCE || zone == Zone.RED_OUTPOST);
	}

	/**
	 * Check if robot is in neutral zone.
	 *
	 * @param pose
	 *          Current robot pose
	 * @return True if in neutral zone
	 */
	public static boolean isInNeutralZone(Pose2d pose) {
		return getZone(pose) == Zone.NEUTRAL;
	}

	/**
	 * Get distance to the nearest zone boundary.
	 *
	 * @param pose
	 *          Current robot pose
	 * @return Distance in meters to nearest zone line
	 */
	public static double getDistanceToNearestZoneBoundary(Pose2d pose) {
		double x = pose.getX();

		double distToBlueZoneLine = Math.abs(x - NEUTRAL_ZONE_START);
		double distToRedZoneLine = Math.abs(x - NEUTRAL_ZONE_END);
		double distToCenterLine = Math.abs(x - NEUTRAL_ZONE_CENTER);

		return Math.min(distToBlueZoneLine, Math.min(distToRedZoneLine, distToCenterLine));
	}

	/**
	 * Check if robot has crossed the center line.
	 *
	 * @param pose
	 *          Current robot pose
	 * @param isRedAlliance
	 *          True if on red alliance
	 * @return True if on opponent's side of center line
	 */
	public static boolean hasCrossedCenterLine(Pose2d pose, boolean isRedAlliance) {
		double x = pose.getX();
		return isRedAlliance
				? x < NEUTRAL_ZONE_CENTER
				: x > NEUTRAL_ZONE_CENTER;
	}
}
