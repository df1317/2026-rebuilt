package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field zone detection for 2026 REBUILT.
 *
 * <p>Field dimensions: 650.12" x 316.64" (16.52m x 8.04m)
 *
 * <p>Zone layout (X measured from the blue alliance wall):
 * <ul>
 *   <li>Blue Alliance Zone: 0" to 158.32"</li>
 *   <li>Neutral Zone: 158.32" to 491.8"</li>
 *   <li>Red Alliance Zone: 491.8" to 650.12"</li>
 * </ul>
 *
 * <p>Each alliance zone contains obstacle strips running parallel to the center line:
 * <pre>
 * Y=FIELD_WIDTH (top guardrail)
 * |----[TRENCH]----|
 * |----[ BUMP ]----| (extends from trench to hub)
 * |-----[HUB]------| (center Y)
 * |----[ BUMP ]----| (extends from hub to trench)
 * |----[TRENCH]----|
 * Y=0 (bottom guardrail)
 * </pre>
 */
public final class FieldZones {

	// Field dimensions
	/** Total field length in meters (X axis, from blue to red wall). */
	public static final double FIELD_LENGTH = Units.inchesToMeters(650.12);

	/** Total field width in meters (Y axis). */
	public static final double FIELD_WIDTH = Units.inchesToMeters(316.64);

	// Zone boundaries
	/** Depth of each alliance zone from their wall. */
	public static final double ALLIANCE_ZONE_DEPTH = Units.inchesToMeters(158.32);

	/** X coordinate where neutral zone begins (end of blue alliance zone). */
	public static final double NEUTRAL_ZONE_START = ALLIANCE_ZONE_DEPTH;

	/** X coordinate where the neutral zone ends (start of red alliance zone). */
	public static final double NEUTRAL_ZONE_END = FIELD_LENGTH - ALLIANCE_ZONE_DEPTH;

	/** X coordinate of the field center line. */
	public static final double CENTER_LINE = FIELD_LENGTH / 2;

	// Hub dimensions
	/** Width of the scoring hub structure. */
	public static final double HUB_WIDTH = Units.inchesToMeters(46.8);

	/** Position of the blue alliance hub. */
	public static final Pose2d HUB_POSE_BLUE = new Pose2d(
			new Translation2d(ALLIANCE_ZONE_DEPTH + HUB_WIDTH / 2, FIELD_WIDTH / 2),
			Rotation2d.kZero);

	/** Position of the red alliance hub. */
	public static final Pose2d HUB_POSE_RED = new Pose2d(
			new Translation2d(FIELD_LENGTH - HUB_POSE_BLUE.getX(), HUB_POSE_BLUE.getY()),
			Rotation2d.kZero);

	// Bump dimensions (6.5" tall ramps with 15-degree HDPE surface)
	/** Bump width in meters (73.0", extent along Y towards guardrails). */
	public static final double BUMP_WIDTH = Units.inchesToMeters(73.0);

	/** Bump depth in meters (44.4", extent along X parallel to center line). */
	public static final double BUMP_DEPTH = Units.inchesToMeters(44.4);

	// Trench dimensions (22.25" clearance under arm)
	// Width not needed - trenches fill remaining space from bump to guardrail
	/** Trench depth in meters (47.0", extent along X parallel to center line). */
	public static final double TRENCH_DEPTH = Units.inchesToMeters(47.0);

	// Calculated obstacle positions
	// Bumps and trenches form continuous strips from hub to guardrails
	// Layout (Y direction): guardrail -> trench -> bump -> hub -> bump -> trench -> guardrail
	private static final double HUB_Y_MIN = FIELD_WIDTH / 2 - HUB_WIDTH / 2;
	// Bump Y regions (adjacent to hub, extending BUMP_WIDTH towards guardrails)
	private static final double BUMP_BOTTOM_Y_MIN = HUB_Y_MIN - BUMP_WIDTH;
	// Trench Y regions (from where bumps end to the guardrails)
	private static final double TRENCH_BOTTOM_Y_MAX = BUMP_BOTTOM_Y_MIN;
	private static final double BUMP_BOTTOM_Y_MAX = HUB_Y_MIN;
	private static final double HUB_Y_MAX = FIELD_WIDTH / 2 + HUB_WIDTH / 2;
	private static final double BUMP_TOP_Y_MIN = HUB_Y_MAX;
	private static final double BUMP_TOP_Y_MAX = HUB_Y_MAX + BUMP_WIDTH;
	private static final double TRENCH_TOP_Y_MIN = BUMP_TOP_Y_MAX;
	// X bounds for bump/trench strips (centered on hub, using larger depth to cover both)
	private static final double STRIP_DEPTH = Math.max(BUMP_DEPTH, TRENCH_DEPTH);
	private static final double BLUE_STRIP_X_MIN = HUB_POSE_BLUE.getX() - STRIP_DEPTH / 2;
	private static final double BLUE_STRIP_X_MAX = HUB_POSE_BLUE.getX() + STRIP_DEPTH / 2;
	private static final double RED_STRIP_X_MIN = HUB_POSE_RED.getX() - STRIP_DEPTH / 2;
	private static final double RED_STRIP_X_MAX = HUB_POSE_RED.getX() + STRIP_DEPTH / 2;

	private FieldZones() {
	}

	/**
	 * Determines which zone contains the given pose.
	 *
	 * @param pose
	 * 		the robot pose to check
	 * @return the zone containing the pose
	 */
	public static Zone getZone(Pose2d pose) {
		return getZone(pose.getTranslation());
	}

	/**
	 * Determines which zone contains the given position.
	 *
	 * <p>Checks obstacles (bumps, trenches) before general alliance zones.
	 *
	 * @param position
	 * 		the field position to check
	 * @return the zone containing the position
	 */
	public static Zone getZone(Translation2d position) {
		double x = position.getX();
		double y = position.getY();

		if (x < 0 || x > FIELD_LENGTH || y < 0 || y > FIELD_WIDTH) {
			return Zone.OUT_OF_BOUNDS;
		}

		// Check blue alliance obstacles
		if (isInBlueTrench(x, y)) {
			return Zone.BLUE_TRENCH;
		}
		if (isInBlueBump(x, y)) {
			return Zone.BLUE_BUMP;
		}

		// Check red alliance obstacles
		if (isInRedTrench(x, y)) {
			return Zone.RED_TRENCH;
		}
		if (isInRedBump(x, y)) {
			return Zone.RED_BUMP;
		}

		// General zone detection
		if (x < NEUTRAL_ZONE_START) {
			return Zone.BLUE_ALLIANCE;
		}
		if (x > NEUTRAL_ZONE_END) {
			return Zone.RED_ALLIANCE;
		}

		return Zone.NEUTRAL;
	}

	private static boolean isInBlueTrench(double x, double y) {
		if (x < BLUE_STRIP_X_MIN || x > BLUE_STRIP_X_MAX) {
			return false;
		}
		return y < TRENCH_BOTTOM_Y_MAX || y > TRENCH_TOP_Y_MIN;
	}

	private static boolean isInRedTrench(double x, double y) {
		if (x < RED_STRIP_X_MIN || x > RED_STRIP_X_MAX) {
			return false;
		}
		return y < TRENCH_BOTTOM_Y_MAX || y > TRENCH_TOP_Y_MIN;
	}

	private static boolean isInBlueBump(double x, double y) {
		if (x < BLUE_STRIP_X_MIN || x > BLUE_STRIP_X_MAX) {
			return false;
		}
		return (y >= BUMP_BOTTOM_Y_MIN && y <= BUMP_BOTTOM_Y_MAX)
				|| (y >= BUMP_TOP_Y_MIN && y <= BUMP_TOP_Y_MAX);
	}

	private static boolean isInRedBump(double x, double y) {
		if (x < RED_STRIP_X_MIN || x > RED_STRIP_X_MAX) {
			return false;
		}
		return (y >= BUMP_BOTTOM_Y_MIN && y <= BUMP_BOTTOM_Y_MAX)
				|| (y >= BUMP_TOP_Y_MIN && y <= BUMP_TOP_Y_MAX);
	}

	/**
	 * Checks if the robot is in its own alliance zone (excluding obstacles).
	 *
	 * @param pose
	 * 		the robot pose
	 * @param alliance
	 * 		the robot's alliance
	 * @return true if in own alliance zone
	 */
	public static boolean isInOwnAllianceZone(Pose2d pose, Alliance alliance) {
		return getZone(pose).belongsTo(alliance);
	}

	/**
	 * Checks if the robot is in the opponent's alliance zone.
	 *
	 * @param pose
	 * 		the robot pose
	 * @param alliance
	 * 		the robot's alliance
	 * @return true if in opponent's alliance zone
	 */
	public static boolean isInOpponentAllianceZone(Pose2d pose, Alliance alliance) {
		Zone zone = getZone(pose);
		return zone.isAllianceZone() && !zone.belongsTo(alliance);
	}

	/**
	 * Checks if the robot is in the neutral zone.
	 *
	 * @param pose
	 * 		the robot pose
	 * @return true if in the neutral zone
	 */
	public static boolean isInNeutralZone(Pose2d pose) {
		return getZone(pose).isNeutral();
	}

	/**
	 * Checks if the robot is on a bump (should slow down).
	 *
	 * @param pose
	 * 		the robot pose
	 * @return true if on any bump
	 */
	public static boolean isOnBump(Pose2d pose) {
		return getZone(pose).isBump();
	}

	/**
	 * Checks if the robot is in a trench.
	 *
	 * @param pose
	 * 		the robot pose
	 * @return true if in any trench
	 */
	public static boolean isInTrench(Pose2d pose) {
		return getZone(pose).isTrench();
	}

	/**
	 * Checks if the robot is on any obstacle (bump or trench).
	 *
	 * @param pose
	 * 		the robot pose
	 * @return true if on any obstacle
	 */
	public static boolean isOnObstacle(Pose2d pose) {
		return getZone(pose).isObstacle();
	}

	/**
	 * Calculates the distance to the nearest zone boundary line.
	 *
	 * @param pose
	 * 		the robot pose
	 * @return distance in meters to the nearest zone boundary
	 */
	public static double getDistanceToNearestZoneBoundary(Pose2d pose) {
		double x = pose.getX();

		double distToBlueZoneLine = Math.abs(x - NEUTRAL_ZONE_START);
		double distToRedZoneLine = Math.abs(x - NEUTRAL_ZONE_END);
		double distToCenterLine = Math.abs(x - CENTER_LINE);

		return Math.min(distToBlueZoneLine, Math.min(distToRedZoneLine, distToCenterLine));
	}

	/**
	 * Checks if the robot has crossed the center line into the opponent's half.
	 *
	 * @param pose
	 * 		the robot pose
	 * @param alliance
	 * 		the robot's alliance
	 * @return true if on the opponent's side of center line
	 */
	public static boolean hasCrossedCenterLine(Pose2d pose, Alliance alliance) {
		double x = pose.getX();
		return alliance == Alliance.Red ? x < CENTER_LINE : x > CENTER_LINE;
	}

	/**
	 * Returns the hub pose for the given alliance.
	 *
	 * @param alliance
	 * 		the alliance
	 * @return the hub pose for that alliance
	 */
	public static Pose2d getHubPose(Alliance alliance) {
		return alliance == Alliance.Red ? HUB_POSE_RED : HUB_POSE_BLUE;
	}

	/**
	 * Represents distinct zones on the field.
	 *
	 * <p>Each zone knows which alliance it belongs to (if any) and whether it's an obstacle.
	 */
	public enum Zone {
		/** Blue alliance zone (clear area). */
		BLUE_ALLIANCE(Alliance.Blue, false, false),

		/** Blue alliance bump (6.5" elevated obstacle). */
		BLUE_BUMP(Alliance.Blue, true, false),

		/** Blue alliance trench (corner obstacle). */
		BLUE_TRENCH(Alliance.Blue, false, true),

		/** Red alliance zone (clear area). */
		RED_ALLIANCE(Alliance.Red, false, false),

		/** Red alliance bump (6.5" elevated obstacle). */
		RED_BUMP(Alliance.Red, true, false),

		/** Red alliance trench (corner obstacle). */
		RED_TRENCH(Alliance.Red, false, true),

		/** Center field neutral zone. */
		NEUTRAL(null, false, false),

		/** Outside field boundaries. */
		OUT_OF_BOUNDS(null, false, false);

		private final Alliance alliance;
		private final boolean bump;
		private final boolean trench;

		Zone(Alliance alliance, boolean bump, boolean trench) {
			this.alliance = alliance;
			this.bump = bump;
			this.trench = trench;
		}

		/**
		 * Returns the alliance this zone belongs to.
		 *
		 * @return the owning alliance, or null for neutral/out-of-bounds
		 */
		public Alliance getAlliance() {
			return alliance;
		}

		/**
		 * Checks if this zone belongs to the specified alliance.
		 *
		 * @param alliance
		 * 		the alliance to check
		 * @return true if this zone belongs to that alliance
		 */
		public boolean belongsTo(Alliance alliance) {
			return this.alliance == alliance;
		}

		/**
		 * Checks if this is an alliance zone (blue or red, including obstacles).
		 *
		 * @return true if this is an alliance-owned zone
		 */
		public boolean isAllianceZone() {
			return alliance != null;
		}

		/**
		 * Checks if this is a bump zone (should slow down when driving over).
		 *
		 * @return true if this is a bump
		 */
		public boolean isBump() {
			return bump;
		}

		/**
		 * Checks if this is a trench zone.
		 *
		 * @return true if this is a trench
		 */
		public boolean isTrench() {
			return trench;
		}

		/**
		 * Checks if this is any obstacle (bump or trench).
		 *
		 * @return true if this is an obstacle zone
		 */
		public boolean isObstacle() {
			return bump || trench;
		}

		/**
		 * Checks if this is the neutral zone.
		 *
		 * @return true if this is the neutral zone
		 */
		public boolean isNeutral() {
			return this == NEUTRAL;
		}

		/**
		 * Checks if this zone is within field boundaries.
		 *
		 * @return true if within bounds
		 */
		public boolean isInBounds() {
			return this != OUT_OF_BOUNDS;
		}
	}
}
