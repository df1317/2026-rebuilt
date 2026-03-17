package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field zone detection for 2026 REBUILT. Field: 650.12" x 316.64" (16.52m x 8.04m).
 */
public final class FieldZones {

	public static final double FIELD_LENGTH = Units.inchesToMeters(650.12);
	public static final double FIELD_WIDTH = Units.inchesToMeters(316.64);

	public static final double ALLIANCE_ZONE_DEPTH = Units.inchesToMeters(158.32);
	public static final double NEUTRAL_ZONE_START = ALLIANCE_ZONE_DEPTH;
	public static final double NEUTRAL_ZONE_END = FIELD_LENGTH - ALLIANCE_ZONE_DEPTH;
	public static final double CENTER_LINE = FIELD_LENGTH / 2;

	public static final double HUB_WIDTH = Units.inchesToMeters(46.8);

	public static final Pose2d HUB_POSE_BLUE = new Pose2d(
			new Translation2d(ALLIANCE_ZONE_DEPTH + HUB_WIDTH / 2, FIELD_WIDTH / 2),
			Rotation2d.kZero);

	public static final Pose2d HUB_POSE_RED = new Pose2d(
			new Translation2d(FIELD_LENGTH - HUB_POSE_BLUE.getX(), HUB_POSE_BLUE.getY()),
			Rotation2d.kZero);

	public static final Pose2d CENTER_OF_ALLIANCE_BLUE = new Pose2d(
			new Translation2d(ALLIANCE_ZONE_DEPTH / 2, FIELD_WIDTH / 2),
			Rotation2d.kZero);

	public static final Pose2d CENTER_OF_ALLIANCE_RED = new Pose2d(
			new Translation2d(FIELD_LENGTH - CENTER_OF_ALLIANCE_BLUE.getX(), CENTER_OF_ALLIANCE_BLUE.getY()),
			Rotation2d.kZero);

	// Bump: 6.5" tall, 73.0" wide (Y), 44.4" deep (X)
	public static final double BUMP_WIDTH = Units.inchesToMeters(73.0);
	public static final double BUMP_DEPTH = Units.inchesToMeters(44.4);

	// Trench: 22.25" clearance, 47.0" deep (X)
	public static final double TRENCH_DEPTH = Units.inchesToMeters(47.0);

	// Calculated obstacle positions
	private static final double HUB_Y_MIN = FIELD_WIDTH / 2 - HUB_WIDTH / 2;
	private static final double BUMP_BOTTOM_Y_MIN = HUB_Y_MIN - BUMP_WIDTH;
	private static final double TRENCH_BOTTOM_Y_MAX = BUMP_BOTTOM_Y_MIN;
	private static final double BUMP_BOTTOM_Y_MAX = HUB_Y_MIN;
	private static final double HUB_Y_MAX = FIELD_WIDTH / 2 + HUB_WIDTH / 2;
	private static final double BUMP_TOP_Y_MIN = HUB_Y_MAX;
	private static final double BUMP_TOP_Y_MAX = HUB_Y_MAX + BUMP_WIDTH;
	private static final double TRENCH_TOP_Y_MIN = BUMP_TOP_Y_MAX;
	private static final double STRIP_DEPTH = Math.max(BUMP_DEPTH, TRENCH_DEPTH);
	private static final double BLUE_STRIP_X_MIN = HUB_POSE_BLUE.getX() - STRIP_DEPTH / 2;
	private static final double BLUE_STRIP_X_MAX = HUB_POSE_BLUE.getX() + STRIP_DEPTH / 2;
	private static final double RED_STRIP_X_MIN = HUB_POSE_RED.getX() - STRIP_DEPTH / 2;
	private static final double RED_STRIP_X_MAX = HUB_POSE_RED.getX() + STRIP_DEPTH / 2;

	private FieldZones() {
	}

	public static Zone getZone(Pose2d pose) {
		return getZone(pose.getTranslation());
	}

	public static Zone getZone(Translation2d position) {
		double x = position.getX();
		double y = position.getY();

		if (x < 0 || x > FIELD_LENGTH || y < 0 || y > FIELD_WIDTH) {
			return Zone.OUT_OF_BOUNDS;
		}

		if (isInBlueTrench(x, y))
			return Zone.BLUE_TRENCH;
		if (isInBlueBump(x, y))
			return Zone.BLUE_BUMP;
		if (isInRedTrench(x, y))
			return Zone.RED_TRENCH;
		if (isInRedBump(x, y))
			return Zone.RED_BUMP;

		if (x < NEUTRAL_ZONE_START)
			return Zone.BLUE_ALLIANCE;
		if (x > NEUTRAL_ZONE_END)
			return Zone.RED_ALLIANCE;

		return Zone.NEUTRAL;
	}

	private static boolean isInBlueTrench(double x, double y) {
		if (x < BLUE_STRIP_X_MIN || x > BLUE_STRIP_X_MAX)
			return false;
		return y < TRENCH_BOTTOM_Y_MAX || y > TRENCH_TOP_Y_MIN;
	}

	private static boolean isInRedTrench(double x, double y) {
		if (x < RED_STRIP_X_MIN || x > RED_STRIP_X_MAX)
			return false;
		return y < TRENCH_BOTTOM_Y_MAX || y > TRENCH_TOP_Y_MIN;
	}

	private static boolean isInBlueBump(double x, double y) {
		if (x < BLUE_STRIP_X_MIN || x > BLUE_STRIP_X_MAX)
			return false;
		return (y >= BUMP_BOTTOM_Y_MIN && y <= BUMP_BOTTOM_Y_MAX)
				|| (y >= BUMP_TOP_Y_MIN && y <= BUMP_TOP_Y_MAX);
	}

	private static boolean isInRedBump(double x, double y) {
		if (x < RED_STRIP_X_MIN || x > RED_STRIP_X_MAX)
			return false;
		return (y >= BUMP_BOTTOM_Y_MIN && y <= BUMP_BOTTOM_Y_MAX)
				|| (y >= BUMP_TOP_Y_MIN && y <= BUMP_TOP_Y_MAX);
	}

	public static boolean isInOwnAllianceZone(Pose2d pose, Alliance alliance) {
		return getZone(pose).belongsTo(alliance);
	}

	public static boolean isInOpponentAllianceZone(Pose2d pose, Alliance alliance) {
		Zone zone = getZone(pose);
		return zone.isAllianceZone() && !zone.belongsTo(alliance);
	}

	public static boolean isInNeutralZone(Pose2d pose) {
		return getZone(pose).isNeutral();
	}

	public static boolean isOnBump(Pose2d pose) {
		return getZone(pose).isBump();
	}

	public static boolean isInTrench(Pose2d pose) {
		return getZone(pose).isTrench();
	}

	public static boolean isOnObstacle(Pose2d pose) {
		return getZone(pose).isObstacle();
	}

	public static double getDistanceToNearestZoneBoundary(Pose2d pose) {
		double x = pose.getX();
		double distToBlueZoneLine = Math.abs(x - NEUTRAL_ZONE_START);
		double distToRedZoneLine = Math.abs(x - NEUTRAL_ZONE_END);
		double distToCenterLine = Math.abs(x - CENTER_LINE);
		return Math.min(distToBlueZoneLine, Math.min(distToRedZoneLine, distToCenterLine));
	}

	public static boolean hasCrossedCenterLine(Pose2d pose, Alliance alliance) {
		double x = pose.getX();
		return alliance == Alliance.Red ? x < CENTER_LINE : x > CENTER_LINE;
	}

	public static Pose2d getHubPose(Alliance alliance) {
		return alliance == Alliance.Red ? HUB_POSE_RED : HUB_POSE_BLUE;
	}

	public static Pose2d getShuttlePose(Alliance alliance, Translation2d robotPose) {
		// return the closest shuttle position right behind bump, halfway between center and the near wall.
		Pose2d pose = alliance == Alliance.Red ? CENTER_OF_ALLIANCE_RED : CENTER_OF_ALLIANCE_BLUE;
		double shuttleY = robotPose.getY() < FIELD_WIDTH / 2
				? FIELD_WIDTH * 0.25
				: FIELD_WIDTH * 0.75;
		return new Pose2d(
				new Translation2d(pose.getX(), shuttleY),
				Rotation2d.kZero);
	}

	public enum Zone {
		BLUE_ALLIANCE(Alliance.Blue, false, false), BLUE_BUMP(Alliance.Blue, true, false), BLUE_TRENCH(Alliance.Blue, false,
				true), RED_ALLIANCE(Alliance.Red, false, false), RED_BUMP(Alliance.Red, true, false), RED_TRENCH(Alliance.Red,
						false, true), NEUTRAL(null, false, false), OUT_OF_BOUNDS(null, false, false);

		private final Alliance alliance;
		private final boolean bump;
		private final boolean trench;

		Zone(Alliance alliance, boolean bump, boolean trench) {
			this.alliance = alliance;
			this.bump = bump;
			this.trench = trench;
		}

		public Alliance getAlliance() {
			return alliance;
		}

		public boolean belongsTo(Alliance alliance) {
			return this.alliance == alliance;
		}

		public boolean isAllianceZone() {
			return alliance != null;
		}

		public boolean isBump() {
			return bump;
		}

		public boolean isTrench() {
			return trench;
		}

		public boolean isObstacle() {
			return bump || trench;
		}

		public boolean isNeutral() {
			return this == NEUTRAL;
		}

		public boolean isInBounds() {
			return this != OUT_OF_BOUNDS;
		}
	}
}
