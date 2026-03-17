package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.repulsor.Repulsor;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;

/**
 * Auto target positions and pre-built routines.
 *
 * <p>
 * All positions are blue-alliance. Use {@link AutoBuilder} to compose custom autos. Alliance flipping happens at auto
 * start.
 */
public final class AutoPositions {

	/** Hub center point, useful as an aim target for {@link AutoBuilder#driveToFacing}. */
	public static final Translation2d HUB_CENTER = _Rebuilt2026.hubAimpointBlue();
	/** Field center. */
	public static final Pose2d CENTER_COLLECT = new Pose2d(
			RepulsorConstants.FIELD_LENGTH / 2.0,
			RepulsorConstants.FIELD_WIDTH / 2.0,
			Rotation2d.kZero);
	private static final double HUB_RADIUS = 0.9;
	/** Hub radius + robot half-length in front of hub, facing the hub. */
	public static final Pose2d HUB_FRONT = hubPose(0);

	private static final double CLIMB_OFFSET_Y = 2.5;
	private static final double CLIMB_OFFSET_X = 2.0;

	// ===== Climb =====
	/** Left climb position, 2 m left of hub center. */
	public static final Pose2d CLIMB_LEFT = new Pose2d(
			HUB_CENTER.getX() + CLIMB_OFFSET_X, HUB_CENTER.getY() + CLIMB_OFFSET_Y, Rotation2d.kZero);

	/** Right climb position, 2 m right of hub center. */
	public static final Pose2d CLIMB_RIGHT = new Pose2d(
			HUB_CENTER.getX() + CLIMB_OFFSET_X, HUB_CENTER.getY() - CLIMB_OFFSET_Y, Rotation2d.kZero);

	// empty constructor
	private AutoPositions() {
	}

	// ===== Pre-built Autos =====

	/** Drive to hub front and stop. */
	public static Command scoreAuto(Repulsor repulsor) {
		return new AutoBuilder(repulsor)
				.driveTo(HUB_FRONT)
				.build();
	}

	/** Score at hub front, wait 1s, then drive to the given climb position. */
	public static Command scoreAndClimbAuto(Repulsor repulsor, Pose2d climbPose) {
		return new AutoBuilder(repulsor)
				.driveTo(HUB_FRONT)
				.waitSeconds(1.0)
				.driveToAndHold(climbPose)
				.build();
	}

	/** Drive to field center and hold. */
	public static Command centerFieldAuto(Repulsor repulsor) {
		return new AutoBuilder(repulsor)
				.driveToAndHold(CENTER_COLLECT)
				.build();
	}

	/** Scoring pose at the given angle around the hub (0 = front), offset by robot half-length. */
	private static Pose2d hubPose(double angleDeg) {
		Rotation2d angle = Rotation2d.fromDegrees(angleDeg);
		double standoff = HUB_RADIUS + Constants.DrivebaseConstants.ROBOT_HALF_LENGTH;
		Translation2d pos = HUB_CENTER.minus(new Translation2d(standoff, angle));
		Rotation2d faceHub = HUB_CENTER.minus(pos).getAngle();
		return new Pose2d(pos, faceHub);
	}
}
