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

	// ===== Climb =====
	/** Left climb position, must move .2 m negative x to engage climber */
	public static final Pose2d CLIMB_LEFT = new Pose2d(
			1.062, 4.922, Rotation2d.kZero);
	/** Right climb position, must move .2 positive x to engage climber */
	public static final Pose2d CLIMB_RIGHT = new Pose2d(
			1.062, 2.629, Rotation2d.k180deg);

	private static final double HUB_RADIUS = 0.9;
	/** Hub radius + robot half-length in front of hub, facing the hub. */
	public static final Pose2d HUB_FRONT = hubPose(0);

	private static final Pose2d CORNER_HIDE_NEAR_BALLS = new Pose2d(new Translation2d(0.749, 7.324),
			Rotation2d.fromDegrees(0));
	private static final Pose2d CORNER_HIDE = new Pose2d(new Translation2d(0.645, 0.645), Rotation2d.fromDegrees(0));

	// empty constructor
	private AutoPositions() {
	}

	// ===== Pre-built Autos =====

	/** Drive to hub front and stop. */
	public static Command frontHubAuto(Repulsor repulsor) {
		return new AutoBuilder(repulsor)
				.driveTo(HUB_FRONT)
				.build();
	}

	public static Command leftCornerHideAndShoot(Repulsor repulsor, Command shootCommand) {
		return new AutoBuilder(repulsor)
				.driveToFacing(CORNER_HIDE_NEAR_BALLS, HUB_CENTER)
				.run(shootCommand)
				.build();
	}

	public static Command rightCornerHideAndShoot(Repulsor repulsor, Command shootCommand) {
		return new AutoBuilder(repulsor)
				.driveToFacing(CORNER_HIDE, HUB_CENTER)
				.run(shootCommand)
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
