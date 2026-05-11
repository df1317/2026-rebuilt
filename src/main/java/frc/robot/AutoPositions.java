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
	static final Pose2d CORNER_HIDE_NEAR_BALLS = new Pose2d(new Translation2d(0.749, 7.324),
			Rotation2d.fromDegrees(0));
	static final Pose2d CORNER_HIDE = new Pose2d(new Translation2d(0.645, 0.645), Rotation2d.fromDegrees(0));
	private static final double HUB_RADIUS = 0.9;
	/** Hub radius + robot half-length in front of hub, facing the hub. */
	public static final Pose2d HUB_FRONT = hubPose(0);
	/** 1m further back from HUB_FRONT, rotated 180 (intake facing hub). */
	public static final Pose2d HUB_FRONT_SHOOT = hubPoseBack(0, 1.0);

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

	/** Drive to hub front, hold position, and shoot. */
	public static Command frontHubAndShoot(Repulsor repulsor, Command shootCommand) {
		return new AutoBuilder(repulsor)
				.driveTo(HUB_FRONT_SHOOT)
				.run(shootCommand)
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

	// ===== Collect & Shoot Autos =====

	/** Shoot to clear, collect from closest side, return to start, shoot. */
	public static Command collectAndShoot1(Repulsor repulsor, Command shootCommand) {
		return new AutoBuilder(repulsor)
				.run(shootCommand)
				.driveToCollect()
				.driveToStart()
				.run(shootCommand)
				.build();
	}

	/** Shoot to clear, collect from closest side, return and shoot, repeat once more. */
	public static Command collectAndShoot2(Repulsor repulsor, Command shootCommand) {
		return new AutoBuilder(repulsor)
				.run(shootCommand)
				.driveToCollect()
				.driveToStart()
				.run(shootCommand)
				.driveToCollect()
				.driveToStart()
				.run(shootCommand)
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

	/** Like hubPose but with extra standoff and rotated 180 (intake facing hub). */
	private static Pose2d hubPoseBack(double angleDeg, double extraStandoffM) {
		Rotation2d angle = Rotation2d.fromDegrees(angleDeg);
		double standoff = HUB_RADIUS + Constants.DrivebaseConstants.ROBOT_HALF_LENGTH + extraStandoffM;
		Translation2d pos = HUB_CENTER.minus(new Translation2d(standoff, angle));
		Rotation2d awayFromHub = HUB_CENTER.minus(pos).getAngle().rotateBy(Rotation2d.k180deg);
		return new Pose2d(pos, awayFromHub);
	}
}
