package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopZoneAutomation;
import frc.robot.repulsor.Repulsor;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Setpoints.SetpointUtil;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.AutoChooser;

import java.util.function.Supplier;

/**
 * Declares autonomous modes and adds them to the dashboard.
 *
 * <p>
 * All positions are blue-alliance. Alliance flipping happens inside goal suppliers each cycle.
 */
public final class Autos {

	// ===== Speed Profiles =====
	private static final double SLOW_VELOCITY = 1.5; // m/s
	private static final double SLOW_DECEL = 3.0; // m/s²

	// ===== Field Positions =====
	static final Translation2d HUB_CENTER = _Rebuilt2026.hubAimpointBlue();
	private static final Pose2d CENTER_COLLECT = new Pose2d(
			RepulsorConstants.FIELD_LENGTH / 2.0,
			RepulsorConstants.FIELD_WIDTH / 2.0,
			Rotation2d.kZero);
	static final Pose2d CLIMB_LEFT = new Pose2d(1.062, 4.922, Rotation2d.kZero);
	static final Pose2d CLIMB_RIGHT = new Pose2d(1.062, 2.629, Rotation2d.k180deg);
	private static final Pose2d CORNER_HIDE_NEAR_BALLS = new Pose2d(
			new Translation2d(0.749, 7.324), Rotation2d.fromDegrees(0));
	private static final Pose2d CORNER_HIDE = new Pose2d(
			new Translation2d(0.645, 0.645), Rotation2d.fromDegrees(0));
	private static final double CLIMB_ENGAGE_OFFSET = 0.2;
	static final Pose2d CLIMB_LEFT_ENGAGE = new Pose2d(
			CLIMB_LEFT.getX() - CLIMB_ENGAGE_OFFSET, CLIMB_LEFT.getY(), CLIMB_LEFT.getRotation());
	static final Pose2d CLIMB_RIGHT_ENGAGE = new Pose2d(
			CLIMB_RIGHT.getX() + CLIMB_ENGAGE_OFFSET, CLIMB_RIGHT.getY(), CLIMB_RIGHT.getRotation());
	private static final double HUB_RADIUS = 0.9;
	private static final Pose2d HUB_FRONT = hubPose(0);
	private static final Pose2d HUB_FRONT_SHOOT = hubPoseBack(0, 1.0);
	private static final double COLLECT_Y_OFFSET = 1.8;
	private static final double COLLECT_X_OFFSET = -0.5;

	// ===== Subsystem References =====
	private final Repulsor repulsor;
	private final TeleopZoneAutomation teleopAutomation;
	private final ClimberSubsystem climber;
	private final IntakeSubsystem intake;
	private final RollerSubsystem roller;

	public Autos(Repulsor repulsor, SwerveSubsystem drivebase, TeleopZoneAutomation teleopAutomation,
			ClimberSubsystem climber, IntakeSubsystem intake, RollerSubsystem roller) {
		this.repulsor = repulsor;
		this.teleopAutomation = teleopAutomation;
		this.climber = climber;
		this.intake = intake;
		this.roller = roller;

		AutoChooser chooser = new AutoChooser("misc/Auto Chooser");
		chooser.add("Score Front", this::frontHubAndShoot);
		chooser.add("Left Hide + Shoot", this::leftCornerHideAndShoot);
		chooser.add("Right Hide + Shoot", this::rightCornerHideAndShoot);
		chooser.add("Go to center", this::centerFieldAuto);
		chooser.add("Just Shoot", this::shoot);
		if (intake != null && roller != null) {
			chooser.add("Collect + Shoot x1", this::collectAndShoot1);
			chooser.add("Collect + Shoot x2", this::collectAndShoot2);
		}
		if (Constants.ENABLE_CLIMBER && climber != null) {
			chooser.add("Climb Left", () -> climbAuto(CLIMB_LEFT, CLIMB_LEFT_ENGAGE));
			chooser.add("Climb Right", () -> climbAuto(CLIMB_RIGHT, CLIMB_RIGHT_ENGAGE));
		}
		chooser.setDefault("Score Front");
	}

	// ===== Auto Routines =====

	private Command frontHubAndShoot() {
		return sequence(
				apfDefaults(HUB_FRONT_SHOOT),
				shoot());
	}

	private Command leftCornerHideAndShoot() {
		return sequence(
				apfDefaultsFacing(() -> {
					boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
					return red ? CORNER_HIDE : CORNER_HIDE_NEAR_BALLS;
				}, HUB_CENTER, 180),
				shoot());
	}

	private Command rightCornerHideAndShoot() {
		return sequence(
				apfDefaultsFacing(() -> {
					boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
					return red ? CORNER_HIDE_NEAR_BALLS : CORNER_HIDE;
				}, HUB_CENTER, 180),
				shoot());
	}

	private Command centerFieldAuto() {
		return apfForever(CENTER_COLLECT);
	}

	private Command collectAndShoot1() {
		Pose2d startPose = repulsor.getDrive().getPose();
		Pose2d collectPose = computeCollectPose();
		return sequence(
				shoot(),
				deadline(apfDefaults(collectPose), collect()),
				apfDefaults(startPose),
				shoot());
	}

	private Command collectAndShoot2() {
		Pose2d startPose = repulsor.getDrive().getPose();
		Pose2d collectPose = computeCollectPose();
		return sequence(
				shoot(),
				deadline(apfDefaults(collectPose), collect()),
				apfDefaults(startPose),
				shoot(),
				deadline(apfDefaults(collectPose), collect()),
				apfDefaults(startPose),
				shoot());
	}

	private Command climbAuto(Pose2d climbPose, Pose2d engagePose) {
		return sequence(
				apfDefaults(climbPose),
				climber.climbBottomCommand(),
				apfSlow(engagePose),
				climber.climbTopCommand(),
				climber.climbHangCommand());
	}

	// ===== APF Drive Helpers =====

	/** Drive to pose with default speed, ends when within default tolerances. */
	private Command apfDefaults(Pose2d bluePose) {
		return repulsor.apfDrive(() -> resolveAlliance(bluePose),
				Repulsor.DEFAULT_POS_TOLERANCE, Repulsor.DEFAULT_ANG_TOLERANCE);
	}

	/** Drive to pose with default speed, never ends. */
	private Command apfForever(Pose2d bluePose) {
		return repulsor.apfDrive(() -> resolveAlliance(bluePose));
	}

	/** Drive to pose slowly (precision), ends when within default tolerances. */
	private Command apfSlow(Pose2d bluePose) {
		return repulsor.apfDrive(() -> resolveAlliance(bluePose), () -> SLOW_VELOCITY, () -> SLOW_DECEL,
				() -> Repulsor.DEFAULT_POS_TOLERANCE, () -> Repulsor.DEFAULT_ANG_TOLERANCE);
	}

	/** Drive to pose while facing a target, ends when within default tolerances. */
	private Command apfDefaultsFacing(Supplier<Pose2d> bluePoseSupplier, Translation2d blueAimTarget,
			double rotationOffsetDeg) {
		return repulsor.apfDrive(() -> computeFacingPose(bluePoseSupplier.get(), blueAimTarget, rotationOffsetDeg),
				Repulsor.DEFAULT_POS_TOLERANCE, Repulsor.DEFAULT_ANG_TOLERANCE);
	}

	// ===== Subsystem Helpers =====

	private Command shoot() {
		return teleopAutomation.shootCommand().withTimeout(4);
	}

	private Command collect() {
		return Commands.parallel(
				intake.extendCommand().andThen(intake.holdExtendedCommand()),
				roller.intakeCommand());
	}

	// ===== Utilities =====

	private Pose2d computeCollectPose() {
		Pose2d startPose = repulsor.getDrive().getPose();
		double fieldCenterX = RepulsorConstants.FIELD_LENGTH / 2.0;
		double fieldCenterY = RepulsorConstants.FIELD_WIDTH / 2.0;
		boolean fromTop = startPose.getY() > fieldCenterY;
		double collectY = fromTop ? fieldCenterY + COLLECT_Y_OFFSET : fieldCenterY - COLLECT_Y_OFFSET;
		double collectDeg = fromTop ? -120.0 : -60.0;
		return new Pose2d(fieldCenterX + COLLECT_X_OFFSET, collectY, Rotation2d.fromDegrees(collectDeg));
	}

	private static Pose2d resolveAlliance(Pose2d bluePose) {
		Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
		return alliance == Alliance.Red ? SetpointUtil.flipToRed(bluePose) : bluePose;
	}

	private static Pose2d computeFacingPose(Pose2d bluePose, Translation2d blueAimTarget, double rotationOffsetDeg) {
		Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
		Pose2d flipped = alliance == Alliance.Red ? SetpointUtil.flipToRed(bluePose) : bluePose;
		Translation2d target = alliance == Alliance.Red ? SetpointUtil.flipToRed(blueAimTarget) : blueAimTarget;
		Rotation2d towardTarget = target.minus(flipped.getTranslation()).getAngle();
		Rotation2d facing = towardTarget.rotateBy(Rotation2d.fromDegrees(rotationOffsetDeg));
		return new Pose2d(flipped.getTranslation(), facing);
	}

	private static Pose2d hubPose(double angleDeg) {
		Rotation2d angle = Rotation2d.fromDegrees(angleDeg);
		double standoff = HUB_RADIUS + Constants.DrivebaseConstants.ROBOT_HALF_LENGTH;
		Translation2d pos = HUB_CENTER.minus(new Translation2d(standoff, angle));
		Rotation2d faceHub = HUB_CENTER.minus(pos).getAngle();
		return new Pose2d(pos, faceHub);
	}

	private static Pose2d hubPoseBack(double angleDeg, double extraStandoffM) {
		Rotation2d angle = Rotation2d.fromDegrees(angleDeg);
		double standoff = HUB_RADIUS + Constants.DrivebaseConstants.ROBOT_HALF_LENGTH + extraStandoffM;
		Translation2d pos = HUB_CENTER.minus(new Translation2d(standoff, angle));
		Rotation2d awayFromHub = HUB_CENTER.minus(pos).getAngle().rotateBy(Rotation2d.k180deg);
		return new Pose2d(pos, awayFromHub);
	}
}
