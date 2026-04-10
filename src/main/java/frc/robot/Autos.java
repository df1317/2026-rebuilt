package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopZoneAutomation;
import frc.robot.repulsor.Repulsor;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.AutoChooser;
import frc.robot.util.FieldPose;
import frc.robot.util.FieldTranslation;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Declares autonomous modes and adds them to the dashboard.
 *
 * <p>
 * All positions are blue-alliance. Alliance flipping is handled by {@link FieldPose} and {@link FieldTranslation} at
 * runtime.
 */
public final class Autos {

	private static final double SLOW_VELOCITY = 1.5; // m/s
	private static final double SLOW_DECEL = 3.0; // m/s²

	// ===== Field Positions =====
	private static final FieldTranslation HUB_CENTER = new FieldTranslation(_Rebuilt2026.hubAimpointBlue());
	private static final FieldPose CLIMB_LEFT = new FieldPose(1.062, 4.922, Rotation2d.kZero);
	private static final FieldPose CLIMB_RIGHT = new FieldPose(1.062, 2.629, Rotation2d.k180deg);
	private static final FieldPose CORNER_HIDE_LEFT = new FieldPose(0.749, 7.324, Rotation2d.fromDegrees(0));
	private static final FieldPose CORNER_HIDE_RIGHT = new FieldPose(0.645, 0.645, Rotation2d.fromDegrees(0));
	private static final double CLIMB_ENGAGE_OFFSET = 0.2;
	private static final FieldPose CLIMB_LEFT_ENGAGE = new FieldPose(
			CLIMB_LEFT.getBlue().getX() - CLIMB_ENGAGE_OFFSET, CLIMB_LEFT.getBlue().getY(),
			CLIMB_LEFT.getBlue().getRotation());
	private static final FieldPose CLIMB_RIGHT_ENGAGE = new FieldPose(
			CLIMB_RIGHT.getBlue().getX() + CLIMB_ENGAGE_OFFSET, CLIMB_RIGHT.getBlue().getY(),
			CLIMB_RIGHT.getBlue().getRotation());
	private static final double HUB_RADIUS = 0.9;
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
		chooser.add("Right Hide + Shoot + Outpost", this::rightCornerHideAndShootOutpost);
		chooser.add("Go to center", this::centerFieldAuto);
		chooser.add("Just Shoot", this::shoot);
		if (intake != null && roller != null) {
			chooser.add("Collect + Shoot x1", this::collectAndShoot1);
			chooser.add("Collect + Shoot x2", this::collectAndShoot2);
			chooser.add("Test Intake Movement", this::testIntakeMovement);
		}
		if (climber != null) {
			chooser.add("Climb Left", () -> climbAuto(CLIMB_LEFT, CLIMB_LEFT_ENGAGE));
			chooser.add("Climb Right", () -> climbAuto(CLIMB_RIGHT, CLIMB_RIGHT_ENGAGE));
		}
		chooser.setDefault("Test Intake Movement");
	}

	// ===== Auto Routines =====

	private Command testIntakeMovement() {
		return sequence(
				intake.extendCommand().andThen(intake.holdExtendedCommand().withTimeout(1.0)),
				intake.stowCommand(),
				Commands.waitSeconds(1.0),
				intake.extendCommand().andThen(intake.holdExtendedCommand().withTimeout(1.0)),
				intake.stowCommand());
	}

	private static Pose2d hubPoseBack(double angleDeg, double extraStandoffM) {
		Translation2d hubCenter = _Rebuilt2026.hubAimpointBlue();
		Rotation2d angle = Rotation2d.fromDegrees(angleDeg);
		double standoff = HUB_RADIUS + Constants.DrivebaseConstants.ROBOT_HALF_LENGTH + extraStandoffM;
		Translation2d pos = hubCenter.minus(new Translation2d(standoff, angle));
		Rotation2d awayFromHub = hubCenter.minus(pos).getAngle().rotateBy(Rotation2d.k180deg);
		return new Pose2d(pos, awayFromHub);
	}

	private Command frontHubAndShoot() {
		return sequence(
				apfDefaults(_Rebuilt2026.HUB_FRONT_SHOOT),
				shoot());
	}

	private Command leftCornerHideAndShoot() {
		return sequence(
				apfDefaultsFacing(CORNER_HIDE_LEFT, HUB_CENTER, 180),
				shoot());
	}

	private Command rightCornerHideAndShoot() {
		return sequence(
				apfDefaultsFacing(CORNER_HIDE_RIGHT, HUB_CENTER, 180),
				shoot());
	}

	private Command rightCornerHideAndShootOutpost() {
		return sequence(
				apfDefaultsFacing(CORNER_HIDE_RIGHT, HUB_CENTER, 180),
				shoot(),
				apfTurnThenDrive(_Rebuilt2026.OUTPOST_COLLECT.withRotationOffset(Rotation2d.fromDegrees(90))),
				apfDefaultsFacing(CORNER_HIDE_RIGHT, HUB_CENTER, 180),
				shoot());
	}

	private Command centerFieldAuto() {
		return apfForever(_Rebuilt2026.CENTER_COLLECT);
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

	// ===== APF Drive Helpers =====

	private Command climbAuto(FieldPose climbPose, FieldPose engagePose) {
		return sequence(
				apfDefaults(climbPose),
				climber.climbBottomCommand(),
				apfSlow(engagePose),
				climber.climbTopCommand(),
				climber.climbHangCommand());
	}

	/** Apply the auto speed scale to a command. */
	private Command withSpeedScale(Command cmd) {
		return Commands.sequence(
				Commands.runOnce(() -> repulsor.setAutoSpeedScale(Constants.AutoConstants.SPEED_SCALE)),
				cmd).finallyDo(() -> repulsor.resetSpeedScale());
	}

	/** Drive to a {@link FieldPose} with default speed, ends when within default tolerances. */
	private Command apfDefaults(FieldPose pose) {
		return withSpeedScale(repulsor.apfDrive(pose, Repulsor.DEFAULT_POS_TOLERANCE, Repulsor.DEFAULT_ANG_TOLERANCE));
	}

	/** Drive to a {@link frc.robot.repulsor.Setpoints.GameSetpoint} with default speed. */
	private Command apfDefaults(frc.robot.repulsor.Setpoints.GameSetpoint pose) {
		return withSpeedScale(repulsor.apfDrive(pose, Repulsor.DEFAULT_POS_TOLERANCE, Repulsor.DEFAULT_ANG_TOLERANCE));
	}

	/** Drive to a raw pose (already alliance-resolved), ends when within default tolerances. */
	private Command apfDefaults(Pose2d resolvedPose) {
		return withSpeedScale(
				repulsor.apfDrive(() -> resolvedPose, Repulsor.DEFAULT_POS_TOLERANCE, Repulsor.DEFAULT_ANG_TOLERANCE));
	}

	/** Turn in place to the goal's heading, then drive to it. */
	private Command apfTurnThenDrive(frc.robot.repulsor.Setpoints.GameSetpoint pose) {
		return sequence(
				defer(
						() -> apfDefaults(new Pose2d(repulsor.getDrive().getPose().getTranslation(),
								pose.poseForCurrentAlliance(frc.robot.repulsor.Setpoints.SetpointContext.EMPTY).getRotation())),
						java.util.Set.of()),
				apfDefaults(pose));
	}

	/** Drive to a {@link FieldPose} with default speed, never ends. */
	private Command apfForever(FieldPose pose) {
		return withSpeedScale(repulsor.apfDrive(pose));
	}

	/** Drive to a {@link frc.robot.repulsor.Setpoints.GameSetpoint} with default speed, never ends. */
	private Command apfForever(frc.robot.repulsor.Setpoints.GameSetpoint pose) {
		return withSpeedScale(repulsor.apfDrive(pose));
	}

	/** Drive to a {@link FieldPose} slowly (precision), ends when within default tolerances. */
	private Command apfSlow(FieldPose pose) {
		return withSpeedScale(repulsor.apfDrive(pose, () -> SLOW_VELOCITY, () -> SLOW_DECEL,
				() -> Repulsor.DEFAULT_POS_TOLERANCE, () -> Repulsor.DEFAULT_ANG_TOLERANCE));
	}

	// ===== Subsystem Helpers =====

	/** Drive to a pose while facing a target, ends when within default tolerances. */
	private Command apfDefaultsFacing(Supplier<Pose2d> poseSupplier, FieldTranslation aimTarget,
			double rotationOffsetDeg) {
		return withSpeedScale(repulsor.apfDriveFacing(poseSupplier, aimTarget, rotationOffsetDeg,
				Repulsor.DEFAULT_POS_TOLERANCE, Repulsor.DEFAULT_ANG_TOLERANCE));
	}

	private Command shoot() {
		return teleopAutomation.shootCommand().withTimeout(4);
	}

	// ===== Utilities =====

	private Command collect() {
		return Commands.parallel(
				intake.extendCommand().andThen(intake.holdExtendedCommand()),
				roller.intakeCommand());
	}

	private Pose2d computeCollectPose() {
		Pose2d startPose = repulsor.getDrive().getPose();
		double fieldCenterX = RepulsorConstants.FIELD_LENGTH / 2.0;
		double fieldCenterY = RepulsorConstants.FIELD_WIDTH / 2.0;
		boolean fromTop = startPose.getY() > fieldCenterY;
		double collectY = fromTop ? fieldCenterY + COLLECT_Y_OFFSET : fieldCenterY - COLLECT_Y_OFFSET;
		double collectDeg = fromTop ? -120.0 : -60.0;
		return new Pose2d(fieldCenterX + COLLECT_X_OFFSET, collectY, Rotation2d.fromDegrees(collectDeg));
	}
}
