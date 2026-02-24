package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopZoneAutomation;
import frc.robot.repulsor.Repulsor;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.Setpoints;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import frc.robot.repulsor.Tracking.FieldTrackerCore;
import frc.robot.repulsor.Tracking.Vision.FieldVision;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldZones;
import frc.robot.util.GamePieceTracker;
import swervelib.SwerveInputStream;

public class RobotContainer {

	private SendableChooser<Command> autoChooser;

	// HID
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final CommandJoystick m_JoystickR = new CommandJoystick(2);

	// Subsystems
	private final SwerveSubsystem drivebase = Constants.ENABLE_SWERVE
			? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"))
			: null;
	private final ClimberSubsystem climber = Constants.ENABLE_CLIMBER ? new ClimberSubsystem() : null;
	private final ShooterSubsystem shooter = Constants.ENABLE_SHOOTER ? new ShooterSubsystem() : null;
	private final IntakeSubsystem intake = Constants.ENABLE_INTAKE ? new IntakeSubsystem() : null;
	public boolean robotRelative = false;

	// Repulsor
	private Repulsor repulsor;
	private SwerveInputStream driveAngularVelocity;

	// Game piece tracking
	private final GamePieceTracker gamePieceTracker = new GamePieceTracker();

	// Ball camera vision
	private FieldVision ballCamera;

	// Teleop automation
	private TeleopZoneAutomation teleopAutomation;

	// Scoring setpoints (pose-specific)
	private static final RepulsorSetpoint SCORE_FRONT = new RepulsorSetpoint(
			_Rebuilt2026.HUB_SCORE_FRONT, HeightSetpoint.NET);
	private static final RepulsorSetpoint SCORE_FRONT_LEFT = new RepulsorSetpoint(
			_Rebuilt2026.HUB_SCORE_FRONT_LEFT, HeightSetpoint.NET);
	private static final RepulsorSetpoint SCORE_FRONT_RIGHT = new RepulsorSetpoint(
			_Rebuilt2026.HUB_SCORE_FRONT_RIGHT, HeightSetpoint.NET);
	private static final RepulsorSetpoint SCORE_REAR_LEFT = new RepulsorSetpoint(
			_Rebuilt2026.HUB_SCORE_REAR_LEFT, HeightSetpoint.NET);
	private static final RepulsorSetpoint SCORE_REAR_RIGHT = new RepulsorSetpoint(
			_Rebuilt2026.HUB_SCORE_REAR_RIGHT, HeightSetpoint.NET);

	// Climb setpoints
	private static final RepulsorSetpoint CLIMB_LEFT = new RepulsorSetpoint(
			_Rebuilt2026.CLIMB_LEFT, HeightSetpoint.NONE);
	private static final RepulsorSetpoint CLIMB_RIGHT = new RepulsorSetpoint(
			_Rebuilt2026.CLIMB_RIGHT, HeightSetpoint.NONE);

	// Legacy setpoints
	private static final RepulsorSetpoint SHOOT_SETPOINT = new RepulsorSetpoint(
			Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET);
	private static final RepulsorSetpoint COLLECT_SETPOINT = new RepulsorSetpoint(
			Setpoints.Rebuilt2026.CENTER_COLLECT, HeightSetpoint.NONE);
	private static final RepulsorSetpoint CENTRE_DEFENCE = new RepulsorSetpoint(
			Setpoints.Rebuilt2026.CENTER_COLLECT, HeightSetpoint.NONE);

	public RobotContainer() {
		if (Constants.ENABLE_SWERVE) {
			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> {
						double stickRotation = driverXbox.getRightX() * -1;
						double leftTrigger = Math.pow(driverXbox.getLeftTriggerAxis(), 3);
						double rightTrigger = Math.pow(driverXbox.getRightTriggerAxis(), 3);
						double triggerRotation = (leftTrigger - rightTrigger) * 0.3;
						return MathUtil.clamp(stickRotation + triggerRotation, -1.0, 1.0);
					}).aim(FieldZones.HUB_POSE_RED).aimWhile(driverXbox.b())
					.deadband(OperatorConstants.DEADBAND)
					.scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);

			// Initialize Repulsor path planner
			repulsor = new Repulsor(drivebase, Repulsor.UsageType.kAutoDrive,
					DrivebaseConstants.ROBOT_HALF_LENGTH, DrivebaseConstants.ROBOT_HALF_WIDTH,
					0.0, 0.0, gamePieceTracker);
			if (Constants.ENABLE_SHOOTER && shooter != null) {
				repulsor.withShooterReleaseHeightMetersSupplier(
						() -> shooter.getTargetHoodAngle().in(Units.Radians) * 0.3);
			}

			// Create FieldVision for YOLO camera
			// TODO: measure actual camera mount position and angle on robot
			ballCamera = FieldTrackerCore.getInstance().createFieldVision("yolo",
					new Transform3d(
							new Translation3d(0.3, 0.0, 0.4),
							new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)));

			// Setup teleop automation
			teleopAutomation = new TeleopZoneAutomation(
					repulsor, intake, shooter, gamePieceTracker,
					() -> drivebase.getPose());
			teleopAutomation.configureTriggers();

			// Build expanded auto chooser
			autoChooser = new SendableChooser<>();
			autoChooser.setDefaultOption("Score Front + Cycle", buildScoreCycleAuto(SCORE_FRONT));
			autoChooser.addOption("Score Front-Left + Cycle", buildScoreCycleAuto(SCORE_FRONT_LEFT));
			autoChooser.addOption("Score Front-Right + Cycle", buildScoreCycleAuto(SCORE_FRONT_RIGHT));
			autoChooser.addOption("Score Rear-Left + Cycle", buildScoreCycleAuto(SCORE_REAR_LEFT));
			autoChooser.addOption("Score Rear-Right + Cycle", buildScoreCycleAuto(SCORE_REAR_RIGHT));
			autoChooser.addOption("Score + Climb Left", buildScoreAndClimbAuto(CLIMB_LEFT));
			autoChooser.addOption("Score + Climb Right", buildScoreAndClimbAuto(CLIMB_RIGHT));
			autoChooser.addOption("Defence Only", buildDefenceOnlyAuto());
			autoChooser.addOption("Do Nothing", Commands.none());
			SmartDashboard.putData("misc/Auto Chooser", autoChooser);
		}

		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	private void configureBindings() {
		// ========== Swerve Controls ==========
		if (Constants.ENABLE_SWERVE) {
			drivebase
					.setDefaultCommand(drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative));

			driverXbox.x().whileTrue(
					drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, FieldZones.HUB_POSE_BLUE));

			driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

			driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative))
					.and(DriverStation::isTeleop);

			driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

			driverXbox.back().whileTrue(
					Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));

			// Repulsor teleop buttons - navigate to nearest scoring pose
			driverXbox.b().whileTrue(Commands.defer(() -> {
				var nearest = _Rebuilt2026.nearestScoringPose(drivebase.getPose().getTranslation());
				var sp = new RepulsorSetpoint(nearest, HeightSetpoint.NET);
				return repulsor.alignTo(sp, CategorySpec.kScore);
			}, java.util.Set.of(drivebase)));

			// Navigate to collect position
			driverXbox.start().whileTrue(repulsor.alignTo(COLLECT_SETPOINT, CategorySpec.kEndgame));
		}

		// ========== Shooter Controls ==========
		if (Constants.ENABLE_SHOOTER) {
			driverXbox.povRight().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(Units.RPM.of(1000.0));
				shooter.setFeederVelocity(Units.RPM.of(1000.0));
				System.out.println("shooter set to 1000RPM");
			}));
			driverXbox.povLeft().onTrue(Commands.runOnce(() -> {
				System.out.println("shooter stopped");
				shooter.stop();
			}));
			driverXbox.povUp().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(shooter.getTargetVelocity().plus(Units.RPM.of(100.0)));
				shooter.setFeederVelocity(shooter.getTargetVelocity());
				System.out.println(
						"shooter increased by 100rpm to " + (shooter.getTargetVelocity().baseUnitMagnitude()));
			}));
			driverXbox.povDown().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(shooter.getTargetVelocity().minus(Units.RPM.of(100.0)));
				shooter.setFeederVelocity(shooter.getTargetVelocity());
				System.out.println(
						"shooter decreased by 100rpm to " + (shooter.getTargetVelocity().baseUnitMagnitude()));
			}));

			driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {
				shooter.setHoodAngle(shooter.getTargetHoodAngle().plus(Degrees.of(10)));
			}));
			driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {
				shooter.setHoodAngle(shooter.getTargetHoodAngle().minus(Degrees.of(10)));
			}));
		}

		// ========== Climber Controls (Left Joystick) ==========
		if (Constants.ENABLE_CLIMBER) {
			m_JoystickL.button(3).whileTrue(climber.extendCommand());
			m_JoystickL.button(4).whileTrue(climber.retractCommand());
			m_JoystickL.trigger().whileTrue(
					climber.manualControlCommand(() -> MathUtil.applyDeadband(-m_JoystickL.getY(), 0.1)));
		}

		// ========== Intake Controls ==========
		if (Constants.ENABLE_INTAKE) {
			driverXbox.y().onTrue(Commands.sequence(
					intake.extendCommand(),
					Commands.runOnce(gamePieceTracker::startIntake)));
			driverXbox.y().onFalse(Commands.sequence(
					Commands.runOnce(gamePieceTracker::stopIntake),
					intake.retractCommand()));
		}
	}

	// ===== Auto Routines =====

	private Command buildScoreCycleAuto(RepulsorSetpoint scoreSetpoint) {
		return Commands.sequence(
				// Score preloaded piece
				repulsor.alignTo(scoreSetpoint, CategorySpec.kScore)
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(0.5),
				// Collect
				repulsor.alignTo(COLLECT_SETPOINT, CategorySpec.kCollect)
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(1.0),
				// Score again
				repulsor.alignTo(scoreSetpoint, CategorySpec.kScore)
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(0.5));
	}

	private Command buildScoreAndClimbAuto(RepulsorSetpoint climbSetpoint) {
		return Commands.sequence(
				repulsor.alignTo(SCORE_FRONT, CategorySpec.kScore)
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(1.0),
				repulsor.alignTo(climbSetpoint, CategorySpec.kEndgame));
	}

	private Command buildDefenceOnlyAuto() {
		return repulsor.alignTo(CENTRE_DEFENCE, CategorySpec.kEndgame);
	}

	public Command getAutonomousCommand() {
		if (Constants.ENABLE_SWERVE && autoChooser != null) {
			return autoChooser.getSelected();
		}
		return Commands.none();
	}

	public void updateRepulsor() {
		if (repulsor != null) {
			repulsor.update();
		}
		if (ballCamera != null && drivebase != null) {
			ballCamera.update(drivebase.getPose());
		}
		gamePieceTracker.update();
	}

	public void autonomousInit() {
		gamePieceTracker.setHasPiece(true);
		FieldTrackerCore.getInstance().resetAll();
	}

	public void setMotorBrake(boolean brake) {
		if (Constants.ENABLE_SWERVE) {
			drivebase.setMotorBrake(brake);
		}
	}

	private Distance getDistanceToTarget() {
		Pose2d hubPose = DriverStation.getAlliance()
				.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? FieldZones.HUB_POSE_RED
						: FieldZones.HUB_POSE_BLUE;
		return Meters.of(drivebase.getPose().getTranslation().getDistance(hubPose.getTranslation()));
	}
}
