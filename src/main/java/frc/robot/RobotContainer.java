package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.io.File;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
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
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import frc.robot.repulsor.Tracking.FieldTrackerCore;
import frc.robot.repulsor.Tracking.Vision.FieldVision;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
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

	// Subsystems
	private final SwerveSubsystem drivebase = Constants.ENABLE_SWERVE
			? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"))
			: null;
	private final ClimberSubsystem climber = Constants.ENABLE_CLIMBER ? new ClimberSubsystem() : null;
	private final ShooterSubsystem shooter = Constants.ENABLE_SHOOTER ? new ShooterSubsystem() : null;
	private final IntakeSubsystem intake = Constants.ENABLE_INTAKE ? new IntakeSubsystem() : null;
	private final HopperSubsystem hopper = Constants.ENABLE_HOPPER ? new HopperSubsystem() : null;
	public boolean robotRelative = false;

	// Repulsor
	private Repulsor repulsor;
	private SwerveInputStream driveAngularVelocity;
	private final BooleanSubscriber obstacleClampEnabled = DogLog.tunable("Drive/ObstacleClampEnabled", false);

	// Game piece tracking
	private final GamePieceTracker gamePieceTracker = new GamePieceTracker();

	// Ball camera vision
	private FieldVision ballCamera;

	// Teleop automation
	private TeleopZoneAutomation teleopAutomation;

	public RobotContainer() {
		if (Constants.ENABLE_SWERVE) {
			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
					.aim(FieldZones.HUB_POSE_RED).aimWhile(driverXbox.y())
					.deadband(OperatorConstants.DEADBAND)
					.scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);

			// Initialize Repulsor path planner
			repulsor = new Repulsor(drivebase,
					DrivebaseConstants.ROBOT_HALF_LENGTH, DrivebaseConstants.ROBOT_HALF_WIDTH);

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

			// Build auto chooser
			autoChooser = new SendableChooser<>();
			autoChooser.setDefaultOption("Score Front + Cycle",
					buildScoreCycleAuto(_Rebuilt2026.HUB_SCORE_FRONT));
			autoChooser.addOption("Score Front-Left + Cycle",
					buildScoreCycleAuto(_Rebuilt2026.HUB_SCORE_FRONT_LEFT));
			autoChooser.addOption("Score Front-Right + Cycle",
					buildScoreCycleAuto(_Rebuilt2026.HUB_SCORE_FRONT_RIGHT));
			autoChooser.addOption("Score Rear-Left + Cycle",
					buildScoreCycleAuto(_Rebuilt2026.HUB_SCORE_REAR_LEFT));
			autoChooser.addOption("Score Rear-Right + Cycle",
					buildScoreCycleAuto(_Rebuilt2026.HUB_SCORE_REAR_RIGHT));
			autoChooser.addOption("Score + Climb Left",
					buildScoreAndClimbAuto(_Rebuilt2026.CLIMB_LEFT));
			autoChooser.addOption("Score + Climb Right",
					buildScoreAndClimbAuto(_Rebuilt2026.CLIMB_RIGHT));
			autoChooser.addOption("Defence Only", buildDefenceOnlyAuto());
			autoChooser.addOption("Do Nothing", Commands.none());
			SmartDashboard.putData("misc/Auto Chooser", autoChooser);
		}

		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	private void configureBindings() {
		// ===== Driver Controls (Xbox port 0) =====
		if (Constants.ENABLE_SWERVE) {
			drivebase.setDefaultCommand(
					drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative,
							speeds -> {
								if (obstacleClampEnabled.get() && repulsor != null) {
									return repulsor.clampDriveSpeed(speeds, drivebase.getPose());
								}
								return speeds;
							}));

			// A once: gyro reset (disabled in test mode)
			driverXbox.a().and(() -> !DriverStation.isTest())
					.onTrue(Commands.runOnce(drivebase::zeroGyro));

			// Left bumper toggle: field relative
			driverXbox.leftBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative));
		}

		// Right trigger hold: auto shoot (distance-based)
		if (Constants.ENABLE_SWERVE && Constants.ENABLE_SHOOTER) {
			driverXbox.rightTrigger(0.3).whileTrue(
					shooter.shootForDistanceCommand(this::getDistanceToTarget));
		}

		// Left trigger hold: intake
		if (Constants.ENABLE_SWERVE && Constants.ENABLE_INTAKE) {
			driverXbox.leftTrigger(0.3).whileTrue(Commands.parallel(
					intake.intakeCommand(),
					Commands.runOnce(gamePieceTracker::startIntake)))
					.onFalse(Commands.sequence(
							Commands.runOnce(gamePieceTracker::stopIntake),
							intake.stowCommand()));
		}

		// ===== Operator Controls (Joystick port 2) =====

		if (Constants.ENABLE_HOPPER) {
			// Hopper runs automatically with intake
			if (Constants.ENABLE_INTAKE) {
				driverXbox.leftTrigger(0.3).whileTrue(hopper.forwardCommand());
			}

			// Hopper feeds when shooting
			if (Constants.ENABLE_SHOOTER) {
				driverXbox.rightTrigger(0.3).whileTrue(hopper.forwardCommand());
			}

			// Button 1: reverse hopper (unclog)
			m_JoystickL.button(1).whileTrue(hopper.reverseCommand());
		}

		if (Constants.ENABLE_SHOOTER) {
			// Button 2: shooter toggle on/off at 3000 RPM
			m_JoystickL.button(2).toggleOnTrue(
					shooter.shootCommand(Units.RPM.of(3000)));

			// POV up: +100 RPM
			m_JoystickL.povUp().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(shooter.getTargetVelocity().plus(Units.RPM.of(100.0)));
				shooter.setFeederVelocity(shooter.getTargetVelocity());
			}));

			// POV down: -100 RPM
			m_JoystickL.povDown().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(shooter.getTargetVelocity().minus(Units.RPM.of(100.0)));
				shooter.setFeederVelocity(shooter.getTargetVelocity());
			}));

			// Button 3: reverse shoot (declog)
			m_JoystickL.button(3).whileTrue(Commands.startEnd(
					() -> {
						shooter.setVelocity(Units.RPM.of(-1000));
						shooter.setFeederVelocity(Units.RPM.of(-1000));
					},
					shooter::stop, shooter));
		}

		if (Constants.ENABLE_INTAKE) {
			// Button 4: intake toggle down/up
			m_JoystickL.button(4).toggleOnTrue(intake.intakeCommand());
			m_JoystickL.button(4).toggleOnFalse(intake.stowCommand());

			// Button 5: reverse intake (eject)
			m_JoystickL.button(5).whileTrue(intake.ejectCommand());
		}

		if (Constants.ENABLE_CLIMBER) {
			// Button 6: climber extend/retract toggle
			m_JoystickL.button(6).toggleOnTrue(climber.extendCommand());
			m_JoystickL.button(6).toggleOnFalse(climber.retractCommand());

			// Joystick Y axis: manual climber (while button 7 held)
			m_JoystickL.button(7).whileTrue(
					climber.manualControlCommand(
							() -> MathUtil.applyDeadband(-m_JoystickL.getY(), 0.1)));
		}

		// ===== Test Mode Controls =====
		// Use dashboard tunables (Test/*) to set values, then hold buttons to run

		if (Constants.ENABLE_SHOOTER) {
			driverXbox.a().and(DriverStation::isTest)
					.whileTrue(shooter.testShooterCommand());
		}

		if (Constants.ENABLE_INTAKE) {
			driverXbox.b().and(DriverStation::isTest)
					.whileTrue(intake.testIntakeCommand());
		}

		if (Constants.ENABLE_CLIMBER) {
			driverXbox.x().and(DriverStation::isTest)
					.whileTrue(climber.testClimberCommand());
		}

		if (Constants.ENABLE_HOPPER) {
			driverXbox.y().and(DriverStation::isTest)
					.whileTrue(hopper.testHopperCommand());
		}
	}

	// ===== Auto Routines =====

	private Command buildScoreCycleAuto(frc.robot.repulsor.Setpoints.GameSetpoint scoreSetpoint) {
		return Commands.sequence(
				// Score preloaded piece
				repulsor.navigateTo(() -> scoreSetpoint.poseForCurrentAlliance(SetpointContext.EMPTY))
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(0.5),
				// Collect (vision-aware)
				repulsor.navigateTo(() -> {
					Pose2d collectPose = FieldTrackerCore.getInstance()
							.nextCollectionGoalBlue(drivebase.getPose(), 0.0, 0);
					return collectPose;
				}).until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(1.0),
				// Score again
				repulsor.navigateTo(() -> scoreSetpoint.poseForCurrentAlliance(SetpointContext.EMPTY))
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(0.5));
	}

	private Command buildScoreAndClimbAuto(frc.robot.repulsor.Setpoints.GameSetpoint climbSetpoint) {
		return Commands.sequence(
				repulsor.navigateTo(
						() -> _Rebuilt2026.HUB_SCORE_FRONT.poseForCurrentAlliance(SetpointContext.EMPTY))
						.until(repulsor.within(Meters.of(0.15))),
				Commands.waitSeconds(1.0),
				repulsor.navigateTo(
						() -> climbSetpoint.poseForCurrentAlliance(SetpointContext.EMPTY)));
	}

	private Command buildDefenceOnlyAuto() {
		return repulsor.navigateTo(
				() -> _Rebuilt2026.CENTER_COLLECT.poseForCurrentAlliance(SetpointContext.EMPTY));
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
