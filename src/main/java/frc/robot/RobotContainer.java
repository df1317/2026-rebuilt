package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.io.File;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.measure.Angle;
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
import swervelib.SwerveInputStream;

public class RobotContainer {

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
	private final BooleanSubscriber obstacleClampEnabled = DogLog.tunable("Drive/ObstacleClampEnabled", false);
	private final SendableChooser<Command> autoChooser;
	// Repulsor
	private final Repulsor repulsor;
	private final SwerveInputStream driveAngularVelocity;
	// Ball camera vision
	private final FieldVision ballCamera;
	// Teleop automation
	private final TeleopZoneAutomation teleopAutomation;
	public boolean robotRelative = false;

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
					repulsor, intake, shooter,
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
		if (Constants.ENABLE_SHOOTER) {
			if (Constants.ENABLE_HOPPER) {
				m_JoystickL.button(2).toggleOnTrue(shooter.spinUpAndWaitCommand(RPM.of(3000))
						.andThen(Commands.sequence(Commands.runOnce(() -> shooter.setFeederVelocity(RPM.of(3000))),
								Commands.waitUntil(shooter::isFeederAtSpeed)))
						.andThen(Commands.runOnce(() -> hopper.setHopperVelocity(RPM.of(2000)))));
				m_JoystickL.button(2).toggleOnFalse(Commands.runOnce(() -> {
					shooter.stop();
					hopper.setHopperVelocity(RPM.of(0.0));
				}));

			}
			driverXbox.rightTrigger(0.3).whileTrue(Commands.runEnd(() -> {
				// System.out.println("move HOOD! 1");
				final Angle newSetpoint = shooter.getTargetHoodAngle().plus(Degree.of(5));
				shooter.setHoodAngle(newSetpoint);
			}, () -> shooter.HoodStop()));
			driverXbox.leftTrigger(0.3).whileTrue(Commands.runEnd(() -> {
				// System.out.println("move HOOD! 2");
				final Angle newSetpoint = shooter.getTargetHoodAngle().minus(Degree.of(5));
				shooter.setHoodAngle(newSetpoint);
			}, () -> shooter.HoodStop()));
		}

		// ===== Test Mode Controls =====
		if (DriverStation.isTest()) {
			if (Constants.ENABLE_SHOOTER && shooter != null) {
				m_JoystickL.button(4).onTrue(shooter.homeHoodCommand());
				m_JoystickL.button(5).whileTrue(shooter.testFullMotorCommand());
				m_JoystickL.button(7).whileTrue(shooter.testShooterMotorCommand());
				m_JoystickL.button(8).whileTrue(shooter.testFeederCommand());
				m_JoystickL.button(9).whileTrue(shooter.testHoodCommand());
			}
			if (Constants.ENABLE_INTAKE && intake != null) {
				m_JoystickL.button(10).whileTrue(intake.testPivotCommand());
				m_JoystickL.button(11).whileTrue(intake.testRollerCommand());
			}
			if (Constants.ENABLE_CLIMBER && climber != null) {
				if (false) {
					m_JoystickL.button(12).whileTrue(climber.testClimberCommand());
					driverXbox.rightTrigger(0.7).whileTrue(Commands.runEnd(() -> {
						System.out.println("CLIMBER GO UP");

					}, () -> {
						System.out.println("CLIMBER STOP");
					}));
					m_JoystickL.button(12).whileTrue(climber.testClimberCommand());
					driverXbox.rightTrigger(0.7).whileTrue(Commands.runEnd(() -> {
						System.out.println("CLIMBER GO DOWN");
					}, () -> {
						System.out.println("CLIMBER STOP");
					}));
				} else {
					driverXbox.leftTrigger(0.7).whileTrue(
              climber.manualControlCommand(() -> (m_JoystickL.getY() / 70.0)));
				}
			}
			if (Constants.ENABLE_HOPPER && hopper != null) {
				m_JoystickL.button(3).whileTrue(hopper.testHopperCommand());
			}
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
	}

	public void autonomousInit() {
		FieldTrackerCore.getInstance().resetAll();
	}

	public void setMotorBrake(boolean brake) {
		if (Constants.ENABLE_SWERVE) {
			drivebase.setMotorBrake(brake);
		}
	}
}
