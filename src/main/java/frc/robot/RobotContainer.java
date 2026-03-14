package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
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
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldZones;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

public class RobotContainer {

	// HID
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final OperatorPanel panel = new OperatorPanel(2);
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
	// Teleop automation
	private final TeleopZoneAutomation teleopAutomation;
	public boolean robotRelative = false;

	public RobotContainer() {
		if (Constants.ENABLE_SWERVE) {
			// Initialize Repulsor path planner
			repulsor = new Repulsor(drivebase,
					DrivebaseConstants.ROBOT_HALF_LENGTH, DrivebaseConstants.ROBOT_HALF_WIDTH);

			// Setup teleop automation
			teleopAutomation = new TeleopZoneAutomation(
					repulsor, intake, shooter, hopper,
					drivebase::getPose, drivebase::getFieldVelocity);
			drivebase.setTargetDistanceSupplier(teleopAutomation::getTargetDistance);
			drivebase.setAimTargetSupplier(teleopAutomation::getVirtualAimTarget);

			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
					.aimWhile(driverXbox.y())
					.deadband(OperatorConstants.DEADBAND)
					.scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);

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
			driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

			// Left bumper toggle: field relative
			driverXbox.leftBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative));
		}
		if (Constants.ENABLE_SHOOTER) {
			driverXbox.rightBumper().whileTrue(teleopAutomation.shootCommand());
		}
		if (Constants.ENABLE_INTAKE && intake != null) {
			driverXbox.x().toggleOnTrue(intake.stowToggleCommand());
			driverXbox.leftTrigger().whileTrue(intake.intakeCommand());
		}

		// ===== Test Mode Controls (Maypad — see OperatorPanel for layout) =====
		if (DriverStation.isTest()) {
			// Row 0 — Shooter
			if (Constants.ENABLE_SHOOTER && shooter != null) {
				panel.key(0, 0).whileTrue(shooter.testShooterMotorCommand());
				panel.key(0, 1).whileTrue(shooter.testFeederCommand());
				if (Constants.ENABLE_HOPPER && hopper != null) {
					panel.key(0, 2).whileTrue(
							shooter.spinUpAndWaitCommand(shooter::getShooterTestRPM, shooter::getFeederTestRPM)
									.andThen(hopper.setHopperVelocityCommand(hopper::getHopperTestRPM))
									.finallyDo(() -> {
										shooter.stop();
										hopper.setHopperVelocity(RPM.of(0));
									}));
				}
				panel.key(0, 3).onTrue(shooter.stopCommand());
			}
			// Row 1 — Hood
			if (Constants.ENABLE_SHOOTER && shooter != null) {
				panel.key(1, 0).onTrue(shooter.homeHoodCommand());
				panel.key(1, 1).onTrue(shooter.testHoodCommand());
				panel.key(1, 2).onTrue(shooter.testFullMotorCommand());
			}
			// Row 2 — Intake
			if (Constants.ENABLE_INTAKE && intake != null) {
				panel.key(2, 0).whileTrue(intake.extendCommand());
				panel.key(2, 1).whileTrue(intake.retractCommand());
				panel.key(2, 2).whileTrue(intake.runRollerCommand());
				panel.key(2, 3).whileTrue(intake.ejectCommand());
			}
			// Row 3 — Hopper + Aim tests
			if (Constants.ENABLE_HOPPER && hopper != null) {
				panel.key(3, 0).whileTrue(hopper.testHopperCommand());
				panel.key(3, 1).whileTrue(hopper.feedCommand());
			}
			if (Constants.ENABLE_SWERVE && drivebase != null) {
				Supplier<Pose2d> hubPose = () -> FieldZones.getHubPose(
						DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
				panel.key(3, 2).whileTrue(drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, hubPose)); // bang-bang
				panel.key(3, 3)
						.whileTrue(drivebase.aimAtPID(driverXbox::getLeftX, driverXbox::getLeftY, hubPose)); // profiled PID
			}
			// Row 4 — Climber
			if (Constants.ENABLE_CLIMBER && climber != null) {
				panel.key(4, 0).onTrue(climber.homeClimberCommand());
				panel.key(4, 1).onTrue(climber.extendCommand());
				panel.key(4, 2).onTrue(climber.retractCommand());
				panel.key(4, 3).onTrue(climber.zeroCommand());
				// Xbox left trigger + joystick: fine position control
				driverXbox.leftTrigger(0.7).whileTrue(
						climber.manualControlCommand(() -> (m_JoystickL.getY() / 70.0)));
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
				// Collect
				repulsor.navigateTo(
								() -> _Rebuilt2026.CENTER_COLLECT.poseForCurrentAlliance(SetpointContext.EMPTY))
						.until(repulsor.within(Meters.of(0.15))),
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
	}

	public void autonomousInit() {
	}

	public void setMotorBrake(boolean brake) {
		if (Constants.ENABLE_SWERVE) {
			drivebase.setMotorBrake(brake);
		}
	}
}
