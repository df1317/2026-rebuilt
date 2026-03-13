package frc.robot;

import dev.doglog.DogLog;
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
import swervelib.SwerveInputStream;

import java.io.File;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

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
					drivebase::getPose);
			teleopAutomation.configureTriggers(driverXbox.rightBumper());

			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
					.aim(teleopAutomation.getShootingPose()).aimWhile(driverXbox.y())
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
			if (Constants.ENABLE_HOPPER) {
				m_JoystickL.button(2).toggleOnTrue(shooter.spinUpAndWaitCommand(RPM.of(3000))
						.andThen(Commands.sequence(Commands.runOnce(() -> shooter.setFeederVelocity(RPM.of(3000))),
								Commands.waitUntil(shooter::isFeederAtSpeed)))
						.andThen(Commands.runOnce(() -> hopper.setHopperVelocity(RPM.of(2000))))
						.finallyDo(() -> {
							shooter.stop();
							hopper.setHopperVelocity(RPM.of(0.0));
						}));

			}
			driverXbox.rightTrigger(0.3).whileTrue(Commands.runEnd(() -> {
				shooter.setHoodPercent(shooter.getTargetHoodPercent() + 0.05);
			}, shooter::hoodStop, shooter));
			driverXbox.leftTrigger(0.3).whileTrue(Commands.runEnd(() -> {
				shooter.setHoodPercent(shooter.getTargetHoodPercent() - 0.05);
			}, shooter::hoodStop, shooter));
		}

		// ===== Test Mode Controls =====
		if (DriverStation.isTest()) {
			if (Constants.ENABLE_SHOOTER && shooter != null) {
				// Hood homing: hold 9 + joystick to jog, press 5 to mark min, press 6 to mark max
				// Button 4: auto-home (drives to hard stops automatically)
				// Button 7/8: test flywheel / feeder individually
				m_JoystickL.button(4).onTrue(shooter.homeHoodCommand());
				m_JoystickL.button(5).onTrue(shooter.testHoodCommand());
				m_JoystickL.button(6).onTrue(shooter.testFullMotorCommand());
				// m_JoystickL.button(5).onTrue(shooter.markHoodMinHereCommand());
				// m_JoystickL.button(6).onTrue(shooter.markHoodMaxHereCommand());
				m_JoystickL.button(7).whileTrue(shooter.testShooterMotorCommand());
				m_JoystickL.button(8).whileTrue(shooter.testFeederCommand());
				// m_JoystickL.button(9).whileTrue(shooter.jogHoodCommand(m_JoystickL::getY));
			}
			if (Constants.ENABLE_INTAKE && intake != null) {
				m_JoystickL.button(10).whileTrue(intake.testPivotCommand());
				m_JoystickL.button(11).whileTrue(intake.testRollerCommand());
			}
			if (Constants.ENABLE_CLIMBER && climber != null) {
				driverXbox.leftTrigger(0.7).whileTrue(
						climber.manualControlCommand(() -> (m_JoystickL.getY() / 70.0)));

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
