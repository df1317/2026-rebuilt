package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final DoubleSubscriber HoodPrecent = DogLog.tunable("hoodprcent",0.0);
  private final DoubleSubscriber shooterRPM = DogLog.tunable("shooterRPM",3000.0);

	// HID
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final OperatorPanel panel = new OperatorPanel(1);
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
		var inTeleop = new edu.wpi.first.wpilibj2.command.button.Trigger(DriverStation::isTeleop);
		var inTest = new edu.wpi.first.wpilibj2.command.button.Trigger(DriverStation::isTest);

		// ===== Driver Controls (Xbox port 0) — teleop only =====
		if (Constants.ENABLE_SWERVE) {
			drivebase.setDefaultCommand(
					drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative,
							speeds -> {
								if (obstacleClampEnabled.get() && repulsor != null) {
									return repulsor.clampDriveSpeed(speeds, drivebase.getPose());
								}
								return speeds;
							}));

			driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
			driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative));
		}
		if (Constants.ENABLE_SHOOTER) {
			driverXbox.rightTrigger().whileTrue(Constants.ENABLE_SWERVE && drivebase != null
					? Commands.parallel(
							teleopAutomation.shootCommand(drivebase::isAimed),
							drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY,
									teleopAutomation::getShootingPose))
					: teleopAutomation.shootCommand());
		}
		if (Constants.ENABLE_INTAKE && intake != null) {
			driverXbox.x().onTrue(intake.stowToggleCommand());
			driverXbox.leftTrigger().whileTrue(intake.intakeCommand());
		}

    panel.key(1,3).whileTrue(shooter.tune(shooterRPM::get,shooterRPM::get,HoodPrecent::get).finallyDo(()->{
        shooter.stop();
    }));

		// ===== Teleop Panel Controls (Maypad — see docs for layout) =====
		// Row 2 — Feed / Intake
		if (Constants.ENABLE_INTAKE && intake != null) {
			panel.key(2, 1).whileTrue(intake.runRollerCommand());   // intakeForward
			panel.key(3, 1).whileTrue(intake.ejectCommand());       // intakeReverse
		}
		if (Constants.ENABLE_HOPPER && hopper != null) {
			panel.key(2, 2).and(inTeleop).whileTrue(hopper.feedCommand());         // hopperForward
			panel.key(3, 2).and(inTeleop).whileTrue(hopper.reverseCommand());      // hopperReverse
		}
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			panel.key(2, 3).and(inTeleop).whileTrue(teleopAutomation.shootCommand()); // shoot+feed (no aim)
			panel.key(3, 3).and(inTeleop).whileTrue(shooter.reverseFeederCommand()); // feederReverse
		}

		// ===== Test Mode Controls (Maypad — see docs for layout) =====
		// Row 0 — Climber
		if (Constants.ENABLE_CLIMBER && climber != null) {
			panel.key(0, 0).and(inTest).onTrue(climber.homeClimberCommand());
			panel.key(0, 1).and(inTest).onTrue(climber.zeroCommand());
			panel.key(0, 2).and(inTest).whileTrue(climber.jogVoltageCommand(() -> 1.0));  // climberUp
			panel.key(0, 3).and(inTest).whileTrue(climber.jogVoltageCommand(() -> -1.0)); // climberDown
		}
		// Row 1 — Hood + Aim
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			panel.key(1, 1).onTrue(shooter.homeHoodCommand());
//			panel.key(1, 2).and(inTest).whileTrue(shooter.testHoodCommand());
		}
		if (Constants.ENABLE_SWERVE && drivebase != null) {
			Supplier<Pose2d> hubPose = () -> FieldZones.getHubPose(
					DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
			panel.key(1, 0).and(inTest).whileTrue(drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, hubPose));
			panel.key(1, 3).and(inTest).whileTrue(drivebase.aimAtPID(driverXbox::getLeftX, driverXbox::getLeftY, hubPose));
		}
		// Row 2 — Shoot all
		if (Constants.ENABLE_SHOOTER && shooter != null && Constants.ENABLE_HOPPER && hopper != null) {
			panel.key(2, 3).and(inTest).whileTrue(
					shooter.spinUpAndWaitCommand(shooter::getShooterTestRPM, shooter::getFeederTestRPM)
							.andThen(hopper.setHopperVelocityCommand(hopper::getHopperTestRPM))
							.finallyDo(() -> {
								shooter.stop();
								hopper.setHopperVelocity(RPM.of(0));
							}));
		}
		// Row 3 — Individual subsystem tests
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			panel.key(3, 0).and(inTest).whileTrue(shooter.testShooterMotorCommand());
			panel.key(3, 1).and(inTest).whileTrue(shooter.testFeederCommand());
		}
		if (Constants.ENABLE_HOPPER && hopper != null) {
			panel.key(3, 2).and(inTest).whileTrue(hopper.testHopperCommand());
		}
		if (Constants.ENABLE_INTAKE && intake != null) {
			panel.key(3, 3).and(inTest).whileTrue(intake.testPivotCommand());
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
