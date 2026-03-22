package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopZoneAutomation;
import frc.robot.repulsor.IntakeFootprint;
import frc.robot.repulsor.Repulsor;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldZones;
import frc.robot.util.TunableBoolean;
import frc.robot.util.TunableTable;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RPM;

public class RobotContainer {

	// HID
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final OperatorPanel panel = new OperatorPanel(1);
	// Subsystems
	private final SwerveSubsystem drivebase = Constants.ENABLE_SWERVE
			? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"))
			: null;
	private final ClimberSubsystem climber = Constants.ENABLE_CLIMBER ? new ClimberSubsystem() : null;
	private final ShooterSubsystem shooter = Constants.ENABLE_SHOOTER ? new ShooterSubsystem() : null;
	private final RollerSubsystem roller = Constants.ENABLE_INTAKE ? new RollerSubsystem() : null;
	private final IntakeSubsystem intake = Constants.ENABLE_INTAKE ? new IntakeSubsystem(roller) : null;
	private final HopperSubsystem hopper = Constants.ENABLE_HOPPER ? new HopperSubsystem() : null;
	private static final TunableTable driveTunables = new TunableTable("Drive");
	private final TunableBoolean obstacleClampEnabled = driveTunables.value("ObstacleClampEnabled", true);
	private final TunableBoolean repulsorRumbleEnabled = driveTunables.value("RepulsorRumbleEnabled", true);
	// Repulsor
	private final Repulsor repulsor;
	private final Autos autos;
	private final SwerveInputStream driveAngularVelocity;
	// Teleop automation
	private final TeleopZoneAutomation teleopAutomation;
	public boolean robotRelative = false;

	public RobotContainer() {
		if (Constants.ENABLE_SWERVE) {
			// Initialize Repulsor path planner with intake footprint
			IntakeFootprint stowedFootprint = IntakeFootprint.robotRect(
					DrivebaseConstants.ROBOT_HALF_LENGTH * 2.0,
					DrivebaseConstants.ROBOT_HALF_WIDTH * 2.0);
			IntakeFootprint extendedFootprint = IntakeFootprint.robotWithIntake(
					DrivebaseConstants.ROBOT_HALF_LENGTH * 2.0,
					DrivebaseConstants.ROBOT_HALF_WIDTH * 2.0,
					DrivebaseConstants.INTAKE_LENGTH_METERS,
					DrivebaseConstants.INTAKE_ANGLE_DEG);
			IntakeFootprint.setFootprints(stowedFootprint, extendedFootprint,
					() -> intake != null && intake.isExtended());
			repulsor = new Repulsor(drivebase,
					stowedFootprint.getEffectiveHalfLength(),
					stowedFootprint.getEffectiveHalfWidth());

			// Setup teleop automation
			teleopAutomation = new TeleopZoneAutomation(
					repulsor, intake, shooter, hopper,
					drivebase::getPose, drivebase::getFieldVelocity);
			drivebase.setTargetDistanceSupplier(teleopAutomation::getTargetDistance);
			drivebase.setAimTargetSupplier(teleopAutomation::getVirtualAimTarget);
			if (Constants.ENABLE_SHOOTER && shooter != null) {
				shooter.setAutoDistanceSupplier(teleopAutomation::getTargetDistance);
			}
			if (Constants.ENABLE_INTAKE && roller != null) {
				roller.setRobotSpeedSupplier(() -> {
					var vel = drivebase.getFieldVelocity();
					return Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
				});
			}

			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
					.aimWhile(driverXbox.y())
					.deadband(OperatorConstants.DEADBAND)
					.scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);

			autos = new Autos(repulsor, drivebase, teleopAutomation, climber, intake, roller);
		}

		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	// ===== Auto Routines =====

	private void configureBindings() {
		var inTeleop = new edu.wpi.first.wpilibj2.command.button.Trigger(DriverStation::isTeleop);
		var inTest = new edu.wpi.first.wpilibj2.command.button.Trigger(DriverStation::isTest);

		// ===== Driver Controls (Xbox port 0) =====
		if (Constants.ENABLE_SWERVE) {
			drivebase.setDefaultCommand(
					drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative,
							speeds -> {
								if (obstacleClampEnabled.get() && repulsor != null) {
									ChassisSpeeds clamped = repulsor.clampDriveSpeed(speeds, drivebase.getPose());
									if (repulsorRumbleEnabled.get()) {
										double rumble = repulsor.getRepulsionIntensity() * 0.5;
										driverXbox.getHID().setRumble(RumbleType.kBothRumble, rumble);
									} else {
										driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
									}
									return clamped;
								}
								driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
								return speeds;
							}));

			driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
			driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative));
		}
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			driverXbox.rightTrigger().whileTrue(Commands.runOnce(() -> {
				// If vision is healthy, reset to auto distance
				if (Constants.ENABLE_SWERVE && drivebase != null
						&& drivebase.hasVision() && !drivebase.isVisionStale()) {
					shooter.clearManualDistanceOverride();
				}
			}).andThen(Constants.ENABLE_SWERVE && drivebase != null
					? Commands.either(
							// Vision healthy: aim + auto distance
							Commands.parallel(
									teleopAutomation.shootCommand(drivebase::isAimed),
									drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY,
											teleopAutomation::getShootingPose)),
							// Vision stale/disabled: manual distance, no aim
							teleopAutomation.shootCommand(),
							() -> drivebase.hasVision() && !drivebase.isVisionStale())
					: teleopAutomation.shootCommand()));
		}
		if (Constants.ENABLE_INTAKE && intake != null) {
			driverXbox.x().onTrue(intake.stowToggleCommand());
			panel.key(1, 2).and(inTeleop).onTrue(intake.stowToggleCommand());
			driverXbox.leftTrigger().and(inTeleop).whileTrue(roller.intakeCommand());
			driverXbox.leftTrigger().and(inTeleop).whileTrue(intake.holdExtendedCommand());
			panel.key(1, 3).and(inTeleop).whileTrue(roller.intakeCommand());
			panel.key(1, 3).and(inTeleop).whileTrue(intake.holdExtendedCommand());
		}

		// ===== Teleop Panel Controls (Maypad — see docs for layout) =====
		// Row 2 — Feed / Intake
		if (Constants.ENABLE_INTAKE && roller != null && intake != null) {
			panel.key(2, 1).and(inTeleop).whileTrue(roller.runRollerCommand()); // intakeForward
			panel.key(3, 1).and(inTeleop).whileTrue(roller.ejectCommand()); // intakeReverse
			panel.key(0, 1).and(inTeleop).onTrue(intake.zeroIntakeCommand());
			panel.key(0, 2).and(inTeleop).whileTrue(intake.jogDownCommand());
		}
		if (Constants.ENABLE_HOPPER && hopper != null) {
			panel.key(2, 2).and(inTeleop).whileTrue(hopper.feedCommand()); // hopperForward
			panel.key(3, 2).and(inTeleop).whileTrue(hopper.reverseCommand()); // hopperReverse
		}
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			panel.key(2, 3).and(inTeleop).whileTrue(teleopAutomation.shootCommand()); // shoot+feed (no aim)
			panel.key(3, 3).and(inTeleop).whileTrue(shooter.reverseFeederCommand()); // feederReverse
			panel.key(1, 0).and(inTeleop).onTrue(Commands.runOnce(() -> { // autoDistance
				if (Constants.ENABLE_SWERVE && drivebase != null
						&& drivebase.hasVision() && !drivebase.isVisionStale()) {
					shooter.clearManualDistanceOverride();
				}
			}));
			panel.key(2, 0).and(inTeleop).onTrue(shooter.advanceDistanceCommand()); // distanceAdvance
			panel.key(3, 0).and(inTeleop).onTrue(shooter.reduceDistanceCommand()); // distanceReduce
		}
		// Row 4 — Climber positions
		if (Constants.ENABLE_CLIMBER && climber != null) {
			panel.key(4, 0).and(inTeleop).onTrue(climber.climbBottomCommand()); // climbBottom
			panel.key(4, 1).and(inTeleop).onTrue(climber.climbTopCommand()); // climbTop
			panel.key(4, 2).and(inTeleop).onTrue(climber.climbHangCommand()); // climbHang
			panel.key(4, 3).and(inTeleop).onTrue(climber.climbReleaseCommand()); // climbRelease
		}

		// ===== Test Mode Controls (Maypad — see docs for layout) =====
		// Row 0 — Climber / Intake
		if (Constants.ENABLE_INTAKE && intake != null) {
			panel.key(0, 0).onTrue(intake.homeCommand()); // intakeHome
			panel.key(2, 1).and(inTest).onTrue(intake.zeroIntakeCommand());
			panel.key(2, 2).and(inTest).whileTrue(intake.jogDownCommand());
		}
		if (Constants.ENABLE_CLIMBER && climber != null) {
			panel.key(0, 1).and(inTest).onTrue(climber.zeroCommand());
			panel.key(0, 2).and(inTest).whileTrue(climber.jogVoltageCommand(() -> 1.0)); // climberUp
			panel.key(0, 3).and(inTest).whileTrue(climber.jogVoltageCommand(() -> -1.0)); // climberDown
		}
		// Row 1 — Hood + Aim
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			panel.key(1, 1).onTrue(shooter.homeHoodCommand());
			panel.key(1, 2).and(inTest).whileTrue(shooter.testHoodCommand());
			panel.key(1, 3).and(inTest).whileTrue(shooter.testShooterMotorCommand());
		}
		if (Constants.ENABLE_SWERVE && drivebase != null) {
			Supplier<Pose2d> hubPose = () -> FieldZones.getHubPose(
					DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
			panel.key(1, 0).and(inTest).whileTrue(drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, hubPose));
		}
		// Row 3 — Individual subsystem tests
		if (Constants.ENABLE_SHOOTER && shooter != null) {
			panel.key(2, 3).and(inTest).whileTrue(
					Commands.parallel(
							Commands.runOnce(() -> shooter.setTestHoodPercent()),
							shooter.spinUpAndWaitCommand(shooter::getShooterTestRPM, shooter::getFeederTestRPM))
							.andThen(Constants.ENABLE_HOPPER && hopper != null
									? hopper.setHopperVelocityCommand(hopper::getHopperTestRPM)
									: Commands.none())
							.finallyDo(() -> {
								shooter.stop();
								if (Constants.ENABLE_HOPPER && hopper != null) {
									hopper.setHopperVelocity(RPM.of(0));
								}
							}));
			panel.key(3, 3).and(inTest).whileTrue(shooter.testFeederCommand());
		}
		if (Constants.ENABLE_HOPPER && hopper != null) {
			panel.key(3, 2).and(inTest).whileTrue(hopper.testHopperCommand());
		}
		if (Constants.ENABLE_INTAKE && intake != null) {
			panel.key(3, 1).and(inTest).whileTrue(intake.testPivotCommand());
		}
	}

	public void updateRepulsor() {
		if (repulsor != null) {
			repulsor.update();
		}
	}

	public void setMotorBrake(boolean brake) {
		if (Constants.ENABLE_SWERVE) {
			drivebase.setMotorBrake(brake);
		}
	}
}
