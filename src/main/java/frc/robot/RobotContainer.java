package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldZones;
import swervelib.SwerveInputStream;

/**
 * ---------- RobotContainer Class --- This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be
 * handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the
 * structure of the robot (including subsystems, commands, and trigger mappings) should be declared
 * here. ---
 */
public class RobotContainer {

	private SendableChooser<Command> autoChooser;
	/**
	 * ---------- HID Initialization ------------
	 */
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final CommandJoystick m_JoystickR = new CommandJoystick(2);
	/**
	 * ---------- Subsystems ------------
	 */
	private final SwerveSubsystem drivebase = Constants.ENABLE_SWERVE
			? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"))
			: null;
	private final ClimberSubsystem climber = Constants.ENABLE_CLIMBER ? new ClimberSubsystem() : null;
	private final ShooterSubsystem shooter = Constants.ENABLE_SHOOTER ? new ShooterSubsystem() : null;
	private final IntakeSubsystem intake = Constants.ENABLE_INTAKE ? new IntakeSubsystem() : null;
	public boolean robotRelative = false;

	/**
	 * ---------- Swerve Drive Input Streams ------------
	 * -------------------------------------------------- Converts driver input into a field-relative
	 * ChassisSpeeds that is controlled by angular velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream
			.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
					() -> driverXbox.getLeftX() * -1)
			.withControllerRotationAxis(() -> {
				// Right stick X for rotation, plus triggers for fine-tuning (cubic scaling)
				// Right trigger = clockwise (negative), Left trigger = counter-clockwise (positive)
				double stickRotation = driverXbox.getRightX() * -1;
				double leftTrigger = Math.pow(driverXbox.getLeftTriggerAxis(), 3);
				double rightTrigger = Math.pow(driverXbox.getRightTriggerAxis(), 3);
				double triggerRotation = (leftTrigger - rightTrigger) * 0.3;
				return MathUtil.clamp(stickRotation + triggerRotation, -1.0, 1.0);
			}).aim(FieldZones.HUB_POSE_RED).aimWhile(driverXbox.b())
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);
	 /* Converts driver input into a field-relative ChassisSpeeds controlled by angular velocity.
	 */
	private SwerveInputStream driveAngularVelocity;

	/**
	 * The container for the robot. Contains subsystems, input devices, and commands.
	 */
	public RobotContainer() {
		if (Constants.ENABLE_SWERVE) {
			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> {
						// Right stick X for rotation, plus triggers for fine-tuning (cubic scaling)
						double stickRotation = driverXbox.getRightX() * -1;
						double leftTrigger = Math.pow(driverXbox.getLeftTriggerAxis(), 3);
						double rightTrigger = Math.pow(driverXbox.getRightTriggerAxis(), 3);
						double triggerRotation = (leftTrigger - rightTrigger) * 0.3;
						return MathUtil.clamp(stickRotation + triggerRotation, -1.0, 1.0);
					}).aim(FieldZones.HUB_POSE_RED).aimWhile(driverXbox.b())
					.deadband(OperatorConstants.DEADBAND)
					.scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);

			autoChooser = AutoBuilder.buildAutoChooser();
			SmartDashboard.putData("misc/Auto Chooser", autoChooser);
		}

		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	/**
	 * Configure the button bindings for driver and operator controls.
	 */
	private void configureBindings() {
		// ========== Swerve Controls ==========
		if (Constants.ENABLE_SWERVE) {
			drivebase
					.setDefaultCommand(drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative));

			// Hold X to aim at the target
			driverXbox.x().whileTrue(
					drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, FieldZones.HUB_POSE_BLUE));

			// Zero gyro
			driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

			// Toggle robot relative
			driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative))
					.and(DriverStation::isTeleop);

			// Lock drivebase
			driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

			// Center modules (test mode only)
			driverXbox.back().whileTrue(
					Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));
		}

		// ========== Shooter Controls ==========
		if (Constants.ENABLE_SHOOTER) {
			// X: Hold to shoot based on distance to target
			// driverXbox.x().whileTrue(shooter.shootForDistanceCommand(this::getDistanceToTarget));
			// Y: Hold to shoot at fixed RPM
			// driverXbox.y().whileTrue(shooter.shootCommand(RPM.of(3500)));

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
			// Thumb cluster top: Extend climber
			m_JoystickL.button(3).whileTrue(climber.extendCommand());

			// Thumb cluster bottom: Retract climber
			m_JoystickL.button(4).whileTrue(climber.retractCommand());

			// Trigger: Manual control with joystick Y axis
			m_JoystickL.trigger().whileTrue(
					climber.manualControlCommand(() -> MathUtil.applyDeadband(-m_JoystickL.getY(), 0.1)));
		}

		// ========== Intake Controls ==========
		if (Constants.ENABLE_INTAKE) {
			// driverXbox.y().onTrue(intake.runRollerCommand());
			// driverXbox.y().onFalse(intake.stopRollerCommand());

			driverXbox.y().onTrue(intake.extendCommand());
			driverXbox.y().onFalse(intake.retractCommand());
		}

		// ========== Autopilot Examples ==========
		// Uncomment these to enable Autopilot drive-to-pose commands during testing
		//
		// Example 1: Drive to scoring position (field coordinates)
		// driverXbox.x().whileTrue(
		// drivebase.driveToPoseAutopilot(() -> new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0)))
		// );
		//
		// Example 2: Drive to amp with entry angle (approach from specific direction)
		// driverXbox.y().whileTrue(
		// drivebase.driveToPoseAutopilot(() -> FieldConstants.ampPose, true)
		// );
		//
		// Example 3: Drive to pose and finish (command completes when at target)
		// driverXbox.b().whileTrue(
		// drivebase.driveToPoseAutopilotUntilFinished(
		// () -> new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(45)),
		// 0.05, // 5cm tolerance
		// Math.toRadians(2) // 2 degree tolerance
		// )
		// );
		//
		// Example 4: Static target convenience method
		// driverXbox.povUp().whileTrue(
		// drivebase.driveToPoseAutopilot(new Pose2d(1.0, 1.0, new Rotation2d()))
		// );
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		if (Constants.ENABLE_SWERVE && autoChooser != null) {
			return autoChooser.getSelected();
		}
		return Commands.none();
	}

	/**
	 * Sets brake mode on all swerve drive motors.
	 *
	 * @param brake
	 *          true to enable brake mode, false for coast mode
	 */
	public void setMotorBrake(boolean brake) {
		if (Constants.ENABLE_SWERVE) {
			drivebase.setMotorBrake(brake);
		}
	}

	/**
	 * Gets the distance to our alliance's scoring target.
	 */
	private Distance getDistanceToTarget() {
		Pose2d hubPose = DriverStation.getAlliance()
				.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? FieldZones.HUB_POSE_RED
						: FieldZones.HUB_POSE_BLUE;
		return Meters.of(drivebase.getPose().getTranslation().getDistance(hubPose.getTranslation()));
	}
}
