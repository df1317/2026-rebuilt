package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.OurSwerveInputStream;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.avoidance.FieldZones;

import java.io.File;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

/**
 * ---------- RobotContainer Class --- This class is where the bulk of the robot should be declared. Since Command-based
 * is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and trigger
 * mappings) should be declared here. ---
 */
public class RobotContainer {

	private final SendableChooser<Command> autoChooser;
	/**
	 * ---------- HID Initialization ------------
	 */
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final CommandJoystick m_JoystickR = new CommandJoystick(2);
	/**
	 * ---------- Subsystems ------------
	 */
	private final SwerveSubsystem drivebase = new SwerveSubsystem(
			new File(Filesystem.getDeployDirectory(), "swerve/neo"));
	private final ClimberSubsystem climber = new ClimberSubsystem();
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	public boolean robotRelative = false;

	/**
	 * ---------- Swerve Drive Input Streams ------------
	 * -------------------------------------------------- Converts driver input into a field-relative
	 * ChassisSpeeds that is controlled by angular velocity.
	 */
	OurSwerveInputStream driveAngularVelocity = OurSwerveInputStream
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

	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
	 */
	OurSwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
			.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
	 */
	OurSwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
			.allianceRelativeControl(false);

	/**
	 * The container for the robot. Contains subsystems, input devices, and commands.
	 */
	public RobotContainer() {
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("misc/Auto Chooser", autoChooser);
	}

	/**
	 * Configure the button bindings for driver and operator controls.
	 */
	private void configureBindings() {
		Command driveFieldOrientedAnglularVelocity = drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative);

		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		// Zero gyro
		driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

		// Toggle robot relative
		driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative))
				.and(DriverStation::isTeleop);

		// Lock drivebase
		driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

		// ========== Shooter Controls ==========
		// X: Hold to shoot based on distance to target
		driverXbox.x().whileTrue(shooter.shootForDistanceCommand(this::getDistanceToTarget));
		// Y: Hold to shoot at fixed RPM
		driverXbox.y().whileTrue(shooter.shootCommand(RPM.of(3500)));

		// Center modules (test mode only)
		driverXbox.back().whileTrue(
				Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));

		// ========== Climber Controls (Left Joystick) ==========
		// Thumb cluster top: Extend climber
		m_JoystickL.button(3).whileTrue(climber.extendCommand());

		// Thumb cluster bottom: Retract climber
		m_JoystickL.button(4).whileTrue(climber.retractCommand());

		// Trigger: Manual control with joystick Y axis
		m_JoystickL.trigger().whileTrue(
				climber.manualControlCommand(() -> MathUtil.applyDeadband(-m_JoystickL.getY(), 0.1)));

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
		return autoChooser.getSelected();
	}

	/**
	 * Sets brake mode on all swerve drive motors.
	 *
	 * @param brake
	 *          true to enable brake mode, false for coast mode
	 */
	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	/**
	 * Gets the distance to our alliance's scoring target.
	 */
	private Distance getDistanceToTarget() {
		Pose2d hubPose = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
				? FieldZones.HUB_POSE_RED
				: FieldZones.HUB_POSE_BLUE;
		return Meters.of(drivebase.getPose().getTranslation().getDistance(hubPose.getTranslation()));
	}
}
