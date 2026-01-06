package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

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
			new File(Filesystem.getDeployDirectory(), "swerve/neo")
	);
	public boolean robotRelative = false;

	/** ----------
	 * Swerve Drive Input Streams
	 * ------------ */
	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
					drivebase.getSwerveDrive(),
					() -> driverXbox.getLeftY() * -1,
					() -> driverXbox.getLeftX() * -1
			)
			.withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity
			.copy()
			.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
			.headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity
			.copy()
			.robotRelative(true)
			.allianceRelativeControl(false);

	/**
	 * ---------- RobotContainer Root Class --- This is the root class for the robot. It is responsible for configuring
	 * the robot, its subsystems, and bindings. ---
	 */
	public RobotContainer() {
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("misc/Auto Chooser", autoChooser);
	}

	/**
	 * ---------- Configure the button bindings --- Use this method to define your button->command mappings. Buttons can
	 * be created by instantiating a {@link CommandButton} with a {@link Command} and then calling the various
	 * button-press functions on it. ---
	 */
	private void configureBindings() {
		Command driveFieldOrientedAnglularVelocity = drivebase.robotDriveCommand(driveAngularVelocity, () ->
				robotRelative
		);

		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		// Zero gyro
		driverXbox
				.a()
				.onTrue(Commands.runOnce(drivebase::zeroGyro));

		// Toggle robot relative
		driverXbox
				.rightBumper()
				.onTrue(Commands.runOnce(() -> robotRelative = !robotRelative))
				.and(DriverStation::isTeleop);

		// Lock drivebase
		driverXbox
				.leftBumper()
				.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

		// Center modules (test mode only)
		driverXbox
				.back()
				.whileTrue(Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));
	}

	/**
	 * ---------- Use this to pass the autonomous command to the main {@link Robot} class. ---
	 *
	 * @return the command to run in autonomous ---
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}
