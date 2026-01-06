package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.TargetingSubsystem.Side;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/** ----------
 * RobotContainer Class
 * ---
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 * ---
 */
public class RobotContainer {

	private final SendableChooser<Command> autoChooser;

	public boolean robotRelative = false;

	/** ----------
	 * HID Initialization
	 * ------------ */
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final CommandJoystick m_JoystickR = new CommandJoystick(2);

	/** ----------
	 * Subsystems
	 * ------------ */
	private final SwerveSubsystem drivebase = new SwerveSubsystem(
		new File(Filesystem.getDeployDirectory(), "swerve/neo")
	);
	private final TargetingSubsystem targetingSubsystem = new TargetingSubsystem();

	/** ----------
	 * Swerve Drive Input Streams
	 * ------------ */

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled
	 * by angular velocity.
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
	 * Clone's the angular velocity input stream and converts it to a fieldRelative
	 * input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity
		.copy()
		.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
		.headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative
	 * input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity
		.copy()
		.robotRelative(true)
		.allianceRelativeControl(false);

	SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
		drivebase.getSwerveDrive(),
		() -> -driverXbox.getLeftY(),
		() -> -driverXbox.getLeftX()
	)
		.withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(0.9)
		.allianceRelativeControl(true);
	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
		.copy()
		.withControllerHeadingAxis(
			() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
			() -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)
		)
		.headingWhile(true);

	/** ----------
	 * RobotContainer Root Class
	 * ---
	 * This is the root class for the robot. It is responsible for configuring the robot, its subsystems, and bindings.
	 * ---
	 */
	public RobotContainer() {
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("misc/Auto Chooser", autoChooser);
	}

	/** ----------
	 * Configure the button bindings
	 * ---
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link CommandButton} with a {@link Command} and then calling the
	 * various button-press functions on it.
	 * ---
	 */
	private void configureBindings() {
		// NOTE: Avoid duplicating button bindings in the same mode (teleop, test, etc.)
		// Always check for existing bindings before assigning new ones
		// Use comments to document button assignments for better clarity

		Command driveFieldOrientedAnglularVelocity = drivebase.robotDriveCommand(driveAngularVelocity, () ->
			robotRelative
		);

		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		/** ------------------------------------- *
		 * Xbox Swerve and Navigation bindings
		 * ---
		 * drivebase locking and fake vision reading
		 * zero gyro and print mode
		 * center modules and none
		 * zero elevator and none
		 * drive to pose (for sysid) and none
		 * ---
		 */

		driverXbox
			.x()
			.onTrue(
				targetingSubsystem
					.autoTargetPairCommand(drivebase::getPose, Side.LEFT)
					.andThen(
						Commands.either(
							targetingSubsystem.driveToCoralTarget(drivebase, 0.6, 1),
							Commands.runEnd(
								() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 1),
								() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0)
							).withTimeout(.15),
							() -> targetingSubsystem.areWeAllowedToDrive(drivebase::getPose)
						)
					)
			);

		driverXbox
			.b()
			.onTrue(
				targetingSubsystem
					.autoTargetPairCommand(drivebase::getPose, Side.RIGHT)
					.andThen(
						Commands.either(
							targetingSubsystem.driveToCoralTarget(drivebase, 0.6, 1),
							Commands.runEnd(
								() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 1),
								() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0)
							).withTimeout(.15),
							() -> targetingSubsystem.areWeAllowedToDrive(drivebase::getPose)
						)
					)
			);

		// this effectively cancels the auto align
		driverXbox.y().onTrue(drivebase.driveToPose(drivebase::getPose));

		driverXbox
			.a()
			.onTrue(
				Commands.runOnce(drivebase::zeroGyro).andThen(
					Commands.print(DriverStation.isTest() ? "Test Mode: Reset Gyro" : "Other Mode: Reset Gyro")
				)
			);

		driverXbox
			.back()
			.whileTrue(Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));

		driverXbox
			.rightBumper()
			.onTrue(
				Commands.runOnce(() -> {
					robotRelative = robotRelative ? false : true;
				})
			)
			.and(DriverStation::isTeleop);

		/** -------------------------------------
		 * Xbox Scoring and Intake bindings
		 * ---
		 * duck song and drivebase locking
		 * intake and duck song
		 * eject and none
		 * ---
		 */

		driverXbox
			.leftBumper()
			.whileTrue(
					Commands.runOnce(drivebase::lock, drivebase).repeatedly()
			);
  }

	/** ----------
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * ---
	 * @return the command to run in autonomous
	 * ---
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}
