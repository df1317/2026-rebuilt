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

public class RobotContainer {

	private SendableChooser<Command> autoChooser;

	// HID
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final CommandJoystick m_JoystickR = new CommandJoystick(2);

	// Subsystems
	private final SwerveSubsystem drivebase = Constants.ENABLE_SWERVE
			? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"))
			: null;
	private final ClimberSubsystem climber = Constants.ENABLE_CLIMBER ? new ClimberSubsystem() : null;
	private final ShooterSubsystem shooter = Constants.ENABLE_SHOOTER ? new ShooterSubsystem() : null;
	private final IntakeSubsystem intake = Constants.ENABLE_INTAKE ? new IntakeSubsystem() : null;
	public boolean robotRelative = false;

	private SwerveInputStream driveAngularVelocity;

	public RobotContainer() {
		if (Constants.ENABLE_SWERVE) {
			driveAngularVelocity = SwerveInputStream
					.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
							() -> driverXbox.getLeftX() * -1)
					.withControllerRotationAxis(() -> {
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

	private void configureBindings() {
		if (Constants.ENABLE_SWERVE) {
			drivebase
					.setDefaultCommand(drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative));

			driverXbox.x().whileTrue(
					drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, FieldZones.HUB_POSE_BLUE));

			driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

			driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative))
					.and(DriverStation::isTeleop);

			driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

			driverXbox.back().whileTrue(
					Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));
		}

		if (Constants.ENABLE_SHOOTER) {
			m_JoystickL.povRight().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(Units.RPM.of(1000.0));
				shooter.setFeederVelocity(Units.RPM.of(1000.0));
				System.out.println("shooter set to 1000RPM");
			}));
			m_JoystickL.povLeft().onTrue(Commands.runOnce(() -> {
				System.out.println("shooter stopped");
				shooter.stop();
			}));
			m_JoystickL.povUp().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(shooter.getTargetVelocity().plus(Units.RPM.of(100.0)));
				shooter.setFeederVelocity(shooter.getTargetVelocity());
				System.out.println(
						"shooter increased by 100rpm to " + (shooter.getTargetVelocity().baseUnitMagnitude()));
			}));
			m_JoystickL.povDown().onTrue(Commands.runOnce(() -> {
				shooter.setVelocity(shooter.getTargetVelocity().minus(Units.RPM.of(100.0)));
				shooter.setFeederVelocity(shooter.getTargetVelocity());
				System.out.println(
						"shooter decreased by 100rpm to " + (shooter.getTargetVelocity().baseUnitMagnitude()));
			}));

			m_JoystickL.button(3).onTrue(Commands.runOnce(() -> {
				shooter.setHoodAngle(shooter.getTargetHoodAngle().plus(Degrees.of(10)));
			}));
			m_JoystickL.button(4).onTrue(Commands.runOnce(() -> {
				shooter.setHoodAngle(shooter.getTargetHoodAngle().minus(Degrees.of(10)));
			}));
		}

		if (Constants.ENABLE_CLIMBER) {
			m_JoystickL.button(5).whileTrue(climber.extendCommand());
			m_JoystickL.button(6).whileTrue(climber.retractCommand());
			m_JoystickL.trigger().whileTrue(
					climber.manualControlCommand(() -> MathUtil.applyDeadband(-m_JoystickL.getY(), 0.1)));
		}

		if (Constants.ENABLE_INTAKE) {
			driverXbox.y().toggleOnTrue(intake.runRollerCommand());
			driverXbox.y().toggleOnFalse(intake.stopRollerCommand());

			m_JoystickL.button(8).onTrue(intake.extendCommand());
			m_JoystickL.button(7).onFalse(intake.retractCommand());
		}
	}

	public Command getAutonomousCommand() {
		if (Constants.ENABLE_SWERVE && autoChooser != null) {
			return autoChooser.getSelected();
		}
		return Commands.none();
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
