package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * ---------- RobotContainer Class --- This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be
 * handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the
 * structure of the robot (including subsystems, commands, and trigger mappings) should be declared
 * here. ---
 */
public class RobotContainer {

  // private final SendableChooser<Command> autoChooser;
  /**
   * ---------- HID Initialization ------------
   */
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandJoystick m_JoystickL = new CommandJoystick(1);
  private final CommandJoystick m_JoystickR = new CommandJoystick(2);
  /**
   * ---------- Subsystems ------------
   */
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(
  // new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  public boolean robotRelative = false;

  /**
   * ---------- Swerve Drive Input Streams ------------
   * -------------------------------------------------- Converts driver input into a field-relative
   * ChassisSpeeds that is controlled by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity =
  // SwerveInputStream.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
  // () -> driverXbox.getLeftX() * -1).withControllerRotationAxis(() -> {
  // Right stick X for rotation, plus triggers for fine-tuning (cubic scaling)
  // Right trigger = clockwise (negative), Left trigger = counter-clockwise (positive)
  // double stickRotation = driverXbox.getRightX() * -1;
  // double leftTrigger = Math.pow(driverXbox.getLeftTriggerAxis(), 3);
  // double rightTrigger = Math.pow(driverXbox.getRightTriggerAxis(), 3);
  // double triggerRotation = (leftTrigger - rightTrigger) * 0.3;
  // return MathUtil.clamp(stickRotation + triggerRotation, -1.0, 1.0);
  // }).aim(FieldZones.HUB_POSE_RED).aimWhile(driverXbox.b())
  // .deadband(OperatorConstants.DEADBAND)
  // .scaleTranslation(DrivebaseConstants.TRANSLATION_SCALE).allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, input devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("misc/Auto Chooser", autoChooser);
  }

  /**
   * Configure the button bindings for driver and operator controls.
   */
  private void configureBindings() {
    // drivebase
    // .setDefaultCommand(drivebase.robotDriveCommand(driveAngularVelocity, () -> robotRelative));

    // Hold X to aim at the target (overrides default drive command while held)
    // driverXbox.x().whileTrue(
    // drivebase.aimAt(driverXbox::getLeftX, driverXbox::getLeftY, FieldZones.HUB_POSE_BLUE));

    // Zero gyro
    // driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Toggle robot relative
    // driverXbox.rightBumper().onTrue(Commands.runOnce(() -> robotRelative = !robotRelative))
    // .and(DriverStation::isTeleop);

    // Lock drivebase
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // ========== Shooter Controls ==========
    // X: Hold to shoot based on distance to target
    // driverXbox.x().whileTrue(shooter.shootForDistanceCommand(this::getDistanceToTarget));
    // Y: Hold to shoot at fixed RPM
    // driverXbox.y().whileTrue(shooter.shootCommand(RPM.of(3500)));

    // Center modules (test mode only)
    // driverXbox.back().whileTrue(
    // Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));

    // ========== Climber Controls (Left Joystick) ==========
    // Thumb cluster top: Extend climber
    m_JoystickL.button(3).whileTrue(climber.extendCommand());

    // Thumb cluster bottom: Retract climber
    m_JoystickL.button(4).whileTrue(climber.retractCommand());

    // Trigger: Manual control with joystick Y axis
    m_JoystickL.trigger().whileTrue(
        climber.manualControlCommand(() -> MathUtil.applyDeadband(-m_JoystickL.getY(), 0.1)));

    // driverXbox.povUp().whileTrue(climber.manualControlCommand(() -> 0.5 / (1000.0 / 20.0)));
    // driverXbox.povDown().whileTrue(climber.manualControlCommand(() -> -0.5 / (1000.0 / 20.0)));
    // driverXbox.povRight().onTrue(climber.goToHeightCommand(5));

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
      System.out.println(
          "shooter increased by 100rpm to " + (shooter.getTargetVelocity().baseUnitMagnitude()));

    }));
    driverXbox.povDown().onTrue(Commands.runOnce(() -> {
      shooter.setVelocity(shooter.getTargetVelocity().minus(Units.RPM.of(100.0)));
      System.out.println(
          "shooter decreased by 100rpm to " + (shooter.getTargetVelocity().baseUnitMagnitude()));

    }));

    driverXbox.rightBumper().onTrue(shooter.hoodSetpoint(Degrees.of(1080.0)));
    driverXbox.leftBumper().onTrue(shooter.hoodSetpoint(Degrees.of(0.0)));

    // ========== Autopilot Examples ==========
    // Uncomment these to enable Autopilot drive-to-pose commands during testing
    //
    // Example 1: Drive to scoring pos ition (field coordinates)()
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
  // public Command getAutonomousCommand() {
  // return autoChooser.getSelected();
  // }

  /**
   * Sets brake mode on all swerve drive motors.
   *
   * @param brake true to enable brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }

  /**
   * Gets the distance to our alliance's scoring target.
   */
  // private Distance getDistanceToTarget() {
  // Pose2d hubPose = DriverStation.getAlliance()
  // .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? FieldZones.HUB_POSE_RED
  // : FieldZones.HUB_POSE_BLUE;
  // return Meters.of(drivebase.getPose().getTranslation().getDistance(hubPose.getTranslation()));
  // }
}
