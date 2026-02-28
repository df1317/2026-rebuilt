package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.util.FieldZones;
import frc.robot.util.RobotLog;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class SwerveSubsystem extends SubsystemBase {

	private static final double AIM_TOLERANCE = Math.toRadians(1);

	private final SwerveDrive swerveDrive;
	private final BooleanSubscriber visionEnabled = DogLog.tunable("Swerve/VisionEnabled", true);
	private final double AIM_SPEED_FAST = 5.0;
	private final double AIM_SPEED_MID = 1.0;
	private final double AIM_SPEED_SLOW = 2.0;
	private final double AIM_SPEED_LOWEST = 0.3;

	Optional<Alliance> prevAlliance = Optional.empty();
	private Vision vision;
	private AutopilotController autopilotController;

	public SwerveSubsystem(File directory) {
		SwerveDriveTelemetry.verbosity = Constants.SwerveTelemetryVerbosity;
		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
					new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
		} catch (Exception e) {
			throw RobotLog.fatal("Swerve/Init", "Swerve init failed",
					"Failed to create SwerveDrive from config directory", e);
		}
		this.replaceSwerveModuleFeedforward(DrivebaseConstants.DRIVE_KS, DrivebaseConstants.DRIVE_KV,
				DrivebaseConstants.DRIVE_KA);

		swerveDrive.setMaximumAllowableSpeeds(Constants.MAX_SPEED, Constants.MAX_ANGULAR_SPEED);

		swerveDrive.swerveController.addSlewRateLimiters(
				new SlewRateLimiter(Constants.MAX_ACCELERATION),
				new SlewRateLimiter(Constants.MAX_ACCELERATION),
				new SlewRateLimiter(Constants.MAX_ANGULAR_ACCELERATION));

		swerveDrive.setHeadingCorrection(true);
		swerveDrive.setCosineCompensator(true);
		swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
		swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
		Arrays.stream(swerveDrive.getModules()).forEach(m -> m.setAntiJitter(true));

		if (visionEnabled.get()) {
			setupPhotonVision();
			swerveDrive.stopOdometryThread();
		}
		setupPathPlanner();
		setupAutopilot();
	}

	public SwerveSubsystem(SwerveDriveConfiguration driveCfg,
			SwerveControllerConfiguration controllerCfg) {
		swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED,
				new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
	}

	public void setupPhotonVision() {
		vision = new Vision(swerveDrive::getPose, swerveDrive.field);
	}

	@Override
	public void periodic() {
		swerveDrive.updateOdometry();
		if (visionEnabled.get() && vision != null) {
			vision.updatePoseEstimation(swerveDrive);
		}

		if (vision != null) {
			vision.updateVisionField();
		}

		if (autopilotController != null) {
			DogLog.log("Autopilot/Acceleration", autopilotController.getAcceleration());
			DogLog.log("Autopilot/Jerk", autopilotController.getJerk());
		}

		DogLog.log("currentPose", swerveDrive.getPose());

		FieldZones.Zone currentZone = FieldZones.getZone(getPose());
		DogLog.log("Field/Zone", currentZone.name());
		DogLog.log("Field/DistanceToZoneBoundary",
				FieldZones.getDistanceToNearestZoneBoundary(getPose()));
	}

	/** Aim at a target pose while allowing translation control (bang-bang). */
	public Command aimAt(DoubleSupplier translateX, DoubleSupplier translateY, Pose2d target) {
		return run(() -> {
			Pose2d currentPose = getPose();

			double desiredAngle = Math.atan2(
					target.getY() - currentPose.getY(),
					target.getX() - currentPose.getX());

			double error = currentPose.getRotation().getRadians() - desiredAngle;
			error = Math.atan2(Math.sin(error), Math.cos(error));

			double omega = 0.0;
			if (Math.abs(error) > AIM_TOLERANCE) {
				double speed = getAimSpeed(Math.abs(error));
				omega = error > 0 ? -speed : speed;
			}

			ChassisSpeeds speeds = SwerveInputStream.of(getSwerveDrive(),
					() -> -translateY.getAsDouble(), () -> -translateX.getAsDouble()).get();
			speeds.omegaRadiansPerSecond = omega;
			swerveDrive.driveFieldOrientedAndRobotOriented(speeds, new ChassisSpeeds());

			DogLog.log("Aim/Error", error, Radians);
			DogLog.log("Aim/DesiredAngle", desiredAngle, Radians);
			DogLog.log("Aim/CurrentAngle", currentPose.getRotation().getRadians(), Radians);
			DogLog.log("Aim/Omega", omega);
		});
	}

	private double getAimSpeed(double absError) {
		if (absError > Math.PI / 2)
			return AIM_SPEED_FAST;
		if (absError > Math.PI / 8)
			return AIM_SPEED_MID;
		if (absError > Math.PI / 12)
			return AIM_SPEED_SLOW;
		return AIM_SPEED_LOWEST;
	}

	@Override
	public void simulationPeriodic() {
	}

	public void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = true;
			AutoBuilder.configure(
					this::getPose,
					this::resetOdometry,
					this::getRobotVelocity,
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							swerveDrive.drive(speedsRobotRelative,
									swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					},
					new PPHolonomicDriveController(
							new PIDConstants(5.0, 0.0, 0.0),
							new PIDConstants(5.0, 0.0, 0.0)),
					config,
					() -> {
						var alliance = DriverStation.getAlliance();
						return alliance.filter(value -> value == Alliance.Red).isPresent();
					}, this);
		} catch (Exception e) {
			RobotLog.error("Auto/PathPlannerConfig", "PathPlanner config failed",
					"AutoBuilder/RobotConfig setup failed; autos may be unavailable", e);
			RobotLog.setErrorAlert("Auto/Unavailable", "Autos unavailable (PathPlanner config failed)",
					true);
		}

		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
	}

	public void setupAutopilot() {
		autopilotController = new AutopilotController();
	}

	public Command aimAtTarget(Cameras camera) {
		return run(() -> {
			Optional<PhotonPipelineResult> resultO = camera.getBestResult();
			if (resultO.isPresent()) {
				var result = resultO.get();
				if (result.hasTargets()) {
					drive(getTargetSpeeds(0, 0, Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
				}
			}
		});
	}

	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	public Command driveToPose(Supplier<Pose2d> pose) {
		return defer(() -> {
			PathConstraints constraints = new PathConstraints(1, 1,
					swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
			return AutoBuilder.pathfindToPose(pose.get(), constraints,
					edu.wpi.first.units.Units.MetersPerSecond.of(0));
		});
	}

	public Command driveToPose(Supplier<Pose2d> pose, double velocity, double acceleration) {
		return defer(() -> {
			PathConstraints constraints = new PathConstraints(velocity, acceleration,
					swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
			return AutoBuilder.pathfindToPose(pose.get(), constraints,
					edu.wpi.first.units.Units.MetersPerSecond.of(0));
		});
	}

	private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
			throws IOException, ParseException {
		SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
				RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
		AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
				new SwerveSetpoint(swerveDrive.getRobotVelocity(),
						swerveDrive.getStates(), DriveFeedforwards.zeros(swerveDrive.getModules().length)));
		AtomicReference<Double> previousTime = new AtomicReference<>();

		return startRun(() -> previousTime.set(Timer.getFPGATimestamp()), () -> {
			double newTime = Timer.getFPGATimestamp();
			SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
					robotRelativeChassisSpeed.get(), newTime - previousTime.get());
			swerveDrive.drive(newSetpoint.robotRelativeSpeeds(), newSetpoint.moduleStates(),
					newSetpoint.feedforwards().linearForces());
			prevSetpoint.set(newSetpoint);
			previousTime.set(newTime);
		});
	}

	public Command driveWithSetpointGeneratorFieldRelative(
			Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
		try {
			return driveWithSetpointGenerator(
					() -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading()));
		} catch (Exception e) {
			RobotLog.error("Swerve/SetpointGenerator", "Setpoint generator failed",
					"Failed to create setpoint generator command", e);
		}
		return Commands.none();
	}

	public Command sysIdDriveMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(SwerveDriveTest.setDriveSysIdRoutine(
				new Config(null, Voltage.ofBaseUnits(9, Volts), null, null), this, swerveDrive, 9, false),
				3.0, 5.0, 2.0);
	}

	public Command sysIdAngleMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
	}

	public Command centerModulesCommand() {
		return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
	}

	public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
		return new Command() {
			private Translation2d startTranslation;

			@Override
			public void initialize() {
				startTranslation = swerveDrive.getPose().getTranslation();
			}

			@Override
			public void execute() {
				driveFieldOriented(new ChassisSpeeds(speedInMetersPerSecond, 0, 0));
			}

			@Override
			public boolean isFinished() {
				return (swerveDrive.getPose().getTranslation()
						.getDistance(startTranslation) > distanceInMeters);
			}
		};
	}

	public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
		swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
	}

	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return run(() -> {
			swerveDrive.drive(
					SwerveMath.scaleTranslation(new Translation2d(
							translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
							translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
					Math.pow(angularRotationX.getAsDouble(), 3)
							* swerveDrive.getMaximumChassisAngularVelocity(),
					true, false);
		});
	}

	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier headingX, DoubleSupplier headingY) {
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
					new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
					scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(),
					swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
		});
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(translation, rotation, fieldRelative, false);
	}

	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
	}

	public Command robotDriveCommand(SwerveInputStream velocity, BooleanSupplier robotRelative) {
		return run(() -> {
			Optional<Alliance> ally = DriverStation.getAlliance();

			if (ally.isPresent() && !ally.equals(prevAlliance)) {
				prevAlliance = ally;
				if (ally.get() == Alliance.Red) {
					velocity.aim(FieldZones.HUB_POSE_RED);
					DogLog.log("misc/team", "RED");
				}
				if (ally.get() == Alliance.Blue) {
					velocity.aim(FieldZones.HUB_POSE_BLUE);
					DogLog.log("misc/team", "BLUE");
				}
			}
			ChassisSpeeds speeds = velocity.get();
			DogLog.log("Swerve/Input/AngularVelocity", speeds.omegaRadiansPerSecond);
			DogLog.log("Swerve/Input/XVelocity", speeds.vxMetersPerSecond);
			DogLog.log("Swerve/Input/YVelocity", speeds.vyMetersPerSecond);

			if (robotRelative.getAsBoolean()) {
				swerveDrive.driveFieldOrientedAndRobotOriented(new ChassisSpeeds(0, 0, 0), speeds);
			} else {
				swerveDrive.driveFieldOriented(speeds);
			}
		});
	}

	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}

	@NotLogged
	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	@NotLogged
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	private boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
	}

	public void zeroGyroWithAlliance() {
		if (isRedAlliance()) {
			zeroGyro();
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
		} else {
			zeroGyro();
		}
	}

	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX,
			double headingY) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
				headingX, headingY, getHeading().getRadians(), Constants.MAX_SPEED);
	}

	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
				angle.getRadians(), getHeading().getRadians(), Constants.MAX_SPEED);
	}

	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}

	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	public void lock() {
		swerveDrive.lockPose();
	}

	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	public void addFakeVisionReading() {
		swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)),
				Timer.getFPGATimestamp());
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	/** Drive to a pose using Autopilot (no obstacle avoidance). */
	public Command driveToPoseAutopilot(Supplier<Pose2d> targetPose) {
		return run(() -> {
			ChassisSpeeds speeds = autopilotController.calculate(getPose(), getRobotVelocity(), targetPose.get());
			driveFieldOriented(speeds);
		});
	}

	/** Drive to a pose using Autopilot with optional entry angle for curved approach. */
	public Command driveToPoseAutopilot(Supplier<Pose2d> targetPose, boolean respectEntryAngle) {
		return run(() -> {
			ChassisSpeeds speeds = autopilotController.calculate(getPose(), getRobotVelocity(),
					targetPose.get(), respectEntryAngle);
			driveFieldOriented(speeds);
		});
	}

	/** Drive to a pose using Autopilot, finishing when within tolerance. */
	public Command driveToPoseAutopilotUntilFinished(Supplier<Pose2d> targetPose,
			double translationTolerance, double rotationTolerance) {
		return run(() -> {
			ChassisSpeeds speeds = autopilotController.calculate(getPose(), getRobotVelocity(), targetPose.get());
			driveFieldOriented(speeds);
		}).until(() -> autopilotController.atTarget(getPose(), targetPose.get(), translationTolerance,
				rotationTolerance));
	}

	public Command driveToPoseAutopilot(Pose2d targetPose) {
		return driveToPoseAutopilot(() -> targetPose);
	}

	public AutopilotController getAutopilotController() {
		return autopilotController;
	}
}
