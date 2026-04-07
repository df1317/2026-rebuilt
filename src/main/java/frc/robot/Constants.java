package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.util.DevMode;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public final class Constants {

	// Subsystem enable flags
	public static final boolean ENABLE_SWERVE = true;
	public static final boolean ENABLE_CLIMBER = false;
	public static final boolean ENABLE_INTAKE = true;
	public static final boolean ENABLE_SHOOTER = true;
	public static final boolean ENABLE_HOPPER = true;

	public static final double ROBOT_MASS = Units.lbsToKilograms(120.4); // 60lbs * kg per pound

	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(540.0);

	// HIGH in dev mode, LOW at competition to reduce NT traffic
	public static final TelemetryVerbosity SwerveTelemetryVerbosity = DevMode.isEnabled()
			? TelemetryVerbosity.HIGH
			: TelemetryVerbosity.LOW;

	public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

	public static final class AutoConstants {
		public static final double SPEED_SCALE = 0.65;
	}

	public static final class DrivebaseConstants {
		public static final double WHEEL_LOCK_TIME = 10; // seconds
		public static final double JOYSTICK_DEADBAND = 0.1;
		public static final double DRIVE_KS = 0.024309;
		public static final double DRIVE_KV = 2.7435;
		public static final double DRIVE_KA = 2.0788;
		public static final double TRANSLATION_SCALE = 0.8;

		// Robot half-dimensions for Repulsor obstacle avoidance (meters)
		// 27 inches square (frame only, excludes bumpers so repulsion doesn't kick in too early)
		public static final double ROBOT_HALF_LENGTH = Units.inchesToMeters(27) / 2.0;
		public static final double ROBOT_HALF_WIDTH = Units.inchesToMeters(27) / 2.0;

		// Intake extension for repulsor footprint
		// Angle in degrees relative to robot frame (-90 = right side)
		public static final double INTAKE_ANGLE_DEG = -90.0;
		public static final double INTAKE_LENGTH_METERS = Units.inchesToMeters(4);
	}

	public static final class VisionConstants {
		public static final double MAX_POSE_JUMP_METERS = 1.0;
		public static final double POSE_AMBIGUITY_THRESHOLD = 0.2;
		public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
		public static final double HIGH_LATENCY_THRESHOLD_MS = 100.0;

		public static final class CameraStdDevs {
			public static final double[] SINGLE_TAG = { 4.0, 4.0, 6.0 };
			public static final double[] MULTI_TAG = { 0.5, 0.5, 4.0 };
		}
	}
}
