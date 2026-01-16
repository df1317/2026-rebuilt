package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * ---------- Constants --- The Constants class provides a convenient place for
 * teams to hold robot-wide numerical or
 * boolean constants. This class should not be used for any other purpose. All
 * constants should be declared globally
 * (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity. ---
 */
public final class Constants {

	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(120.0);

	public static final class DrivebaseConstants {

		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds

		// Feedforward characterization values from SysId
		public static final double DRIVE_KS = 0.024309;
		public static final double DRIVE_KV = 2.7435;
		public static final double DRIVE_KA = 2.0788;

		// Translation scaling factor for driver input
		public static final double TRANSLATION_SCALE = 0.8;
	}

	public static final class VisionConstants {

		// Outlier rejection: maximum pose jump between updates
		public static final double MAX_POSE_JUMP_METERS = 1.0;

		// Single tag filtering thresholds
		public static final double POSE_AMBIGUITY_THRESHOLD = 0.2;
		public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;

		// Camera standard deviations for pose estimation (X, Y, Theta in radians)
		public static final class CameraStdDevs {
			// Single tag: Higher uncertainty
			public static final double[] SINGLE_TAG = { 4.0, 4.0, 8.0 };
			// Multi tag: Lower uncertainty
			public static final double[] MULTI_TAG = { 0.5, 0.5, 1.0 };
		}
	}

	public static class OperatorConstants {

		// Joystick Deadband
		public static final double DEADBAND = 0.1;
	}
}
