package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.DevMode;
import swervelib.math.Matter;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.*;

/**
 * ---------- Constants --- The Constants class provides a convenient place for teams to hold robot-wide numerical or
 * boolean constants. This class should not be used for any other purpose. All constants should be declared globally
 * (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed to
 * reduce verbosity. ---
 */
public final class Constants {

	public static final double ROBOT_MASS = 60 * 0.453592; // 60lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);

	/**
	 * Expected control loop time including robot periodic (20ms) and motor controller velocity control latency.
	 *
	 * <p>
	 * <b>Note:</b> The 110ms "SparkMAX velocity lag" comment appears to be outdated.
	 * Typical SparkMAX closed-loop velocity control adds 10-20ms of latency, not 110ms.
	 *
	 * <p>
	 * If you're experiencing 130ms total loop times, investigate:
	 * <ul>
	 * <li>Telemetry verbosity (now auto-switches to LOW at competition)</li>
	 * <li>Vision processing time</li>
	 * <li>CAN bus utilization (check DriverStation diagnostics)</li>
	 * </ul>
	 *
	 * <p>
	 * Typical values:
	 * <ul>
	 * <li>20ms - Robot periodic cycle time</li>
	 * <li>10-20ms - Motor controller latency</li>
	 * <li>Total: 30-40ms expected, not 130ms</li>
	 * </ul>
	 *
	 * @deprecated This constant may need revision based on actual measured loop times
	 */
	@Deprecated
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(120.0);

	/**
	 * Swerve telemetry verbosity - automatically switches based on dev/comp mode. HIGH for development/testing, LOW for
	 * competition to reduce NT traffic.
	 */
	public static final TelemetryVerbosity SwerveTelemetryVerbosity = DevMode.isEnabled()
			? TelemetryVerbosity.HIGH
			: TelemetryVerbosity.LOW;

	public static final class DrivebaseConstants {

		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds

		// Feedforward characterization values from SysId
		public static final double DRIVE_KS = 0.024309;
		public static final double DRIVE_KV = 2.7435;
		public static final double DRIVE_KA = 2.0788;

		/**
		 * Translation scaling factor for driver input (0.0 to 1.0).
		 *
		 * <p>
		 * Limits maximum translational speed to a percentage of the robot's theoretical max. This provides finer control
		 * for drivers during teleop without sacrificing full speed capability in autonomous.
		 *
		 * <p>
		 * Common values:
		 * <ul>
		 * <li>0.8 - Good balance of speed and control (current)</li>
		 * <li>0.6-0.7 - More precise control for intricate maneuvering</li>
		 * <li>1.0 - Full speed (requires very experienced drivers)</li>
		 * </ul>
		 *
		 * <p>
		 * Note: This does NOT affect rotation speed, which uses cubic scaling
		 * (see {@code Math.pow(input, 3)}) for smooth control.
		 */
		public static final double TRANSLATION_SCALE = 0.8;
	}

	public static final class VisionConstants {

		// Outlier rejection: maximum pose jump between updates
		public static final double MAX_POSE_JUMP_METERS = 1.0;

		// Single tag filtering thresholds
		public static final double POSE_AMBIGUITY_THRESHOLD = 0.2;
		public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;

		// High latency threshold in milliseconds
		public static final double HIGH_LATENCY_THRESHOLD_MS = 100.0;

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

	public static class IntakeConstants {

		// Motor CAN IDs
		public static final int PIVOT_MOTOR_ID = 20;
		public static final int ROLLER_MOTOR_ID = 21;

		// Motor configuration
		public static final boolean PIVOT_INVERTED = false;
		public static final boolean ROLLER_INVERTED = false;
		public static final int PIVOT_CURRENT_LIMIT = 15;
		public static final int ROLLER_CURRENT_LIMIT = 35;

		// Pivot geometry
		public static final Angle PIVOT_EXTENDED_ANGLE = Degrees.of(90);
		public static final Angle PIVOT_RETRACTED_ANGLE = Degrees.of(0);
		public static final Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(3);
		public static final double PIVOT_GEAR_RATIO = 25.0; // motor rotations per pivot rotation
		public static final Distance PIVOT_ARM_LENGTH = Inches.of(12);

		// Pivot PID constants
		public static final double PIVOT_KP = 0.1;
		public static final double PIVOT_KI = 0.0;
		public static final double PIVOT_KD = 0.0;

		// Roller velocity control
		public static final AngularVelocity ROLLER_INTAKE_VELOCITY = RPM.of(2000);
		public static final AngularVelocity ROLLER_EJECT_VELOCITY = RPM.of(-1500);
		public static final AngularVelocity ROLLER_VELOCITY_TOLERANCE = RPM.of(100);

		// Roller PID constants
		public static final double ROLLER_KP = 0.0002;
		public static final double ROLLER_KI = 0.0;
		public static final double ROLLER_KD = 0.0;
		public static final double ROLLER_KV = 0.000175;

		// Debounce time for state checks
		public static final double AT_POSITION_DEBOUNCE_TIME = 0.1;
	}

	public static class ShooterConstants {

		// Motor CAN ID
		public static final int MOTOR_ID = 30;

		// Motor configuration
		public static final boolean INVERTED = false;
		public static final int CURRENT_LIMIT = 40;

		// PID constants (tune these for your flywheel)
		public static final double KP = 0.0002;
		public static final double KI = 0.0;
		public static final double KD = 0.0;
		public static final double KV = 0.000175;

		// Velocity control
		public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
		public static final double AT_SPEED_DEBOUNCE_TIME = 0.1; // seconds

		// SysId configuration
		public static final Voltage SYSID_STEP_VOLTAGE = Volts.of(7);
	}

	public static class ClimberConstants {

		// Motor CAN IDs
		public static final int MOTOR_LEFT_ID = 24;
		public static final int MOTOR_RIGHT_ID = 25;

		// Motor configuration
		public static final boolean INVERTED = false;
		public static final int CURRENT_LIMIT = 40;

		// Mechanism geometry
		public static final Distance MAX_HEIGHT = Meters.of(1.23);
		public static final Distance MIN_HEIGHT = Meters.of(0.0);
		public static final double ROTATIONS_PER_METER = 42.4;
		public static final Distance POSITION_TOLERANCE = Centimeters.of(2);

		// Motion profile constraints
		public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1.0);
		public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.0);

		// PID constants (tune with SysId)
		public static final double KP = 0.00065;
		public static final double KI = 0.0;
		public static final double KD = 0.0;

		// Feedforward constants (tune with SysId)
		public static final double KS = 0.37;
		public static final double KG = 0.49;
		public static final double KV = 4.7;
	}

	public static class HopperConstants {

		// Motor configuration
		public static final int MOTOR_ID = 30;
		public static final int CURRENT_LIMIT = 35;
		public static final boolean INVERTED = false;

		// Feed speed (duty cycle, -1.0 to 1.0)
		public static final double FEED_SPEED = 0.5;
		public static final double REVERSE_SPEED = -0.3;
	}
}
