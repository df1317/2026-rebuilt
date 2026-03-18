package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.DevMode;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.*;

public final class Constants {

	// Subsystem enable flags
	public static final boolean ENABLE_SWERVE = true;
	public static final boolean ENABLE_CLIMBER = true;
	public static final boolean ENABLE_INTAKE = true;
	public static final boolean ENABLE_SHOOTER = true;
	public static final boolean ENABLE_HOPPER = true;

	public static final double ROBOT_MASS = Units.lbsToKilograms(120.4); // 60lbs * kg per pound

	@Deprecated
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(540.0);

	// HIGH in dev mode, LOW at competition to reduce NT traffic
	public static final TelemetryVerbosity SwerveTelemetryVerbosity = DevMode.isEnabled() ? TelemetryVerbosity.HIGH
			: TelemetryVerbosity.LOW;

	public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

	public static final class AutoConstants {
		public static final double SPEED_SCALE = 0.05;
	}

	public static final class DrivebaseConstants {
		public static final double WHEEL_LOCK_TIME = 10; // seconds
		public static final double DRIVE_KS = 0.024309;
		public static final double DRIVE_KV = 2.7435;
		public static final double DRIVE_KA = 2.0788;
		public static final double TRANSLATION_SCALE = 0.8;

		// Robot half-dimensions for Repulsor obstacle avoidance (meters)
		public static final double ROBOT_HALF_LENGTH = 0.4;
		public static final double ROBOT_HALF_WIDTH = 0.4;
	}

	public static final class VisionConstants {
		public static final double MAX_POSE_JUMP_METERS = 1.0;
		public static final double POSE_AMBIGUITY_THRESHOLD = 0.2;
		public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
		public static final double HIGH_LATENCY_THRESHOLD_MS = 100.0;

		public static final class CameraStdDevs {
			public static final double[] SINGLE_TAG = { 4.0, 4.0, 8.0 };
			public static final double[] MULTI_TAG = { 0.5, 0.5, 1.0 };
		}
	}

	public static class OperatorConstants {
		public static final double DEADBAND = 0.1;
	}

	public static class IntakeConstants {
		public static final int PIVOT_MOTOR_ID = 25;
		public static final int ROLLER_MOTOR_ID = 30;

		public static final boolean PIVOT_INVERTED = false;
		public static final boolean ROLLER_INVERTED = true;
		public static final int PIVOT_CURRENT_LIMIT = 35;
		public static final int ROLLER_CURRENT_LIMIT = 40;

		public static final Angle PIVOT_EXTENDED_ANGLE = Degrees.of(0);
		public static final Angle PIVOT_RETRACTED_ANGLE = Degrees.of(90); // TODO: verify physical travel
		public static final Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(3);
		public static final double PIVOT_GEAR_RATIO = (48.0 * 22.0) / 14.0;
		public static final Distance PIVOT_ARM_LENGTH = Inches.of(12);

		public static final double PIVOT_HOMING_OFFSET_DEG = -20; // degrees past current reading toward extended (down)
		// hard stop

		public static final double PIVOT_KP = 0.05;
		public static final double PIVOT_KI = 0.0;
		public static final double PIVOT_KD = 0.0;

		// Motion profile constraints for the pivot (degrees/s and degrees/s²)
		public static final double PIVOT_MAX_VELOCITY_DEG_PER_S = 120.0;
		public static final double PIVOT_MAX_ACCEL_DEG_PER_S2 = 240.0;
		public static final double PIVOT_EXTEND_MAX_VELOCITY_DEG_PER_S = 60.0;
		public static final double PIVOT_EXTEND_MAX_ACCEL_DEG_PER_S2 = 60.0;

		public static final AngularVelocity ROLLER_INTAKE_VELOCITY = RPM.of(2800);
		public static final double ROLLER_SPEED_SCALE_MAX_RPM = 4000;
		public static final double ROLLER_SPEED_SCALE_MAX_ROBOT_MPS = 3.0;
		public static final AngularVelocity ROLLER_EJECT_VELOCITY = RPM.of(-1500);
		public static final AngularVelocity ROLLER_VELOCITY_TOLERANCE = RPM.of(100);

		public static final double ROLLER_KP = 2E-4;
		public static final double ROLLER_KI = 1.3E-4;
		public static final double ROLLER_KD = 0.0;
		public static final double ROLLER_KV = 1.5E-4;
		public static final double ROLLER_I_ZONE = 1E-3;

		public static final double AT_POSITION_DEBOUNCE_TIME = 0.1;
	}

	public static class ShooterConstants {
		public static final int MOTOR_ID = 40;
		// check above ID
		public static final int FEEDER_ID = 28;
		public static final int HOOD_ID = 24;

		public static final boolean INVERTED = true;
		public static final boolean FEEDER_INVERTED = true;
		public static final boolean HOOD_INVERTED = true;
		public static final int CURRENT_LIMIT = 25;
		public static final int FEEDER_CURRENT_LIMIT = 35;
		public static final int HOOD_CURRENT_LIMIT = 20;

		public static final double HOOD_GEAR_RATIO = 24.0;

		public static final Angle HOOD_TOLERANCE = Degrees.of(3);

		public static final double SHOOTER_KP = 0.17;
		public static final double SHOOTER_KI = 0.001;
		public static final double SHOOTER_KD = 0.0;
		public static final double SHOOTER_KV = 0.115;
		public static final double SHOOTER_KG = 0.0;
		public static final double SHOOTER_KS = 0.0;

		public static final double FEEDER_KP = 0.0002;
		public static final double FEEDER_KI = 0.0;
		public static final double FEEDER_KD = 0.0;
		public static final double FEEDER_KV = 0.000175;

		public static final double FEEDER_RPM = 3000;

		public static final double HOOD_KP = 0.013;
		public static final double HOOD_KI = 0.0;
		public static final double HOOD_KD = 0.000;

		public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
		public static final double AT_SPEED_DEBOUNCE_TIME = 0.1;

		public static final Voltage SYSID_STEP_VOLTAGE = Volts.of(7);

		public static final double HOOD_STALL_RPM = 2.0;
		public static final double BALL_SPEED_LOW_M_S = 4.97; // ball speed at min RPM (2555)
		public static final double BALL_SPEED_HIGH_M_S = 6.15; // ball speed at max RPM (3250)
		public static final double CURRENT_DEBOUNCE_TIME = 0.1;
		public static final double HOOD_HOMING_VOLTAGE = 2.0;
	}

	public static class ClimberConstants {
		public static final int MOTOR_LEFT_ID = 29;

		public static final boolean INVERTED = true;
		public static final int CURRENT_LIMIT = 20;

		public static final Distance MAX_HEIGHT = Meters.of(3.6);
		public static final Distance MIN_HEIGHT = Meters.of(0.0);
		public static final Distance HANG_HEIGHT = Meters.of(0.5); // TODO: measure
		public static final Distance RELEASE_HEIGHT = Meters.of(0.2); // TODO: measure
		public static final double ROTATIONS_PER_METER = 42.4;
		public static final Distance POSITION_TOLERANCE = Centimeters.of(2);

		public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(0.25);
		public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.5);

		public static final double KP = 0.0; // 0.00065
		public static final double KI = 0.0;
		public static final double KD = 0.0;

		public static final double KS = 0.31; // 0.37
		public static final double KG = 0.0; // 0.49
		public static final double KV = 4.7; // 4.7

		public static final double JOG_SPEED_METERS_PER_SECOND = 0.3;
		public static final double JOG_CURRENT_LIMIT = 1;
		public static final double JOG_SOFT_LIMIT_ROTATIONS = 9999.0;
	}

	public static class HopperConstants {
		public static final int HOPPER_MOTOR_ID = 26;
		public static final int HOPPER_CURRENT_LIMIT = 20;
		public static final boolean INVERTED = true;

		public static final AngularVelocity FEED_SPEED = RPM.of(2000);
		public static final AngularVelocity REVERSE_SPEED = RPM.of(-2000);
		public static final AngularVelocity HOPPER_VELOCITY_TOLERANCE = RPM.of(100);

		public static final double HOPPER_KP = 2E-4;
		public static final double HOPPER_KI = 1E-5;
		public static final double HOPPER_KD = 0.0;
		public static final double HOPPER_KV = 1.8E-4; // usless
		public static final double HOPPER_KS = 0.5; // usless
		public static final double HOPPER_I_ZONE = 1E-3;
		public static final double HOPPER_KF = 1.75E-4;

		public static final double GEAR_RATIO = 24.0;
	}
}
