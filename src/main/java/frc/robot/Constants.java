package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.DevMode;
import swervelib.math.Matter;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public final class Constants {

	// Subsystem enable flags
	public static final boolean ENABLE_SWERVE = true;
	public static final boolean ENABLE_CLIMBER = false;
	public static final boolean ENABLE_INTAKE = true;
	public static final boolean ENABLE_SHOOTER = true;
	public static final boolean ENABLE_HOPPER = true;

	public static final double ROBOT_MASS = 60 * 0.453592; // 60lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);

	@Deprecated
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(540.0);

	// HIGH in dev mode, LOW at competition to reduce NT traffic
	public static final TelemetryVerbosity SwerveTelemetryVerbosity = DevMode.isEnabled() ? TelemetryVerbosity.HIGH
			: TelemetryVerbosity.LOW;

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
		public static final boolean ROLLER_INVERTED = false;
		public static final int PIVOT_CURRENT_LIMIT = 25;
		public static final int ROLLER_CURRENT_LIMIT = 40;

		public static final Angle PIVOT_EXTENDED_ANGLE = Degrees.of(90);
		public static final Angle PIVOT_RETRACTED_ANGLE = Degrees.of(0);
		public static final Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(3);
		public static final double PIVOT_GEAR_RATIO = (48.0 * 22.0) / 14.0;
		public static final Distance PIVOT_ARM_LENGTH = Inches.of(12);

		public static final double PIVOT_KP = 0.01;
		public static final double PIVOT_KI = 0.0;
		public static final double PIVOT_KD = 0.0;

		public static final AngularVelocity ROLLER_INTAKE_VELOCITY = RPM.of(2000);
		public static final AngularVelocity ROLLER_EJECT_VELOCITY = RPM.of(-1500);
		public static final AngularVelocity ROLLER_VELOCITY_TOLERANCE = RPM.of(100);

		public static final double ROLLER_KP = 2E-4;
		public static final double ROLLER_KI = 1E-5;
		public static final double ROLLER_KD = 0.0;
		public static final double ROLLER_KV = 1.8E-4;
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
		public static final int FEEDER_CURRENT_LIMIT = CURRENT_LIMIT;
		public static final int HOOD_CURRENT_LIMIT = 20;

		public static final double HOOD_GEAR_RATIO = 24.0;

		public static final Angle MAX_HOOD = Degrees.of(180.0);
		public static final Angle MIN_HOOD = Degrees.of(0.0);

		public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(1440);
		public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(1440);

		public static final Angle HOOD_TOLERANCE = Degrees.of(3);

		public static final double SHOOTER_KP = 0.05;
		public static final double SHOOTER_KI = 0.0;
		public static final double SHOOTER_KD = 0.0;
		public static final double SHOOTER_KV = 0.115;
		public static final double SHOOTER_KG = 0.0;
		public static final double SHOOTER_KS = 0.0;

		public static final double FEEDER_KP = 0.0002;
		public static final double FEEDER_KI = 0.0;
		public static final double FEEDER_KD = 0.0;
		public static final double FEEDER_KV = 0.000175;

		public static final double HOOD_KP = 0.0012;
		public static final double HOOD_KI = 0.0;
		public static final double HOOD_KD = 0.000;

		public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
		public static final double AT_SPEED_DEBOUNCE_TIME = 0.1;

		public static final Voltage SYSID_STEP_VOLTAGE = Volts.of(7);

		public static final double AT_POSITION_DEBOUNCE_TIME = 0.1;

		public static final double HOOD_STALL_RPM = 2.0;
		public static final double CURRENT_DEBOUNCE_TIME = 0.1;
		public static final double HOOD_HOMING_VOLTAGE = 2.0;
	}

	public static class ClimberConstants {
		public static final int MOTOR_LEFT_ID = 29;

		public static final boolean INVERTED = false;
		public static final int CURRENT_LIMIT = 40;

		public static final Distance MAX_HEIGHT = Meters.of(1.23);
		public static final Distance MIN_HEIGHT = Meters.of(0.0);
		public static final double ROTATIONS_PER_METER = 42.4;
		public static final Distance POSITION_TOLERANCE = Centimeters.of(2);

		public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1.0);
		public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.0);

		public static final double KP = 0.00065;
		public static final double KI = 0.0;
		public static final double KD = 0.0;

		public static final double KS = 0.37;
		public static final double KG = 0.49;
		public static final double KV = 4.7;
	}

	public static class HopperConstants {
		public static final int HOPPER_MOTOR_ID = 26;
		public static final int HOPPER_CURRENT_LIMIT = 20;
		public static final boolean INVERTED = true;

		public static final AngularVelocity FEED_SPEED = RPM.of(0.5);
		public static final AngularVelocity REVERSE_SPEED = RPM.of(-0.3);
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
