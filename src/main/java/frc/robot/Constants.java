package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * ---------- Constants --- The Constants class provides a convenient place for teams to hold robot-wide numerical or
 * boolean constants. This class should not be used for any other purpose. All constants should be declared globally
 * (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity. ---
 */
public final class Constants {

	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(120.0);

	public static final class DrivebaseConstants {

		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {

		// Joystick Deadband
		public static final double DEADBAND = 0.1;
	}
}
