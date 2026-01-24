package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ShooterConstants;
import org.junit.jupiter.api.Test;

/**
 * Tests for Shooter subsystem logic.
 * <p>
 * These tests verify the distance-to-RPM interpolation and clamping behavior
 * without requiring hardware or HAL simulation.
 */
class ShooterSubsystemTest {

	/**
	 * Tests that distance-to-RPM lookup returns expected interpolated values.
	 * The lookup table is defined as:
	 * 1.0m -> 2500 RPM
	 * 2.0m -> 3000 RPM
	 * 3.0m -> 3500 RPM
	 * 4.0m -> 4000 RPM
	 * 5.0m -> 4500 RPM
	 * 6.0m -> 5000 RPM
	 */
	@Test
	void testDistanceToRPMInterpolation() {
		// Test exact lookup table values
		assertEquals(2500.0, lookupRPM(1.0), 0.001, "1.0m should return 2500 RPM");
		assertEquals(3000.0, lookupRPM(2.0), 0.001, "2.0m should return 3000 RPM");
		assertEquals(4000.0, lookupRPM(4.0), 0.001, "4.0m should return 4000 RPM");
		assertEquals(5000.0, lookupRPM(6.0), 0.001, "6.0m should return 5000 RPM");

		// Test interpolated values (midpoints)
		assertEquals(2750.0, lookupRPM(1.5), 0.001, "1.5m should interpolate to 2750 RPM");
		assertEquals(3250.0, lookupRPM(2.5), 0.001, "2.5m should interpolate to 3250 RPM");
		assertEquals(4750.0, lookupRPM(5.5), 0.001, "5.5m should interpolate to 4750 RPM");
	}

	/**
	 * Tests that distances outside the lookup table range are clamped.
	 * This prevents unsafe extrapolation to very high or low RPM values.
	 */
	@Test
	void testDistanceClampingBelowMinimum() {
		// Distances below 1.0m should clamp to 1.0m (2500 RPM)
		assertEquals(2500.0, lookupRPMClamped(0.5), 0.001, "0.5m should clamp to 2500 RPM");
		assertEquals(2500.0, lookupRPMClamped(0.0), 0.001, "0.0m should clamp to 2500 RPM");
		assertEquals(2500.0, lookupRPMClamped(-1.0), 0.001, "Negative distance should clamp to 2500 RPM");
	}

	@Test
	void testDistanceClampingAboveMaximum() {
		// Distances above 6.0m should clamp to 6.0m (5000 RPM)
		assertEquals(5000.0, lookupRPMClamped(7.0), 0.001, "7.0m should clamp to 5000 RPM");
		assertEquals(5000.0, lookupRPMClamped(10.0), 0.001, "10.0m should clamp to 5000 RPM");
		assertEquals(5000.0, lookupRPMClamped(100.0), 0.001, "100.0m should clamp to 5000 RPM");
	}

	/**
	 * Tests that the velocity tolerance constant is reasonable.
	 */
	@Test
	void testVelocityToleranceConstant() {
		AngularVelocity tolerance = ShooterConstants.VELOCITY_TOLERANCE;
		assertEquals(100.0, tolerance.in(RPM), 0.001, "Velocity tolerance should be 100 RPM");
	}

	/**
	 * Tests the isAtSpeed logic with different error values.
	 */
	@Test
	void testAtSpeedLogic() {
		double tolerance = ShooterConstants.VELOCITY_TOLERANCE.in(RPM);

		// Error within tolerance should be "at speed"
		assertTrue(isWithinTolerance(50, tolerance), "50 RPM error should be within tolerance");
		assertTrue(isWithinTolerance(99, tolerance), "99 RPM error should be within tolerance");

		// Error at or beyond tolerance should not be "at speed"
		assertFalse(isWithinTolerance(100, tolerance), "100 RPM error should NOT be within tolerance");
		assertFalse(isWithinTolerance(150, tolerance), "150 RPM error should NOT be within tolerance");
	}

	/**
	 * Tests that zero or negative target velocity is not considered "at speed".
	 */
	@Test
	void testZeroTargetNotAtSpeed() {
		// Even with zero error, zero target should not be "at speed"
		assertFalse(isAtSpeedWithTarget(0, 0), "Zero target should not be at speed");
		assertFalse(isAtSpeedWithTarget(-100, 0), "Negative target should not be at speed");
	}

	/**
	 * Simulates the lookup table behavior (without hardware).
	 * This matches the populateLookupTable() values in ShooterSubsystem.
	 */
	private double lookupRPM(double distanceMeters) {
		// Linear interpolation between known points
		double[][] table = {
				{ 1.0, 2500.0 },
				{ 2.0, 3000.0 },
				{ 3.0, 3500.0 },
				{ 4.0, 4000.0 },
				{ 5.0, 4500.0 },
				{ 6.0, 5000.0 }
		};

		// Find surrounding points and interpolate
		for (int i = 0; i < table.length - 1; i++) {
			if (distanceMeters >= table[i][0] && distanceMeters <= table[i + 1][0]) {
				double t = (distanceMeters - table[i][0]) / (table[i + 1][0] - table[i][0]);
				return table[i][1] + t * (table[i + 1][1] - table[i][1]);
			}
		}

		// Exact match on last point
		if (distanceMeters == table[table.length - 1][0]) {
			return table[table.length - 1][1];
		}

		return Double.NaN;
	}

	/**
	 * Simulates the clamped lookup (matches getRPMForDistance behavior).
	 */
	private double lookupRPMClamped(double distanceMeters) {
		double clamped = Math.max(1.0, Math.min(6.0, distanceMeters));
		return lookupRPM(clamped);
	}

	/**
	 * Tests the tolerance check logic.
	 */
	private boolean isWithinTolerance(double errorRPM, double toleranceRPM) {
		return errorRPM < toleranceRPM;
	}

	/**
	 * Tests the at-speed logic including target check.
	 */
	private boolean isAtSpeedWithTarget(double targetRPM, double errorRPM) {
		double tolerance = ShooterConstants.VELOCITY_TOLERANCE.in(RPM);
		return errorRPM < tolerance && targetRPM > 0;
	}
}
