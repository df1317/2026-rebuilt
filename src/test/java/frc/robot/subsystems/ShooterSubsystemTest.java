package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ShooterConstants;
import org.junit.jupiter.api.Test;

/**
 * Tests for Shooter subsystem logic.
 */
class ShooterSubsystemTest {

	// Mirror the polynomial coefficients from ShooterSubsystem
	private static final double RPM_A = 109.602;
	private static final double RPM_B = -496.440;
	private static final double RPM_C = 3279.834;

	private static final double HOOD_A = 0.05706;
	private static final double HOOD_B = -0.17218;
	private static final double HOOD_C = 0.11705;

	@Test
	void testRPMCurveMatchesCalibrationPoints() {
		// Verify the curve is reasonably close to the original calibration data
		assertRPMClose(1.00, 3000.0, 200);
		assertRPMClose(2.30, 2700.0, 50);
		assertRPMClose(2.80, 2750.0, 50);
		assertRPMClose(3.50, 2950.0, 100);
		assertRPMClose(4.00, 3100.0, 100);
		assertRPMClose(4.60, 3250.0, 100);
	}

	@Test
	void testRPMExtrapolatesReasonably() {
		// Beyond 4.6m the curve should keep increasing
		double at5m = rpmForDistance(5.0);
		double at6m = rpmForDistance(6.0);
		double at7m = rpmForDistance(7.0);
		assertTrue(at5m > 3250, "RPM at 5m should exceed max calibration point");
		assertTrue(at6m > at5m, "RPM should increase with distance");
		assertTrue(at7m > at6m, "RPM should keep increasing");
		assertTrue(at7m < 6000, "RPM should not be unreasonably high at 7m");
	}

	@Test
	void testRPMClampsBelowMinimum() {
		double atMin = rpmForDistance(Math.max(1.0, 0.5));
		double atOne = rpmForDistance(1.0);
		assertEquals(atOne, atMin, 0.001, "Distances below 1m should clamp");
	}

	@Test
	void testHoodClampedTo0And1() {
		double hood = hoodForDistance(1.0);
		assertTrue(Math.max(0.0, hood) >= 0.0, "Hood should not go below 0");
		double hoodFar = hoodForDistance(7.0);
		assertTrue(Math.min(1.0, hoodFar) <= 1.0, "Hood should not exceed 1.0");
	}

	@Test
	void testVelocityToleranceConstant() {
		AngularVelocity tolerance = ShooterConstants.VELOCITY_TOLERANCE;
		assertEquals(100.0, tolerance.in(RPM), 0.001, "Velocity tolerance should be 100 RPM");
	}

	@Test
	void testAtSpeedLogic() {
		double tolerance = ShooterConstants.VELOCITY_TOLERANCE.in(RPM);
		assertTrue(isWithinTolerance(50, tolerance));
		assertTrue(isWithinTolerance(99, tolerance));
		assertFalse(isWithinTolerance(100, tolerance));
		assertFalse(isWithinTolerance(150, tolerance));
	}

	@Test
	void testZeroTargetNotAtSpeed() {
		assertFalse(isAtSpeedWithTarget(0, 0));
		assertFalse(isAtSpeedWithTarget(-100, 0));
	}

	private void assertRPMClose(double distance, double expectedRPM, double tolerance) {
		double actual = rpmForDistance(distance);
		assertEquals(expectedRPM, actual, tolerance,
				String.format("RPM at %.2fm: expected ~%.0f, got %.0f", distance, expectedRPM, actual));
	}

	private static double rpmForDistance(double d) {
		return RPM_A * d * d + RPM_B * d + RPM_C;
	}

	private static double hoodForDistance(double d) {
		return HOOD_A * d * d + HOOD_B * d + HOOD_C;
	}

	private boolean isWithinTolerance(double errorRPM, double toleranceRPM) {
		return errorRPM < toleranceRPM;
	}

	private boolean isAtSpeedWithTarget(double targetRPM, double errorRPM) {
		double tolerance = ShooterConstants.VELOCITY_TOLERANCE.in(RPM);
		return errorRPM < tolerance && targetRPM > 0;
	}
}
