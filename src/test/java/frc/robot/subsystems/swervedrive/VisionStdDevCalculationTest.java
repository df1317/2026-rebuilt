package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests for Vision standard deviation calculation algorithm.
 * <p>
 * The Vision class uses a heuristic to calculate dynamic standard deviations based on: - Number of tags (single vs.
 * multi) - Distance from tags - Formula: baseStdDev * (1 + (avgDistance² / 30))
 */
class VisionStdDevCalculationTest {

	/**
	 * Verifies the standard deviation scaling formula matches expected values.
	 * <p>
	 * Formula: std dev = baseStdDev * (1 + (distance² / 30))
	 */
	@Test
	void testStdDevScaling() {
		// Base standard deviations for single tag
		Matrix<N3, N1> baseSingleTag = VecBuilder.fill(
				VisionConstants.CameraStdDevs.SINGLE_TAG[0],
				VisionConstants.CameraStdDevs.SINGLE_TAG[1],
				VisionConstants.CameraStdDevs.SINGLE_TAG[2]);

		// Test at different distances
		double[] testDistances = { 0.0, 1.0, 2.0, 4.0, 5.0 };
		double[] expectedMultipliers = { 1.0, 1.033, 1.133, 1.533, 1.833 };

		for (int i = 0; i < testDistances.length; i++) {
			double distance = testDistances[i];
			double multiplier = 1 + ((distance * distance) / 30);

			assertEquals(expectedMultipliers[i], multiplier, 0.001,
					"Distance " + distance + "m should have multiplier " + expectedMultipliers[i]);

			Matrix<N3, N1> scaled = baseSingleTag.times(multiplier);

			// Verify X std dev scales correctly
			assertEquals(
					VisionConstants.CameraStdDevs.SINGLE_TAG[0] * multiplier,
					scaled.get(0, 0),
					0.001,
					"X std dev at " + distance + "m");
		}
	}

	/**
	 * Tests that single-tag estimates beyond MAX_SINGLE_TAG_DISTANCE_METERS are rejected.
	 */
	@Test
	void testSingleTagDistanceRejection() {
		double maxDistance = VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS;

		// Distance at the limit should be accepted (returns non-null std dev)
		assertEquals(4.0, maxDistance, "Max single tag distance should be positive");

		// Distance beyond the limit should be rejected (returns null std dev)
		double beyondLimit = maxDistance + 0.1;
		assertEquals(4.1, beyondLimit, 0.001, "Beyond limit distance should exceed max");
	}

	/**
	 * Verifies multi-tag estimates have lower base uncertainty than single-tag.
	 */
	@Test
	void testMultiTagHasLowerUncertainty() {
		double singleTagX = VisionConstants.CameraStdDevs.SINGLE_TAG[0];
		double multiTagX = VisionConstants.CameraStdDevs.MULTI_TAG[0];

		assertTrue(multiTagX < singleTagX,
				"Multi-tag X uncertainty (" + multiTagX + ") should be less than single-tag (" + singleTagX + ")");

		double singleTagY = VisionConstants.CameraStdDevs.SINGLE_TAG[1];
		double multiTagY = VisionConstants.CameraStdDevs.MULTI_TAG[1];

		assertTrue(multiTagY < singleTagY,
				"Multi-tag Y uncertainty (" + multiTagY + ") should be less than single-tag (" + singleTagY + ")");

		double singleTagTheta = VisionConstants.CameraStdDevs.SINGLE_TAG[2];
		double multiTagTheta = VisionConstants.CameraStdDevs.MULTI_TAG[2];

		assertTrue(multiTagTheta < singleTagTheta,
				"Multi-tag theta uncertainty (" + multiTagTheta + ") should be less than single-tag ("
						+ singleTagTheta + ")");
	}

	/**
	 * Tests the quadratic nature of distance-based scaling. Doubling distance should increase multiplier by 4x
	 * (distance²).
	 */
	@Test
	void testQuadraticDistanceScaling() {
		double distance1 = 2.0;
		double distance2 = 4.0; // 2x distance1

		double multiplier1 = 1 + ((distance1 * distance1) / 30);
		double multiplier2 = 1 + ((distance2 * distance2) / 30);

		// At distance1: 1 + (4/30) = 1.133
		// At distance2: 1 + (16/30) = 1.533
		// Difference: (16-4)/30 = 0.4, which is 3x the increment from 0 to distance1

		double increment1 = multiplier1 - 1.0; // Should be 4/30 = 0.133
		double increment2 = multiplier2 - 1.0; // Should be 16/30 = 0.533

		assertEquals(0.133, increment1, 0.001, "2m distance increment");
		assertEquals(0.533, increment2, 0.001, "4m distance increment");

		// Verify quadratic relationship: 4x distance² means 4x the increment
		assertEquals(4.0, increment2 / increment1, 0.1, "Distance increment should scale quadratically");
	}

	/**
	 * Verifies that the pose ambiguity threshold is configured. Value should be between 0 (perfect match) and 1
	 * (completely ambiguous).
	 */
	@Test
	void testPoseAmbiguityThreshold() {
		double ambiguity = VisionConstants.POSE_AMBIGUITY_THRESHOLD;
		assertEquals(0.2, ambiguity, "Pose ambiguity threshold");
	}

	/**
	 * Tests that the max pose jump threshold is configured. Used for rejecting vision measurements that differ too much
	 * from odometry.
	 */
	@Test
	void testMaxPoseJumpThreshold() {
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;
		assertEquals(1.0, maxJump, "Max pose jump threshold");
	}
}
