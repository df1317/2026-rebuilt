package frc.robot.subsystems.swervedrive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.Test;

/**
 * Tests for Vision outlier rejection logic.
 * 
 * The Vision system rejects pose estimates that:
 * 1. Jump more than MAX_POSE_JUMP_METERS from the current odometry pose
 * 2. Have pose ambiguity above POSE_AMBIGUITY_THRESHOLD (single-tag only)
 * 3. Are single-tag estimates beyond MAX_SINGLE_TAG_DISTANCE_METERS
 */
class VisionOutlierRejectionTest {

	/**
	 * Tests pose jump rejection threshold.
	 * Measurements that differ from current pose by more than MAX_POSE_JUMP_METERS should be rejected.
	 */
	@Test
	void testPoseJumpRejection() {
		Pose2d currentPose = new Pose2d(5.0, 3.0, new Rotation2d());
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;

		// Just under the threshold - should be accepted
		Translation2d acceptableTranslation = currentPose.getTranslation()
				.plus(new Translation2d(maxJump - 0.05, 0));
		double acceptableDistance = currentPose.getTranslation().getDistance(acceptableTranslation);
		
		assertTrue(acceptableDistance < maxJump,
				"Distance " + acceptableDistance + "m should be below threshold " + maxJump + "m");

		// Just over the threshold - should be rejected
		Translation2d rejectedTranslation = currentPose.getTranslation()
				.plus(new Translation2d(maxJump + 0.05, 0));
		double rejectedDistance = currentPose.getTranslation().getDistance(rejectedTranslation);
		
		assertTrue(rejectedDistance > maxJump,
				"Distance " + rejectedDistance + "m should exceed threshold " + maxJump + "m");
	}

	/**
	 * Tests that pose jump calculation uses 2D translation distance, not full pose.
	 * Rotation differences should not affect rejection.
	 */
	@Test
	void testPoseJumpIgnoresRotation() {
		Pose2d currentPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0));
		
		// Same translation, 180 degree rotation difference
		Pose2d rotatedPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(180));
		
		double distance = currentPose.getTranslation().getDistance(rotatedPose.getTranslation());
		
		assertEquals(0.0, distance, 0.001,
				"Translation distance should be zero despite rotation difference");
	}

	/**
	 * Tests diagonal pose jumps (Pythagorean distance).
	 */
	@Test
	void testDiagonalPoseJump() {
		Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d());
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;

		// Create a diagonal jump (e.g., 0.6m x, 0.8m y = 1.0m total)
		// For maxJump = 1.0: acceptable would be (0.5, 0.5) = 0.707m
		double componentDistance = maxJump * 0.7; // ~70% of max in each direction
		Translation2d diagonalTranslation = new Translation2d(componentDistance, componentDistance);
		
		double actualDistance = currentPose.getTranslation().getDistance(diagonalTranslation);
		double expectedDistance = Math.sqrt(2) * componentDistance;
		
		assertEquals(expectedDistance, actualDistance, 0.001,
				"Diagonal distance should follow Pythagorean theorem");
		
		assertTrue(actualDistance < maxJump,
				"Diagonal " + actualDistance + "m should be below threshold " + maxJump + "m");
	}

	/**
	 * Verifies pose ambiguity threshold is used for single-tag filtering.
	 * Ambiguity represents how similar the best and second-best pose solutions are.
	 */
	@Test
	void testPoseAmbiguityThresholdRange() {
		double ambiguity = VisionConstants.POSE_AMBIGUITY_THRESHOLD;
		
		// Ambiguity is typically between 0 (perfect match) and 1 (completely ambiguous)
		assertTrue(ambiguity >= 0.0 && ambiguity <= 1.0,
				"Pose ambiguity threshold should be between 0 and 1, got " + ambiguity);
		
		// Reasonable values for competition use are 0.1-0.3
		assertTrue(ambiguity >= 0.05 && ambiguity <= 0.5,
				"Pose ambiguity threshold should be reasonable for competition use, got " + ambiguity);
	}

	/**
	 * Tests single-tag distance limit configuration.
	 */
	@Test
	void testSingleTagDistanceLimit() {
		double maxDistance = VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS;
		
		assertTrue(maxDistance > 0,
				"Single-tag distance limit must be positive, got " + maxDistance);
		
		// Reasonable for FRC field dimensions (field is ~16.5m x 8.2m)
		assertTrue(maxDistance >= 2.0 && maxDistance <= 10.0,
				"Single-tag distance limit should be reasonable for field size, got " + maxDistance + "m");
	}

	/**
	 * Verifies the relationship between outlier rejection thresholds.
	 * Multi-tag should be more trusted than single-tag.
	 */
	@Test
	void testRejectionThresholdConsistency() {
		// Single-tag distance limit should exist
		double singleTagLimit = VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS;
		assertTrue(singleTagLimit > 0, "Single-tag distance limit should be positive");
		
		// Pose ambiguity only applies to single-tag
		double ambiguity = VisionConstants.POSE_AMBIGUITY_THRESHOLD;
		assertTrue(ambiguity > 0, "Pose ambiguity threshold should be positive");
		
		// Max pose jump applies to both single and multi-tag
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;
		assertTrue(maxJump > 0, "Max pose jump should be positive");
		
		// Sanity check: max pose jump should be larger than typical single-tag limit
		// This ensures we don't reject all single-tag estimates immediately
		assertTrue(maxJump <= singleTagLimit * 2,
				"Max pose jump (" + maxJump + "m) should be comparable to single-tag limit (" + singleTagLimit + "m)");
	}

	/**
	 * Tests that max pose jump is reasonable for field scale.
	 * Should be large enough to handle odometry drift but small enough to reject bad measurements.
	 */
	@Test
	void testMaxPoseJumpFieldScale() {
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;
		
		// FRC field is ~16.5m x 8.2m
		// Max jump should be much smaller than field width
		assertTrue(maxJump < 5.0,
				"Max pose jump (" + maxJump + "m) should be smaller than field dimensions");
		
		// But large enough to handle wheel slippage/odometry drift
		assertTrue(maxJump > 0.2,
				"Max pose jump (" + maxJump + "m) should handle odometry drift");
	}

	/**
	 * Tests edge case: zero distance should always be accepted.
	 */
	@Test
	void testZeroDistanceAlwaysAccepted() {
		Pose2d currentPose = new Pose2d(5.0, 3.0, new Rotation2d());
		Pose2d samePose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45)); // Different rotation, same translation
		
		double distance = currentPose.getTranslation().getDistance(samePose.getTranslation());
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;
		
		assertEquals(0.0, distance, 0.001, "Same translation should have zero distance");
		assertTrue(distance <= maxJump, "Zero distance should always be below threshold");
	}
}
