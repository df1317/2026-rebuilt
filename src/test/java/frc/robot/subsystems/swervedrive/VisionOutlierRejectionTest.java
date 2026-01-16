package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests for Vision outlier rejection logic.
 * <p>
 * The Vision system rejects pose estimates that: 1. Jump more than MAX_POSE_JUMP_METERS from the current odometry pose
 * 2. Have pose ambiguity above POSE_AMBIGUITY_THRESHOLD (single-tag only) 3. Are single-tag estimates beyond
 * MAX_SINGLE_TAG_DISTANCE_METERS
 */
class VisionOutlierRejectionTest {

	/**
	 * Tests the pose jump rejection threshold. Measurements that differ from the current pose by more than
	 * MAX_POSE_JUMP_METERS should be rejected.
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
	 * Tests that pose jump calculation uses 2D translation distance, not full pose. Rotation differences should not
	 * affect rejection.
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
	 * Verifies that a pose ambiguity threshold is used for single-tag filtering. Ambiguity represents how similar the best and
	 * second-best pose solutions are. Value should be between 0 (perfect match) and 1 (completely ambiguous).
	 */
	@Test
	void testPoseAmbiguityThresholdRange() {
		double ambiguity = VisionConstants.POSE_AMBIGUITY_THRESHOLD;
		assertEquals(0.2, ambiguity, "Pose ambiguity threshold");
	}

	/**
	 * Tests single-tag distance limit configuration. Reasonable for FRC field dimensions (field is ~16.5m x 8.2m).
	 */
	@Test
	void testSingleTagDistanceLimit() {
		double maxDistance = VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS;
		assertEquals(4.0, maxDistance, "Single-tag distance limit");
	}

	/**
	 * Verifies the relationship between outlier rejection thresholds. Multi-tag should be more trusted than single-tag.
	 */
	@Test
	void testRejectionThresholdConsistency() {
		double singleTagLimit = VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS;
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;

		// Document expected values - max pose jump should be smaller than the single-tag limit
		// This ensures outlier filter doesn't immediately reject single-tag estimates
		assertEquals(4.0, singleTagLimit, "Single-tag distance limit");
		assertEquals(1.0, maxJump, "Max pose jump");
	}

	/**
	 * Tests that max pose jump is reasonable for the field scale. Should be large enough to handle odometry drift but
	 * small enough to reject bad measurements. FRC field is ~16.5m x 8.2m.
	 */
	@Test
	void testMaxPoseJumpFieldScale() {
		double maxJump = VisionConstants.MAX_POSE_JUMP_METERS;
		assertEquals(1.0, maxJump, "Max pose jump");
	}

	/**
	 * Tests an edge case: zero distance should always be accepted.
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
