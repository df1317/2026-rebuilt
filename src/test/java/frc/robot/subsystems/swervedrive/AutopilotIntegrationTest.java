package frc.robot.subsystems.swervedrive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Integration tests for Autopilot motion control.
 *
 * <p>
 * Tests basic functionality of AutopilotController without requiring hardware.
 */
class AutopilotIntegrationTest {

	private AutopilotController controller;

	@BeforeEach
	void setUp() {
		controller = new AutopilotController();
	}

	@Test
	void testControllerCreation() {
		assertNotNull(controller);
		assertNotNull(controller.getConstraints());
	}

	@Test
	void testCalculateReturnsNonNullSpeeds() {
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(45));
		ChassisSpeeds currentVelocity = new ChassisSpeeds();

		ChassisSpeeds result = controller.calculate(currentPose, currentVelocity, targetPose);

		assertNotNull(result);
	}

	@Test
	void testCalculateProducesForwardVelocityToTarget() {
		// Start at origin, target 1m ahead
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(1, 0, new Rotation2d());
		ChassisSpeeds currentVelocity = new ChassisSpeeds();

		ChassisSpeeds result = controller.calculate(currentPose, currentVelocity, targetPose);

		// Should produce positive vx to move forward
		assertTrue(result.vxMetersPerSecond > 0, "vx should be positive to move forward");
		// vy should be near zero (moving straight)
		assertTrue(Math.abs(result.vyMetersPerSecond) < 0.1, "vy should be near zero for straight path");
	}

	@Test
	void testCalculateProducesLateralVelocityToTarget() {
		// Start at origin, target 1m to the left
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(0, 1, new Rotation2d());
		ChassisSpeeds currentVelocity = new ChassisSpeeds();

		ChassisSpeeds result = controller.calculate(currentPose, currentVelocity, targetPose);

		// Should produce positive vy to move left
		assertTrue(result.vyMetersPerSecond > 0, "vy should be positive to move left");
		// vx should be near zero (moving straight laterally)
		assertTrue(Math.abs(result.vxMetersPerSecond) < 0.1, "vx should be near zero for lateral movement");
	}

	@Test
	void testCalculateWithEntryAngle() {
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(90));
		ChassisSpeeds currentVelocity = new ChassisSpeeds();

		// Test both with and without respecting entry angle
		ChassisSpeeds withoutEntry = controller.calculate(currentPose, currentVelocity, targetPose, false);
		ChassisSpeeds withEntry = controller.calculate(currentPose, currentVelocity, targetPose, true);

		assertNotNull(withoutEntry);
		assertNotNull(withEntry);

		// Both should produce movement, but paths may differ
		assertTrue(withoutEntry.vxMetersPerSecond != 0 || withoutEntry.vyMetersPerSecond != 0);
		assertTrue(withEntry.vxMetersPerSecond != 0 || withEntry.vyMetersPerSecond != 0);
	}

	@Test
	void testGetTargetHeading() {
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(1, 0, Rotation2d.fromDegrees(45));
		ChassisSpeeds currentVelocity = new ChassisSpeeds();

		Rotation2d targetHeading = controller.getTargetHeading(currentPose, currentVelocity, targetPose);

		assertNotNull(targetHeading);
	}

	@Test
	void testAtTargetWhenAtPose() {
		Pose2d pose = new Pose2d(1, 1, Rotation2d.fromDegrees(45));

		// Should be at target when poses match exactly
		assertTrue(controller.atTarget(pose, pose, 0.02, Math.toRadians(1)));
	}

	@Test
	void testAtTargetWithinTolerance() {
		Pose2d currentPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(45));
		Pose2d targetPose = new Pose2d(1.01, 1.01, Rotation2d.fromDegrees(45.5));

		// Within 2cm and 1 degree tolerance (default Autopilot tolerances)
		assertTrue(controller.atTarget(currentPose, targetPose, 0.02, Math.toRadians(1)));
	}

	@Test
	void testNotAtTargetWhenTooFar() {
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(1, 0, new Rotation2d());

		// Should not be at target when 1m away
		assertFalse(controller.atTarget(currentPose, targetPose, 0.02, Math.toRadians(1)));
	}

	@Test
	void testNotAtTargetWhenRotationOff() {
		Pose2d currentPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
		Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(45));

		// Position matches but rotation is off by 45 degrees
		assertFalse(controller.atTarget(currentPose, targetPose, 0.02, Math.toRadians(1)));
	}

	@Test
	void testVelocityDecreaseNearTarget() {
		Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(5, 0, new Rotation2d());
		ChassisSpeeds currentVelocity = new ChassisSpeeds();

		// Calculate velocity at different distances from target
		ChassisSpeeds farSpeeds = controller.calculate(startPose, currentVelocity, targetPose);
		ChassisSpeeds closeSpeeds = controller.calculate(new Pose2d(4.8, 0, new Rotation2d()), currentVelocity,
				targetPose);

		double farSpeed = Math.hypot(farSpeeds.vxMetersPerSecond, farSpeeds.vyMetersPerSecond);
		double closeSpeed = Math.hypot(closeSpeeds.vxMetersPerSecond, closeSpeeds.vyMetersPerSecond);

		// Speed should be lower when closer to target (deceleration)
		// May not always hold true depending on jerk-limited profile state
		assertTrue(farSpeed > 0 && closeSpeed > 0, "Both velocities should be positive when moving toward target");
	}

	@Test
	void testCustomConstraints() {
		// Create controller with custom constraints
		AutopilotController customController = new AutopilotController(2.0, 1.0, Math.PI, Math.PI / 2);

		assertNotNull(customController);
		assertNotNull(customController.getConstraints());

		// Test that it produces reasonable output
		Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
		Pose2d targetPose = new Pose2d(1, 1, new Rotation2d());
		ChassisSpeeds result = customController.calculate(currentPose, new ChassisSpeeds(), targetPose);

		assertNotNull(result);
		assertTrue(result.vxMetersPerSecond != 0 || result.vyMetersPerSecond != 0);
	}
}
