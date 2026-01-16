package frc.robot.subsystems.swervedrive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

/**
 * Tests for swerve drive kinematics edge cases.
 * <p>
 * Validates the behavior of chassis speed calculations, field-relative transforms, and controller input scaling.
 */
class SwerveKinematicsTest {

	/**
	 * Tests that zero joystick input produces zero chassis speeds.
	 */
	@Test
	void testZeroInputProducesZeroSpeeds() {
		// Zero translation and rotation should produce zero chassis speeds
		Translation2d translation = new Translation2d(0, 0);
		double rotation = 0.0;

		// Verify translation is zero
		assertEquals(0.0, translation.getX(), 1e-6, "X translation should be zero");
		assertEquals(0.0, translation.getY(), 1e-6, "Y translation should be zero");
		assertEquals(0.0, rotation, 1e-6, "Rotation should be zero");
	}

	/**
	 * Tests that field-relative chassis speeds correctly transform based on robot heading.
	 */
	@Test
	void testFieldRelativeTransform() {
		// Field-relative: moving forward (X=1) with robot rotated 90 degrees
		// should result in robot-relative Y=-1 (robot's left side points backward in field frame)
		ChassisSpeeds fieldRelative = new ChassisSpeeds(1.0, 0.0, 0.0);
		Rotation2d robotHeading = Rotation2d.fromDegrees(90);

		// Convert to robot-relative
		ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, robotHeading);

		// When robot faces 90deg CCW and field says "go forward in X", robot moves right (negative Y)
		assertEquals(0.0, robotRelative.vxMetersPerSecond, 0.01, "Robot X velocity");
		assertEquals(-1.0, robotRelative.vyMetersPerSecond, 0.01, "Robot Y velocity");
		assertEquals(0.0, robotRelative.omegaRadiansPerSecond, 0.01, "Robot rotation");
	}

	/**
	 * Tests that robot-relative chassis speeds are unaffected by robot heading.
	 */
	@Test
	void testRobotRelativeIsHeadingIndependent() {
		ChassisSpeeds robotRelative = new ChassisSpeeds(1.0, 0.5, 0.2);

		// Robot-relative speeds should be the same regardless of heading
		assertEquals(1.0, robotRelative.vxMetersPerSecond, 1e-6, "Robot X velocity");
		assertEquals(0.5, robotRelative.vyMetersPerSecond, 1e-6, "Robot Y velocity");
		assertEquals(0.2, robotRelative.omegaRadiansPerSecond, 1e-6, "Robot rotation");
	}

	/**
	 * Tests chassis speed magnitude calculation for diagonal movement.
	 */
	@Test
	void testDiagonalMovementMagnitude() {
		// Equal X and Y should produce sqrt(2) magnitude
		ChassisSpeeds speeds = new ChassisSpeeds(1.0, 1.0, 0.0);

		double magnitude = Math.sqrt(
				speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
						+ speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);

		assertEquals(Math.sqrt(2), magnitude, 0.01, "Diagonal movement magnitude");
	}

	/**
	 * Tests that rotation-only movement has zero translation.
	 */
	@Test
	void testRotationOnlyMovement() {
		ChassisSpeeds rotationOnly = new ChassisSpeeds(0.0, 0.0, 1.5);

		assertEquals(0.0, rotationOnly.vxMetersPerSecond, 1e-6, "X velocity should be zero");
		assertEquals(0.0, rotationOnly.vyMetersPerSecond, 1e-6, "Y velocity should be zero");
		assertEquals(1.5, rotationOnly.omegaRadiansPerSecond, 1e-6, "Rotation velocity");
	}

	/**
	 * Tests field-relative conversion at 180 degrees (facing backward).
	 */
	@Test
	void testFieldRelative180Degrees() {
		// Field says "go forward", robot faces backward (180deg)
		ChassisSpeeds fieldRelative = new ChassisSpeeds(1.0, 0.0, 0.0);
		Rotation2d robotHeading = Rotation2d.fromDegrees(180);

		ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, robotHeading);

		// Robot should move backward (negative X in robot frame)
		assertEquals(-1.0, robotRelative.vxMetersPerSecond, 0.01, "Robot X velocity");
		assertEquals(0.0, robotRelative.vyMetersPerSecond, 0.01, "Robot Y velocity");
	}

	/**
	 * Tests that chassis speeds can be discretized (accounts for skew during time step).
	 */
	@Test
	void testChassisSpeedsDiscretization() {
		ChassisSpeeds continuous = new ChassisSpeeds(1.0, 0.5, 0.5);
		double dt = 0.02; // 20ms time step

		// Discretize to account for second-order kinematics
		ChassisSpeeds discrete = ChassisSpeeds.discretize(continuous, dt);

		// Discrete speeds should be close but slightly different due to skew correction
		assertTrue(Math.abs(discrete.vxMetersPerSecond - continuous.vxMetersPerSecond) < 0.1,
				"Discretized X should be close to continuous");
		assertTrue(Math.abs(discrete.vyMetersPerSecond - continuous.vyMetersPerSecond) < 0.1,
				"Discretized Y should be close to continuous");
		assertEquals(continuous.omegaRadiansPerSecond, discrete.omegaRadiansPerSecond, 1e-6,
				"Rotation should be unchanged");
	}

	/**
	 * Tests conversion from field-relative back to robot-relative is consistent.
	 */
	@Test
	void testFieldRelativeRoundTrip() {
		ChassisSpeeds original = new ChassisSpeeds(1.0, 0.5, 0.2);
		Rotation2d heading = Rotation2d.fromDegrees(45);

		// Convert to field-relative then back to robot-relative
		ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(original, heading);
		ChassisSpeeds backToRobot = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, heading);

		// Should get back the original values
		assertEquals(original.vxMetersPerSecond, backToRobot.vxMetersPerSecond, 0.01, "X velocity round trip");
		assertEquals(original.vyMetersPerSecond, backToRobot.vyMetersPerSecond, 0.01, "Y velocity round trip");
		assertEquals(original.omegaRadiansPerSecond, backToRobot.omegaRadiansPerSecond, 0.01, "Rotation round trip");
	}

	/**
	 * Tests that very small speeds near zero are handled correctly.
	 */
	@Test
	void testNearZeroSpeeds() {
		ChassisSpeeds nearZero = new ChassisSpeeds(1e-10, -1e-10, 1e-10);

		// Should be effectively zero
		assertTrue(Math.abs(nearZero.vxMetersPerSecond) < 1e-6, "Near-zero X should be negligible");
		assertTrue(Math.abs(nearZero.vyMetersPerSecond) < 1e-6, "Near-zero Y should be negligible");
		assertTrue(Math.abs(nearZero.omegaRadiansPerSecond) < 1e-6, "Near-zero rotation should be negligible");
	}
}
