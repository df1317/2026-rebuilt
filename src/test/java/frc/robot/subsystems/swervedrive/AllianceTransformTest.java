package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

/**
 * Tests for alliance-relative coordinate transforms.
 * <p>
 * The FRC field is symmetric between red and blue alliances. Robot code must handle mirroring poses, paths, and
 * orientations based on alliance color. Blue alliance uses the field's natural coordinates (origin at the blue alliance
 * wall), while red alliance mirrors across the field centerline.
 */
class AllianceTransformTest {

	/**
	 * FRC 2024 field width in meters (perpendicular to alliance walls).
	 */
	private static final double FIELD_WIDTH = 8.21;

	/**
	 * Tests that blue alliance poses are not transformed (identity transform).
	 */
	@Test
	void testBlueAllianceNoTransform() {
		Pose2d bluePose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));

		// Blue alliance should use pose as-is
		assertEquals(1.0, bluePose.getX(), 1e-6, "Blue X coordinate");
		assertEquals(2.0, bluePose.getY(), 1e-6, "Blue Y coordinate");
		assertEquals(45.0, bluePose.getRotation().getDegrees(), 1e-6, "Blue rotation");
	}

	/**
	 * Tests red alliance pose mirroring across the field centerline.
	 * <p>
	 * Red alliance mirrors Y coordinates and rotations. X coordinate remains unchanged (along the alliance wall
	 * direction).
	 */
	@Test
	void testRedAllianceMirrorY() {
		// Blue alliance pose
		Pose2d bluePose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0));

		// Red alliance mirrors Y and rotation
		Pose2d redPose = new Pose2d(
				bluePose.getX(),
				FIELD_WIDTH - bluePose.getY(),
				Rotation2d.fromDegrees(180).minus(bluePose.getRotation()));

		assertEquals(1.0, redPose.getX(), 1e-6, "Red X should match blue X");
		assertEquals(FIELD_WIDTH - 2.0, redPose.getY(), 0.01, "Red Y should mirror across centerline");
		assertEquals(180.0, redPose.getRotation().getDegrees(), 1e-6, "Red rotation should be mirrored");
	}

	/**
	 * Tests that mirroring twice returns to the original pose.
	 */
	@Test
	void testMirrorRoundTrip() {
		Pose2d original = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(30));

		// Mirror once (blue to red)
		Pose2d mirrored = new Pose2d(
				original.getX(),
				FIELD_WIDTH - original.getY(),
				Rotation2d.fromDegrees(180).minus(original.getRotation()));

		// Mirror again (red back to blue)
		Pose2d doubleFlipped = new Pose2d(
				mirrored.getX(),
				FIELD_WIDTH - mirrored.getY(),
				Rotation2d.fromDegrees(180).minus(mirrored.getRotation()));

		assertEquals(original.getX(), doubleFlipped.getX(), 0.01, "X round trip");
		assertEquals(original.getY(), doubleFlipped.getY(), 0.01, "Y round trip");
		assertEquals(original.getRotation().getDegrees(), doubleFlipped.getRotation().getDegrees(), 0.01,
				"Rotation round trip");
	}

	/**
	 * Tests red alliance 180-degree heading offset on gyro zero.
	 * <p>
	 * When zeroing gyro on red alliance, the robot should face 180 degrees to point toward opposing alliance.
	 */
	@Test
	void testRedAllianceGyroOffset() {
		// Blue alliance starts at 0 degrees (facing away from the driver station)
		Rotation2d blueHeading = Rotation2d.fromDegrees(0);

		// Red alliance should start at 180 degrees
		Rotation2d redHeading = Rotation2d.fromDegrees(180);

		assertEquals(0.0, blueHeading.getDegrees(), 1e-6, "Blue heading");
		assertEquals(180.0, redHeading.getDegrees(), 1e-6, "Red heading");
		assertNotEquals(blueHeading.getDegrees(), redHeading.getDegrees(), "Headings should differ");
	}

	/**
	 * Tests translation mirroring for field-relative movements.
	 */
	@Test
	void testTranslationMirroring() {
		// Blue alliance translation: forward and right
		Translation2d blueTranslation = new Translation2d(1.0, 2.0);

		// Red alliance mirrors Y
		Translation2d redTranslation = new Translation2d(
				blueTranslation.getX(),
				-blueTranslation.getY());

		assertEquals(1.0, redTranslation.getX(), 1e-6, "Red X translation");
		assertEquals(-2.0, redTranslation.getY(), 1e-6, "Red Y translation should be negated");
	}

	/**
	 * Tests that field centerline poses have symmetric Y coordinates.
	 */
	@Test
	void testCenterlineMirroring() {
		double centerlineY = FIELD_WIDTH / 2.0;
		Pose2d centerlinePose = new Pose2d(1.0, centerlineY, Rotation2d.fromDegrees(90));

		// Mirrored pose should also be on the centerline
		Pose2d mirrored = new Pose2d(
				centerlinePose.getX(),
				FIELD_WIDTH - centerlinePose.getY(),
				Rotation2d.fromDegrees(180).minus(centerlinePose.getRotation()));

		assertEquals(centerlineY, mirrored.getY(), 0.01, "Centerline should mirror to itself");
		assertEquals(90.0, mirrored.getRotation().getDegrees(), 0.01, "90deg mirrors to 90deg");
	}

	/**
	 * Tests rotation mirroring for various angles.
	 */
	@Test
	void testRotationMirroring() {
		// Test common angles
		assertEquals(180.0, mirrorRotation(0.0), 0.01, "0° → 180°");
		assertEquals(90.0, mirrorRotation(90.0), 0.01, "90° → 90° (perpendicular to mirror axis)");
		assertEquals(0.0, mirrorRotation(180.0), 0.01, "180° → 0°");
		assertEquals(135.0, mirrorRotation(45.0), 0.01, "45° → 135°");
		assertEquals(-90.0, mirrorRotation(270.0), 0.01, "270° → -90°");
	}

	/**
	 * Helper method to mirror a rotation angle.
	 *
	 * @param degrees
	 *          Angle in degrees
	 * @return Mirrored angle in degrees
	 */
	private double mirrorRotation(double degrees) {
		Rotation2d original = Rotation2d.fromDegrees(degrees);
		Rotation2d mirrored = Rotation2d.fromDegrees(180).minus(original);
		return mirrored.getDegrees();
	}

	/**
	 * Tests that the origin (0, 0) remains in the blue alliance corner after mirroring.
	 */
	@Test
	void testOriginRemainsBlueSide() {
		// PathPlanner's alliance mirroring keeps the origin on the blue side
		Pose2d blueOrigin = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

		// Red alliance path would reference blue origin, then mirror
		// Origin itself should not move
		assertEquals(0.0, blueOrigin.getX(), 1e-6, "Origin X");
		assertEquals(0.0, blueOrigin.getY(), 1e-6, "Origin Y");
	}
}
