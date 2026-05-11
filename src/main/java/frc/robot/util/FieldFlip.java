package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Utility class for flipping field locations across lines of symmetry.
 * Field dimensions and symmetry type are derived from the AprilTag field layout.
 *
 * <p>
 * Adapted from Team 340 (GRR) — licensed under GPLv3.
 */
public final class FieldFlip {

	private FieldFlip() {
	}

	/**
	 * Specifies the direction in which the field is symmetric.
	 */
	public enum SymmetryType {
		/** The field is symmetric over its length (e.g. 2024 Crescendo). */
		MIRROR,
		/** The field is symmetric over its diagonal / 180° rotation (e.g. 2026 Rebuilt). */
		ROTATE
	}

	private static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(Constants.FIELD_LAYOUT);

	/** The symmetry type for the current game field, derived from the AprilTag field layout. */
	public static final SymmetryType SYMMETRY = symmetryFor(Constants.FIELD_LAYOUT);

	/** Returns the AprilTag field layout for the current game. */
	public static AprilTagFieldLayout aprilTagLayout() {
		return LAYOUT;
	}

	private static SymmetryType symmetryFor(AprilTagFields field) {
		return switch (field) {
			case k2026RebuiltWelded, k2026RebuiltAndymark,
					k2025ReefscapeWelded, k2025ReefscapeAndyMark,
					k2022RapidReact ->
				SymmetryType.ROTATE;
			case k2024Crescendo, k2023ChargedUp -> SymmetryType.MIRROR;
		};
	}

	/** Returns the field length (X-axis) in meters. */
	public static double fieldLength() {
		return LAYOUT.getFieldLength();
	}

	/** Returns the field width (Y-axis) in meters. */
	public static double fieldWidth() {
		return LAYOUT.getFieldWidth();
	}

	// ===== Over Length (hamburger flip — across center line perpendicular to X) =====

	public static Translation2d overLength(Translation2d t) {
		return new Translation2d(fieldLength() - t.getX(), t.getY());
	}

	public static Rotation2d overLength(Rotation2d r) {
		return new Rotation2d(-r.getCos(), r.getSin());
	}

	public static Pose2d overLength(Pose2d p) {
		return new Pose2d(overLength(p.getTranslation()), overLength(p.getRotation()));
	}

	// ===== Over Width (hotdog flip — across center line perpendicular to Y) =====

	public static Translation2d overWidth(Translation2d t) {
		return new Translation2d(t.getX(), fieldWidth() - t.getY());
	}

	public static Rotation2d overWidth(Rotation2d r) {
		return r.unaryMinus();
	}

	public static Pose2d overWidth(Pose2d p) {
		return new Pose2d(overWidth(p.getTranslation()), overWidth(p.getRotation()));
	}

	// ===== Over Diagonal (180° rotation around field center) =====

	public static Translation2d overDiagonal(Translation2d t) {
		return new Translation2d(fieldLength() - t.getX(), fieldWidth() - t.getY());
	}

	public static Rotation2d overDiagonal(Rotation2d r) {
		return new Rotation2d(-r.getCos(), -r.getSin());
	}

	public static Pose2d overDiagonal(Pose2d p) {
		return new Pose2d(overDiagonal(p.getTranslation()), overDiagonal(p.getRotation()));
	}

	// ===== Alliance Flip (uses current field symmetry) =====

	/** Flips a blue-origin pose to the red alliance using the current field's symmetry type. */
	public static Pose2d toRed(Pose2d bluePose) {
		return switch (SYMMETRY) {
			case MIRROR -> overLength(bluePose);
			case ROTATE -> overDiagonal(bluePose);
		};
	}

	/** Flips a blue-origin translation to the red alliance using the current field's symmetry type. */
	public static Translation2d toRed(Translation2d blueTranslation) {
		return switch (SYMMETRY) {
			case MIRROR -> overLength(blueTranslation);
			case ROTATE -> overDiagonal(blueTranslation);
		};
	}
}
