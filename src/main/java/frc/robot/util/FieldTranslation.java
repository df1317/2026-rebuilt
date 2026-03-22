package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Alliance-aware {@link Translation2d} wrapper. Stores a blue-origin translation and
 * pre-computes all flipped variants at construction time. Call {@link #get()} to retrieve
 * the correct variant for the current alliance.
 *
 * <p>Adapted from Team 340 (GRR) — licensed under GPLv3.
 */
public final class FieldTranslation implements Supplier<Translation2d> {

	private final Translation2d original;
	private final Translation2d overWidth;
	private final Translation2d overLength;
	private final Translation2d overDiagonal;

	public FieldTranslation(double x, double y) {
		this(new Translation2d(x, y));
	}

	public FieldTranslation(double distance, Rotation2d angle) {
		this(new Translation2d(distance, angle));
	}

	public FieldTranslation(Translation2d blueTranslation) {
		this.original = blueTranslation;
		this.overWidth = FieldFlip.overWidth(blueTranslation);
		this.overLength = FieldFlip.overLength(blueTranslation);
		this.overDiagonal = FieldFlip.overDiagonal(blueTranslation);
	}

	/** Returns the translation for the current alliance. */
	@Override
	public Translation2d get() {
		return get(false);
	}

	/** Returns the translation for the current alliance, optionally flipped across field width. */
	public Translation2d get(boolean flipWidth) {
		return get(isBlue(), flipWidth);
	}

	/** Returns the translation for the specified alliance, optionally flipped across field width. */
	public Translation2d get(boolean blue, boolean flipWidth) {
		if (blue) {
			return !flipWidth ? original : overWidth;
		}
		return switch (FieldFlip.SYMMETRY) {
			case MIRROR -> !flipWidth ? overLength : overDiagonal;
			case ROTATE -> !flipWidth ? overDiagonal : overLength;
		};
	}

	/** Returns the blue-alliance translation. */
	public Translation2d getBlue() {
		return original;
	}

	/** Returns the red-alliance translation. */
	public Translation2d getRed() {
		return get(false, false);
	}

	private static boolean isBlue() {
		return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
	}
}
