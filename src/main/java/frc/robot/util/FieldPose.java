package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Alliance-aware {@link Pose2d} wrapper. Stores a blue-origin pose and pre-computes
 * all flipped variants at construction time. Call {@link #get()} to retrieve the
 * correct variant for the current alliance.
 *
 * <p>
 * Adapted from Team 340 (GRR) — licensed under GPLv3.
 */
public final class FieldPose implements Supplier<Pose2d> {

	private final Pose2d original;
	private final Pose2d overWidth;
	private final Pose2d overLength;
	private final Pose2d overDiagonal;

	public FieldPose(double x, double y, Rotation2d rotation) {
		this(new Pose2d(x, y, rotation));
	}

	public FieldPose(Translation2d translation, Rotation2d rotation) {
		this(new Pose2d(translation, rotation));
	}

	public FieldPose(Pose2d bluePose) {
		this.original = bluePose;
		this.overWidth = FieldFlip.overWidth(bluePose);
		this.overLength = FieldFlip.overLength(bluePose);
		this.overDiagonal = FieldFlip.overDiagonal(bluePose);
	}

	/** Returns the pose for the current alliance. */
	@Override
	public Pose2d get() {
		return get(false);
	}

	/** Returns the pose for the current alliance, optionally flipped across field width. */
	public Pose2d get(boolean flipWidth) {
		return get(isBlue(), flipWidth);
	}

	/** Returns the pose for the specified alliance, optionally flipped across field width. */
	public Pose2d get(boolean blue, boolean flipWidth) {
		if (blue) {
			return !flipWidth ? original : overWidth;
		}
		return switch (FieldFlip.SYMMETRY) {
			case MIRROR -> !flipWidth ? overLength : overDiagonal;
			case ROTATE -> !flipWidth ? overDiagonal : overLength;
		};
	}

	/** Returns the blue-alliance pose. */
	public Pose2d getBlue() {
		return original;
	}

	/** Returns the red-alliance pose. */
	public Pose2d getRed() {
		return get(false, false);
	}

	private static boolean isBlue() {
		return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
	}
}
