package frc.robot.repulsor.FieldPlanner.Obstacles;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.repulsor.FieldPlanner.Obstacle;
import frc.robot.repulsor.Force;

public class LineObstacle extends Obstacle {
	private final Translation2d start;
	private final Translation2d end;
	private final boolean sidesOnly;

	private final double length;
	private final Rotation2d inverse;
	private final Rotation2d perpendicular;

	private final double range;

	public LineObstacle(Translation2d start, Translation2d end, double strength, double range) {
		this(start, end, false, strength, range);
	}

	public LineObstacle(Translation2d start, Translation2d end, boolean sidesOnly, double strength, double range) {
		super(strength, strength > 0);
		this.start = start;
		this.end = end;
		this.sidesOnly = sidesOnly;
		this.range = range;

		Translation2d difference = end.minus(start);
		length = difference.getNorm();
		inverse = difference.getAngle().unaryMinus();
		perpendicular = difference.getAngle().rotateBy(Rotation2d.kCCW_Pi_2);
	}

	@Override
	public Force getForceAtPosition(Translation2d position, Translation2d target) {
		double startDist_x = position.getX() - start.getX();
		double startDist_y = position.getY() - start.getY();

		double proj_x = startDist_x * inverse.getCos() - startDist_y * inverse.getSin();
		double proj_y = startDist_x * inverse.getSin() + startDist_y * inverse.getCos();

		if (proj_x > 0.0 && proj_x < length) {
			double magnitude = getForceMagnitude(proj_y) * Math.signum(proj_y);
			return new Force(magnitude * perpendicular.getCos(), magnitude * perpendicular.getSin());
		} else if (!sidesOnly) {
			Translation2d closest = proj_x <= 0.0 ? start : end;
			double d_x = position.getX() - closest.getX();
			double d_y = position.getY() - closest.getY();
			double d_norm = Math.max(1e-6, Math.hypot(d_x, d_y));
			double magnitude = getForceMagnitude(d_norm);

			return new Force(magnitude * (d_x / d_norm), magnitude * (d_y / d_norm));
		}
		return new Force(0, 0);
	}

	private double getForceMagnitude(double dist) {
		dist = Math.abs(dist);
		if (dist > range)
			return 0.0;

		double t = (positive ? (range - dist) : dist) / range;
		return strength * (t * t); // Default quadratic heuristic from 340
	}
}
