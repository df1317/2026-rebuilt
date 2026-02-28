package frc.robot.subsystems.swervedrive;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

/**
 * Wrapper for the Autopilot motion control library. Stateless holonomic controller using
 * jerk-limited profiles. Best for teleop drive-to-pose; use PathPlanner for obstacle avoidance.
 */
public class AutopilotController {

	private final Autopilot autopilot;
	private final APConstraints constraints;
	private final double acceleration;
	private final double jerk;

	public AutopilotController() {
		this(Constants.MAX_ACCELERATION);
	}

	public AutopilotController(double maxAcceleration) {
		this.acceleration = maxAcceleration;
		this.jerk = 1.0;

		this.constraints = new APConstraints()
				.withAcceleration(maxAcceleration)
				.withJerk(this.jerk);

		APProfile profile = new APProfile(constraints)
				.withErrorXY(Centimeters.of(2))
				.withErrorTheta(Degrees.of(1))
				.withBeelineRadius(Centimeters.of(10));

		this.autopilot = new Autopilot(profile);
	}

	public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds currentVelocity, Pose2d targetPose) {
		APTarget target = new APTarget(targetPose);
		APResult result = autopilot.calculate(currentPose, currentVelocity, target);
		double vx = result.vx().in(MetersPerSecond);
		double vy = result.vy().in(MetersPerSecond);
		return new ChassisSpeeds(vx, vy, 0.0);
	}

	public ChassisSpeeds calculate(
			Pose2d currentPose,
			ChassisSpeeds currentVelocity,
			Pose2d targetPose,
			boolean respectEntryAngle) {
		APTarget target = new APTarget(targetPose);
		if (respectEntryAngle) {
			target = target.withEntryAngle(targetPose.getRotation());
		}

		APResult result = autopilot.calculate(currentPose, currentVelocity, target);
		double vx = result.vx().in(MetersPerSecond);
		double vy = result.vy().in(MetersPerSecond);
		return new ChassisSpeeds(vx, vy, 0.0);
	}

	public edu.wpi.first.math.geometry.Rotation2d getTargetHeading(
			Pose2d currentPose,
			ChassisSpeeds currentVelocity,
			Pose2d targetPose) {
		APTarget target = new APTarget(targetPose);
		APResult result = autopilot.calculate(currentPose, currentVelocity, target);
		return result.targetAngle();
	}

	public boolean atTarget(Pose2d currentPose, Pose2d targetPose, double translationTolerance,
			double rotationTolerance) {
		double translationError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		double rotationError = Math
				.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
		return translationError < translationTolerance && rotationError < rotationTolerance;
	}

	public APConstraints getConstraints() {
		return constraints;
	}

	public double getAcceleration() {
		return acceleration;
	}

	public double getJerk() {
		return jerk;
	}
}
