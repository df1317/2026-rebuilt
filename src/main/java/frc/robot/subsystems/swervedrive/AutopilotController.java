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
 * Wrapper class for Autopilot motion control library.
 *
 * <h2>Overview</h2>
 * <p>
 * Autopilot is a stateless holonomic motion controller integrated as a complementary solution to PathPlanner.
 * It uses jerk-limited motion profiles for smooth deceleration and excels at real-time drive-to-pose scenarios.
 *
 * <h2>When to Use Autopilot vs PathPlanner</h2>
 * <p>
 * <b>Use Autopilot for:</b>
 * <ul>
 * <li><b>Teleop vision alignment:</b> Real-time tracking of AprilTags for scoring positions</li>
 * <li><b>Dynamic targets:</b> Following moving game pieces detected by vision</li>
 * <li><b>Simple point-to-point:</b> Direct navigation without complex waypoints</li>
 * <li><b>Teleop assist:</b> Driver-initiated positioning commands</li>
 * <li><b>Smooth deceleration:</b> Jerk-limited profiles prevent overshoot</li>
 * </ul>
 *
 * <p>
 * <b>Use PathPlanner for:</b>
 * <ul>
 * <li><b>Complex autonomous routines:</b> Multiple waypoints with event markers</li>
 * <li><b>Obstacle avoidance:</b> Built-in pathfinding around dynamic obstacles</li>
 * <li><b>Pre-planned paths:</b> GUI-designed trajectories for consistent execution</li>
 * <li><b>Auto coordination:</b> Timeline-based multi-subsystem choreography</li>
 * </ul>
 *
 * <h2>Configuration</h2>
 * <p>
 * Motion constraints loaded from Constants:
 * <ul>
 * <li><b>Max Velocity:</b> Constants.MAX_SPEED (m/s)</li>
 * <li><b>Max Acceleration:</b> Constants.MAX_ACCELERATION (m/s²)</li>
 * <li><b>Max Jerk:</b> 1.0 m/s³ (hardcoded default, tune based on robot performance)</li>
 * </ul>
 *
 * <p>
 * Error tolerances:
 * <ul>
 * <li><b>Translation:</b> 2cm (0.02m)</li>
 * <li><b>Rotation:</b> 1 degree (π/180 rad)</li>
 * <li><b>Beeline Radius:</b> 10cm (distance to switch from curved to straight approach)</li>
 * </ul>
 *
 * <h2>Rotation Control</h2>
 * <p>
 * This wrapper returns ChassisSpeeds with omega=0. The YAGSL field-relative drive handles rotation separately.
 * For explicit rotation control, use {@link #getTargetHeading(Pose2d, ChassisSpeeds, Pose2d)} with YAGSL's heading controller.
 *
 * <h2>Tuning Recommendations</h2>
 * <ul>
 * <li><b>Start conservative:</b> Use default jerk of 1.0 m/s³</li>
 * <li><b>Increase jerk for faster response:</b> Try 1.5-2.0 m/s³ if deceleration is too slow</li>
 * <li><b>Tighten tolerances for precision:</b> Reduce to 1cm / 0.5° for scoring</li>
 * <li><b>Monitor telemetry:</b> Autopilot constraints shown in SmartDashboard (dev mode only)</li>
 * </ul>
 *
 * <p>
 * <b>Key Limitation:</b> Autopilot does NOT support obstacle avoidance. For navigation through cluttered fields, use PathPlanner.
 *
 * <p>
 * <b>References:</b>
 * <ul>
 * <li><a href="https://therekrab.github.io/autopilot">Autopilot Documentation</a></li>
 * <li><a href="https://therekrab.github.io/autopilot/javadoc">Autopilot JavaDoc</a></li>
 * <li><a href="https://www.chiefdelphi.com/t/autopilot-iri-2025">Team 3414 Case Study - IRI 2025</a></li>
 * </ul>
 *
 * @see SwerveSubsystem#driveToPoseAutopilot(java.util.function.Supplier)
 */
public class AutopilotController {

	private final Autopilot autopilot;
	private final APConstraints constraints;
	private final double acceleration;
	private final double jerk;

	/**
	 * Creates an AutopilotController with default motion constraints from Constants.
	 */
	public AutopilotController() {
		this(
				Constants.MAX_SPEED,
				Constants.MAX_ACCELERATION,
				Constants.MAX_ANGULAR_SPEED,
				Constants.MAX_ANGULAR_ACCELERATION);
	}

	/**
	 * Creates an AutopilotController with custom motion constraints.
	 *
	 * @param maxVelocity
	 *          Maximum linear velocity (m/s)
	 * @param maxAcceleration
	 *          Maximum linear acceleration (m/s²)
	 * @param maxAngularVelocity
	 *          Maximum angular velocity (rad/s)
	 * @param maxAngularAcceleration
	 *          Maximum angular acceleration (rad/s²)
	 */
	public AutopilotController(
			double maxVelocity,
			double maxAcceleration,
			double maxAngularVelocity,
			double maxAngularAcceleration) {
		// Store values for getter access
		this.acceleration = maxAcceleration;
		this.jerk = 1.0;

		// Create constraints with default jerk values
		// Jerk of 1.0 provides smooth deceleration without being overly conservative
		this.constraints = new APConstraints()
				.withAcceleration(maxAcceleration)
				.withJerk(this.jerk);

		// Create profile with reasonable error tolerances
		APProfile profile = new APProfile(constraints)
				.withErrorXY(Centimeters.of(2)) // 2cm position tolerance
				.withErrorTheta(Degrees.of(1)) // 1 degree rotation tolerance
				.withBeelineRadius(Centimeters.of(10)); // 10cm beeline radius

		this.autopilot = new Autopilot(profile);
	}

	/**
	 * Calculate field-relative chassis speeds to reach the target pose.
	 *
	 * @param currentPose
	 *          Current robot pose
	 * @param currentVelocity
	 *          Current robot velocity (robot-relative)
	 * @param targetPose
	 *          Target pose to reach
	 * @return Field-relative ChassisSpeeds command
	 */
	public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds currentVelocity, Pose2d targetPose) {
		APTarget target = new APTarget(targetPose);
		APResult result = autopilot.calculate(currentPose, currentVelocity, target);

		// Convert LinearVelocity to m/s and create ChassisSpeeds
		double vx = result.vx().in(MetersPerSecond);
		double vy = result.vy().in(MetersPerSecond);

		return new ChassisSpeeds(vx, vy, 0.0); // Autopilot handles rotation via heading controller
	}

	/**
	 * Calculate field-relative chassis speeds with entry angle control.
	 *
	 * <p>
	 * Entry angle determines the direction from which the robot approaches the target.
	 * This creates curved paths for smoother approach trajectories.
	 *
	 * @param currentPose
	 *          Current robot pose
	 * @param currentVelocity
	 *          Current robot velocity (robot-relative)
	 * @param targetPose
	 *          Target pose to reach
	 * @param respectEntryAngle
	 *          Whether to respect the target's rotation as entry angle
	 * @return Field-relative ChassisSpeeds command
	 */
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

		// Convert LinearVelocity to m/s and create ChassisSpeeds
		double vx = result.vx().in(MetersPerSecond);
		double vy = result.vy().in(MetersPerSecond);

		return new ChassisSpeeds(vx, vy, 0.0);
	}

	/**
	 * Get the target heading for heading correction.
	 *
	 * <p>
	 * Use this with your swerve drive's heading controller for rotation control.
	 *
	 * @param currentPose
	 *          Current robot pose
	 * @param currentVelocity
	 *          Current robot velocity (robot-relative)
	 * @param targetPose
	 *          Target pose to reach
	 * @return Target heading as Rotation2d
	 */
	public edu.wpi.first.math.geometry.Rotation2d getTargetHeading(
			Pose2d currentPose,
			ChassisSpeeds currentVelocity,
			Pose2d targetPose) {
		APTarget target = new APTarget(targetPose);
		APResult result = autopilot.calculate(currentPose, currentVelocity, target);
		return result.targetAngle();
	}

	/**
	 * Check if the robot is at the target pose within tolerances.
	 *
	 * @param currentPose
	 *          Current robot pose
	 * @param targetPose
	 *          Target pose
	 * @param translationTolerance
	 *          Translation tolerance (meters)
	 * @param rotationTolerance
	 *          Rotation tolerance (radians)
	 * @return True if robot is within tolerance of target
	 */
	public boolean atTarget(Pose2d currentPose, Pose2d targetPose, double translationTolerance,
			double rotationTolerance) {
		double translationError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		double rotationError = Math
				.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

		return translationError < translationTolerance && rotationError < rotationTolerance;
	}

	/**
	 * Gets the current motion constraints.
	 *
	 * @return Current motion constraints
	 */
	public APConstraints getConstraints() {
		return constraints;
	}

	/**
	 * Gets the configured acceleration value.
	 *
	 * @return Acceleration (m/s²)
	 */
	public double getAcceleration() {
		return acceleration;
	}

	/**
	 * Gets the configured jerk value.
	 *
	 * @return Jerk (m/s³)
	 */
	public double getJerk() {
		return jerk;
	}
}
