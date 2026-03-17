package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.repulsor.Repulsor;
import frc.robot.repulsor.Setpoints.SetpointUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Fluent builder for autonomous command sequences using Repulsor path planning.
 *
 * <p>
 * Poses are defined as blue-alliance and resolved for the current alliance when the
 * auto starts (not at construction). Uses {@link Commands#defer} so alliance is read at schedule time.
 *
 * <pre>{@code
 * new AutoBuilder(repulsor)
 * 		.driveTo(HUB_FRONT)
 * 		.waitSeconds(0.5)
 * 		.run(shooter.shootCommand())
 * 		.driveToFacing(CENTER_COLLECT, HUB_CENTER)
 * 		.build();
 * }</pre>
 */
public final class AutoBuilder {

	private static final Distance DEFAULT_TOLERANCE = Meters.of(0.15);

	private final Repulsor repulsor;
	private final List<Step> steps = new ArrayList<>();
	private final List<Resolver> resolvers = new ArrayList<>();
	private double speedScale = Constants.AutoConstants.SPEED_SCALE;

	public AutoBuilder(Repulsor repulsor) {
		this.repulsor = repulsor;
	}

	/** Navigate to pose, finishing when within 15cm. */
	public AutoBuilder driveTo(Pose2d bluePose) {
		return driveTo(bluePose, DEFAULT_TOLERANCE);
	}

	/** Navigate to pose, finishing when within the given tolerance. */
	public AutoBuilder driveTo(Pose2d bluePose, Distance tolerance) {
		var ref = refFor(bluePose);
		steps.add(() -> repulsor.navigateTo(ref::get)
				.until(repulsor.within(tolerance)));
		return this;
	}

	/** Navigate to pose, facing the aim target on arrival. Finishes when within 15cm. */
	public AutoBuilder driveToFacing(Pose2d bluePose, Translation2d blueAimTarget) {
		return driveToFacing(bluePose, blueAimTarget, DEFAULT_TOLERANCE, null);
	}

	/** Navigate to pose, facing the aim target on arrival. Custom tolerance. */
	public AutoBuilder driveToFacing(Pose2d bluePose, Translation2d blueAimTarget, Distance tolerance) {
		return driveToFacing(bluePose, blueAimTarget, tolerance, null);
	}

	/**
	 * Navigate to pose, facing the aim target on arrival.
	 *
	 * @param tolerance
	 *          how close to the target before this step finishes
	 * @param headingBlendDist
	 *          distance over which heading blends toward the aim target. Null for default (0.75m).
	 */
	public AutoBuilder driveToFacing(Pose2d bluePose, Translation2d blueAimTarget, Distance tolerance,
			Distance headingBlendDist) {
		var ref = refForFacing(bluePose, blueAimTarget);
		steps.add(() -> {
			if (headingBlendDist != null)
				repulsor.setHeadingBlendDist(headingBlendDist.in(Meters));
			return repulsor.navigateTo(ref::get)
					.until(repulsor.within(tolerance))
					.finallyDo(interrupted -> repulsor.resetHeadingBlendDist());
		});
		return this;
	}

	/** Navigate to pose and hold position indefinitely. */
	public AutoBuilder driveToAndHold(Pose2d bluePose) {
		var ref = refFor(bluePose);
		steps.add(() -> repulsor.navigateTo(ref::get));
		return this;
	}

	/** Navigate to pose, facing the aim target, and hold position indefinitely. */
	public AutoBuilder driveToAndHoldFacing(Pose2d bluePose, Translation2d blueAimTarget) {
		return driveToAndHoldFacing(bluePose, blueAimTarget, null);
	}

	/**
	 * Navigate to pose, facing the aim target, and hold position indefinitely.
	 *
	 * @param headingBlendDist
	 *          distance over which heading blends toward the target. Null for default (0.75m).
	 */
	public AutoBuilder driveToAndHoldFacing(Pose2d bluePose, Translation2d blueAimTarget, Distance headingBlendDist) {
		var ref = refForFacing(bluePose, blueAimTarget);
		steps.add(() -> {
			if (headingBlendDist != null)
				repulsor.setHeadingBlendDist(headingBlendDist.in(Meters));
			return repulsor.navigateTo(ref::get)
					.finallyDo(interrupted -> repulsor.resetHeadingBlendDist());
		});
		return this;
	}

	/** Pause for the given duration. */
	public AutoBuilder waitSeconds(double seconds) {
		steps.add(() -> Commands.waitSeconds(seconds));
		return this;
	}

	/** Scale the Repulsor max speed for the duration of the auto (0.0–1.0). */
	public AutoBuilder speedScale(double scale) {
		this.speedScale = scale;
		return this;
	}

	/** Insert any WPILib command into the sequence. */
	public AutoBuilder run(Command command) {
		steps.add(() -> command);
		return this;
	}

	/** Builds the auto command. Alliance and speed scale are resolved when the command is scheduled. */
	public Command build() {
		var capturedSteps = List.copyOf(steps);
		var capturedResolvers = List.copyOf(resolvers);
		var capturedScale = speedScale;

		return Commands.defer(() -> {
			Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
			for (var resolver : capturedResolvers) {
				resolver.resolve(alliance);
			}

			repulsor.setAutoSpeedScale(capturedScale);

			Command[] commands = new Command[capturedSteps.size()];
			for (int i = 0; i < capturedSteps.size(); i++) {
				commands[i] = capturedSteps.get(i).create();
			}
			return Commands.sequence(commands)
					.finallyDo(interrupted -> repulsor.resetSpeedScale());
		}, Set.of(repulsor.getDrive().asSubsystem()));
	}

	// ===== Internal =====

	private AtomicReference<Pose2d> refFor(Pose2d bluePose) {
		var ref = new AtomicReference<Pose2d>();
		resolvers.add(alliance -> ref.set(flip(bluePose, alliance)));
		return ref;
	}

	private AtomicReference<Pose2d> refForFacing(Pose2d bluePose, Translation2d blueAimTarget) {
		var ref = new AtomicReference<Pose2d>();
		resolvers.add(alliance -> {
			Pose2d flipped = flip(bluePose, alliance);
			Translation2d target = alliance == Alliance.Red
					? SetpointUtil.flipToRed(blueAimTarget)
					: blueAimTarget;
			Rotation2d facing = target.minus(flipped.getTranslation()).getAngle();
			ref.set(new Pose2d(flipped.getTranslation(), facing));
		});
		return ref;
	}

	private static Pose2d flip(Pose2d blue, Alliance alliance) {
		return alliance == Alliance.Red ? SetpointUtil.flipToRed(blue) : blue;
	}

	@FunctionalInterface
	private interface Step {
		Command create();
	}

	@FunctionalInterface
	private interface Resolver {
		void resolve(Alliance alliance);
	}
}
