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
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Setpoints.SetpointUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

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
	/** Y offset from field center for collect positions (meters). */
	private static final double COLLECT_Y_OFFSET = 1.8;
	/** X offset (negative = toward blue alliance wall) for collect positions. */
	private static final double COLLECT_X_OFFSET = -0.5;

	/** Global preamble command run before every auto (e.g. hood homing). */
	private static Supplier<Command> preamble;

	/** Set a command to run at the start of every auto (before any steps). Use Commands.sequence() to chain multiple. */
	public static void setPreamble(Supplier<Command> command) {
		preamble = command;
	}

	private final Repulsor repulsor;
	private final List<Step> steps = new ArrayList<>();
	private final List<Resolver> resolvers = new ArrayList<>();
	private double speedScale = Constants.AutoConstants.SPEED_SCALE;

	public AutoBuilder(Repulsor repulsor) {
		this.repulsor = repulsor;
	}

	/** Returns the current number of steps in the builder. */
	public int stepCount() {
		return steps.size();
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

	/** Navigate to pose, facing the aim target on arrival (front faces target). Finishes when within 15cm. */
	public AutoBuilder driveToFacing(Pose2d bluePose, Translation2d blueAimTarget) {
		return driveToFacing(bluePose, blueAimTarget, 0.0);
	}

	/**
	 * Navigate to pose, facing the aim target on arrival with a rotation offset.
	 *
	 * @param rotationOffsetDeg
	 *          offset from facing angle (0 = front faces target, 180 = back faces target)
	 */
	public AutoBuilder driveToFacing(Pose2d bluePose, Translation2d blueAimTarget, double rotationOffsetDeg) {
		return driveToFacing(
				new Pose2d(bluePose.getTranslation(), Rotation2d.fromDegrees(rotationOffsetDeg)),
				blueAimTarget, DEFAULT_TOLERANCE, null);
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

	/**
	 * Navigate back to wherever the robot was when the auto started.
	 * The pose is captured at schedule time (no alliance flipping).
	 */
	public AutoBuilder driveToStart() {
		return driveToStart(DEFAULT_TOLERANCE);
	}

	/** Navigate back to the starting pose with custom tolerance. */
	public AutoBuilder driveToStart(Distance tolerance) {
		var ref = startPoseRef();
		steps.add(() -> repulsor.navigateTo(ref::get)
				.until(repulsor.within(tolerance)));
		return this;
	}

	/**
	 * Navigate to a collect position on the closest side of the field center line,
	 * based on the robot's starting Y position. Avoids driving all the way to dead center.
	 */
	public AutoBuilder driveToCollect() {
		return driveToCollect(DEFAULT_TOLERANCE);
	}

	/** Navigate to collect with custom tolerance. */
	public AutoBuilder driveToCollect(Distance tolerance) {
		var ref = new AtomicReference<Pose2d>();
		resolvers.add(alliance -> {
			Pose2d startPose = repulsor.getDrive().getPose();
			double fieldCenterX = RepulsorConstants.FIELD_LENGTH / 2.0;
			double fieldCenterY = RepulsorConstants.FIELD_WIDTH / 2.0;
			boolean fromTop = startPose.getY() > fieldCenterY;
			double collectY = fromTop
					? fieldCenterY + COLLECT_Y_OFFSET
					: fieldCenterY - COLLECT_Y_OFFSET;
			double collectDeg = fromTop ? -120.0 : -60.0;
			ref.set(new Pose2d(fieldCenterX + COLLECT_X_OFFSET, collectY, Rotation2d.fromDegrees(collectDeg)));
		});
		steps.add(() -> repulsor.navigateTo(ref::get)
				.until(repulsor.within(tolerance)));
		return this;
	}

	/**
	 * Insert a command into the sequence.
	 * Takes a Supplier so each scheduling creates a fresh instance — WPILib prohibits composing the same command twice.
	 */
	public AutoBuilder run(Supplier<Command> commandSupplier) {
		steps.add(commandSupplier::get);
		return this;
	}

	/**
	 * Run a command alongside the <em>previous</em> drive step. The drive step remains the
	 * deadline &mdash; when it finishes the alongside command is interrupted.
	 *
	 * <p>
	 * Takes a Supplier so each scheduling creates a fresh instance — WPILib prohibits composing the same command twice.
	 *
	 * <pre>{@code
	 * new AutoBuilder(repulsor)
	 * 		.driveTo(COLLECT_POSE)
	 * 		.alongside(intake::extendCommand)
	 * 		.build();
	 * }</pre>
	 *
	 * @throws IllegalStateException
	 *           if there is no previous step to attach to
	 */
	public AutoBuilder alongside(Supplier<Command> commandSupplier) {
		if (steps.isEmpty()) {
			throw new IllegalStateException("alongside() requires a preceding step");
		}
		Step previous = steps.remove(steps.size() - 1);
		steps.add(() -> previous.create().deadlineFor(commandSupplier.get()));
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

			List<Command> commands = new ArrayList<>();
			if (preamble != null) {
				commands.add(preamble.get());
			}
			for (var step : capturedSteps) {
				commands.add(step.create());
			}
			return Commands.sequence(commands.toArray(Command[]::new))
					.finallyDo(interrupted -> repulsor.resetSpeedScale());
		}, Set.of(repulsor.getDrive().asSubsystem()));
	}

	// ===== Internal =====

	private AtomicReference<Pose2d> startPoseRef() {
		var ref = new AtomicReference<Pose2d>();
		resolvers.add(alliance -> ref.set(repulsor.getDrive().getPose()));
		return ref;
	}

	private AtomicReference<Pose2d> refFor(Pose2d bluePose) {
		var ref = new AtomicReference<Pose2d>();
		resolvers.add(alliance -> ref.set(flip(bluePose, alliance)));
		return ref;
	}

	/**
	 * The rotation in {@code bluePose} is used as an offset from the computed facing angle.
	 * 0 = front faces target, 180 = back faces target (e.g. rear-mounted shooter).
	 */
	private AtomicReference<Pose2d> refForFacing(Pose2d bluePose, Translation2d blueAimTarget) {
		var ref = new AtomicReference<Pose2d>();
		resolvers.add(alliance -> {
			Pose2d flipped = flip(bluePose, alliance);
			Translation2d target = alliance == Alliance.Red
					? SetpointUtil.flipToRed(blueAimTarget)
					: blueAimTarget;
			Rotation2d towardTarget = target.minus(flipped.getTranslation()).getAngle();
			Rotation2d facing = towardTarget.rotateBy(bluePose.getRotation());
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
