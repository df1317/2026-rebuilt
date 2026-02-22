package frc.robot.repulsor.Behaviours;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.FieldPlanner.RepulsorSample;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.MutablePoseSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.repulsor.Setpoints.SetpointType;
import frc.robot.repulsor.Setpoints.SetpointUtil;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import frc.robot.repulsor.Tracking.FieldTrackerCore;

public class ShuttleBehaviour extends Behaviour {
	private final int prio;
	private final Supplier<Boolean> hasPiece;
	private final Supplier<Double> ourSpeedCap;
	private Pose2d lastCollectBluePose = null;

	private final AtomicLong pieceCount = new AtomicLong(0L);

	public ShuttleBehaviour(int priority, Supplier<Boolean> hasPiece, Supplier<Double> ourSpeedCap) {
		this.prio = priority;
		this.hasPiece = hasPiece;
		this.ourSpeedCap = ourSpeedCap;
	}

	@Override
	public String name() {
		return "Shuttle";
	}

	@Override
	public int priority() {
		return prio;
	}

	@Override
	public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
		return flags.contains(BehaviourFlag.SHUTTLE_MODE);
	}

	private static boolean nearPose(Pose2d a, Pose2d b, double posTol, double degTol) {
		if (a.getTranslation().getDistance(b.getTranslation()) > posTol)
			return false;
		double e = Math.abs(MathUtil.angleModulus(
				a.getRotation().getRadians() - b.getRotation().getRadians()));
		return e <= Math.toRadians(degTol);
	}

	private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
		double release;
		try {
			var ht = ctx.repulsor.getTargetHeight();
			var d = ht != null ? ht.getHeight() : null;
			release = d != null ? Math.max(0.0, d.in(Meters)) : 0.0;
		} catch (Exception ignored) {
			release = 0.0;
		}
		return new SetpointContext(
				Optional.of(robotPose),
				Math.max(0.0, ctx.robot_x) * 2.0,
				Math.max(0.0, ctx.robot_y) * 2.0,
				release,
				ctx.vision.getObstacles());
	}

	@Override
	public Command build(BehaviourContext ctx) {
		AtomicReference<RepulsorSetpoint> lastActive = new AtomicReference<>(null);
		AtomicReference<CategorySpec> lastCat = new AtomicReference<>(null);

		AtomicReference<RepulsorSetpoint> lastEpisodeGoal = new AtomicReference<>(null);
		AtomicLong lastEpisodeFinalizeNs = new AtomicLong(0L);

		final long EP_COOLDOWN_NS = 1_000_000_000L;
		final long STUCK_FAIL_NS = 3_000_000_000L;
		final double PROGRESS_EPS_METERS = 0.03;
		final double STUCK_DIST_MIN_METERS = 0.5;
		final double SUCCESS_NEAR_DIST_METERS = 0.40;
		final int COLLECT_GOAL_UNITS = 2;

		AtomicLong episodeStartNs = new AtomicLong(0L);
		AtomicReference<Double> episodeBestDist = new AtomicReference<>(null);
		AtomicLong lastProgressNs = new AtomicLong(0L);
		AtomicBoolean episodeEverNearGoal = new AtomicBoolean(false);

		AtomicReference<Pose2d> collectBluePoseRef = new AtomicReference<>(Pose2d.kZero);
		RepulsorSetpoint collectRoute = new RepulsorSetpoint(
				new MutablePoseSetpoint("COLLECT_ROUTE", SetpointType.kOther, collectBluePoseRef),
				HeightSetpoint.NONE);

		Consumer<Boolean> finalizeEpisode = forceSuccess -> {
			if (lastEpisodeGoal.get() == null)
				return;
			long now = System.nanoTime();
			long last = lastEpisodeFinalizeNs.get();
			if (last != 0L && now - last < EP_COOLDOWN_NS)
				return;

			boolean success = forceSuccess || episodeEverNearGoal.get();
			if (ctx.planner.bypass != null) {
				ctx.planner.bypass.finalizeEpisode(success);
			}
			lastEpisodeFinalizeNs.set(now);
			episodeStartNs.set(0L);
			episodeBestDist.set(null);
			lastProgressNs.set(0L);
			episodeEverNearGoal.set(false);
		};

		return Commands.run(
				() -> {
					Pose2d robotPose = ctx.robotPose.get();
					boolean piece = hasPiece.get();
					double cap = ourSpeedCap != null ? Math.max(0.25, ourSpeedCap.get()) : 3.5;

					CategorySpec cat;
					RepulsorSetpoint sp;

					if (piece) {
						// Have piece -> go score at nearest scoring pose
						cat = CategorySpec.kScore;
						var nearest = _Rebuilt2026.nearestScoringPose(robotPose.getTranslation());
						sp = new RepulsorSetpoint(nearest, HeightSetpoint.NET);
					} else {
						// No piece -> go collect
						cat = CategorySpec.kCollect;
						sp = chooseCollect(ctx, robotPose, cap, COLLECT_GOAL_UNITS,
								collectBluePoseRef, collectRoute);
					}

					CategorySpec prevCat = lastCat.get();
					if (prevCat != null && prevCat != cat) {
						if (lastEpisodeGoal.get() != null) {
							finalizeEpisode.accept(false);
						}
						ctx.planner.clearCommitted();
						lastActive.set(null);
					}
					lastCat.set(cat);

					lastActive.set(sp);
					lastEpisodeGoal.set(sp);

					Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));
					ctx.repulsor.setCurrentGoal(sp);
					ctx.planner.setRequestedGoal(goalPose);

					double distToGoal = robotPose.getTranslation().getDistance(goalPose.getTranslation());
					long nowNs = System.nanoTime();

					if (episodeStartNs.get() == 0L) {
						episodeStartNs.set(nowNs);
						episodeBestDist.set(distToGoal);
						lastProgressNs.set(nowNs);
					} else {
						Double best = episodeBestDist.get();
						if (best == null || distToGoal < best - PROGRESS_EPS_METERS) {
							episodeBestDist.set(distToGoal);
							lastProgressNs.set(nowNs);
						}
					}

					if (distToGoal <= SUCCESS_NEAR_DIST_METERS) {
						episodeEverNearGoal.set(true);
					}

					RepulsorSample sample = ctx.planner.calculate(
							robotPose,
							ctx.vision.getObstacles(),
							ctx.robot_x,
							ctx.robot_y,
							cat,
							false,
							0.0);

					// Stuck detection
					if (lastEpisodeGoal.get() != null && sp == lastEpisodeGoal.get()) {
						long lastProg = lastProgressNs.get();
						if (lastProg != 0L) {
							long sinceProgressNs = nowNs - lastProg;
							if (sinceProgressNs >= STUCK_FAIL_NS && distToGoal > STUCK_DIST_MIN_METERS) {
								episodeEverNearGoal.set(false);
								finalizeEpisode.accept(false);
							}
						}
					}

					ctx.drive.runVelocity(
							sample.asChassisSpeeds(
									ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation()));
				},
				ctx.drive.asSubsystem())
				.finallyDo(interrupted -> {
					if (lastEpisodeGoal.get() != null) {
						finalizeEpisode.accept(!interrupted);
					}
					ctx.drive.runVelocity(new ChassisSpeeds());
				});
	}

	private RepulsorSetpoint chooseCollect(
			BehaviourContext ctx,
			Pose2d robotPose,
			double cap,
			int goalUnits,
			AtomicReference<Pose2d> collectBluePoseRef,
			RepulsorSetpoint collectRoute) {

		DriverStation.Alliance wpA = DriverStation.getAlliance()
				.orElse(DriverStation.Alliance.Blue);

		Pose2d robotPoseBlue = wpA == DriverStation.Alliance.Red
				? SetpointUtil.flipToRed(robotPose)
				: robotPose;

		Pose2d nextBlue = FieldTrackerCore.getInstance()
				.nextCollectionGoalBlue(robotPoseBlue, cap, goalUnits);

		if (nextBlue == null) {
			nextBlue = new Pose2d(
					RepulsorConstants.FIELD_LENGTH * 0.5,
					RepulsorConstants.FIELD_WIDTH * 0.5,
					robotPoseBlue.getRotation());
		}

		nextBlue = new Pose2d(nextBlue.getTranslation(), nextBlue.getRotation());
		lastCollectBluePose = nextBlue;
		collectBluePoseRef.set(nextBlue);

		return collectRoute;
	}
}
