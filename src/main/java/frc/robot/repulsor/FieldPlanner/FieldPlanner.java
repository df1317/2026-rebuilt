/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package frc.robot.repulsor.FieldPlanner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import dev.doglog.DogLog;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.ExtraPathing;
import frc.robot.repulsor.Fallback.PlannerFallback;
import frc.robot.repulsor.FieldPlanner.Helpers.FieldPlannerForceModel;
import frc.robot.repulsor.FieldPlanner.Helpers.FieldPlannerGeometry;
import frc.robot.repulsor.FieldPlanner.Helpers.FieldPlannerGoalManager;
import frc.robot.repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Force;
import frc.robot.repulsor.HeadingGate;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.repulsor.Tuning.DefaultDriveTuning;
import frc.robot.repulsor.Tuning.DefaultTurnTuning;
import frc.robot.repulsor.Tuning.DriveTuning;
import frc.robot.repulsor.Tuning.TurnTuning;

public class FieldPlanner {
	private static final double FORCE_THROUGH_GOAL_DIST = 2.0;
	private static final double FORCE_THROUGH_WALL_DIST = 0.7;
	public static final double GOAL_STRENGTH = 2.2;
	private static final double DEFAULT_HEADING_BLEND_DIST = 0.75;

	// P-APF predictive lookahead parameters
	private static final double PAPF_HORIZON = 8.0; // meters
	private static final double PAPF_RESOLUTION = 0.25; // meters

	private double headingBlendDist = DEFAULT_HEADING_BLEND_DIST;
	private Rotation2d headingOffset = null;

	private static final class ClearMemo {
		Boolean toGoalDyn;
		Boolean toGoalNoDyn;

		boolean toGoalDyn(
				Translation2d a, Translation2d b, List<? extends Obstacle> dyn, double rx, double ry) {
			if (toGoalDyn != null)
				return toGoalDyn.booleanValue();
			toGoalDyn = ExtraPathing.isClearPath("Repulsor/IsClear", a, b, dyn, rx, ry, true);
			return toGoalDyn.booleanValue();
		}

		boolean toGoalNoDyn(Translation2d a, Translation2d b, double rx, double ry) {
			if (toGoalNoDyn != null)
				return toGoalNoDyn.booleanValue();
			toGoalNoDyn = ExtraPathing.isClearPath(
					"Repulsor/ForceThrough/NoDyn", a, b, Collections.emptyList(), rx, ry, false);
			return toGoalNoDyn.booleanValue();
		}
	}

	public interface ObstacleProvider {
		List<Obstacle> fieldObstacles();

		List<Obstacle> walls();
	}

	public static final class DefaultObstacleProvider implements ObstacleProvider {
		@Override
		public List<Obstacle> fieldObstacles() {
			return List.of();
		}

		@Override
		public List<Obstacle> walls() {
			return List.of();
		}
	}

	private Optional<RepulsorSetpoint> lastChosenSetpoint = Optional.empty();

	private final TurnTuning turnTuning;
	private final DriveTuning driveTuning;
	private final HeadingGate headingGate = new HeadingGate();

	private final ObstacleProvider obstacleProvider;
	private final List<Obstacle> fieldObstacles;
	private final List<Obstacle> walls;
	private final List<GatedAttractorObstacle> gatedAttractors = new ArrayList<>();

	private final FieldPlannerForceModel forceModel;
	private final FieldPlannerGoalManager goalManager;

	private Optional<Distance> currentErr = Optional.empty();
	private Optional<PlannerFallback> fallback = Optional.empty();

	public boolean suppressIsClearPath = false;
	private int stuckStepCount = 0;
	private static final int MAX_STUCK_STEPS = 40;

	public ObstacleProvider getObstacleProvider() {
		return obstacleProvider;
	}

	private Pose2d[] lastTrajectory = new Pose2d[0];

	public Pose2d[] getLastTrajectory() {
		return lastTrajectory;
	}

	public FieldPlanner() {
		this(new DefaultTurnTuning(), new DefaultDriveTuning(), RepulsorConstants.FIELD);
	}

	public FieldPlanner(ObstacleProvider obstacleProvider, DriveTuning driveTuning) {
		this(new DefaultTurnTuning(), driveTuning, obstacleProvider);
	}

	public FieldPlanner(TurnTuning turnTuning, DriveTuning driveTuning) {
		this(turnTuning, driveTuning, new DefaultObstacleProvider());
	}

	public FieldPlanner(
			TurnTuning turnTuning, DriveTuning driveTuning, ObstacleProvider obstacleProvider) {
		this.turnTuning = turnTuning;
		this.driveTuning = driveTuning;
		this.obstacleProvider = obstacleProvider == null ? new DefaultObstacleProvider() : obstacleProvider;
		this.fieldObstacles = new ArrayList<>(this.obstacleProvider.fieldObstacles());
		this.walls = new ArrayList<>(this.obstacleProvider.walls());

		for (Obstacle obs : this.fieldObstacles) {
			if (obs instanceof GatedAttractorObstacle gated) {
				if (gated.waypoint) {
					gatedAttractors.add(gated);
				}
			}
		}

		this.forceModel = new FieldPlannerForceModel(fieldObstacles, walls);
		this.goalManager = new FieldPlannerGoalManager(gatedAttractors);
	}

	public static boolean segmentIntersectsPolygonOuter(
			Translation2d a, Translation2d b, Translation2d[] poly) {
		return FieldPlannerGeometry.segmentIntersectsPolygonOuter(a, b, poly);
	}

	public static Translation2d[] robotRect(
			Translation2d center, Rotation2d yaw, double rx, double ry) {
		return TurnTuning.robotRect(center, yaw, rx, ry);
	}

	private boolean rectIntersectsDynamic(Translation2d[] rect, List<? extends Obstacle> dynamics) {
		for (Obstacle d : dynamics)
			if (d.intersectsRectangle(rect))
				return true;
		return false;
	}

	private boolean rectIntersectsAny(Translation2d[] rect, List<? extends Obstacle> dynamics) {
		for (Obstacle w : walls)
			if (w.intersectsRectangle(rect))
				return true;
		for (Obstacle f : fieldObstacles)
			if (f.intersectsRectangle(rect))
				return true;
		for (Obstacle d : dynamics)
			if (d.intersectsRectangle(rect))
				return true;
		return false;
	}

	public List<Obstacle> getObstacles() {
		return fieldObstacles;
	}

	public Translation2d getGoal() {
		return goalManager.getGoalTranslation();
	}

	public FieldPlanner withFallback(PlannerFallback _fallback) {
		fallback = Optional.of(_fallback);
		return this;
	}

	public void updateArrows(List<? extends Obstacle> dynamicObstacles) {
		forceModel.updateArrows(goalManager.getGoalTranslation(), dynamicObstacles);
	}

	public ArrayList<Pose2d> getArrows() {
		return forceModel.getArrows();
	}

	public Force getGoalForce(Translation2d curLocation, Translation2d goal) {
		return forceModel.getGoalForce(curLocation, goal);
	}

	public Force getWallForce(Translation2d curLocation, Translation2d target) {
		return forceModel.getWallForce(curLocation, target);
	}

	public Force getObstacleForce(
			Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
		return forceModel.getObstacleForce(curLocation, target, extra);
	}

	public Force getObstacleForce(Translation2d curLocation, Translation2d target) {
		return forceModel.getObstacleForce(curLocation, target);
	}

	public Force getForce(Translation2d curLocation, Translation2d target) {
		return forceModel.getForce(curLocation, target);
	}

	public void setHeadingBlendDist(double meters) {
		this.headingBlendDist = meters;
	}

	public void resetHeadingBlendDist() {
		this.headingBlendDist = DEFAULT_HEADING_BLEND_DIST;
	}

	public void setRequestedGoal(Pose2d requested) {
		goalManager.setRequestedGoal(requested);
		lastChosenSetpoint = Optional.empty();
		stuckStepCount = 0;
		headingOffset = null;
	}

	void setActiveGoal(Pose2d active) {
		goalManager.setActiveGoal(active);
	}

	public Optional<Distance> getErr() {
		return currentErr;
	}

	public void clearCommitted() {
	}

	public RepulsorSample calculateAndClear(
			Pose2d pose,
			List<? extends Obstacle> dynamicObstacles,
			double robot_x,
			double robot_y,
			CategorySpec cat) {
		return calculate(
				pose, dynamicObstacles, robot_x, robot_y, cat, false);
	}

	public Translation2d getForceTarget(Pose2d pose, List<? extends Obstacle> dynamicObstacles) {
		Pose2d effectiveGoal = goalManager.getGoalPose();
		Translation2d curTrans = pose.getTranslation();
		Translation2d forceTarget = effectiveGoal.getTranslation();

		ArrayList<Pose2d> traj = new ArrayList<>();
		traj.add(pose);

		double e_x = forceTarget.getX() - curTrans.getX();
		double e_y = forceTarget.getY() - curTrans.getY();
		double error = Math.hypot(e_x, e_y);

		if (error > PAPF_RESOLUTION) {
			double simX = curTrans.getX();
			double simY = curTrans.getY();
			double dMax = 0.0;
			double seg_c = forceTarget.getX() * curTrans.getY()
					- forceTarget.getY() * curTrans.getX();

			// We use a fixed prediction horizon of ~24 iterations to get a good lookahead
			// without consuming too many CPU cycles.
			int maxSteps = 24;
			for (int i = 0; i < maxSteps; i++) {
				Translation2d simPos = new Translation2d(simX, simY);
				Force force = forceModel.getGoalForce(simPos, forceTarget)
						.plus(forceModel.getObstacleForce(simPos, forceTarget, dynamicObstacles))
						.plus(forceModel.getWallForce(simPos, forceTarget));
				double norm = force.getNorm();
				if (norm < 1e-6)
					break;

				double alpha = PAPF_RESOLUTION / norm;
				simX += force.getX() * alpha;
				simY += force.getY() * alpha;

				traj.add(new Pose2d(simX, simY, force.getAngle()));

				// Perpendicular distance from simulated point to line(robot -> goal)
				double d = Math.abs(e_y * simX - e_x * simY + seg_c) / error;
				if (d > PAPF_RESOLUTION && d >= dMax) {
					forceTarget = new Translation2d(simX, simY);
					dMax = d;
				}

				double remainX = effectiveGoal.getTranslation().getX() - simX;
				double remainY = effectiveGoal.getTranslation().getY() - simY;
				if (remainX * remainX + remainY * remainY <= PAPF_RESOLUTION * PAPF_RESOLUTION) {
					// We've reached the target, let's reverse the trajectory to go back 24 points
					int pointsToReverse = Math.min(24, traj.size());
					for (int j = 0; j < pointsToReverse; j++) {
						Pose2d p = traj.get(traj.size() - 1 - j);
						traj.add(new Pose2d(p.getTranslation(), p.getRotation().plus(Rotation2d.k180deg)));
					}
					break;
				}
			}
		}

		traj.add(effectiveGoal);
		lastTrajectory = traj.toArray(Pose2d[]::new);

		return forceTarget;
	}

	public RepulsorSample calculate(
			Pose2d pose,
			List<? extends Obstacle> dynamicObstacles,
			double robot_x,
			double robot_y,
			CategorySpec cat,
			boolean suppressFallback) {

		Translation2d curTrans = pose.getTranslation();
		double distToGoal = curTrans.getDistance(goalManager.getGoalTranslation());

		boolean slowDown = goalManager.updateStagedGoal(curTrans, dynamicObstacles);
		distToGoal = curTrans.getDistance(goalManager.getGoalTranslation());

		ClearMemo memo = new ClearMemo();

		boolean forceThrough = false;
		List<? extends Obstacle> effectiveDynamics = dynamicObstacles;

		if (!forceThrough && !suppressFallback) {
			boolean blockedWithDynamics = !ExtraPathing.isClearPath(
					"Repulsor/ForceThrough/WithDyn",
					curTrans,
					goalManager.getGoalTranslation(),
					dynamicObstacles,
					robot_x,
					robot_y,
					false);

			boolean blockedWithoutDynamics = !memo.toGoalNoDyn(curTrans, goalManager.getGoalTranslation(), robot_x, robot_y);

			double dxWall = Math.min(curTrans.getX(), RepulsorConstants.FIELD_LENGTH - curTrans.getX());
			double dyWall = Math.min(curTrans.getY(), RepulsorConstants.FIELD_WIDTH - curTrans.getY());
			double dWall = Math.min(dxWall, dyWall);
			boolean nearWall = dWall < FORCE_THROUGH_WALL_DIST;
			boolean nearGoal = distToGoal <= FORCE_THROUGH_GOAL_DIST;

			if (blockedWithDynamics && !blockedWithoutDynamics && nearGoal && nearWall) {
				forceThrough = true;
				effectiveDynamics = Collections.emptyList();
			}
		}

		if (!suppressFallback) {
			if (!forceThrough
					&& ExtraPathing.robotIntersects(curTrans, robot_x, robot_y, dynamicObstacles)) {
				currentErr = Optional.of(Meters.of(curTrans.getDistance(goalManager.getGoalTranslation())));
				return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
			}

			boolean pathBlocked = false;
			if (!suppressIsClearPath) {
				pathBlocked = !memo.toGoalDyn(
						curTrans, goalManager.getGoalTranslation(), effectiveDynamics, robot_x, robot_y);
			}

			if (pathBlocked && !suppressFallback) {
				var cands = Collections.<RepulsorSetpoint>emptyList();

				SetpointContext spCtx = new SetpointContext(
						Optional.of(pose),
						Math.max(0.0, robot_x) * 2.0,
						Math.max(0.0, robot_y) * 2.0,
						effectiveDynamics);

				for (RepulsorSetpoint sp : cands) {
					Pose2d altGoal = sp.get(spCtx);

					if (altGoal.getTranslation().getDistance(goalManager.getGoalTranslation()) < 1e-3)
						continue;

					boolean clear = ExtraPathing.isClearPath(
							"Repulsor/IsClear/Reroute",
							curTrans,
							altGoal.getTranslation(),
							effectiveDynamics,
							robot_x,
							robot_y,
							true);

					if (clear) {
						setActiveGoal(altGoal);
						lastChosenSetpoint = Optional.of(sp);
						pathBlocked = false;
						break;
					}
				}

				if (pathBlocked) {
					return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
				}
			}
		}

		final List<? extends Obstacle> effectiveDynamicsFinal = effectiveDynamics;

		updateArrows(effectiveDynamicsFinal);

		var err = curTrans.minus(goalManager.getGoalTranslation());
		currentErr = Optional.of(Meters.of(err.getNorm()));

		if (err.getNorm() < 0.04) {
			return new RepulsorSample(
					curTrans, 0, 0, Radians.of(goalManager.getGoalPose().getRotation().getRadians()));
		}

		if (fallback.isPresent() && fallback.get().within(err)) {
			var speeds = fallback.get().calculate(curTrans, goalManager.getGoalTranslation());
			return new RepulsorSample(
					goalManager.getGoalTranslation(), speeds, Radians.of(pose.getRotation().getRadians()));
		}

		Pose2d effectiveGoal = goalManager.getGoalPose();

		// P-APF: Simulate forward along the force field to find an intermediate
		// setpoint that smooths the path around obstacles. The setpoint is the
		// point along the predicted path that deviates most from the straight
		// line to the goal.
		Translation2d forceTarget = getForceTarget(pose, effectiveDynamicsFinal);

		var obstacleForce = getObstacleForce(curTrans, forceTarget, effectiveDynamicsFinal)
				.plus(getWallForce(curTrans, forceTarget));
		var netForce = getGoalForce(curTrans, forceTarget).plus(obstacleForce);
		var dist = curTrans.getDistance(effectiveGoal.getTranslation());

		double stepSize_m = driveTuning.stepSizeMeters(
				dist, obstacleForce.getNorm(), (cat == CategorySpec.kScore), slowDown);
		var step = new Translation2d(stepSize_m, netForce.getAngle());

		if (step.getNorm() < 1e-3) {
			stuckStepCount++;
		} else {
			stuckStepCount = 0;
		}

		if (stuckStepCount >= MAX_STUCK_STEPS) {
			DogLog.log("Repulsor/Stuck", true);
			return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
		}

		Rotation2d desiredHeadingRaw;
		if (cat == CategorySpec.kCollect) {
			desiredHeadingRaw = effectiveGoal.getRotation();
		} else {
			// Compute a 90°-quantized offset on first call so the robot picks the
			// closest side (front/back/left/right) to face the travel direction
			// and holds it through tight spaces.
			if (headingOffset == null) {
				double diff = pose.getRotation().minus(netForce.getAngle()).getDegrees();
				double snapped = Math.round(diff / 90.0) * 90.0;
				headingOffset = Rotation2d.fromDegrees(snapped);
			}
			Rotation2d travelHeading = netForce.getAngle().plus(headingOffset);
			double t = MathUtil.clamp(1.0 - dist / headingBlendDist, 0.0, 1.0);
			desiredHeadingRaw = travelHeading.interpolate(effectiveGoal.getRotation(), t);
		}
		Rotation2d desiredHeading = headingGate.filter(pose.getRotation(), desiredHeadingRaw, driveTuning.dtSeconds());

		var turn = turnTuning.plan(
				pose,
				effectiveGoal,
				desiredHeading,
				step,
				(cat == CategorySpec.kScore),
				robot_x,
				robot_y,
				rect -> rectIntersectsAny(rect, effectiveDynamicsFinal));

		step = step.times(turn.speedScale);

		return new RepulsorSample(
				effectiveGoal.getTranslation(),
				step.getX() / driveTuning.dtSeconds(),
				step.getY() / driveTuning.dtSeconds(),
				Radians.of(turn.yaw.getRadians()));
	}

	public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
		return FieldPlannerGeometry.isPointInPolygon(point, polygon);
	}

	public static double dot(Translation2d a, Translation2d b) {
		return FieldPlannerGeometry.dot(a, b);
	}

	public static double distanceFromPointToSegment(
			Translation2d p, Translation2d a, Translation2d b) {
		return FieldPlannerGeometry.distanceFromPointToSegment(p, a, b);
	}

	public Optional<RepulsorSetpoint> pollChosenSetpoint() {
		var out = lastChosenSetpoint;
		lastChosenSetpoint = Optional.empty();
		return out;
	}
}
