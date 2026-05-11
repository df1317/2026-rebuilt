package frc.robot.planners;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;

public class GlobalPlannerLayer {

	private static final double WAYPOINT_ADVANCE_RADIUS = 0.3;
	private static final double REPLAN_DEVIATION_THRESHOLD = 0.5;
	private static final double REPLAN_COOLDOWN_SECONDS = 0.25;

	public record GateDefinition(
			Translation2d wallStart,
			Translation2d wallEnd,
			Translation2d bypassPoint) {
	}

	private final ThetaStarPlanner planner;
	private final List<GateDefinition> gates;

	private Pose2d currentGoal = null;
	private List<Translation2d> waypoints = Collections.emptyList();
	private int waypointIndex = 0;
	private double lastReplanTime = -999;
	private Translation2d lastRobotPos = null;

	public GlobalPlannerLayer(ThetaStarGrid grid, List<GateDefinition> gates) {
		this.planner = new ThetaStarPlanner(grid);
		this.gates = gates;
	}

	public void setGoal(Pose2d goal) {
		if (currentGoal == null
				|| goal.getTranslation().getDistance(currentGoal.getTranslation()) > 0.05) {
			currentGoal = goal;
			waypoints = Collections.emptyList();
			waypointIndex = 0;
			lastReplanTime = -999;
		}
	}

	public void clearGoal() {
		currentGoal = null;
		waypoints = Collections.emptyList();
	}

	public boolean hasGoal() {
		return currentGoal != null;
	}

	public List<Translation2d> getWaypoints() {
		return waypoints;
	}

	/**
	 * Call every periodic. Returns the Pose2d to pass to goalManager.setRequestedGoal().
	 */
	public Pose2d update(Pose2d currentPose) {
		if (currentGoal == null)
			return null;

		Translation2d pos = currentPose.getTranslation();
		lastRobotPos = pos;
		double now = Timer.getFPGATimestamp();

		boolean needsReplan = waypoints.isEmpty()
				|| (now - lastReplanTime > REPLAN_COOLDOWN_SECONDS && isDeviating(pos));

		if (needsReplan)
			replan(pos, now);

		if (waypoints.isEmpty())
			return currentGoal;

		while (waypointIndex < waypoints.size() - 1
				&& pos.getDistance(waypoints.get(waypointIndex)) < WAYPOINT_ADVANCE_RADIUS) {
			waypointIndex++;
		}

		Translation2d next = waypoints.get(waypointIndex);
		boolean isFinal = waypointIndex == waypoints.size() - 1;
		Rotation2d rot = isFinal ? currentGoal.getRotation() : currentPose.getRotation();
		return new Pose2d(next, rot);
	}

	// ── private ───────────────────────────────────────────────────────

	private void replan(Translation2d pos, double now) {
		lastReplanTime = now; // Set immediately so failures don't spam the CPU
		List<Translation2d> path = planWithGates(pos, currentGoal.getTranslation());
		if (!path.isEmpty()) {
			waypoints = path;
			waypointIndex = 0;
		}
	}

	/**
	 * Plans through gate bypass points when the direct path crosses a gate wall.
	 * Supports chaining — if both start→bypass and bypass→goal cross further gates,
	 * those are handled recursively (capped at 3 hops to avoid cycles).
	 */
	private List<Translation2d> planWithGates(Translation2d start, Translation2d goal) {
		return planWithGates(start, goal, 0);
	}

	private List<Translation2d> planWithGates(Translation2d start, Translation2d goal, int depth) {
		if (depth > 3)
			return planner.plan(start, goal); // safety cap

		GateDefinition crossed = findCrossedGate(start, goal);
		if (crossed == null)
			return planner.plan(start, goal);

		Translation2d bp = crossed.bypassPoint();
		List<Translation2d> seg1 = planWithGates(start, bp, depth + 1);
		List<Translation2d> seg2 = planWithGates(bp, goal, depth + 1);

		if (seg1.isEmpty() || seg2.isEmpty())
			return Collections.emptyList();

		List<Translation2d> full = new ArrayList<>(seg1);
		full.addAll(seg2);
		return full;
	}

	private GateDefinition findCrossedGate(Translation2d a, Translation2d b) {
		GateDefinition bestGate = null;
		double minDistance = Double.MAX_VALUE;
		for (GateDefinition gate : gates) {
			if (segmentsIntersect(a, b, gate.wallStart(), gate.wallEnd())) {
				double distToBypass = a.getDistance(gate.bypassPoint());
				if (distToBypass < minDistance) {
					minDistance = distToBypass;
					bestGate = gate;
				}
			}
		}
		return bestGate;
	}

	private boolean segmentsIntersect(Translation2d p1, Translation2d p2,
			Translation2d p3, Translation2d p4) {
		double d1x = p2.getX() - p1.getX(), d1y = p2.getY() - p1.getY();
		double d2x = p4.getX() - p3.getX(), d2y = p4.getY() - p3.getY();
		double cross = d1x * d2y - d1y * d2x;
		if (Math.abs(cross) < 1e-10)
			return false;
		double dx = p3.getX() - p1.getX(), dy = p3.getY() - p1.getY();
		double t = (dx * d2y - dy * d2x) / cross;
		double u = (dx * d1y - dy * d1x) / cross;
		return t >= 0 && t <= 1 && u >= 0 && u <= 1;
	}

	private boolean isDeviating(Translation2d pos) {
		if (waypoints.isEmpty())
			return false;
		return pos.getDistance(waypoints.get(waypointIndex)) > REPLAN_DEVIATION_THRESHOLD;
	}
}
