package frc.robot.planners;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;

public class ThetaStarPlanner {

	private final ThetaStarGrid grid;

	public ThetaStarPlanner(ThetaStarGrid grid) {
		this.grid = grid;
	}

	public List<Translation2d> plan(Translation2d start, Translation2d goal) {
		int sc = grid.toCol(start.getX()), sr = grid.toRow(start.getY());
		int gc = grid.toCol(goal.getX()), gr = grid.toRow(goal.getY());
		if (grid.hasLOS(sc, sr, gc, gr))
			return List.of(goal);
		return thetaStar(sc, sr, gc, gr, goal);
	}

	private static final int[] DC = { -1, -1, -1, 0, 0, 1, 1, 1 };
	private static final int[] DR = { -1, 0, 1, -1, 1, -1, 0, 1 };
	private static final double[] COST = {
			Math.sqrt(2), 1, Math.sqrt(2), 1, 1, Math.sqrt(2), 1, Math.sqrt(2)
	};

	private List<Translation2d> thetaStar(int sc, int sr, int gc, int gr, Translation2d goalMetric) {
		int size = grid.cols * grid.rows;
		float[] g = new float[size];
		int[] parentC = new int[size];
		int[] parentR = new int[size];
		boolean[] closed = new boolean[size];

		Arrays.fill(g, Float.MAX_VALUE);
		Arrays.fill(parentC, -1);

		int startIdx = idx(sc, sr);
		g[startIdx] = 0;
		parentC[startIdx] = sc;
		parentR[startIdx] = sr;

		PriorityQueue<int[]> open = new PriorityQueue<>(Comparator.comparingInt(a -> a[0]));
		open.offer(new int[] { 0, sc, sr });

		while (!open.isEmpty()) {
			int[] cur = open.poll();
			int c = cur[1], r = cur[2];
			int ci = idx(c, r);
			if (closed[ci])
				continue;
			closed[ci] = true;

			if (c == gc && r == gr)
				return reconstructPath(gc, gr, sc, sr, goalMetric, parentC, parentR);

			for (int i = 0; i < 8; i++) {
				int nc = c + DC[i], nr = r + DR[i];
				if (!grid.inBounds(nc, nr) || grid.isOccupied(nc, nr))
					continue;
				int ni = idx(nc, nr);
				if (closed[ni])
					continue;

				int bestPC, bestPR;
				float gNew;

				if (grid.hasLOS(parentC[ci], parentR[ci], nc, nr)) {
					bestPC = parentC[ci];
					bestPR = parentR[ci];
					gNew = g[idx(parentC[ci], parentR[ci])] + dist(parentC[ci], parentR[ci], nc, nr);
				} else {
					bestPC = c;
					bestPR = r;
					gNew = g[ci] + (float) (COST[i] * ThetaStarGrid.RESOLUTION);
				}

				if (gNew < g[ni]) {
					g[ni] = gNew;
					parentC[ni] = bestPC;
					parentR[ni] = bestPR;
					float h = dist(nc, nr, gc, gr);
					open.offer(new int[] { (int) ((gNew + h) * 1000), nc, nr });
				}
			}
		}
		return Collections.emptyList();
	}

	private List<Translation2d> reconstructPath(
			int gc, int gr, int sc, int sr,
			Translation2d goalMetric,
			int[] parentC, int[] parentR) {

		List<Translation2d> path = new ArrayList<>();
		path.add(goalMetric);
		int c = gc, r = gr;
		while (c != sc || r != sr) {
			int pi = idx(c, r);
			int pc = parentC[pi], pr = parentR[pi];
			if (pc == c && pr == r)
				break;
			c = pc;
			r = pr;
			if (c != sc || r != sr)
				path.add(new Translation2d(grid.toX(c), grid.toY(r)));
		}
		Collections.reverse(path);
		return path;
	}

	private int idx(int c, int r) {
		return r * grid.cols + c;
	}

	private float dist(int c0, int r0, int c1, int r1) {
		double dx = (c1 - c0) * ThetaStarGrid.RESOLUTION;
		double dy = (r1 - r0) * ThetaStarGrid.RESOLUTION;
		return (float) Math.sqrt(dx * dx + dy * dy);
	}
}
