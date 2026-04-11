package frc.robot.planners;

import edu.wpi.first.math.geometry.Translation2d;

public class ThetaStarGrid {

	public static final double RESOLUTION = 0.15;
	public static final double FIELD_WIDTH = 16.46;
	public static final double FIELD_HEIGHT = 8.23;

	public final int cols;
	public final int rows;
	private final boolean[] occupied;

	public ThetaStarGrid() {
		cols = (int) Math.ceil(FIELD_WIDTH / RESOLUTION) + 1;
		rows = (int) Math.ceil(FIELD_HEIGHT / RESOLUTION) + 1;
		occupied = new boolean[cols * rows];
	}

	public int toCol(double x) {
		return (int) Math.round(x / RESOLUTION);
	}

	public int toRow(double y) {
		return (int) Math.round(y / RESOLUTION);
	}

	public double toX(int col) {
		return col * RESOLUTION;
	}

	public double toY(int row) {
		return row * RESOLUTION;
	}

	public boolean inBounds(int col, int row) {
		return col >= 0 && col < cols && row >= 0 && row < rows;
	}

	private int idx(int col, int row) {
		return row * cols + col;
	}

	public boolean isOccupied(int col, int row) {
		return !inBounds(col, row) || occupied[idx(col, row)];
	}

	public void markCircle(double cx, double cy, double radius) {
		int rc = (int) Math.ceil(radius / RESOLUTION);
		int cc = toCol(cx), cr = toRow(cy);
		for (int dc = -rc; dc <= rc; dc++)
			for (int dr = -rc; dr <= rc; dr++)
				if (dc * dc + dr * dr <= rc * rc && inBounds(cc + dc, cr + dr))
					occupied[idx(cc + dc, cr + dr)] = true;
	}

	public void markRect(double xMin, double yMin, double xMax, double yMax) {
		int cMin = Math.max(0, toCol(xMin)), cMax = Math.min(cols - 1, toCol(xMax));
		int rMin = Math.max(0, toRow(yMin)), rMax = Math.min(rows - 1, toRow(yMax));
		for (int c = cMin; c <= cMax; c++)
			for (int r = rMin; r <= rMax; r++)
				occupied[idx(c, r)] = true;
	}

	public void markSegment(double x1, double y1, double x2, double y2, double halfWidth) {
		markRect(
				Math.min(x1, x2) - halfWidth, Math.min(y1, y2) - halfWidth,
				Math.max(x1, x2) + halfWidth, Math.max(y1, y2) + halfWidth);
	}

	public boolean hasLOS(int c0, int r0, int c1, int r1) {
		int dc = Math.abs(c1 - c0), dr = Math.abs(r1 - r0);
		int sc = c0 < c1 ? 1 : -1, sr = r0 < r1 ? 1 : -1;
		int err = dc - dr, c = c0, r = r0;
		while (true) {
			if (isOccupied(c, r))
				return false;
			if (c == c1 && r == r1)
				return true;
			int e2 = 2 * err;
			if (e2 > -dr) {
				err -= dr;
				c += sc;
			}
			if (e2 < dc) {
				err += dc;
				r += sr;
			}
		}
	}

	public boolean hasLOS(Translation2d a, Translation2d b) {
		return hasLOS(toCol(a.getX()), toRow(a.getY()),
				toCol(b.getX()), toRow(b.getY()));
	}
}
