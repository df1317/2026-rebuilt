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

package frc.robot.repulsor.Fields;

import static frc.robot.repulsor.RepulsorConstants.FIELD_LENGTH;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import frc.robot.planners.ThetaStarGrid;
import frc.robot.planners.GlobalPlannerLayer.GateDefinition;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.FieldPlanner.Obstacle;
import frc.robot.repulsor.FieldPlanner.Obstacles.CorridorCenterlineRail;
import frc.robot.repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import frc.robot.repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import frc.robot.repulsor.FieldPlanner.Obstacles.RectangleObstacle;
import frc.robot.repulsor.FieldPlanner.Obstacles.SquareObstacle;
import frc.robot.repulsor.FieldPlanner.Obstacles.VerticalObstacle;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Heatmap;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.Setpoints;
import frc.robot.repulsor.Tracking.Model.Alliance;
import frc.robot.repulsor.Tracking.Model.GameElement;

public final class Rebuilt2026 implements FieldDefinition {
	private static final double CORNER_CHAMFER = 0;

	private static final double GRID_Z_MIN_M = -0.05;
	private static final double GRID_Z_MAX_M = 0.35;

	private static final double GRID_CELL_M = 0.75;
	private static final double GRID_REGION_HALF_X_M = RepulsorConstants.FIELD_LENGTH * 0.5;
	private static final double GRID_REGION_HALF_Y_M = RepulsorConstants.FIELD_WIDTH * 0.5;

	@Override
	public GameElement[] build() {
		var b = new FieldMapBuilder();

		Pose2d blueOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateBluePose();
		Pose2d redOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateRedPose();

		Pose2d blueHub = Setpoints.Rebuilt2026.HUB_SCORE_FRONT.approximateBluePose();
		Pose2d redHub = Setpoints.Rebuilt2026.HUB_SCORE_FRONT.approximateRedPose();

		b.begin()
				.alliance(Alliance.kBlue)
				.capacity(999)
				.pose(new Pose3d(blueHub.getX(), blueHub.getY(), 0, null))
				.related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SCORE_FRONT, HeightSetpoint.NET))
				.category(CategorySpec.kScore)
				.add();

		b.begin()
				.alliance(Alliance.kRed)
				.capacity(999)
				.pose(new Pose3d(redHub.getX(), redHub.getY(), 0, null))
				.related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SCORE_FRONT, HeightSetpoint.NET))
				.category(CategorySpec.kScore)
				.add();

		b.begin()
				.alliance(Alliance.kBlue)
				.capacity(999)
				.pose(new Pose3d(blueOutpost.getX(), blueOutpost.getY(), 0, null))
				.related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
				.category(CategorySpec.kCollect)
				.add();

		b.begin()
				.alliance(Alliance.kRed)
				.capacity(999)
				.pose(new Pose3d(redOutpost.getX(), redOutpost.getY(), 0, null))
				.related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
				.category(CategorySpec.kCollect)
				.add();

		double cx = RepulsorConstants.FIELD_LENGTH * 0.5;
		double cy = RepulsorConstants.FIELD_WIDTH * 0.5;

		double x0 = cx - GRID_REGION_HALF_X_M;
		double x1 = cx + GRID_REGION_HALF_X_M;
		double y0 = cy - GRID_REGION_HALF_Y_M;
		double y1 = cy + GRID_REGION_HALF_Y_M;

		int nx = (int) Math.floor((x1 - x0) / GRID_CELL_M);
		int ny = (int) Math.floor((y1 - y0) / GRID_CELL_M);

		for (int ix = 0; ix <= nx; ix++) {
			double x = x0 + ix * GRID_CELL_M + GRID_CELL_M * 0.5;
			for (int iy = 0; iy <= ny; iy++) {
				double y = y0 + iy * GRID_CELL_M + GRID_CELL_M * 0.5;

				b.begin()
						.alliance(Alliance.kBlue)
						.capacity(999)
						.pose(new Pose3d(x, y, 0, null))
						.primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
						.filterType("fuel")
						.category(CategorySpec.kCollect)
						.add();

				b.begin()
						.alliance(Alliance.kRed)
						.capacity(999)
						.pose(new Pose3d(x, y, 0, null))
						.primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
						.filterType("fuel")
						.category(CategorySpec.kCollect)
						.add();
			}
		}

		return b.build();
	}

	@Override
	public String gameName() {
		return "REBUILT";
	}

	@Override
	public int gameYear() {
		return 2026;
	}

	@Override
	public List<Obstacle> fieldObstacles() {
		// double maxRangeY = 1;
		// double maxRangeX = 1.2;

		double rectWidth = 0.5929315;
		double rectHeight = 5.711800;
		double rectHalfY = rectHeight * 0.5;

		double leftRectX = (FIELD_LENGTH / 2) - 3.648981; // 2.5 meters 34 degrees 16 ms
		double rightRectX = (FIELD_LENGTH / 2) + 3.648981;
		double rectCy = RepulsorConstants.FIELD_WIDTH / 2;

		double gapHeight = Math.max(0.0, (RepulsorConstants.FIELD_WIDTH * 0.5) - rectHalfY);
		double gapTopY = RepulsorConstants.FIELD_WIDTH - (gapHeight * 0.5);
		double gapBottomY = gapHeight * 0.5;

		double rectStrength = 2.1; // Reduced from 2.5 to make edges less harsh
		double rectRangeX = 1.0; // Reduced from 1.2 to limit horizontal bulge
		double rectRangeY = 1.3; // Reduced from 1.5

		double biasStrength = 0.5; // Increased from 0.3 to pull stronger to center
		double biasRange = 1.5; // Increased from 1.2 to grab robot earlier

		double bypassStrengthScale = 2.0; // Increased to suck through tunnel harder
		double bypassRange = 2.0; // Increased to grab earlier and pull through

		double edgeOffset = 0.06;

		double rectHalfX = rectWidth * 0.5;
		Translation2d[] leftGate = new Translation2d[] {
				new Translation2d(leftRectX - rectHalfX, rectCy - rectHalfY),
				new Translation2d(leftRectX + rectHalfX, rectCy - rectHalfY),
				new Translation2d(leftRectX + rectHalfX, rectCy + rectHalfY),
				new Translation2d(leftRectX - rectHalfX, rectCy + rectHalfY)
		};
		Translation2d[] rightGate = new Translation2d[] {
				new Translation2d(rightRectX - rectHalfX, rectCy - rectHalfY),
				new Translation2d(rightRectX + rectHalfX, rectCy - rectHalfY),
				new Translation2d(rightRectX + rectHalfX, rectCy + rectHalfY),
				new Translation2d(rightRectX - rectHalfX, rectCy + rectHalfY)
		};

		double leftInsideX = leftRectX + rectHalfX + edgeOffset;
		double rightInsideX = rightRectX - rectHalfX - edgeOffset;

		double sidePullDx = 1.0;

		double leftPullXOut = leftRectX - sidePullDx;
		double rightPullXOut = rightRectX + sidePullDx;

		double sideBiasStrength = biasStrength * 0.50;
		double sideBiasRange = biasRange * 0.60;

		double sideBypassStrengthScale = bypassStrengthScale * 0.85;
		double sideBypassRange = bypassRange * 0.65;

		double leftPullXIn = leftRectX + sidePullDx;

		double rightPullXIn = rightRectX - sidePullDx;

		double railXWindow = 0.5;
		double corridorHalfWidthGuess = Math.max(0.22, gapHeight * 0.45);
		double railStrength = 2.5;
		double railMaxForce = 3.0;

		return List.of(
				new SquareObstacle(
						new Translation2d(0.259, 5.963), // Blue low force keepout
						0.5,
						0.8,
						0.5),
				new SquareObstacle(
						new Translation2d(RepulsorConstants.FIELD_LENGTH - 0.259, 5.963), // Red low force keepout
						0.5,
						0.8,
						0.5),
				new RectangleObstacle(
						new Translation2d(0.518, 4.178 - 0.4305), // Blue top
						1.036,
						0.861,
						1.4,
						0.5,
						0.5),
				new RectangleObstacle(
						new Translation2d(0.518, 3.317 + 0.4305), // Blue bottom
						1.036,
						0.861,
						1.4,
						0.5,
						0.5),
				new RectangleObstacle(
						new Translation2d(RepulsorConstants.FIELD_LENGTH - 0.518, 4.178 - 0.4305), // Red top
						1.036,
						0.861,
						1.4,
						0.5,
						0.5),
				new RectangleObstacle(
						new Translation2d(RepulsorConstants.FIELD_LENGTH - 0.518, 3.317 + 0.4305), // Red bottom
						1.036,
						0.861,
						1.4,
						0.5,
						0.5),
				new SquareObstacle(
						new Translation2d(4.625594, RepulsorConstants.FIELD_WIDTH / 2),
						1.1938,
						1.0,
						1.4),
				new SquareObstacle(
						new Translation2d(RepulsorConstants.FIELD_LENGTH - 4.625594, RepulsorConstants.FIELD_WIDTH / 2),
						1.1938,
						1.0,
						1.4),
				new RectangleObstacle(
						new Translation2d(leftRectX, rectCy),
						rectWidth,
						rectHeight,
						rectStrength,
						rectRangeX,
						rectRangeY),
				new RectangleObstacle(
						new Translation2d(rightRectX, rectCy),
						rectWidth,
						rectHeight,
						rectStrength,
						rectRangeX,
						rectRangeY),
				new GatedAttractorObstacle(
						new Translation2d(leftRectX, gapTopY),
						biasStrength,
						biasRange,
						leftGate,
						new Translation2d(leftInsideX, gapTopY),
						bypassStrengthScale,
						bypassRange,
						true),
				new GatedAttractorObstacle(
						new Translation2d(leftRectX, gapBottomY),
						biasStrength,
						biasRange,
						leftGate,
						new Translation2d(leftInsideX, gapBottomY),
						bypassStrengthScale,
						bypassRange,
						true),
				new GatedAttractorObstacle(
						new Translation2d(rightRectX, gapTopY),
						biasStrength,
						biasRange,
						rightGate,
						new Translation2d(rightInsideX, gapTopY),
						bypassStrengthScale,
						bypassRange,
						true),
				new GatedAttractorObstacle(
						new Translation2d(rightRectX, gapBottomY),
						biasStrength,
						biasRange,
						rightGate,
						new Translation2d(rightInsideX, gapBottomY),
						bypassStrengthScale,
						bypassRange,
						true),
				new GatedAttractorObstacle(
						new Translation2d(leftPullXIn, gapTopY),
						sideBiasStrength,
						sideBiasRange,
						leftGate,
						new Translation2d(leftInsideX, gapTopY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(leftPullXOut, gapTopY),
						sideBiasStrength,
						sideBiasRange,
						leftGate,
						new Translation2d(leftInsideX, gapTopY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(leftPullXIn, gapBottomY),
						sideBiasStrength,
						sideBiasRange,
						leftGate,
						new Translation2d(leftInsideX, gapBottomY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(leftPullXOut, gapBottomY),
						sideBiasStrength,
						sideBiasRange,
						leftGate,
						new Translation2d(leftInsideX, gapBottomY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(rightPullXIn, gapTopY),
						sideBiasStrength,
						sideBiasRange,
						rightGate,
						new Translation2d(rightInsideX, gapTopY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(rightPullXOut, gapTopY),
						sideBiasStrength,
						sideBiasRange,
						rightGate,
						new Translation2d(rightInsideX, gapTopY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(rightPullXIn, gapBottomY),
						sideBiasStrength,
						sideBiasRange,
						rightGate,
						new Translation2d(rightInsideX, gapBottomY),
						sideBypassStrengthScale,
						sideBypassRange),
				new GatedAttractorObstacle(
						new Translation2d(rightPullXOut, gapBottomY),
						sideBiasStrength,
						sideBiasRange,
						rightGate,
						new Translation2d(rightInsideX, gapBottomY),
						sideBypassStrengthScale,
						sideBypassRange),
				new CorridorCenterlineRail(
						leftRectX, railXWindow, gapTopY, corridorHalfWidthGuess, railStrength, railMaxForce),
				new CorridorCenterlineRail(
						leftRectX, railXWindow, gapBottomY, corridorHalfWidthGuess, railStrength, railMaxForce),
				new CorridorCenterlineRail(
						rightRectX, railXWindow, gapTopY, corridorHalfWidthGuess, railStrength, railMaxForce),
				new CorridorCenterlineRail(
						rightRectX,
						railXWindow,
						gapBottomY,
						corridorHalfWidthGuess,
						railStrength,
						railMaxForce));
	}

	// @Override // TRENCH + BUMP
	// public List<Obstacle> fieldObstacles() {
	// double maxRangeY = 0.9;
	// double maxRangeX = 0.5;
	// double strength = 1.4;
	// return List.of(
	// new SquareObstacle(new Translation2d(4.625594, RepulsorConstants.FIELD_WIDTH / 2), 1.1938, 1,
	// 1.4),
	// new SquareObstacle(
	// new Translation2d(FIELD_LENGTH - 4.625594, RepulsorConstants.FIELD_WIDTH / 2), 1.1938, 1,
	// 1.4),
	// new RectangleObstacle(
	// new Translation2d((FIELD_LENGTH / 2) - 3.63982, 1.4224),
	// 1.1938,
	// 0.255336,
	// strength,
	// maxRangeX,
	// maxRangeY),
	// new RectangleObstacle(
	// new Translation2d((FIELD_LENGTH / 2) - 3.63982, RepulsorConstants.FIELD_WIDTH - 1.4224),
	// 1.1938,
	// 0.255336,
	// strength,
	// maxRangeX,
	// maxRangeY),
	// new RectangleObstacle(
	// new Translation2d((FIELD_LENGTH / 2) + 3.63982, 1.4224),
	// 1.1938,
	// 0.255336,
	// strength,
	// maxRangeX,
	// maxRangeY),
	// new RectangleObstacle(
	// new Translation2d((FIELD_LENGTH / 2) + 3.63982, RepulsorConstants.FIELD_WIDTH - 1.4224),
	// 1.1938,
	// 0.255336,
	// strength,
	// maxRangeX,
	// maxRangeY));
	// }

	@Override
	public List<Obstacle> walls() {
		return List.of(
				new HorizontalObstacle(0.0, 1, true),
				new HorizontalObstacle(RepulsorConstants.FIELD_WIDTH, 1, false),
				new VerticalObstacle(0.0, 2, true),
				new VerticalObstacle(RepulsorConstants.FIELD_LENGTH, 2, false));
	}

	public ThetaStarGrid buildGrid() {
		ThetaStarGrid grid = new ThetaStarGrid();
		double ROBOT_INFLATION = 0.45;
		double GATE_INFLATION = 0.15;

		for (Obstacle o : fieldObstacles()) {
			if (o instanceof RectangleObstacle) {
				RectangleObstacle ro = (RectangleObstacle) o;
				grid.markRect(
						ro.center.getX() - ro.halfX - ROBOT_INFLATION,
						ro.center.getY() - ro.halfY - ROBOT_INFLATION,
						ro.center.getX() + ro.halfX + ROBOT_INFLATION,
						ro.center.getY() + ro.halfY + ROBOT_INFLATION);
			} else if (o instanceof SquareObstacle) {
				SquareObstacle so = (SquareObstacle) o;
				grid.markRect(
						so.center.getX() - so.halfSize - ROBOT_INFLATION,
						so.center.getY() - so.halfSize - ROBOT_INFLATION,
						so.center.getX() + so.halfSize + ROBOT_INFLATION,
						so.center.getY() + so.halfSize + ROBOT_INFLATION);
			} else if (o instanceof GatedAttractorObstacle) {
				GatedAttractorObstacle go = (GatedAttractorObstacle) o;
				// Estimate the wall segment from the bounding box of the gatePoly
				double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
				double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
				for (Translation2d p : go.gatePoly) {
					if (p.getX() < minX)
						minX = p.getX();
					if (p.getX() > maxX)
						maxX = p.getX();
					if (p.getY() < minY)
						minY = p.getY();
					if (p.getY() > maxY)
						maxY = p.getY();
				}
				grid.markRect(
						minX - GATE_INFLATION,
						minY - GATE_INFLATION,
						maxX + GATE_INFLATION,
						maxY + GATE_INFLATION);
			}
		}

		for (Obstacle w : walls()) {
			if (w instanceof HorizontalObstacle) {
				HorizontalObstacle ho = (HorizontalObstacle) w;
				grid.markSegment(0, ho.y, RepulsorConstants.FIELD_LENGTH, ho.y, ROBOT_INFLATION);
			} else if (w instanceof VerticalObstacle) {
				VerticalObstacle vo = (VerticalObstacle) w;
				grid.markSegment(vo.x, 0, vo.x, RepulsorConstants.FIELD_WIDTH, ROBOT_INFLATION);
			}
		}

		return grid;
	}

	public java.util.List<GateDefinition> buildGates() {
		java.util.List<GateDefinition> gates = new java.util.ArrayList<>();
		java.util.Set<Translation2d> addedBypassPoints = new java.util.HashSet<>();
		for (Obstacle o : fieldObstacles()) {
			if (o instanceof GatedAttractorObstacle) {
				GatedAttractorObstacle go = (GatedAttractorObstacle) o;
				double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
				double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
				for (Translation2d p : go.gatePoly) {
					if (p.getX() < minX)
						minX = p.getX();
					if (p.getX() > maxX)
						maxX = p.getX();
					if (p.getY() < minY)
						minY = p.getY();
					if (p.getY() > maxY)
						maxY = p.getY();
				}
				boolean isVertical = (maxY - minY) > (maxX - minX);
				Translation2d p1, p2;
				if (isVertical) {
					double avgX = (minX + maxX) / 2.0;
					p1 = new Translation2d(avgX, minY);
					p2 = new Translation2d(avgX, maxY);
				} else {
					double avgY = (minY + maxY) / 2.0;
					p1 = new Translation2d(minX, avgY);
					p2 = new Translation2d(maxX, avgY);
				}

				if (!addedBypassPoints.contains(go.bypassPoint)) {
					gates.add(new GateDefinition(p1, p2, go.bypassPoint));
					addedBypassPoints.add(go.bypassPoint);
				}
			}
		}
		return gates;
	}

	@Override
	public Heatmap getHeatmap() {
		final double SLOW_HEAT = 0.6;
		final double FAST_HEAT = 1.0;

		final double TRANS_M = 0.30;

		double cx = RepulsorConstants.FIELD_LENGTH * 0.5;
		double cy = RepulsorConstants.FIELD_WIDTH * 0.5;

		double x0 = cx - GRID_REGION_HALF_X_M;
		double x1 = cx + GRID_REGION_HALF_X_M;
		double y0 = cy - GRID_REGION_HALF_Y_M;
		double y1 = cy + GRID_REGION_HALF_Y_M;

		double half = 1.1938 * 0.5;

		double leftSqCx = 4.625594;
		double leftRectCx = (FIELD_LENGTH * 0.5) - 3.63982;

		double rightSqCx = FIELD_LENGTH - 4.625594;
		double rightRectCx = (FIELD_LENGTH * 0.5) + 3.63982;

		double leftBandX0 = Math.min(leftSqCx, leftRectCx) - half;
		double leftBandX1 = Math.max(leftSqCx, leftRectCx) + half;

		double rightBandX0 = Math.min(rightSqCx, rightRectCx) - half;
		double rightBandX1 = Math.max(rightSqCx, rightRectCx) + half;

		leftBandX0 = MathUtil.clamp(leftBandX0, x0, x1);
		leftBandX1 = MathUtil.clamp(leftBandX1, x0, x1);
		rightBandX0 = MathUtil.clamp(rightBandX0, x0, x1);
		rightBandX1 = MathUtil.clamp(rightBandX1, x0, x1);

		if (leftBandX1 > rightBandX0) {
			double mid = 0.5 * (leftBandX1 + rightBandX0);
			leftBandX1 = mid;
			rightBandX0 = mid;
		}

		double fastL = Math.max(0.0, leftBandX0 - x0);
		double fastM = Math.max(0.0, rightBandX0 - leftBandX1);
		double fastR = Math.max(0.0, x1 - rightBandX1);

		double eL = MathUtil.clamp(TRANS_M, 0.0, fastL);
		double eML = MathUtil.clamp(TRANS_M, 0.0, fastM * 0.5);
		double eMR = MathUtil.clamp(TRANS_M, 0.0, fastM * 0.5);
		double eR = MathUtil.clamp(TRANS_M, 0.0, fastR);

		double fastLCoreW = Math.max(0.0, fastL - eL);
		double fastMCoreW = Math.max(0.0, fastM - eML - eMR);
		double fastRCoreW = Math.max(0.0, fastR - eR);

		var hb = Heatmap.builder();

		double curX = x0;

		if (fastLCoreW > 0.0) {
			hb.block("FAST_L_CORE", new Translation2d(curX, y0), fastLCoreW, y1 - y0, FAST_HEAT);
			curX += fastLCoreW;
		}
		if (eL > 0.0) {
			hb.block("FAST_L_EDGE", new Translation2d(curX, y0), eL, y1 - y0, FAST_HEAT);
			curX += eL;
		}

		hb.block(
				"SLOW_L", new Translation2d(leftBandX0, y0), leftBandX1 - leftBandX0, y1 - y0, SLOW_HEAT);
		curX = leftBandX1;

		if (eML > 0.0) {
			hb.block("FAST_M_L_EDGE", new Translation2d(curX, y0), eML, y1 - y0, FAST_HEAT);
			curX += eML;
		}
		if (fastMCoreW > 0.0) {
			hb.block("FAST_M_CORE", new Translation2d(curX, y0), fastMCoreW, y1 - y0, FAST_HEAT);
			curX += fastMCoreW;
		}
		if (eMR > 0.0) {
			hb.block("FAST_M_R_EDGE", new Translation2d(curX, y0), eMR, y1 - y0, FAST_HEAT);
			curX += eMR;
		}

		hb.block(
				"SLOW_R",
				new Translation2d(rightBandX0, y0),
				rightBandX1 - rightBandX0,
				y1 - y0,
				SLOW_HEAT);
		curX = rightBandX1;

		if (eR > 0.0) {
			hb.block("FAST_R_EDGE", new Translation2d(curX, y0), eR, y1 - y0, FAST_HEAT);
			curX += eR;
		}
		if (fastRCoreW > 0.0) {
			hb.block("FAST_R_CORE", new Translation2d(curX, y0), fastRCoreW, y1 - y0, FAST_HEAT);
			curX += fastRCoreW;
		}

		if (eL > 0.0)
			hb.transition("SLOW_L", "FAST_L_EDGE", 1.0);
		if (eML > 0.0)
			hb.transition("SLOW_L", "FAST_M_L_EDGE", 1.0);

		if (eMR > 0.0)
			hb.transition("SLOW_R", "FAST_M_R_EDGE", 1.0);
		if (eR > 0.0)
			hb.transition("SLOW_R", "FAST_R_EDGE", 1.0);

		return hb.build();
	}
}
