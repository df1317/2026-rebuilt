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

package frc.robot.repulsor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;

public final class IntakeFootprint {
	private static IntakeFootprint stowedInstance = null;
	private static IntakeFootprint extendedInstance = null;
	private static java.util.function.BooleanSupplier intakeExtendedSupplier = () -> false;

	/** Returns the active footprint based on whether the intake is extended. */
	public static IntakeFootprint getFootprint() {
		if (stowedInstance == null) {
			throw new IllegalStateException(
					"IntakeFootprint instance not initialized. Call setFootprints first.");
		}
		return intakeExtendedSupplier.getAsBoolean() ? extendedInstance : stowedInstance;
	}

	/**
	 * Sets both the stowed and extended footprints, plus a supplier to determine which is active.
	 */
	public static void setFootprints(IntakeFootprint stowed, IntakeFootprint extended,
			java.util.function.BooleanSupplier isExtended) {
		stowedInstance = Objects.requireNonNull(stowed);
		extendedInstance = Objects.requireNonNull(extended);
		intakeExtendedSupplier = Objects.requireNonNull(isExtended);
	}

	/** @deprecated Use setFootprints instead. */
	@Deprecated
	public static void setFootprint(IntakeFootprint footprint) {
		stowedInstance = Objects.requireNonNull(footprint);
		if (extendedInstance == null) {
			extendedInstance = footprint;
		}
	}

	public static IntakeFootprint robotSquare(double robotSideMeters) {
		double h = 0.5 * robotSideMeters;
		return new IntakeFootprint(new Rect(new Translation2d(0.0, 0.0), h, h));
	}

	public static IntakeFootprint robotRect(double robotLengthMeters, double robotWidthMeters) {
		return new IntakeFootprint(
				new Rect(new Translation2d(0.0, 0.0), 0.5 * robotLengthMeters, 0.5 * robotWidthMeters));
	}

	public static IntakeFootprint frontRect(
			double robotLengthMeters, double intakeDepthMeters, double intakeWidthMeters) {
		double hx = 0.5 * intakeDepthMeters;
		double hy = 0.5 * intakeWidthMeters;
		double cx = 0.5 * robotLengthMeters + hx;
		return new IntakeFootprint(new Rect(new Translation2d(cx, 0.0), hx, hy));
	}

	/**
	 * Creates a footprint combining the robot body with an intake extension at a given angle.
	 *
	 * @param robotSideMeters
	 *          full side length of the square robot
	 * @param intakeLengthMeters
	 *          length the intake extends beyond the robot edge
	 * @param intakeAngleDeg
	 *          angle in robot frame where the intake extends (0=front, -90=right, 90=left, 180=rear)
	 */
	public static IntakeFootprint robotWithIntake(
			double robotLengthMeters, double robotWidthMeters,
			double intakeLengthMeters, double intakeAngleDeg) {
		double halfX = 0.5 * robotLengthMeters;
		double halfY = 0.5 * robotWidthMeters;
		Rect body = new Rect(new Translation2d(0.0, 0.0), halfX, halfY);

		double angleRad = Math.toRadians(intakeAngleDeg);
		double dirX = Math.cos(angleRad);
		double dirY = Math.sin(angleRad);

		// Distance from center to robot edge in the intake direction
		double edgeDist = Math.abs(dirX) * halfX + Math.abs(dirY) * halfY;
		double intakeHalf = 0.5 * intakeLengthMeters;
		double cx = dirX * (edgeDist + intakeHalf);
		double cy = dirY * (edgeDist + intakeHalf);

		double intakeWidth = 0.15;
		double hAlong = intakeHalf;
		double hPerp = 0.5 * intakeWidth;

		Rect intake = new Rect(new Translation2d(cx, cy), hAlong, hPerp, angleRad);

		return new IntakeFootprint(new CompoundShape(body, intake));
	}

	public static IntakeFootprint robotWithIntake(
			double robotSideMeters, double intakeLengthMeters, double intakeAngleDeg) {
		double halfRobot = 0.5 * robotSideMeters;
		Rect body = new Rect(new Translation2d(0.0, 0.0), halfRobot, halfRobot);

		double angleRad = Math.toRadians(intakeAngleDeg);
		double dirX = Math.cos(angleRad);
		double dirY = Math.sin(angleRad);

		// Intake center: offset from robot edge by half the intake length in the given direction
		double intakeHalf = 0.5 * intakeLengthMeters;
		double cx = dirX * (halfRobot + intakeHalf);
		double cy = dirY * (halfRobot + intakeHalf);

		// Intake rect: thin along the extension direction, same width as intake length
		// Use a small fixed width perpendicular to extension direction
		double intakeWidth = 0.15; // 15cm wide perpendicular to extension
		double hAlong = intakeHalf;
		double hPerp = 0.5 * intakeWidth;

		// Build the intake rect aligned to the extension angle
		Rect intake = new Rect(new Translation2d(cx, cy), hAlong, hPerp, angleRad);

		return new IntakeFootprint(new CompoundShape(body, intake));
	}

	private final Shape shape;

	private IntakeFootprint(Shape shape) {
		this.shape = Objects.requireNonNull(shape);
	}

	public boolean containsPointRobotFrame(Translation2d pRobot) {
		return shape.contains(pRobot);
	}

	public Translation2d supportPointRobotFrame(Translation2d dirRobot) {
		return shape.support(dirRobot);
	}

	/** Returns the maximum distance from origin to any point in the footprint (for corridor radius). */
	public double getMaxRadius() {
		// Sample support in many directions and take the max distance
		double max = 0;
		for (int i = 0; i < 36; i++) {
			double angle = i * Math.PI / 18.0;
			Translation2d dir = new Translation2d(Math.cos(angle), Math.sin(angle));
			Translation2d sp = shape.support(dir);
			max = Math.max(max, sp.getNorm());
		}
		return max;
	}

	/** Returns the effective half-length (max extent in X from center) for AABB calculations. */
	public double getEffectiveHalfLength() {
		double px = shape.support(new Translation2d(1.0, 0.0)).getX();
		double nx = -shape.support(new Translation2d(-1.0, 0.0)).getX();
		return Math.max(px, nx);
	}

	/** Returns the effective half-width (max extent in Y from center) for AABB calculations. */
	public double getEffectiveHalfWidth() {
		double py = shape.support(new Translation2d(0.0, 1.0)).getY();
		double ny = -shape.support(new Translation2d(0.0, -1.0)).getY();
		return Math.max(py, ny);
	}

	public Translation2d snapCenterSoFootprintTouchesPoint(
			Translation2d desiredCenterField, Rotation2d robotHeading, Translation2d pointField) {

		Translation2d dirField = pointField.minus(desiredCenterField);
		Translation2d dirRobot = dirField.rotateBy(robotHeading.unaryMinus());

		double n2 = dirRobot.getX() * dirRobot.getX() + dirRobot.getY() * dirRobot.getY();
		if (n2 < 1e-12) {
			dirRobot = new Translation2d(1.0, 0.0);
		}

		Translation2d contactRobot = shape.support(dirRobot);
		Translation2d contactField = contactRobot.rotateBy(robotHeading);

		return pointField.minus(contactField);
	}

	public Translation2d snapCenterSoPointIsInsideFootprint(
			Translation2d desiredCenterField, Rotation2d robotHeading, Translation2d pointField) {

		Translation2d pRobot = pointField.minus(desiredCenterField).rotateBy(robotHeading.unaryMinus());
		Translation2d clampedRobot = shape.closestPointInside(pRobot);
		Translation2d deltaRobot = pRobot.minus(clampedRobot);

		return desiredCenterField.plus(deltaRobot.rotateBy(robotHeading));
	}

	private interface Shape {
		boolean contains(Translation2d p);

		Translation2d support(Translation2d dir);

		Translation2d closestPointInside(Translation2d p);
	}

	private static final class Rect implements Shape {
		private final Translation2d c;
		private final double hx;
		private final double hy;
		private final double cosA;
		private final double sinA;

		Rect(Translation2d center, double halfX, double halfY) {
			this(center, halfX, halfY, 0.0);
		}

		Rect(Translation2d center, double halfX, double halfY, double angleRad) {
			this.c = Objects.requireNonNull(center);
			this.hx = Math.max(0.0, halfX);
			this.hy = Math.max(0.0, halfY);
			this.cosA = Math.cos(angleRad);
			this.sinA = Math.sin(angleRad);
		}

		/** Rotate a point from parent frame into this rect's local frame. */
		private Translation2d toLocal(Translation2d p) {
			double dx = p.getX() - c.getX();
			double dy = p.getY() - c.getY();
			return new Translation2d(dx * cosA + dy * sinA, -dx * sinA + dy * cosA);
		}

		/** Rotate a point from this rect's local frame back to parent frame. */
		private Translation2d toParent(Translation2d local) {
			double lx = local.getX();
			double ly = local.getY();
			return new Translation2d(c.getX() + lx * cosA - ly * sinA, c.getY() + lx * sinA + ly * cosA);
		}

		@Override
		public boolean contains(Translation2d p) {
			Translation2d local = toLocal(p);
			return Math.abs(local.getX()) <= hx + 1e-9 && Math.abs(local.getY()) <= hy + 1e-9;
		}

		@Override
		public Translation2d support(Translation2d dir) {
			Translation2d localDir = new Translation2d(dir.getX() * cosA + dir.getY() * sinA,
					-dir.getX() * sinA + dir.getY() * cosA);
			double sx = localDir.getX() >= 0.0 ? hx : -hx;
			double sy = localDir.getY() >= 0.0 ? hy : -hy;
			return toParent(new Translation2d(sx, sy));
		}

		@Override
		public Translation2d closestPointInside(Translation2d p) {
			Translation2d local = toLocal(p);
			double x = clamp(local.getX(), -hx, hx);
			double y = clamp(local.getY(), -hy, hy);
			return toParent(new Translation2d(x, y));
		}

		private static double clamp(double v, double lo, double hi) {
			return Math.max(lo, Math.min(hi, v));
		}
	}

	private static final class CompoundShape implements Shape {
		private final Shape a;
		private final Shape b;

		CompoundShape(Shape a, Shape b) {
			this.a = Objects.requireNonNull(a);
			this.b = Objects.requireNonNull(b);
		}

		@Override
		public boolean contains(Translation2d p) {
			return a.contains(p) || b.contains(p);
		}

		@Override
		public Translation2d support(Translation2d dir) {
			Translation2d sa = a.support(dir);
			Translation2d sb = b.support(dir);
			double dotA = sa.getX() * dir.getX() + sa.getY() * dir.getY();
			double dotB = sb.getX() * dir.getX() + sb.getY() * dir.getY();
			return dotA >= dotB ? sa : sb;
		}

		@Override
		public Translation2d closestPointInside(Translation2d p) {
			if (a.contains(p) || b.contains(p)) {
				return p;
			}
			Translation2d ca = a.closestPointInside(p);
			Translation2d cb = b.closestPointInside(p);
			double da = ca.minus(p).getNorm();
			double db = cb.minus(p).getNorm();
			return da <= db ? ca : cb;
		}
	}
}
