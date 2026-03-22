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

package frc.robot.repulsor.Tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import frc.robot.repulsor.RepulsorConstants;
import frc.robot.repulsor.Heatmap;

public class DriveTuningHeat extends DriveTuning {
	private double baseMaxSpeed = 5.14;
	private double speedScale = 1.0;
	private double sqrtScale = 6.0;
	private double minStep = 0.02;
	private double nearStart = 0.40;
	private double nearEnd = 0.02;

	// Per-command overrides (set by Repulsor.apfDrive, reset on command end)
	private double velocityOverride = -1.0;
	private double decelOverride = -1.0;

	private final Heatmap heatmap;
	private final Supplier<Pose2d> robotPoseSupplier;

	public DriveTuningHeat(Supplier<Pose2d> robotPoseSupplier) {
		super("Drive/Heat");
		this.heatmap = RepulsorConstants.FIELD.getHeatmap();
		this.robotPoseSupplier = robotPoseSupplier;
	}

	public DriveTuningHeat withBaseMaxSpeed(double mps) {
		this.baseMaxSpeed = mps;
		return this;
	}

	public void setSpeedScale(double scale) {
		this.speedScale = MathUtil.clamp(scale, 0.0, 1.0);
	}

	public void resetSpeedScale() {
		this.speedScale = 1.0;
	}

	public void setVelocityOverride(double mps) {
		this.velocityOverride = mps;
	}

	public void setDecelOverride(double mps2) {
		this.decelOverride = mps2;
	}

	public void clearOverrides() {
		this.velocityOverride = -1.0;
		this.decelOverride = -1.0;
	}

	public DriveTuningHeat withSqrtScale(double s) {
		this.sqrtScale = s;
		return this;
	}

	public DriveTuningHeat withMinStep(double m) {
		this.minStep = m;
		return this;
	}

	public DriveTuningHeat withNearWindow(double startM, double endM) {
		this.nearStart = startM;
		this.nearEnd = endM;
		return this;
	}

	@Override
	public void applyDefaults() {
		setDtSeconds(0.02);
	}

	@Override
	public void reset() {
	}

	public double maxLinearSpeedMps(Pose2d robotPose) {
		if (velocityOverride > 0.0)
			return velocityOverride;
		double effectiveMax = baseMaxSpeed * speedScale;
		if (robotPose == null)
			return effectiveMax;
		double heat = heatmap.heatAt(robotPose.getTranslation());
		double heatScale = MathUtil.clamp(heat, 0.0, 1.0);
		return effectiveMax * heatScale;
	}

	@Override
	public double maxLinearSpeedMps() {
		if (velocityOverride > 0.0)
			return velocityOverride;
		return baseMaxSpeed * speedScale;
	}

	@Override
	public double minStepMeters() {
		return minStep;
	}

	@Override
	public double baseStepMeters(double distanceMeters, boolean slowDown) {
		double d = Math.max(0.0, distanceMeters);
		if (d <= 0.0) {
			return 0.0;
		}

		double effectiveMax = velocityOverride > 0.0 ? velocityOverride : baseMaxSpeed * speedScale;

		if (!slowDown) {
			return Math.min(effectiveMax * dtSeconds(), d);
		}

		double dt = dtSeconds();
		if (dt <= 0.0) {
			dt = 0.02;
		}

		Pose2d pose = getRobotPoseOrNull();
		double vMaxHeat = effectiveMax;
		double heat = 1.0;
		if (velocityOverride <= 0.0 && pose != null) {
			Translation2d p = pose.getTranslation();
			heat = heatmap.heatAt(p);
			vMaxHeat = effectiveMax * MathUtil.clamp(heat, 0.0, 1.0);
		}

		double vMax = vMaxHeat;
		double aMax = Math.max(0.01, decelOverride > 0.0 ? decelOverride : sqrtScale);

		double dBrake = vMax * vMax / (2.0 * aMax);

		double v;
		if (d > dBrake) {
			v = vMax;
		} else {
			v = Math.sqrt(2.0 * aMax * d);
		}

		if (d < nearStart && nearStart > nearEnd) {
			double t = (d - nearEnd) / (nearStart - nearEnd);
			t = MathUtil.clamp(t, 0.0, 1.0);
			double s = t * t * (3.0 - 2.0 * t);
			v *= s;
		}

		double step = v * dt;

		if (step < minStep && d > minStep) {
			step = minStep;
		}

		if (step > d) {
			step = d;
		}

		return step;
	}

	@Override
	public double nearGoalScale(double distanceMeters) {
		return 1.0;
	}

	@Override
	public double scaleForTurning(double yawDeltaRad, boolean isScoring) {
		return 1.0;
	}

	private Pose2d getRobotPoseOrNull() {
		try {
			return robotPoseSupplier.get();
		} catch (Exception e) {
			return null;
		}
	}
}
