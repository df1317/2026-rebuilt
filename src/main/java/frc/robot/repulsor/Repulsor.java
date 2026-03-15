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

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Collections;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import frc.robot.repulsor.Fallback.PlannerFallback;
import frc.robot.repulsor.FieldPlanner.FieldPlanner;
import frc.robot.repulsor.FieldPlanner.RepulsorSample;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Fields.Rebuilt2026;
import frc.robot.repulsor.Setpoints.GameSetpoint;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.SetpointType;
import frc.robot.repulsor.Tuning.DriveTuningHeat;

public class Repulsor {

	private double robot_x;
	private double robot_y;

	private FieldPlanner m_planner;
	private DriveRepulsor m_drive;

	private RepulsorSetpoint m_currentGoal;

	public boolean atSetpoint() {
		Optional<Distance> err = m_planner.getErr();
		if (err.isEmpty())
			return false;
		return err.get().lt(Meters.of(0.1));
	}

	public Repulsor(
			DriveRepulsor drive,
			double robot_x,
			double robot_y) {
		this.m_drive = drive;
		this.robot_x = robot_x;
		this.robot_y = robot_y;

		m_planner = new FieldPlanner(new Rebuilt2026(), new DriveTuningHeat(() -> m_drive.getPose()));
	}

	public Repulsor withFallback(PlannerFallback fallback) {
		m_planner = m_planner.withFallback(fallback);
		return this;
	}

	public FieldPlanner getFieldPlanner() {
		return m_planner;
	}

	public void update() {
		DeltaTime.update();
	}

	public DriveRepulsor getDrive() {
		return m_drive;
	}

	public HeightSetpoint getTargetHeight() {
		return m_currentGoal == null ? HeightSetpoint.NONE : m_currentGoal.height();
	}

	// ===== Navigate To =====

	public Command navigateTo(Pose2d target) {
		return navigateTo(() -> target);
	}

	public Command navigateTo(Supplier<Pose2d> target) {
		final AtomicReference<Pose2d> activeRef = new AtomicReference<>();
		final AtomicBoolean initialized = new AtomicBoolean(false);

		Command cmd = Commands.run(
				() -> {
					if (!initialized.get()) {
						activeRef.set(target.get());
						initialized.set(true);
					}

					Pose2d goalPose = activeRef.get();
					if (goalPose == null)
						return;

					// Reject if target is inside an obstacle
					if (ExtraPathing.robotIntersects(
							goalPose.getTranslation(), robot_x, robot_y,
							m_planner.getObstacles())) {
						return;
					}

					m_planner.setRequestedGoal(goalPose);

					Pose2d robotPose = m_drive.getPose();
					RepulsorSample sample = m_planner.calculate(
							robotPose,
							Collections.emptyList(),
							robot_x,
							robot_y,
							CategorySpec.kScore,
							false,
							0.0);

					m_drive.runVelocity(
							sample.asChassisSpeeds(m_drive.getOmegaPID(), robotPose.getRotation()));
				},
				m_drive.asSubsystem())
				.finallyDo(interrupted -> m_drive.runVelocity(new ChassisSpeeds()));

		return cmd;
	}

	// ===== Clamp Drive Speed =====

	public ChassisSpeeds clampDriveSpeed(ChassisSpeeds requested, Pose2d currentPose) {
		Translation2d robotPos = currentPose.getTranslation();

		// Get obstacle repulsion force at current position (use a far-away dummy goal
		// so goal force doesn't interfere)
		Translation2d dummyGoal = robotPos.plus(new Translation2d(100.0, 0.0));
		Force obstacleForce = m_planner.getObstacleForce(robotPos, dummyGoal);
		Force wallForce = m_planner.getWallForce(robotPos, dummyGoal);
		Force totalRepulsion = obstacleForce.plus(wallForce);

		if (totalRepulsion.getNorm() < 1e-6) {
			return requested;
		}

		// Convert requested chassis speeds to a velocity vector
		double vx = requested.vxMetersPerSecond;
		double vy = requested.vyMetersPerSecond;

		// Compute dot product of velocity with repulsion force direction
		// Negative dot product means driving toward the obstacle
		double repX = totalRepulsion.getX();
		double repY = totalRepulsion.getY();
		double repNorm = totalRepulsion.getNorm();
		double repDirX = repX / repNorm;
		double repDirY = repY / repNorm;

		double dot = vx * repDirX + vy * repDirY;

		if (dot >= 0) {
			// Already driving away from obstacles, no clamping needed
			return requested;
		}

		// Scale factor: how much to suppress the toward-obstacle component
		// Use repulsion magnitude to determine aggressiveness (stronger repulsion = more clamping)
		double scale = Math.max(0.0, 1.0 - Math.min(1.0, repNorm / 10.0));

		// Remove the toward-obstacle component and scale it
		double clampedVx = vx - dot * repDirX * (1.0 - scale);
		double clampedVy = vy - dot * repDirY * (1.0 - scale);

		return new ChassisSpeeds(clampedVx, clampedVy, requested.omegaRadiansPerSecond);
	}

	// ===== Setpoint Queries =====

	public Trigger within(Distance d) {
		return new Trigger(
				() -> {
					Optional<Distance> err = m_planner.getErr();
					if (err.isEmpty())
						return false;
					return err.get().lt(d);
				});
	}

	public Trigger within(Distance d, SetpointType t) {
		return new Trigger(
				() -> {
					Optional<Distance> err = m_planner.getErr();
					if (err.isEmpty())
						return false;
					boolean within = err.get().lt(d);
					return within && m_currentGoal != null && m_currentGoal.point().type() == t;
				});
	}

	public Trigger within(Distance d, GameSetpoint p) {
		return new Trigger(
				() -> {
					Optional<Distance> err = m_planner.getErr();
					if (err.isEmpty())
						return false;
					boolean within = err.get().lt(d);
					return within && m_currentGoal != null && m_currentGoal.point() == p;
				});
	}
}
