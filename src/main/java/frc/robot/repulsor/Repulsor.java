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

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FieldTranslation;
import java.util.Collections;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.repulsor.Fallback.PlannerFallback;
import frc.robot.repulsor.FieldPlanner.FieldPlanner;
import frc.robot.repulsor.FieldPlanner.RepulsorSample;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Fields.Rebuilt2026;
import frc.robot.repulsor.Setpoints.GameSetpoint;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.repulsor.Setpoints.SetpointType;
import frc.robot.repulsor.Tuning.DriveTuningHeat;

public class Repulsor {

	private static final int TRAJ_MAX_STEPS = 30;
	private static final double TRAJ_STEP_SIZE = 0.15; // meters

	public static final double DEFAULT_VELOCITY = 3.0; // m/s
	public static final double DEFAULT_DECEL = 4.5; // m/s²
	public static final double DEFAULT_POS_TOLERANCE = 0.15; // meters
	public static final double DEFAULT_ANG_TOLERANCE = Math.toRadians(6.0); // radians

	private final double robot_x_stowed;
	private final double robot_y_stowed;

	private FieldPlanner m_planner;
	private DriveRepulsor m_drive;
	private final DriveTuningHeat m_driveTuning;
	private double m_lastRepulsionIntensity = 0.0;

	private RepulsorSetpoint m_currentGoal;

	public boolean atSetpoint() {
		Optional<Distance> err = m_planner.getErr();
		if (err.isEmpty())
			return false;
		return err.get().lt(Meters.of(0.1));
	}

	private double getRobotX() {
		try {
			return IntakeFootprint.getFootprint().getEffectiveHalfLength();
		} catch (IllegalStateException e) {
			return robot_x_stowed;
		}
	}

	private double getRobotY() {
		try {
			return IntakeFootprint.getFootprint().getEffectiveHalfWidth();
		} catch (IllegalStateException e) {
			return robot_y_stowed;
		}
	}

	public Repulsor(
			DriveRepulsor drive,
			double robot_x,
			double robot_y) {
		this.m_drive = drive;
		this.robot_x_stowed = robot_x;
		this.robot_y_stowed = robot_y;

		m_driveTuning = new DriveTuningHeat(() -> m_drive.getPose());
		m_planner = new FieldPlanner(new Rebuilt2026(), m_driveTuning);
	}

	public void setAutoSpeedScale(double scale) {
		m_driveTuning.setSpeedScale(scale);
	}

	public void resetSpeedScale() {
		m_driveTuning.resetSpeedScale();
	}

	public void setHeadingBlendDist(double meters) {
		m_planner.setHeadingBlendDist(meters);
	}

	public void resetHeadingBlendDist() {
		m_planner.resetHeadingBlendDist();
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

	// ===== APF Drive =====

	// ===== APF Drive GameSetpoint Overloads =====

	public Command apfDrive(GameSetpoint goal) {
		return apfDrive(() -> goal.poseForCurrentAlliance(SetpointContext.EMPTY));
	}

	public Command apfDrive(GameSetpoint goal, double endTolerance, double endAngTolerance) {
		return apfDrive(() -> goal.poseForCurrentAlliance(SetpointContext.EMPTY), endTolerance, endAngTolerance);
	}

	public Command apfDrive(GameSetpoint goal, DoubleSupplier maxVelocity, DoubleSupplier maxDeceleration) {
		return apfDrive(() -> goal.poseForCurrentAlliance(SetpointContext.EMPTY), maxVelocity, maxDeceleration);
	}

	public Command apfDrive(GameSetpoint goal, DoubleSupplier maxVelocity, DoubleSupplier maxDeceleration,
			DoubleSupplier endTolerance, DoubleSupplier endAngTolerance) {
		return apfDrive(() -> goal.poseForCurrentAlliance(SetpointContext.EMPTY), maxVelocity, maxDeceleration,
				endTolerance, endAngTolerance);
	}

	public Command apfDriveFacing(GameSetpoint goal, FieldTranslation aimTarget, double rotationOffsetDeg,
			double endTolerance, double endAngTolerance) {
		return apfDriveFacing(() -> goal.poseForCurrentAlliance(SetpointContext.EMPTY), aimTarget, rotationOffsetDeg,
				endTolerance, endAngTolerance);
	}

	/**
	 * Drives to the goal using P-APF with the default speed profile
	 * ({@value #DEFAULT_VELOCITY} m/s, {@value #DEFAULT_DECEL} m/s²). This command does not end.
	 *
	 * @param goal
	 *          supplier for the target blue-origin pose
	 */
	public Command apfDrive(Supplier<Pose2d> goal) {
		return apfDrive(goal, () -> DEFAULT_VELOCITY, () -> DEFAULT_DECEL);
	}

	/**
	 * Drives to the goal using P-APF with the default speed profile. Ends when within the
	 * specified tolerances.
	 *
	 * @param goal
	 *          supplier for the target blue-origin pose
	 * @param endTolerance
	 *          position tolerance in meters
	 * @param endAngTolerance
	 *          rotation tolerance in radians
	 */
	public Command apfDrive(Supplier<Pose2d> goal, double endTolerance, double endAngTolerance) {
		return apfDrive(goal, () -> DEFAULT_VELOCITY, () -> DEFAULT_DECEL, () -> endTolerance, () -> endAngTolerance);
	}

	/**
	 * Drives to the goal using P-APF with specified speed profile. This command does not end.
	 *
	 * @param goal
	 *          supplier for the target blue-origin pose
	 * @param maxVelocity
	 *          cruise velocity in m/s
	 * @param maxDeceleration
	 *          deceleration rate in m/s²
	 */
	public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxVelocity, DoubleSupplier maxDeceleration) {
		final AtomicReference<Pose2d> activeRef = new AtomicReference<>();
		final AtomicBoolean initialized = new AtomicBoolean(false);

		Command cmd = Commands.run(
				() -> {
					if (!initialized.get()) {
						activeRef.set(goal.get());
						initialized.set(true);
					}

					m_driveTuning.setVelocityOverride(maxVelocity.getAsDouble());
					m_driveTuning.setDecelOverride(maxDeceleration.getAsDouble());

					Pose2d goalPose = activeRef.get();
					if (goalPose == null)
						return;

					if (ExtraPathing.robotIntersects(
							goalPose.getTranslation(), getRobotX(), getRobotY(),
							m_planner.getObstacles())) {
						return;
					}

					m_planner.setRequestedGoal(goalPose);

					Pose2d robotPose = m_drive.getPose();
					RepulsorSample sample = m_planner.calculate(
							robotPose,
							Collections.emptyList(),
							getRobotX(),
							getRobotY(),
							CategorySpec.kScore,
							false);

					ChassisSpeeds commanded = sample.asChassisSpeeds(m_drive.getOmegaPID(), robotPose.getRotation());
					m_drive.runVelocity(commanded);

					DogLog.forceNt.log("Repulsor/Target", goalPose);
					DogLog.forceNt.log("Repulsor/Trajectory", m_planner.getLastTrajectory());
					DogLog.log("Repulsor/Error", robotPose.getTranslation().getDistance(goalPose.getTranslation()));
					DogLog.log("Repulsor/CommandedVx", commanded.vxMetersPerSecond);
					DogLog.log("Repulsor/CommandedVy", commanded.vyMetersPerSecond);
					DogLog.log("Repulsor/CommandedOmega", commanded.omegaRadiansPerSecond);
				},
				m_drive.asSubsystem())
				.finallyDo(interrupted -> {
					m_driveTuning.clearOverrides();
					m_drive.lock();
					DogLog.forceNt.log("Repulsor/Target", new Pose2d());
					DogLog.forceNt.log("Repulsor/Trajectory", new edu.wpi.first.math.geometry.Translation2d[] {});
				});

		return cmd;
	}

	/**
	 * Drives to the goal using P-APF with specified speed profile. Ends when within tolerances.
	 *
	 * @param goal
	 *          supplier for the target blue-origin pose
	 * @param maxVelocity
	 *          cruise velocity in m/s
	 * @param maxDeceleration
	 *          deceleration rate in m/s²
	 * @param endTolerance
	 *          position tolerance in meters
	 * @param endAngTolerance
	 *          rotation tolerance in radians
	 */
	public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxVelocity, DoubleSupplier maxDeceleration,
			DoubleSupplier endTolerance, DoubleSupplier endAngTolerance) {
		return apfDrive(goal, maxVelocity, maxDeceleration)
				.until(() -> {
					Pose2d goalPose = goal.get();
					Pose2d robotPose = m_drive.getPose();
					if (goalPose == null || robotPose == null)
						return false;
					double posErr = robotPose.getTranslation().getDistance(goalPose.getTranslation());
					double angErr = Math.abs(robotPose.getRotation().minus(goalPose.getRotation()).getRadians());
					return posErr <= endTolerance.getAsDouble() && angErr <= endAngTolerance.getAsDouble();
				});
	}

	// ===== APF Drive Facing =====

	/**
	 * Drives to the goal using P-APF while facing a specific target, with default speed profile.
	 * Ends when within tolerances.
	 */
	public Command apfDriveFacing(Supplier<Pose2d> goal, FieldTranslation aimTarget, double rotationOffsetDeg,
			double endTolerance, double endAngTolerance) {
		return apfDriveFacing(goal, aimTarget, rotationOffsetDeg, () -> DEFAULT_VELOCITY, () -> DEFAULT_DECEL,
				() -> endTolerance, () -> endAngTolerance);
	}

	/**
	 * Drives to the goal using P-APF while facing a specific target, with specified speed profile.
	 * Ends when within tolerances.
	 */
	public Command apfDriveFacing(Supplier<Pose2d> goal, FieldTranslation aimTarget, double rotationOffsetDeg,
			DoubleSupplier maxVelocity, DoubleSupplier maxDeceleration,
			DoubleSupplier endTolerance, DoubleSupplier endAngTolerance) {
		Supplier<Pose2d> facingGoal = () -> {
			Pose2d pose = goal.get();
			Translation2d target = aimTarget.get();
			edu.wpi.first.math.geometry.Rotation2d towardTarget = target.minus(pose.getTranslation()).getAngle();
			edu.wpi.first.math.geometry.Rotation2d facing = towardTarget
					.rotateBy(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(rotationOffsetDeg));
			return new Pose2d(pose.getTranslation(), facing);
		};
		return apfDrive(facingGoal, maxVelocity, maxDeceleration, endTolerance, endAngTolerance);
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
			m_lastRepulsionIntensity = 0.0;
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
			m_lastRepulsionIntensity = 0.0;
			return requested;
		}

		// Scale factor: how much to suppress the toward-obstacle component
		// Use repulsion magnitude to determine aggressiveness (stronger repulsion = more clamping)
		double scale = Math.max(0.0, 1.0 - Math.min(1.0, repNorm / 10.0));

		// Store repulsion intensity (0 = no clamping, 1 = full clamping)
		m_lastRepulsionIntensity = 1.0 - scale;

		// Remove the toward-obstacle component and scale it
		double clampedVx = vx - dot * repDirX * (1.0 - scale);
		double clampedVy = vy - dot * repDirY * (1.0 - scale);

		return new ChassisSpeeds(clampedVx, clampedVy, requested.omegaRadiansPerSecond);
	}

	public double getRepulsionIntensity() {
		return m_lastRepulsionIntensity;
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
