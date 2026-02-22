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

package frc.robot.repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Supplier;
import frc.robot.repulsor.FieldPlanner.RepulsorSample;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.SetpointContext;

/**
 * Simplified auto behaviour that navigates to fixed setpoints via FieldPlanner.
 * If hasPiece -> navigate to score setpoint. If !hasPiece -> navigate to collect setpoint.
 * No FieldTracker or Predictive dependencies.
 */
public final class SimpleAutoPathBehaviour extends Behaviour {
	private final int prio;
	private final Supplier<Boolean> hasPiece;
	private final Supplier<RepulsorSetpoint> scoreSetpoint;
	private final Supplier<RepulsorSetpoint> collectSetpoint;

	public SimpleAutoPathBehaviour(int priority, Supplier<Boolean> hasPiece,
			Supplier<RepulsorSetpoint> scoreSetpoint, Supplier<RepulsorSetpoint> collectSetpoint) {
		this.prio = priority;
		this.hasPiece = hasPiece;
		this.scoreSetpoint = scoreSetpoint;
		this.collectSetpoint = collectSetpoint;
	}

	@Override
	public String name() {
		return "SimpleAutoPath";
	}

	@Override
	public int priority() {
		return prio;
	}

	@Override
	public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
		return !flags.contains(BehaviourFlag.DEFENCE_MODE);
	}

	private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
		return new SetpointContext(Optional.ofNullable(robotPose),
				Math.max(0.0, ctx.robot_x) * 2.0, Math.max(0.0, ctx.robot_y) * 2.0, 0.0,
				ctx.vision.getObstacles());
	}

	@Override
	public Command build(BehaviourContext ctx) {
		return Commands
				.run(() -> {
					Pose2d robotPose = ctx.robotPose.get();
					boolean piece = hasPiece.get();
					CategorySpec cat = piece ? CategorySpec.kScore : CategorySpec.kCollect;

					RepulsorSetpoint sp = piece ? scoreSetpoint.get() : collectSetpoint.get();
					if (sp == null) {
						ctx.drive.runVelocity(new ChassisSpeeds());
						return;
					}

					Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));
					ctx.repulsor.setCurrentGoal(sp);
					ctx.planner.setRequestedGoal(goalPose);

					RepulsorSample sample = ctx.planner.calculate(robotPose,
							ctx.vision.getObstacles(), ctx.robot_x, ctx.robot_y, cat, false, 0.0);

					ChassisSpeeds speeds = sample.asChassisSpeeds(
							ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation());

					ctx.drive.runVelocity(speeds);
				}, ctx.drive.asSubsystem())
				.finallyDo(i -> ctx.drive.runVelocity(new ChassisSpeeds()));
	}
}
