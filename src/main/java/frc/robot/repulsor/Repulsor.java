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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import frc.robot.repulsor.Behaviours.Behaviour;
import frc.robot.repulsor.Behaviours.BehaviourContext;
import frc.robot.repulsor.Behaviours.BehaviourFlag;
import frc.robot.repulsor.Behaviours.BehaviourManager;
import frc.robot.repulsor.Commands.Triggers;
import frc.robot.repulsor.DriverStation.NtRepulsorDriverStation;
import frc.robot.repulsor.DriverStation.RepulsorDriverStation;
import frc.robot.repulsor.Fallback.PlannerFallback;
import frc.robot.repulsor.FieldPlanner.FieldPlanner;
import frc.robot.repulsor.FieldPlanner.Obstacle;
import frc.robot.repulsor.FieldPlanner.RepulsorSample;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Fields.Rebuilt2026;
import frc.robot.repulsor.Flags.FlagManager;
import frc.robot.repulsor.Setpoints.GameSetpoint;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.repulsor.Setpoints.SetpointType;
import frc.robot.repulsor.Setpoints.Setpoints;
import frc.robot.repulsor.State.StateManager;
import frc.robot.repulsor.Tuning.DriveTuningHeat;

public class Repulsor {

	public enum UsageType {
		kFullAuto, kAutoDrive
	}

	private double robot_x;
	private double robot_y;
	private double algae_offset;
	private double coral_offset;

	private Supplier<Double> shooterReleaseHeightMeters = () -> 0.0;

	private FieldPlanner m_planner;
	private VisionPlanner m_visionPlanner = new VisionPlanner();
	private DriveRepulsor m_drive;
	private UsageType m_usageType = UsageType.kAutoDrive;

	private RepulsorSetpoint m_currentGoal = new RepulsorSetpoint(Setpoints.Rebuilt2026.CENTER_COLLECT,
			HeightSetpoint.NONE);

	private RepulsorSetpoint m_nextScore = new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET);

	private Optional<Trigger> m_gateInScoring = Optional.empty();
	private Optional<Trigger> m_gateInCollecting = Optional.empty();

	private Supplier<Boolean> m_hasPiece = () -> false;

	private BehaviourManager m_behaviourManager;
	private FlagManager<BehaviourFlag> m_behaviourFlags;

	public boolean atSetpoint() {
		Optional<Distance> err = m_planner.getErr();
		if (err.isEmpty())
			return false;
		// Logger.recordOutput("Repulsor/err", err.get());
		return err.isPresent() && err.get().lt(Meters.of(0.1));
	}

	public Repulsor withHasPieceSupplier(Supplier<Boolean> hasPiece) {
		this.m_hasPiece = hasPiece;
		return this;
	}

	public Repulsor withShooterReleaseHeightMetersSupplier(Supplier<Double> supplier) {
		this.shooterReleaseHeightMeters = supplier == null ? () -> 0.0 : supplier;
		return this;
	}

	public Repulsor addBehaviour(Behaviour behaviour) {
		m_behaviourManager.add(Objects.requireNonNull(behaviour, "behaviour"));
		return this;
	}

	public Repulsor addBehaviours(Behaviour... behaviours) {
		if (behaviours == null)
			return this;
		for (Behaviour behaviour : behaviours) {
			addBehaviour(behaviour);
		}
		return this;
	}

	public Repulsor clearBehaviours() {
		m_behaviourManager.clear();
		return this;
	}

	public Repulsor setReasoner(Object reasoner) {
		return this;
	}

	public boolean isInScoringGate() {
		return m_gateInScoring.map(Trigger::getAsBoolean).orElse(true);
	}

	public boolean isInCollectingGate() {
		return m_gateInCollecting.map(Trigger::getAsBoolean).orElse(false);
	}

	public RepulsorSetpoint getNextScore() {
		return m_nextScore;
	}

	public <T> Repulsor followGate(Triggers.PhaseGate<T> gate, T collecting, T scoring) {
		Trigger inScoring = gate.when(scoring);
		Trigger inCollecting = gate.when(collecting);

		m_gateInScoring = Optional.of(inScoring);
		m_gateInCollecting = Optional.of(inCollecting);
		return this;
	}

	public <E extends Enum<E>> Repulsor followGate(
			Triggers.ParallelGate<E> gate, EnumSet<E> collectingTags, EnumSet<E> scoringTags) {

		if (collectingTags.isEmpty() || scoringTags.isEmpty()) {
			throw new IllegalArgumentException("collectingTags and scoringTags must be non-empty");
		}

		Supplier<Boolean> scoringAllOn = () -> collectingTags.stream().allMatch(gate::isOn)
				&& scoringTags.stream().allMatch(gate::isOn);
		Supplier<Boolean> collectingAllOn = () -> collectingTags.stream().allMatch(gate::isOn);

		Trigger inScoring = new Trigger(scoringAllOn::get);
		Trigger inCollecting = new Trigger(collectingAllOn::get);

		m_gateInScoring = Optional.of(inScoring);
		m_gateInCollecting = Optional.of(inCollecting);
		return this;
	}

	public Repulsor(
			DriveRepulsor drive,
			UsageType usageType,
			double robot_x,
			double robot_y,
			double coral_offset,
			double algae_offset,
			Supplier<Boolean> hasPiece) {
		this.m_drive = drive;
		this.m_usageType = usageType;
		this.robot_x = robot_x;
		this.robot_y = robot_y;
		this.coral_offset = coral_offset;
		this.algae_offset = algae_offset;
		this.m_hasPiece = hasPiece;

		m_planner = new FieldPlanner(new Rebuilt2026(), new DriveTuningHeat(() -> m_drive.getPose()));
		m_behaviourManager = new BehaviourManager();
	}

	public Repulsor(
			DriveRepulsor drive,
			double robot_x,
			double robot_y,
			double coral_offset,
			double algae_offset,
			Supplier<Boolean> hasPiece) {
		this(drive, UsageType.kFullAuto, robot_x, robot_y, coral_offset, algae_offset, hasPiece);
	}

	public Repulsor withInitialNext(RepulsorSetpoint setpoint) {
		if (setpoint != null && setpoint.point().type() == SetpointType.kHumanPlayer) {
			throw new Error("Next score setpoint cannot be a human-player one");
		}
		m_nextScore = setpoint;
		return this;
	}

	public Repulsor withInitialHP(RepulsorSetpoint setpoint) {
		if (setpoint != null && setpoint.point().type() != SetpointType.kHumanPlayer) {
			throw new Error("Next collect setpoint must be a human-player/collect one");
		}
		m_nextScore = setpoint;
		return this;
	}

	public void setNextScore(RepulsorSetpoint next) {
		if (next != null && next.point().type() == SetpointType.kHumanPlayer) {
			throw new Error("Next score setpoint cannot be a human-player one");
		}
		m_nextScore = next;
	}

	public Repulsor withFallback(PlannerFallback fallback) {
		m_planner = m_planner.withFallback(fallback);
		return this;
	}

	public Repulsor withVision(Object vision) {
		return this;
	}

	public FieldPlanner getFieldPlanner() {
		return m_planner;
	}

	public VisionPlanner getVisionPlanner() {
		return m_visionPlanner;
	}

	public void update() {
		DeltaTime.update();

		StateManager.update(DeltaTime.get());

		boolean enabled = true;
		RepulsorDriverStation dsBase = RepulsorDriverStation.getInstance();
		if (dsBase instanceof NtRepulsorDriverStation ds) {
			enabled = ds.getConfigBool("force_controller_override");
		}

		if (enabled) {
			m_behaviourManager.stop();
			return;
		}

		m_behaviourManager.update(
				new BehaviourContext(
						this, m_planner, m_visionPlanner, m_drive, robot_x, robot_y, m_drive::getPose));
	}

	public DriveRepulsor getDrive() {
		return m_drive;
	}

	public void setup() {
		if (m_usageType != UsageType.kFullAuto)
			return;
	}

	private SetpointContext ctxFor(Pose2d robotPose, List<? extends Obstacle> dyn) {
		double len = Math.max(0.0, robot_x) * 2.0;
		double wid = Math.max(0.0, robot_y) * 2.0;
		double release = shooterReleaseHeightMeters == null ? 0.0 : Math.max(0.0, shooterReleaseHeightMeters.get());
		return new SetpointContext(Optional.ofNullable(robotPose), len, wid, release, dyn);
	}

	private Command alignCore(
			Supplier<RepulsorSetpoint> supplier,
			Optional<Trigger> untilOpt,
			CategorySpec cat,
			boolean suppressFallback) {
		final AtomicReference<RepulsorSetpoint> activeRef = new AtomicReference<>();
		final AtomicBoolean initialized = new AtomicBoolean(false);

		Command cmd = Commands.run(
				() -> {
					if (!initialized.get()) {
						activeRef.set(supplier.get());
						initialized.set(true);
					}

					m_planner.pollChosenSetpoint().ifPresent(activeRef::set);

					RepulsorSetpoint effective = activeRef.get();
					if (effective == null)
						return;

					m_currentGoal = effective;

					Pose2d robotPose = m_drive.getPose();
					List<? extends Obstacle> dyn = m_visionPlanner.getObstacles();
					Pose2d goalPose = effective.get(ctxFor(robotPose, dyn));
					m_planner.setRequestedGoal(goalPose);

					RepulsorSample sample = m_planner.calculate(
							robotPose,
							dyn,
							robot_x,
							robot_y,
							cat,
							suppressFallback,
							shooterReleaseHeightMeters == null
									? 0.0
									: Math.max(0.0, shooterReleaseHeightMeters.get()));

					m_planner
							.pollChosenSetpoint()
							.ifPresent(
									sp -> {
										activeRef.set(sp);
										m_currentGoal = sp;
										Pose2d g = sp.get(ctxFor(robotPose, dyn));
										m_planner.setRequestedGoal(g);
									});

					m_drive.runVelocity(
							sample.asChassisSpeeds(m_drive.getOmegaPID(), robotPose.getRotation()));
				},
				m_drive.asSubsystem())
				.finallyDo(interrupted -> m_drive.runVelocity(new ChassisSpeeds()));

		if (untilOpt.isPresent()) {
			cmd = cmd.until(untilOpt.get());
		}
		return cmd;
	}

	public Command alignTo(Supplier<RepulsorSetpoint> point, Trigger until, CategorySpec cat) {
		return alignCore(point, Optional.of(until), cat, false);
	}

	public Command alignTo(RepulsorSetpoint point, Trigger until, CategorySpec cat) {
		return alignCore(() -> point, Optional.of(until), cat, false);
	}

	public Command alignTo(RepulsorSetpoint point, CategorySpec cat) {
		return alignCore(() -> point, Optional.empty(), cat, false);
	}

	public Command alignTo(Supplier<RepulsorSetpoint> point, CategorySpec cat) {
		return alignCore(point, Optional.empty(), cat, false);
	}

	public Command alignTo(
			Supplier<RepulsorSetpoint> point, Trigger until, CategorySpec cat, boolean suppressFallback) {
		return alignCore(point, Optional.of(until), cat, suppressFallback);
	}

	public Command alignTo(
			RepulsorSetpoint point, Trigger until, CategorySpec cat, boolean suppressFallback) {
		return alignCore(() -> point, Optional.of(until), cat, suppressFallback);
	}

	public Command alignTo(RepulsorSetpoint point, CategorySpec cat, boolean suppressFallback) {
		return alignCore(() -> point, Optional.empty(), cat, suppressFallback);
	}

	public Command alignTo(
			Supplier<RepulsorSetpoint> point, CategorySpec cat, boolean suppressFallback) {
		return alignCore(point, Optional.empty(), cat, suppressFallback);
	}

	public void setCurrentGoal(RepulsorSetpoint sp) {
		m_currentGoal = sp;
	}

	public HeightSetpoint getTargetHeight() {
		return m_currentGoal == null ? HeightSetpoint.NONE : m_currentGoal.height();
	}

	private Trigger withinCore(
			Distance d, Optional<SetpointType> typeOpt, Optional<GameSetpoint> pointOpt) {
		return new Trigger(
				() -> {
					Optional<Distance> err = m_planner.getErr();
					if (err.isEmpty()) {
						return false;
					}
					boolean within = err.get().lt(d);
					if (typeOpt.isPresent()) {
						within = within && m_currentGoal != null && m_currentGoal.point().type() == typeOpt.get();
					}
					if (pointOpt.isPresent()) {
						within = within && m_currentGoal != null && m_currentGoal.point() == pointOpt.get();
					}
					return within;
				});
	}

	public Trigger within(Distance d) {
		return withinCore(d, Optional.empty(), Optional.empty());
	}

	public Trigger within(Distance d, SetpointType t) {
		return withinCore(d, Optional.of(t), Optional.empty());
	}

	public Trigger within(Distance d, GameSetpoint p) {
		return withinCore(d, Optional.empty(), Optional.of(p));
	}
}
