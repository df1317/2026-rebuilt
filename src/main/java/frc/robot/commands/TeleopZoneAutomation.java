package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import frc.robot.repulsor.Repulsor;
import frc.robot.repulsor.Fields.FieldMapBuilder.CategorySpec;
import frc.robot.repulsor.Setpoints.HeightSetpoint;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FieldZones;
import frc.robot.util.GamePieceTracker;

public class TeleopZoneAutomation {

	public enum TeleopMode {
		SHUTTLE, SHOOT
	}

	private static final double SCORE_PROXIMITY_M = 1.0;
	private static final double ALIGN_TOLERANCE_M = 0.15;
	private static final double SHOOT_RPM = 3000.0;

	private final Repulsor repulsor;
	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;
	private final GamePieceTracker tracker;
	private final Supplier<Pose2d> robotPose;
	private final SendableChooser<TeleopMode> modeChooser;

	public TeleopZoneAutomation(
			Repulsor repulsor,
			IntakeSubsystem intake,
			ShooterSubsystem shooter,
			GamePieceTracker tracker,
			Supplier<Pose2d> robotPose) {
		this.repulsor = repulsor;
		this.intake = intake;
		this.shooter = shooter;
		this.tracker = tracker;
		this.robotPose = robotPose;

		modeChooser = new SendableChooser<>();
		modeChooser.setDefaultOption("Shuttle", TeleopMode.SHUTTLE);
		modeChooser.addOption("Shoot", TeleopMode.SHOOT);
		SmartDashboard.putData("misc/Teleop Mode", modeChooser);
	}

	public TeleopMode getMode() {
		return modeChooser.getSelected();
	}

	public void configureTriggers() {
		Trigger isTeleop = new Trigger(DriverStation::isTeleopEnabled);
		Trigger hasPiece = new Trigger(tracker::get);
		Trigger noPiece = hasPiece.negate();

		Trigger isShuttleMode = new Trigger(() -> getMode() == TeleopMode.SHUTTLE);

		Trigger inOwnZone = new Trigger(() -> {
			DriverStation.Alliance alliance = DriverStation.getAlliance()
					.orElse(DriverStation.Alliance.Blue);
			return FieldZones.isInOwnAllianceZone(robotPose.get(), alliance);
		});

		Trigger nearScoringPose = new Trigger(() -> {
			Translation2d pos = robotPose.get().getTranslation();
			var nearest = _Rebuilt2026.nearestScoringPose(pos);
			Pose2d scorePose = nearest.approximateBluePose();
			return pos.getDistance(scorePose.getTranslation()) < SCORE_PROXIMITY_M;
		});

		Trigger aligned = new Trigger(() -> repulsor.within(Meters.of(ALIGN_TOLERANCE_M)).getAsBoolean());

		// Shuttle mode: auto-intake when in own zone without piece
		if (intake != null) {
			isTeleop.and(isShuttleMode).and(inOwnZone).and(noPiece)
					.whileTrue(Commands.parallel(
							intake.extendCommand(),
							intake.runRollerCommand(),
							Commands.runOnce(tracker::startIntake)));

			// Auto-retract intake when piece acquired
			isTeleop.and(isShuttleMode).and(hasPiece)
					.onTrue(Commands.sequence(
							Commands.runOnce(tracker::stopIntake),
							intake.retractCommand(),
							intake.stopRollerCommand()));
		}

		// Both modes: auto-align when near scoring pose with piece
		isTeleop.and(nearScoringPose).and(hasPiece)
				.whileTrue(Commands.defer(() -> {
					var nearest = _Rebuilt2026.nearestScoringPose(robotPose.get().getTranslation());
					var sp = new RepulsorSetpoint(nearest, HeightSetpoint.NET);
					return repulsor.alignTo(sp, CategorySpec.kScore);
				}, java.util.Set.of(repulsor.getDrive().asSubsystem())));

		// Auto-fire when aligned
		if (shooter != null) {
			isTeleop.and(nearScoringPose).and(hasPiece).and(aligned)
					.onTrue(Commands.sequence(
							Commands.runOnce(tracker::startShoot),
							shooter.shootCommand(RPM.of(SHOOT_RPM))
									.withTimeout(1.0),
							Commands.runOnce(tracker::stopShoot),
							shooter.stopCommand()));
		}
	}
}
