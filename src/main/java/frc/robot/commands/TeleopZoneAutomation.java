package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import frc.robot.repulsor.Repulsor;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FieldZones;
import frc.robot.util.GamePieceTracker;

public class TeleopZoneAutomation {

	public enum TeleopMode {
		SHUTTLE, SHOOT
	}

	private static final double ALIGN_TOLERANCE_M = 0.15;

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

		// Scoring zone: behind the hub on your own alliance wall side
		Trigger inScoringZone = new Trigger(() -> {
			DriverStation.Alliance alliance = DriverStation.getAlliance()
					.orElse(DriverStation.Alliance.Blue);
			Pose2d hub = FieldZones.getHubPose(alliance);
			double robotX = robotPose.get().getX();
			// Blue: scoring zone is x < hub_x (between wall and hub)
			// Red: scoring zone is x > hub_x (between wall and hub)
			return alliance == DriverStation.Alliance.Red
					? robotX > hub.getX()
					: robotX < hub.getX();
		});

		// Shuttle mode: auto-intake when in own zone without piece
		if (intake != null) {
			isTeleop.and(isShuttleMode).and(inOwnZone).and(noPiece)
					.whileTrue(Commands.sequence(
							Commands.runOnce(tracker::startIntake),
							intake.extendCommand(),
							intake.runRollerCommand()));

			// Auto-retract intake when piece acquired
			isTeleop.and(isShuttleMode).and(hasPiece)
					.onTrue(Commands.sequence(
							Commands.runOnce(tracker::stopIntake),
							intake.retractCommand(),
							intake.stopRollerCommand()));
		}

		// Auto-fire when in scoring zone with piece (distance-based)
		if (shooter != null) {
			isTeleop.and(inScoringZone).and(hasPiece)
					.whileTrue(shooter.shootForDistanceCommand(() -> {
						Translation2d pos = robotPose.get().getTranslation();
						DriverStation.Alliance alliance = DriverStation.getAlliance()
								.orElse(DriverStation.Alliance.Blue);
						Translation2d hub = FieldZones.getHubPose(alliance).getTranslation();
						return Meters.of(pos.getDistance(hub));
					}));

			isTeleop.and(inScoringZone).and(hasPiece)
					.and(new Trigger(shooter::isAtSpeed))
					.onTrue(Commands.sequence(
							Commands.runOnce(tracker::startShoot),
							Commands.waitSeconds(0.5),
							Commands.runOnce(tracker::stopShoot)));
		}
	}
}
