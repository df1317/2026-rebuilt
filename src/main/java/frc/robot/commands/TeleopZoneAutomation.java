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

public class TeleopZoneAutomation {

	public enum TeleopMode {
		SHUTTLE, SHOOT
	}

	private static final double ALIGN_TOLERANCE_M = 0.15;

	private final Repulsor repulsor;
	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;
	private final Supplier<Pose2d> robotPose;
	private final SendableChooser<TeleopMode> modeChooser;

	public TeleopZoneAutomation(
			Repulsor repulsor,
			IntakeSubsystem intake,
			ShooterSubsystem shooter,
			Supplier<Pose2d> robotPose) {
		this.repulsor = repulsor;
		this.intake = intake;
		this.shooter = shooter;
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

		// Shuttle mode: auto-intake when in own zone
		if (intake != null) {
			isTeleop.and(isShuttleMode).and(inOwnZone)
					.whileTrue(Commands.sequence(
							intake.extendCommand(),
							intake.runRollerCommand()));
		}

		// Auto-fire when in scoring zone (distance-based)
		if (shooter != null) {
			isTeleop.and(inScoringZone)
					.whileTrue(shooter.shootForDistanceCommand(() -> {
						Translation2d pos = robotPose.get().getTranslation();
						DriverStation.Alliance alliance = DriverStation.getAlliance()
								.orElse(DriverStation.Alliance.Blue);
						Translation2d hub = FieldZones.getHubPose(alliance).getTranslation();
						return Meters.of(pos.getDistance(hub));
					}));
		}
	}
}
