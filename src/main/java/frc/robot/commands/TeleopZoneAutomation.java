package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.repulsor.Repulsor;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FieldZones;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

public class TeleopZoneAutomation {

	private static final double ALIGN_TOLERANCE_M = 0.15;
	private final Repulsor repulsor;
	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;
	private final HopperSubsystem hopper;
	private final Supplier<Pose2d> robotPose;

	public TeleopZoneAutomation(
			Repulsor repulsor,
			IntakeSubsystem intake,
			ShooterSubsystem shooter,
			HopperSubsystem hopper,
			Supplier<Pose2d> robotPose) {
		this.repulsor = repulsor;
		this.intake = intake;
		this.shooter = shooter;
		this.hopper = hopper;
		this.robotPose = robotPose;
	}

	public TeleopMode getMode() {
		if (DriverStation.isTest())
			return TeleopMode.SHOOT;
		DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
		return FieldZones.isInOwnAllianceZone(robotPose.get(), alliance) ? TeleopMode.SHOOT : TeleopMode.SHUTTLE;
	}

	public void configureTriggers(Trigger fireTrigger) {
		Trigger isTeleop = new Trigger(DriverStation::isTeleopEnabled);
		Trigger isShootMode = new Trigger(() -> getMode() == TeleopMode.SHOOT);

		// Fire button + shoot mode: shoot at hub distance with hopper feeding
		if (shooter != null && hopper != null) {
			// hub shooting
			isTeleop.and(fireTrigger).and(isShootMode)
					.whileTrue(shooter.shootForDistanceCommand(() -> {
						Translation2d pos = robotPose.get().getTranslation();
						DriverStation.Alliance alliance = DriverStation.getAlliance()
								.orElse(DriverStation.Alliance.Blue);
						Translation2d hub = FieldZones.getHubPose(alliance).getTranslation();
						return Meters.of(pos.getDistance(hub));
					}).alongWith(hopper.feedCommand()));

			// shuttle command
			isTeleop.and(fireTrigger).and(isShootMode.negate())
					.whileTrue(shooter.shootForDistanceCommand(() -> {
						Translation2d pos = robotPose.get().getTranslation();
						DriverStation.Alliance alliance = DriverStation.getAlliance()
								.orElse(DriverStation.Alliance.Blue);
						Translation2d shuttleSpot = FieldZones.getShuttlePose(alliance, pos).getTranslation();
						return Meters.of(pos.getDistance(shuttleSpot));
					}).alongWith(hopper.feedCommand()));
		}
	}

	public Pose2d getShootingPose() {
		Translation2d pos = robotPose.get().getTranslation();
		TeleopMode mode = getMode();
		DriverStation.Alliance alliance = DriverStation.getAlliance()
				.orElse(DriverStation.Alliance.Blue);
		if (mode == TeleopMode.SHOOT)
			return FieldZones.getHubPose(alliance);
		else
			return FieldZones.getShuttlePose(alliance, pos);
	}

	public enum TeleopMode {
		SHUTTLE, SHOOT
	}
}
