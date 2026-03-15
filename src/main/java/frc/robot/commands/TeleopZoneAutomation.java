package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.repulsor.Repulsor;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FieldZones;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

public class TeleopZoneAutomation {

	private static final double ALIGN_TOLERANCE_M = 0.15;

	private final Repulsor repulsor;
	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;
	private final HopperSubsystem hopper;
	private final Supplier<Pose2d> robotPose;
	private final Supplier<ChassisSpeeds> fieldVelocity;

	public TeleopZoneAutomation(
			Repulsor repulsor,
			IntakeSubsystem intake,
			ShooterSubsystem shooter,
			HopperSubsystem hopper,
			Supplier<Pose2d> robotPose,
			Supplier<ChassisSpeeds> fieldVelocity) {
		this.repulsor = repulsor;
		this.intake = intake;
		this.shooter = shooter;
		this.hopper = hopper;
		this.robotPose = robotPose;
		this.fieldVelocity = fieldVelocity;
	}

	public TeleopMode getMode() {
		if (DriverStation.isTest())
			return TeleopMode.SHOOT;
		DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
		return FieldZones.isInOwnAllianceZone(robotPose.get(), alliance) ? TeleopMode.SHOOT : TeleopMode.SHUTTLE;
	}

	public Command shootCommand() {
		if (shooter == null || hopper == null)
			return Commands.none();
		return Commands.parallel(
				shooter.shootForDistanceCommand(this::getTargetDistance),
				Commands.waitUntil(shooter::isAtSpeed).andThen(hopper.feedCommand()));
	}
  public Command shootCommand(BooleanSupplier aimed) {
    if (shooter == null || hopper == null)
      return Commands.none();
    return Commands.parallel(
      shooter.shootForDistanceCommand(this::getTargetDistance),
      Commands.waitUntil(()-> shooter.isAtSpeed() && aimed.getAsBoolean()).andThen(hopper.feedCommand()));
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

	public Distance getTargetDistance() {
		Translation2d pos = robotPose.get().getTranslation();
		Translation2d target = getShootingPose().getTranslation();
		double distance = pos.getDistance(target);
		double tof = distance / shooter.getHorizontalBallSpeedMPS(Meters.of(distance));
		ChassisSpeeds vel = fieldVelocity.get();
		Translation2d predictedPos = new Translation2d(
				pos.getX() + vel.vxMetersPerSecond * tof,
				pos.getY() + vel.vyMetersPerSecond * tof);
		return Meters.of(predictedPos.getDistance(target));
	}

	/**
	 * Returns a virtual aim target adjusted for robot velocity so the robot leads
	 * its shot when moving.
	 */
	public Pose2d getVirtualAimTarget() {
		Translation2d pos = robotPose.get().getTranslation();
		Translation2d target = getShootingPose().getTranslation();
		double distance = pos.getDistance(target);
		double tof = distance / shooter.getHorizontalBallSpeedMPS(Meters.of(distance));
		ChassisSpeeds vel = fieldVelocity.get();
		// Shift the aim target opposite to robot motion so the robot leads the shot
		Translation2d virtualTarget = new Translation2d(
				target.getX() - vel.vxMetersPerSecond * tof,
				target.getY() - vel.vyMetersPerSecond * tof);
		return new Pose2d(virtualTarget, new Rotation2d());
	}

	public enum TeleopMode {
		SHUTTLE, SHOOT
	}
}
