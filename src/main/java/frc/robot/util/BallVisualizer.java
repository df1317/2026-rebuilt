package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.List;

public class BallVisualizer {
	private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
	private static final Map<Integer, Pose3d> activeBalls = new HashMap<>();
	private static int ballCounter = 0;

	public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
		robotPoseSupplier = supplier;
	}

	private static void updateLog() {
		List<Pose3d> poses = new ArrayList<>(activeBalls.values());
		DogLog.log("BallVisualizer", poses.toArray(new Pose3d[0]));
	}

	public static Command shoot(Supplier<Double> launchSpeedMPS, Supplier<Double> launchAngleDeg) {
		return Commands.runOnce(() -> {
			final int ballId = ballCounter++;
			final double angleRad = Math.toRadians(launchAngleDeg.get());
			final double initialVelocity = launchSpeedMPS.get();

			final double vx = initialVelocity * Math.cos(angleRad);
			final double vz = initialVelocity * Math.sin(angleRad);
			final double g = 9.81;

			final double duration = (2.0 * vz) / g;

			Transform3d launcherTransform = new Transform3d(-0.1, 0, 0.4,
					new Rotation3d(0.0, -angleRad, Math.PI));

			final Pose3d startPose = new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
			final Timer timer = new Timer();
			timer.start();

			Command cmd = Commands.run(
					() -> {
						double t = timer.get();
						double distanceMeters = vx * t;
						// Add Pi to rotate 180 degrees since the shooter shoots out the back
						double headingRad = robotPoseSupplier.get().getRotation().getRadians() + Math.PI;
						double offsetX = distanceMeters * Math.cos(headingRad);
						double offsetY = distanceMeters * Math.sin(headingRad);
						double height = startPose.getZ() + (vz * t) - (0.5 * g * t * t);

						Translation3d currentPos = new Translation3d(
								startPose.getX() + offsetX,
								startPose.getY() + offsetY,
								Math.max(0.0, height));

						activeBalls.put(ballId, new Pose3d(currentPos, startPose.getRotation()));
						updateLog();
					})
					.until(() -> timer.hasElapsed(duration))
					.finallyDo(
							() -> {
								activeBalls.remove(ballId);
								updateLog();
							})
					.ignoringDisable(true);
			edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(cmd);
		});
	}
}
