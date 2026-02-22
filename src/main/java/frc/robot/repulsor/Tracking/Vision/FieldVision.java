package frc.robot.repulsor.Tracking.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Set;
import frc.robot.repulsor.Tracking.FieldTrackerCore;

public class FieldVision {
	private static final int MAX_OBJECTS_PER_TICK = 256;

	private final FieldTrackerCore owner;
	private final String name;
	private final NetworkTable table;

	public FieldVision(FieldTrackerCore owner, String name) {
		if (owner == null)
			throw new IllegalArgumentException("owner cannot be null");
		if (name == null || name.isEmpty())
			throw new IllegalArgumentException("name cannot be null/empty");
		this.owner = owner;
		this.name = name;
		this.table = NetworkTableInstance.getDefault().getTable("FieldVision/" + name);
	}

	public String getName() {
		return name;
	}

	public void update(Pose2d currentPose) {
		if (currentPose == null)
			return;

		Pose3d field_T_robot = new Pose3d(
				currentPose.getX(), currentPose.getY(), 0.0,
				new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));

		double ex = table.getEntry("extrinsics/x").getDouble(0.0);
		double ey = table.getEntry("extrinsics/y").getDouble(0.0);
		double ez = table.getEntry("extrinsics/z").getDouble(0.0);
		double eroll = table.getEntry("extrinsics/roll").getDouble(0.0);
		double epitch = table.getEntry("extrinsics/pitch").getDouble(0.0);
		double eyaw = table.getEntry("extrinsics/yaw").getDouble(0.0);

		Transform3d robot_T_camera = new Transform3d(
				new Translation3d(ex, ey, ez), new Rotation3d(eroll, epitch, eyaw));

		Pose3d field_T_camera = field_T_robot.transformBy(
				new Transform3d(robot_T_camera.getTranslation(), robot_T_camera.getRotation()));

		Set<String> keys = table.getKeys();
		int seen = 0;
		long nowNs = System.nanoTime();

		for (String key : keys) {
			if (seen >= MAX_OBJECTS_PER_TICK)
				break;
			if (!key.startsWith("object_"))
				continue;

			String frame = table.getEntry(key + "/frame").getString("field");
			Rotation3d localRot = new Rotation3d(
					table.getEntry(key + "/roll").getDouble(0.0),
					table.getEntry(key + "/pitch").getDouble(0.0),
					table.getEntry(key + "/yaw").getDouble(0.0));

			String rawType = table.getEntry(key + "/type").getString("unknown");

			Pose3d fieldPose;

			if ("camera".equalsIgnoreCase(frame)) {
				double px = table.getEntry(key + "/px").getDouble(0.0);
				double py = table.getEntry(key + "/py").getDouble(0.0);
				double pz = table.getEntry(key + "/pz").getDouble(0.0);
				Pose3d camera_T_object = new Pose3d(px, py, pz, localRot);
				fieldPose = field_T_camera.transformBy(
						new Transform3d(camera_T_object.getTranslation(), camera_T_object.getRotation()));
			} else if ("robot".equalsIgnoreCase(frame)) {
				double px = table.getEntry(key + "/px").getDouble(0.0);
				double py = table.getEntry(key + "/py").getDouble(0.0);
				double pz = table.getEntry(key + "/pz").getDouble(0.0);
				Pose3d robot_T_object = new Pose3d(px, py, pz, localRot);
				fieldPose = field_T_robot.transformBy(
						new Transform3d(robot_T_object.getTranslation(), robot_T_object.getRotation()));
			} else {
				double x = table.getEntry(key + "/x").getDouble(0.0);
				double y = table.getEntry(key + "/y").getDouble(0.0);
				double z = table.getEntry(key + "/z").getDouble(0.0);
				fieldPose = new Pose3d(x, y, z, localRot);
			}

			owner.ingestTracked(key.substring(7), rawType, fieldPose, nowNs);
			seen++;
		}
	}
}
