package frc.robot.repulsor.Tracking.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.repulsor.Tracking.FieldTrackerCore;

public class FieldVision {
	private static final String OBJECT_TYPE = "fuel";
	private static final double OBJECT_HEIGHT_METERS = 0.12;

	private final FieldTrackerCore owner;
	private final PhotonCamera camera;
	private final Transform3d robotToCamera;

	public FieldVision(FieldTrackerCore owner, String cameraName, Transform3d robotToCamera) {
		if (owner == null)
			throw new IllegalArgumentException("owner cannot be null");
		if (cameraName == null || cameraName.isEmpty())
			throw new IllegalArgumentException("cameraName cannot be null/empty");
		this.owner = owner;
		this.camera = new PhotonCamera(cameraName);
		this.robotToCamera = robotToCamera;
	}

	public String getName() {
		return camera.getName();
	}

	public void update(Pose2d currentPose) {
		if (currentPose == null)
			return;

		List<PhotonPipelineResult> results = camera.getAllUnreadResults();
		if (results.isEmpty())
			return;

		PhotonPipelineResult latest = results.get(results.size() - 1);
		if (!latest.hasTargets())
			return;

		Pose3d fieldToRobot = new Pose3d(
				currentPose.getX(), currentPose.getY(), 0.0,
				new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));

		Pose3d fieldToCamera = fieldToRobot.transformBy(robotToCamera);
		double cameraZ = fieldToCamera.getZ();
		double cameraPitch = fieldToCamera.getRotation().getY();

		long nowNs = System.nanoTime();
		int idx = 0;

		for (PhotonTrackedTarget target : latest.getTargets()) {
			double yawRad = Math.toRadians(target.getYaw());
			double pitchRad = Math.toRadians(target.getPitch());

			// Ray angle downward from camera optical axis
			double totalPitch = cameraPitch + pitchRad;

			// Distance along ground from camera to object
			double heightAboveObject = cameraZ - OBJECT_HEIGHT_METERS;
			if (heightAboveObject <= 0 || totalPitch >= 0)
				continue;

			double groundDist = heightAboveObject / Math.tan(-totalPitch);
			if (groundDist <= 0 || groundDist > 8.0)
				continue;

			// Object position in field frame
			double cameraYaw = fieldToCamera.getRotation().getZ();
			double objectYaw = cameraYaw + yawRad;
			double fieldX = fieldToCamera.getX() + groundDist * Math.cos(objectYaw);
			double fieldY = fieldToCamera.getY() + groundDist * Math.sin(objectYaw);

			Pose3d fieldPose = new Pose3d(fieldX, fieldY, OBJECT_HEIGHT_METERS, new Rotation3d());
			owner.ingestTracked("pv_" + idx, OBJECT_TYPE, fieldPose, nowNs);
			idx++;
		}
	}
}
