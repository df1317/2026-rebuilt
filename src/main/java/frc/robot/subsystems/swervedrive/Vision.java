package frc.robot.subsystems.swervedrive;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.util.RobotLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * PhotonVision-based vision system for AprilTag pose estimation. Multi-camera support with outlier rejection and
 * dynamic std devs.
 */
public class Vision {

	public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
			Constants.FIELD_LAYOUT);

	private final Supplier<Pose2d> currentPose;
	private final VisionTelemetry telemetry;
	public VisionSystemSim visionSim;

	@SuppressWarnings("resource")
	public Vision(Supplier<Pose2d> currentPose, Field2d field) {
		this.currentPose = currentPose;
		this.telemetry = new VisionTelemetry(field);

		if (Robot.isSimulation()) {
			visionSim = new VisionSystemSim("Vision");
			visionSim.addAprilTags(fieldLayout);

			for (Cameras c : Cameras.values()) {
				c.addToVisionSim(visionSim);
			}
		}

		boolean hasCamera = false;
		for (Cameras c : Cameras.values()) {
			if (c.camera.isConnected()) {
				hasCamera = true;
				break;
			}
		}
		RobotLog.setWarningAlert("Vision/NoClients", "No PhotonVision clients found", !hasCamera);
		if (!hasCamera) {
			RobotLog.warn("Vision/NoClientsNotify", "Vision warning",
					"No PhotonVision clients found; odometry may drift");
		}
	}

	private final Deque<Double> recentAcceptedMeasurements = new ArrayDeque<>();
	private static final double MEASUREMENT_WINDOW_SECONDS = 1.0;

	public void updatePoseEstimation(SwerveDrive swerveDrive) {
		if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
			visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
		}

		List<VisionMeasurement> measurements = new ArrayList<>();
		int connectedCameras = 0;

		for (Cameras camera : Cameras.values()) {
			if (!camera.camera.isConnected()) {
				continue;
			}
			connectedCameras++;

			camera.poseEstimator.addHeadingData(
					swerveDrive.getGyro().getYawAngularVelocity().in(edu.wpi.first.units.Units.RadiansPerSecond),
					swerveDrive.getOdometryHeading());
			Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
			if (poseEst.isPresent() && camera.curStdDevs != null) {
				measurements.add(new VisionMeasurement(poseEst.get(), camera.curStdDevs));
			}
		}

		DogLog.forceNt.log("Vision/ConnectedCameras", connectedCameras);

		measurements.sort(Comparator.comparingDouble(m -> m.pose.timestampSeconds));

		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		for (VisionMeasurement m : measurements) {
			swerveDrive.addVisionMeasurement(
					m.pose.estimatedPose.toPose2d(),
					m.pose.timestampSeconds,
					m.stdDevs);
			recentAcceptedMeasurements.add(now);
		}

		while (!recentAcceptedMeasurements.isEmpty()
				&& now - recentAcceptedMeasurements.peekFirst() > MEASUREMENT_WINDOW_SECONDS) {
			recentAcceptedMeasurements.pollFirst();
		}

		DogLog.forceNt.log("Vision/AcceptedMeasurements", recentAcceptedMeasurements.size());
		DogLog.forceNt.log("Vision/HasVision", hasVision());
	}

	public boolean hasVision() {
		return hasRecentVision(5.0);
	}

	/** Returns true if any camera has received a pose estimate within the given timeout. */
	public boolean hasRecentVision(double timeoutSeconds) {
		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		for (Cameras camera : Cameras.values()) {
			if (camera.lastPoseTimestamp > 0 && (now - camera.lastPoseTimestamp) < timeoutSeconds) {
				return true;
			}
		}
		return false;
	}

	private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
		Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose(currentPose.get());
		if (Robot.isSimulation()) {
			Field2d debugField = visionSim.getDebugField();
			poseEst.ifPresentOrElse(
					est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
					() -> debugField.getObject("VisionEstimation").setPoses());
		}
		return poseEst;
	}

	public double getDistanceFromAprilTag(int id) {
		Optional<Pose3d> tag = fieldLayout.getTagPose(id);
		return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
	}

	public void updateVisionField() {
		telemetry.update();
	}

	public VisionTelemetry getTelemetry() {
		return telemetry;
	}

	// named according to closest part of the robot
	// positive x is towards front of robot and positive y is perpendicularly left
	public enum Cameras {
		SHOOTER(
				"BOULDER",
				new Rotation3d(0, Units.degreesToRadians(15.0), 0),
				new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(-8.5), Units.inchesToMeters(10.5)),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.SINGLE_TAG[0],
						VisionConstants.CameraStdDevs.SINGLE_TAG[1],
						VisionConstants.CameraStdDevs.SINGLE_TAG[2]),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.MULTI_TAG[0],
						VisionConstants.CameraStdDevs.MULTI_TAG[1],
						VisionConstants.CameraStdDevs.MULTI_TAG[2])),

		BATTERY(
				"STONE",
				new Rotation3d(0, Units.degreesToRadians(15.0), Units.degreesToRadians(180)),
				new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(-8.5), Units.inchesToMeters(10.5)),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.SINGLE_TAG[0],
						VisionConstants.CameraStdDevs.SINGLE_TAG[1],
						VisionConstants.CameraStdDevs.SINGLE_TAG[2]),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.MULTI_TAG[0],
						VisionConstants.CameraStdDevs.MULTI_TAG[1],
						VisionConstants.CameraStdDevs.MULTI_TAG[2])),

		INTAKE(
				"PEBBLE",
				new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(90)),
				new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(3.5), Units.inchesToMeters(16.0)),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.SINGLE_TAG[0],
						VisionConstants.CameraStdDevs.SINGLE_TAG[1],
						VisionConstants.CameraStdDevs.SINGLE_TAG[2]),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.MULTI_TAG[0],
						VisionConstants.CameraStdDevs.MULTI_TAG[1],
						VisionConstants.CameraStdDevs.MULTI_TAG[2]));

		public final PhotonCamera camera;
		public final PhotonPoseEstimator poseEstimator;
		private final String cameraName;
		private final Matrix<N3, N1> singleTagStdDevs;
		private final Matrix<N3, N1> multiTagStdDevs;
		private final Transform3d robotToCamTransform;
		public Matrix<N3, N1> curStdDevs;
		public PhotonCameraSim cameraSim;
		public List<PhotonPipelineResult> resultsList = new ArrayList<>();
		private static final Matrix<N3, N1> STARTUP_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
		private static final double STARTUP_TRUST_DURATION_S = 5.0;
		private EstimatedRobotPose estimatedRobotPose;
		private double firstMeasurementTimestamp = -1;
		private double lastPoseTimestamp = -1;

		Cameras(
				String name,
				Rotation3d robotToCamRotation,
				Translation3d robotToCamTranslation,
				Matrix<N3, N1> singleTagStdDevs,
				Matrix<N3, N1> multiTagStdDevsMatrix) {
			this.cameraName = name;
			camera = new PhotonCamera(name);

			robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
			poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, robotToCamTransform);

			this.singleTagStdDevs = singleTagStdDevs;
			this.multiTagStdDevs = multiTagStdDevsMatrix;

			if (Robot.isSimulation()) {
				SimCameraProperties cameraProp = new SimCameraProperties();
				cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
				cameraProp.setCalibError(0.25, 0.08);
				cameraProp.setFPS(30);
				cameraProp.setAvgLatencyMs(35);
				cameraProp.setLatencyStdDevMs(5);

				cameraSim = new PhotonCameraSim(camera, cameraProp);
				cameraSim.enableDrawWireframe(true);
			}
		}

		public void addToVisionSim(VisionSystemSim systemSim) {
			if (Robot.isSimulation()) {
				systemSim.addCamera(cameraSim, robotToCamTransform);
			}
		}

		/** Returns the cached result with the least ambiguous best target. */
		public Optional<PhotonPipelineResult> getBestResult() {
			if (resultsList.isEmpty()) {
				return Optional.empty();
			}

			PhotonPipelineResult bestResult = null;
			double bestAmbiguity = Double.MAX_VALUE;
			for (PhotonPipelineResult result : resultsList) {
				if (!result.hasTargets()) {
					continue;
				}
				double ambiguity = result.getBestTarget().getPoseAmbiguity();
				if (ambiguity > 0 && ambiguity < bestAmbiguity) {
					bestResult = result;
					bestAmbiguity = ambiguity;
				}
			}
			return Optional.ofNullable(bestResult);
		}

		private boolean isInStartupWindow() {
			return firstMeasurementTimestamp < 0
					|| (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - firstMeasurementTimestamp) < STARTUP_TRUST_DURATION_S;
		}

		public boolean hasRecentPose() {
			return lastPoseTimestamp > 0
					&& (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - lastPoseTimestamp) < 0.5;
		}

		public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d referencePose) {
			updateUnreadResults(referencePose);
			return Optional.ofNullable(estimatedRobotPose);
		}

		private void updateUnreadResults(Pose2d referencePose) {
			List<PhotonPipelineResult> newResults = Robot.isReal()
					? camera.getAllUnreadResults()
					: cameraSim.getCamera().getAllUnreadResults();

			if (newResults.isEmpty()) {
				estimatedRobotPose = null;
				return;
			}

			resultsList.addAll(newResults);
			resultsList.sort(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds).reversed());
			if (resultsList.size() > 5) {
				resultsList = new ArrayList<>(resultsList.subList(0, 5));
			}

			PhotonPipelineResult latest = resultsList.get(0);
			double latencyMs = latest.metadata.getLatencyMillis();
			boolean highLatency = latencyMs > VisionConstants.HIGH_LATENCY_THRESHOLD_MS;
			RobotLog.setWarningAlert(
					"Vision/Latency/" + cameraName,
					"'" + cameraName + "' camera high latency (" + (int) latencyMs + "ms)",
					highLatency);

			updateEstimatedGlobalPose(referencePose);
		}

		private void updateEstimatedGlobalPose(Pose2d referencePose) {
			estimatedRobotPose = null;
			curStdDevs = singleTagStdDevs;

			Pose3d referencePose3d = new Pose3d(referencePose);

			for (var result : resultsList) {
				if (!result.hasTargets()) {
					continue;
				}

				var est = poseEstimator.estimateCoprocMultiTagPose(result);
				boolean isMultiTag = est.isPresent();

				if (est.isEmpty()) {
					if (!isInStartupWindow() && !DriverStation.isAutonomous()
							&& result.getBestTarget().getPoseAmbiguity() > 0.2) {
						continue;
					}
					est = poseEstimator.estimateClosestToReferencePose(result, referencePose3d);
				}

				if (est.isPresent()) {
					updateEstimationStdDevs(est.get(), result.getTargets(), isMultiTag);
					estimatedRobotPose = est.get();
					lastPoseTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
					return;
				}
			}
		}

		private void updateEstimationStdDevs(
				EstimatedRobotPose estimatedPose,
				List<PhotonTrackedTarget> targets,
				boolean isMultiTag) {
			var estStdDevs = isMultiTag ? multiTagStdDevs : singleTagStdDevs;
			int numTags = 0;
			double avgDist = 0;

			for (var tgt : targets) {
				var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
				if (tagPose.isEmpty()) {
					continue;
				}
				numTags++;
				avgDist += tagPose
						.get()
						.toPose2d()
						.getTranslation()
						.getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
			}

			if (numTags == 0) {
				curStdDevs = singleTagStdDevs;
				return;
			}

			avgDist /= numTags;

			if (!isInStartupWindow() && !DriverStation.isAutonomous()
					&& numTags == 1 && avgDist > VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS) {
				curStdDevs = null;
				return;
			}

			double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
			if (firstMeasurementTimestamp < 0) {
				firstMeasurementTimestamp = now;
			}
			if ((now - firstMeasurementTimestamp) < STARTUP_TRUST_DURATION_S) {
				curStdDevs = STARTUP_STD_DEVS;
				return;
			}

			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30));
			curStdDevs = estStdDevs;
		}
	}

	private record VisionMeasurement(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
	}
}
