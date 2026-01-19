package frc.robot.subsystems.swervedrive;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

/**
 * PhotonVision-based vision system for improving odometry accuracy through AprilTag detection.
 *
 * <p>
 * This class manages multiple PhotonVision cameras and fuses their pose estimates with swerve drive odometry using a
 * pose estimator. It includes outlier rejection, dynamic standard deviation calculation, and simulation support.
 *
 * <h2>Features:</h2>
 * <ul>
 * <li>Multi-camera support with independent pose estimation</li>
 * <li>Outlier rejection based on pose jump distance (see {@link frc.robot.Constants.VisionConstants})</li>
 * <li>Dynamic standard deviations based on tag count and distance</li>
 * <li>Single-tag filtering by ambiguity and distance thresholds</li>
 * <li>PhotonVision simulation integration</li>
 * </ul>
 *
 * <h2>Usage:</h2>
 *
 * <pre>
 * `Vision vision = new Vision(swerveDrive::getPose, swerveDrive.field);`
 * // In periodic:
 * vision.updatePoseEstimation(swerveDrive);
 * </pre>
 *
 * <p>
 * Adapted from <a href="https://gitlab.com/ironclad_code/ironclad-2024">Ironclad 2024</a>
 *
 * @see frc.robot.Constants.VisionConstants
 * @see org.photonvision.PhotonPoseEstimator
 */
public class Vision {

	/**
	 * April Tag Field Layout of the year.
	 */
	public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
			AprilTagFields.k2025ReefscapeWelded);

	/**
	 * Current pose from the pose estimator using wheel odometry.
	 */
	private final Supplier<Pose2d> currentPose;
	/**
	 * Dev-only telemetry handler for vision debug visualization.
	 */
	private final VisionTelemetry telemetry;
	/**
	 * Photon Vision Simulation
	 */
	public VisionSystemSim visionSim;

	/**
	 * Constructor for the Vision class.
	 *
	 * @param currentPose
	 *          Current pose supplier, should reference {@link SwerveDrive#getPose()}
	 * @param field
	 *          Current field, should be {@link SwerveDrive#field}
	 */
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

		// Check if any cameras are connected
		boolean hasCamera = false;
		for (Cameras c : Cameras.values()) {
			if (c.camera.isConnected()) {
				hasCamera = true;
				break;
			}
		}
		RobotLog.setWarningAlert("Vision/NoClients", "No PhotonVision clients found", !hasCamera);
		if (!hasCamera) {
			RobotLog.warn(
					"Vision/NoClientsNotify",
					"Vision warning",
					"No PhotonVision clients found; odometry may drift");
		}
	}

	/**
	 * Updates pose estimation by fusing vision measurements from all connected cameras with the swerve drive's wheel
	 * odometry.
	 *
	 * <p>
	 * This method should be called in the subsystem's periodic() after manually calling
	 * {@code swerveDrive.updateOdometry()} to ensure proper ordering of updates.
	 *
	 * <p>
	 * <b>Processing steps:</b>
	 * <ol>
	 * <li>Collects pose estimates from all connected cameras</li>
	 * <li>Sorts measurements by timestamp (oldest first)</li>
	 * <li>Applies outlier rejection based on pose jump distance</li>
	 * <li>Adds valid measurements to the swerve drive's pose estimator with calculated standard deviations</li>
	 * </ol>
	 *
	 * @param swerveDrive
	 *          The swerve drive instance to update with vision measurements
	 */
	public void updatePoseEstimation(SwerveDrive swerveDrive) {
		if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
			/*
			 * In the maple-sim, odometry is simulated using encoder values, accounting for
			 * factors like skidding and drifting.
			 * As a result, the odometry may not always be 100% accurate.
			 * However, the vision system should be able to provide a reasonably accurate
			 * pose estimation, even when odometry is incorrect.
			 * (This is why teams implement a vision system to correct odometry.)
			 * Therefore, we must ensure that the actual robot pose is provided in the
			 * simulator when updating the vision simulation during the simulation.
			 */
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

		DogLog.log("Vision/ConnectedCameras", connectedCameras);

		measurements.sort(Comparator.comparingDouble(m -> m.pose.timestampSeconds));

		// Feed all measurements to the Kalman filter - it handles outlier weighting via std devs
		for (VisionMeasurement m : measurements) {
			swerveDrive.addVisionMeasurement(
					m.pose.estimatedPose.toPose2d(),
					m.pose.timestampSeconds,
					m.stdDevs);
		}

		DogLog.log("Vision/AcceptedMeasurements", measurements.size());
	}

	/**
	 * Retrieves the estimated robot pose from a specific camera and updates simulation debug visualization.
	 *
	 * <p>
	 * In simulation mode, this also updates the Field2d debug field with the vision estimate for visualization in
	 * AdvantageScope or Glass.
	 *
	 * @param camera
	 *          The camera to get the pose estimate from
	 * @return Optional containing the estimated pose, or empty if no valid estimate available
	 */
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

	/**
	 * Calculates the distance from the robot's current pose to a specific AprilTag.
	 *
	 * <p>
	 * Useful for game-specific logic that depends on proximity to field elements (e.g., scoring positions, pickup
	 * zones).
	 *
	 * @param id
	 *          The AprilTag ID to measure distance to
	 * @return Distance in meters, or -1.0 if the tag ID is not found in the field layout
	 */
	public double getDistanceFromAprilTag(int id) {
		Optional<Pose3d> tag = fieldLayout.getTagPose(id);
		return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
	}

	/**
	 * Updates dev-only vision telemetry (tracked targets on Field2d).
	 *
	 * <p>
	 * This visualization shows detected AprilTags and their pose estimates on the field. Should only be called in
	 * development mode to avoid NetworkTables traffic during competition.
	 *
	 * @see VisionTelemetry
	 */
	public void updateVisionField() {
		telemetry.update();
	}

	/**
	 * Gets the vision telemetry handler for external configuration.
	 *
	 * <p>
	 * Allows other classes to access vision debug information and configure telemetry settings.
	 *
	 * @return The vision telemetry instance
	 */
	public VisionTelemetry getTelemetry() {
		return telemetry;
	}

	/**
	 * Enum defining all PhotonVision cameras on the robot.
	 *
	 * <p>
	 * Each camera entry specifies:
	 * <ul>
	 * <li>Camera name (must match PhotonVision UI)</li>
	 * <li>Robot-to-camera transform (rotation and translation)</li>
	 * <li>Standard deviations for single-tag and multi-tag estimates</li>
	 * </ul>
	 *
	 * <p>
	 * To add a new camera, create a new enum constant with its configuration.
	 */
	public enum Cameras {
		/**
		 * Center Camera
		 */
		CENTER_CAM(
				"PEBBLE",
				new Rotation3d(0, Units.degreesToRadians(15.0), 0),
				new Translation3d(0.32, 0.32, 0.30),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.SINGLE_TAG[0],
						VisionConstants.CameraStdDevs.SINGLE_TAG[1],
						VisionConstants.CameraStdDevs.SINGLE_TAG[2]),
				VecBuilder.fill(
						VisionConstants.CameraStdDevs.MULTI_TAG[0],
						VisionConstants.CameraStdDevs.MULTI_TAG[1],
						VisionConstants.CameraStdDevs.MULTI_TAG[2]));

		/**
		 * Camera name for alert keys.
		 */
		private final String cameraName;
		/**
		 * Camera instance for comms.
		 */
		public final PhotonCamera camera;
		/**
		 * Pose estimator for camera.
		 */
		public final PhotonPoseEstimator poseEstimator;
		/**
		 * Standard Deviation for single tag readings for pose estimation.
		 */
		private final Matrix<N3, N1> singleTagStdDevs;
		/**
		 * Standard deviation for multi-tag readings for pose estimation.
		 */
		private final Matrix<N3, N1> multiTagStdDevs;
		/**
		 * Transform of the camera rotation and translation relative to the center of the robot
		 */
		private final Transform3d robotToCamTransform;
		/**
		 * Last read from the camera timestamp to prevent lag due to slow data fetches.
		 */
		private final double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
		/**
		 * Current standard deviations used.
		 */
		public Matrix<N3, N1> curStdDevs;
		/**
		 * Simulated camera instance which only exists during simulations.
		 */
		public PhotonCameraSim cameraSim;
		/**
		 * Results list to be updated periodically and cached to avoid unnecessary queries.
		 */
		public List<PhotonPipelineResult> resultsList = new ArrayList<>();
		/**
		 * Estimated robot pose (null if no valid estimate).
		 */
		private EstimatedRobotPose estimatedRobotPose;

		/**
		 * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
		 * estimation noise on an actual robot.
		 *
		 * @param name
		 *          Name of the PhotonVision camera found in the PV UI.
		 * @param robotToCamRotation
		 *          {@link Rotation3d} of the camera.
		 * @param robotToCamTranslation
		 *          {@link Translation3d} relative to the center of the robot.
		 * @param singleTagStdDevs
		 *          Single AprilTag standard deviations of estimated poses from the camera.
		 * @param multiTagStdDevsMatrix
		 *          Multi AprilTag standard deviations of estimated poses from the camera.
		 */
		Cameras(
				String name,
				Rotation3d robotToCamRotation,
				Translation3d robotToCamTranslation,
				Matrix<N3, N1> singleTagStdDevs,
				Matrix<N3, N1> multiTagStdDevsMatrix) {
			this.cameraName = name;
			camera = new PhotonCamera(name);

			// https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
			robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

			poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, robotToCamTransform);

			this.singleTagStdDevs = singleTagStdDevs;
			this.multiTagStdDevs = multiTagStdDevsMatrix;

			if (Robot.isSimulation()) {
				SimCameraProperties cameraProp = new SimCameraProperties();
				// A 640 x 480 camera with a 100-degree diagonal FOV.
				cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
				// Approximate detection noise with average and standard deviation error in
				// pixels.
				cameraProp.setCalibError(0.25, 0.08);
				// Set the camera image capture framerate (Note: this is limited by robot loop
				// rate).
				cameraProp.setFPS(30);
				// The average and standard deviation in milliseconds of image data latency.
				cameraProp.setAvgLatencyMs(35);
				cameraProp.setLatencyStdDevMs(5);

				cameraSim = new PhotonCameraSim(camera, cameraProp);
				cameraSim.enableDrawWireframe(true);
			}
		}

		/**
		 * Add camera to {@link VisionSystemSim} for simulated photon vision.
		 *
		 * @param systemSim
		 *          {@link VisionSystemSim} to use.
		 */
		public void addToVisionSim(VisionSystemSim systemSim) {
			if (Robot.isSimulation()) {
				systemSim.addCamera(cameraSim, robotToCamTransform);
			}
		}

		/**
		 * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
		 * recent result!
		 *
		 * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
		 */
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

		/**
		 * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
		 * cache of results.
		 *
		 * @param referencePose
		 *          Current robot pose from odometry for reference
		 * @return Estimated pose.
		 */
		public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d referencePose) {
			updateUnreadResults(referencePose);
			return Optional.ofNullable(estimatedRobotPose);
		}

		/**
		 * Update the latest results from the camera. Always fetches new results and keeps the most recent one with
		 * targets.
		 */
		private void updateUnreadResults(Pose2d referencePose) {
			List<PhotonPipelineResult> newResults = Robot.isReal()
					? camera.getAllUnreadResults()
					: cameraSim.getCamera().getAllUnreadResults();

			// Add new results to cache
			if (!newResults.isEmpty()) {
				resultsList.addAll(newResults);
				// Sort by timestamp (newest first) and keep only recent results
				resultsList.sort(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds).reversed());
				// Keep only the 5 most recent results to prevent memory growth
				if (resultsList.size() > 5) {
					resultsList = new ArrayList<>(resultsList.subList(0, 5));
				}

				// Check for high latency on the most recent result
				PhotonPipelineResult latest = resultsList.get(0);
				double latencyMs = latest.metadata.getLatencyMillis();
				boolean highLatency = latencyMs > VisionConstants.HIGH_LATENCY_THRESHOLD_MS;
				RobotLog.setWarningAlert(
						"Vision/Latency/" + cameraName,
						"'" + cameraName + "' camera high latency (" + (int) latencyMs + "ms)",
						highLatency);
			}

			// Process results if we have any
			if (resultsList.isEmpty()) {
				estimatedRobotPose = null;
				curStdDevs = singleTagStdDevs;
			} else {
				updateEstimatedGlobalPose(referencePose);
			}
		}

		/**
		 * Updates the estimated robot pose from vision data. Also updates standard deviations based on tag count and
		 * distance.
		 *
		 * @param referencePose
		 *          Current robot pose from odometry for single-tag disambiguation.
		 */
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
					if (result.getBestTarget().getPoseAmbiguity() > 0.2) {
						continue;
					}
					est = poseEstimator.estimateClosestToReferencePose(result, referencePose3d);
				}

				if (est.isPresent()) {
					updateEstimationStdDevs(est.get(), result.getTargets(), isMultiTag);
					estimatedRobotPose = est.get();
					return;
				}
			}
		}

		/**
		 * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic standard deviations based
		 * on the number of tags, estimation strategy, and distance from the tags.
		 *
		 * @param estimatedPose
		 *          The estimated pose to calculate standard deviations for.
		 * @param targets
		 *          All targets in this camera frame
		 * @param isMultiTag
		 *          Whether this estimate came from multi-tag solving
		 */
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

			if (numTags == 1 && avgDist > VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS) {
				curStdDevs = null;
				return;
			}

			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30));
			curStdDevs = estStdDevs;
		}
	}

	private record VisionMeasurement(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
	}
}
