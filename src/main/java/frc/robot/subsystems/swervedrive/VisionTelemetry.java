package frc.robot.subsystems.swervedrive;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.util.DevMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Handles dev-only vision debug telemetry.
 *
 * <p>
 * This class extracts debug visualization logic from Vision to avoid class
 * pollution.
 * Field2d overlays only run when {@link DevMode#isEnabled()} returns true.
 *
 * <p>
 * Logging uses DogLog, which automatically handles dev vs. competition:
 * <ul>
 * <li>Always logs to DataLog (.wpilog) for AdvantageScope</li>
 * <li>NT publishing auto-disabled when FMS connected</li>
 * </ul>
 */
public class VisionTelemetry {

	private final Field2d field2d;
	private final BooleanEntry showTrackedTargetsToggle;

	/**
	 * Constructs a new VisionTelemetry instance for debug visualization.
	 *
	 * @param field
	 *          the Field2d object to display vision targets on
	 */
	public VisionTelemetry(Field2d field) {
		this.field2d = field;

		// Create dashboard toggle - BooleanEntry supports both get() and set()
		var table = NetworkTableInstance.getDefault().getTable("Vision");
		showTrackedTargetsToggle = table.getBooleanTopic("ShowTrackedTargets").getEntry(false);
	}

	/**
	 * Update Field2d to display tracked AprilTag targets and log vision debug data.
	 * Field2d updates only run in dev mode; DogLog handles NT publishing
	 * automatically.
	 */
	public void update() {
		List<PhotonTrackedTarget> targets = collectTrackedTargets();

		// Log target count - DogLog auto-disables NT at competition
		DogLog.log("Vision/TrackedTargetCount", targets.size());

		// Read toggle value from the dashboard
		boolean showTrackedTargets = showTrackedTargetsToggle.get();

		// Field2d overlays only in dev mode (expensive, not needed at competition)
		if (!DevMode.isEnabled()) {
			field2d.getObject("tracked targets").setPoses();
			return;
		}

		if (!showTrackedTargets) {
			field2d.getObject("tracked targets").setPoses();
			return;
		}

		List<Pose2d> poses = new ArrayList<>();
		for (PhotonTrackedTarget target : targets) {
			Optional<Pose3d> tagPose = Vision.fieldLayout.getTagPose(target.getFiducialId());
			tagPose.ifPresent(pose3d -> poses.add(pose3d.toPose2d()));
		}

		field2d.getObject("tracked targets").setPoses(poses);

		// Log tracked tag IDs for AdvantageScope analysis
		int[] tagIds = targets.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
		DogLog.log("Vision/TrackedTagIds", tagIds);
	}

	private List<PhotonTrackedTarget> collectTrackedTargets() {
		List<PhotonTrackedTarget> targets = new ArrayList<>();
		for (Cameras c : Cameras.values()) {
			if (!c.resultsList.isEmpty()) {
				PhotonPipelineResult latest = c.resultsList.get(0);
				if (latest.hasTargets()) {
					targets.addAll(latest.targets);
				}
			}
		}
		return targets;
	}

	/**
	 * Enable/disable tracked target visualization on Field2d.
	 *
	 * @param show
	 *          true to show tracked targets, false to hide
	 */
	public void setShowTrackedTargets(boolean show) {
		showTrackedTargetsToggle.set(show);
	}

	/**
	 * Get current state of tracked target visualization toggle.
	 *
	 * @return true if showing tracked targets, false otherwise
	 */
	public boolean isShowingTrackedTargets() {
		return showTrackedTargetsToggle.get();
	}
}
