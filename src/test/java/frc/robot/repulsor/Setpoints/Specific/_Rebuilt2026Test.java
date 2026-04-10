package frc.robot.repulsor.Setpoints.Specific;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.repulsor.Setpoints.SetpointContext;
import frc.robot.util.FieldFlip;
import org.junit.jupiter.api.Test;

public class _Rebuilt2026Test {
	@Test
	public void testOutpostCollectPoseFlipping() {
		Pose2d bluePose = _Rebuilt2026.OUTPOST_COLLECT.bluePose(SetpointContext.EMPTY);
		Pose2d redPose = _Rebuilt2026.OUTPOST_COLLECT.redPose(SetpointContext.EMPTY);

		System.out.println("Blue OUTPOST_COLLECT: " + bluePose);
		System.out.println("Red OUTPOST_COLLECT: " + redPose);

		// Print the actual tag locations
		System.out.println("Blue Outpost Tag 13: " + FieldFlip.aprilTagLayout().getTagPose(13).get().toPose2d());
		System.out.println("Red Outpost Tag 2: " + FieldFlip.aprilTagLayout().getTagPose(2).get().toPose2d());
		System.out.println("Red Outpost Tag 3: " + FieldFlip.aprilTagLayout().getTagPose(3).get().toPose2d());

		// Check what the red pose would be if calculated directly from the tag
		Pose2d redTag3Pose = FieldFlip.aprilTagLayout().getTagPose(3).get().toPose2d();
		Pose2d calculatedRedFromTag3 = new Pose2d(
				redTag3Pose.getTranslation()
						.plus(new edu.wpi.first.math.geometry.Translation2d(0.5, redTag3Pose.getRotation())),
				redTag3Pose.getRotation().plus(edu.wpi.first.math.geometry.Rotation2d.kPi));
		System.out.println("Calculated Red from Tag 3: " + calculatedRedFromTag3);
	}
}
