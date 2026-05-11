package frc.robot.repulsor.Setpoints.Specific;

import frc.robot.util.FieldFlip;
import org.junit.jupiter.api.Test;

public class TagLayoutTest {
	@Test
	public void printAllTags() {
		System.out.println("Field Length: " + FieldFlip.fieldLength());
		System.out.println("Field Width: " + FieldFlip.fieldWidth());
		for (edu.wpi.first.apriltag.AprilTag tag : FieldFlip.aprilTagLayout().getTags()) {
			System.out.println("Tag " + tag.ID + ": " + tag.pose.toPose2d());
		}
	}
}
