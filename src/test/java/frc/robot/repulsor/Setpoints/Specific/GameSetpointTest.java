package frc.robot.repulsor.Setpoints;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.repulsor.Setpoints.Specific._Rebuilt2026;
import org.junit.jupiter.api.Test;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameSetpointTest {
	@Test
	public void testWithRotationOffset() {
		GameSetpoint base = _Rebuilt2026.OUTPOST_COLLECT;
		GameSetpoint offset = base.withRotationOffset(Rotation2d.fromDegrees(90));

		System.out.println("Base Blue: " + base.bluePose(SetpointContext.EMPTY));
		System.out.println("Base Red: " + base.redPose(SetpointContext.EMPTY));

		System.out.println("Offset Blue: " + offset.bluePose(SetpointContext.EMPTY));
		System.out.println("Offset Red: " + offset.redPose(SetpointContext.EMPTY));

		System.out.println("Offset poseForAlliance(Blue): " + offset.poseForAlliance(Alliance.Blue, SetpointContext.EMPTY));
		System.out.println("Offset poseForAlliance(Red): " + offset.poseForAlliance(Alliance.Red, SetpointContext.EMPTY));
	}
}
