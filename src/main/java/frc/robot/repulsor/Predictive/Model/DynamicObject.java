package frc.robot.repulsor.Predictive.Model;

import edu.wpi.first.math.geometry.Translation2d;

public class DynamicObject {
	public final String id;
	public final String type;
	public final Translation2d pos;
	public final Translation2d vel;
	public final double ageS;

	public DynamicObject(String id, String type, Translation2d pos, Translation2d vel, double ageS) {
		this.id = id;
		this.type = type != null ? type : "unknown";
		this.pos = pos != null ? pos : new Translation2d();
		this.vel = vel != null ? vel : new Translation2d();
		this.ageS = Math.max(0.0, ageS);
	}
}
