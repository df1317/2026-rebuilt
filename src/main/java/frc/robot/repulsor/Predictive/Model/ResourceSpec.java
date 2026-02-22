package frc.robot.repulsor.Predictive.Model;

public class ResourceSpec {
	public final double radiusM;
	public final double unitValue;
	public final double sigmaM;

	public ResourceSpec(double radiusM, double unitValue, double sigmaM) {
		this.radiusM = Math.max(0.01, radiusM);
		this.unitValue = unitValue;
		this.sigmaM = Math.max(0.02, sigmaM);
	}
}
