package frc.robot.repulsor.Predictive.Model;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.repulsor.Setpoints.RepulsorSetpoint;

public class Candidate {
	public final RepulsorSetpoint setpoint;
	public final Translation2d targetXY;
	public final double ourEtaS;
	public final double enemyEtaS;
	public final double allyEtaS;
	public final double congestion;
	public final double pressure;
	public final double score;

	public Candidate(
			RepulsorSetpoint sp,
			Translation2d xy,
			double ourEtaS,
			double enemyEtaS,
			double allyEtaS,
			double congestion,
			double pressure,
			double score) {
		this.setpoint = sp;
		this.targetXY = xy;
		this.ourEtaS = ourEtaS;
		this.enemyEtaS = enemyEtaS;
		this.allyEtaS = allyEtaS;
		this.congestion = congestion;
		this.pressure = pressure;
		this.score = score;
	}
}
