package frc.robot.util;

import java.util.function.Supplier;

public class GamePieceTracker implements Supplier<Boolean> {
	private static final double INTAKE_COLLECT_TIME_S = 0.5;
	private static final double SHOOT_EJECT_TIME_S = 0.3;

	private boolean hasPiece = false;
	private boolean intaking = false;
	private boolean shooting = false;
	private double intakeStartTime = 0.0;
	private double shootStartTime = 0.0;

	@Override
	public Boolean get() {
		return hasPiece;
	}

	public void startIntake() {
		if (!intaking) {
			intaking = true;
			intakeStartTime = now();
		}
	}

	public void stopIntake() {
		intaking = false;
	}

	public void startShoot() {
		if (!shooting) {
			shooting = true;
			shootStartTime = now();
		}
	}

	public void stopShoot() {
		shooting = false;
	}

	public void setHasPiece(boolean value) {
		hasPiece = value;
	}

	public void update() {
		double t = now();
		if (intaking && !hasPiece) {
			if (t - intakeStartTime >= INTAKE_COLLECT_TIME_S) {
				hasPiece = true;
				intaking = false;
			}
		}
		if (shooting && hasPiece) {
			if (t - shootStartTime >= SHOOT_EJECT_TIME_S) {
				hasPiece = false;
				shooting = false;
			}
		}
	}

	private static double now() {
		return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
	}
}
