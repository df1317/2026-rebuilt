package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Handles telemetry logging for the intake subsystem.
 */
public class IntakeTelemetry {

	private final IntakeSubsystem intake;

	public IntakeTelemetry(IntakeSubsystem intake) {
		this.intake = intake;
	}

	/**
	 * Logs all intake telemetry data.
	 */
	public void log() {
		double pivotAngleDeg = intake.pivotEncoder.getPosition();
		double pivotTargetAngleDeg = intake.targetPivotAngle.in(Degrees);
		double rollerVelocityRPM = intake.rollerEncoder.getVelocity();
		double rollerTargetRPM = intake.targetRollerVelocity.in(RPM);

		// Status color for the drivers
		DogLog.forceNt.log("Intake/Status", getStatusColor().toHexString());

		// Pivot tracking
		DogLog.log("Intake/PivotAngleDeg", pivotAngleDeg);
		DogLog.log("Intake/PivotTargetAngleDeg", pivotTargetAngleDeg);
		DogLog.log("Intake/PivotErrorDeg", pivotTargetAngleDeg - pivotAngleDeg);
		DogLog.log("Intake/PivotAtPosition", intake.isPivotAtPosition());
		DogLog.log("Intake/IsExtended", isExtended());
		DogLog.log("Intake/IsRetracted", isRetracted());

		// Roller tracking
		DogLog.log("Intake/RollerVelocityRPM", rollerVelocityRPM);
		DogLog.log("Intake/RollerTargetRPM", rollerTargetRPM);
		DogLog.log("Intake/RollerErrorRPM", rollerTargetRPM - rollerVelocityRPM);
		DogLog.log("Intake/RollerRunning", isRollerRunning());
		DogLog.log("Intake/RollerAtSpeed", intake.isRollerAtSpeed());

		// Motor data
		DogLog.log("Intake/PivotCurrentAmps", intake.pivotMotor.getOutputCurrent());
		DogLog.log("Intake/PivotVoltage", intake.pivotMotor.getBusVoltage() * intake.pivotMotor.getAppliedOutput());
		DogLog.log("Intake/RollerCurrentAmps", intake.rollerMotor.getOutputCurrent());
		DogLog.log("Intake/RollerVoltage", intake.rollerMotor.getBusVoltage() * intake.rollerMotor.getAppliedOutput());
	}

	private boolean isExtended() {
		return intake.isPivotAtPosition()
				&& Math.abs(intake.targetPivotAngle.in(Degrees) - PIVOT_EXTENDED_ANGLE.in(Degrees)) < 1.0;
	}

	private boolean isRetracted() {
		return intake.isPivotAtPosition()
				&& Math.abs(intake.targetPivotAngle.in(Degrees) - PIVOT_RETRACTED_ANGLE.in(Degrees)) < 1.0;
	}

	private boolean isRollerRunning() {
		return Math.abs(intake.targetRollerVelocity.in(RPM)) > ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	private Color getStatusColor() {
		double targetRPM = intake.targetRollerVelocity.in(RPM);
		if (targetRPM > 0 && isExtended()) {
			return intake.isRollerAtSpeed() ? Color.kGreen : Color.kYellow;
		} else if (targetRPM < 0) {
			return Color.kOrange;
		} else if (!isRetracted()) {
			return Color.kYellow;
		}
		return Color.kRed;
	}
}
