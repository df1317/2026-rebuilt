package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Handles telemetry logging for the intake subsystem.
 */
public class IntakeTelemetry {

	private final SparkMax pivotMotor;
	private final SparkMax rollerMotor;

	public IntakeTelemetry(SparkMax pivotMotor, SparkMax rollerMotor) {
		this.pivotMotor = pivotMotor;
		this.rollerMotor = rollerMotor;
	}

	/**
	 * Logs all intake telemetry data.
	 */
	public void log(
			double pivotAngleDeg,
			double pivotTargetAngleDeg,
			boolean pivotAtPosition,
			boolean isExtended,
			boolean isRetracted,
			double rollerVelocityRPM,
			double rollerTargetRPM,
			boolean rollerRunning,
			boolean rollerAtSpeed,
			Color statusColor) {

		// Status color for the drivers
		DogLog.forceNt.log("Intake/Status", statusColor.toHexString());

		// Pivot tracking
		DogLog.log("Intake/PivotAngleDeg", pivotAngleDeg);
		DogLog.log("Intake/PivotTargetAngleDeg", pivotTargetAngleDeg);
		DogLog.log("Intake/PivotErrorDeg", pivotTargetAngleDeg - pivotAngleDeg);
		DogLog.log("Intake/PivotAtPosition", pivotAtPosition);
		DogLog.log("Intake/IsExtended", isExtended);
		DogLog.log("Intake/IsRetracted", isRetracted);

		// Roller tracking
		DogLog.log("Intake/RollerVelocityRPM", rollerVelocityRPM);
		DogLog.log("Intake/RollerTargetRPM", rollerTargetRPM);
		DogLog.log("Intake/RollerErrorRPM", rollerTargetRPM - rollerVelocityRPM);
		DogLog.log("Intake/RollerRunning", rollerRunning);
		DogLog.log("Intake/RollerAtSpeed", rollerAtSpeed);

		// Motor data
		DogLog.log("Intake/PivotCurrentAmps", pivotMotor.getOutputCurrent());
		DogLog.log("Intake/PivotVoltage", pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
		DogLog.log("Intake/RollerCurrentAmps", rollerMotor.getOutputCurrent());
		DogLog.log("Intake/RollerVoltage", rollerMotor.getBusVoltage() * rollerMotor.getAppliedOutput());
	}
}
