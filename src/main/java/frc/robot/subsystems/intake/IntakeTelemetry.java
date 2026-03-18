package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.RobotLog;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeTelemetry {

	private final IntakeSubsystem intake;
	private final RollerSubsystem roller;

	public IntakeTelemetry(IntakeSubsystem intake, RollerSubsystem roller) {
		this.intake = intake;
		this.roller = roller;
	}

	public void log() {
		double pivotAngleDeg = intake.pivotEncoder.getPosition();
		double pivotTargetAngleDeg = intake.targetPivotAngle.in(Degrees);
		double rollerVelocityRPM = roller.rollerEncoder.getVelocity();
		double rollerTargetRPM = roller.targetRollerVelocity.in(RPM);

		DogLog.forceNt.log("Intake/Status", getStatusColor().toHexString());
		DogLog.forceNt.log("Intake/Homed", intake.homed);

		DogLog.log("Intake/PivotAngleDeg", pivotAngleDeg);
		DogLog.log("Intake/PivotTargetAngleDeg", pivotTargetAngleDeg);
		DogLog.log("Intake/PivotErrorDeg", pivotTargetAngleDeg - pivotAngleDeg);
		DogLog.log("Intake/PivotAtPosition", intake.isPivotAtPosition());
		DogLog.log("Intake/IsExtended", intake.isExtended());
		DogLog.log("Intake/IsRetracted", intake.isRetracted());
		DogLog.log("Intake/IsStalled", intake.isPivotStalled());

		DogLog.log("Intake/RollerVelocityRPM", rollerVelocityRPM);
		DogLog.log("Intake/RollerTargetRPM", rollerTargetRPM);
		DogLog.log("Intake/RollerErrorRPM", rollerTargetRPM - rollerVelocityRPM);
		DogLog.log("Intake/RollerRunning", isRollerRunning());
		DogLog.log("Intake/RollerAtSpeed", roller.isRollerAtSpeed());

		DogLog.log("Intake/PivotCurrentAmps", intake.pivotMotor.getOutputCurrent());
		DogLog.log("Intake/PivotVoltage", intake.pivotMotor.getBusVoltage() * intake.pivotMotor.getAppliedOutput());
		DogLog.log("Intake/RollerCurrentAmps", roller.rollerMotor.getOutputCurrent());
		DogLog.log("Intake/RollerVoltage",
				roller.rollerMotor.getBusVoltage() * roller.rollerMotor.getAppliedOutput());
	}

	private boolean isRollerRunning() {
		return Math.abs(roller.targetRollerVelocity.in(RPM)) > ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	private Color getStatusColor() {
		double targetRPM = roller.targetRollerVelocity.in(RPM);
		if (targetRPM > 0 && intake.isExtended()) {
			return roller.isRollerAtSpeed() ? RobotLog.GREEN : RobotLog.YELLOW;
		} else if (targetRPM < 0) {
			return RobotLog.BLUE;
		} else if (!intake.isRetracted()) {
			return RobotLog.YELLOW;
		}
		return RobotLog.RED;
	}
}
