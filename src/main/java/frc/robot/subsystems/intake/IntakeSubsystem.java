package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Intake subsystem with a pivoting arm and roller mechanism.
 */
public class IntakeSubsystem extends SubsystemBase {

	// ==================== Hardware (package-private for telemetry/visualization) ====================
	final SparkMax pivotMotor;
	final SparkMax rollerMotor;
	private final SparkClosedLoopController pivotController;
	private final SparkClosedLoopController rollerController;
	final RelativeEncoder pivotEncoder;
	final RelativeEncoder rollerEncoder;

	// ==================== Control State (package-private for telemetry/visualization) ====================
	Angle targetPivotAngle = PIVOT_RETRACTED_ANGLE;
	AngularVelocity targetRollerVelocity = RPM.of(0);
	private final Debouncer atPositionDebouncer;

	// ==================== Visualization & Telemetry ====================
	private final IntakeVisualization visualization;
	private final IntakeTelemetry telemetry;

	public IntakeSubsystem() {
		pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
		rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
		pivotController = pivotMotor.getClosedLoopController();
		rollerController = rollerMotor.getClosedLoopController();
		pivotEncoder = pivotMotor.getEncoder();
		rollerEncoder = rollerMotor.getEncoder();

		configurePivotMotor();
		configureRollerMotor();

		atPositionDebouncer = new Debouncer(AT_POSITION_DEBOUNCE_TIME, DebounceType.kRising);

		visualization = new IntakeVisualization(this);
		telemetry = new IntakeTelemetry(this);
	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(PIVOT_CURRENT_LIMIT)
				.inverted(PIVOT_INVERTED);
		config.encoder
				// Converts encoder rotations to degrees: (360 deg/rot) / gear_ratio
				.positionConversionFactor(360.0 / PIVOT_GEAR_RATIO);
		config.closedLoop
				.pid(PIVOT_KP, PIVOT_KI, PIVOT_KD)
				.allowedClosedLoopError(PIVOT_ANGLE_TOLERANCE.in(Degrees), ClosedLoopSlot.kSlot0);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	private void configureRollerMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kCoast)
				.smartCurrentLimit(ROLLER_CURRENT_LIMIT)
				.inverted(ROLLER_INVERTED);
		config.closedLoop
				.pid(ROLLER_KP, ROLLER_KI, ROLLER_KD);
		config.closedLoop.feedForward
				.kV(ROLLER_KV);

		rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void periodic() {
		visualization.update();
		telemetry.log();
	}

	// ==================== State Query Methods ====================

	public boolean isPivotAtPosition() {
		boolean atPositionRaw = Math.abs(pivotEncoder.getPosition() - targetPivotAngle.in(Degrees)) < PIVOT_ANGLE_TOLERANCE
				.in(Degrees);
		return atPositionDebouncer.calculate(atPositionRaw);
	}

	boolean isRollerAtSpeed() {
		return Math.abs(rollerEncoder.getVelocity() - targetRollerVelocity.in(RPM)) < ROLLER_VELOCITY_TOLERANCE.in(RPM);
	}

	// ==================== Control Methods ====================

	public void setPivotAngle(Angle angle) {
		targetPivotAngle = angle;
		pivotController.setSetpoint(angle.in(Degrees), ControlType.kPosition);
	}

	public void setRollerVelocity(AngularVelocity velocity) {
		targetRollerVelocity = velocity;
		rollerController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
	}

	public void stopRoller() {
		setRollerVelocity(RPM.of(0));
	}

	public void stop() {
		stopRoller();
		pivotMotor.stopMotor();
	}

	// ==================== Command Factory Methods ====================

	public Command extendCommand() {
		return runOnce(() -> setPivotAngle(PIVOT_EXTENDED_ANGLE))
				.withName("Intake Extend");
	}

	public Command retractCommand() {
		return runOnce(() -> setPivotAngle(PIVOT_RETRACTED_ANGLE))
				.withName("Intake Retract");
	}

	public Command runRollerCommand() {
		return runOnce(() -> setRollerVelocity(ROLLER_INTAKE_VELOCITY))
				.withName("Intake Run Roller");
	}

	public Command ejectCommand() {
		return runOnce(() -> setRollerVelocity(ROLLER_EJECT_VELOCITY))
				.withName("Intake Eject");
	}

	public Command stopRollerCommand() {
		return runOnce(this::stopRoller)
				.withName("Intake Stop Roller");
	}

	public Command intakeCommand() {
		return Commands.sequence(
				extendCommand(),
				Commands.waitUntil(this::isPivotAtPosition),
				runRollerCommand())
				.withName("Intake Full Sequence");
	}

	public Command stowCommand() {
		return Commands.sequence(
				stopRollerCommand(),
				retractCommand())
				.withName("Intake Stow");
	}
}
